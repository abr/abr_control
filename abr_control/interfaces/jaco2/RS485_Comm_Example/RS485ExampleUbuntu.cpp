/*
 * This example shows how to send RS-485 command through the USB port. Basically, that means communicating directly to the actuators
 * via the USB port. If applied on a 6 axis JACO or MICO, this example will rotate the actuator #6, the one holding the end effector.
 * It will also start a thread that read and display information during the process.
 */

//INCLUDE SECTION
#include <iostream>
#include <dlfcn.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include "kinova-api/Kinova.API.CommLayerUbuntu.h"
#include <string.h>

using namespace std;

#define LOOPCOUNT 500
#define SOURCE_ADDRESS 0x00
#define RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND 0x0014
#define DESTINATION_ADDRESS 0x15
#define GET_TORQUE_VALIDATION_REQUEST 0x0201
#define SEND_TORQUE_VALIDATION 0x0202
#define SWITCH_CONTROL_MODE_REQUEST 0x0203
#define SWITCH_CONTROL_MODE_REPLY 0x0204
#define REPORT_ERROR 0x0030
#define CLEAR_ERROR_FLAG 0x0033
#define POSITION_LIMIT 0x0021

struct Status {
    int flag;
    bool torqueValidation;
    bool switchValidation;
    int pos;
    bool read_input;
};

//This is the function that will be executed by the worker thread.(Information reading).
void *SerialRead(void *args);

//A mutex to protect the access to the API.
pthread_mutex_t APIMutex;

//A global variable that will be shared between the main thread(SEND) and the worker thread(RECEIVE).
volatile float Joint6Command;

//A handle needed to open the API(library).
void *commLayer_Handle;

//Function pointers to access API's function.
int(*fptrInitCommunication)();
int(*MyRS485_Activate)();     // FUNCTION TO ACTIVATE USB - RS485 MODE //
int(*MyRS485_Read)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn);
int(*MyRS485_Write)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn);

//MAIN(SEND information)
int main()
{
    Status status;
    status.flag = 0;
    status.torqueValidation = false;
    status.switchValidation = false;
    status.read_input = true;
    
	//Variable used during the communication process.
	int WriteCount = 0;
	int ReadCount = 0;

	//Flag used during initialization.
	bool ActuatorInitialized = false;

	//Variable needed during worker thread creation but not used after.
	int ThreadArgument = 0;

	//The worker thread.
	pthread_t GetPositionThread;

	//Message to initialize the actuator 6's command.
	RS485_Message InitMessage;

	//Message to receive the actuator 6's position.
	RS485_Message ReceiveInitMessage;

	//Message to move the actuator 6.
	RS485_Message TrajectoryMessage;

	cout << "RS-485 communication Example." << endl;

	//We load the API.
	commLayer_Handle = dlopen("./kinova-api/Kinova.API.CommLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL);

	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int (*)()) dlsym(commLayer_Handle,"InitCommunication");
	MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"RS485_Activate");
	MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn)) dlsym(commLayer_Handle,"RS485_Read");
	MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted, int &ReceivedQtyIn)) dlsym(commLayer_Handle,"RS485_Write");

	//If all functions are loaded correctly.
	if(fptrInitCommunication != NULL || MyRS485_Activate != NULL || MyRS485_Read != NULL || MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication();

		//If API's initialization is correct.
		if(result == NO_ERROR_KINOVA)
		{
			cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

			//We activate the RS-485 comm API. From here you cannot control the robot with the Joystick or
			//with the normal USB function. Only RS-485 command will be accepted. Reboot the robot to get
			//back to normal control.
			MyRS485_Activate();

			//Initialize the INIT message
			InitMessage.Command = RS485_MSG_GET_ACTUALPOSITION; //Setting the command ID
			InitMessage.SourceAddress = SOURCE_ADDRESS;                   //Setting the source address (0 means the API)
			InitMessage.DestinationAddress = DESTINATION_ADDRESS;              //Setting the destinaiton address(0x15 is the actuator 6's default value)

			//Those value are not used for this command.
			InitMessage.DataLong[0] = 0x00000000;
			InitMessage.DataLong[1] = 0x00000000;
			InitMessage.DataLong[2] = 0x00000000;
			InitMessage.DataLong[3] = 0x00000000;

			//Send the Init Message. 1 is the message's quantity and WriteCount will stored the Qty of messages sent.
			MyRS485_Write(&InitMessage, 1, WriteCount);

			//In case we did not received the answer, we continue reading until it's done
			while(ReadCount != 1 && !ActuatorInitialized)
			{
				MyRS485_Write(&InitMessage, 1, WriteCount);
				usleep(4000);
				MyRS485_Read(&ReceiveInitMessage, 1, ReadCount);

				//We make sure that the mesage come from actuator 6(0x15) and that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				//which is the answer of our message. (See document Kinova RS485 Communication protocol).
				if(ReceiveInitMessage.SourceAddress == DESTINATION_ADDRESS && ReceiveInitMessage.Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					Joint6Command = ReceiveInitMessage.DataFloat[1];
					ActuatorInitialized = true;
				}
			}

			//Creation of the thread that will get information from the robot.
			if(pthread_create(&GetPositionThread, NULL, &SerialRead, (void *)&status))
			{
				cout << "Error while creating thread" << endl;
				return ERROR_OPERATION_INCOMPLETED;
			}
            
            while (status.flag == 0)
            {
                // ========== BEGIN MAIN COMM ==========
			     
			    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
			    TrajectoryMessage.Command = RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND; //send position and torque command 
			    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
			    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
			    TrajectoryMessage.DataFloat[0] = 0x00000000; //32F position command [deg]
			    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
			    TrajectoryMessage.DataFloat[2] = 0; //32F torque command [1Nm]
			    unsigned char torqueDamping = 0x01;
                unsigned char controlMode = 0x00; // not used
                unsigned short torqueKp = 1750; // torque kp 0 1.75 multiplied by 1000
                TrajectoryMessage.DataLong[3] = (unsigned long) torqueDamping | ((unsigned long) controlMode << 8) | ((unsigned long) torqueKp <<   16); //U16|U8|U8 

			    //We send the command and we protect the process with a mutex
			    /*
			     * Param1(IN):  The buffer that contains all the messages. In our case, only one.
			     * Param2(IN):  Messages count.
			     * Param3(OUT): Message sent count.
			     */
			    cout << "STEP 1: Sending Torque Command for Verification" << endl;
			    
			    for (int ii = 0; ii<500; ii++)
			    {
			        //Joint6Command -= 40 * (0.0025);
				    TrajectoryMessage.DataFloat[0] = status.pos;
			        pthread_mutex_lock (&APIMutex);
			        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
			        pthread_mutex_unlock (&APIMutex);
			        usleep(2000);
			    }
			    
			    // STEP 2: Validate the torque command
			    
			    /*status.torqueValidation = false;
			    
			    TrajectoryMessage.Command = GET_TORQUE_VALIDATION_REQUEST; //send position and torque command 
			    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
			    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
			    TrajectoryMessage.DataFloat[0] = 0x00000000; //32F torque command to be tested [Nm]
			    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
			    TrajectoryMessage.DataFloat[2] = 0x00000000; //not used
			    TrajectoryMessage.DataLong[3] = 0x00000000; //not used
			
			    cout << "STEP 2: Requesting Torque Command Verification" << endl;
			    
			    while (status.torqueValidation == false)
			    {
			        pthread_mutex_lock (&APIMutex);
			        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
			        pthread_mutex_unlock (&APIMutex);
			        usleep(2000);
                }*/
                
			    // STEP 3: Switch to torque control mode
			    
			    status.switchValidation = false;
			    
			    TrajectoryMessage.Command = SWITCH_CONTROL_MODE_REQUEST; //send position and torque command 
			    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
			    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
			    TrajectoryMessage.DataFloat[0] = 1.0; //control mode
			    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
			    TrajectoryMessage.DataFloat[2] = 0x00000000; //not used
			    TrajectoryMessage.DataLong[3] = 0x00000000; //not used
			
			    cout << "STEP 3: Waiting for Torque Verification" << endl;
			    
			    do
			    {
			        cout << "waiting on reply for mode switch" << endl;
			        pthread_mutex_lock (&APIMutex);
			        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
			        pthread_mutex_unlock (&APIMutex);
			        usleep(2000);
			        //cout << "flag = " << status.flag << endl;
			        //cout << "switch validation = " << status.switchValidation << endl;
			    } while (status.switchValidation == false);
			    
			     cout << "Verified: Switching to Torque Mode" << endl;
			
			    // Step 4: Enjoy torque control mode!
			    
			    TrajectoryMessage.Command = RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND; //send position and torque command 
			    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
			    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
			    TrajectoryMessage.DataFloat[0] = 0x00000000; //32F position command [deg]
			    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
			    TrajectoryMessage.DataFloat[2] = 0; //32F torque command [1Nm]
			    torqueDamping = 0x01;
                controlMode = 0x00; // not used
                torqueKp = 1750; // torque kp 0 1.75 multiplied by 1000
                TrajectoryMessage.DataLong[3] = (unsigned long) torqueDamping | ((unsigned long) controlMode << 8) | ((unsigned long) torqueKp <<   16); //U16|U8|U8 
                

			    cout << "STEP 4: Enjoy Torque Mode" << endl;
			    
			    for(int i=0; i<10000; i++)
			    {
			        pthread_mutex_lock (&APIMutex);
		            MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
		            pthread_mutex_unlock (&APIMutex);
		            usleep(2000);
	            }
		    }
		    // Step 5: Swith back to position mode in case of error or if shutting down
		    // need to read current position to set the target to match it
		    // to avoid sudden fast movements
		    
		    status.switchValidation = false;
		    
		    TrajectoryMessage.Command = SWITCH_CONTROL_MODE_REQUEST; //send position and torque command 
		    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
		    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
		    TrajectoryMessage.DataFloat[0] = 0x00000000; //control mode
		    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
		    TrajectoryMessage.DataFloat[2] = 0x00000000; //not used
		    TrajectoryMessage.DataLong[3] = 0x00000000; //not used
		
		    cout << "STEP 5: Request to switch back to Position Control" << endl;
		
		    while (status.switchValidation == false)
		    {
		        pthread_mutex_lock (&APIMutex);
		        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
		        pthread_mutex_unlock (&APIMutex);
		        usleep(2000);
		    }
		    
		    cout << "Verified: Switching to Position Mode" << endl;
		    
		    TrajectoryMessage.Command = RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES; //send position and torque command 
		    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
		    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
		    TrajectoryMessage.DataFloat[0] = status.pos; //32F position command [deg] MAY NEED TO BE CONVERTED TO HEX
		    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
		    TrajectoryMessage.DataFloat[2] = 0x00000000; //32F torque command [1Nm]
		    TrajectoryMessage.DataLong[3] = 0x00010001; //U16|U16
		    
		    pthread_mutex_lock (&APIMutex);
	        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
	        pthread_mutex_unlock (&APIMutex);
	        usleep(2000);
		    
		    pthread_join(GetPositionThread, NULL);
		    status.read_input = false;
		}
		
		cout << "Exiting Main Control Loop" << endl;


	}
	else
	{
		cout << "Errors while loading API's function" << endl;
	}

	return 0;
}

// ========== RS485 READ ==========
void *SerialRead(void *args)
{
    Status* status = (Status *)args;
	RS485_Message MessageListIn [50];
	int MessageReadCount = 0;

	short accelX = 0;
	short accelY = 0;
	short accelZ = 0;
	short temperature = 0;

	float fAccelX = 0;
	float fAccelY = 0;
	float fAccelZ = 0;
	float fTemperature = 0;

    while (status->read_input == true)
    {
        MessageReadCount = 0;
	    pthread_mutex_lock (&APIMutex);
	    MyRS485_Read(MessageListIn, 3, MessageReadCount);

	    pthread_mutex_unlock (&APIMutex);

	    for(int j = 0; j < MessageReadCount; j++)
	    {
		    if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_1)
		    {
			    //cout << "Current  = " << MessageListIn[j].DataFloat[0] << endl;
			    //cout << "Position = " << MessageListIn[j].DataFloat[1] << endl;
			    //cout << "Velocity = " << MessageListIn[j].DataFloat[2] << endl;
			    //cout << "Torque   = " << MessageListIn[j].DataFloat[3] << endl;
			    status->pos = MessageListIn[j].DataFloat[1];
		    }

		    if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_2)
		    {
			    //cout << "PWM  = " << MessageListIn[j].DataFloat[0] << endl;
			    //cout << "Position encoder = " << MessageListIn[j].DataFloat[1] << endl;

			    accelX = (short)(MessageListIn[j].DataLong[2] & 0x0000FFFF);
			    accelY = (short)((MessageListIn[j].DataLong[2] & 0xFFFF0000) >> 16);
			    accelZ = (short)(MessageListIn[j].DataLong[3] & 0x0000FFFF);
			    temperature = (short)((MessageListIn[j].DataLong[3] & 0xFFFF0000) >> 16);

			    fAccelX = (float)accelX * 0.001;
			    fAccelY = (float)accelY * 0.001;
			    fAccelZ = (float)accelZ * 0.001;
			    fTemperature = (float)temperature * 0.01;

			    //cout << "Accel X = " << fAccelX << endl;
			    //cout << "Accel Y = " << fAccelY << endl;
			    //cout << "Accel Z = " << fAccelZ << endl;
			    //cout << "Temperature = " << fTemperature << endl;
		    }

		    if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_3)
		    {
			    cout << "Motor current = " << MessageListIn[j].DataFloat[0] << endl;
			    cout << "Absolute position = " << MessageListIn[j].DataFloat[1] << endl;
		    }
		
		    // ---------- Safety Measure Checks
		    if(MessageListIn[j].Command == SEND_TORQUE_VALIDATION)
		    {
		        if(MessageListIn[j].DataLong[0] == 0)
		        {
		            cout << "Torque Validation True : " << MessageListIn[j].DataLong[0] << endl;
		            status->torqueValidation = true;
		        }
		        
		        else if(MessageListIn[j].DataLong[0] == 1)
		        {
		            cout << "Torque Validation False : " << MessageListIn[j].DataLong[0] << endl;
		            status->flag = 10;
		        } 
		    }
		    if(MessageListIn[j].Command == SWITCH_CONTROL_MODE_REPLY)
		    {
		        if(MessageListIn[j].DataLong[0] == 1)
		        {
		            cout << "Switch Control Mode True : " << MessageListIn[j].DataLong[0] << endl;
		            status->switchValidation = true;
		        }
		        
		        else if(MessageListIn[j].DataLong[0] == 0)
		        {
		            cout << "Switch Control Mode False : " << MessageListIn[j].DataLong[0] << endl;
		            status->flag = 11;
		        } 
		        else
		        {
		            cout << "ERROR READING SWITCH CONTROL MODE REPLY" << endl;
	            }
		    }
		    if(MessageListIn[j].Command == REPORT_ERROR)
		    {
		        if(MessageListIn[j].DataLong[0] != 0)
		        {
		            cout << "Error number : " << MessageListIn[j].DataLong[0] << endl; //may be [1] instead of 0
		            status->flag = 1; // for now, will add error specific values later
		        }
		        //ADD CHECK FOR SPECIFIC ERROR
		        //0 no error
		            // no errors or error has been reset by user, set flag = 0;
		        //1 or 2 temperature error
		        //3 velocity error
		        //4 position limit error
		        //5 abs position error
		        //6relative position error
		        //7 command error 
		        //8 current error
		        //9 torque error
		    }
	    }
	    usleep(2000);
	}
	
	pthread_exit(0);
    return NULL;
}
