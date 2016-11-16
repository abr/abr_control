#include "jaco2_rs485.h"
#include <math.h>

Jaco2::Jaco2(void) {
    //set common variables
    flag = 0;
    torqueValidation = false;
    switchValidation = false;
    read_input = true;
    delay = 2000;
    qtyWanted = 1;

    torqueDamping = 0x01;
    controlMode = 0x01;
    torqueKp = 1750; // torque kp 1.75 * 1000

    WriteCount = 0;
    ReadCount = 0;
    //joint addresses from base to wrist
    joint[0] = 0x10;
    joint[1] = 0x11;
    joint[2] = 0x12;
    joint[3] = 0x10;
    joint[4] = 0x14;
    joint[5] = 0x15;

    us[0] = 0.0;
    us[1] = 0.0;
    us[2] = 0.0;
    us[3] = 1.0;
    us[4] = 0.0;
    us[5] = 0.0;
    //We load the API.
	commLayer_Handle = dlopen("./kinova-api/Kinova.API.CommLayerUbuntu.so",
	                          RTLD_NOW|RTLD_GLOBAL);

	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int (*)()) dlsym(commLayer_Handle,
	                                          "InitCommunication");
	MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"RS485_Activate");
	MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted,
	                        int &ReceivedQtyIn)) dlsym(commLayer_Handle,
	                        "RS485_Read");
	MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted,
                             int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                             "RS485_Write");
}

int main()
{
    Jaco2 j2 = Jaco2();
    float u[6] = {1.0,0.0,0.0,1.0,0.0,0.0};
    
    j2.joint[3] = 0x10;
    u[3] = 4.0;
    j2.Connect(j2.joint[0]);
    j2.InitForceMode(j2.joint[0]);
    for (int i = 0; i<1000; i++)
    {
        j2.us[3] = u[3] * sin(2.0*3.14159 * i/1000);
        j2.ApplyU(j2.joint[0], j2.us);
    } 
    j2.Disconnect(j2.joint[0]);
    
    
    j2.joint[3] = 0x13;
    u[3] = 4.0;
    j2.Connect(j2.joint[0]);
    j2.InitForceMode(j2.joint[0]);
    for (int i = 0; i<1000; i++)
    {
        j2.us[3] = u[3] * sin(2.0*3.14159 * i/1000);
        j2.ApplyU(j2.joint[0], j2.us);
    } 
    j2.Disconnect(j2.joint[0]);
    
    j2.joint[3] = 0x14;
    u[3] = 4.0;
    j2.Connect(j2.joint[0]);
    j2.InitForceMode(j2.joint[0]);
    for (int i = 0; i<1000; i++)
    {
        j2.us[3] = u[3] * sin(2.0*3.14159 * i/1000);
        j2.ApplyU(j2.joint[0], j2.us);
    } 
    j2.Disconnect(j2.joint[0]);
    
    //j2.joint[3] = 0x15;
    u[5] = 4.0;
    u[3] = 0.0;
    j2.us[3] = 0;
    j2.Connect(j2.joint[0]);
    j2.InitForceMode(j2.joint[0]);
    for (int i = 0; i<1000; i++)
    {
        j2.us[5] = u[5] * sin(2.0*3.14159 * i/1000);
        j2.ApplyU(j2.joint[0], j2.us);
    } 
    j2.Disconnect(j2.joint[0]);
}

void Jaco2::Connect(unsigned char DESTINATION_ADDRESS)
{
	cout << "RS-485 communication Initialization" << endl;
	//Flag used during initialization.
	bool ActuatorInitialized1 = false;
	bool ActuatorInitialized2 = false;
    int result;

	//If all functions are loaded correctly.
	if(fptrInitCommunication != NULL || MyRS485_Activate != NULL ||
	   MyRS485_Read != NULL || MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication();

		//If API's initialization is correct.
		if(result == NO_ERROR_KINOVA)
		{
			cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D"
			     << endl << endl;

			/*We activate the RS-485 comm API. From here you cannot control the
			  robot with the Joystick or with the normal USB function. Only
			  RS-485 command will be accepted. Reboot the robot to get back to
			  normal control.*/

			MyRS485_Activate();

			//Initialize the INIT message
			InitMessage[0].Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage[0].SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;//destinaiton

			//Those value are not used for this command.
			InitMessage[0].DataLong[0] = 0x00000000;
			InitMessage[0].DataLong[1] = 0x00000000;
			InitMessage[0].DataLong[2] = 0x00000000;
			InitMessage[0].DataLong[3] = 0x00000000;
			
			//Initialize the INIT message
			InitMessage[1].Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage[1].SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;//destinaiton

			//Those value are not used for this command.
			InitMessage[1].DataLong[0] = 0x00000000;
			InitMessage[1].DataLong[1] = 0x00000000;
			InitMessage[1].DataLong[2] = 0x00000000;
			InitMessage[1].DataLong[3] = 0x00000000;


			//If we did not receive the answer, continue reading until done
			while(ReadCount != 1 && (!ActuatorInitialized1 || !ActuatorInitialized2))
			{
				MyRS485_Write(InitMessage, 2, WriteCount);
				usleep(2000);
				//GetFeedback(&ReceiveInitMessage);
				MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
		        
		        cout << "W R I T E  C O U N T  " << WriteCount << endl;
                cout << "R E A D  C O U N T  " << ReadCount << endl;

				/*We make sure that the mesage come from actuator 6(0x15) and
				that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				which is the answer of our message. (See document Kinova RS485
				Communication protocol).*/
				if(ReceiveInitMessage[0].SourceAddress == joint[3] &&
				   ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				   //&& ReceiveInitMessage[1].SourceAddress == 0x15 &&
				   //ReceiveInitMessage[1].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					ActuatorInitialized1 = true;
					cout << "actuator 1 initialized" << endl;
					cout << "base " << ReceiveInitMessage[0].SourceAddress << endl;
					//cout << "wrist" << ReceiveInitMessage[1].SourceAddress << endl;
				}
				else
				{
				    cout << "Error while initializing actuator 1" << endl;
				}
				
				if(ReceiveInitMessage[1].SourceAddress == 0x15 &&
				   ReceiveInitMessage[1].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					ActuatorInitialized2 = true;
					cout << "actuator 2 initialized" << endl;
					cout << "wrist " << ReceiveInitMessage[0].SourceAddress << endl;
					//cout << "wrist" << ReceiveInitMessage[1].SourceAddress << endl;
				}
				else
				{
				    cout << "Error while initializing actuator 2" << endl;
				}
			}
		}
		else
		{
		    cout << "Errors while Initializing" << endl;
		}
    }
	else
	{
		cout << "Errors while loading API's function" << endl;
	}
}

void Jaco2::InitForceMode(unsigned char DESTINATION_ADDRESS)
{
    // ========== BEGIN MAIN COMM ==========

    // STEP 0: Get initial position
    TrajectoryMessage[0].Command = 0x0001;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    TrajectoryMessage[0].DataFloat[0] = 0x00000000;
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    TrajectoryMessage[0].DataFloat[2] = 0x00000000;
    TrajectoryMessage[0].DataLong[3] = 0x00000000;
    
    TrajectoryMessage[1].Command = 0x0001;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    TrajectoryMessage[1].DataFloat[0] = 0x00000000;
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    TrajectoryMessage[1].DataFloat[2] = 0x00000000;
    TrajectoryMessage[1].DataLong[3] = 0x00000000;
    //pthread_mutex_lock (&APIMutex);
    MyRS485_Write(TrajectoryMessage, 2, WriteCount);
    //pthread_mutex_unlock (&APIMutex);
    usleep(delay);



    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
	    ReceiveInitMessage[1].SourceAddress == 0x15 &&
	    ReceiveInitMessage[0].Command == 0x0002 &&
	    ReceiveInitMessage[1].Command == 0x0002)
    {
	    pos[0] = ReceiveInitMessage[0].DataFloat[1];
	    cout << "Current position base is: "
		    << ReceiveInitMessage[0].DataFloat[1] << endl;
		    
	    pos[1] = ReceiveInitMessage[1].DataFloat[1];
	    cout << "Current position wrist is: "
		    << ReceiveInitMessage[1].DataFloat[1] << endl;
    }

    bool ack = false;

    SafetyMessage[0].Command =
	    0x0208;
    SafetyMessage[0].SourceAddress = SOURCE_ADDRESS;
    SafetyMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    SafetyMessage[0].DataFloat[0] = 100.0; //10 Nm maximum torque
    SafetyMessage[0].DataFloat[1] = 1.0; //0.75 safety factor
    SafetyMessage[0].DataFloat[2] = 0.0; //not used
    SafetyMessage[0].DataFloat[3] = 0.0; //not used
    
    SafetyMessage[1].Command =
	    0x0208;
    SafetyMessage[1].SourceAddress = SOURCE_ADDRESS;
    SafetyMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    SafetyMessage[1].DataFloat[0] = 100.0; //10 Nm maximum torque
    SafetyMessage[1].DataFloat[1] = 1.0; //0.75 safety factor
    SafetyMessage[1].DataFloat[2] = 0.0; //not used
    SafetyMessage[1].DataFloat[3] = 0.0; //not used

    MyRS485_Write(SafetyMessage, 2, WriteCount);
    while (ack = false)
    {
	    MyRS485_Write(SafetyMessage, 2, WriteCount);
	    usleep(delay);
	    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
		    ReceiveInitMessage[0].Command == 0x003E &&
		    ReceiveInitMessage[0].SourceAddress == 0x15 &&
		    ReceiveInitMessage[0].Command == 0x003E)
	    {
		    ack = true;
		    cout << "safety passed" << endl;
	    }

    }



    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    TrajectoryMessage[0].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[0].DataFloat[0] = pos[0];
    //not used
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[0].DataFloat[2] = 0;
    TrajectoryMessage[0].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8
        
    TrajectoryMessage[1].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[1].DataFloat[0] = pos[1];
    //not used
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[1].DataFloat[2] = 0;
    TrajectoryMessage[1].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8

    //We send the command and we protect the process with a mutex
    /*
     * Param1(IN):  The buffer that contains all the messages.
     * Param2(IN):  Messages count.
     * Param3(OUT): Message sent count.
     */
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    cout << "Initializing base position to: " << pos[0] << endl;
    cout << "Initializing wrist position to: " << pos[1] << endl;
    //for (int ii = 0; ii<500; ii++)
    //{
        //Joint6Command -= 40 * (0.0025);
        TrajectoryMessage[0].DataFloat[0] = pos[0];
        TrajectoryMessage[1].DataFloat[0] = pos[1];
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 2, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(delay);
    //}

    // STEP 2: Validate the torque command

    torqueValidation = false;

    //send position and torque command
    TrajectoryMessage[0].Command = GET_TORQUE_VALIDATION_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage[0].DataFloat[0] = 0;
    TrajectoryMessage[0].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[0].DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage[0].DataLong[3] = 0x00000000; //not used
    
    TrajectoryMessage[1].Command = GET_TORQUE_VALIDATION_REQUEST;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage[1].DataFloat[0] = 0;
    TrajectoryMessage[1].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[1].DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage[1].DataLong[3] = 0x00000000; //not used

    cout << "STEP 2: Request Torque Command Verification" << endl;

    while (torqueValidation == false)
    {
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 2, WriteCount);
       // pthread_mutex_unlock (&APIMutex);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
		    ReceiveInitMessage[0].Command == SEND_TORQUE_VALIDATION &&
		    ReceiveInitMessage[1].SourceAddress == 0x15 &&
		    ReceiveInitMessage[1].Command == SEND_TORQUE_VALIDATION)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 1 &&
		        ReceiveInitMessage[1].DataLong[0] == 1)
		    {
			    cout << "Torque Validation True base : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    torqueValidation = true;
			    cout << "Torque Validation True wrist : "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0 ||
		             ReceiveInitMessage[1].DataLong[0] == 0)
		    {
			    cout << "Torque Validation False base : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Torque Validation False wrist : "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    flag = 10;
		    }
		    else
		    {
			    cout << "ERROR READING TORQUE VALIDATION REPLY BASE: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "ERROR READING TORQUE VALIDATION REPLY WRIST: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
		    }
	    }
    }

    // STEP 3: Switch to torque control mode

    switchValidation = false;

    TrajectoryMessage[0].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    TrajectoryMessage[0].DataLong[0] = ((unsigned short) 0x01 |
        ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
        ((unsigned short) d1 << 24))); //U24|U8
    TrajectoryMessage[0].DataLong[1]=0x00000000;
    TrajectoryMessage[0].DataLong[2]=0x00000000;
    TrajectoryMessage[0].DataLong[3]=0x00000000;
    
    TrajectoryMessage[1].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    TrajectoryMessage[1].DataLong[0] = ((unsigned short) 0x01 |
        ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
        ((unsigned short) d1 << 24))); //U24|U8
    TrajectoryMessage[1].DataLong[1]=0x00000000;
    TrajectoryMessage[1].DataLong[2]=0x00000000;
    TrajectoryMessage[1].DataLong[3]=0x00000000;

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    do
    {
        cout << "waiting on reply for mode switch" << endl;
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 2, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY &&
		    ReceiveInitMessage[1].SourceAddress == 0x15 &&
		    ReceiveInitMessage[1].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257 &&
		        ReceiveInitMessage[1].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode TORQUE True Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 1 &&
		             ReceiveInitMessage[1].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode POSITION True Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0 ||
	                 ReceiveInitMessage[1].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode False Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
		    }
	    }
    } while (switchValidation == false);

     cout << "Verified: Switching to Torque Mode" << endl;
}

void* Jaco2::ApplyU(unsigned char DESTINATION_ADDRESS, float us[6])
{
    // Step 4: Enjoy torque control mode!

    TrajectoryMessage[0].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    //32F position command [deg]
    //TrajectoryMessage[0].DataFloat[0] = pos[0];
    TrajectoryMessage[0].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[0].DataFloat[2] = us[3]; //32F torque command [1Nm]
    TrajectoryMessage[0].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) |
        ((unsigned long) torqueKp << 16)); //U16|U8|U8
    
    TrajectoryMessage[1].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F position command [deg]
    //TrajectoryMessage[1].DataFloat[0] = pos[1];
    TrajectoryMessage[1].DataLong[1] = 0x00000000; //not used
    TrajectoryMessage[1].DataFloat[2] = us[5]; //32F torque command [1Nm]
    TrajectoryMessage[1].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) |
        ((unsigned long) torqueKp << 16)); //U16|U8|U8


    //cout << "STEP 4: Enjoy Torque Mode" << endl;
    //for(int i=0; i<2000; i++)
    //{
        TrajectoryMessage[0].DataFloat[0] = pos[0];
        TrajectoryMessage[1].DataFloat[0] = pos[1];
        //cout << "pos0: " << pos[0] << endl;
        //cout << "pos1: " << pos[1] << endl;
        MyRS485_Write(TrajectoryMessage, 2, WriteCount);

        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, 6, ReadCount);
        //cout << "R E A D  C O U N T  " << ReadCount << endl;
        //cout << "Base source address: " << ReceiveInitMessage[0].SourceAddress << endl;
        //cout << "Wrist source address: " << ReceiveInitMessage[1].SourceAddress << endl;
        //cout << "Base command: " << ReceiveInitMessage[0].Command << endl;
        //cout << "Wrist command: " << ReceiveInitMessage[1].Command << endl;
        //cout << "expected command: " << RS485_MSG_SEND_ALL_VALUES_1 << endl;
        cout << "base pos: " << ReceiveInitMessage[0].DataFloat[1] << endl;
        cout << "wrist pos: " << ReceiveInitMessage[1].DataFloat[1] << endl;
        //pos[0] = ReceiveInitMessage[0].DataFloat[1];
        //pos[1] = ReceiveInitMessage[3].DataFloat[1];
	    /*if (ReceiveInitMessage[0].SourceAddress == joint[3])
	    {
		    //pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    //pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    //cout << "base pos: " << pos[0] << endl;
		    //cout << "wrist pos: " << pos[1] << endl;
		    cout << "base source correct: " << ReceiveInitMessage[0].SourceAddress << endl;
	    }
	    else
	    {
	        cout << "base source incorrect: " << ReceiveInitMessage[0].SourceAddress << endl;
	    }
	    if (ReceiveInitMessage[3].SourceAddress == 0x15)
	    {
		    //pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    //pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    //cout << "base pos: " << pos[0] << endl;
		    //cout << "wrist pos: " << pos[1] << endl;
		    cout << "wrist source correct: " << ReceiveInitMessage[1].SourceAddress << endl;
	    }
	    else
	    {
	        cout << "wrist source incorrect: " << ReceiveInitMessage[1].SourceAddress << endl;
	    }
	    if (ReceiveInitMessage[0].Command == RS485_MSG_SEND_ALL_VALUES_1)
	    {
		    pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    //pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    cout << "base pos: " << pos[0] << endl;
		    //cout << "wrist pos: " << pos[1] << endl;
	    }
	    else
	    {
	        cout << "base response type incorrect: " << ReceiveInitMessage[0].Command << endl;
	    }
	    if (ReceiveInitMessage[3].Command == RS485_MSG_SEND_ALL_VALUES_1)
	    {
		    //pos[0] = ReceiveInitMessage[0].DataFloat[1];
		    pos[1] = ReceiveInitMessage[1].DataFloat[1];
		    //cout << "base pos: " << pos[0] << endl;
		    cout << "wrist pos: " << pos[1] << endl;
	    }
	    else
	    {
	        cout << "wrist response type incorrect: " << ReceiveInitMessage[1].Command << endl;
	    }*/
	    
	    for (int ii = 0; ii < 6; ii++) {
            if (ReceiveInitMessage[ii].SourceAddress == joint[3]) {
                pos[0] = ReceiveInitMessage[ii].DataFloat[1];
                cout << "pos[0]: " << pos[0] << endl;;
                break;
            }
        }

        for (int ii = 0; ii < 6; ii++) {
            if (ReceiveInitMessage[ii].SourceAddress == 0x15) {
                pos[1] = ReceiveInitMessage[ii].DataFloat[1];
                cout << "pos[1]: " << pos[1] << endl;
                break;
            }
        }
	    
    //}
}

void* Jaco2::GetFeedback(RS485_Message* args)
{
    /*RS485_Message* MessageListIn = (RS485_Message *)args;*/
	//ReadCount = 0;

	/*short accelX = 0;
	short accelY = 0;
	short accelZ = 0;
	short temperature = 0;
	float fAccelX = 0;
	float fAccelY = 0;
	float fAccelZ = 0;
	float fTemperature = 0;*/
/*
	ReadCount = 0;
	MyRS485_Read(MessageListIn, 3, ReadCount);
	for(int j = 0; j < ReadCount; j++)
	{
		if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_1)
		{
			cout << "Current  = " << MessageListIn[j]->DataFloat[0] << endl;
			cout << "Position = " << MessageListIn[j]->DataFloat[1] << endl;
			cout << "Velocity = " << MessageListIn[j]->DataFloat[2] << endl;
			cout << "Torque   = " << MessageListIn[j]->DataFloat[3] << endl;
		}
		if(MessageListIn[j]->Command == RS485_MSG_SEND_ALL_VALUES_2)
		{
			cout << "PWM  = " << MessageListIn[j]->DataFloat[0] << endl;
			cout << "Position encoder = " << MessageListIn[j]->DataFloat[1] << endl;
			cout << "Accel X = " << (MessageListIn[j]->DataLong[2] & 0x0000FFFF)* 0.001 << endl;
			cout << "Accel Y = " << (MessageListIn[j]->DataLong[2] & 0xFFFF0000) >> 16)* 0.001 << endl;
			cout << "Accel Z = " << (MessageListIn[j]->DataLong[3] & 0x0000FFFF)* 0.001 << endl;
			cout << "Temperature = " << (MessageListIn[j]->DataLong[3] & 0xFFFF0000) >> 16)* 0.01 << endl;
		}
		if(MessageListIn[j]->Command == RS485_MSG_SEND_ALL_VALUES_3)
		{
			cout << "Motor current = " << MessageListIn[j]->DataFloat[0] << endl;
			cout << "Absolute position = " << MessageListIn[j]->DataFloat[1] << endl;
		}
		// ---------- Safety Measure Checks
	    if(MessageListIn[j]->Command == SEND_TORQUE_VALIDATION)
	    {
	        if(MessageListIn[j]->DataLong[0] == 1)
	        {
	            cout << "Torque Validation True : " 
	                 << MessageListIn[j]->DataLong[0] << endl;
	            torqueValidation = true;
	        }
	        
	        else if(MessageListIn[j]->DataLong[0] == 0)
	        {
	            cout << "Torque Validation False : " 
	                 << MessageListIn[j]->DataLong[0] << endl;
	            flag = 10;
	        } 
	        else
	        {
	            cout << "ERROR READING TORQUE VALIDATION REPLY: " 
	                 << MessageListIn[j]->DataLong[0]<< endl;
            }
	    }
	    if(MessageListIn[j]->Command == SWITCH_CONTROL_MODE_REPLY)
	    {
	        if(MessageListIn[j]->DataLong[0] == 257)
	        {
	            cout << "Switch Control Mode True : " 
	                 << MessageListIn[j]->DataLong[0] << endl;
	            switchValidation = true;
	        }
	        
	        else if(MessageListIn[j]->DataLong[0] == 0)
	        {
	            cout << "Switch Control Mode False : " 
	                 << MessageListIn[j]->DataLong[0] << endl;
	            flag = 11;
	        } 
	        else
	        {
	            cout << "ERROR READING SWITCH CONTROL MODE REPLY: " 
	                 << MessageListIn[j]->DataLong[0]<< endl;
            }
	    }
	    if(MessageListIn[j]->Command == REPORT_ERROR)
	    {
	        if(MessageListIn[j]->DataLong[0] != 0)
	        {
	            cout << "Error number : " << MessageListIn[j]->DataLong[0] 
	                 << endl; //may be [1] instead of 0
	            flag = 1; // will add error specific values later
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
	usleep(delay);*/
}

void Jaco2::Disconnect(unsigned char DESTINATION_ADDRESS)
{
    switchValidation = false;

    TrajectoryMessage[0].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    TrajectoryMessage[0].DataLong[0] = ((unsigned short)0x00 |
	    ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
	    ((unsigned short)d1 << 24))); //U24|U8
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    TrajectoryMessage[0].DataLong[2] = 0x00000000;
    TrajectoryMessage[0].DataLong[3] = 0x00000000;
    
    TrajectoryMessage[1].Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    TrajectoryMessage[1].DataLong[0] = ((unsigned short)0x00 |
	    ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
	    ((unsigned short)d1 << 24))); //U24|U8
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    TrajectoryMessage[1].DataLong[2] = 0x00000000;
    TrajectoryMessage[1].DataLong[3] = 0x00000000;

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    while (switchValidation == false)
    {
	    cout << "waiting on reply for mode switch" << endl;
	    //pthread_mutex_lock(&APIMutex);
	    MyRS485_Write(TrajectoryMessage, 2, WriteCount);
	    //pthread_mutex_unlock(&APIMutex);
	    usleep(delay);
	    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY &&
		    ReceiveInitMessage[1].SourceAddress == 0x15 &&
		    ReceiveInitMessage[1].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257 &&
		        ReceiveInitMessage[1].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode TORQUE True Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 1 &&
		        ReceiveInitMessage[1].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode POSITION True Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0 ||
		             ReceiveInitMessage[1].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "Switch Control Mode False Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
			    flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Base: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY Wrist: "
				    << ReceiveInitMessage[1].DataLong[0] << endl;
		    }
	    }

    }
    cout << "Verified: Switching to position Mode" << endl;


    cout << "Exiting Main Control Loop" << endl;
    /*
    // STEP 0: Get initial position
    TrajectoryMessage[0].Command = 0x0001;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    TrajectoryMessage[0].DataFloat[0] = 0x00000000;
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    TrajectoryMessage[0].DataFloat[2] = 0x00000000;
    TrajectoryMessage[0].DataLong[3] = 0x00000000;
    
    TrajectoryMessage[1].Command = 0x0001;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    TrajectoryMessage[1].DataFloat[0] = 0x00000000;
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    TrajectoryMessage[1].DataFloat[2] = 0x00000000;
    TrajectoryMessage[1].DataLong[3] = 0x00000000;
    //pthread_mutex_lock (&APIMutex);
    MyRS485_Write(TrajectoryMessage, 2, WriteCount);
    //pthread_mutex_unlock (&APIMutex);
    usleep(delay);
    MyRS485_Read(ReceiveInitMessage, 3, ReadCount);
    if (ReceiveInitMessage[0].SourceAddress == joint[3] &&
	    ReceiveInitMessage[1].SourceAddress == 0x15 &&
	    ReceiveInitMessage[0].Command == 0x0002 &&
	    ReceiveInitMessage[1].Command == 0x0002)
    {
	    pos[0] = ReceiveInitMessage[0].DataFloat[1];
	    cout << "Current position base is: "
		    << ReceiveInitMessage[0].DataFloat[1] << endl;
		    
	    pos[1] = ReceiveInitMessage[1].DataFloat[1];
	    cout << "Current position wrist is: "
		    << ReceiveInitMessage[1].DataFloat[1] << endl;
    }
    
    //send position and torque command
    TrajectoryMessage[0].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[0].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[0].DestinationAddress = joint[3];//DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[0].DataFloat[0] = pos[0];
    //not used
    TrajectoryMessage[0].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[0].DataFloat[2] = 0;
    TrajectoryMessage[0].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8
        
    TrajectoryMessage[1].Command =
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
    TrajectoryMessage[1].SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage[1].DestinationAddress = 0x15;//DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage[1].DataFloat[0] = pos[1];
    //not used
    TrajectoryMessage[1].DataLong[1] = 0x00000000;
    //32F torque command [Nm]
    TrajectoryMessage[1].DataFloat[2] = 0;
    TrajectoryMessage[1].DataLong[3] = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp << 16)); //U16|U8|U8
    //We send the command and we protect the process with a mutex
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    cout << "Initializing base position to: " << pos[0] << endl;
    cout << "Initializing wrist position to: " << pos[1] << endl;
    //for (int ii = 0; ii<500; ii++)
    //{
        //Joint6Command -= 40 * (0.0025);
        TrajectoryMessage[0].DataFloat[0] = pos[0];
        TrajectoryMessage[1].DataFloat[0] = pos[1];
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(TrajectoryMessage, 2, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(delay);
    //}*/
}
