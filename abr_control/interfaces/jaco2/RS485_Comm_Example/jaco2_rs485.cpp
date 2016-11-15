#include "jaco2_rs485.h"

int main()
{
    Connect();
    InitForceMode();
    ApplyU();
    Disconnect();
}

void Connect()
{
	//Flag used during initialization.
	bool ActuatorInitialized = false;

	//Message to initialize the actuator 6's command.
	RS485_Message InitMessage;
	
	RS485_Message SafetyMessage;

	//Message to receive the actuator 6's position.
	RS485_Message ReceiveInitMessage[5000];

	//Message to move the actuator 6.
	RS485_Message TrajectoryMessage;

	cout << "RS-485 communication Example." << endl;

	//We load the API.
	commLayer_Handle = dlopen("./kinova-api/Kinova.API.CommLayerUbuntu.so", 
	                          RTLD_NOW|RTLD_GLOBAL);

    int result;
	AngularPosition torque;
	AngularPosition torqueGravityFree;

	//int programResult = 0;

	//We load the functions from the library
	MyInitAPI = (int(*)()) dlsym(commLayer_Handle, "InitAPI");
	MyCloseAPI = (int(*)()) dlsym(commLayer_Handle, "CloseAPI");
	MyGetAngularForce = (int(*)(AngularPosition &Response)) dlsym(commLayer_Handle, "GetAngularForce");
	MyGetAngularForceGravityFree = (int(*)(AngularPosition &Response)) dlsym(commLayer_Handle, "GetAngularForceGravityFree");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commLayer_Handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) dlsym(commLayer_Handle, "SetActiveDevice");
	


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
			InitMessage.Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage.SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage.DestinationAddress = DESTINATION_ADDRESS;//destinaiton 

			//Those value are not used for this command.
			InitMessage.DataLong[0] = 0x00000000;
			InitMessage.DataLong[1] = 0x00000000;
			InitMessage.DataLong[2] = 0x00000000;
			InitMessage.DataLong[3] = 0x00000000;


			//If we did not receive the answer, continue reading until done
			while(ReadCount != 1 && !ActuatorInitialized)
			{
				MyRS485_Write(&InitMessage, 1, WriteCount);
				usleep(4000);
				MyRS485_Read(ReceiveInitMessage, 1, ReadCount);

				/*We make sure that the mesage come from actuator 6(0x15) and 
				that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				which is the answer of our message. (See document Kinova RS485 
				Communication protocol).*/
				if(ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS && 
				   ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					ActuatorInitialized = true;
				}
			}
		}
    }
	else
	{
		cout << "Errors while loading API's function" << endl;
	}

	return 0;
}

void InitForceMode()
{
    // ========== BEGIN MAIN COMM ==========
     
    // STEP 0: Get initial position 
    TrajectoryMessage.Command = 0x0001;  
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
    TrajectoryMessage.DataFloat[0] = 0x00000000; 
    TrajectoryMessage.DataLong[1] = 0x00000000; 
    TrajectoryMessage.DataFloat[2] = 0x00000000; 
    TrajectoryMessage.DataLong[3] = 0x00000000;
    //pthread_mutex_lock (&APIMutex);
    MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
    //pthread_mutex_unlock (&APIMutex);
    usleep(2000);
     
     
     
    MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
	    ReceiveInitMessage[0].Command == 0x0002)
    {
	    status.pos = ReceiveInitMessage[0].DataFloat[1];
	    cout << "Current position is: "
		    << ReceiveInitMessage[0].DataFloat[1] << endl;
    }

    bool ack = false; 

    SafetyMessage.Command =
	    0x0208;
    SafetyMessage.SourceAddress = SOURCE_ADDRESS;
    SafetyMessage.DestinationAddress = DESTINATION_ADDRESS;
    SafetyMessage.DataFloat[0] = 100.0; //10 Nm maximum torque
    SafetyMessage.DataFloat[1] = 1.0; //0.75 safety factor
    SafetyMessage.DataFloat[2] = 0.0; //not used
    SafetyMessage.DataFloat[3] = 0.0; //not used

    MyRS485_Write(&SafetyMessage, 1, WriteCount);
    while (ack = false)
    {
	    MyRS485_Write(&SafetyMessage, 1, WriteCount);
	    usleep(2000);
	    MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == 0x003E)
	    {
		    ack = true;
	    }

    }
     
     
     
    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    TrajectoryMessage.Command = 
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;  
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage.DataFloat[0] = status.pos;
    //not used 
    TrajectoryMessage.DataLong[1] = 0x00000000; 
    //32F torque command [Nm]
    TrajectoryMessage.DataFloat[2] = 0;
    TrajectoryMessage.DataLong[3] = ((unsigned long) torqueDamping | 
        ((unsigned long) controlMode << 8) | ((unsigned long) 
        torqueKp << 16)); //U16|U8|U8 

    //We send the command and we protect the process with a mutex
    /*
     * Param1(IN):  The buffer that contains all the messages.
     * Param2(IN):  Messages count.
     * Param3(OUT): Message sent count.
     */
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    cout << "Initializing position to: " << status.pos << endl;
    //for (int ii = 0; ii<500; ii++)
    //{
        //Joint6Command -= 40 * (0.0025);
        TrajectoryMessage.DataFloat[0] = status.pos;
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(2000);
    //}

    // STEP 2: Validate the torque command

    status.torqueValidation = false;

    //send position and torque command
    TrajectoryMessage.Command = GET_TORQUE_VALIDATION_REQUEST;  
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage.DataFloat[0] = 0; 
    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
    TrajectoryMessage.DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage.DataLong[3] = 0x00000000; //not used

    cout << "STEP 2: Request Torque Command Verification" << endl;

    while (status.torqueValidation == false)
    {
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
       // pthread_mutex_unlock (&APIMutex);
        usleep(2000);
        MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SEND_TORQUE_VALIDATION)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Torque Validation True : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.torqueValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Torque Validation False : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.flag = 10;
		    }
		    else
		    {
			    cout << "ERROR READING TORQUE VALIDATION REPLY: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }
    }

    // STEP 3: Switch to torque control mode

    status.switchValidation = false;

    TrajectoryMessage.Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;


    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00; 
    unsigned short d3 = 0x00; 
    TrajectoryMessage.DataLong[0] = ((unsigned short) 0x01 | 
        ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 | 
        ((unsigned short) d1 << 24))); //U24|U8 
    TrajectoryMessage.DataLong[1]=0x00000000; 
    TrajectoryMessage.DataLong[2]=0x00000000; 
    TrajectoryMessage.DataLong[3]=0x00000000;

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    do
    {
        cout << "waiting on reply for mode switch" << endl;
        //pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
        //pthread_mutex_unlock (&APIMutex);
        usleep(2000);
        MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.switchValidation = true;
		    }

		    if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }
    } while (status.switchValidation == false);

     cout << "Verified: Switching to Torque Mode" << endl;
}

void ApplyU(float us[6])
{
    // Step 4: Enjoy torque control mode!
			    
    TrajectoryMessage.Command = 
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND; 
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    //TrajectoryMessage.DataFloat[0] = status.pos; 
    TrajectoryMessage.DataLong[1] = 0x00000000; //not used
    TrajectoryMessage.DataFloat[2] = 2.0; //32F torque command [1Nm]
    TrajectoryMessage.DataLong[3] = ((unsigned long) torqueDamping | 
        ((unsigned long) controlMode << 8) | 
        ((unsigned long) torqueKp << 16)); //U16|U8|U8 


    cout << "STEP 4: Enjoy Torque Mode" << endl;

    for(int i=0; i<1000; i++)
    {
       // pthread_mutex_lock (&APIMutex);
        TrajectoryMessage.DataFloat[0] = status.pos;
        MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
       // pthread_mutex_unlock (&APIMutex);
        usleep(2000);
        MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == RS485_MSG_SEND_ALL_VALUES_1)
	    {
		    status.pos = ReceiveInitMessage[0].DataFloat[1];
		    cout << status.pos << endl;
	    }
    }
}

void GetFeedback(float q[6], float dq[6])
{

}

void Disconnect()
{
    status.switchValidation = false;

    TrajectoryMessage.Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage.SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage.DestinationAddress = DESTINATION_ADDRESS;


    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    TrajectoryMessage.DataLong[0] = ((unsigned short)0x00 |
	    ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
	    ((unsigned short)d1 << 24))); //U24|U8 
    TrajectoryMessage.DataLong[1] = 0x00000000;
    TrajectoryMessage.DataLong[2] = 0x00000000;
    TrajectoryMessage.DataLong[3] = 0x00000000;

    cout << "STEP 3: Waiting for Torque Verification" << endl;

    while (status.switchValidation == false)
    {
	    cout << "waiting on reply for mode switch" << endl;
	    //pthread_mutex_lock(&APIMutex);
	    MyRS485_Write(&TrajectoryMessage, 1, WriteCount);
	    //pthread_mutex_unlock(&APIMutex);
	    usleep(2000);
	    MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
	    if (ReceiveInitMessage[0].SourceAddress == DESTINATION_ADDRESS &&
		    ReceiveInitMessage[0].Command == SWITCH_CONTROL_MODE_REPLY)
	    {
		    if (ReceiveInitMessage[0].DataLong[0] == 257)
		    {
			    cout << "Switch Control Mode TORQUE True : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.switchValidation = true;
		    }

		    if (ReceiveInitMessage[0].DataLong[0] == 1)
		    {
			    cout << "Switch Control Mode POSITION True : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.switchValidation = true;
		    }

		    else if (ReceiveInitMessage[0].DataLong[0] == 0)
		    {
			    cout << "Switch Control Mode False : "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
			    status.flag = 11;
		    }
		    else
		    {
			    cout << "ERROR READING SWITCH CONTROL MODE REPLY: "
				    << ReceiveInitMessage[0].DataLong[0] << endl;
		    }
	    }


    cout << "Verified: Switching to position Mode" << endl;


    cout << "Exiting Main Control Loop" << endl;
}
