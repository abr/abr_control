#include "jaco2_rs485.h"

//A mutex to protect the access to the API.
pthread_mutex_t APIMutex;


// ========== Initialization ==========
int Init(InitStruct* args)
{  
    InitStruct* init = (InitStruct *)args;
    
    ConnectStruct c;
    
    InitForceModeStruct ifm;
    
    ApplyUStruct au;

    FeedbackStruct fb;
    fb.flag = 0;
    fb.torqueValidation = false;
    fb.switchValidation = false;
    fb.read_input = true;
    
    DisconnectStruct d;

	//The worker thread.
	pthread_t GetPositionThread;

	//Message to initialize the actuator command.
	RS485_Message InitMessage;

	//Message to receive the actuator position.
	RS485_Message ReceiveInitMessage;

	//Message to move the actuator.
	RS485_Message TrajectoryMessage;
}

// ========== Connect to Jaco2 via RS485 ==========
void* Connect(ConnectStruct* args, RS485_Message* initArgs, RS485_Message* rcvArgs, InitStruct* initArgs)
{
    ConnectStruct* c = (ConnectStruct *)args;
    RS485_Message* InitMessage = (RS485_Message *)initArgs;
    RS485_Message* ReceiveInitMessage = (RS485_Message *)rcvArgs;
    InitStruct* init = (InitStruct *)initArgs;
    
	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int (*)()) dlsym(c->commLayer_Handle,
	                                          "InitCommunication");
	MyRS485_Activate = (int (*)()) dlsym(c->commLayer_Handle,"RS485_Activate");
	MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted, 
	                        int &ReceivedQtyIn)) dlsym(c->commLayer_Handle,
	                        "RS485_Read");
	MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted, 
                             int &ReceivedQtyIn)) dlsym(c->commLayer_Handle,
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
			InitMessage->Command = RS485_MSG_GET_ACTUALPOSITION; //Set command ID
			InitMessage->SourceAddress = SOURCE_ADDRESS;        //0 means the API
			InitMessage->DestinationAddress = DESTINATION_ADDRESS;//destinaiton 

			//Those value are not used for this command.
			InitMessage->DataLong[0] = 0x00000000;
			InitMessage->DataLong[1] = 0x00000000;
			InitMessage->DataLong[2] = 0x00000000;
			InitMessage->DataLong[3] = 0x00000000;

			//Send the Init Message. 1 is the message's quantity and WriteCount 
			//will stored the Qty of messages sent.
			MyRS485_Write(&InitMessage, 1, init->WriteCount);

			//If we did not receive the answer, continue reading until done
			while(init->ReadCount != 1 && !init->ActuatorInitialized)
			{
				MyRS485_Write(&InitMessage, 1, init->WriteCount);
				usleep(4000);
				MyRS485_Read(&ReceiveInitMessage, 1, init->ReadCount);

				/*We make sure that the mesage come from actuator 6(0x15) and 
				that the command ID is RS485_MSG_SEND_ACTUALPOSITION
				which is the answer of our message. (See document Kinova RS485 
				Communication protocol).*/
				if(ReceiveInitMessage->SourceAddress == DESTINATION_ADDRESS && 
				   ReceiveInitMessage->Command == RS485_MSG_SEND_ACTUALPOSITION)
				{
					//Joint6Command = ReceiveInitMessage->DataFloat[1];
					init->ActuatorInitialized = true;
				}
			}

			//Creation of the thread that will get information from the robot.
			if(pthread_create(&GetPositionThread, NULL, 
			                  &get_feedback, (void *)&fb))
			{
				cout << "Error while creating thread" << endl;
				return ERROR_OPERATION_INCOMPLETED;
			}
			else
			{
			    cout << "Reading Thread Initialized" << endl;
			    c->connection_init = true;
			}
	    }
    }
    else
	{
		cout << "Errors while loading API's function" << endl;
	}

	return 0;
}
// ========== Initialize Force Mode ==========
void* InitForceMode(InitForceModeStruct* args, RS485_Message* trjArgs, InitStruct* initArgs, ApplyUStruct* trqArgs)
{
    InitForceModeStruct* f = (InitForceModeStruct *)args;
    RS485_Message* TrajectoryMessage = (RS485_Message *)trjArgs
    InitStruct* init = (InitStruct *)initArgs;
    ApplyUStruct* au = (ApplyUStruct *)trqArgs;
    
    // ========== BEGIN MAIN COMM ==========
			     
    // STEP 0: Get initial position 
    TrajectoryMessage->Command = 0x0001;  
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    TrajectoryMessage->DataFloat[0] = 0x00000000; 
    TrajectoryMessage->DataLong[1] = 0x00000000; 
    TrajectoryMessage->DataFloat[2] = 0x00000000; 
    TrajectoryMessage->DataLong[3] = 0x00000000;
    pthread_mutex_lock (&APIMutex);
    MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
    pthread_mutex_unlock (&APIMutex);
    usleep(init->delay);
     
    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    TrajectoryMessage->Command = 
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;  
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage->DataFloat[0] = fb.pos;
    //not used 
    TrajectoryMessage->DataLong[1] = 0x00000000; 
    //32F torque command [Nm]
    TrajectoryMessage->DataFloat[2] = 0;
    TrajectoryMessage->DataLong[3] = ((unsigned long) au->torqueDamping | 
        ((unsigned long) au->controlMode << 8) | ((unsigned long) 
        au->torqueKp <<   16)); //U16|U8|U8 

    //We send the command and we protect the process with a mutex
    /*
     * Param1(IN):  The buffer that contains all the messages.
     * Param2(IN):  Messages count.
     * Param3(OUT): Message sent count.
     */
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    cout << "Initializing position to: " << fb.pos << endl;
    for (int ii = 0; ii<500; ii++)
    {
        //Joint6Command -= 40 * (0.0025);
	    TrajectoryMessage->DataFloat[0] = fb.pos;
        pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
        pthread_mutex_unlock (&APIMutex);
        usleep(init->delay);
    }
    
    // STEP 2: Validate the torque command
    
    fb.torqueValidation = false;
    
    //send position and torque command
    TrajectoryMessage->Command = GET_TORQUE_VALIDATION_REQUEST;  
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    //32F torque command to be tested [Nm]
    TrajectoryMessage->DataFloat[0] = 0; 
    TrajectoryMessage->DataLong[1] = 0x00000000; //not used
    TrajectoryMessage->DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage->DataLong[3] = 0x00000000; //not used

    cout << "STEP 2: Request Torque Command Verification" << endl;
    
    while (fb.torqueValidation == false)
    {
        pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
        pthread_mutex_unlock (&APIMutex);
        usleep(init->delay);
    }
    
    // STEP 3: Switch to torque control mode
    
    fb.switchValidation = false;
    
    TrajectoryMessage->Command = SWITCH_CONTROL_MODE_REQUEST;
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    /*TrajectoryMessage->DataFloat[0] = 1.0; //control mode
    TrajectoryMessage->DataLong[1] = 0x00000000; //not used
    TrajectoryMessage->DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage->DataLong[3] = 0x00000000; //not used*/

    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00; 
    unsigned short d3 = 0x00; 
    TrajectoryMessage->DataLong[0] = ((unsigned short) 0x01 | 
        ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 | 
        ((unsigned short) d1 << 24))); //U24|U8 
    TrajectoryMessage->DataLong[1]=0x00000000; 
    TrajectoryMessage->DataLong[2]=0x00000000; 
    TrajectoryMessage->DataLong[3]=0x00000000;
    
    cout << "STEP 3: Waiting for Torque Verification" << endl;
    
    do
    {
        cout << "waiting on reply for mode switch" << endl;
        pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
        pthread_mutex_unlock (&APIMutex);
        usleep(init->delay);
    } while (fb.switchValidation == false);
    
     cout << "Verified: Switching to Torque Mode" << endl;
			     
// ========== Terminate Serial Connection ==========
void* Disconnect(void* args, RS485_Message* trjArgs, InitStruct* initArgs)
{
    DisconnectStruct* d = (DisconnectStruct *)args;
    RS485_Message* TrajectoryMessage = (RS485_Message *)trjArgs; 
    InitStruct* init = (InitStruct *)initArgs;
    // Step 5: Swith back to position mode in case of error shut down
    // need to read current position to set the target to match it
    // to avoid sudden fast movements
    
    /*fb.switchValidation = false;
    
    TrajectoryMessage->Command = SWITCH_CONTROL_MODE_REQUEST; 
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    TrajectoryMessage->DataFloat[0] = 0x00000000; //control mode
    TrajectoryMessage->DataLong[1] = 0x00000000; //not used
    TrajectoryMessage->DataFloat[2] = 0x00000000; //not used
    TrajectoryMessage->DataLong[3] = 0x00000000; //not used

    cout << "STEP 5: Request to switch to Position Control" << endl;

    while (fb.switchValidation == false)
    {
        pthread_mutex_lock (&APIMutex);
        MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
        pthread_mutex_unlock (&APIMutex);
        usleep(init->delay);
    }
    
    cout << "Verified: Switching to Position Mode" << endl;
    
    //send position and torque command
    TrajectoryMessage->Command = 
        RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;  
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage->DataFloat[0] = fb.pos; 
    TrajectoryMessage->DataLong[1] = 0x00000000; //not used
    TrajectoryMessage->DataFloat[2] = 0x00000000; //32F torque [Nm]
    TrajectoryMessage->DataLong[3] = 0x00010001; //U16|U16
    
    pthread_mutex_lock (&APIMutex);
    MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
    pthread_mutex_unlock (&APIMutex);
    usleep(init->delay);
    
    pthread_join(GetPositionThread, NULL);
    fb.read_input = false;*/
}
// ========== RS485 APPLY U ========== 
void* ApplyU(void* args, RS485_Message* trjArg, InitStruct* initArgs)
{
    ApplyUStruct* au = (ApplyUStruct *)args;
    RS485_Message* TrajectoryMessage = (RS485_Message *)trjArgs;
    InitStruct* init = (InitStruct *)initArgs;
    // Step 4: Enjoy torque control mode!
	        
    TrajectoryMessage->Command = 
        RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND; 
    TrajectoryMessage->SourceAddress = SOURCE_ADDRESS;
    TrajectoryMessage->DestinationAddress = DESTINATION_ADDRESS;
    //32F position command [deg]
    TrajectoryMessage->DataFloat[0] = fb.pos; 
    TrajectoryMessage->DataLong[1] = 0x00000000; //not used
    TrajectoryMessage->DataFloat[2] = 0; //32F torque command [1Nm]
    TrajectoryMessage->DataLong[3] = ((unsigned long) au->torqueDamping | 
        ((unsigned long) au->controlMode << 8) | 
        ((unsigned long) au->torqueKp << 16)); //U16|U8|U8 


    //cout << "STEP 4: Enjoy Torque Mode" << endl;

    pthread_mutex_lock (&APIMutex);
    MyRS485_Write(&TrajectoryMessage, 1, init->WriteCount);
    pthread_mutex_unlock (&APIMutex);
    usleep(init->delay);
}
// ========== RS485 READ ==========
void* GetFeedback(void *args, InitStruct* initArgs)
{
    GetFeedbackStruct* fb = (GetFeedbackStruct *)args;
    InitStruct* init = (InitStruct *)initArgs;
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

    while (fb->read_input == true)
    {
        MessageReadCount = 0;
	    pthread_mutex_lock (&APIMutex);
	    MyRS485_Read(MessageListIn, 3, MessageReadCount);

	    pthread_mutex_unlock (&APIMutex);

	    for(int j = 0; j < MessageReadCount; j++)
	    {
		    if(MessageListIn[j].Command == 0x0002)
		    {
		        fb->pos = MessageListIn[j].DataFloat[1];
		        cout << "Current position is: " 
		             << MessageListIn[j].DataFloat[1] << endl;
		    }
		    if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_1)
		    {
			  //cout << "Current  = " << MessageListIn[j].DataFloat[0] << endl;
			  //cout << "Position = " << MessageListIn[j].DataFloat[1] << endl;
			  //cout << "Velocity = " << MessageListIn[j].DataFloat[2] << endl;
			  //cout << "Torque   = " << MessageListIn[j].DataFloat[3] << endl;
			    fb->pos = MessageListIn[j].DataFloat[1];
		    }

		    if(MessageListIn[j].Command == RS485_MSG_SEND_ALL_VALUES_2)
		    {
			    //cout << "PWM  = " << MessageListIn[j].DataFloat[0] << endl;
			    //cout << "Position encoder = " << MessageListIn[j].DataFloat[1] 
			    //     << endl;

			    accelX = (short)(MessageListIn[j].DataLong[2] & 0x0000FFFF);
			    accelY = (short)((MessageListIn[j].DataLong[2] & 0xFFFF0000) 
			                     >> 16);
			    accelZ = (short)(MessageListIn[j].DataLong[3] & 0x0000FFFF);
			    temperature = (short)((MessageListIn[j].DataLong[3] & 
			                          0xFFFF0000) >> 16);

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
			    cout << "Motor current = " << MessageListIn[j].DataFloat[0] 
			         << endl;
			    cout << "Absolute position = " << MessageListIn[j].DataFloat[1] 
			         << endl;
		    }
		
		    // ---------- Safety Measure Checks
		    if(MessageListIn[j].Command == SEND_TORQUE_VALIDATION)
		    {
		        if(MessageListIn[j].DataLong[0] == 1)
		        {
		            cout << "Torque Validation True : " 
		                 << MessageListIn[j].DataLong[0] << endl;
		            fb->torqueValidation = true;
		        }
		        
		        else if(MessageListIn[j].DataLong[0] == 0)
		        {
		            cout << "Torque Validation False : " 
		                 << MessageListIn[j].DataLong[0] << endl;
		            fb->flag = 10;
		        } 
		        else
		        {
		            cout << "ERROR READING TORQUE VALIDATION REPLY: " 
		                 << MessageListIn[j].DataLong[0]<< endl;
	            }
		    }
		    if(MessageListIn[j].Command == SWITCH_CONTROL_MODE_REPLY)
		    {
		        if(MessageListIn[j].DataLong[0] == 257)
		        {
		            cout << "Switch Control Mode True : " 
		                 << MessageListIn[j].DataLong[0] << endl;
		            fb->switchValidation = true;
		        }
		        
		        else if(MessageListIn[j].DataLong[0] == 0)
		        {
		            cout << "Switch Control Mode False : " 
		                 << MessageListIn[j].DataLong[0] << endl;
		            fb->flag = 11;
		        } 
		        else
		        {
		            cout << "ERROR READING SWITCH CONTROL MODE REPLY: " 
		                 << MessageListIn[j].DataLong[0]<< endl;
	            }
		    }
		    if(MessageListIn[j].Command == REPORT_ERROR)
		    {
		        if(MessageListIn[j].DataLong[0] != 0)
		        {
		            cout << "Error number : " << MessageListIn[j].DataLong[0] 
		                 << endl; //may be [1] instead of 0
		            fb->flag = 1; // will add error specific values later
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
	    usleep(init->delay);
	}
	
	pthread_exit(0);
    return NULL;
}
