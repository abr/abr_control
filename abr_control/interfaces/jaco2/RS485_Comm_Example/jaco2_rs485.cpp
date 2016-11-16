#include "jaco2_rs485.h"
#include <math.h>

Jaco2::Jaco2(void) {
    //set common variables
    flag = 0;
    torqueValidation = false;
    switchValidation = false;
    read_input = true;
    delay = 2000;
    packets_sent = 6;
    packets_read = 18; //response 14 + 15 + 16 per servo
    ActuatorInitialized = 0;

    torqueDamping = 0x01;
    controlMode = 0x01;
    torqueKp = 1750; // torque kp 1.75 * 1000
    
    // set max torque in Nm
    maxT[0] = 100.0;
    maxT[1] = 100.0;
    maxT[2] = 100.0;
    maxT[3] = 100.0;
    maxT[4] = 100.0;
    maxT[5] = 100.0;

    WriteCount = 0;
    ReadCount = 0;
    
    //joint addresses from base to wrist
    joint[0] = 0x10;
    joint[1] = 0x11;
    joint[2] = 0x12;
    joint[3] = 0x13;
    joint[4] = 0x14;
    joint[5] = 0x15;

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
    cout << "T H R E E" << endl;
    usleep(1000000);
    cout << "T W O" << endl;
    usleep(1000000);
    cout << "O N E" << endl;
    usleep(1000000);
    
    Jaco2 j2 = Jaco2();
    float u[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    
    j2.Connect(); 
    j2.InitForceMode();
    
    for (int i = 0; i<5000; i++)
    {
        //u[5] = 4.0 * sin(2.0*3.14159 * i/3500);
        j2.ApplyU(u);
    }
    
    j2.Disconnect();
}

void Jaco2::Connect()
{
	cout << "RS-485 communication Initialization" << endl;
	//Flag used during initialization.
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

            for (int ii = 0; ii<6; ii++)
            {
			    //Initialize the INIT message
			    InitMessage[ii].Command = RS485_MSG_GET_ACTUALPOSITION; 
			    InitMessage[ii].SourceAddress = SOURCE_ADDRESS;//0 means the API
			    InitMessage[ii].DestinationAddress = joint[ii];         

			    //Those value are not used for this command.
			    InitMessage[ii].DataLong[0] = 0x00000000;
			    InitMessage[ii].DataLong[1] = 0x00000000;
			    InitMessage[ii].DataLong[2] = 0x00000000;
			    InitMessage[ii].DataLong[3] = 0x00000000;
            }

			//If we did not receive the answer, continue reading until done
			while(ReadCount != 1 && ActuatorInitialized < 6)
			{
				MyRS485_Write(InitMessage, packets_sent, WriteCount);
				usleep(2000);
				MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);

                ActuatorInitialized = 0;
                
				// verify message type and servo address
				for (int ii = 0; ii<6; ii++)
				{
				    if(ReceiveInitMessage[ii].SourceAddress == joint[ii] &&
				       ReceiveInitMessage[ii].Command == RS485_MSG_SEND_ACTUALPOSITION)
				    {
					    ActuatorInitialized += 1;
					    cout << "Actuator " << ii << " initialized" << endl;
				    }
				    else
				    {
				        cout << "Error while initializing actuator " << ii << endl;
				    }
			    }
			}
			ActuatorInitialized = 0; // reset for next use
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

void Jaco2::InitForceMode()
{
    // ========== BEGIN MAIN COMM ==========
    // STEP 0: Get initial position    
    
    for (int ii=0; ii<6; ii++)
    {
        TrajectoryMessage[ii].Command = 0x0001;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];
        TrajectoryMessage[ii].DataFloat[0] = 0x00000000;
        TrajectoryMessage[ii].DataLong[1] = 0x00000000;
        TrajectoryMessage[ii].DataFloat[2] = 0x00000000;
        TrajectoryMessage[ii].DataLong[3] = 0x00000000;
     }   

    MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);
    usleep(delay);
    MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
    
    while(ReadCount != 1 && ActuatorInitialized < 6)
    {
        ActuatorInitialized = 0;
        
        for (int ii = 0; ii<6; ii++)
	    {
	        if(ReceiveInitMessage[ii].SourceAddress == joint[ii] &&
	           ReceiveInitMessage[ii].Command == SEND_ACTUAL_POSITION)
	        {
	            pos[ii] = ReceiveInitMessage[ii].DataFloat[1];
		        ActuatorInitialized += 1;
		        cout << "Actuator " << ii << " position is " << pos[ii] << endl;
	        }
	        else
	        {
	            cout << "Error while obtaining actuator " << ii 
                     << " position" <<endl;
	        }
        }
    }
    
    ActuatorInitialized = 0; // reset for next use

    // Set torque safety parameters
    for (int ii=0; ii<6; ii++)
    {
        SafetyMessage[ii].Command = SEND_TORQUE_CONFIG_SAFETY;
        SafetyMessage[ii].SourceAddress = SOURCE_ADDRESS;
        SafetyMessage[ii].DestinationAddress = joint[ii];
        SafetyMessage[ii].DataFloat[0] = maxT[ii]; // Nm maximum torque
        SafetyMessage[ii].DataFloat[1] = 1.0; //0.75 safety factor
        SafetyMessage[ii].DataFloat[2] = 0.0; //not used
        SafetyMessage[ii].DataFloat[3] = 0.0; //not used
    }    

    MyRS485_Write(SafetyMessage, packets_sent, WriteCount);
    usleep(delay);
    MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
    
    /*while(ReadCount != 1 && ActuatorInitialized < 6)
    {
        ActuatorInitialized = 0;
        
        for (int jj = 0; jj<6; jj++)
        {
            for (int ii = 0; ii<18; ii++)
            {
	            if (ReceiveInitMessage[ii].SourceAddress == joint[jj] &&
		            ReceiveInitMessage[ii].Command == GET_TORQUE_CONFIG_SAFETY)
	            {
		            ActuatorInitialized += 1;
		            cout << "Safety passed for servo " << jj << " with read entry: " << ii << endl;
		            break;
	            }
	            else
                {
                    cout << "Error passing safety test for servo " << jj << endl;
                }
            }
        }
        MyRS485_Write(SafetyMessage, packets_sent, WriteCount);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);

    }*/
    
    // FIX THIS ++++++++++++++++++++=====================fawfgafavasafasjkvbawkfjN~~!!!!!!!
    /*bool ack = false;
    while (ack == false)
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

    }*/
    
    ActuatorInitialized = 0; // reset for next use

    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    for (int ii=0; ii<6; ii++)
    {
        TrajectoryMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];
        //32F position command [deg]
        TrajectoryMessage[ii].DataFloat[0] = pos[ii];
        //not used
        TrajectoryMessage[ii].DataLong[1] = 0x00000000;
        //32F torque command [Nm]
        TrajectoryMessage[ii].DataFloat[2] = 0;
        TrajectoryMessage[ii].DataLong[3] = ((unsigned long) torqueDamping |
            ((unsigned long) controlMode << 8) | ((unsigned long)
            torqueKp << 16)); //U16|U8|U8
    }
    
    cout << "STEP 1: Send Torque Command for Verification" << endl;
    //cout << "Initializing base position to: " << pos[0] << endl;
    //cout << "Initializing wrist position to: " << pos[1] << endl;
    //for (int ii = 0; ii<6; ii++)
    //{
        //TrajectoryMessage[ii].DataFloat[0] = pos[ii];
        //TrajectoryMessage[ii].DataFloat[0] = pos[ii];
    MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);
    usleep(delay);
    //}

    // STEP 2: Validate the torque command

    torqueValidation = false;
    for (int ii=0; ii<6; ii++)
    {
    //send position and torque command
        TrajectoryMessage[ii].Command = GET_TORQUE_VALIDATION_REQUEST;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];//DESTINATION_ADDRESS;
        //32F torque command to be tested [Nm]
        TrajectoryMessage[ii].DataFloat[0] = 0;
        TrajectoryMessage[ii].DataLong[1] = 0x00000000; //not used
        TrajectoryMessage[ii].DataFloat[2] = 0x00000000; //not used
        TrajectoryMessage[ii].DataLong[3] = 0x00000000; //not used
    }

    cout << "STEP 2: Request Torque Command Verification" << endl;

    while (ReadCount != 1 && ActuatorInitialized < 6)
    {
        ActuatorInitialized = 0;
        for (int ii=0; ii<6; ii++)
        {
            MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);
            usleep(delay);
            MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
            usleep(delay); // always misses first read, maybe delay will help
            
	        if (ReceiveInitMessage[ii].SourceAddress == joint[ii] &&
		        ReceiveInitMessage[ii].Command == SEND_TORQUE_VALIDATION)
	        {
		        if (ReceiveInitMessage[ii].DataLong[0] == 1)
		        {
			        cout << "Torque Validation True servo : " << ii << " response: "
				        << ReceiveInitMessage[ii].DataLong[0] << endl;
			        ActuatorInitialized += 1;
		        }

		        else if (ReceiveInitMessage[ii].DataLong[0] == 0)
		        {
			        cout << "Torque Validation False servo : " << ii << " response: "
				        << ReceiveInitMessage[ii].DataLong[0] << endl;
			        flag = 10;
		        }
		        else
		        {
			        cout << "ERROR READING TORQUE VALIDATION REPLY SERVO: " << ii << " RESPONSE: "
				        << ReceiveInitMessage[ii].DataLong[0] << endl;
		        }
	        }
	        else
	        {
	            cout << "Torque Validation Reply Not Matching" << endl;
            }
        }
    }
    
    ActuatorInitialized = 0;

    // STEP 3: Switch to torque control mode

    switchValidation = false;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    
    for(int ii=0; ii<6; ii++)
    {
        TrajectoryMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];//DESTINATION_ADDRESS;
        TrajectoryMessage[ii].DataLong[0] = ((unsigned short) 0x01 |
            ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
            ((unsigned short) d1 << 24))); //U24|U8
        TrajectoryMessage[ii].DataLong[1]=0x00000000;
        TrajectoryMessage[ii].DataLong[2]=0x00000000;
        TrajectoryMessage[ii].DataLong[3]=0x00000000;
    }
    cout << "STEP 3: Waiting for Torque Verification" << endl;

    while (ReadCount != 1 && ActuatorInitialized < 6)
    {
        ActuatorInitialized = 0;
        cout << "waiting on reply for mode switch" << endl;
        MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);
        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
        
        for(int jj=0; jj<6; jj++)
        {
            for(int ii=0; ii<18; ii++)
            {
	            if (ReceiveInitMessage[ii].SourceAddress == joint[jj] &&
		            ReceiveInitMessage[ii].Command == SWITCH_CONTROL_MODE_REPLY)
	            {
		            if (ReceiveInitMessage[ii].DataLong[0] == 257)
		            {
			            cout << "Switch Control Mode TORQUE True servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            ActuatorInitialized += 1;
			            ii = 18;
		            }

		            else if (ReceiveInitMessage[ii].DataLong[0] == 1)
		            {
			            cout << "Switch Control Mode POSITION True servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            //ActuatorInitialized += 1;
		            }

		            else if (ReceiveInitMessage[ii].DataLong[0] == 0)
		            {
			            cout << "Switch Control Mode False servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            flag = 11;
		            }
		            else
		            {
			            cout << "ERROR READING SWITCH CONTROL MODE REPLY servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
		            }
	            }
            }
        }
    } 

     cout << "Step 4: Verified: Switching to Torque Mode" << endl;
}

void* Jaco2::ApplyU(float us[6])
{
    // Step 4: Enjoy torque control mode!
    for (int ii=0; ii<6; ii++)
    {
        TrajectoryMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];//DESTINATION_ADDRESS;
        //32F position command [deg]
        TrajectoryMessage[ii].DataFloat[0] = pos[ii];
        TrajectoryMessage[ii].DataLong[1] = 0x00000000; //not used
        TrajectoryMessage[ii].DataFloat[2] = us[ii]; //32F torque command [1Nm]
        TrajectoryMessage[ii].DataLong[3] = ((unsigned long) torqueDamping |
            ((unsigned long) controlMode << 8) |
            ((unsigned long) torqueKp << 16)); //U16|U8|U8
    }
    //cout << "STEP 4: Enjoy Torque Mode" << endl;
    //for(int i=0; i<2000; i++)
    //{
        //TrajectoryMessage[0].DataFloat[0] = pos[0];
        //TrajectoryMessage[1].DataFloat[0] = pos[1];
        //cout << "pos0: " << pos[0] << endl;
        //cout << "pos1: " << pos[1] << endl;
        MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);

        usleep(delay);
        MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
        //cout << "R E A D  C O U N T  " << ReadCount << endl;
        //cout << "Base source address: " << ReceiveInitMessage[0].SourceAddress << endl;
        //cout << "Wrist source address: " << ReceiveInitMessage[1].SourceAddress << endl;
        //cout << "Base command: " << ReceiveInitMessage[0].Command << endl;
        //cout << "Wrist command: " << ReceiveInitMessage[1].Command << endl;
        //cout << "expected command: " << RS485_MSG_SEND_ALL_VALUES_1 << endl;
        //cout << "base pos: " << ReceiveInitMessage[0].DataFloat[1] << endl;
        //cout << "wrist pos: " << ReceiveInitMessage[1].DataFloat[1] << endl;
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
	    
	    for (int jj = 0; jj < 6; jj++)
	    {
	        for (int ii = 0; ii < packets_read; ii++) {
	            ii = ii;
                if (ReceiveInitMessage[ii].SourceAddress == joint[jj]) {
                    pos[jj] = ReceiveInitMessage[ii].DataFloat[1];
                    //cout << "pos[0]: " << pos[0] << endl;;
                    break;
                }
            }
        }

        /*for (int ii = 0; ii < 6; ii++) {
            if (ReceiveInitMessage[ii].SourceAddress == 0x15) {
                pos[1] = ReceiveInitMessage[ii].DataFloat[1];
                //cout << "pos[1]: " << pos[1] << endl;
                break;
            }
        }*/
	    
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

void Jaco2::Disconnect()
{
    switchValidation = false;
    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;
    
    for (int ii=0; ii<6; ii++)
    {
        TrajectoryMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        TrajectoryMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TrajectoryMessage[ii].DestinationAddress = joint[ii];
        TrajectoryMessage[ii].DataLong[0] = ((unsigned short)0x00 |
	        ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
	        ((unsigned short)d1 << 24))); //U24|U8
        TrajectoryMessage[ii].DataLong[1] = 0x00000000;
        TrajectoryMessage[ii].DataLong[2] = 0x00000000;
        TrajectoryMessage[ii].DataLong[3] = 0x00000000;
    }
    
    cout << "STEP 5: Waiting for Position Verification" << endl;
    
    ActuatorInitialized = 0;
    while (ReadCount != 1 && ActuatorInitialized < 6)
    {
        ActuatorInitialized = 0;
	    cout << "waiting on reply for mode switch" << endl;
	    MyRS485_Write(TrajectoryMessage, packets_sent, WriteCount);
	    usleep(delay);
	    MyRS485_Read(ReceiveInitMessage, packets_read, ReadCount);
	    
	    for (int jj=0; jj<6; jj++)
	    {
	        for (int ii=0; ii<18; ii++)
	        {
	            if (ReceiveInitMessage[ii].SourceAddress == joint[jj] &&
		            ReceiveInitMessage[ii].Command == SWITCH_CONTROL_MODE_REPLY)
	            {
		            if (ReceiveInitMessage[ii].DataLong[0] == 257)
		            {
			            cout << "Switch Control Mode TORQUE True servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            //ActuatorInitialized += 1;
		            }

		            else if (ReceiveInitMessage[ii].DataLong[0] == 1)
		            {
			            cout << "Switch Control Mode POSITION True servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            ActuatorInitialized += 1;
			            ii = 18;
		            }

		            else if (ReceiveInitMessage[ii].DataLong[0] == 0)
		            {
			            cout << "Switch Control Mode False servo: " << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
			            flag = 11;
		            }
		            else
		            {
			            cout << "ERROR READING SWITCH CONTROL MODE REPLY SERVO: "  << jj << " msg: "
				            << ReceiveInitMessage[ii].DataLong[0] << endl;
		            }
	            }
            }
        }
    }
    cout << "Verified: Switching to position Mode" << endl;


    cout << "Exiting Main Control Loop" << endl;
}
