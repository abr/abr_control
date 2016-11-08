//MAIN(SEND information)
int main()
{
	//Variable needed during worker thread creation but not used after.
	int ThreadArgument = 0;

	//Instance of the controller
	MyController6DOF controller; 

	//status of the controller
	KinematicStatus status;

	//Joystickcontrol mode (initially in translation = 1; 2 for rotation)
	int Joymode = 1;

	//Control mode (Joystick or other)
	Ctrlmode mycrt= Joy;

	//We load the API.
	commLayer_Handle = LoadLibrary(L"CommunicationLayerWindows.dll");

	//Initialization of the fucntion pointers.
	fptrInitCommunication = (int(*)()) GetProcAddress(commLayer_Handle, "InitCommunication");
	MyRS485_Activate = (int(*)()) GetProcAddress(commLayer_Handle, "OpenRS485_Activate");
	MyRS485_Read = (int(*)(RS485_Message*, int, int &)) GetProcAddress(commLayer_Handle, "OpenRS485_Read");
	MyRS485_Write = (int(*)(RS485_Message*, int, int &)) GetProcAddress(commLayer_Handle, "OpenRS485_Write");

	

	//If all functions are loaded correctly.
	if (fptrInitCommunication != NULL || MyRS485_Activate != NULL || MyRS485_Read != NULL || MyRS485_Write != NULL)
	{
		//Initialization of the API
		int result = fptrInitCommunication();

		//If API's initialization is correct.
		if (result == NO_ERROR_KINOVA)
		{
			cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

			//We activate the RS-485 comm API. From here you cannot control the robot with the Joystick or
			//with the normal USB function. Only RS-485 command will be accepted. Reboot the robot to get
			//back to normal control.
			MyRS485_Activate();

			//Flag used during initialization.
			bool ActuatorInitialized = false;
			float JointPosition[NB_ACTUATOR_MAX] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			float Torque[300];
			float Current[300];

			//Message to ask for the actuators' position.
			RS485_Message InitMessage;
			//Message to receive the actuators' position.
			RS485_Message ReceiveInitMessage[5];

			//Variable used during the communication process.
			int WriteCount = 0;
			int ReadCount = 0;

			int check_jj;

			//  1 is the message's quantity and WriteCount will stored the Qty of messages sent.
			//MyRS485_Write(&InitMessage, 1, WriteCount);
			for (int i = 0; i < 1; i++)
			{
				// Reinitializations
				ReadCount = 0;
				WriteCount = 0;
				ActuatorInitialized = false;

				//Initialize the getposition message
				InitMessage.Command = RS485_MSG_GET_ACTUALPOSITION; //Setting the command ID
				InitMessage.SourceAddress = 0x00;                   //Setting the source address (0 means the API)
				InitMessage.DestinationAddress = ActuatorAddress[i];              //Setting the destinaiton address(0x10 is the actuator 6's default value)

																				  //Those value are not used for this command.
				InitMessage.DataLong[0] = 0x00000000;
				InitMessage.DataLong[1] = 0x00000000;
				InitMessage.DataLong[2] = 0x00000000;
				InitMessage.DataLong[3] = 0x00000000;

				//In case we did not received the answer, we continue reading until it's done
				while (ReadCount != 1 && !ActuatorInitialized)
				{
					MyRS485_Write(&InitMessage, 1, WriteCount);
					Sleep(4);
					MyRS485_Read(ReceiveInitMessage, 1, ReadCount);

					//We make sure that the mesage come from actuator i and that the command ID is RS485_MSG_SEND_ACTUALPOSITION
					//which is the answer of our message. (See document Kinova RS485 Communication protocol).
					if (ReceiveInitMessage[0].SourceAddress == ActuatorAddress[i] && ReceiveInitMessage[0].Command == RS485_MSG_SEND_ACTUALPOSITION)
					{
						JointPosition[i] = ReceiveInitMessage[0].DataFloat[1];
						ActuatorInitialized = true;
					}
					MyRS485_Read(ReceiveInitMessage, 1, ReadCount); //we read answer 2 twice
					// Reinitialization to 0
					ReadCount = 0;
					WriteCount = 0;
				}
			}

			//send command
			for (int i = 0; i < 300; i++)
			{
				ReadCount = 0;
				int messageread = 0;

				//Initialize the getposition message
				InitMessage.Command = 0x0010; //Setting the command ID
				InitMessage.SourceAddress = 0x00;                   //Setting the source address (0 means the API)
				InitMessage.DestinationAddress = ActuatorAddress[i];              //Setting the destinaiton address(0x10 is the actuator 6's default value)

																				  //Those value are not used for this command.
				InitMessage.DataLong[0] = JointPosition[0] + (float)(40 * (0.0025));
				InitMessage.DataLong[1] = JointPosition[0] + (float)(40 * (0.0025));
				InitMessage.DataLong[2] = 0x00000000;
				InitMessage.DataLong[3] = 0x00000000;

				MyRS485_Write(&InitMessage, 1, WriteCount);
				Sleep(2);
				while (messageread < 1)
				{
					MyRS485_Read(ReceiveInitMessage, 1, ReadCount);
					messageread += ReadCount;
				}
				
			}
		}
	}
}