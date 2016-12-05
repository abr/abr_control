/*
AUTHORS: Martine Blouin, Pawel Jaworski, Travis Dewolf
*/

#include "jaco2_rs485.h"

Jaco2::Jaco2() {

    //set common variables
    flag = 0;
    read_input = true;
    delay = 2000;
    packets_sent = 6;
    packets_read = 18; //response 14 + 15 + 16 per servo
    messageReceived = 0; //counter to see if all joint messages read correctly
    ActuatorInitialized = 0; // TO DO: DELETE
    currentJoint = 6; // only 6 joints so if source address is not <6 after
                      // reading should receive error due do array size

    memset(updated, 0, (size_t)sizeof(int)*6);

    torqueDamping = 0x01;
    controlMode = 0x01;
    torqueKp = 1750; // torque kp 1.75 * 1000

    // set max torque in Nm
    memset(maxT, 100.0, (size_t)sizeof(float)*6);

    WriteCount = 0;
    ReadCount = 0;

    //joint addresses from base to wrist
    joint[0] = 0x10;
    joint[1] = 0x11;
    joint[2] = 0x12;
    joint[3] = 0x13;
    joint[4] = 0x14;
    joint[5] = 0x15;

    // error messages from arm
    errorMessage.push_back("NO");
    errorMessage.push_back("TEMPERATURE");
    errorMessage.push_back("TEMPERATURE");
    errorMessage.push_back("VELOCITY");
    errorMessage.push_back("POSITION LIMITATION");
    errorMessage.push_back("ABSOLUTE POSITION");
    errorMessage.push_back("RELATIVE POSITION");
    errorMessage.push_back("COMMAND");
    errorMessage.push_back("CURRENT");
    errorMessage.push_back("TORQUE");

    // set constants in force message to increase loop speed
    for (int ii=0; ii<6; ii++)
    {
        ForceMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        ForceMessage[ii].SourceAddress = SOURCE_ADDRESS;
        ForceMessage[ii].DataLong[1] = 0x00000000; //not used
        ForceMessage[ii].DataLong[3] = ((unsigned long) torqueDamping |
            ((unsigned long) controlMode << 8) |
            ((unsigned long) torqueKp << 16)); //U16|U8|U8
    }

    //We load the API.
    commLayer_Handle = dlopen(
        "./Kinova.API.CommLayerUbuntu.so",
        RTLD_NOW|RTLD_GLOBAL);

    if (!commLayer_Handle)
    {
        fprintf(stderr, "dlopen failed: %s\n", dlerror());
        exit(EXIT_FAILURE);
    }

    //Initialization of the function pointers.
    fptrInitCommunication = (int (*)()) dlsym(commLayer_Handle,
                                              "InitCommunication");
    fptrCloseCommunication = (int (*)()) dlsym(commLayer_Handle,
                                              "CloseCommunication");
    MyRS485_Activate = (int (*)()) dlsym(commLayer_Handle,"RS485_Activate");
    MyRS485_Read = (int (*)(RS485_Message* PackagesIn, int QuantityWanted,
                            int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                            "RS485_Read");
    MyRS485_Write = (int (*)(RS485_Message* PackagesOut, int QuantityWanted,
                               int &ReceivedQtyIn)) dlsym(commLayer_Handle,
                               "RS485_Write");

    unsigned short d1 = 0x00;
    unsigned short d2 = 0x00;
    unsigned short d3 = 0x00;

    // Set up static parts of messages sent across
    // Set up the message used by ApplyQ
    for (int ii = 0; ii<6; ii++)
    {
        ApplyQMessage[ii].Command = 0x0014;//RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES;
        ApplyQMessage[ii].SourceAddress = 0x00;
        ApplyQMessage[ii].DestinationAddress = joint[ii];
        ApplyQMessage[ii].DataLong[2] = 0x1;
        ApplyQMessage[ii].DataLong[3] = 0x00000000;
    }

    // Set up get position message
    for (int ii=0; ii<6; ii++)
    {
        GetPositionMessage[ii].Command = 0x0001;
        GetPositionMessage[ii].SourceAddress = SOURCE_ADDRESS;
        GetPositionMessage[ii].DestinationAddress = joint[ii];
        GetPositionMessage[ii].DataFloat[0] = 0x00000000;
        GetPositionMessage[ii].DataLong[1] = 0x00000000;
        GetPositionMessage[ii].DataFloat[2] = 0x00000000;
        GetPositionMessage[ii].DataLong[3] = 0x00000000;
     }

    // Set up robot initialization message
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

    // Set up initialize position mode message
    for (int ii=0; ii<6; ii++)
    {
        InitPositionMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        InitPositionMessage[ii].SourceAddress = SOURCE_ADDRESS;
        InitPositionMessage[ii].DestinationAddress = joint[ii];
        InitPositionMessage[ii].DataLong[0] = ((unsigned short)0x00 |
          ((unsigned short)d2 << 8) | ((unsigned short)d3 << 16 |
          ((unsigned short)d1 << 24))); //U24|U8
        InitPositionMessage[ii].DataLong[1] = 0x00000000;
        InitPositionMessage[ii].DataLong[2] = 0x00000000;
        InitPositionMessage[ii].DataLong[3] = 0x00000000;
    }

    // Set up the initialize torque mode message
    for(int ii=0; ii<6; ii++)
    {
        InitTorqueMessage[ii].Command = SWITCH_CONTROL_MODE_REQUEST;
        InitTorqueMessage[ii].SourceAddress = SOURCE_ADDRESS;
        InitTorqueMessage[ii].DestinationAddress = joint[ii];//DESTINATION_ADDRESS;
        InitTorqueMessage[ii].DataLong[0] = ((unsigned short) 0x01 |
            ((unsigned short) d2 << 8) | ((unsigned short) d3 << 16 |
            ((unsigned short) d1 << 24))); //U24|U8
        InitTorqueMessage[ii].DataLong[1]=0x00000000;
        InitTorqueMessage[ii].DataLong[2]=0x00000000;
        InitTorqueMessage[ii].DataLong[3]=0x00000000;
    }

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

    // Set up the test torque message
    for (int ii=0; ii<6; ii++)
    {
        TestTorquesMessage[ii].Command =
            RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND;
        TestTorquesMessage[ii].SourceAddress = SOURCE_ADDRESS;
        TestTorquesMessage[ii].DestinationAddress = joint[ii];
        //not used
        TestTorquesMessage[ii].DataLong[1] = 0x00000000;
        //32F torque command [Nm]
        TestTorquesMessage[ii].DataFloat[2] = 0;
        TestTorquesMessage[ii].DataLong[3] = ((unsigned long) torqueDamping |
            ((unsigned long) controlMode << 8) | ((unsigned long)
            torqueKp << 16)); //U16|U8|U8
    }

    // Set up the validate torque message
    for (int ii=0; ii<6; ii++)
    {
        ValidateTorquesMessage[ii].Command = GET_TORQUE_VALIDATION_REQUEST;
        ValidateTorquesMessage[ii].SourceAddress = SOURCE_ADDRESS;
        ValidateTorquesMessage[ii].DestinationAddress = joint[ii];  //DESTINATION_ADDRESS;
        ValidateTorquesMessage[ii].DataLong[1] = 0x00000000; //not used
        ValidateTorquesMessage[ii].DataFloat[2] = 0x00000000; //not used
        ValidateTorquesMessage[ii].DataLong[3] = 0x00000000; //not used
    }

}

Jaco2::~Jaco2() { }

int main()
{
    // cout << "T H R E E" << endl;
    // usleep(1000000);
    // cout << "T W O" << endl;
    // usleep(1000000);
    // cout << "O N E" << endl;
    // usleep(1000000);

    Jaco2 j2 = Jaco2();
    float us[6] = {0.0,0.0,0.0,0.0,0.0,0.0};

    j2.Connect();
    // float setTorque[6] = {-0.138, -0.116, 3.339, -0.365, -0.113, 0.061};
    // j2.InitForceMode(setTorque);
    //
    // for (int i = 0; i<2000; i++)
    // {
    //     //u[5] = 4.0 * sin(2.0*3.14159 * i/3500);
    //     j2.ApplyU(us);
    // }
    //
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
        result = fptrInitCommunication();

        //If API's initialization is correct.
        if(result == NO_ERROR_KINOVA)
        {
            cout << "U S B   I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

            /*We activate the RS-485 comm API. From here you cannot control the
              robot with the Joystick or with the normal USB function. Only
              RS-485 command will be accepted. Reboot the robot to get back to
              normal control.*/

            MyRS485_Activate();

            //If we did not receive the answer, continue reading until done
            messageReceived = 0;
            while(messageReceived < 6)
            {
                messageReceived = 0;
                memset(updated, 0, (size_t)sizeof(int)*6);

                MyRS485_Write(InitMessage, packets_sent, WriteCount);
                usleep(delay);
                GetFeedback(SEND_ACTUAL_POSITION);
                for (int ii = 0; ii<6; ii++)
                {
                    if (updated[ii] == 0)
                    {
                        cout << "Connect: Error while obtaining actuator " << ii
                             << " position" <<endl;
                    }
                }
            }

            messageReceived = 0; // reset for next use
        }
        else
        {
            cout << "Error " << result << " while Initializing" << endl;
        }
    }
    else
    {
        cout << "Errors while loading API's function" << endl;
    }
}

void Jaco2::ApplyQ(float q_target[6])
{
    // STEP 0: Get initial position
    messageReceived = 0;
    while(messageReceived < 6)
    {
        // reset to zero, want all to be set in a single read
        messageReceived = 0;
        memset(updated, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(GetPositionMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(SEND_ACTUAL_POSITION);

        for (int ii = 0; ii<6; ii++)
        {
            if (updated[ii] == 0)
            {
                cout << "ApplyQ: Error while obtaining actuator " << ii
                         << " position" <<endl;
            }
        }

    }

    messageReceived = 0;
    // STEP 1: move to rest position
    int TargetReached = 0;
    int ctr = 0;
    float Joint6Command[6];
    for (int ii = 0; ii<6; ii++)
    {
        Joint6Command[ii] = pos[ii];
        cout << "target " << ii << " is: " << q_target[ii]<< endl;
        cout << "Current pos: " << pos[ii] << endl;
        ApplyQMessage[ii].DataFloat[0] = Joint6Command[ii];
        ApplyQMessage[ii].DataFloat[1] = Joint6Command[ii];
    }

    while(TargetReached < 6)
    {
        TargetReached = 0;
        // increment joint command by 1 degree until target reached
        for (int ii = 0; ii<6; ii++)
        {
            // compare target to current angle to see if should add or subtract
            if(q_target[ii] > (fmod(int(pos[ii]),360)))
            {
                Joint6Command[ii] += 0.05;
            }
            else if(q_target[ii] < (fmod(int(pos[ii]),360)))
            {
                Joint6Command[ii] -= 0.05;
            }

            //We assign the new command (increment added)
            ApplyQMessage[ii].DataFloat[0] = Joint6Command[ii];
            ApplyQMessage[ii].DataFloat[1] = Joint6Command[ii];
        }

        messageReceived = 0;
        while(messageReceived < 6)
        {
            messageReceived = 0;
            memset(updated, 0, (size_t)sizeof(int)*6);

            MyRS485_Write(ApplyQMessage, packets_sent, WriteCount);
            usleep(delay);
            GetFeedback(RS485_MSG_SEND_ALL_VALUES_1);
            for (int ii = 0; ii<6; ii++)
            {
                if (updated[ii] == 0)
                {
                    cout << "ApplyQ: Error while obtaining actuator " << ii
                             << " position" <<endl;
                }
            }
        }

        messageReceived = 0;

        for (int jj = 0; jj < 6; jj++)
        {
            if (abs(fmod(int(pos[jj]),360) - q_target[jj]) < 2.0 )
            {
                TargetReached += 1;
                // cout << "Actuator " << jj << " at home, Position is " << pos[jj]
                //      << ", Target is: " << q_target[jj] <<endl;
            }
            else if (ctr == 1000)
            {
                cout << "Actuator: " << jj
                     << " position is: " << pos[jj]
                     << " with target: " << q_target[jj]
                     << " mod 360: " << fmod(int(pos[jj]),360) << endl;
                ctr = 0;
            }

            /*for (int ii = 0; ii < 18; ii++) {
                if (ReceiveInitMessage[ii].SourceAddress == joint[jj] &&
                    ReceiveInitMessage[ii].Command == 0x0015) {
                    pos[jj] = ReceiveInitMessage[ii].DataFloat[1];
                    ii = packets_read;
                    //cout << "Actuator: " << jj
                     //<< " position is: " << pos[jj] << endl;
                }
            }*/
        }
        ctr += 1;
    }
}


void Jaco2::InitForceMode(float setTorque[6])
{
    // ========== BEGIN MAIN COMM ==========
    // STEP 0: Get current position
    messageReceived = 0;
    while(ReadCount != 1 && messageReceived < 6)
    {
        messageReceived = 0;
        memset(updated, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(GetPositionMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(SEND_ACTUAL_POSITION);

        for (int ii = 0; ii<6; ii++)
        {
            if (updated[ii] == 0)
            {
                cout << "InitForceMode: Error while obtaining actuator " << ii
                         << " position" <<endl;
            }
        }
    }
    messageReceived = 0; // reset for next use
    while(ReadCount != 1 && messageReceived < 6)
    {
        messageReceived = 0;
        memset(updated, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(SafetyMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(GET_TORQUE_CONFIG_SAFETY);

        for (int ii = 0; ii<6; ii++)
        {
            if (updated[ii] == 0)
            {
                cout << "InitForceMode: Error Safety Config Failed for Joint "
                     << ii <<endl;
            }
        }
    }

    // STEP 1: SEND TORQUE COMMAND FOR VERIFICATION
    //send position and torque command
    for (int ii=0; ii<6; ii++)
    {
        TestTorquesMessage[ii].DataFloat[0] = pos[ii];
    }

    cout << "STEP 1: Send Torque Command for Verification" << endl;

    MyRS485_Write(TestTorquesMessage, packets_sent, WriteCount);
    usleep(delay);

    // STEP 2: Validate the torque command
    //float setTorque[6] = {-0.138, -0.116, 3.339, -0.365, -0.113, 0.061};
    for (int ii=0; ii<6; ii++)
    {
        ValidateTorquesMessage[ii].DataFloat[0] = setTorque[ii];//0;
    }

    cout << "STEP 2: Request Torque Command Verification" << endl;

    messageReceived = 0; // reset for next use
    // TODO: set a limit on the number of times validation is attempted before exiting
    while (ReadCount != 1 && messageReceived < 6)
    {
        messageReceived = 0;
        memset(updated, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(ValidateTorquesMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(SEND_TORQUE_VALIDATION);
        for (int ii = 0; ii<6; ii++)
        {
            if (updated[ii] == 0)
            {
                cout << "InitForceMode: Error while obtaining torque Validation for joint " << ii <<endl;
            }
        }
    }

    // STEP 3: Switch to torque control mode
    cout << "STEP 3: Waiting for Torque Verification" << endl;

    messageReceived = 0;
    while (messageReceived < 6)
    {
        messageReceived = 0;
        memset(updated, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(InitTorqueMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(SWITCH_CONTROL_MODE_REPLY);
        cout << "exit read function" << endl;
        for (int ii = 0; ii<6; ii++)
        {
            if (updated[ii] == 0)
            {
                cout << "InitForceMode: Error while switching to torque mode for joint " << ii <<endl;
            }
        }
    }

    cout << "Step 4: Switching to Torque Mode" << endl;
}

void Jaco2::ApplyU(float u[6])
{
    // Step 4: Enjoy torque control mode!
    for (int ii=0; ii<6; ii++)
    {
        ForceMessage[ii].DestinationAddress = joint[ii];
        ForceMessage[ii].DataFloat[0] = pos[ii];
        ForceMessage[ii].DataFloat[2] = u[ii]; //32F torque command [1Nm]

    }

    messageReceived = 0;
    MyRS485_Write(ForceMessage, packets_sent, WriteCount);
    usleep(1250);
    //usleep(delay);
    MyRS485_Read(ReceivedInitMessage, packets_read, ReadCount);

    for(int jj = 0; jj < ReadCount; jj++)
    {
        if(ReceivedInitMessage[jj].Command == RS485_MSG_SEND_ALL_VALUES_1)
        {
            //actuator 0 is 16
            currentJoint = ReceivedInitMessage[jj].SourceAddress - 16;
            pos[currentJoint] = ReceivedInitMessage[jj].DataFloat[1];
            vel[currentJoint] = ReceivedInitMessage[jj].DataFloat[2];
            messageReceived +=1;
            if(messageReceived == 6)
            {
                break;
            }
        }
        if(ReceivedInitMessage[jj].Command == REPORT_ERROR)
        {
            // ERROR MESSAGES
            if(ReceivedInitMessage[jj].DataLong[1] != 0)
            {
                cout << "SERVO " << currentJoint << " ERROR NUMBER : "
                     << ReceivedInitMessage[jj].DataLong[1]
                     << " "
                     << errorMessage[ReceivedInitMessage[jj].DataLong[1]]
                     << " ERROR"
                     << endl;
                cout << "RESPONSE: "
                     << ReceivedInitMessage[jj].DataLong[2]
                     << endl;
            }
            else
            {
                cout << "No Error" << endl;
            }
        }
    }
}

void Jaco2::GetPos()
{
    MyRS485_Write(GetPositionMessage, packets_sent, WriteCount);
    usleep(delay);
    MyRS485_Read(ReceivedInitMessage, packets_read, ReadCount);

    for(int jj = 0; jj < ReadCount; jj++)
    {
        //actuator 0 is 16
        currentJoint = ReceivedInitMessage[jj].SourceAddress - 16;

        if(ReceivedInitMessage[jj].Command == SEND_ACTUAL_POSITION)
        {
            pos[currentJoint] = ReceivedInitMessage[jj].DataFloat[1];
        }
    }
}
void Jaco2::GetFeedback(int messageType)
{
    int MessageReadCount = 0;
    MyRS485_Read(MessageListIn, packets_read, MessageReadCount);

    // cycle through all of the received messages and assign the read values to
    // the corresponding joint
    for(int jj = 0; jj < MessageReadCount; jj++)
    {
        if(MessageListIn[jj].Command == REPORT_ERROR)
        {
            // ERROR MESSAGES
            if(MessageListIn[jj].DataLong[1] != 0)
            {
                cout << "ERROR NUMBER : "
                     << MessageListIn[jj].DataLong[1]
                     << " "
                     << errorMessage[MessageListIn[jj].DataLong[1]]
                     << " ERROR"
                     << endl;
                cout << "RESPONSE: "
                     << MessageListIn[jj].DataLong[2]
                     << endl;
            }
        }

        //actuator 0 is 16
        currentJoint = MessageListIn[jj].SourceAddress - 16;
        if (currentJoint > 5)
        {
            break;
        }

        switch(messageType)
        {
            case SEND_ACTUAL_POSITION :
                // GET POSITION HALLS
                if(MessageListIn[jj].Command == SEND_ACTUAL_POSITION)
                {
                    pos[currentJoint] = MessageListIn[jj].DataFloat[1];
                    messageReceived +=1;
                    updated[currentJoint] = 1;
                }
                break;

            case RS485_MSG_SEND_ALL_VALUES_1:
                // ALL VALUES 1
                if(MessageListIn[jj].Command == RS485_MSG_SEND_ALL_VALUES_1)
                {
                    /*cout << "Current  = " << MessageListIn[jj].DataFloat[0] << endl;
                    cout << "Position Halls = " << MessageListIn[jj].DataFloat[1] << endl;
                    cout << "Velocity = " << MessageListIn[jj].DataFloat[2] << endl;
                    cout << "Torque   = " << MessageListIn[jj].DataFloat[3] << endl;*/
                    pos[currentJoint] = MessageListIn[jj].DataFloat[1];
                    vel[currentJoint] = MessageListIn[jj].DataFloat[2];
                    messageReceived +=1;
                    updated[currentJoint] = 1;
                }
                break;

            case RS485_MSG_SEND_ALL_VALUES_2 :
                // ALL VALUES 2
                /*cout << "PWM  = " << MessageListIn[jj].DataFloat[0] << endl;
                cout << "Position Optical = " << MessageListIn[jj].DataFloat[1]
                     << endl;

                short accelX = (short)(MessageListIn[jj].DataLong[2] & 0x0000FFFF);
                short accelY = (short)((MessageListIn[jj].DataLong[2] & 0xFFFF0000)
                            >> 16);
                short accelZ = (short)(MessageListIn[jj].DataLong[3] & 0x0000FFFF);
                short temperature = (short)((MessageListIn[jj].DataLong[3] &
                                0xFFFF0000) >> 16);

                float fAccelX = (float)accelX * 0.001;
                float fAccelY = (float)accelY * 0.001;
                float fAccelZ = (float)accelZ * 0.001;
                float fTemperature = (float)temperature * 0.01;

                cout << "Accel X = " << fAccelX << endl;
                cout << "Accel Y = " << fAccelY << endl;
                cout << "Accel Z = " << fAccelZ << endl;
                cout << "Temperature = " << fTemperature << endl;*/
                break;

            case RS485_MSG_SEND_ALL_VALUES_3 :
                // ALL VALUES 3
                /*cout << "Motor current = " << MessageListIn[j].DataFloat[0]
                     << endl;
                cout << "Absolute position magnetic = " << MessageListIn[jj].DataFloat[1]
                     << endl;*/
                break;

            case SEND_TORQUE_VALIDATION :
                // SAFETY MEASURE CHECKS
                if (MessageListIn[jj].Command == SEND_TORQUE_VALIDATION)
                {
                    if (MessageListIn[jj].DataLong[0] == 1)
                    {
                        messageReceived += 1;
                        updated[currentJoint] = 1;
                      cout << "Torque Validation True for Servo "
                           << currentJoint
                           << " , Response: "
                         << MessageListIn[jj].DataLong[0]
                         << endl;
                    }

                    else if (MessageListIn[jj].DataLong[0] == 0)
                    {
                      cout << "Torque Validation False for Servo "
                           << currentJoint
                           << " , Response: "
                           << MessageListIn[jj].DataLong[0]
                           << endl;
                    }
                    else
                    {
                      cout << "ERROR READING TORQUE VALIDATION REPLY FOR SERVO: "
                           << currentJoint
                           << " , RESPONSE: "
                           << MessageListIn[jj].DataLong[0]
                           << endl;
                    }
                }
                break;

            case SWITCH_CONTROL_MODE_REPLY :
                // SWITCH MODE REPLY
                if (MessageListIn[jj].Command == SWITCH_CONTROL_MODE_REPLY)
                {
                    if (MessageListIn[jj].DataLong[0] == 257)
                    {
                        cout << "Switch Control Mode TORQUE True for Servo "
                             << currentJoint
                             << " , Response: "
                             << MessageListIn[jj].DataLong[0]
                             << endl;
                        messageReceived += 1;
                        updated[currentJoint] = 1;
                    }

                    else if (MessageListIn[jj].DataLong[0] == 1)
                    {
                        cout << "Switch Control Mode POSITION True for Servo "
                             << currentJoint
                             << " , Response: "
                             << MessageListIn[jj].DataLong[0]
                             << endl;
                        messageReceived2 += 1;
                        updated2[currentJoint] = 1;
                    }

                    else if (MessageListIn[jj].DataLong[0] == 0)
                    {
                        cout << "Switch Control Mode False for Servo "
                           << currentJoint
                           << " , Response: "
                           << MessageListIn[jj].DataLong[0]
                           << endl;
                    }

                    else
                    {
                      cout << "ERROR READING SWITCH CONTROL MODE REPLY FOR SERVO "
                           << currentJoint
                           << " , RESPONSE: "
                           << MessageListIn[jj].DataLong[0]
                           << endl;
                    }
                    break;
                }
                else
                {
                    cout << "command is " << ReceivedInitMessage[jj].Command << endl;
                }

            case GET_TORQUE_CONFIG_SAFETY :
                if (MessageListIn[jj].Command == GET_TORQUE_CONFIG_SAFETY)
                {
                    messageReceived +=1;
                    updated[currentJoint] = 1;
                    cout << "Safety passed for servo " << currentJoint  << endl;
                    break;
                }
        }
    }
}

void Jaco2::InitPositionMode()
{
    cout << "STEP 5: Waiting for Position Verification" << endl;

    messageReceived2 = 0;

    while (ReadCount != 1 && messageReceived2 < 6)
    {
        messageReceived2 = 0;
        memset(updated2, 0, (size_t)sizeof(int)*6);

        MyRS485_Write(InitPositionMessage, packets_sent, WriteCount);
        usleep(delay);
        GetFeedback(SWITCH_CONTROL_MODE_REPLY);
        for (int ii = 0; ii<6; ii++)
        {
            if (updated2[ii] == 0)
            {
                cout << "InitPositionMode: Error while switching to position mode" <<endl;
            }
        }
    }
    cout << endl << "Switching to Position Mode & Exiting Main Control Loop" << endl;
}

void Jaco2::Disconnect()
{
    fptrCloseCommunication();
    cout << "Communication closed" << endl;
}
