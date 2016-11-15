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
#include "kinova-api/Kinova.API.UsbCommandLayerUbuntu.h"
#include "kinova-api/KinovaTypes.h"
#include <string.h>

using namespace std;

#define LOOPCOUNT 500
#define SOURCE_ADDRESS 0x00
#define RS485_MSG_SEND_POSITION_AND_TORQUE_COMMAND 0x0014
//#define DESTINATION_ADDRESS 0x15
#define GET_TORQUE_VALIDATION_REQUEST 0x0201
#define SEND_TORQUE_VALIDATION 0x0202
#define SWITCH_CONTROL_MODE_REQUEST 0x0203
#define SWITCH_CONTROL_MODE_REPLY 0x0204
#define REPORT_ERROR 0x0030
#define CLEAR_ERROR_FLAG 0x0033
#define POSITION_LIMIT 0x0021

class Jaco2 {
    public:
        // misc variables
        int delay;
        // main functions
        void Connect(unsigned char DESTINATION_ADDRESS);
        void InitForceMode(unsigned char DESTINATION_ADDRESS);
        void* ApplyU(unsigned char DESTINATION_ADDRESS, float us[6]);
        void* GetFeedback(RS485_Message* MessageListIn);//, unsigned char DESTINATION_ADDRESS, float q[6], float dq[6]);
        void Disconnect(unsigned char DESTINATION_ADDRESS);

        // read variables
        int flag;
        bool torqueValidation;
        bool switchValidation;
        int pos[2];
        bool read_input;
        int qtyWanted;

        // torque variables
        unsigned char torqueDamping;
        unsigned char controlMode;
        unsigned short torqueKp;

    	// variables used during the communication process.
    	int WriteCount;
    	int ReadCount;
    	unsigned char joint[6];

        // RS485 Structs
        RS485_Message InitMessage[2];
        RS485_Message SafetyMessage[2];
        RS485_Message ReceiveInitMessage[6];
        RS485_Message TrajectoryMessage[2];
        RS485_Message ForceMessage[2];

        // A handle needed to open the API(library).
        void *commLayer_Handle;

        // function pointers
        int (*fptrInitCommunication)();
        int (*MyRS485_Activate)();
        int (*MyRS485_Read)(RS485_Message*, int, int&);
        int (*MyRS485_Write)(RS485_Message*, int, int&);

        Jaco2(); //constructor
};

// //Function pointers to access API's function.
// int(*MyInitAPI)();
// int(*MyCloseAPI)();
// int(*MyGetAngularForce)(AngularPosition &Response);
// int(*MyGetAngularForceGravityFree)(AngularPosition &Response);
// int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
// int(*MySetActiveDevice)(KinovaDevice device);
//
// int(*fptrInitCommunication)();
// int(*MyRS485_Activate)();     // FUNCTION TO ACTIVATE USB - RS485 MODE //
// int(*MyRS485_Read)(RS485_Message* PackagesIn, int QuantityWanted,
//                    int &ReceivedQtyIn);
// int(*MyRS485_Write)(RS485_Message* PackagesOut, int QuantityWanted,
//                     int &ReceivedQtyIn);
