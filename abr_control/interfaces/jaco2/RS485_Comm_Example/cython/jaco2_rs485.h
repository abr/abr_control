/*
 * This example shows how to send RS-485 command through the USB port.
   Basically, that means communicating directly to the actuators
 * via the USB port. If applied on a 6 axis JACO or MICO, this example will
   rotate the actuator #6, the one holding the end effector.
 * It will also start a thread that read and display information during the
   process.
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
#include "../kinova-api/Kinova.API.CommLayerUbuntu.h"
#include "../kinova-api/Kinova.API.UsbCommandLayerUbuntu.h"
#include "../kinova-api/KinovaTypes.h"
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

struct InitStruct {
    //Variable used during the communication process.
	int WriteCount;
	int ReadCount;
	//Flag used during initialization.
	bool ActuatorInitialized;
	//Variable needed during worker thread creation but not used after.
	int ThreadArgument;
	int delay;
};

InitStruct default_init = {0, 0, false, 0, 2000};

struct ConnectStruct {
	//We load the API.
	void * commLayer_Handle;
    bool connection_init;
};

ConnectStruct default_connect = {
    dlopen("./../kinova-api/Kinova.API.CommLayerUbuntu.so", RTLD_NOW|RTLD_GLOBAL),
    false};

struct InitForceModeStruct {
    int servoNumber;
};

struct DisconnectStruct {

} d;

class jaco2 {
    void Init(InitStruct* args);
    void* Connect(ConnectStruct* args, RS485_Message* RS485initArgs,
                  RS485_Message* rcvInitArgs, InitStruct* initArgs);
    void InitForceMode(InitForceModeStruct* args);
    void ApplyU(void* args);
    void GetFeedback(void* args);
    void Disconnect(DisconnectStruct* args);
};

struct ApplyUStruct {
    float us[6]; //joint torques
    unsigned char torqueDamping = 0x01;
    unsigned char controlMode = 0x01;
    unsigned short torqueKp = 1750; //1.75 * 1000
    unsigned long torqueConst = ((unsigned long) torqueDamping |
        ((unsigned long) controlMode << 8) | ((unsigned long)
        torqueKp <<   16));
} au;

struct GetFeedbackStruct {
    int flag;
    bool torqueValidation;
    bool switchValidation;
    int pos;
    bool read_input;
    float q[6];
    float dq[6];
} fb;

//Function pointers to access API's function.
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MyGetAngularForce)(AngularPosition &Response);
int(*MyGetAngularForceGravityFree)(AngularPosition &Response);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);

int(*fptrInitCommunication)();
int(*MyRS485_Activate)();     // FUNCTION TO ACTIVATE USB - RS485 MODE //
int(*MyRS485_Read)(RS485_Message* PackagesIn, int QuantityWanted,
                   int &ReceivedQtyIn);
int(*MyRS485_Write)(RS485_Message* PackagesOut, int QuantityWanted,
                    int &ReceivedQtyIn);
