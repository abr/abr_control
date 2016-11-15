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
#define DESTINATION_ADDRESS 0x15
#define GET_TORQUE_VALIDATION_REQUEST 0x0201
#define SEND_TORQUE_VALIDATION 0x0202
#define SWITCH_CONTROL_MODE_REQUEST 0x0203
#define SWITCH_CONTROL_MODE_REPLY 0x0204
#define REPORT_ERROR 0x0030
#define CLEAR_ERROR_FLAG 0x0033
#define POSITION_LIMIT 0x0021

class Jaco2 {
    //int Init(InitStruct* args);
    public
        void* Connect();
        void* InitForceMode();
        void* ApplyU(float us[6]);
        void* GetFeedback(float q[6], float dq[6]);
        void* Disconnect();
        
        //read variables
        int flag;
        bool torqueValidation;
        bool switchValidation;
        int pos;
        bool read_input;
        
        //torque variables
        unsigned char torqueDamping;
        unsigned char controlMode;
        unsigned short torqueKp;
        
    	//Variable used during the communication process.
    	int WriteCount;
    	int ReadCount;

        Jaco2(); //constructor
};

Jaco2::Jaco2(void){
};

//set common variables
int Jaco2::flag = 0;
bool Jaco2::torqueValidation = false;
bool Jaco2::switchValidation = false;
bool Jaco2::read_input = true;

unsigned char Jaco2::torqueDamping = 0x01;
unsigned char Jaco2::controlMode = 0x01;
unsigned short torqueKp = 1750; // torque kp 1.75 * 1000

int Jaco2::WriteCount = 0;
int Jaco2::ReadCount = 0;

//A handle needed to open the API(library).
void *commLayer_Handle;

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
