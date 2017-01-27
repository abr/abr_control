/*
 * Kinova.DLL.CommLayerUbuntu.h
 *
 *  Created on: Oct 16, 2012
 *      Author: H. Lamontagne, Kinova
 */

#ifndef KINOVA_DLL_COMMLAYERUBUNTU_H_
#define KINOVA_DLL_COMMLAYERUBUNTU_H_
/*
#ifdef KINOVADLLCOMMLAYER_EXPORTS
#define KINOVADLLCOMMLAYER_API __declspec(dllexport)
#else
#define KINOVADLLCOMMLAYER_API __declspec(dllimport)
#endif
*/
#include <vector>

// ***** E R R O R   C O D E S ******

//No error, everything is fine.
#define NO_ERROR_KINOVA 1

//We know that an error has occured but we don't know where it comes from.
#define UNKNOWN_ERROR 666

//Unable to load the USB library.
#define ERROR_LOAD_USB_LIBRARY 1001

//Unable to access the Open method from the USB library.
#define ERROR_OPEN_METHOD  1002

//Unable to access the Write method from the USB library.
#define ERROR_WRITE_METHOD  1003

//Unable to access the Read method from the USB library.
#define ERROR_READ_METHOD  1004

//Unable to access the Read Int method from the USB library.
#define ERROR_READ_INT_METHOD  1005

//Unable to access the Free Library method from the USB library.
#define ERROR_FREE_LIBRARY  1006

//There is a problem with the USB connection between the device and the computer.
#define ERROR_JACO_CONNECTION 1007

//Unable to claim the USB interface.
#define ERROR_CLAIM_INTERFACE 1008

//Unknown type of device.
#define ERROR_UNKNOWN_DEVICE 1009

//The functionality you are trying to use has not been initialized.
#define ERROR_NOT_INITIALIZED 1010

//The USB library cannot find the device.
#define ERROR_LIBUSB_NO_DEVICE 1011

//The USB Library is bussy and could not perform the action.
#define ERROR_LIBUSB_BUSY 1012

//The functionality you are trying to perform is not supported by the version installed.
#define ERROR_LIBUSB_NOT_SUPPORTED 1013

//Unknown error while sending a packet.
#define ERROR_SENDPACKET_UNKNOWN 1014

//Cannot find the requested device.
#define ERROR_NO_DEVICE_FOUND 1015

//The operation was not entirely completed :)
#define ERROR_OPERATION_INCOMPLETED 1016

//Handle used is not valid.
#define ERROR_RS485_INVALID_HANDLE 1017

//An overlapped I/O operation is in progress but has not completed.
#define ERROR_RS485_IO_PENDING 1018

//Not enough memory to complete the opreation.
#define ERROR_RS485_NOT_ENOUGH_MEMORY 1019

//The operation has timed out.
#define ERROR_RS485_TIMEOUT 1020

//You are trying to call a USB function that is not available in the current context.
#define ERROR_FUNCTION_NOT_ACCESSIBLE 1021 
// ***** E N D  O F  E R R O R   C O D E S ******


// ***** R S - 4 8 5   C O M M A N D   I D   L I S T ******
/*
 * All RS-485 commands refer to the RS-485 communication protocol documentation.
 */

/** This message report an error. */
#define RS485_MSG_REPORT_ERROR                            0x30

/** This message clear the error flag. */
#define RS485_MSG_CLEAR_FAULT_FLAG                        0x33

/** This message represents a NACK. */
#define RS485_MSG_NACK                                    0x3E

/** This message represents a ACK. */
#define RS485_MSG_ACK                                     0x3F

/** This message type is sent to initialise de device. */
#define RS485_MSG_SET_ADDRESS                             0x00

/** This message type is used to read position without sending a new command. */
#define RS485_MSG_GET_ACTUALPOSITION                      0x01

/** This message type sends back the position with Current, Speed and Torque. */
#define RS485_MSG_SEND_ACTUALPOSITION                     0x02

/** Start control of the motor. */
#define RS485_MSG_STAR_ASSERV                             0x03

/** Stop the control of the motor. */
#define RS485_MSG_STOP_ASSERV                             0x04

 /** This message type sends the Feed through value to bypass the PID of the
actuator. Value range is -1.0 to +1.0, and both DataLow and DataHigh must
be same value for command to be accepted. When using this command, the
gains Kp, Ki and Kd must be set to zero first. */
#define RS485_MSG_FEEDTHROUGH                             0x09

/** This message type sends the position command to the slave device. */
#define RS485_MSG_GET_POSITION_COMMAND                    0x10

/** This message contains the slave device actual position and current consumption. */
#define RS485_MSG_SEND_POSITION_CURRENT                   0x11

/** Send a new position command like the message 0x10 but it returns more insformation about the robot. */
#define RS485_MSG_GET_POSITION_COMMAND_ALL_VALUES         0x14

/** This message is an answer to the message 0x14. */
#define RS485_MSG_SEND_ALL_VALUES_1                       0x15

/** This message is an answer to the message 0x14. */
#define RS485_MSG_SEND_ALL_VALUES_2                       0x16

/** This message is an answer to the message 0x14. */
#define RS485_MSG_SEND_ALL_VALUES_3                       0x17

/** This message contains the maximum and minimum position of the joint in degrees. */
#define RS485_MSG_POSITION_MAX_MIN                        0x21

/** This message contains the Kp gain of the system. */
#define RS485_MSG_KP_GAIN                                 0x24

/** This message contains the Ki dans Kd gain of the system. */
#define RS485_MSG_KI_KD_GAIN                              0x25

/** This message tells the drive that its actual position is the position zero. */
#define RS485_MSG_PROGRAM_JOINT_ZERO                      0x26

/** This message requests the code version of the slave device. */
#define RS485_MSG_GET_CODE_VERSION                        0x27

/** This message contains the code version of the slave device. */
#define RS485_MSG_SEND_CODE_VERSION                       0x28

/** This message requests the slave device’s specific information. */
#define RS485_MSG_GET_DEVICE_INFO                         0x29

/** This message contains device’s specific information. */
#define RS485_MSG_SEND_DEVICE_INFO                        0x2A

/** This message get the temperature of the actuator. */
#define RS485_MSG_GET_TEMPERATURE                         0x2E

/** This message set the temperature of the actuator. */
#define RS485_MSG_SET_TEMPERATURE                         0x2F

/** This message set all the filters applied to the PID controller. */
#define RS485_SET_PID_FILTERS                             0x40

/** This message set the current position as the reference(180 degrees). WARNING: do not use if you are not familiar with the procedure. */
#define RS485_SET_ZERO_TORQUESENSOR                       0x41

/** This message set the gain applied on the torque sensor value. WARNING: do not use if you are not familiar with the procedure. */
#define RS485_SET_GAIN_TORQUESENSOR                       0x42

/** Activate or Deactivate the control with encoder. */
#define RS485_SET_CONTROL_WITH_ENCODER                    0x43

/** This message returns the encoder's statuses. */
#define RS485_GET_ENCODER_STATUSSES                       0x44

/** This message set the advanced PID parameters. */
#define RS485_SET_PID_ADVANCED_PARAMETERS                 0x45

// ***** E N D   O F  R S - 4 8 5   C O M M A N D   I D   L I S T ******




//Time out in ms.
#define COMMUNICATION_TIME_OUT 5000

//Total size of our packet in bytes.
#define PACKET_SIZE 64

//Data's size of a single packet.
#define PACKET_DATA_SIZE 56

//Header's size of a packet.
#define PACKET_HEADER_SIZE 8

//Version of this library.
#define COMM_LAYER_VERSION 10002

//Max character count in our string.
#define SERIAL_LENGTH 20

//The maximum devices count that the API can control.
#define MAX_KINOVA_DEVICE 20

//Size of a RS485 message in bytes.
#define RS485_MESSAGE_SIZE 20

//Max Qty of RS-485 message hold by a USB packet.
#define RS485_MESSAGE_MAX_COUNT 3

//That represents a packet. As a developper you don't have to use this structure
struct Packet
{
	short IdPacket;
	short TotalPacketCount;
	short IdCommand;
	short TotalDataSize;
	unsigned char Data[PACKET_DATA_SIZE];
};

//That is simply a list of packet
struct PacketList
{
	std::vector<Packet> packets;
};

//That is a device you can communicate with via this library.
struct KinovaDevice
{
	//The serial number of the device. If you are communicating with more than 1 device, this will be used to identify
	//the devices.
	char SerialNumber[SERIAL_LENGTH];

	//The model of the device.
	char Model[SERIAL_LENGTH];

	//Those variables represents the code version - Major.Minor.Release
	int VersionMajor;
	int VersionMinor;
	int VersionRelease;

	//The type of the device.
	int DeviceType;

	//This is a device ID used by the API. User should not use it.
	int DeviceID;
};

//This structure represents a RS-485 message
struct RS485_Message
{
	//Command ID of the message. Use #define from the COMMAND ID LIST above.
	short Command;

	//Source of the message. If this is an actuator, it will be an address.
	unsigned char SourceAddress;
	
	//Destination of the message. Use the address of the actuator.
	unsigned char DestinationAddress;
	
	//Data of the message displayed as unsigned char, float or unsigned long.
	union
	{
		unsigned char DataByte[16];
		float DataFloat[4];
		unsigned int DataLong[4];
	};
};



//N O R M A L   U S B   F U N C T I O N S
extern "C" __attribute__ ((visibility ("default"))) int InitCommunication(void);

extern "C" __attribute__ ((visibility ("default"))) int CloseCommunication(void);

extern "C" __attribute__ ((visibility ("default"))) int GetDeviceCount(int &result);

extern "C" __attribute__ ((visibility ("default"))) Packet SendPacket(Packet &packetOut, Packet &packetIn, int &result);

extern "C" __attribute__ ((visibility ("default"))) int ScanForNewDevice();

extern "C" __attribute__ ((visibility ("default"))) int GetDevices(KinovaDevice list[MAX_KINOVA_DEVICE], int &result);

extern "C" __attribute__ ((visibility ("default"))) int SetActiveDevice(KinovaDevice device);

extern "C" __attribute__ ((visibility ("default"))) int GetActiveDevice(KinovaDevice &device);




// R S - 4 8 5   F U N C T I O N S 
/*
This section hold the function to send command via directly to the robot's actuators on the RS-485 internal bus.
The data will be sent via the USB port and then transfered on the RS-485 bus. In order to use the OpenRS485_Read function
and the OpenRS485_Write function, you need to call the OpenRS485_Activate function. Once the OpenRS485_Activate is called
you cannot used the joystick anymore and the normal USB API(functions above) will not be accessible.
*/
extern "C" __attribute__ ((visibility ("default"))) int RS485_Read(RS485_Message* PackagesIn, int QuantityWanted, int &ReceivedQtyIn);

extern "C" __attribute__ ((visibility ("default"))) int RS485_Write(RS485_Message* PackagesOut, int QtyToSend, int &QtySent);

extern "C" __attribute__ ((visibility ("default"))) int RS485_Activate(void);

#endif
