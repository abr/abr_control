/**
 * \file	KeosComm.h
 *
 * \date	Created on: 2012-04-26
 * \author  Mathieu Noirot
 *
 * \ingroup 	KeosComGroup
 */

#ifndef KEOSCOMM_H_


#include "KeosType.h"


// Main board adress
#define MAIN_BOARD_ADRESS 0x00

#define SIZE_MSG_ARRAY			18 	// Size of the reception and sending buffers (spi in/out for both ports)


#define SIZE_MSG_ARRAY_UART		27 	// Size of the reception and sending buffers (uart in/out for both ports)


struct messageSpi
{
	Uint8 m[SIZE_MSG_ARRAY];
};

struct messageUart
{
	Uint8 m[SIZE_MSG_ARRAY_UART];
};


// Defines for spi array indexes
#define SPI_DEVICE_0 	0
#define SPI_DEVICE_1 	1
#define SPI_NO_DEVICE 	0xFFFF
#define SPI_ERROR		3


#define SPI_PORT_0 0
#define SPI_PORT_1 1
#define SPI_PORT_COUNT 2


#define UART_PORT_0 0
#define UART_PORT_1 1	//Port not in used on MiniJaco
#define UART_PORT_2 2

#define UART_PORT_COUNT 3


// Values used to set "typeCommand" variable
#define CMD_CONFIG_WRITE	0x00 // not used
#define CMD_CONFIG_READ		0x01 // not used
#define CMD_DATA_READ0		0x02 // not used
#define CMD_DATA_READ1		0x03 // not used
#define CMD_DATA_WRITE0		0x04 // the only which is used
#define CMD_DATA_WRITE1		0x05 // not used
#define CMD_CLEAR_INT		0x06 // not used


#define KEOSCOMM_MAX_DATA		4
#define KEOSCOMM_FORMAT_SPI		0
#define KEOSCOMM_FORMAT_UART	1

// Indexes for spi in and out buffers
#define START_BYTE  0	// Start frame value : always equal to 0x55
#define NB_BYTES	1	// Number of bytes in the message			// Old DSP name : NB_OCTETS
#define MSG_ID		2	// MSG_ID and NB_MSGS are used together : They usually are set at 0x01 but ...
#define NB_MSGS		3	// ... If a module wants to send x messages in a row, NB_MSGS will be set at x and MSG_ID will be the message number i of the chain of messages (with i <= x)
#define CHECKSUM	4							// Old DSP name : CHEKSUM
#define COMMAND		5
#define ADDR_SRC	6							// Old DSP name : ADR_SRC
#define ADDR_DEST	7							// Old DSP name : ADR_DEST
#define SPI_LOW_0 	8							// Old DSP appellation : canlow0
#define SPI_LOW_1 	9							// Old DSP appellation : canlow1
#define SPI_LOW_2 	10							// Old DSP appellation : canlow2
#define SPI_LOW_3 	11							// Old DSP appellation : canlow3
#define SPI_HIGH_0 	12							// Old DSP appellation : canhigh0
#define SPI_HIGH_1 	13							// Old DSP appellation : canhigh1
#define SPI_HIGH_2	14							// Old DSP appellation : canhigh2
#define SPI_HIGH_3 	15							// Old DSP appellation : canhigh3
#define STOP_BYTE	16	// stop frame value : always 0xAA
// La case 17 correspond au fait que le tableaux de 18 Uint8 est transformé en 5 Uint32




// Indexes for uart in and out buffers
#define UART_START_BYTE  		0	// Start frame value : always equal to 0x55
#define UART_NB_BYTES			1	// Number of bytes in the message			// Old DSP name : NB_OCTETS
#define UART_MSG_ID				2	// MSG_ID and NB_MSGS are used together : They usually are set at 0x01 but ...
#define UART_NB_MSGS			3	// ... If a module wants to send x messages in a row, NB_MSGS will be set at x and MSG_ID will be the message number i of the chain of messages (with i <= x)
#define UART_CHECKSUM_LOW		4
#define UART_CHECKSUM_HIGH		5
#define UART_COMMAND_LOW		6
#define UART_COMMAND_HIGH		7
#define UART_ADDR_SRC			8
#define UART_ADDR_DEST			9
#define UART_DATA_LOW_0 		10
#define UART_DATA_LOW_1 		11
#define UART_DATA_LOW_2 		12
#define UART_DATA_LOW_3 		13
#define UART_DATA_LOW_4 		14
#define UART_DATA_LOW_5 		15
#define UART_DATA_LOW_6 		16
#define UART_DATA_LOW_7 		17
#define UART_DATA_HIGH_0 		18
#define UART_DATA_HIGH_1 		19
#define UART_DATA_HIGH_2 		20
#define UART_DATA_HIGH_3 		21
#define UART_DATA_HIGH_4 		22
#define UART_DATA_HIGH_5 		23
#define UART_DATA_HIGH_6 		24
#define UART_DATA_HIGH_7 		25
#define UART_STOP_BYTE			26	// stop frame value : always 0xAA

/**
 * 	\class		C_Comm_Data
 * 	\ingroup 	KeosComGroup
 */
class C_Comm_Data
{
	public:

	Uint8  nbBytes; 																	// Old DSP : NbOctets
	Uint16 command; 																	// Old DSP : Commande
	Uint8  typeCommand; 			//!< takes always the same value					// Old DSP : TypeCommande
	Uint8  srcAddress; 																	// Old DSP : AdresseSrc
	Uint8  destAddress; 																// Old DSP : AdresseDest

	longUnion comData[KEOSCOMM_MAX_DATA]; 													// 0= CanLow, 1 = canHigh

	C_Comm_Data &operator =(const C_Comm_Data & data)
	{
		this->nbBytes 	  = data.nbBytes;
		this->command 	  = data.command;
		this->typeCommand = data.typeCommand;
		this->srcAddress  = data.srcAddress;
		this->destAddress = data.destAddress;
		for (int i = 0 ; i< KEOSCOMM_MAX_DATA; i++){
			this->comData[i]      = data.comData[i];
		}

		return *this;
	}
};



/// Function to translate a Uint8[SIZE_MSG_ARRAY] to a C_Comm_Data
void Uint8ToC_Comm_Data(Uint8* data, C_Comm_Data &dataTranslated);

/// Function to translate a Uint8[SIZE_MSG_ARRAY] coming from uart to a C_Comm_Data
void Uint8ToC_Comm_Data_UART(Uint8* data, C_Comm_Data & dataTranslated);

/// Function to obtain the checksum of a message buffer (Kinova protocol specific)
Uint8 ChecksumCalculation_SPI(Uint8 *buffer, Uint16 sizeBuffer);
Uint16 ChecksumCalculation_UART(Uint8 *buffer, Uint16 sizeBuffer);


/// Function treating received messages (SPI and CAN)
void CommRxTreatment(Uint16 portSelect, Uint16 deviceSelect, Uint8* msg, Uint16 sizeMsg);


/// Function incrementing the appropriate "ChecksumErrors"
void SetChecksumErrorsCount(Uint16 portSelect, Uint16  );

// Checksum errors TODO : DELETE ?
//static int ChecksumErrors_CanPrincipal = 0 ;
//static int ChecksumErrors_CanExterne   = 0 ;
//static int ChecksumErrors_SPI0 		 = 0 ;
//static int ChecksumErrors_SPI1 		 = 0 ;



#endif /* KEOSSPI_H_ */
