/**
 * 	\file 		KeosComm.cpp
 *
 * 	\date		Created on: 2012-04-26
 * 	\author  	Mathieu Noirot
 * 	\ingroup 	KeosComGroup
 */

#include "KeosComm.h"

/**
 * \brief		This function translates a "Uint8[SIZE_MSG_ARRAY]" to a "C_SPI_Data"
 * \details		Where "Uint8[SIZE_MSG_ARRAY]" is the type to manipulate messages inside KEOS
 * 				while "C_SPI_Data" is an easier but protocol dependent way to treat them
 *
 *	\param 		data is the table of data
 *
 *	\return 	the message translated into C_Comm_Data
 *
 * 	\ingroup 	KeosComGroup
 */
void Uint8ToC_Comm_Data(Uint8* data, C_Comm_Data & dataTranslated)
{
	dataTranslated.nbBytes		         = data[NB_BYTES];
	dataTranslated.command		         = data[COMMAND];
	dataTranslated.srcAddress	         = data[ADDR_SRC] & 0x3F;		// todo : attention : source d'erreur ...
	dataTranslated.destAddress			 = data[ADDR_DEST];

	dataTranslated.comData[0].uByteData[0]	 = data[SPI_LOW_0];
	dataTranslated.comData[0].uByteData[1]	 = data[SPI_LOW_1];
	dataTranslated.comData[0].uByteData[2]	 = data[SPI_LOW_2];
	dataTranslated.comData[0].uByteData[3]	 = data[SPI_LOW_3];

	dataTranslated.comData[1].uByteData[0] = data[SPI_HIGH_0];
	dataTranslated.comData[1].uByteData[1] = data[SPI_HIGH_1];
	dataTranslated.comData[1].uByteData[2] = data[SPI_HIGH_2];
	dataTranslated.comData[1].uByteData[3] = data[SPI_HIGH_3];

	dataTranslated.comData[2].uLongData = 0;
	dataTranslated.comData[3].uLongData = 0;


}

/**
 * 	\brief		This function translates a "Uint8[SIZE_MSG_ARRAY]" to a "C_Comm_Data"
 * 	\details	Where "Uint8[SIZE_MSG_ARRAY]" is the type to manipulate messages inside KEOS
 * 				while "C_Comm_Data" is an easier but protocol dependent way to treat them
 *
 *	\param 		data is the table of data
 *
 *	\return 	the message translated into C_Comm_Data
 *
 * 	\ingroup 	KeosComGroup
 */
void Uint8ToC_Comm_Data_UART(Uint8* data, C_Comm_Data & dataTranslated)
{
	dataTranslated.nbBytes		         = data[UART_NB_BYTES];
	dataTranslated.command		         = ((Uint16)data[UART_COMMAND_HIGH] << 8) |  (Uint16)data[UART_COMMAND_LOW] ;
	dataTranslated.srcAddress	         = data[UART_ADDR_SRC] & 0x3F;		// todo : attention : source d'erreur ...
	dataTranslated.destAddress			 = data[UART_ADDR_DEST];

	dataTranslated.comData[0].uByteData[0]	 = data[UART_DATA_LOW_0];
	dataTranslated.comData[0].uByteData[1]	 = data[UART_DATA_LOW_1];
	dataTranslated.comData[0].uByteData[2]	 = data[UART_DATA_LOW_2];
	dataTranslated.comData[0].uByteData[3]	 = data[UART_DATA_LOW_3];

	dataTranslated.comData[1].uByteData[0] = data[UART_DATA_LOW_4];
	dataTranslated.comData[1].uByteData[1] = data[UART_DATA_LOW_5];
	dataTranslated.comData[1].uByteData[2] = data[UART_DATA_LOW_6];
	dataTranslated.comData[1].uByteData[3] = data[UART_DATA_LOW_7];

	dataTranslated.comData[2].uByteData[0]	 = data[UART_DATA_HIGH_0];
	dataTranslated.comData[2].uByteData[1]	 = data[UART_DATA_HIGH_1];
	dataTranslated.comData[2].uByteData[2]	 = data[UART_DATA_HIGH_2];
	dataTranslated.comData[2].uByteData[3]	 = data[UART_DATA_HIGH_3];

	dataTranslated.comData[3].uByteData[0] = data[UART_DATA_HIGH_4];
	dataTranslated.comData[3].uByteData[1] = data[UART_DATA_HIGH_5];
	dataTranslated.comData[3].uByteData[2] = data[UART_DATA_HIGH_6];
	dataTranslated.comData[3].uByteData[3] = data[UART_DATA_HIGH_7];

}



/**
 * \brief		This functions compute the spi Kinova protocol checksum of the buffer
 *
 *	\param 		buffer[] is the buffer we want the checksum of
 *	\param 		sizeBuffer is the size of the buffer, which has to be SIZE_MSG_ARRAY
 *
 *	\return		- the spi checksum of the buffer is correct sizeBuffer
 * 				- 0x00 error code(true if good arguments passed
 *
 * \note		The result is the Uint8 truncated checksum : the result of the computation could be mode than 0xFF
 * \ingroup 	KeosComGroup
 */
Uint8 ChecksumCalculation_SPI(Uint8 *buffer, Uint16 sizeBuffer)
{
	Uint8 checksum=0;

	if (sizeBuffer!=SIZE_MSG_ARRAY) return 0x00;

	checksum = buffer[SPI_LOW_0]   + buffer[SPI_LOW_1]   + buffer[SPI_LOW_2]  + buffer[SPI_LOW_3];		// Low byte
	checksum += buffer[SPI_HIGH_0] + buffer[SPI_HIGH_1]  + buffer[SPI_HIGH_2] + buffer[SPI_HIGH_3];		// High byte
	checksum += buffer[COMMAND]    + (buffer[ADDR_DEST]&0x3F) + (buffer[ADDR_SRC]&0x3F);

	return checksum;

}


/**
 * \brief		This functions compute the uart Kinova protocol checksum of the buffer
 *
 *	\param 		buffer[] is the buffer we want the checksum of
 *	\param 		sizeBuffer is the size of the buffer, which has to be SIZE_MSG_ARRAY
 *
 *	\return		- the spi checksum of the buffer is correct sizeBuffer
 * 				- 0x00 error code(true if good arguments passed
 *
 * \note		The result is the Uint8 truncated checksum : the result of the computation could be mode than 0xFF
 */
Uint16 ChecksumCalculation_UART(Uint8 *buffer, Uint16 sizeBuffer)
{
	Uint16 checksum=0;

	for(int i=UART_COMMAND_LOW; i<=UART_DATA_HIGH_7; i++)
	{
		//Checksum calculation
		checksum += (Uint16)buffer[i];
	}

	return checksum;

}


/**
 * \brief	Function incrementing the appropriate "ChecksumErrors"
 *
 * \param 	portSelect is the port selection
 * \param	deviceSelect is the device selection on port
 *
 * \warning	This function has no effect for now
 */
void SetChecksumErrorsCount(Uint16 portSelect, Uint16 deviceSelect)
{
//	if		(portSelect == 0 && deviceSelect == 0 )	ChecksumErrors_CanPrincipal ++ ;
//	else if	(portSelect == 0 && deviceSelect == 1 )	ChecksumErrors_CanExterne ++ ;
//	else if	(portSelect == 1 && deviceSelect == 0 )	ChecksumErrors_SPI0 ++ ;
//	else if	(portSelect == 1 && deviceSelect == 1 )	ChecksumErrors_SPI1 ++ ;
}

