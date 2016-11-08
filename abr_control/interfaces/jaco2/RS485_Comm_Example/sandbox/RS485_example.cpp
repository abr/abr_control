#include <iostream>
#include "KeosComm.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>

void UartSendDataExample(Uint32 destAddr, Uint32 command, Uint32 data3, Uint32 data2, Uint32 data1, Uint32 data0);

int main()
{
    //for (int i=0; i<100; i++){
     Uint32 destAddr; // Actuator 1
     Uint32 command; // Position Command
     longUnion dataToSend[4];

     destAddr = 0x10; // Actuator 1
     command = 0x00;  
                // x10; // Position Command

     dataToSend[0].FloatData = destAddr;
     dataToSend[1].FloatData = 45.5; // move to 45.5 degrees? 
     dataToSend[2].FloatData = 0;
     dataToSend[3].FloatData = 0x00;

     UartSendDataExample(
        destAddr,
        command,
        dataToSend[3].uLongData,
        dataToSend[2].uLongData,
        dataToSend[1].uLongData,
        dataToSend[0].uLongData);
     
     return 1;
}

void UartSendDataExample(Uint32 destAddr, Uint32 command, Uint32 data3, Uint32 data2, Uint32 data1, Uint32 data0)
{

    int fd;
    fd = open("/dev/bus/usb/001/012", O_RDWR | O_NOCTTY);
    if (fd==1) {
         printf("\n Error connecting to device\n");
    }
    else {
         printf("\n Connected to device\n");
    }

    Uint8 messageToSend[SIZE_MSG_ARRAY_UART];
    Uint8 messageToSendLength = 0;

    longUnion sendData[4];

    sendData[0].uLongData = data0;
    sendData[1].uLongData = data1;
    sendData[2].uLongData = data2;
    sendData[3].uLongData = data3;

    messageToSendLength = SIZE_MSG_ARRAY_UART;

    messageToSend[UART_START_BYTE] = 0x55;
    messageToSend[UART_NB_BYTES] = 16;
    messageToSend[UART_MSG_ID] = 1 ;
    messageToSend[UART_NB_MSGS] = 1;
    messageToSend[UART_COMMAND_LOW] = (Uint8)(command & 0x000000FF);
    messageToSend[UART_COMMAND_HIGH] = (Uint8)((command >> 8) & 0x000000FF);

    messageToSend[UART_ADDR_SRC] = MAIN_BOARD_ADRESS; //0x00
    messageToSend[UART_ADDR_DEST] = (Uint8)destAddr;

    messageToSend[UART_DATA_LOW_0]  = sendData[0].uByteData[0];
    messageToSend[UART_DATA_LOW_1]  = sendData[0].uByteData[1];
    messageToSend[UART_DATA_LOW_2]  = sendData[0].uByteData[2];
    messageToSend[UART_DATA_LOW_3]  = sendData[0].uByteData[3];

    messageToSend[UART_DATA_LOW_4]  = sendData[1].uByteData[0];
    messageToSend[UART_DATA_LOW_5]  = sendData[1].uByteData[1];
    messageToSend[UART_DATA_LOW_6]  = sendData[1].uByteData[2];
    messageToSend[UART_DATA_LOW_7]  = sendData[1].uByteData[3];

    messageToSend[UART_DATA_HIGH_0]  = sendData[2].uByteData[0];
    messageToSend[UART_DATA_HIGH_1]  = sendData[2].uByteData[1];
    messageToSend[UART_DATA_HIGH_2]  = sendData[2].uByteData[2];
    messageToSend[UART_DATA_HIGH_3]  = sendData[2].uByteData[3];

    messageToSend[UART_DATA_HIGH_4]  = sendData[3].uByteData[0];
    messageToSend[UART_DATA_HIGH_5]  = sendData[3].uByteData[1];
    messageToSend[UART_DATA_HIGH_6]  = sendData[3].uByteData[2];
    messageToSend[UART_DATA_HIGH_7]  = sendData[3].uByteData[3];

    messageToSend[UART_STOP_BYTE]  = 0xAA;

    longUnion calculatedChecksum;
    calculatedChecksum.ShortData[0] = ChecksumCalculation_UART(messageToSend,SIZE_MSG_ARRAY_UART);

    // Checksum calculation
    messageToSend[UART_CHECKSUM_LOW]  = calculatedChecksum.ByteData[0];
    messageToSend[UART_CHECKSUM_HIGH] = calculatedChecksum.ByteData[1];

    //WriteSendingBuffer(messageToSend , messageToSendLength);

    printf("\n Sending message \n");
    // write the message out to serial
    int n_written = 0, spot = 0;
    /**while (messageToSend[spot] != 0xAA) {
        n_written = write(fd, &messageToSend[spot], 1 );
        spot += n_written; 
    }**/
    for (int ii = 0; ii < messageToSendLength; ii++) {
        n_written = write(fd, &messageToSend[ii], 1);
    }  

    // now read the returning message
    //char buffer[2];
    long buffer[2];

    printf("\n Reading message \n");
 
    int n = read(fd, buffer, sizeof buffer);
    if (n > 0) {
        printf("\n size: %d \n", n);

        for (int ii = 0; ii < 2; ii++) {
            printf("\n buffer[%d]: %ld\n", ii, buffer[ii]);
        }
    }

    /** Uint8 bufLen = 7;
    Uint8 inBuf[bufLen];

    while (Serial.available()) {
        hdr1 = read(fd, inBuf, bufLen);
        // Check header. If 0xFFFF, it's okay!
        if (hdr1 == 0x55) {
            read(fd, inBuf, bufLen);

            // Read in joint angle targets
            temp_in = float(((inBuf[1] << 8) + inBuf[2])) / 100.0 - 50.0;
            temp_F1 = float(((inBuf[3] << 8) + inBuf[4])) / 100.0 - 50.0;
            temp_F2 = float(((inBuf[5] << 8) + inBuf[6])) / 100.0 - 50.0;
    } **/ 

    close(fd);
}
