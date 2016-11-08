#include <stdio.h>
#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */
int main()
{
  int fd;
  fd = open("/dev/bus/usb/001/012", O_RDWR | O_NOCTTY);
  if(fd == 1)
  {
     printf("\n  Error! in Opening ttyUSB0\n"); 
  }
  else 
  {
     printf("\n  ttyUSB0 Opened Successfully\n");
  }
  close(fd);
  
  return 1;
}
