#ifndef _SERIALRPI_H_
#define _SERIALRPI_H_

  #include <stdio.h>
  #include <stdint.h>
  #include <stdbool.h>
  #include <string.h>
  #include <stdlib.h>

  #include <fcntl.h> 
  #include <errno.h> 
  #include <termios.h>
  #include <unistd.h>
  #include <errno.h>
  #include <dirent.h>
  #include <sys/poll.h>

  #define BUFF_SIZE 512

  typedef struct {
    char buff[BUFF_SIZE];
    struct pollfd port;
  }
  device;

  int canReadByte(device *com);
  int readByte(device *com, uint8_t* byte);
  void sendByte(device *com, uint8_t byte);
  void flushBuffer(device *com);

  int readLine(device *com);
  ssize_t sendLine(device *com, char* data);

  void setupDevice(device *com, const char* devicePort, int baudrate);
  void closeDevice(device *com, bool waitSend);

#endif