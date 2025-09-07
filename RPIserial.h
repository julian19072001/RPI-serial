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
    struct pollfd port;
    const char* devicePort; 
    int baudrate;
  }device_t;

  int canReadByte(device_t *com);
  int readByte(device_t *com, uint8_t* byte);
  void sendByte(device_t *com, uint8_t byte);
  void flushBuffer(device_t *com);

  void setupDevice(device_t *com);
  void closeDevice(device_t *com, bool waitSend);

#endif