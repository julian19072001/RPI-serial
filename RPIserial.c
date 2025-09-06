/*!
 *  \file    serialRPI.c
 *  \author  Julian Della Guardia
 *  \date    06-09-2025
 *  \version 1.2
 *
 *  \brief   Library to use serial communication on the raspberry pi
 */

#include "serialRPI.h"

/*! \brief checks if there is a byte to be read. 
 *  
 *  \param portName Port which the device is connected to (tty location)
 * 
 *  \param baudrate communication speed of the device
 * 
 *  \return returns the port number
 */
int openSerial(const char *portName, int baud){
  
  int serial_port = open(portName, O_RDWR | O_NONBLOCK);

  struct termios tty;

  if (serial_port < 0) {
      printf("Error %i from open: %s\n", errno, strerror(errno));
      exit(1);
  }
  
  // Create new termios struct, we call it 'tty' for convention
  // No need for "= {0}" at the end as we'll immediately write the existing
  // config to this struct

  // Read in existing settings, and handle any error
  // NOTE: This is important! POSIX states that the struct passed to tcsetattr()
  // must have been initialized with a call to tcgetattr() overwise behaviour
  // is undefined
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      exit(2);
  }
  
  
  
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 1;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
      exit(3);
  }
  
  return serial_port;
} /*openSerial*/

/*! \brief checks if there is a byte to be read. 
 *  
 *  \param com location that needs to be read from
 * 
 *  \return returns 0 if there is no byte, 1 if there is a byte
 */
int canReadByte(device *com){
    return poll(&com->port, 1, 0);
} /*canReadByte*/

/*! \brief Read byte form incomming buffer and check if it indicates a end of line
 *  
 *  \param com location that needs to be read from
 * 
 *  \param byte location where the byte needs to save
 * 
 *  \return returns -1 if there is an error, 0 if EOF is reached, 1 if byte is read succesfully
 * 
 *  \details Waits for byte to be availible, call canReadByte() to prevent a busy wait
 */
int readByte(device *com, uint8_t* byte){
    while(!canReadByte(com));
    return read(com->port.fd, byte, 1);
}

/*! \brief Send single byte to outgoing buffer
 *  
 *  \param com location that needs to be send to
 * 
 *  \param byte byte that needs to be send
 */
void sendByte(device *com, uint8_t byte){
    write(com->port.fd, byte, 1);
}

/*! \brief Completely clear incomming buffer from data
 *  
 *  \param com location that needs to be read from
 * 
 */
void flushBuffer(device *com){
    uint8_t byte;
    while(canReadByte(com))
        read(com->port.fd, &byte, 1);
} /*flushBuffer*/

/*! \brief Read byte form incomming buffer and check if it indicates a end of line
 *  
 *  \param com location that needs to be read from
 * 
 *  \return returns 0 if it isnt a end of line, returns 1 if the end of line has been reached
 * 
 */
int readData(device *com){
    uint8_t c = 0;
    int idx = 0;
    if(canReadByte(com)){
        int n = readByte(com, &c);
        if(n > 0){
            if(c == '\r' || c =='\n') {
                com->buff[idx] = '\0';
                return 1;
            }
            com->buff[idx] = c;
            idx++;
            if(idx > BUFF_SIZE) idx = 0;
        }
    }
    return 0;
} /*readData*/

/*! \brief Reads complete line from the incomming buffer
 *  
 *  \param com location that needs to be read from
 * 
 *  \return returns number of bytes read, returns -1 if it couldnt read a full line
 * 
 *  \details Clears the previous line from the buffer
 * 
 */
int readLine(device *com){
    memset(com->buff, 0 ,BUFF_SIZE);
    int idx = 0;

    while(!readData(com)){
        if(!canReadByte(com)) return -1;
        idx++;
    }
    return idx;
} /*readLine*/

/*! \brief  Send a line of data with a new line character at the end.
 *  
 *  \param com struct with device's serial information
 *
 *  \param data Data to be written to device
 * 
 *  \return Returns the number of bytes send, or -1 if there is an error
 */
ssize_t sendLine(device *com, char* data){
    size_t dataLength = strlen(data);  // Get the length of the string
    
    char sendData[dataLength + 1];
    
    for(int i = 0; i < dataLength; i++){
        sendData[i] = data[i];
    }

    sendData[dataLength] = '\n';

    return write(com->port.fd, sendData, sizeof(sendData));
} /*sendLine*/

/*! \brief  Initialize device and open the serial communication
 *  
 *  \param com struct with device's serial information
 *
 *  \param devicePort Port which the device is connected to (tty location)
 * 
 *  \param baudrate communication speed of the device
 * 
 */
void setupDevice(device *com, const char* devicePort, int baudrate){
    com->port.fd = openSerial(devicePort, baudrate);
    com->port.events = POLLIN;
} /*setupDevice*/

/*! \brief properly close serial device and empty buffer
*
*  \param com struct with device's serial information
*
*  \param waitSend True if you want to wait until the whole buffer has been send before closing
*
*/
void closeDevice(device *com, bool waitSend){
    if(waitSend) tcdrain(com->port.fd);
    else tcflush(com->port.fd, TCIOFLUSH);
    close(com->port.fd);
} /*closeDevice*/