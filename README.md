# RPI-serial
Library used for serial communication on the raspberry PI

### `void setupDevice(device *com, const char *devicePort, int baudrate)`

**Description:**  
Function to setup all parameters for a serial device

**Parameters:**  
- `*com` — Array where device information should be saved 
- `*devicePort` — TTY port name 
- `baudrate` — Communication baudrate


### `void closeDevice(device *com, bool waitSend)`

**Description:**  
Function to close serial port

**Parameters:**  
- `*com` — Array with device variables
- `waitSend` — Set true if you want to halt the code until the port is closed


### `int canReadByte(device *com)`

**Description:**  
Check if a byte can be read for the serial device

**Parameters:**  
- `*com` — Array with device variables

**Returns:**  
`1` if there is a byte to be read.
`0` if there is no bytes to be read.


### `int readByte(device *com, uint8_t* byte)`

**Description:**  
Read a single byte for the serial device

**Parameters:**  
- `*com` — Array with device variables
- `*byte` — Location where the byte should be saved

**Returns:**  
`1` if reading byte is succesfull.
`0` if 'EOF' is reached.
`-1` if an error occured.


### `void flushBuffer(device *com)`

**Description:**  
Completely clear serial device's buffer of data. (Needs to be done if data has been send from serial device but has not been read with the code)

**Parameters:**  
- `*com` — Array with device variables


### `int readLine(device *com)`

**Description:**  
Read a full line from serial device

**Parameters:**  
- `*com` — Array with device variables

**Returns:**  
`1` if reading a full line is succesfull (ends with EOL).
`0` if reading a full line is not succesfull (doesn't ends with EOL).


### `ssize_t sendLine(device *com, char* data)`

**Description:**  
Send data with a EOL at the end of it.

**Parameters:**  
- `*com` — Array with device variables
- `*data` — Data to be send.

**Returns:**  
Number of bytes that have been send.
`-1` if an error occured.