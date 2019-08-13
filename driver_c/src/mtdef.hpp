/******************************************************************************
 *
 * Copyright (c) 2019 yanglei.shi
 * All Rights Reserved
 *
 ******************************************************************************/

#ifndef MTDEF_HPP_
#define MTDEF_HPP_

#include <stdint.h>
#include <termios.h>

namespace MTI{

//Header marking for each frame
#define FRAME_A1 0xfa
#define FRAME_A2 0xff

/*********************************************************************************
 * MID message id
 ********************************************************************************/
//Newer data packet (MTi-10/100 series only)
const uint16_t  FRAME_MTI_DAtA2 = 0x36;

//Error message, 1 data byte
const uint16_t  ERROR_MID = 0x42;



/*********************************************************************************
 * ERROR_MID ErrorCodes
 ********************************************************************************
0: "Operation was performed successfully",
1: "No bus communication possible",
2: "InitBus and/or SetBID are not issued",
3: "Period sent is invalid",
4: "The message is invalid or not implemented",
16: "A slave did not respond to WaitForSetBID",
17: "An incorrect answer received after WaitForSetBID",
18: "After four bus-scans still undetected Motion Trackers",
20: "No reply to SetBID message during SetBID procedure",
21: "Other than SetBIDAck received",
24: "Timer overflow - period too short to collect all data from Motion Trackers",
25: "Motion Tracker responds with other than SlaveData message",
26: "Total bytes of data of Motion Trackers including sample counter exceeds 255 bytes",
27: "Timer overflows during measurement",
28: "Timer overflows during measurement",
29: "No correct response from Motion Tracker during measurement",
30: "Timer overflows during measurement",
32: "Baud rate does not comply with valid range",
33: "An invalid parameter is supplied",
35: "TX PC Buffer is full",
36: "TX PC Buffer overflow, cannot fit full message",
37: "Wireless subsystem failed",
40: "The device generated an error, try updating the firmware",
41: "The device generates more data than the bus communication can handle (baud rate may be too low)",
42: "The sample buffer of the device was full during a communication outage",
43: "The external trigger is not behaving as configured",
44: "The sample stream detected an error in the ordering of sample data",
45: "A dip in the power supply was detected and recovered from",
46: "A current limiter has been activated, shutting down the device",
47: "Device temperature is not within operational limits",
48: "Battery level reached lower limit",
49: "Specified filter profile ID is not available on the device or the user is trying to duplicate an existing filter profile type",
50: "The settings stored in the device's non volatile memory are invalid",
*/
const uint8_t ERROR_CODE_OPERATION_SUCCESSFULLY = 0;
const uint8_t ERROR_CODE_NO_BUS_COMMUNICATION = 1;
const uint8_t ERROR_CODE_INITBUS_NOT_ISSUED = 2;
const uint8_t ERROR_CODE_PERIOD_SEND_INVALID = 3;
const uint8_t ERROR_CODE_MESSAGE_INVALID = 4;
const uint8_t ERROR_CODE_BAUD_RATE_LOW = 41;

/**********************************************************************************
波特率变量,RS232 max baud rate is 115200, RS485 max baud rate can set 921600.
**********************************************************************************/
typedef enum BAUD_RATE {
  BAUD_B4800 = B4800,
  BAUD_B9600 = B9600,
  BAUD_B19200 = B19200,
  BAUD_B38400 = B38400,
  BAUD_B57600 = B57600,
  BAUD_B115200 = B115200,
  BAUD_B230400 = B230400,
  BAUD_B460800 = B460800,
  BAUD_B921600 = B921600,
} BAUD_RATE;

///Values for the XDI groups
const uint16_t XDI_GAROUPS_TEMPERATURE = 0x0800;
const uint16_t XDI_GAROUPS_TIMESTAMP = 0x1000;
const uint16_t XDI_GAROUPS_ORIENTATION = 0x2000;
const uint16_t XDI_GAROUPS_PRESSURE = 0x3000;
const uint16_t XDI_GAROUPS_ACCELERATION = 0x4000;
const uint16_t XDI_GAROUPS_POSITION = 0x5000;
const uint16_t XDI_GAROUPS_GNSS = 0x7000;
const uint16_t XDI_GAROUPS_ANGULAR_V = 0x8000;
const uint16_t XDI_GAROUPS_GPS = 0x8800;
const uint16_t XDI_GAROUPS_SENSOR_READOUT = 0xA000;
const uint16_t XDI_GAROUPS_AnalogIn = 0x4000;
const uint16_t XDI_GAROUPS_MAGNETIC = 0xC000;
const uint16_t XDI_GAROUPS_VElOCITY = 0xD000;
const uint16_t XDI_GAROUPS_STATUS = 0xE000;

//check data is float or double
const uint16_t XDI_DATA_FLOAT_FORMAT = 0x0003;

//check MTI using which coordinate sys,include ENU NED NWU
const uint16_t XDI_DATA_TYPE = 0x00F0;

//check MTI using which coordinate sys,include ENU NED NWU
const uint16_t XDI_GROUP = 0xF800;

//check MTI using which coordinate sys,include ENU NED NWU
const uint16_t XDI_USING_COORDINATE_SYS = 0x000C;

}//end namespace MTI


#endif /* MTDEF_HPP_ */
