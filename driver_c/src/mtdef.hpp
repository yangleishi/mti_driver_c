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

//Newer data packet (MTi-10/100 series only)
#define FRAME_MTI_DAtA2 0x36

/**********************************************************************************
波特率变量
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
