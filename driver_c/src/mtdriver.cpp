/******************************************************************************
 *
 * Copyright (c) 2019 yanglei.shi
 * All Rights Reserved
 *
 ******************************************************************************/

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/termios.h>
#include <sys/time.h>
#include <sys/select.h>


#include "mtdef.hpp"
#include "mtdriver.hpp"
using namespace std;

namespace MTI{

//uint16_t  16位
#define BSWAP_16(x) \
    (uint16_t)((((uint16_t)(x) & 0x00ff) << 8) | \
              (((uint16_t)(x) & 0xff00) >> 8) \
             )
//uint32_t 32位
#define BSWAP_32(x) \
    (uint32_t)((((uint32_t)(x) & 0xff000000) >> 24) | \
              (((uint32_t)(x) & 0x00ff0000) >> 8) | \
              (((uint32_t)(x) & 0x0000ff00) << 8) | \
              (((uint32_t)(x) & 0x000000ff) << 24) \
             )
//uint64_t 64位
#define BSWAP_64(x) \
    (uint64_t)((((uint64_t)(x) & 0xff00000000000000) >> 56) | \
              (((uint64_t)(x) & 0x00ff000000000000) >> 40) | \
              (((uint64_t)(x) & 0x0000ff0000000000) >> 24) | \
              (((uint64_t)(x) & 0x000000ff00000000) >> 8) | \
              (((uint64_t)(x) & 0x00000000ff000000) << 8) | \
              (((uint64_t)(x) & 0x0000000000ff0000) << 24) | \
              (((uint64_t)(x) & 0x000000000000ff00) << 40) | \
              (((uint64_t)(x) & 0x00000000000000ff) << 56) \
             )
#if 0
#define BSWAP_64(x) (((uint64_t)(x) & 0x00ff000000000000) >> 40) | \
    (((uint64_t)(x) & 0x0000ff0000000000) >> 24) | \
    (((uint64_t)(x) & 0x000000ff00000000) >> 8) | \
    (((uint64_t)(x) & 0x00000000ff000000) << 8) | \
    (((uint64_t)(x) & 0x0000000000ff0000) << 24) | \
    (((uint64_t)(x) & 0x000000000000ff00) << 40) | \
    (((uint64_t)(x) & 0x00000000000000ff) << 56) |
#endif
typedef union FLOAT_CONV{
  float f;
  char c[4];
} float_conv;

static float swapFloat(const char* value) {
  float_conv f;
  for (int i=0; i<4; i++) {
    f.c[i] = value[3-i];
  }
  return f.f;
}

static int32_t swapFloats(float *pFs, const char* datas, int32_t pFSize) {
  int32_t iRet = 0;
  for (int i=0; i<pFSize; i++) {
    pFs[i] = swapFloat(datas + i*4);
  }
  return iRet;
}

#define REC_BUFFER_MAX_SIZE 1024
#define TIME_OUT 2000       //timeout is 2 ms millisecond

static int32_t imuFd = -1;
struct timeval time;
fd_set fs_read;
static char recBits[REC_BUFFER_MAX_SIZE] = {0};

static float swapFloat(const char* value);
static int32_t swapFloats(float *pFs, const char* datas, int32_t pFSize);
static int32_t openUart(int32_t &mFd, const char *pDev);
static int32_t setNewOptions(int32_t mFd, const BAUD_RATE cBaudRate, const int mDatabits,
                             const int mStop, const bool sHandshake, const bool hHandshake);
static int32_t closeUart(int32_t &cFd);
static int32_t pollFdSelect();
static int32_t waitBits(char *pRecData, const int32_t recBitcnt);
static int32_t dataCrc(const char* mRecBuff, const int32_t mCalSize, const char mCrc);

////TODO unpack frame data
static int32_t choiceParse(const char* mRecBuff, const uint16_t dataId);
static int32_t parseTemperature(const char* mRecBuff, const uint16_t dataId);
static int32_t parseTimestamp(const char* mRecBuff, const uint16_t dataId);
static int32_t parseOrientation(const char* mRecBuff, const uint16_t dataId);
static int32_t parsePressure(const char* mRecBuff, const uint16_t dataId);
static int32_t parseAcceleration(const char* mRecBuff, const uint16_t dataId);
static int32_t parsePosition(const char* mRecBuff, const uint16_t dataId);
static int32_t parseGNSS(const char* mRecBuff, const uint16_t dataId);
static int32_t parseAngularVelocity(const char* mRecBuff, const uint16_t dataId);
static int32_t parseGPS(const char* mRecBuff, const uint16_t dataId);
static int32_t parseSCR(const char* mRecBuff, const uint16_t dataId);
static int32_t parseMagnetic(const char* mRecBuff, const uint16_t dataId);
static int32_t parseVelocity(const char* mRecBuff, const uint16_t dataId);
static int32_t parseStatus(const char* mRecBuff, const uint16_t dataId);


////////////////Internal function/////////////////////
static int32_t parseTemperature(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  float temp = 0.0;
  if ((dataId & XDI_DATA_FLOAT_FORMAT) == 0x0003) {
    //data format is double
    //double *pdt = (double*)mRecBuff;
    //temp = (double)(*pdt);
  } else {
    //data format is float
    temp = swapFloat(mRecBuff);
  }

  printf("frame data temperature is %f\n", temp);
  return iRet;
}

static int32_t parseTimestamp(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_DATA_TYPE) {
    case 0x10: {
      uint64_t ns = *((uint64_t *)mRecBuff);
      ns = BSWAP_64(ns);
      uint16_t year = BSWAP_16(*((uint16_t *)(mRecBuff + sizeof(unsigned long))));
      uint8_t *ptemp = (uint8_t*)(mRecBuff + sizeof(unsigned long) + sizeof(unsigned short));
      uint8_t month = ptemp[0], day = ptemp[1], hour = ptemp[2];
      uint8_t minute = ptemp[3], second = ptemp[4],flags = ptemp[5];
      printf("ns:%lu year:%u month:%u day:%u hour:%u minnte:%u second:%u flags:%u\n",
    		  ns, year, month, day, hour, minute, second, flags);
      break;
    } case 0x20: {
      uint16_t frameCounter = *((uint16_t*)mRecBuff);
      frameCounter = BSWAP_16(frameCounter);
      printf("frame counter:%d\n", frameCounter);
      break;
    } case 0x30: {
      uint64_t timeOfWeek = 0;
      timeOfWeek = *((uint64_t*)mRecBuff);
      printf("time of week :%lu\n",timeOfWeek);
      break;
    } case 0x40: {
      uint8_t gpsAge = 0;
      gpsAge = *((uint8_t*)mRecBuff);
      printf("gps Age :%d\n",gpsAge);
      break;
    } case 0x50: {
      uint8_t pressureAge = 0;
      pressureAge = *((uint8_t*)mRecBuff);
      printf("pressure Age :%d\n",pressureAge);
      break;
    } case 0x60: {
      uint64_t SampleTimeFine = 0;
      SampleTimeFine  = BSWAP_64(*((uint64_t*)mRecBuff));
      printf("SampleTimeFine :%ld\n",SampleTimeFine);
      break;
    } case 0x70: {
      uint64_t SampleTimeCoarse = 0;
      SampleTimeCoarse = *((uint64_t*)mRecBuff);
      printf("pressure Age :%ld\n",SampleTimeCoarse);
      break;
    } case 0x80: {
      uint16_t startFrame = 0, endFrame = 0;
      uint16_t *pF = (uint16_t*)mRecBuff;
      startFrame = pF[0];
      endFrame = pF[1];
      printf("start frame:%d end frame :%d\n", startFrame, endFrame);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}

static int32_t parseOrientation(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_USING_COORDINATE_SYS) {
    case 0x00: {
      printf("Orientation using ENU system!\n");
      break;
    } case 0x04: {
      printf("Orientation using NED system!\n");
      break;
    } case 0x08: {
      printf("Orientation using NWU system!\n");
      break;
    }
     default: {
      break;
    }
  }

  switch (dataId & XDI_DATA_TYPE) {
    case 0x10: {  // Quaternion
      float quater[4] = {0};
      swapFloats(quater, mRecBuff, 4);
      printf("Quaternion (%f,%f,%f,%f)\n", quater[0], quater[1], quater[2], quater[3]);
      break;
    } case 0x20: {  //Rotation Matrix
      float rotation[9] = {0};
      swapFloats(rotation, mRecBuff, 9);
      printf("rotation:(%f,%f,%f,%f,%f,%f,%f,%f,%f)!\n",
    		  rotation[0], rotation[1], rotation[2],
			  rotation[3], rotation[4], rotation[5],
			  rotation[6], rotation[7], rotation[8]);
      break;
    } case 0x30: {  //Euler Angles
      float euler[3] = {0};
      swapFloats(euler, mRecBuff, 3);
      printf("rool pitch yaw (%f,%f,%f)\n", euler[0], euler[1], euler[2]);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}

static int32_t parsePressure(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  if ((dataId & XDI_DATA_TYPE) == 0x10) {
    uint64_t pressure = *((uint64_t*)mRecBuff);
    printf("pressure is %ld\n", pressure);
  }
  return iRet;
}

static int32_t parseAcceleration(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_USING_COORDINATE_SYS) {
    case 0x00: {
      //printf("Orientation using ENU system!\n");
      break;
    } case 0x04: {
      //printf("Orientation using NED system!\n");
      break;
    } case 0x08: {
      //printf("Orientation using NWU system!\n");
      break;
    }
     default: {
      break;
    }
  }

  switch (dataId & XDI_DATA_TYPE) {
    case 0x10: {  // Delta V
      float Delta[3] = {0};
      swapFloats(Delta, mRecBuff, 3);
      printf("Delta(x,y,z) (%f,%f,%f)\n", Delta[0], Delta[1], Delta[2]);
      break;
    } case 0x20: {  //Acceleration
      float Acceleration[3] = {0};
      swapFloats(Acceleration, mRecBuff, 3);
      printf("Acceleration:(%f,%f,%f)!\n", Acceleration[0], Acceleration[1], Acceleration[2]);
      break;
    } case 0x30: {  //Free Acceleration
      float fAccele[3] = {0};
      swapFloats(fAccele, mRecBuff, 3);
      printf("free Acceleration (%f,%f,%f)\n", fAccele[0], fAccele[1], fAccele[2]);
      break;
    } case 0x40: {  //AccelerationHR
      float AccelerationHR[3] = {0};
      swapFloats(AccelerationHR, mRecBuff, 3);
      printf("AccelerationHR (%f,%f,%f)\n", AccelerationHR[0], AccelerationHR[1], AccelerationHR[2]);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}

static int32_t parsePosition(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_USING_COORDINATE_SYS) {
    case 0x00: {
      //printf("Orientation using ENU system!\n");
      break;
    } case 0x04: {
      //printf("Orientation using NED system!\n");
      break;
    } case 0x08: {
      //printf("Orientation using NWU system!\n");
      break;
    }
     default: {
      break;
    }
  }

  switch (dataId & XDI_DATA_TYPE) {
    case 0x10: {  //  Altitude MSL  # deprecated
      float AltitudeMSL = {0};
      memcpy(&AltitudeMSL, mRecBuff, sizeof(float));
      printf("AltitudeMSL(%f)\n", AltitudeMSL);
      break;
    } case 0x20: {  //Altitude Ellipsoid
      float AltitudeEllipsoid = {0};
      memcpy(&AltitudeEllipsoid, mRecBuff, sizeof(float));
      printf("AltitudeEllipsoid(%f)\n", AltitudeEllipsoid);
      break;
    } case 0x30: {  //Position ECEF
      float ecefX[3] = {0};
      memcpy(ecefX, mRecBuff, sizeof(float)*3);
      printf("Position ECEF (%f,%f,%f)\n", ecefX[0], ecefX[1], ecefX[2]);
      break;
    } case 0x40: {  //LatLon
      float LatLon[2] = {0};
      memcpy(LatLon, mRecBuff, sizeof(float)*2);
      printf("LatLon (%f,%f)\n", LatLon[0], LatLon[1]);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}
//TODO ******
static int32_t parseGNSS(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;

  return iRet;
}

static int32_t parseAngularVelocity(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_USING_COORDINATE_SYS) {
    case 0x00: {
      //printf("Orientation using ENU system!\n");
      break;
    } case 0x04: {
      //printf("Orientation using NED system!\n");
      break;
    } case 0x08: {
      //printf("Orientation using NWU system!\n");
      break;
    }
     default: {
      break;
    }
  }

  switch (dataId & XDI_DATA_TYPE) {
    case 0x20: {  //Rate of Turn
      float gyrXYZ[3] = {0};
      swapFloats(gyrXYZ, mRecBuff, 3);
      printf("gyrXYZ(%f, %f, %f)\n", gyrXYZ[0], gyrXYZ[1], gyrXYZ[2]);
      break;
    } case 0x30: {  //Delta Q
      float DeltaQ[3] = {0};
      memcpy(DeltaQ, mRecBuff, sizeof(float)*3);
      printf("Delta Q (%f,%f,%f)\n", DeltaQ[0], DeltaQ[1], DeltaQ[2]);
      break;
    } case 0x40: {  //RateOfTurnHR
      float gyrXYZ[3] = {0};
      memcpy(gyrXYZ, mRecBuff, sizeof(float)*3);
      printf("RateOfTurnHR gyrXYZ (%f,%f,%f)\n", gyrXYZ[0], gyrXYZ[1], gyrXYZ[2]);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}

//TODO if used
static int32_t parseGPS(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;

  return iRet;
}

//TODO if used
static int32_t parseSCR(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;

  return iRet;
}

//TODO if used
static int32_t parseMagnetic(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;

  return iRet;
}

static int32_t parseVelocity(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_USING_COORDINATE_SYS) {
    case 0x00: {
      printf("Orientation using ENU system!\n");
      break;
    } case 0x04: {
      printf("Orientation using NED system!\n");
      break;
    } case 0x08: {
      printf("Orientation using NWU system!\n");
      break;
    }
     default: {
      break;
    }
  }

  if ((dataId & XDI_DATA_TYPE) == 0x10) {
    float velXYZ[3] = {0};
    memcpy(velXYZ, mRecBuff, sizeof(float)*3);
    printf("RateOfTurnHR gyrXYZ (%f,%f,%f)\n", velXYZ[0], velXYZ[1], velXYZ[2]);
  }
  return iRet;
}

//TODO if used
static int32_t parseStatus(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;

  return iRet;
}

static int32_t choiceParse(const char* mRecBuff, const uint16_t dataId) {
  int32_t iRet = 0;
  switch (dataId & XDI_GROUP) {
    case XDI_GAROUPS_TEMPERATURE: {
      parseTemperature(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_TIMESTAMP: {
      parseTimestamp(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_ORIENTATION: {
      parseOrientation(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_PRESSURE: {
      parsePressure(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_ACCELERATION: {
      parseAcceleration(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_POSITION: {
      parsePosition(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_GNSS: {
      parseGNSS(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_ANGULAR_V: {
      parseAngularVelocity(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_GPS: {
      parseGPS(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_SENSOR_READOUT: {
      parseSCR(mRecBuff, dataId);
      break;
    }  case XDI_GAROUPS_MAGNETIC: {
      parseMagnetic(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_VElOCITY: {
      parseVelocity(mRecBuff, dataId);
      break;
    } case XDI_GAROUPS_STATUS: {
      parseStatus(mRecBuff, dataId);
      break;
    }
    default: {
      break;
    }
  }
  return iRet;
}

static int32_t openUart(int32_t &mFd, const char *pDev) {
  int32_t iRet = 0;

  int32_t fd = -1;
  fd = open(pDev, O_RDWR|O_NDELAY);
  if (-1 == fd) {
    perror("Can't Open Serial Port");
    return -1;
  }
  tcflush(fd, TCIOFLUSH);

  int n = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, n & ~O_NDELAY);

  printf("fd open success:%d\n", fd);
  mFd = fd;

  return iRet;
}

static int32_t setNewOptions(int32_t mFd, const BAUD_RATE cBaudRate, const int mDatabits,
                             const int mStop, const bool sHandshake, const bool hHandshake){
  struct termios newtio;
  memset(&newtio, 0, sizeof(newtio));
  if (tcgetattr(mFd, &newtio)!=0) {
    printf("tcgetattr() 3 failed");
  }
  cfsetospeed(&newtio, (speed_t)cBaudRate);
  cfsetispeed(&newtio, (speed_t)cBaudRate);

   /* We generate mark and space parity ourself. */
  switch (mDatabits) {
    case 5: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS5;
      break;
    }
    case 6: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS6;
      break;
    }
    case 7: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS7;
      break;
    }
    case 8:
    default: {
      newtio.c_cflag = (newtio.c_cflag & ~CSIZE) | CS8;
      break;
    }
  }
  newtio.c_cflag |= CLOCAL | CREAD;

  //parity
  newtio.c_cflag &= ~(PARENB | PARODD);

  newtio.c_cflag &= ~CRTSCTS;

  //stopbits
  if (mStop == 2) {
    newtio.c_cflag |= CSTOPB;
  } else {
    newtio.c_cflag &= ~CSTOPB;
  }

  newtio.c_iflag=IGNBRK;

  //software handshake
  if (sHandshake) {
    newtio.c_iflag |= IXON | IXOFF;
  } else {
    newtio.c_iflag &= ~(IXON|IXOFF|IXANY);
  }

  newtio.c_lflag=0;
  newtio.c_oflag=0;

  newtio.c_cc[VTIME]=1;
  newtio.c_cc[VMIN]=60;

  tcflush(mFd, TCIFLUSH);
  if (tcsetattr(mFd, TCSANOW, &newtio)!=0) {
    printf("tcsetattr() 1 failed");
  }

  int mcs=0;
  ioctl(mFd, TIOCMGET, &mcs);
  mcs |= TIOCM_RTS;
  ioctl(mFd, TIOCMSET, &mcs);

  if (tcgetattr(mFd, &newtio)!=0) {
    printf("tcgetattr() 4 failed");
  }

  //hardware handshake
  if (hHandshake) {
    newtio.c_cflag |= CRTSCTS;
  } else {
    newtio.c_cflag &= ~CRTSCTS;
  }

  if (tcsetattr(mFd, TCSANOW, &newtio)!=0) {
    perror("tcsetattr() 2 failed");
  }
  return 0;
}

static int32_t closeUart(int32_t &cFd){
  int32_t iRet = 0;
  if (cFd < 0) {
    perror("fd is not exist");
    return -1;
  }

  iRet = close(cFd);
  if(iRet != 0){
    perror("close fd failed");
    return -2;
  }

  return iRet;
}

static int32_t pollFdSelect() {
  int32_t iRet = 0;
  FD_ZERO(&fs_read);
  FD_SET(imuFd, &fs_read);
  time.tv_sec = 0;
  time.tv_usec = 100;
  //使用select实现串口的多路通信
  iRet = select(imuFd+1, &fs_read, NULL, NULL, &time);
  return iRet;
}

//not check pRecData validity,you have to make sure that pRecData is large recBitcnt.
static int32_t waitBits(char *pRecData, const int32_t recBitcnt) {
  int32_t iRet = 0;
  int32_t recLen = 0, rectemp = 0, waitTimes = 0;
  #define MAX_WAIT_TIMES 20
  while (recLen < recBitcnt) {
    if (pollFdSelect() <= 0) {
      waitTimes++;
      if (waitTimes > MAX_WAIT_TIMES) {
        perror("select wait time out!\n");
        return -1;
      }
      continue;
    }
    rectemp = read(imuFd, pRecData+recLen, recBitcnt-recLen);
    recLen += rectemp;
  }
  return iRet;
}

static int32_t dataCrc(const char* mRecBuff, const int32_t mCalSize, const char mCrc) {
  int32_t iRet = 0;
  uint16_t iCrc = 0;
  if (mRecBuff == NULL) {
    return -1;
  }
  for(int32_t i = 0; i < mCalSize; i++) {
    iCrc += (uint8_t)mRecBuff[i];
  }
  //TODO  sum(data, 0xFF+mid+length+checksum)
  iCrc += (0xFF + FRAME_MTI_DAtA2 + (uint8_t)mCrc + (uint8_t)mCalSize);
  iCrc = iCrc & 0x00ff;
  if (iCrc) {
    return -2;
  }
  return iRet;
}
////////////////External function/////////////////////
int32_t initMtiImu(const char ttyUsb[]){
  int32_t iRet = 0;
  iRet = openUart(imuFd, ttyUsb);
  if (iRet != 0) {
    return -1;
  }
  iRet = setNewOptions(imuFd, BAUD_B115200, 8, 1, 0, 0);
  if (iRet < 0) {
    return -2;
  }
  printf("set imu success");
  return iRet;
}

int32_t deInitAll() {
  int32_t iRet = 0;
  if (imuFd > 0) {
    closeUart(imuFd);
  }
  return iRet;
}

//TODO*********** do not check pImuData is valid
int32_t recImuBits(char *pImuData, int32_t &rImuBitCnt) {
  int32_t iRet = -1;
  rImuBitCnt = 0;
  struct timeval startT, nowT;
  gettimeofday(&startT, NULL);

  long long int diffT = 0;
  while (diffT <  TIME_OUT) {
    gettimeofday(&nowT, NULL);
    diffT = (nowT.tv_sec - startT.tv_sec)* 1000000 + (nowT.tv_usec - startT.tv_usec);

    iRet = waitBits(recBits, 1);
    if (iRet != 0 || (uint8_t)recBits[0] != FRAME_A1) {
      //printf("not frame a1\n");
      continue;
    }
    iRet = waitBits(recBits, 1);
    if (iRet != 0 || (uint8_t)recBits[0] != FRAME_A2){
      //printf("not frame a2\n");
      continue;
    }
    iRet = waitBits(recBits, 1);
    if (iRet != 0 || (uint8_t)recBits[0] != FRAME_MTI_DAtA2){
      //perror("not frame data2!\n");
      continue;
    }
    iRet = waitBits(recBits, 1);
    if (iRet != 0){
      //perror("read frame size failed\n");
      continue;
    }
    int32_t dataSize = (int32_t)(uint32_t)(recBits[0]);
    iRet = waitBits(recBits, dataSize+1);
    if (iRet != 0){
      //perror("read frame data failed\n");
      continue;
    }
    //TODO  crc
    iRet = dataCrc(recBits, dataSize, recBits[dataSize]);
    if (iRet != 0) {
      printf("crc check failed\n");
      continue;
    }
    rImuBitCnt = dataSize;
    memcpy(pImuData, recBits, dataSize);
    return 0;
  }
  printf("time out, read mti failed\n");
  return iRet;
}

int32_t parseMTIData2(char *pImuData, const int32_t imuDateSize) {
  int32_t iRet = 0;
  char* pData = pImuData;
  for (int i=0; i<imuDateSize;) {
    uint16_t dataiId = BSWAP_16(*((uint16_t*)pData));
    uint8_t dataLen = *((uint8_t*)(pData + 2));
    pData += 3;
    choiceParse(pData, dataiId);
    pData += dataLen;
    i += (3 + dataLen);
  }

  return iRet;
}

}//end namespace


/*******************************************
  main function interface
******************************************/
#include <stdlib.h>
#include <signal.h>

void sig_handler( int sig) {
  if(sig == SIGINT) {
    cout<<"ctrl+c has been keydownd"<<endl;
    exit(0);
  }
}

int main() {
  signal(SIGINT, sig_handler);
  int32_t iRet = 0;
  //printf("float: %d, double:%d",sizeof(unsigned short), sizeof(double));

  iRet = MTI::initMtiImu("/dev/ttyUSB0");
  if (iRet < 0) {
    printf("exit  !\n");
    return -1;
  }
  char bits[1024];
  int32_t recSize = 0;
  int mun = 0;
  while(mun<10){
    if (MTI::recImuBits(bits, recSize) == 0) {
      MTI::parseMTIData2(bits, recSize);
    } else {
      printf("recsize: %d  counter:%d\n", recSize, (uint8_t)bits[4]);
    }
      usleep(100);
      mun++;
  }
  MTI::deInitAll();
  return 0;
}
