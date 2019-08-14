/******************************************************************************
 *
 * Copyright (c) 2019 yanglei.shi
 * All Rights Reserved
 *
 ******************************************************************************/

#ifndef MTDRIVER_HPP_
#define MTDRIVER_HPP_

#include <stdint.h>
#include <string>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "mtdef.hpp"

namespace MTI{

int32_t initMtiImu(const char ttyUsb[]);
int32_t deInitAll();

int32_t recImuBits(char *pImuData, int32_t &rImuBits);
int32_t parseMTIData2(char *pImuData, const int32_t imuDateSize, MT_IMU_MSG *pMsg);

}//end namespace MTI

#endif /* MTDRIVER_HPP_ */
