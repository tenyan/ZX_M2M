/*****************************************************************************
* Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
* All right reserved
* @FileName: config.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-25
* @brief
******************************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

/******************************************************************************
*   Macros
******************************************************************************/
#define SW_VERSION        "ZXM2M_4G_20201225_V001"  //软件版本号
#define SW_VERSION_LEN    (sizeof(SW_VERSION)-1)

#define HW_VERSION           0x0102  //硬件版本号V1.2
#define EP_FIRMWARE_VERSION  02 // 环保协议里的软件版本

/******************************************************************************
* INCLUDES
******************************************************************************/
#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <limits.h>
#include <asm/ioctls.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <syslog.h>
#include <stdint.h>
#include <linux/ioctl.h>
#include <linux/input.h>
#include <linux/types.h>
#include <getopt.h>
#include <signal.h>

//==================================
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

#include "types.h"
#include "Crc32.h"
#include "main.h"
#include "Led.h"
#include "Rtc.h"
#include "PcDebug.h"
#include "CollectModule.h"
#include "CelluraModule.h"
#include "GpsModule.h"
#include "MomiProtocol.h"
#include "M2mProtocol.h"
#include "HJ.h"
#include "GB.h"
#include "iCloud.h"
#include "Parameters.h"
#include "AuxCom.h"
#include "IAP.h"
#include "BlindZone.h"

#endif /* _CONFIG_H_ */

