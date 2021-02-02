/*****************************************************************************
* @FileName: config.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-12-9
* @brief
******************************************************************************/
#ifndef _CONFIG_H_
#define _CONFIG_H_

/******************************************************************************
*   Macros
******************************************************************************/
#define HYCT02_VERSION        ("1.0.1")
#define HYCT02_VERSION_MAJOR  (1)
#define HYCT02_VERSION_MINOR  (0)
#define HYCT02_VERSION_PATCH  (1)

#define SW_VERSION_DATE   "20201222"
#define SW_VERSION        "ZXM2M-ST-20201220-V001"  //软件版本号
#define SW_VERSION_LEN    (sizeof(SW_VERSION)-1)
#define HW_VERSION        0x0108    //硬件版本号1.1

#define EP_FIRMWARE_VERSION     02 // 环保协议里的软件版本

/******************************************************************************
* INCLUDES
******************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <ctype.h>
#include <inttypes.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "stm32f2xx.h"

#include "types.h"
#include "main.h"
#include "led.h"
#include "sfud.h"
#include "rtc.h"
#include "Can.h"
#include "ISO27145.h"
#include "PcDebug.h"
#include "MomiProtocol.h"
#include "tcw.h"

//#include "GpsModule.h"
//#include "CelluraModule.h"
#include "M2mProtocol.h"

#include "CollectModule.h"
#include "parameters.h"
#include "tbox_machine.h"
#include "icloud_machine.h"
#include "crc32.h"
#include "iap.h"
#include "AuxCom.h"

#endif /* _CONFIG_H_ */

