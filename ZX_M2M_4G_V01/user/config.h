
/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: Config.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 用户代码头文件包含
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2020-11-24,  创建本文件
 *
 */

#ifndef CFG_20190306_H
#define CFG_20190306_H


#define SW_VERSION_LEN	           23
#define SW_VERSION		  "WLRC_ZX_M2M_201125_V001"		//软件版本号
/*******************************************************************
WLRCWJ_4G_200222_V002: 2020-02-22
                       1)增加接收到PC或者平台下发的参数设置命令转发给协处理器
WLRCWJ_4G_200415_V003: 2020-04-15
                       1)在上一版本基础采用挖机4G新的总线通讯协议
WLRC_WJ_2CAN_200522_V003:2020-05-22
                       1)增加CAN高频参数设置和查询
WLRC_WJ_2CAN_200613_V001:2020-06-13
WLRC_WJ_2CAN_200728_V002:
                       1)更改信号强度查询结果判断,不满足可以向下执行
                       2)更改at+creg?查询结果判断,不满足可以向下执行
                       3)修改AT+CGPADDR返回IP地址的判断方法,收到任意一路ip都认定为成功
                       4)主\协处理器远程升级地址移植到data路径下      
WLRC_WJ_2CAN_200804_V002:
                       1)增加远程升级器件通知ST协处理器禁止进入休眠功能
WLRC_WJ_2CAN_200929_V006:
                       1)修订MAX_CAN_FRAME_NUM 由56变更为90
                       2)去掉0x030F故障TLV
WLRC_WJ_2CAN_201112_V007:
                       1)参照<4G终端协议B2>总线协议变更部分CAN帧
                       2)获取控制器多包故障采集及多包软件版本号
                       3)获取康明斯、五十铃发动机信息
                       4)获取发动机多包故障信息
                       5) ucOpenTimes++;语句提至AT指令之前
WLRC_ZX_M2M_201125_V001  :2020-11-25
                       1)在挖机WLRC_WJ_2CAN_201112_V007版本基础上修改为重型M2M版本
********************************************************************/
#define HW_VERSION		     0x0102					//硬件版本号V1.2	
#define BIT(x)	         	((uint32)1 << (x))
#define OS_TICKS_PER_SEC     1000u   /* Set the number of ticks in one second                        */

/*********************************************************************************************************
  Date types(Compiler specific)  数据类型（和编译器相关）                
*********************************************************************************************************/
//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef unsigned char  BOOLEAN;

typedef unsigned char  uint8;                   // 无符号8位整型变量  
typedef signed   char  int8;                    // 有符号8位整型变量  
typedef unsigned short uint16;                  // 无符号16位整型变量 
typedef signed   short int16;                   // 有符号16位整型变量 
typedef unsigned int   uint32;                  // 无符号32位整型变量 
typedef signed   int   int32;                   // 有符号32位整型变量 
typedef float          fp32;                    // 单精度浮点数（32位长度） 
typedef double         fp64;                    // 双精度浮点数（64位长度）
typedef unsigned char  BOOL; 


typedef unsigned char  uint8_t;                   // 无符号8位整型变量  
typedef signed   char  int8_t;                    // 有符号8位整型变量  
typedef unsigned short uint16_t;                  // 无符号16位整型变量 
typedef signed   short int16_t;                   // 有符号16位整型变量 
typedef unsigned int   uint32_t;                  // 无符号32位整型变量 
typedef signed   int   int32_t;                   // 有符号32位整型变量 
//typedef float          fp32;                    // 单精度浮点数（32位长度） 
//typedef double         fp64;                    // 双精度浮点数（64位长度）
//typedef unsigned char  BOOL; 


#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#define GLOBAL	 extern	
//#define VIRTUAL_TIMER_EN	0	//虚拟计时器使能, 0:禁能, 1:使能

#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>   
#include <limits.h> 
#include <asm/ioctls.h>
#include <time.h>
#include <unistd.h>
#include "pthread.h"
#include <syslog.h>
#include <stdint.h>
#include <sys/poll.h>

#include <linux/input.h>
/*-------------------------------    */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
//#include  "includes.h"
#include  "BaseFun.h"
#include  "SystemModule.h"
#include  "SystemCommand.h"
#include  "GsmModule.h"
#include  "GpsModule.h"
#include  "CollectModule.h"
#include  "Mcu.h"
#include  "PcDebug.h"
#include  "SystemWorkMode.h"
#include  "rtc.h"
#include  "MX25L1606Mem.h"
#include  "amx8xx.h"
#include  "McuUpload.h"
#include  "Gpio_cfg.h"
#include  "Watchdog.h"
#include  "A5UART0_MCU.h"
#include  "Canconfig.h"
//#include  <tcp_client.h>

#endif




