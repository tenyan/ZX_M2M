/*
 * Copyright(c)2012, 徐工信息技术股份有限公司物联网事业部
 * All right reserved
 *
 * 文件名称: App_cfg.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 用户代码头文件包含
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2012-06-04,  创建本文件
 *
 */

#ifndef CFG_20200604_H
#define CFG_20200604_H

#define SW_VERSION_LEN	           24
#define SW_VERSION		  "AUX_MCU_WJ_G4_V03_201015"		//软件版本号 
/*
AUX_MCU--- ST协处理器代码
G4     --- 国四发动机,终端采用2路CAN
WJ     --- 挖机
*/
/*******************************************************************
4G_MCU_WJ_190502:          2019-5-2 
      1)在挖机LRC终端CAN机型(LRC_WJCAN_P102_190316)基础上更改为4G终端协处理器程序
AUX_MCUWJ_V191215:         2019-12-14
      1)修改4G模块开关机代码保证刚上电时不要在做关机操作
AUX_MCUWJ2CAN_V200402      2020-04-02
      1)挖机2路CAN通讯协议
AUX_MCU_WJ_G4_V02_200402   2020-05-16
      1)增加高频采集CAN配置功能
AUX_MCU_WJ_G4_V02_200731
      1)增加接收主处理升级通知,升级期间禁止进入休眠
AUX_MCU_WJ_G4_V03_201015
      1)增加CAN2指示灯状态
      2)AddCanFrameToBuff函数替换为A5_AddCanFrameToBuff函数
      3)在设置终端ID编号的时候同时开启硬狗
********************************************************************/
#define HW_VERSION		 0x0003					//硬件版本号V0.3	
/*采用硬件版本号来描述ST协处理器固件版本
  HW_VERSION 高字节代表 固件大版本号(不同厂家、不同类型等)，低字节代表版本更新
  大版本号分类:0-挖机中大吨位,1-矿机,2-重型
*/
#define BIT(x)	       	((uint32)1 << (x))

/*********************************************************************************************************
  Date types(Compiler specific)  数据类型（和编译器相关）                
*********************************************************************************************************/
typedef unsigned char  uint8;                   // 无符号8位整型变量  
typedef signed   char  int8;                    // 有符号8位整型变量  
typedef unsigned short uint16;                  // 无符号16位整型变量 
typedef signed   short int16;                   // 有符号16位整型变量 
typedef unsigned int   uint32;                  // 无符号32位整型变量 
typedef signed   int   int32;                   // 有符号32位整型变量 
typedef float          fp32;                    // 单精度浮点数（32位长度） 
typedef double         fp64;                    // 双精度浮点数（64位长度）
typedef unsigned char  BOOL; 


#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#define GLOBAL	 extern	
#define VIRTUAL_TIMER_EN	0	//虚拟计时器使能, 0:禁能, 1:使能

#define osWaitForever   0

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include  "includes.h"
#include  "BaseFun.h"
#include  "SystemModule.h"
#include  "systemcommand.h"
#include  "GsmModule.h"
#include  "CollectModule.h"
#include  "Mcu.h"
#include  "PcDebug.h"
#include  "SystemWorkMode.h"
#include  "rtc.h"
#include  "MX25L1606Mem.h"
#include  "amx8xx_v00.h"
#include  "McuUpload.h"
#include  "A5_Com1ProtocolLayer.h"
#include  "GpsModule.h"
#include  "A5_Com1HW.h"   
#include  "McuChQiUpload.h"
#include  "i2c.h"

#endif
