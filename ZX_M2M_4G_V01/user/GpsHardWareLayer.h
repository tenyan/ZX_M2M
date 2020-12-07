/*
 * Copyright(c)2019,硬件研发部
 * All right reserved
 *
 * 文件名称: GsmHardWareLayer.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块硬件驱动层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by , 创建本文件
 *
 */
 
#ifndef _GPSHARDWARELAYER_H_
#define _GPSHARDWARELAYER_H_

//-----常量定义----------------------------------------------------------------
#define GPS_UART_BAUD_RATE              9600

#define GPS_UART_SEND_BUFF_LENGTH       256
#define GPS_UART_RECV_BUFF_LENGTH       1024

#define GpsPower_SET()          //(GPIO_SetBits(GPIO_PC30)) //打开GPS电源PB4
#define GpsPower_CLR()          //(GPIO_ResetBits(GPIO_PC30)) //关闭GPS电源PB4

//#define GpsRest()			    ()  //GPS模块休眠
//#define GpsWork()				()  //GPS模块恢复工作

//#define GpsLedOn()				//(GPIO_ResetBits(GPIO_PC15))	//PE3
//#define GpsLedOff()     		//(GPIO_SetBits(GPIO_PC15))	//PE3 


//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
void GPS_Uart_Init(void);
BOOL ReadGpsUartData(uint8 **data, uint16* len);
uint16 WriteGpsUartData(uint8 *data, uint16 Len);
#endif


































