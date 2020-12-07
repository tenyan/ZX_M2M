/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: PcDebug.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为PcDebug功能模块协议层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by lxf, 创建本文件
 *
 */

#ifndef _PCDEBUG_H
#define _PCDEBUG_H
#include "PcHardware.h"
//-----常量定义----------------------------------------------------------------
#define TASK_PCDEBUG_STK_SIZE            150  
#define TASK_PCDEBUG_ID                  9   
#define TASK_PCDEBUG_PRIO                9   

#define COMM_READ_TIMEOUT                5             /*uart0 串口超时时间5个时钟节拍*/
#define UART0_RECV_BUFF_LENGTH           256           /*uart0 接收数据最大长度*/
  
#define FORMAT                           0xF9          /*设置程序数据包头/包尾*/


//调试输出函数模块选择定义  详见函数 PC_SendDebugData(uint8* ptr,uint16 usLen,uint8 ucModuleSelect)
#define  DEBUG_GSMMODULE                 1            //GSM模块
#define  DEBUG_GPSMODULE                 2            //GPS模块
#define  DEBUG_MCUMODULE                 3            //MCU模块
#define  DEBUG_RS232MODULE               4            //MCU-RS232接口
#define  DEBUG_SYSMODULE                 5            //system模块
#define  DEBUG_ANYDATA                   10           //调用该数据时，可以输出任意数据 
#define  DEBUG_GPRS						 5			  //打印GPRS数据
#define  DEBUG_AT                        1			  //打印AT指令

#define  PRINTBUFF                       100          //SMS及GPRS打印数据缓冲区

#define  SMS_SEND                        1            //短信发送标识
#define  SMS_RECV                        2            //短信接收标识
#define  GPRS_SEND                       3            //GPRS数据发送标识
#define  GPRS_RECV                       4            //GPRS数据接收标识
#define  GPRS_STO                        5            //GPRS数据超时发送标识



//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
//void PC_Print(const uint8 *pucSrc, uint16 usLen, uint8 ucFlag);
void PC_SendDebugData(uint8* ptr,uint16 usLen,uint8 ucModuleSelect);   //调试输出数据信息的函数,可以被不同模块调用
//void PC_SendToPCData(uint8 *ptr, uint16 usLen);
void PC_SendToPCData2(uint8 *ptr, uint16 usLen);
void TaskPcDebug(void *pdata);
void A5_ExecRecvDebugData(uint8 *p, uint16 usLen);

#endif
