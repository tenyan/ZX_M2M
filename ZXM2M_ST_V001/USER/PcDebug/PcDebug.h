/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebug.h
 * @Engineer: TenYan
 * @Company:  徐工信息智能硬件部
 * @version   V1.0
 * @Date:     2020-6-3
 * @brief     本文件为PcDebug功能模块协议层的头文件
 ********************************************************************************/
#ifndef _PCDEBUG_H_
#define _PCDEBUG_H_

//-----头文件调用------------------------------------------------------------
#include "types.h"

//-----常量定义----------------------------------------------------------------
#define TASK_PCDEBUG_STK_SIZE            150
#define TASK_PCDEBUG_ID                  8
#define TASK_PCDEBUG_PRIO                8

#define COMM_READ_TIMEOUT                5             /*uart0 串口超时时间5个时钟节拍*/
#define UART0_RECV_BUFF_LENGTH           256           /*uart0 接收数据最大长度*/

/******************************************************************************
 * Macros
 ******************************************************************************/
// 调试输出函数模块选择定义
#define DBG_MSG_TYPE_AT       1    // 打印AT指令
#define DBG_MSG_TYPE_MODEM    1    // MODEM模块
#define DBG_MSG_TYPE_GPS      2    // GPS模块
#define DBG_MSG_TYPE_CAN      3    // CAN模块
#define DBG_MSG_TYPE_RS232    4    // MCU-RS232接口
#define DBG_MSG_TYPE_SYS      5    // system模块
#define DBG_MSG_TYPE_GPRS     5    // 打印GPRS数据
#define DBG_MSG_TYPE_ANYDATA  10   // 输出任意数据

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void PcDebug_SetStatus(uint8_t enable_type);
void PcDebug_SendData(uint8_t* pdata,uint16_t size,uint8_t msg_types);
void PcDebug_SendM2mRsp(uint8_t *pdata, uint16_t size);
void PcDebug_SendString(const char *pstr);
void PcDebug_Printf(const char *format,...);
void PcDebug_ServiceInit(void);
void PcDebug_ServiceStart(void);

#endif
