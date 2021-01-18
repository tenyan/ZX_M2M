/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief     本文件为辅助通信模块的头文件
 ********************************************************************************/
#ifndef _AUX_COM_H_
#define _AUX_COM_H_

//-----头文件调用------------------------------------------------------------
#include "types.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define DEV_TTYS4_UART0      "/dev/ttyHS3"

#define AuxCom_Transmit          AUX_UartTransmitData
#define AuxCom_Receive           AUX_UartReceiveData

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// 升级状态
typedef enum
{
  FSRV_STATE_IDLE = 0x00,  /// 空闲
  FSRV_STATE_SEND_UN,      /// 发送通知升级
  FSRV_STATE_WAIT_REQ,     /// 等待ST请求
  FSRV_STATE_SEND_UD,      /// 发送升级数据(响应ST请求)
  FSRV_STATE_REPORT_REQ,   /// 向服务器上报升级结果
}fsrv_state_t;

#define FSRV_FIRMWARE_PACKET_LEN    1024//固件升级包的长度
// 固件升级结构体
typedef struct 
{
  uint8_t state; // 升级状态
  uint8_t sent_flag;         // 请求已发出
  uint8_t rsp_timeout_timer; // 超时设定值
  uint8_t retry_cnt;         // 重试计数器
  uint16_t timeout_cnt;      // 升级超时计数器,如果10分钟内没有升级完成则放弃
  uint8_t result;            // 下包结果0=成功,1=失败
}fsrv_context_t;


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif
