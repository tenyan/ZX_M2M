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
#define DEV_TTYS4_UART0    "/dev/ttyHS3"

#define AuxCom_Transmit    AUX_UartTransmitData
#define AuxCom_Receive     AUX_UartReceiveData

#define FSRV_DEBUG    1

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// 升级状态
typedef enum
{
  FSRV_STATE_IDLE = 0x00,  /// 空闲
  FSRV_STATE_START,        /// 启动升级
  FSRV_STATE_SEND_UN,      /// 发送通知升级
  FSRV_STATE_WAIT_REQ,     /// 等待ST请求和响应
  FSRV_STATE_SEND_UD,      /// 发送升级数据(响应ST请求)
  FSRV_STATE_REPORT,       /// 向服务器上报升级结果
}fsrv_state_t;

#define FSRV_FIRMWARE_PACKET_LEN    1024//固件升级包的长度
// 固件升级结构体
typedef struct 
{
  uint8_t state;  // 升级状态
  uint8_t dev_id;  // 被升级设备号: 0x00=无效, 0x01=控制器, 0x03=协处理器
  uint8_t msg_sn;  // 流水号
  uint16_t fw_packet_index;  // 升级包序号

  uint32_t start_address;  // 已接收长度
  uint32_t ending_address;  // 总长度

  uint16_t fw_block_size; // 分包大小:固定为512或1024
  uint16_t total_block_count;  // 程序总块数
  uint8_t percent;             // 升级进度

  uint8_t retry_sp;  // 重试次数
  uint8_t retry_cnt;  // 重试计数器
  uint8_t timeout_100ms_sp;  // 超时设定值
  uint8_t timeout_100ms;  // 超时计时器

  uint8_t result;  // 下包结果0=成功,1=失败,2=控制器升级成功,3=控制器拒绝升级,4=控制器升级失败

  // 发送
  uint16_t tx_size;  // 发送数据长度
  uint8_t* tx_data;  // 发送数据地址

  // 接收
  uint16_t rx_size;  // 接收数据长度
  uint8_t* rx_data;  // 接收数据地址
}fsrv_context_t;
extern fsrv_context_t fsrv_context;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif

