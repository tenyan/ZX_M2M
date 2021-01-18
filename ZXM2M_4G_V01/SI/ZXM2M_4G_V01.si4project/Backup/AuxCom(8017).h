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

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// 升级状态
typedef enum
{
  FSRV_STATE_IDLE = 0x00,  /// 空闲
  FSRV_STATE_SEND_UN,      /// 发送通知升级
  FSRV_STATE_WAIT_REQ,     /// 等待ST请求和响应
  FSRV_STATE_SEND_UD,      /// 发送升级数据(响应ST请求)
  FSRV_STATE_REPORT_REQ,   /// 向服务器上报升级结果
}fsrv_state_t;

#define FSRV_FIRMWARE_PACKET_LEN    1024//固件升级包的长度
// 固件升级结构体
typedef struct 
{
  uint8_t state;  // 升级状态
  uint8_t dev_id;  // 被升级设备号
  uint8_t msg_sn;  // 流水号
  uint16_t fw_packet_index;  // 升级包序号
  uint16_t fw_packet_size;  // 包大小(固定值)

  uint32_t start_address;  // 已接收长度
  uint32_t ending_address;  // 总长度

  uint16_t total_block_count;  // 程序总块数
  uint8_t percent;             // 升级进度
  
  uint8_t sent_flag;  // 请求已发出
  uint8_t rsp_timeout_sp;  // 超时设定值
  uint8_t rsp_timer;  // 超时设定值
  uint8_t retry_cnt;  // 重试计数器
  uint16_t timeout_cnt;  // 升级超时计数器,如果10分钟内没有升级完成则放弃
  uint8_t result;  // 下包结果0=成功,1=失败

  // 发送
  uint16_t tx_size;  // 发送数据长度
  uint8_t* tx_data;  // 发送数据地址

  // 接收
  uint16_t rx_size;  // 接收数据长度
  uint8_t* rx_data;  // 接收数据地址

}fsrv_context_t;
extern fsrv_context_t fsrv_context;

typedef struct _ExTMCUUpgrade
{
    uint8 ucUpgradeStep;     //升级文件下载步骤  0-无升级进程;1-通知ExtMcu升级;2-发送升级包数据
                             //3-升级结果应答,0xFF-中间步骤(不希望重发?)
    uint16 ucTimer;           //运行定时器
    uint16 usRequestPacketSn; //请求升级包序号
    uint8 ucDev;             //升级设备编号
    uint8 ucResult;          //设备升级结果
    uint16 usTimeOut;        //升级文件下载总超时时间

}STUExtMCUUpgrade;


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif

