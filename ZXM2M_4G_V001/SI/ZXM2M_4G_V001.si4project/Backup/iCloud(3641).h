/*****************************************************************************
* @FileName: iCloud.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-6
* @brief
******************************************************************************/
#ifndef _ICLOUD_H_
#define _ICLOUD_H_

#include "NetSocket.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
// 连接定义
enum
{
  ZXM2M_SOCKET_ID = 0, // ZXM2M链接
  HJEP_SOCKET_ID,        // HJ连接
  //GBEP_SOCKET_ID,      // GB连接
  NUMBER_OF_SOCKET_ID
};

extern skt_context_t skt_context[NUMBER_OF_SOCKET_ID];
#define zxm2m_socket  skt_context[ZXM2M_SOCKET_ID] // ZXM2M数据
#define hjep_socket   skt_context[HJEP_SOCKET_ID]  // HJ环保数据
//#define gbep_socket   skt_context[SOCKET_LINK_GB]  // GB环保数据

#define ZxM2mSocketFd  zxm2m_socket.socket_fd
#define HjepSocketFd   hjep_socket.socket_fd
//#define GbepSocketFd   gbep_socket.socket_fd

#define HJEP_DATA_SEND_TIME     10   //发送周期10秒
#define HJEP_HEART_BEAT_TIME    120  //发送周期2分钟

#define ZXM2M_DATA_SEND_TIME      60   //发送周期1分钟
#define ZXM2M_HEART_BEAT_TIME     60  //发送周期1分钟

#define ZXM2M_HEART_BEAT_TIMEOUT_SP  (3*ZXM2M_HEART_BEAT_TIME*100)   // for 10ms time base
#define HJEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base

#define HJEP_CLOUD_SERVER_IP        "120.195.166.245"
#define HJEP_CLOUD_SERVER_PORT      10012
#define HJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define M2M_CLOUD_SERVER_IP         "58.218.196.200"
#define M2M_CLOUD_SERVER_PORT       10004
#define M2M_CLOUD_SERVER_PROTOCOL   CS_TCP_PROTOCOL

#if 0
//M2M_NetSocketInit();  // M2M数据初始化
//HJEP_NetSocketInit(); // 国六环保数据初始化
#endif

/******************************************************************************
 *   Data Types
 ******************************************************************************/
typedef enum
{
  ICLOUD_FALSE = 0,
  ICLOUD_TRUE  = 1
} icloud_bool_t;
  
typedef enum
{
  ALARM_FALSE = 0, // 消失(NORMAL)
  ALARM_TRUE = 1   // 产生(FAIl)
} alarm_bool_t;

// 总线设备类型定义
enum
{
  SYSBUS_DEVICE_TYPE_CELLURA = 0x01, // 蜂窝网模块
  SYSBUS_DEVICE_TYPE_CAN,     // CAN模块
  SYSBUS_DEVICE_TYPE_GPS,     // GPS模块
  SYSBUS_DEVICE_TYPE_COLLECT, // 采集模块
  SYSBUS_DEVICE_TYPE_PCDEBUG,  // 配置工具
};

//extern uint8_t public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
// ZxM2m平台连接
void ZxM2m_ServiceInit(void);
void ZxM2m_ServiceStart(void);

// HJ环保平台连接
void HJEP_ServiceInit(void);
void HJEP_ServiceStart(void);

void Net_CheckIsModemError(void);

#endif /* _ICLOUD_MACHINE_H_ */

