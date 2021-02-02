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
  MAINEP_SOCKET_ID,    // 环保主连接
  SUBEP_SOCKET_ID,     // 环保副连接
  NUMBER_OF_SOCKET_ID
};

extern skt_context_t skt_context[NUMBER_OF_SOCKET_ID];
#define zxm2m_socket  skt_context[ZXM2M_SOCKET_ID] // ZXM2M数据
#define ZxM2mSocketFd  zxm2m_socket.socket_fd

#define mainep_socket   skt_context[MAINEP_SOCKET_ID]  // 环保数据(主)
#define MainEpSocketFd   mainep_socket.socket_fd

#define subep_socket   skt_context[SUBEP_SOCKET_ID]  // 环保数据(副)
#define SubEpSocketFd   subep_socket.socket_fd

//==HJ环保协议(重型HJ平台,转发国家平台)============
#define hjep_socket   skt_context[MAINEP_SOCKET_ID]  // HJ环保数据
#define HJEP_SOCKET_ID  MAINEP_SOCKET_ID // HJ连接
#define HjepSocketFd   mainep_socket.socket_fd

//==GB环保协议(北京环保平台(直连)和重型GB平台)====
#define bjep_socket   skt_context[MAINEP_SOCKET_ID]  // 北京环保数据(用主连接)
#define BJEP_SOCKET_ID  MAINEP_SOCKET_ID // HJ连接
#define BjepSocketFd   mainep_socket.socket_fd

#define gbep_socket   skt_context[SUBEP_SOCKET_ID]  // GB环保数据(用副连接)
#define GBEP_SOCKET_ID  SUBEP_SOCKET_ID // GB连接
#define GbepSocketFd   subep_socket.socket_fd

//==发送周期定义====================================
#define HJEP_DATA_SEND_TIME     10   //发送周期10秒
#define HJEP_HEART_BEAT_TIME    120  //发送周期2分钟

#define ZXM2M_DATA_SEND_TIME    30   //发送周期30秒
#define ZXM2M_HEART_BEAT_TIME   120  //发送周期2分钟

#define GBEP_DATA_SEND_TIME     10   //发送周期10秒
#define GBEP_HEART_BEAT_TIME    120  //发送周期2分钟

#define ZXM2M_HEART_BEAT_TIMEOUT_SP  (3*ZXM2M_HEART_BEAT_TIME*100)   // for 10ms time base
#define HJEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base
#define GBEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base

#define M2M_CLOUD_SERVER_IP         "58.218.196.200" // ZXM2M平台
#define M2M_CLOUD_SERVER_PORT       10004
#define M2M_CLOUD_SERVER_PROTOCOL   CS_TCP_PROTOCOL

#define HJEP_CLOUD_SERVER_IP        "120.195.166.245"  // HJ环保平台
#define HJEP_CLOUD_SERVER_PORT      10012
#define HJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define GBEP_CLOUD_SERVER_IP    "120.195.166.245"  // GB内部平台
#define GBEP_CLOUD_SERVER_PORT  10012
#define GBEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define BJEP_CLOUD_SERVER_DNS   "jc.bjmemc.com.cn"  // 北京环保平台
#define BJEP_CLOUD_SERVER_PORT  7740
#define BJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

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

// 环保平台连接
void EP_ServiceInit(void);
void EP_ServiceStart(void);

void Net_CheckIsModemError(void);

#endif /* _ICLOUD_MACHINE_H_ */

