/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: NetSocket.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-12-29
 * @brief     本文件为4G网络层的头文件
******************************************************************************/
#ifndef _NET_SOCKET_H_
#define _NET_SOCKET_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// BOOL定义
enum 
{
  SOCKET_FALSE = 0x00,
  SOCKET_TRUE = 0x01
};
  
// 套接字状态定义
typedef enum
{
  NET_SOCKET_STATE_INVALID = 0,
  NET_SOCKET_STATE_CREATE,
  NET_SOCKET_STATE_CONNECT,
  NET_SOCKET_STATE_READY,
  NET_SOCKET_STATE_CLOSE,
  NET_SOCKET_STATE_SILENCE
} net_socket_state_t;

// 状态定义
typedef enum
{
  SOCKET_LINK_STATE_CLOSED = 0,
  SOCKET_LINK_STATE_CONNECTING,
  SOCKET_LINK_STATE_READY,
  SOCKET_LINK_STATE_BUSY
} socket_link_state_t;

#define MAX_IP_ADDR_SIZE   20  // 只支持IPv4
// socket数据对象定义
typedef struct
{
  uint8_t enable_flag;  // 使能连接标志位
  uint8_t link_state;   // 连接状态

  uint8_t conId;        // 连接id

  uint8_t state;            // 状态
  uint8_t started_delay;    // 状态初始化标志
  uint8_t state_transition; // 状态转移标志
  uint32_t timer_10ms;      // 无阻塞10ms定时器

  uint8_t error_cnt;  // 连接失败计数器
  uint8_t error_flag; // 多次连接失败标志

  uint8_t srv_protocol;  // 协议类型
  uint16_t srv_port; // 服务器端口:10012
  int8_t srv_ip[MAX_IP_ADDR_SIZE]; //  服务器IP:"120.195.166.245"

  uint8_t* tx_buff;   //!< Pointer to buffer data array
  uint16_t tx_len;    //!< Length of buffer array
  uint8_t new_tx_data_flag;  // 发送状态

  void (*reinit_server_addr)(void);
}skt_context_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/

#endif  /* _NET_SOCKET_H_ */

