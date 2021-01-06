/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraModule.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-12-31
 * @brief     
******************************************************************************/
#ifndef _CELLURA_MODULE_H_
#define _CELLURA_MODULE_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"
#include "Cellura.h"
#include "NetSocket.h"

#define CELLURA_MAX_CONNS          6
#define SOCKET_CONN_MAX_DATA_LEN   1460


/******************************************************************************
 *   Macros
 ******************************************************************************/
// MODEM parameters
#define MODEM_UART_BAUDRATE (B115200)
#define MODEM_MAX_SOCKET_TX_DATA_SIZE ((uint32_t)1460U)  // cf AT+QISEND
#define MODEM_MAX_SOCKET_RX_DATA_SIZE ((uint32_t)1500U)  // cf AT+QIRD

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
enum
{
  SOCKET_LINK_M2M = 0, // M2M链接
  SOCKET_LINK_HJ,      // HJ连接
  SOCKET_LINK_GB,      // GB连接
  NUMBER_OF_SOCKET_LINKS
};

extern skt_context_t skt_context[NUMBER_OF_SOCKET_LINKS];
#define m2m_socket    skt_context[SOCKET_LINK_M2M] // M2M数据
#define hjep_socket   skt_context[SOCKET_LINK_HJ]  // HJ环保数据
#define gbep_socket   skt_context[SOCKET_LINK_GB]  // GB环保数据

#define HJEP_DATA_SEND_TIME     10   //发送周期10秒
#define HJEP_HEART_BEAT_TIME    120  //发送周期2分钟

#define M2M_DATA_SEND_TIME      60   //发送周期1分钟
#define M2M_HEART_BEAT_TIME     60  //发送周期1分钟

#define M2M_HEART_BEAT_TIMEOUT_SP    (3*M2M_HEART_BEAT_TIME*100)   // for 10ms time base
#define HJEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base

#define HJEP_CLOUD_SERVER_IP        "120.195.166.245"
#define HJEP_CLOUD_SERVER_PORT      10012
#define HJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define M2M_CLOUD_SERVER_IP         "58.218.196.200"
#define M2M_CLOUD_SERVER_PORT       10004
#define M2M_CLOUD_SERVER_PROTOCOL   CS_TCP_PROTOCOL

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Cellura_ServiceInit(void);
void Cellura_ServiceStart(void);

// Cellura信息函数接口
uint8_t *Cellura_GetIccid(void);
uint8_t Cellura_GetCsq(void);
uint32_t Cellura_GetLacCellID(void);
uint8_t Cellura_GetModemStatus(void);
uint8_t Cellura_GetSimCardState(void);
uint8_t Cellura_GetCsRegistState(void);
uint8_t Cellura_GetPsRegistState(void);

#endif  /* _CELLURA_MODULE_H_ */

