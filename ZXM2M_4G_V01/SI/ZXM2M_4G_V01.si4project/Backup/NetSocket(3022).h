/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: NetSocket.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-12-29
 * @brief     ���ļ�Ϊ4G������ͷ�ļ�
******************************************************************************/
#ifndef _NET_SOCKET_H_
#define _NET_SOCKET_H_

//-----ͷ�ļ�����-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// BOOL����
enum 
{
  SOCKET_FALSE = 0x00,
  SOCKET_TRUE = 0x01
};
  
// �׽���״̬����
typedef enum
{
  NET_SOCKET_STATE_INVALID = 0,
  NET_SOCKET_STATE_CREATE,
  NET_SOCKET_STATE_CONNECT,
  NET_SOCKET_STATE_READY,
  NET_SOCKET_STATE_CLOSE,
  NET_SOCKET_STATE_SILENCE
} net_socket_state_t;

// ״̬����
typedef enum
{
  SOCKET_LINK_STATE_CLOSED = 0,
  SOCKET_LINK_STATE_CONNECTING,
  SOCKET_LINK_STATE_READY,
  SOCKET_LINK_STATE_BUSY
} socket_link_state_t;

#define MAX_IP_ADDR_SIZE   20  // ֻ֧��IPv4
// socket���ݶ�����
typedef struct
{
  uint8_t enable_flag;  // ʹ�����ӱ�־λ
  uint8_t link_state;   // ����״̬

  uint8_t conId;        // ����id

  uint8_t state;            // ״̬
  uint8_t started_delay;    // ״̬��ʼ����־
  uint8_t state_transition; // ״̬ת�Ʊ�־
  uint32_t timer_10ms;      // ������10ms��ʱ��

  uint8_t error_cnt;  // ����ʧ�ܼ�����
  uint8_t error_flag; // �������ʧ�ܱ�־

  uint8_t srv_protocol;  // Э������
  uint16_t srv_port; // �������˿�:10012
  int8_t srv_ip[MAX_IP_ADDR_SIZE]; //  ������IP:"120.195.166.245"

  uint8_t* tx_buff;   //!< Pointer to buffer data array
  uint16_t tx_len;    //!< Length of buffer array
  uint8_t new_tx_data_flag;  // ����״̬

  void (*reinit_server_addr)(void);
}skt_context_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/

#endif  /* _NET_SOCKET_H_ */

