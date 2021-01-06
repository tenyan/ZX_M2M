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

//-----ͷ�ļ�����--------------------------------------------------------------------
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
  int32_t socket_fd;    // �ļ���ʶ��

  uint8_t enable_flag;  // ʹ�����ӱ�־λ
  uint8_t link_state;   // ����״̬

  uint8_t socket_id;    // ����id

  uint8_t state;            // ״̬
  uint8_t started_delay;    // ״̬��ʼ����־
  uint8_t state_transition; // ״̬ת�Ʊ�־
  uint32_t socket_10ms_timer;// ������10ms��ʱ��
  uint32_t hb_10ms_timer;  // ������ʱ��(����10ms)
  uint32_t hb_10ms_timer_sp; // ��������ֵ(����10ms)

  uint8_t error_cnt;  // ����ʧ�ܼ�����
  uint8_t error_flag; // �������ʧ�ܱ�־

  uint8_t srv_protocol;  // Э������
  uint16_t srv_port; // �������˿�:10012
  int8_t srv_ip[MAX_IP_ADDR_SIZE]; //  ������IP:"120.195.166.245"

  uint8_t* tx_buff;   //!< Pointer to buffer data array
  uint16_t tx_len;    //!< Length of buffer array
  uint8_t new_tx_data_flag;  // ����״̬

  void (*reinit_server_addr)(void);
  pthread_mutex_t send_mutex;
}skt_context_t;

extern skt_context_t skt_context[NUMBER_OF_LINK];
#define zx_socket    skt_context[0] // ��������
#define zxep_socket  skt_context[1] // ���ͻ�������

#define gGsm_ZxSocketFd    zx_socket.socket_fd
#define gGsm_ZxepSocketFd  zxep_socket.socket_fd

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
int32_t NetSocket_Send(skt_context_t* pThis, uint8_t *pData, uint16_t len);
uint8_t NetSocket_GetLinkState(skt_context_t* pThis);
void NetSocket_SetLinkState(skt_context_t* pThis,uint8_t link_state);

#endif  /* _NET_SOCKET_H_ */

