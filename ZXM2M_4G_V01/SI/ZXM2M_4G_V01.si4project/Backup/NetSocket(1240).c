/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: NetSocket.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-12-29
 * @brief:
 *******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/



/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/


/******************************************************************************
 * socket״̬��
 ******************************************************************************/
void Net_SocketInvalid(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("InvSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->timer_10ms = 100; // 1��
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if ((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4Gģ�������Ѿ���
      {
        pThis->state = NET_SOCKET_STATE_CREATE;
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void Net_SocketCreate(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CrtSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    // ʹ���ֶ�����socket
    (*pThis->reinit_server_addr)(); // ���»�ȡ��������ַ�Ͷ˿�

    pThis->timer_10ms = 100; // 1��
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->state = NET_SOCKET_STATE_CONNECT;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketConnect(skt_context_t* pThis)
{
  int retval = 0;
  int8_t sbuff[50] = {NULL};

  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("ConSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CONNECTING;

    retval = Modem_SocketConnect(pThis);  // ����,�����������Ч,ʱ����2����
    if (retval == 1) // ����ʧ��
    {
#if SOCKET_DEBUG
      PcDebug_Printf("ConSktErr:Id=%d\n",pThis->conId);
#endif
      pThis->error_cnt++;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }
    sprintf((char *)sbuff, "SktId=%d IP-%s:%d\n",pThis->conId,pThis->srv_ip,pThis->srv_port); // ��ʾ����id����������ַ�Ͷ˿�
    PcDebug_Printf((const char *)sbuff);

    pThis->timer_10ms = 50; // 0.5��
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->state = NET_SOCKET_STATE_READY;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketReady(skt_context_t* pThis)
{
  if (pThis->state_transition)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("RdySkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->error_cnt = 0x00; // ����������
    //pThis->error_flag = FALSE;
    pThis->link_state = SOCKET_LINK_STATE_READY;
  }
  else
  {
    // 4Gģ���ж�
    if (Modem_GetState()!= MODEM_STATE_DATA_READY) // 4Gģ�鴦�ڳ�ʼ��
    {
      pThis->link_state = SOCKET_LINK_STATE_CLOSED;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }

    if (pThis->link_state == SOCKET_LINK_STATE_CLOSED)
    {
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }

    // ��������
    Modem_SocketSend(pThis);

    if (pThis->error_cnt > 3) // ����3�ζ�ʧ��
    {
      pThis->state = NET_SOCKET_STATE_CLOSE; // ����
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void Net_SocketClose(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CloSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CLOSED;

    Modem_SocketClose(pThis); // �����ر�,��֤ͨ��˫�����ܹ��յ�Ӧ�ó��򷢳�����������
    pThis->timer_10ms = 500;  // 5��
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if (pThis->error_cnt > 3) // ����3�ζ�ʧ��,��ͣ1���Ӻ�����
      {
        pThis->state = NET_SOCKET_STATE_SILENCE; // ����
        pThis->state_transition = SOCKET_TRUE;
      }
      else
      {
        pThis->state = NET_SOCKET_STATE_INVALID; // ����
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void Net_SocketSilence(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("SilSkt:Id=%d\n",pThis->conId);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->error_flag = SOCKET_TRUE;
    pThis->timer_10ms = 12000; // 2����
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->error_cnt = 0x00; // ����������
      pThis->error_flag = SOCKET_FALSE;

      pThis->state = NET_SOCKET_STATE_INVALID; // ����
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

/******************************************************************************
* �˺�������ά��Socket����,ÿ10ms����һ��(������)
*******************************************************************************/
void Net_SocketManageService(skt_context_t* pThis)
{
  switch (pThis->state)
  {
  case NET_SOCKET_STATE_INVALID: // �ȴ�4G����READY
    Net_SocketInvalid(pThis);
    break;

  case NET_SOCKET_STATE_CREATE: // ����SOCKETͨ�Ŷ˵�
    Net_SocketCreate(pThis);
    break;

  case NET_SOCKET_STATE_CONNECT: // ���ӷ�����
    Net_SocketConnect(pThis);
    break;

  case NET_SOCKET_STATE_READY: // �����ӿ�ͨ��
    Net_SocketReady(pThis);
    break;

  case NET_SOCKET_STATE_CLOSE: // �ر�SOCKET
    Net_SocketClose(pThis);
    break;

  case NET_SOCKET_STATE_SILENCE: // ��Ĭһ��ʱ��
    Net_SocketSilence(pThis);
    break;

  default:
    pThis->state = NET_SOCKET_STATE_CLOSE;
    pThis->state_transition = TRUE;
    break;
  }

  if (pThis->timer_10ms)
    pThis->timer_10ms--;
}


/******************************************************************************
 * ����: CAT1ģ���������ݷ��ͽӿ�
 * ����: pucData=����������ָ��; usLen=���������ݳ���
 * ����: 0 :�ɹ����� -1:����ʧ��
*******************************************************************************/
int32_t net_socket_send(skt_context_t* pThis, uint8_t *pData, uint16_t len)
{
  uint8_t it;

  if ((Modem_GetState()!=MODEM_STATE_DATA_READY))
  {
    return -1;
  }

  if ((pData==NULL) || (len==0) || (len > SOCKET_CONN_MAX_DATA_LEN))
  {
#if SOCKET_DEBUG
    PC_DebugPrintf("SktSendParaErr:Id=%d\n",pThis->conId);
#endif
    return -1;
  }

  for (it=0; it<200; it++)   //���������æ,���ȴ�2s
  {
    if (pThis->link_state==SOCKET_LINK_STATE_READY)
    {
      break;
    }
    CELLURA_DELAY(OS_TICKS_PER_SEC/100);  // �ȴ�10ms
  }

  if (pThis->link_state==SOCKET_LINK_STATE_READY)  // ����״̬
  {
    memcpy(pThis->tx_buff, pData, len); // ��������
    pThis->tx_len = len; // ���ݳ���
    pThis->new_tx_data_flag = 1;  // ��������Ҫ����
    pThis->link_state = SOCKET_LINK_STATE_BUSY; // ����æµ

    return 0;
  }
  else
  {
    PcDebug_Printf("SktSendBusy:Id=%d\n",pThis->conId);
    return -1;
  }
}

/******************************************************************************
*
*******************************************************************************/
uint8_t net_socket_GetLinkState(skt_context_t* pThis)
{
  return pThis->link_state;
}

void net_socket_SetLinkState(skt_context_t* pThis,uint8_t link_state)
{
  pThis->link_state = link_state;
}

/******************************************************************************
* ���溯�����û���д
*******************************************************************************/
//=========================================================================
void Net_SocketInit(void)
{
  M2M_NetSocketInit();  // M2M���ݳ�ʼ��
  HJEP_NetSocketInit(); // �����������ݳ�ʼ��
}

//int32_t net_socket_send(skt_context_t* pThis, uint8_t *pData, uint16_t len);

//=========================================================================
void net_socket_recv(uint8_t conId, uint8_t* pData, uint16_t recv_size)
{
  if (recv_size > 0)
  {
    SbusMsg_Gprs.data_size = recv_size;
    SbusMsg_Gprs.data = pData;
    SbusMsg_Gprs.resv = conId;
    SYSBUS_PutMbox(SbusMsg_Gprs);
  }
}

//=========================================================================
void net_socket_close(uint8_t conId)
{
  if (SOCKET_LINK_M2M == conId)
  {
    net_socket_SetLinkState(&m2m_socket, SOCKET_LINK_STATE_CLOSED);
  }
  else if (SOCKET_LINK_HJ == conId)
  {
    net_socket_SetLinkState(&hjep_socket, SOCKET_LINK_STATE_CLOSED);
  }
  else if (SOCKET_LINK_GB == conId)
  {
    net_socket_SetLinkState(&gbep_socket, SOCKET_LINK_STATE_CLOSED);
  }
}

//=========================================================================
uint8_t net_IsAllSocketClosed(void)
{
  if ((hjep_socket.state==NET_SOCKET_STATE_INVALID) && (m2m_socket.state==NET_SOCKET_STATE_INVALID))
  {
    return SOCKET_TRUE;
  }
  else
  {
    return SOCKET_FALSE;
  }
}

// ��λsocket
void net_socket_reset(void)
{
  if (gGsm_ZxSocketFd >= 0)
  {
    shutdown(gGsm_ZxSocketFd,SHUT_RDWR);
    sleep(1);
    close(gGsm_ZxSocketFd);
    gGsm_ZxSocketFd = -1;
  }

  if (gGsm_ZxepSocketFd >= 0)
  {
    shutdown(gGsm_ZxepSocketFd,SHUT_RDWR);
    sleep(1);
    close(gGsm_ZxepSocketFd);
    gGsm_ZxepSocketFd = -1;
  }

  sleep(1);
}


//==�������===============================================================
void Net_StateManageService(void)
{
  Net_SocketManageService(&m2m_socket);  // m2m���ӹ���
  Net_SocketManageService(&hjep_socket);  // ep���ӹ���
  //Net_SocketManageService(&gbep_socket);  // ep���ӹ���
}

