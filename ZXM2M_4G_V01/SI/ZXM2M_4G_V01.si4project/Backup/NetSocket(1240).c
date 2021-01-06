/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: NetSocket.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-12-29
 * @brief:
 *******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/



/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/


/******************************************************************************
 * socket状态机
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
    pThis->timer_10ms = 100; // 1秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if ((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4G模块网络已就绪
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

    // 使用手动分配socket
    (*pThis->reinit_server_addr)(); // 重新获取服务器地址和端口

    pThis->timer_10ms = 100; // 1秒
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

    retval = Modem_SocketConnect(pThis);  // 阻塞,如果服务器无效,时间大概2分钟
    if (retval == 1) // 连接失败
    {
#if SOCKET_DEBUG
      PcDebug_Printf("ConSktErr:Id=%d\n",pThis->conId);
#endif
      pThis->error_cnt++;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }
    sprintf((char *)sbuff, "SktId=%d IP-%s:%d\n",pThis->conId,pThis->srv_ip,pThis->srv_port); // 显示链接id、服务器地址和端口
    PcDebug_Printf((const char *)sbuff);

    pThis->timer_10ms = 50; // 0.5秒
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
    pThis->error_cnt = 0x00; // 清除错误计数
    //pThis->error_flag = FALSE;
    pThis->link_state = SOCKET_LINK_STATE_READY;
  }
  else
  {
    // 4G模块判断
    if (Modem_GetState()!= MODEM_STATE_DATA_READY) // 4G模块处于初始化
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

    // 发送数据
    Modem_SocketSend(pThis);

    if (pThis->error_cnt > 3) // 发送3次都失败
    {
      pThis->state = NET_SOCKET_STATE_CLOSE; // 重连
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

    Modem_SocketClose(pThis); // 正常关闭,保证通信双方都能够收到应用程序发出的所有数据
    pThis->timer_10ms = 500;  // 5秒
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      if (pThis->error_cnt > 3) // 连接3次都失败,暂停1分钟后再连
      {
        pThis->state = NET_SOCKET_STATE_SILENCE; // 重连
        pThis->state_transition = SOCKET_TRUE;
      }
      else
      {
        pThis->state = NET_SOCKET_STATE_INVALID; // 重连
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
    pThis->timer_10ms = 12000; // 2分钟
  }
  else
  {
    if (pThis->timer_10ms == 0)
    {
      pThis->error_cnt = 0x00; // 清除错误计数
      pThis->error_flag = SOCKET_FALSE;

      pThis->state = NET_SOCKET_STATE_INVALID; // 重连
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

/******************************************************************************
* 此函数负责维护Socket链接,每10ms调用一次(无阻塞)
*******************************************************************************/
void Net_SocketManageService(skt_context_t* pThis)
{
  switch (pThis->state)
  {
  case NET_SOCKET_STATE_INVALID: // 等待4G网络READY
    Net_SocketInvalid(pThis);
    break;

  case NET_SOCKET_STATE_CREATE: // 创建SOCKET通信端点
    Net_SocketCreate(pThis);
    break;

  case NET_SOCKET_STATE_CONNECT: // 连接服务器
    Net_SocketConnect(pThis);
    break;

  case NET_SOCKET_STATE_READY: // 已连接可通信
    Net_SocketReady(pThis);
    break;

  case NET_SOCKET_STATE_CLOSE: // 关闭SOCKET
    Net_SocketClose(pThis);
    break;

  case NET_SOCKET_STATE_SILENCE: // 静默一段时间
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
 * 功能: CAT1模块网络数据发送接口
 * 输入: pucData=待发送数据指针; usLen=待发送数据长度
 * 返回: 0 :成功返回 -1:发送失败
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

  for (it=0; it<200; it++)   //如果该链接忙,最多等待2s
  {
    if (pThis->link_state==SOCKET_LINK_STATE_READY)
    {
      break;
    }
    CELLURA_DELAY(OS_TICKS_PER_SEC/100);  // 等待10ms
  }

  if (pThis->link_state==SOCKET_LINK_STATE_READY)  // 在线状态
  {
    memcpy(pThis->tx_buff, pData, len); // 拷贝数据
    pThis->tx_len = len; // 数据长度
    pThis->new_tx_data_flag = 1;  // 有新数据要发送
    pThis->link_state = SOCKET_LINK_STATE_BUSY; // 连接忙碌

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
* 下面函数由用户编写
*******************************************************************************/
//=========================================================================
void Net_SocketInit(void)
{
  M2M_NetSocketInit();  // M2M数据初始化
  HJEP_NetSocketInit(); // 国六环保数据初始化
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

// 复位socket
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


//==网络管理===============================================================
void Net_StateManageService(void)
{
  Net_SocketManageService(&m2m_socket);  // m2m链接管理
  Net_SocketManageService(&hjep_socket);  // ep链接管理
  //Net_SocketManageService(&gbep_socket);  // ep链接管理
}

