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
* SIGPIPE信号处理函数
* 当服务器close一个连接时(服务器故障),若client继续向服务器发数据,
* 根据TCP协议的规定,客户端会收到一个RST响应,client再往这个服务器发送数据时,
* 系统会发出一个SIGPIPE信号给客户端进程,导致客户端进程退出(结束main进程)。
* 如果使用了多个套接字,该信号的递交无法告诉我们是哪个套接字出的错。
* 如果我们确实需要知道是哪个write出了错,那么必须要么不理会该信号,
* 要么从信号处理函数返回后再处理来自write的EPIPE.
*******************************************************************************/
static void NetSocket_SigpipeHandle(int sig)
{
#if SOCKET_DEBUG
  PcDebug_Printf("Socket SIGPIPE.\r\n");
#endif
}

//==信号处理函数防止僵尸进程===================================================
void NetSocket_sigpipe(void)
{
  struct sigaction sa;

  //sa.sa_handler = SIG_IGN;
  sa.sa_handler = NetSocket_SigpipeHandle;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGPIPE, &sa, NULL);
}

//=============================================================================
void NetSocket_Init(void)
{
  NetSocket_sigpipe();
}

//==============================================================================
void NetSocket_Invalid(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("InvSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    close(pThis->socket_fd);
    pThis->socket_fd = -1;

    pThis->socket_10ms_timer = 100; // 1秒
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      if ((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4G模块网络已就绪
      {
        pThis->state = NET_SOCKET_STATE_CREATE;
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================
void NetSocket_Create(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CrtSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    // sock_fd = socket(AF_INET, SOCK_DGRAM, 0); // AF_INET表示IPv4,SOCK_DGRAM表示UDP
    pThis->socket_fd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET表示IPv4,SOCK_STREAM表示TCP
    if (pThis->socket_fd < 0) // 创建失败
    {
#if SOCKET_DEBUG
      PcDebug_Printf("CrtSktErr:Id=%d\r\n",pThis->socket_id);
#endif
      pThis->error_cnt++;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = TRUE;
      return;
    }

#if SOCKET_DEBUG
    PcDebug_Printf("CrtSkt:Id=%d,Fd=%d\r\n",pThis->socket_id,pThis->socket_fd);
#endif

    pThis->socket_10ms_timer = 200; // 2秒
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      pThis->state = NET_SOCKET_STATE_CONNECT;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================
void NetSocket_Connect(skt_context_t* pThis)
{
  int retval = 0;
  struct sockaddr_in server_addr;
  int8_t sbuff[50] = {0};

  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("ConSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CONNECTING;
    (*pThis->reinit_server_addr)(); // 重新获取服务器地址和端口
    memset(&server_addr,0,sizeof(server_addr));
    server_addr.sin_family = AF_INET;  // IPv4协议
    server_addr.sin_port = htons(pThis->srv_port);
    server_addr.sin_addr.s_addr = inet_addr((const char *)pThis->srv_ip);
    retval = connect(pThis->socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr)); // 阻塞,如果服务器无效,时间大概2分钟
    if (retval < 0)
    {
#if SOCKET_DEBUG
      PcDebug_Printf("ConSktErr:Id=%d\r\n",pThis->socket_id);
      //printf("Connect error %s errno: %d\n",strerror(errno),errno);
#endif
      pThis->error_cnt++;
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }
    sprintf((char *__restrict)sbuff, "SktId=%d IP-%s:%d\r\n",pThis->socket_id,pThis->srv_ip,pThis->srv_port);
    PcDebug_Printf((const char *)sbuff);

    pThis->socket_10ms_timer = 100; // 1秒
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      pThis->state = NET_SOCKET_STATE_READY;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================
void NetSocket_Ready(skt_context_t* pThis)
{
  if (pThis->state_transition)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("RdySkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    //pThis->error_cnt = 0x00; // 清除错误计数
    //pThis->error_flag = FALSE;
    pThis->hb_10ms_timer = pThis->hb_10ms_timer_sp;
    pThis->link_state = SOCKET_LINK_STATE_READY;
  }
  else
  {
    if (pThis->link_state == SOCKET_LINK_STATE_CLOSED)
    {
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
    }

    if (pThis->hb_10ms_timer == 0x00) // 超时判断
    {
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
    }

    if (pThis->error_cnt > 3) // 发送3次都失败
    {
      pThis->state = NET_SOCKET_STATE_CLOSE; // 重连
      pThis->state_transition = SOCKET_TRUE;
    }
  }

  // 4G模块判断
  if (Modem_GetState()!=MODEM_STATE_DATA_READY) // 4G模块处于初始化
  {
    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->state = NET_SOCKET_STATE_CLOSE;
    pThis->state_transition = SOCKET_TRUE;
  }
}

//============================================================================
void NetSocket_Close(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("CloSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    shutdown(pThis->socket_fd,SHUT_RDWR); // 正常关闭,保证通信双方都能够收到应用程序发出的所有数据
    pThis->socket_fd = -1;
    pThis->socket_10ms_timer = 800; // 8秒
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      if (pThis->error_cnt > 3) // 连接5次都失败,暂停1分钟后再连
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

//==========================================================================
void NetSocket_Silence(skt_context_t* pThis)
{
  if (pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PcDebug_Printf("SilSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if (pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = SOCKET_LINK_STATE_CLOSED;
    pThis->error_flag = SOCKET_TRUE;
    pThis->socket_10ms_timer = 12000; // 2分钟
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
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
void NetSocket_Service(skt_context_t* pThis)
{
  switch (pThis->state)
  {
  case NET_SOCKET_STATE_INVALID: // 等待4G网络READY
    NetSocket_Invalid(pThis);
    break;

  case NET_SOCKET_STATE_CREATE: // 创建SOCKET通信端点
    NetSocket_Create(pThis);
    break;

  case NET_SOCKET_STATE_CONNECT: // 连接服务器
    NetSocket_Connect(pThis);
    break;

  case NET_SOCKET_STATE_READY: // 已连接可通信
    NetSocket_Ready(pThis);
    break;

  case NET_SOCKET_STATE_CLOSE: // 关闭SOCKET
    NetSocket_Close(pThis);
    break;

  case NET_SOCKET_STATE_SILENCE: // 静默一段时间
    NetSocket_Silence(pThis);
    break;

  default:
    pThis->state = NET_SOCKET_STATE_CLOSE;
    pThis->state_transition = TRUE;
    break;
  }

  if (pThis->socket_10ms_timer)
    pThis->socket_10ms_timer--;

  if (pThis->hb_10ms_timer)
    pThis->hb_10ms_timer--;
}

/******************************************************************************
 * 功能: GSM模块GPRS数据发送接口
 * 输入: pucData=待发送数据指针; usLen=待发送数据长度
 * 返回: 0 :成功返回 -1:发送失败
*******************************************************************************/
int32_t NetSocket_Send(skt_context_t* pThis, uint8_t *pData, uint16_t len)
{
  int sent_bytes;

  if ((Modem_GetState()!=MODEM_STATE_DATA_READY))
  {
    return -1;
  }

  if ((pData==NULL) || (len==0) || (len > NET_SOCKET_MAX_DATA_LEN))
  {
#if SOCKET_DEBUG
    PcDebug_Printf("SktSendParaErr:Id=%d\r\n",pThis->socket_id);
#endif
    return -1;
  }

  if (pThis->link_state==SOCKET_LINK_STATE_READY)  // 在线状态
  {
    pthread_mutex_lock(&pThis->send_mutex);
    sent_bytes = send(pThis->socket_fd, pData, len, 0);
    pthread_mutex_unlock(&pThis->send_mutex);

    if (sent_bytes < 0)
    {
#if SOCKET_DEBUG
      PcDebug_Printf("SktSendErr:Id=%d\r\n",pThis->socket_id);
#endif
      pThis->error_cnt++;
      //pThis->link_state = SOCKET_LINK_STATE_CLOSED;
      return -1;
    }

    PcDebug_SendData(pData, len, DBG_MSG_TYPE_NET);

    return 0;
  }

  return -1;
}

/***************************************************************************
*
****************************************************************************/
uint8_t NetSocket_GetLinkState(skt_context_t* pThis)
{
  return pThis->link_state;
}

void NetSocket_SetLinkState(skt_context_t* pThis,uint8_t link_state)
{
  pThis->link_state = link_state;
}

/***************************************************************************
* 100ms调用一次(此函数属于应用层,用户根据实际情况实现)
****************************************************************************/
void NetSocket_CheckIsModemError(void)
{
#if 0
  if (zxep_socket.enable_flag == SOCKET_FALSE)
  {
    if (zx_socket.error_flag==TRUE) // 平台都连接不上,重启模块
    {
      zx_socket.error_flag = FALSE;
      gsm_socket_ResetModem();
    }
  }
  else
  {
    if ((zx_socket.error_flag==TRUE) && (zxep_socket.error_flag==TRUE)) // 所有平台都连接不上,重启模块
    {
      zx_socket.error_flag = FALSE;
      zxep_socket.error_flag = FALSE;
      gsm_socket_ResetModem();
    }
  }
#endif
}

#if 0
//extern skt_context_t skt_context[NUMBER_OF_LINK];
//#define zx_socket    skt_context[0] // 重型数据
//#define zxep_socket  skt_context[1] // 重型环保数据
//#define gGsm_ZxSocketFd    zx_socket.socket_fd
//#define gGsm_ZxepSocketFd  zxep_socket.socket_fd
//M2M_NetSocketInit();  // M2M数据初始化
//HJEP_NetSocketInit(); // 国六环保数据初始化

/******************************************************************************
*
*******************************************************************************/
void* pthread_gsm_SocketZxService(void *data)
{
  data = data;

  SYS_ZXParamRead();
  SYS_ParamRead();
  //gsm_socket_init(); // 套接字初始化
  zx_socket_init();   // 重型数据初始化
  gsm_socket_sigpipe();
  while (1)
  {
    msleep(10); // 10ms周期
    NetSocket_Service(&zx_socket);
  }
}

/******************************************************************************
*
*******************************************************************************/
void* pthread_gsm_SocketZxepService(void *data)
{
  data = data;

  zxep_socket_init(); // 国六环保数据初始化
  //gsm_socket_sigpipe();
  while (1)
  {
    msleep(10); // 10ms周期
    NetSocket_Service(&zxep_socket);
  }
}
#endif

