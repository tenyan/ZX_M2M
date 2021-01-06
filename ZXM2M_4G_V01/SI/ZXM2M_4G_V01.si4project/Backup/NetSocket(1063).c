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

/**********************************************************************************************
* SIGPIPE信号处理函数
* 当服务器close一个连接时(服务器故障),若client继续向服务器发数据,根据TCP协议的规定,客户端
* 会收到一个RST响应,client再往这个服务器发送数据时，系统会发出一个SIGPIPE信号给客户端进程,
* 导致客户端进程退出(结束main进程)。
* 如果使用了多个套接字,该信号的递交无法告诉我们是哪个套接字出的错。如果我们确实需要知道是
* 哪个write出了错,那么必须要么不理会该信号,要么从信号处理函数返回后再处理来自write的EPIPE.
***********************************************************************************************/
static void NetSocket_SigpipeHandle(int sig)
{
	//f_stuGsmState.ucLinkState[ZXEP_LINK_NUM] = GPRS_LINK_STATE_CLOSED;
	//g_stuSystem.ucGsmZxepLinkReset = 1;

#if SOCKET_DEBUG
  PcDebug_Printf("Socket SIGPIPE.\r\n");
#endif
}

//=============================================================================================
void NetSocket_sigpipe(void)
{
	struct sigaction sa;
  
  //sa.sa_handler = SIG_IGN;
	sa.sa_handler = NetSocket_SigpipeHandle;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGPIPE, &sa, NULL);
}

//=============================================================================================
void gsm_socket_init(void)
{
  zx_socket_init();   // 重型数据初始化
  zxep_socket_init(); // 国六环保数据初始化

  gsm_socket_sigpipe();
}

//=============================================================================================
void NetSocket_Invalid(skt_context_t* pThis)
{
  if(pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("InvSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = GPRS_LINK_STATE_CLOSED;
    close(pThis->socket_fd);
    pThis->socket_fd = -1;

    pThis->socket_10ms_timer = 100; // 1秒
  }
  else
  {
    if(pThis->socket_10ms_timer == 0)
    {
      if((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4G模块网络已就绪
      {
        pThis->socket_state = SOCKET_CREATE;
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void NetSocket_Create(skt_context_t* pThis)
{
  if(pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("CrtSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    
    pThis->socket_fd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET表示IPv4,SOCK_STREAM表示TCP
    if (pThis->socket_fd < 0) // 创建失败
    {
#if SOCKET_DEBUG
      PC_DebugPrintf("CrtSktErr:Id=%d\r\n",pThis->socket_id);
#endif
      pThis->error_cnt++;
      pThis->socket_state = SOCKET_CLOSE;
      pThis->state_transition = TRUE;
      return;
    }
    
#if SOCKET_DEBUG
    PC_DebugPrintf("CrtSkt:Id=%d,Fd=%d\r\n",pThis->socket_id,pThis->socket_fd);
#endif

    pThis->socket_10ms_timer = 200; // 2秒
  }
  else
  {
    if(pThis->socket_10ms_timer == 0)
    {
      pThis->socket_state = SOCKET_CONNECT;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void NetSocket_Connect(skt_context_t* pThis)
{
  int retval = 0;
  struct sockaddr_in server_addr;
  int8_t sbuff[50] = {0};
  
  if(pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("ConSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = GPRS_LINK_STATE_CONNECTING;
    (*pThis->reinit_server_addr)(); // 重新获取服务器地址和端口
    memset(&server_addr,0,sizeof(server_addr));
    server_addr.sin_family = AF_INET;  // IPv4协议
    server_addr.sin_port = htons(pThis->server_port);
    server_addr.sin_addr.s_addr = inet_addr((const char *)pThis->server_ip);
    retval = connect(pThis->socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr)); // 阻塞,如果服务器无效,时间大概2分钟
    if (retval < 0)
    {
#if SOCKET_DEBUG
      PC_DebugPrintf("ConSktErr:Id=%d\r\n",pThis->socket_id);
      //printf("Connect error %s errno: %d\n",strerror(errno),errno);
#endif
      pThis->error_cnt++;
      pThis->socket_state = SOCKET_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
      return;
    }
    sprintf((char *__restrict)sbuff, "SktId=%d IP-%s:%d\r\n",pThis->socket_id,pThis->server_ip,pThis->server_port);
    PC_DebugPrintf((const char *)sbuff);

    pThis->socket_10ms_timer = 100; // 1秒
  }
  else
  {
    if(pThis->socket_10ms_timer == 0)
    {
      pThis->socket_state = SOCKET_READY;
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

//=============================================================================================
void NetSocket_Ready(skt_context_t* pThis)
{
  if(pThis->state_transition)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("RdySkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    //pThis->error_cnt = 0x00; // 清除错误计数
    //pThis->error_flag = FALSE;
    pThis->hb_10ms_timer = pThis->hb_10ms_timer_sp;
    pThis->link_state = GPRS_LINK_STATE_READY;
  }
  else
  {    
    if(pThis->link_state == GPRS_LINK_STATE_CLOSED)
    {
      pThis->socket_state = SOCKET_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
    }

    if(pThis->hb_10ms_timer == 0x00) // 超时判断
    {
      pThis->socket_state = SOCKET_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
    }
  }

  // 4G模块判断
  if(GSM_GetModemState() == GSM_MODEM_STATE_START) // 4G模块处于初始化
  {
    pThis->link_state = GPRS_LINK_STATE_CLOSED;
    pThis->socket_state = SOCKET_CLOSE;
    pThis->state_transition = SOCKET_TRUE;
  }
}

//=============================================================================================
void NetSocket_Close(skt_context_t* pThis)
{
  if(pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("CloSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;
    pThis->link_state = GPRS_LINK_STATE_CLOSED;
    shutdown(pThis->socket_fd,SHUT_RDWR); // 正常关闭,保证通信双方都能够收到应用程序发出的所有数据
    pThis->socket_fd = -1;
    pThis->socket_10ms_timer = 800; // 8秒
  }
  else
  {
    if(pThis->socket_10ms_timer == 0)
    {
      if(pThis->error_cnt > 3) // 连接5次都失败,暂停1分钟后再连
      {
        pThis->socket_state = SOCKET_SILENCE; // 重连
        pThis->state_transition = SOCKET_TRUE;
      }
      else
      {
        pThis->socket_state = SOCKET_INVALID; // 重连
        pThis->state_transition = SOCKET_TRUE;
      }
    }
  }
}

//=============================================================================================
void NetSocket_Silence(skt_context_t* pThis)
{
  if(pThis->state_transition==SOCKET_TRUE)
  {
    pThis->state_transition = SOCKET_FALSE;
    pThis->started_delay = SOCKET_FALSE;
#if SOCKET_DEBUG
    PC_DebugPrintf("SilSkt:Id=%d\r\n",pThis->socket_id);
#endif
  }

  if(pThis->started_delay==SOCKET_FALSE)
  {
    pThis->started_delay = SOCKET_TRUE;

    pThis->link_state = GPRS_LINK_STATE_CLOSED;
    pThis->error_flag = SOCKET_TRUE;
    pThis->socket_10ms_timer = 12000; // 2分钟
  }
  else
  {
    if(pThis->socket_10ms_timer == 0)
    {
      pThis->error_cnt = 0x00; // 清除错误计数
      pThis->error_flag = SOCKET_FALSE;

      pThis->socket_state = SOCKET_INVALID; // 重连
      pThis->state_transition = SOCKET_TRUE;
    }
  }
}

/******************************************************************************
* 此函数负责维护Socket链接,每10ms调用一次(无阻塞)
*******************************************************************************/
void NetSocket_Service(skt_context_t* pThis)
{
  switch(pThis->socket_state)
  {
    case SOCKET_INVALID: // 等待4G网络READY
      gsm_socket_invalid(pThis);
      break;
      
    case SOCKET_CREATE: // 创建SOCKET通信端点
      gsm_socket_create(pThis);
      break;
      
    case SOCKET_CONNECT: // 连接服务器
      gsm_socket_connect(pThis);
      break;

    case SOCKET_READY: // 已连接可通信
      gsm_socket_ready(pThis);
      break;

    case SOCKET_CLOSE: // 关闭SOCKET
      gsm_socket_close(pThis);
      break;

    case SOCKET_SILENCE: // 静默一段时间
      gsm_socket_silence(pThis);
      break;

    default:
      pThis->socket_state = SOCKET_CLOSE;
      pThis->state_transition = TRUE;
      break;
  }

  if(pThis->socket_10ms_timer)
    pThis->socket_10ms_timer--;

  if(pThis->hb_10ms_timer)
    pThis->hb_10ms_timer--;
}

/******************************************************************************
 * 功能: GSM模块GPRS数据发送接口
 * 输入: pucData=待发送数据指针; usLen=待发送数据长度
 * 返回: 0 :成功返回 -1:发送失败
*******************************************************************************/
int32_t NetSocket_Send(skt_context_t* pThis, uint8_t *pData, uint16_t len)
{
	uint8_t BuffTemp[GSM_RECV_BUFF_MAX_SIZE] = {0};
  int sent_bytes;

	if ((pData==NULL) || (len==0) || (len > GSM_SEND_BUFF_MAX_SIZE))
	{
#if SOCKET_DEBUG
    PC_DebugPrintf("SktSendParaErr:Id=%d\r\n",pThis->socket_id);
#endif
		return -1;
	}

	if (pThis->link_state==GPRS_LINK_STATE_READY)  // 在线状态
	{
		pthread_mutex_lock(&pThis->send_mutex);
    sent_bytes = send(pThis->socket_fd, pData, len, 0);
    pthread_mutex_unlock(&pThis->send_mutex);
    
		if (sent_bytes < 0)
		{
#if SOCKET_DEBUG
      PC_DebugPrintf("SktSendErr:Id=%d\r\n",pThis->socket_id);
#endif
      pThis->error_cnt++;
      pThis->link_state = GPRS_LINK_STATE_CLOSED;
			return -1;
		}
    
		if (SYS_GetDubugStatus()&BIT(0))
		{	
		  DataToHexbuff(BuffTemp, pData, len);
		  PC_SendDebugData(BuffTemp, len*2, DEBUG_AT);
    }
    
    PC_SendDebugData(pData, len, DEBUG_GPRS);

    return 0;
	}

	return -1;
}

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
		usleep(One_MilliSecond*10); // 10ms周期
    gsm_socket_service(&zx_socket);
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
		usleep(One_MilliSecond*10); // 10ms周期
    gsm_socket_service(&zxep_socket);
	}
}

/******************************************************************************
*
*******************************************************************************/
uint8_t NetSocket_GetLinkState(skt_context_t* pThis)
{
	return pThis->link_state;
}

void NetSocket_SetLinkState(skt_context_t* pThis,uint8_t link_state)
{
	pThis->link_state = link_state;
}

/******************************************************************************
* 100ms调用一次
*******************************************************************************/
void NetSocket_CheckIsModemError(void)
{
  if(zxep_socket.enable_flag == SOCKET_FALSE)
  {
    if(zx_socket.error_flag==TRUE) // 平台都连接不上,重启模块
    {
      zx_socket.error_flag = FALSE;
      gsm_socket_ResetModem();
    }
  }
  else
  {
    if((zx_socket.error_flag==TRUE) && (zxep_socket.error_flag==TRUE)) // 所有平台都连接不上,重启模块
    {
      zx_socket.error_flag = FALSE;
      zxep_socket.error_flag = FALSE;
      gsm_socket_ResetModem();
    }
  }
}










/******************************************************************************
 * socket状态机
 ******************************************************************************/
void NetSocket_Invalid(skt_context_t* pThis)
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
void NetSocket_Create(skt_context_t* pThis)
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

