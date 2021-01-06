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
* SIGPIPE�źŴ�����
* ��������closeһ������ʱ(����������),��client�����������������,
* ����TCPЭ��Ĺ涨,�ͻ��˻��յ�һ��RST��Ӧ,client���������������������ʱ,
* ϵͳ�ᷢ��һ��SIGPIPE�źŸ��ͻ��˽���,���¿ͻ��˽����˳�(����main����)��
* ���ʹ���˶���׽���,���źŵĵݽ��޷������������ĸ��׽��ֳ��Ĵ�
* �������ȷʵ��Ҫ֪�����ĸ�write���˴�,��ô����Ҫô�������ź�,
* Ҫô���źŴ��������غ��ٴ�������write��EPIPE.
*******************************************************************************/
static void NetSocket_SigpipeHandle(int sig)
{
#if SOCKET_DEBUG
  PcDebug_Printf("Socket SIGPIPE.\r\n");
#endif
}

//==�źŴ�������ֹ��ʬ����===================================================
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

    pThis->socket_10ms_timer = 100; // 1��
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      if ((Modem_GetState()==MODEM_STATE_DATA_READY) && (pThis->enable_flag==SOCKET_TRUE)) // 4Gģ�������Ѿ���
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

    // sock_fd = socket(AF_INET, SOCK_DGRAM, 0); // AF_INET��ʾIPv4,SOCK_DGRAM��ʾUDP
    pThis->socket_fd = socket(AF_INET, SOCK_STREAM, 0); // AF_INET��ʾIPv4,SOCK_STREAM��ʾTCP
    if (pThis->socket_fd < 0) // ����ʧ��
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

    pThis->socket_10ms_timer = 200; // 2��
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
    (*pThis->reinit_server_addr)(); // ���»�ȡ��������ַ�Ͷ˿�
    memset(&server_addr,0,sizeof(server_addr));
    server_addr.sin_family = AF_INET;  // IPv4Э��
    server_addr.sin_port = htons(pThis->srv_port);
    server_addr.sin_addr.s_addr = inet_addr((const char *)pThis->srv_ip);
    retval = connect(pThis->socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr)); // ����,�����������Ч,ʱ����2����
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

    pThis->socket_10ms_timer = 100; // 1��
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
    //pThis->error_cnt = 0x00; // ����������
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

    if (pThis->hb_10ms_timer == 0x00) // ��ʱ�ж�
    {
      pThis->state = NET_SOCKET_STATE_CLOSE;
      pThis->state_transition = SOCKET_TRUE;
    }

    if (pThis->error_cnt > 3) // ����3�ζ�ʧ��
    {
      pThis->state = NET_SOCKET_STATE_CLOSE; // ����
      pThis->state_transition = SOCKET_TRUE;
    }
  }

  // 4Gģ���ж�
  if (Modem_GetState()!=MODEM_STATE_DATA_READY) // 4Gģ�鴦�ڳ�ʼ��
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
    shutdown(pThis->socket_fd,SHUT_RDWR); // �����ر�,��֤ͨ��˫�����ܹ��յ�Ӧ�ó��򷢳�����������
    pThis->socket_fd = -1;
    pThis->socket_10ms_timer = 800; // 8��
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
    {
      if (pThis->error_cnt > 3) // ����5�ζ�ʧ��,��ͣ1���Ӻ�����
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
    pThis->socket_10ms_timer = 12000; // 2����
  }
  else
  {
    if (pThis->socket_10ms_timer == 0)
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
void NetSocket_Service(skt_context_t* pThis)
{
  switch (pThis->state)
  {
  case NET_SOCKET_STATE_INVALID: // �ȴ�4G����READY
    NetSocket_Invalid(pThis);
    break;

  case NET_SOCKET_STATE_CREATE: // ����SOCKETͨ�Ŷ˵�
    NetSocket_Create(pThis);
    break;

  case NET_SOCKET_STATE_CONNECT: // ���ӷ�����
    NetSocket_Connect(pThis);
    break;

  case NET_SOCKET_STATE_READY: // �����ӿ�ͨ��
    NetSocket_Ready(pThis);
    break;

  case NET_SOCKET_STATE_CLOSE: // �ر�SOCKET
    NetSocket_Close(pThis);
    break;

  case NET_SOCKET_STATE_SILENCE: // ��Ĭһ��ʱ��
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
 * ����: GSMģ��GPRS���ݷ��ͽӿ�
 * ����: pucData=����������ָ��; usLen=���������ݳ���
 * ����: 0 :�ɹ����� -1:����ʧ��
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

  if (pThis->link_state==SOCKET_LINK_STATE_READY)  // ����״̬
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
* 100ms����һ��(�˺�������Ӧ�ò�,�û�����ʵ�����ʵ��)
****************************************************************************/
void NetSocket_CheckIsModemError(void)
{
#if 0
  if (zxep_socket.enable_flag == SOCKET_FALSE)
  {
    if (zx_socket.error_flag==TRUE) // ƽ̨�����Ӳ���,����ģ��
    {
      zx_socket.error_flag = FALSE;
      gsm_socket_ResetModem();
    }
  }
  else
  {
    if ((zx_socket.error_flag==TRUE) && (zxep_socket.error_flag==TRUE)) // ����ƽ̨�����Ӳ���,����ģ��
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
//#define zx_socket    skt_context[0] // ��������
//#define zxep_socket  skt_context[1] // ���ͻ�������
//#define gGsm_ZxSocketFd    zx_socket.socket_fd
//#define gGsm_ZxepSocketFd  zxep_socket.socket_fd
//M2M_NetSocketInit();  // M2M���ݳ�ʼ��
//HJEP_NetSocketInit(); // �����������ݳ�ʼ��

/******************************************************************************
*
*******************************************************************************/
void* pthread_gsm_SocketZxService(void *data)
{
  data = data;

  SYS_ZXParamRead();
  SYS_ParamRead();
  //gsm_socket_init(); // �׽��ֳ�ʼ��
  zx_socket_init();   // �������ݳ�ʼ��
  gsm_socket_sigpipe();
  while (1)
  {
    msleep(10); // 10ms����
    NetSocket_Service(&zx_socket);
  }
}

/******************************************************************************
*
*******************************************************************************/
void* pthread_gsm_SocketZxepService(void *data)
{
  data = data;

  zxep_socket_init(); // �����������ݳ�ʼ��
  //gsm_socket_sigpipe();
  while (1)
  {
    msleep(10); // 10ms����
    NetSocket_Service(&zxep_socket);
  }
}
#endif

