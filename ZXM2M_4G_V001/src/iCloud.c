/*****************************************************************************
* @FileName: iCloud.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-6
* @brief
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Typedef
******************************************************************************/


/******************************************************************************
* Define
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/
#define ICLOUD_DELAY(ms)    msleep(ms)

/******************************************************************************
* Data Types and Globals
******************************************************************************/
skt_context_t skt_context[NUMBER_OF_SOCKET_ID];

uint8_t zxm2m_conn_tx_buffer[SOCKET_CONN_MAX_DATA_LEN];
uint8_t zxm2m_conn_rx_buffer[SOCKET_CONN_MAX_DATA_LEN];
int16_t zxm2m_conn_rx_len = 0;
//uint8_t zxm2m_conn_buffer[SOCKET_CONN_MAX_DATA_LEN*2];

uint8_t mainep_conn_tx_buffer[SOCKET_CONN_MAX_DATA_LEN];
uint8_t mainep_conn_rx_buffer[SOCKET_CONN_MAX_DATA_LEN];
int16_t mainep_conn_rx_len = 0;
//uint8_t mainep_conn_buffer[SOCKET_CONN_MAX_DATA_LEN*2];

uint8_t subep_conn_tx_buffer[SOCKET_CONN_MAX_DATA_LEN];
uint8_t subep_conn_rx_buffer[SOCKET_CONN_MAX_DATA_LEN];
int16_t subep_conn_rx_len = 0;
//uint8_t subep_conn_buffer[SOCKET_CONN_MAX_DATA_LEN*2];

// HJ����
#define hjep_conn_tx_buffer  mainep_conn_tx_buffer

// GB����(ֱ����������ƽ̨)
#define bjep_conn_tx_buffer  mainep_conn_tx_buffer
#define gbep_conn_tx_buffer  subep_conn_tx_buffer

/******************************************************************************
 * ��byte����ת�����ַ���,���ڴ�ӡ���
*******************************************************************************/
void ConvertBinToAcsii(uint8_t* pstr, uint8_t byte)
{
  uint8_t lowHalfByte;
  uint8_t highHalfByte;

  lowHalfByte = byte & 0x0F;  // ��ȡ����λ
  highHalfByte = (byte>>4) & 0x0F; // ��ȡ����λ

  if (highHalfByte > 9)
    pstr[0] = (highHalfByte-0x0A) + 'A';
  else
    pstr[0] = highHalfByte + '0';

  if (lowHalfByte > 9)
    pstr[1] = (lowHalfByte - 0x0A) + 'A';
  else
    pstr[1] = lowHalfByte + '0';
}

//==��Byte����ת�����ַ���=================================================
void ConvertBytesToString(uint8_t* pstr, uint8_t* pbyte, uint16_t byte_num)
{
  while(byte_num--)
  {
    ConvertBinToAcsii(pstr, *pbyte);
    pbyte++;
    pstr += 2;
  }
}

/******************************************************************************
* ���溯�����û���д,���ڵײ���������
*******************************************************************************/
void M2M_ReInitServerAddr(void)
{
  zxm2m_socket.srv_protocol = m2m_asset_data.main_srv_protocol;
  zxm2m_socket.srv_port = m2m_asset_data.main_srv_port;
  sprintf((char *)zxm2m_socket.srv_ip, "%d.%d.%d.%d",m2m_asset_data.main_srv_ip[0],m2m_asset_data.main_srv_ip[1],m2m_asset_data.main_srv_ip[2],m2m_asset_data.main_srv_ip[3]);
}

//==========================================================================
void M2M_NetSocketInit(void)
{
  zxm2m_socket.socket_id = ZXM2M_SOCKET_ID;
  zxm2m_socket.state = NET_SOCKET_STATE_INVALID;
  zxm2m_socket.started_delay = SOCKET_FALSE;
  zxm2m_socket.state_transition = SOCKET_TRUE;
  zxm2m_socket.error_cnt = 0;
  zxm2m_socket.error_flag = SOCKET_FALSE;
  zxm2m_socket.enable_flag = SOCKET_TRUE;
  zxm2m_socket.link_state = SOCKET_LINK_STATE_CLOSED;
  zxm2m_socket.hb_10ms_timer_sp = ZXM2M_HEART_BEAT_TIMEOUT_SP;

  zxm2m_socket.tx_buff = zxm2m_conn_tx_buffer;
  zxm2m_socket.tx_len = 0;
  zxm2m_socket.new_tx_data_flag = 0;

  zxm2m_socket.reinit_server_addr = &M2M_ReInitServerAddr;
  (*zxm2m_socket.reinit_server_addr)();
}

//============================================================================
void* pthread_ZxM2mSocketService(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms����
    NetSocket_Service(&zxm2m_socket);
  }
}

//==����M2M���������߳�=======================================================
void* pthread_ZxM2mProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms����
    M2M_ProduceSendMsg(&m2m_context);
  }
}

//==����M2M�������ݴ����߳�====================================================
void* pthread_ZxM2mProcess(void *argument)
{
  while (1)
  {
    if((NetSocket_GetLinkState(&zxm2m_socket)<SOCKET_LINK_STATE_READY) || (ZxM2mSocketFd<0))
    {
      msleep(100);
    }
    else
    {
      // recv error : Transport endpoint is not connected
      zxm2m_conn_rx_len = recv(ZxM2mSocketFd, zxm2m_conn_rx_buffer, (SOCKET_CONN_MAX_DATA_LEN-1), 0);
      if(zxm2m_conn_rx_len < 0)
      {
        if (errno == EAGAIN)
        {
          printf("RE-Len:%d errno EAGAIN\n", zxm2m_conn_rx_len);
          continue;
        }

        if (errno == EINTR)
        {  continue;}

        perror("recv error\n");
//#if SOCKET_DEBUG
//      PcDebug_Printf("RecvSocketErr:Id=%d\r\n",zxm2m_socket.socket_id);
//#endif
      }
      else if(zxm2m_conn_rx_len > 0)
      {
        if(zxm2m_conn_rx_len < SOCKET_CONN_MAX_DATA_LEN)
        {
          //ConvertBytesToString(zxm2m_conn_buffer, zxm2m_conn_rx_buffer, zxm2m_conn_rx_len);
          //PcDebug_SendData(zxm2m_conn_buffer, zxm2m_conn_rx_len*2, DBG_MSG_TYPE_NET);
          PcDebug_SendData(zxm2m_conn_rx_buffer, zxm2m_conn_rx_len, DBG_MSG_TYPE_NET);
          
          m2m_context.rx_size = zxm2m_conn_rx_len;
          m2m_context.rx_data = zxm2m_conn_rx_buffer;
          m2m_context.rx_from = SYSBUS_DEVICE_TYPE_CELLURA;
          M2M_ProcessRecvMsg(&m2m_context); // ����M2M����
          
          //��Э������ת��������������
          if(zxm2m_conn_rx_buffer[0]==0x05)
          {
            if( (zxm2m_conn_rx_buffer[15]=='P'&&zxm2m_conn_rx_buffer[16]=='W') || (zxm2m_conn_rx_buffer[15]=='R'&&zxm2m_conn_rx_buffer[16]=='C') )
            {
              AuxCom_SendM2mData(zxm2m_conn_rx_buffer, zxm2m_conn_rx_len);
            }
          }
       
          zxm2m_socket.error_cnt = 0x00; // ����������
          zxm2m_socket.error_flag = FALSE;
          zxm2m_socket.hb_10ms_timer = zxm2m_socket.hb_10ms_timer_sp;
//#if SOCKET_DEBUG
//        PcDebug_Printf("RecvSkt:Id=%d\r\n",zxm2m_socket.socket_id);
//#endif
        }
      }
      else  // ����Ϊ0��ʾsocket�ر�
      {
        NetSocket_SetLinkState(&zxm2m_socket, SOCKET_LINK_STATE_CLOSED);
#if SOCKET_DEBUG
        PcDebug_Printf("DiscSkt:Id=%d\r\n", zxm2m_socket.socket_id);
#endif
      }
    }
    msleep(1);
  }
}

//===========================================================================
void ZxM2m_ServiceInit(void)
{
  Parm_ReadM2mAssetData();
  Parm_ReadLvcInfo();
  M2M_Initialize();
  M2M_NetSocketInit();
}

//============================================================================
void ZxM2m_ServiceStart(void)
{
  pthread_attr_t thread_attr;
  int ret ,stacksize = DEFAULT_THREAD_STACK_SIZE; // thread��ջ����Ϊ40KB

  pthread_attr_init(&thread_attr);
  ret = pthread_attr_setstacksize(&thread_attr,stacksize);
  if(ret!=0)
  {
    printf("Set StackSize Error!\n");
  }

  pthread_create(&pthreads[PTHREAD_ZXM2M_PRODUCE_ID], &thread_attr, pthread_ZxM2mProduce, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_PROCESS_ID], &thread_attr, pthread_ZxM2mProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_SOCKET_SERVICE_ID], &thread_attr, pthread_ZxM2mSocketService, NULL);
  usleep(10);
}

/******************************************************************************
* ���溯�����û���д,���ڵײ���������
*******************************************************************************/
void ReInitHjepServerAddr(void)
{
  hjep_socket.srv_protocol = HJEP_CLOUD_SERVER_PROTOCOL;
  hjep_socket.srv_port = HJEP_CLOUD_SERVER_PORT;
  strcpy((char *)hjep_socket.srv_ip,HJEP_CLOUD_SERVER_IP);
}

//=========================================================================
void HJEP_NetSocketInit(void)
{
  hjep_socket.socket_id = HJEP_SOCKET_ID;
  hjep_socket.state = NET_SOCKET_STATE_INVALID;
  hjep_socket.started_delay = SOCKET_FALSE;
  hjep_socket.state_transition = SOCKET_TRUE;
  hjep_socket.error_cnt = 0;
  hjep_socket.error_flag = SOCKET_FALSE;
  hjep_socket.enable_flag = SOCKET_FALSE;
  //hjep_socket.enable_flag = SOCKET_TRUE;
  hjep_socket.link_state = SOCKET_LINK_STATE_CLOSED;
  hjep_socket.hb_10ms_timer_sp = HJEP_HEART_BEAT_TIMEOUT_SP;

  hjep_socket.tx_buff = hjep_conn_tx_buffer;
  hjep_socket.tx_len = 0;
  hjep_socket.new_tx_data_flag = 0;

  hjep_socket.reinit_server_addr = &ReInitHjepServerAddr;
  //(*hjep_socket.reinit_server_addr)();

  subep_socket.enable_flag = SOCKET_FALSE;
}

/******************************************************************************
* ���溯�����û���д,���ڵײ���������
*******************************************************************************/
void ReInitBjepServerAddr(void)
{
  //==��������ת��ΪIP��ַ==================================================
  bjep_socket.srv_protocol = BJEP_CLOUD_SERVER_PROTOCOL;
  bjep_socket.srv_port = BJEP_CLOUD_SERVER_PORT;

  char* hostname = BJEP_CLOUD_SERVER_DNS;  // ��������ƽ̨��������ַ
  struct hostent* phost;
  
  phost = gethostbyname(hostname); // gethostbyname()�ǲ������뺯��
  if (phost == NULL)
  {
    PcDebug_Printf("Bjep:DnsToIpFail!\n");
    //strcpy((char * __restrict__)bjep_socket.srv_ip,"210.73.67.185"); // Ĭ��ip
    return;
  }

  if (AF_INET != phost->h_addrtype)
  {
    PcDebug_Printf("Bjep:DnsToIpFail!\n");
    //strcpy((char * __restrict__)bjep_socket.srv_ip,"210.73.67.185"); // Ĭ��ip
    return;
  }

  // ��ȡʵ��IP��ַ
  sprintf((char * __restrict__)bjep_socket.srv_ip, "%s", inet_ntoa(*((struct in_addr*)phost->h_addr_list[0]))); 
  PcDebug_Printf("BjepSrvIp-%s:%d\n",bjep_socket.srv_ip, bjep_socket.srv_port);
}


//=============================================================================================
void BJEP_NetSocketInit(void)
{
  pthread_mutex_init(&bjep_socket.send_mutex, NULL);

  bjep_socket.socket_id = BJEP_SOCKET_ID;
  bjep_socket.state = NET_SOCKET_STATE_INVALID;
  bjep_socket.started_delay = SOCKET_FALSE;
  bjep_socket.state_transition = SOCKET_TRUE;
  bjep_socket.error_cnt = 0;
  bjep_socket.error_flag = SOCKET_FALSE;
  bjep_socket.socket_fd = -1;
  bjep_socket.enable_flag = SOCKET_FALSE;
  bjep_socket.link_state = SOCKET_LINK_STATE_CLOSED;
  bjep_socket.hb_10ms_timer_sp = BJEP_HEART_BEAT_TIMEOUT_SP;

  bjep_socket.tx_buff = bjep_conn_tx_buffer;
  bjep_socket.tx_len = 0;
  bjep_socket.new_tx_data_flag = 0;

  bjep_socket.reinit_server_addr = &ReInitBjepServerAddr;
  //(*bjep_socket.reinit_server_addr)();
}

/******************************************************************************
* ���溯�����û���д,���ڵײ���������
*******************************************************************************/
void ReInitGbepServerAddr(void)
{
  gbep_socket.srv_protocol =GBEP_CLOUD_SERVER_PROTOCOL;
  gbep_socket.srv_port = GBEP_CLOUD_SERVER_PORT;
  strcpy((char *)gbep_socket.srv_ip,GBEP_CLOUD_SERVER_IP);
}

//=========================================================================
void GBEP_NetSocketInit(void)
{
  gbep_socket.socket_id = GBEP_SOCKET_ID;
  gbep_socket.state = NET_SOCKET_STATE_INVALID;
  gbep_socket.started_delay = SOCKET_FALSE;
  gbep_socket.state_transition = SOCKET_TRUE;
  gbep_socket.error_cnt = 0;
  gbep_socket.error_flag = SOCKET_FALSE;
  gbep_socket.enable_flag = SOCKET_FALSE;
  //gbep_socket.enable_flag = SOCKET_TRUE;
  gbep_socket.link_state = SOCKET_LINK_STATE_CLOSED;
  gbep_socket.hb_10ms_timer_sp = GBEP_HEART_BEAT_TIMEOUT_SP;

  gbep_socket.tx_buff = gbep_conn_tx_buffer;
  gbep_socket.tx_len = 0;
  gbep_socket.new_tx_data_flag = 0;

  gbep_socket.reinit_server_addr = &ReInitGbepServerAddr;
  //(*gbep_socket.reinit_server_addr)();
}

/******************************************************************************
* ����������
*******************************************************************************/
//==����������=============================================================
void* pthread_MainEpSocketService(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms����
    NetSocket_Service(&mainep_socket);
  }
}

//==���������ӷ��������߳�=================================================
void* pthread_MainEpProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms����
    if (CAN_GetEpType()==EP_TYPE_HJ) // �������ܿ���
    {
      HJEP_ProduceSendData();
    }
    else if(CAN_GetEpType()==EP_TYPE_GB)
    {
      GBEP_ProduceSendData();
    }
  }
}

//==���������ӽ������ݴ����߳�=============================================
void* pthread_MainEpProcess(void *argument)
{
  while (1)
  {
    if ((NetSocket_GetLinkState(&mainep_socket)<SOCKET_LINK_STATE_READY) || (MainEpSocketFd<0))
    {
      msleep(100);
    }
    else
    {
      mainep_conn_rx_len = recv(MainEpSocketFd, mainep_conn_rx_buffer, (SOCKET_CONN_MAX_DATA_LEN-1), 0);
      if (mainep_conn_rx_len < 0)
      {
        if (errno == EAGAIN)
        {
          printf("RE-Len:%d errno EAGAIN\n", mainep_conn_rx_len);
          continue;
        }

        if (errno == EINTR)
        {
          continue;
        }

        perror("recv error\n");
//#if SOCKET_DEBUG
//      PcDebug_Printf("RecvSocketErr:Id=%d\r\n",mainep_socket.socket_id);
//#endif
      }
      else if (mainep_conn_rx_len > 0)
      {
        if (mainep_conn_rx_len < SOCKET_CONN_MAX_DATA_LEN)
        {
          PcDebug_SendData(mainep_conn_rx_buffer, mainep_conn_rx_len, DBG_MSG_TYPE_SYS);
          //HJEP_ProcessRecvData(mainep_conn_rx_buffer, mainep_conn_rx_len);
          if((mainep_conn_rx_buffer[0]==0x23) && (mainep_conn_rx_buffer[1]==0x23)) // ֡ͷ
          {
            mainep_socket.error_cnt = 0x00; // ����������
            mainep_socket.error_flag = FALSE;
            mainep_socket.hb_10ms_timer = mainep_socket.hb_10ms_timer_sp;
          }
//#if SOCKET_DEBUG
//        PcDebug_Printf("RecvSkt:Id=%d\r\n",mainep_socket.socket_id);
//#endif
        }
      }
      else  // ����Ϊ0��ʾsocket�ر�
      {
        NetSocket_SetLinkState(&mainep_socket, SOCKET_LINK_STATE_CLOSED);
#if SOCKET_DEBUG
        PcDebug_Printf("DiscSkt:Id=%d\r\n", mainep_socket.socket_id);
#endif
      }
    }
    msleep(1);
  }
}

/******************************************************************************
* ����������
*******************************************************************************/
//==����������=============================================================
void* pthread_SubEpSocketService(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms����
    NetSocket_Service(&subep_socket);
  }
}

//==���������ӷ��������߳�=================================================
void* pthread_SubEpProduce(void *argument)
{
  while (1)
  {
    sleep(5); // 1s����
#if 0
    if(CAN_GetEpType()==EP_TYPE_GB)  // �������ܿ���
    {
      msleep(10); // 10ms����
      //GBEP_ProduceSendData();
    }
    else
    {
      subep_socket.enable_flag = SOCKET_FALSE;
      sleep(1); // 1s����
    }
#endif
  }
}

//==���������ӽ������ݴ����߳�=============================================
void* pthread_SubEpProcess(void *argument)
{
  while (1)
  {
    if ((NetSocket_GetLinkState(&subep_socket)<SOCKET_LINK_STATE_READY) || (SubEpSocketFd<0))
    {
      msleep(100);
    }
    else
    {
      subep_conn_rx_len = recv(SubEpSocketFd, subep_conn_rx_buffer, (SOCKET_CONN_MAX_DATA_LEN-1), 0);
      if (subep_conn_rx_len < 0)
      {
        if (errno == EAGAIN)
        {
          printf("RE-Len:%d errno EAGAIN\n", subep_conn_rx_len);
          continue;
        }

        if (errno == EINTR)
        {
          continue;
        }

        perror("recv error\n");
//#if SOCKET_DEBUG
//      PcDebug_Printf("RecvSocketErr:Id=%d\r\n",subep_socket.socket_id);
//#endif
      }
      else if (subep_conn_rx_len > 0)
      {
        if (subep_conn_rx_len < SOCKET_CONN_MAX_DATA_LEN)
        {
          PcDebug_SendData(subep_conn_rx_buffer, subep_conn_rx_len, DBG_MSG_TYPE_SYS);
          //GBEP_ProcessRecvData(subep_conn_rx_buffer, subep_conn_rx_len);
          if((subep_conn_rx_buffer[0]==0x23) && (subep_conn_rx_buffer[1]==0x23)) // ֡ͷ
          {
            subep_socket.error_cnt = 0x00; // ����������
            subep_socket.error_flag = FALSE;
            subep_socket.hb_10ms_timer = subep_socket.hb_10ms_timer_sp;
          }
//#if SOCKET_DEBUG
//        PcDebug_Printf("RecvSkt:Id=%d\r\n",subep_socket.socket_id);
//#endif
        }
      }
      else  // ����Ϊ0��ʾsocket�ر�
      {
        NetSocket_SetLinkState(&subep_socket, SOCKET_LINK_STATE_CLOSED);
#if SOCKET_DEBUG
        PcDebug_Printf("DiscSkt:Id=%d\r\n", subep_socket.socket_id);
#endif
      }
    }
    msleep(1);
  }
}

//===========================================================================
void EP_ServiceInit(void)
{
  // ������
  m2m_asset_data.vin_valid_flag = 0x01;
  m2m_asset_data.ep_type = EP_TYPE_HJ;
  colt_info.engine_speed = 400;
  colt_info.ep_valid_flag = 1;

  if (CAN_GetEpType()==EP_TYPE_HJ) // �������ܿ���
  {
    HJEP_Initialize();
    HJEP_NetSocketInit();
  }
  else if(CAN_GetEpType()==EP_TYPE_GB)
  {
    GBEP_Initialize();
    BJEP_NetSocketInit();
    GBEP_NetSocketInit();
  }
}

//============================================================================
void EP_ServiceStart(void)
{
  pthread_attr_t thread_attr;
  int ret ,stacksize = DEFAULT_THREAD_STACK_SIZE; // thread��ջ����Ϊ40KB

  pthread_attr_init(&thread_attr);
  ret = pthread_attr_setstacksize(&thread_attr,stacksize);
  if(ret!=0)
  {
    printf("Set StackSize Error!\n");
  }

  // ����������
  pthread_create(&pthreads[PTHREAD_MAINEP_PRODUCE_ID], &thread_attr, pthread_MainEpProduce, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MAINEP_PROCESS_ID], &thread_attr, pthread_MainEpProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MAINEP_SOCKET_SERVICE_ID], &thread_attr, pthread_MainEpSocketService, NULL);
  usleep(10);

  // ����������
  //pthread_create(&pthreads[PTHREAD_SUBEP_PRODUCE_ID], &thread_attr, pthread_SubEpProduce, NULL);
  //usleep(10);
  pthread_create(&pthreads[PTHREAD_SUBEP_PROCESS_ID], &thread_attr, pthread_SubEpProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_SUBEP_SOCKET_SERVICE_ID], &thread_attr, pthread_SubEpSocketService, NULL);
  usleep(10);
}

/***************************************************************************
* 100ms����һ��(�˺�������Ӧ�ò�,�û�����ʵ�����ʵ��)
****************************************************************************/
void Net_CheckIsModemError(void)
{
  if (subep_socket.enable_flag == SOCKET_TRUE)
  {
    if ((zxm2m_socket.error_flag==TRUE) && (mainep_socket.error_flag==TRUE) && (subep_socket.error_flag==TRUE)) // ����ƽ̨�����Ӳ���,����ģ��
    {
      zxm2m_socket.error_flag = FALSE;
      mainep_socket.error_flag = FALSE;
      subep_socket.error_flag = FALSE;
      Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
    }
  }
  else if(mainep_socket.enable_flag == SOCKET_TRUE)
  {
    if ((zxm2m_socket.error_flag==TRUE) && (mainep_socket.error_flag==TRUE)) // ����ƽ̨�����Ӳ���,����ģ��
    {
      zxm2m_socket.error_flag = FALSE;
      mainep_socket.error_flag = FALSE;
      Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
    }
  }
  else
  {
    if (zxm2m_socket.error_flag==TRUE) // ƽ̨�����Ӳ���,����ģ��
    {
      zxm2m_socket.error_flag = FALSE;
      Modem_SetState(MODEM_STATE_MINI_FUN); // ����ģ��
    }
  }
}

