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

uint8_t hjep_conn_tx_buffer[SOCKET_CONN_MAX_DATA_LEN];
uint8_t hjep_conn_rx_buffer[SOCKET_CONN_MAX_DATA_LEN];
int16_t hjep_conn_rx_len = 0;
//uint8_t hjep_conn_buffer[SOCKET_CONN_MAX_DATA_LEN*2];

/******************************************************************************
 * 将byte数组转换成字符串,用于打印输出
*******************************************************************************/
void ConvertBinToAcsii(uint8_t* pstr, uint8_t byte)
{
  uint8_t lowHalfByte;
  uint8_t highHalfByte;

  lowHalfByte = byte & 0x0F;  // 获取第四位
  highHalfByte = (byte>>4) & 0x0F; // 获取高四位

  if (highHalfByte > 9)
    pstr[0] = (highHalfByte-0x0A) + 'A';
  else
    pstr[0] = highHalfByte + '0';

  if (lowHalfByte > 9)
    pstr[1] = (lowHalfByte - 0x0A) + 'A';
  else
    pstr[1] = lowHalfByte + '0';
}

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
* 下面函数由用户编写,用于底层网络连接
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
    msleep(10); // 10ms周期
    NetSocket_Service(&zxm2m_socket);
  }
}

//==重型M2M发送数据线程=======================================================
void* pthread_ZxM2mProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期
    M2M_ProduceSendMsg(&m2m_context);
  }
}

//==重型M2M接收数据处理线程====================================================
void* pthread_ZxM2mProcess(void *argument)
{
  while (1)
  {
    if((NetSocket_GetLinkState(&zxm2m_socket)==SOCKET_LINK_STATE_CLOSED) || (ZxM2mSocketFd<0))
    {
      msleep(10);
    }
    else
    {
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
//      PC_DebugPrintf("RecvSocketErr:Id=%d\r\n",zxm2m_socket.socket_id);
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
          M2M_ProcessRecvMsg(&m2m_context); // 处理M2M数据
          
          //向协处理器转发参数设置命令
          if(zxm2m_conn_rx_buffer[0]==0x05)
          {
            if( (zxm2m_conn_rx_buffer[15]=='P'&&zxm2m_conn_rx_buffer[16]=='W') || (zxm2m_conn_rx_buffer[15]=='R'&&zxm2m_conn_rx_buffer[16]=='C') )
            {
              AuxCom_SendM2mData(zxm2m_conn_rx_buffer, zxm2m_conn_rx_len);
            }
          }
       
          zxm2m_socket.error_cnt = 0x00; // 清除错误计数
          zxm2m_socket.error_flag = FALSE;
          zxm2m_socket.hb_10ms_timer = zxm2m_socket.hb_10ms_timer_sp;
//#if SOCKET_DEBUG
//        PcDebug_Printf("RecvSkt:Id=%d\r\n",zxm2m_socket.socket_id);
//#endif
        }
      }
      else  // 长度为0表示socket关闭
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
  pthread_create(&pthreads[PTHREAD_ZXM2M_PRODUCE_ID], NULL, pthread_ZxM2mProduce, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_PROCESS_ID], NULL, pthread_ZxM2mProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_SOCKET_SERVICE_ID], NULL, pthread_ZxM2mSocketService, NULL);
  usleep(10);
}

/******************************************************************************
* 下面函数由用户编写,用于底层网络连接
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
  (*hjep_socket.reinit_server_addr)();
}

//============================================================================
void* pthread_HjepSocketService(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期
    NetSocket_Service(&hjep_socket);
  }
}

//==重型M2M发送数据线程=======================================================
void* pthread_HjepProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期
    HJEP_ProduceSendData();
  }
}

//==重型M2M接收数据处理线程====================================================
void* pthread_HjepProcess(void *argument)
{
  while (1)
  {
    if ((NetSocket_GetLinkState(&hjep_socket)==SOCKET_LINK_STATE_CLOSED) || (HjepSocketFd<0))
    {
      msleep(10);
    }
    else
    {
      hjep_conn_rx_len = recv(HjepSocketFd, hjep_conn_rx_buffer, (SOCKET_CONN_MAX_DATA_LEN-1), 0);
      if (hjep_conn_rx_len < 0)
      {
        if (errno == EAGAIN)
        {
          printf("RE-Len:%d errno EAGAIN\n", hjep_conn_rx_len);
          continue;
        }

        if (errno == EINTR)
        {
          continue;
        }

        perror("recv error\n");
//#if SOCKET_DEBUG
//      PC_DebugPrintf("RecvSocketErr:Id=%d\r\n",hjep_socket.socket_id);
//#endif
      }
      else if (hjep_conn_rx_len > 0)
      {
        if (hjep_conn_rx_len < SOCKET_CONN_MAX_DATA_LEN)
        {
          PcDebug_SendData(hjep_conn_rx_buffer, hjep_conn_rx_len, DBG_MSG_TYPE_SYS);
          //HJEP_ProcessRecvData(hjep_conn_rx_buffer, hjep_conn_rx_len);
          if((hjep_conn_rx_buffer[0]==0x23) && (hjep_conn_rx_buffer[1]==0x23)) // 帧头
          {
            hjep_socket.error_cnt = 0x00; // 清除错误计数
            hjep_socket.error_flag = FALSE;
            hjep_socket.hb_10ms_timer = hjep_socket.hb_10ms_timer_sp;
          }
//#if SOCKET_DEBUG
//        PcDebug_Printf("RecvSkt:Id=%d\r\n",hjep_socket.socket_id);
//#endif
        }
      }
      else  // 长度为0表示socket关闭
      {
        NetSocket_SetLinkState(&hjep_socket, SOCKET_LINK_STATE_CLOSED);
#if SOCKET_DEBUG
        PcDebug_Printf("DiscSkt:Id=%d\r\n", hjep_socket.socket_id);
#endif
      }
    }
    msleep(1);
  }
}

//===========================================================================
void HJEP_ServiceInit(void)
{
  HJEP_Initialize();
  HJEP_NetSocketInit();
}

//============================================================================
void HJEP_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_HJEP_PRODUCE_ID], NULL, pthread_HjepProduce, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_HJEP_PROCESS_ID], NULL, pthread_HjepProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_HJEP_SOCKET_SERVICE_ID], NULL, pthread_HjepSocketService, NULL);
  usleep(10);
}

/***************************************************************************
* 100ms调用一次(此函数属于应用层,用户根据实际情况实现)
****************************************************************************/
void Net_CheckIsModemError(void)
{
  if (hjep_socket.enable_flag == SOCKET_FALSE)
  {
    if (zxm2m_socket.error_flag==TRUE) // 平台都连接不上,重启模块
    {
      zxm2m_socket.error_flag = FALSE;
      Modem_SetState(MODEM_STATE_MINI_FUN); // 重启模块
    }
  }
  else
  {
    if ((zxm2m_socket.error_flag==TRUE) && (hjep_socket.error_flag==TRUE)) // 所有平台都连接不上,重启模块
    {
      zxm2m_socket.error_flag = FALSE;
      hjep_socket.error_flag = FALSE;
      Modem_SetState(MODEM_STATE_MINI_FUN); // 重启模块
    }
  }
}

