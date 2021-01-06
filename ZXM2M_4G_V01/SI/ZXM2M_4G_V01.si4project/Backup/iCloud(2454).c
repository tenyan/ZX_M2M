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

uint8_t net_public_data_buffer[1460];


/******************************************************************************
 *
*******************************************************************************/


/******************************************************************************
* 下面函数由用户编写,用于底层网络连接
*******************************************************************************/
void M2M_ReInitServerAddr(void)
{
  zxm2m_socket.srv_protocol = m2m_asset_data.main_srv_protocol;
  zxm2m_socket.srv_port = m2m_asset_data.main_srv_port;
  sprintf((char *)zxm2m_socket.srv_ip, "%d.%d.%d.%d",m2m_asset_data.main_srv_ip[0],m2m_asset_data.main_srv_ip[1],m2m_asset_data.main_srv_ip[2],m2m_asset_data.main_srv_ip[3]);
}

uint8_t m2m_conn_tx_buffer[SOCKET_CONN_MAX_DATA_LEN];
//==========================================================================
void M2M_NetSocketInit(void)
{
  zxm2m_socket.socket_id = SOCKET_LINK_M2M;
  zxm2m_socket.state = NET_SOCKET_STATE_INVALID;
  zxm2m_socket.started_delay = SOCKET_FALSE;
  zxm2m_socket.state_transition = SOCKET_TRUE;
  zxm2m_socket.error_cnt = 0;
  zxm2m_socket.error_flag = SOCKET_FALSE;
  zxm2m_socket.enable_flag = SOCKET_TRUE;
  zxm2m_socket.link_state = SOCKET_LINK_STATE_CLOSED;

  zxm2m_socket.tx_buff = m2m_conn_tx_buffer;
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
    NetSocket_Service(&zx_socket);
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
    msleep(10); // 10ms周期
    
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
  pthread_create(&pthreads[PTHREAD_ZXM2M_PROCESS_ID], NULL, pthread_ZxM2mProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_PROCESS_ID], NULL, pthread_ZxM2mProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_ZXM2M_SOCKET_SERVICE_ID], NULL, pthread_ZxM2mSocketService, NULL);
  usleep(10);
}

/******************************************************************************
* 下面函数由用户编写,用于底层网络连接
*******************************************************************************/
//============================================================================
void* pthread_HjepSocketService(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期
    NetSocket_Service(&zxep_socket);
  }
}

//==重型M2M发送数据线程=======================================================
void* pthread_HjepProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期
    //HJEP_ProduceSendData();
  }
}

//==重型M2M接收数据处理线程====================================================
void* pthread_HjepProcess(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms周期

  }
}

//===========================================================================
void HJEP_ServiceInit(void)
{
  // HJEP_NetSocketInit(); // 国六环保数据初始化
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

