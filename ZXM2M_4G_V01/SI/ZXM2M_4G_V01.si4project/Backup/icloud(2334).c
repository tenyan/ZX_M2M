/*****************************************************************************
* @FileName: icloud_machine.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-11-16
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
#define ICLOUD_DELAY(ms)    do { osDelay(ms); } while(0)

/******************************************************************************
* Data Types and Globals
******************************************************************************/
// 总线信息
sysbus_msg_t SbusMsg_Collect={SYSBUS_DEVICE_TYPE_COLLECT, 0, 0, 0, 0, 0};
sysbus_msg_t SbusMsg_GpsWarning={SYSBUS_DEVICE_TYPE_GPS, 0, 0, 0, 0, 0};
sysbus_msg_t SbusMsg_Gprs={SYSBUS_DEVICE_TYPE_CELLURA, CELLURA_MSG_TYPE_DATA, 0, 0, 0, 0};
sysbus_msg_t SbusMsg_Sms={SYSBUS_DEVICE_TYPE_CELLURA, CELLURA_MSG_TYPE_SMS, 0, 0, 0, 0};
sysbus_msg_t SbusMsg_Ftp={SYSBUS_DEVICE_TYPE_CELLURA, CELLURA_MSG_TYPE_FTP, 0, 0, 0, 0};
sysbus_msg_t SbusMsg_PcDebug={SYSBUS_DEVICE_TYPE_PC_DEBUG,1,0,0,0,0};
sysbus_msg_t SbusMsg_Can={SYSBUS_DEVICE_TYPE_CAN,0,0,0,0,0};

uint8_t net_public_data_buffer[1460];

/******************************************************************************
 * RTOS相关
 ******************************************************************************/
#define SIZE_OF_SYSTEM_BUS_QUEUE  10  // 队列大小

#define AppThreadPriority_iCloud  osPriorityLow   // osPriorityNormal7  
osThreadId_t tid_iCloud;
const osThreadAttr_t AppThreadAttr_iCloud =
{
  .priority = AppThreadPriority_iCloud,
  .attr_bits = osThreadDetached,
  .stack_size = 2048, // 字节
};

osMessageQueueId_t mqid_SysBusMbox=NULL;

/******************************************************************************
 *
*******************************************************************************/
void SysBus_ProcessCelluraMsg(sysbus_msg_t* pThis)
{
  uint16_t frame_len = 0;
  
  switch (pThis->type)
  {
  case CELLURA_MSG_TYPE_DATA: // 网络消息
    frame_len = frame_len;
    m2m_context.rx_size = pThis->data_size;
    m2m_context.rx_data = pThis->data;
    m2m_context.rx_from = SYSBUS_DEVICE_TYPE_CELLURA;
    frame_len = M2M_ProcessRecvMsg(&m2m_context);  
    break;

  case CELLURA_MSG_TYPE_SMS: // 短信消息
    osDelay(200);
    //usDataLen = SYS_SMS_CommandAll_Execution_Universal(&msg);
    break;

  case CELLURA_MSG_TYPE_FTP:  // FTP
    //SysBus_ProcessGpsMsg(pThis);
    break;

  case CELLURA_MSG_TYPE_RING: // 电话
    //SysBus_ProcessCollectMsg(pThis);
    break;

  case CELLURA_MSG_TYPE_ONLINE:  // 上线
    //SysBus_ProcessCanMsg(pThis);
    break;

  default:
    break;
  }
}

//============================================================================
void SysBus_ProcessPcDebugMsg(sysbus_msg_t* pThis)
{
  uint16_t frame_len = 0;
  
  frame_len = frame_len;
  m2m_context.rx_size = pThis->data_size;
  m2m_context.rx_data = pThis->data;
  m2m_context.rx_from = SYSBUS_DEVICE_TYPE_PC_DEBUG;
  frame_len = M2M_ProcessRecvMsg(&m2m_context);
}

//============================================================================
void SysBus_ProcessCanMsg(sysbus_msg_t* pThis)
{
  switch (pThis->type)
  {
  case CAN_MSG_TYPE_CAN1_COMM_OK:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BUS_COMM_ERR, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;

  case CAN_MSG_TYPE_CAN1_COMM_ERR:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BUS_COMM_ERR, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;

  case  CAN_MSG_TYPE_CAN1_RECV_START:
  case  CAN_MSG_TYPE_CAN2_RECV_START:
    //M2M_AddNewAlarmToList(5, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 1;
    break;
  
  case  CAN_MSG_TYPE_CAN1_RECV_STOP:
  case  CAN_MSG_TYPE_CAN2_RECV_STOP:
    //M2M_AddNewAlarmToList(5, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;

  default:
    break;
  }
}

//============================================================================
void SysBus_ProcessGpsMsg(sysbus_msg_t* pThis)
{
  switch (pThis->type)
  {
  case GPS_MSG_TYPE_OVER_SPEED:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_OVER_SPEED, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case GPS_MSG_TYPE_NORMAL_SPEED:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_OVER_SPEED, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;
    
  case GPS_MSG_TYPE_ANTENNA_ERR:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_ANT_ERR, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case GPS_MSG_TYPE_ANTENNA_OK:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_ANT_ERR, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;

  case GPS_MSG_TYPE_MODULE_ERR:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_MODULE_ERR, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case GPS_MSG_TYPE_MODULE_OK:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_MODULE_ERR, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;

  case GPS_MSG_TYPE_3D_NOK:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_SQ_LOW, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case GPS_MSG_TYPE_3D_OK:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_GPS_SQ_LOW, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;

  default:
    break;
  }
}

//============================================================================
void SysBus_ProcessCollectMsg(sysbus_msg_t* pThis)
{
  switch (pThis->type)
  {
  case COLLECT_MSG_TYPE_ACC_ON:
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_ACC_OFF:
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_BOX_OPENED:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BOX_OPEN, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_BOX_CLOSED:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BOX_OPEN, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_MAIN_POWER_LOW:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_POWER_LOW, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_MAIN_POWER_NORMAL:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_POWER_LOW, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_MAIN_POWER_ON:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_POWER_OFF, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_MAIN_POWER_OFF:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_POWER_OFF, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;

  case COLLECT_MSG_TYPE_MAIN_BAT_LOW:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BAT_LOW, ALARM_TRUE);
    m2m_context.ss_req.send_timer = 0;
    break;
  case COLLECT_MSG_TYPE_MAIN_BAT_NORMAL:
    M2M_AddNewAlarmToList(M2M_ALARM_TYPE_BAT_LOW, ALARM_FALSE);
    m2m_context.ss_req.send_timer = 0;
    break;

  default:
    break;
  }
}

//============================================================================
void SysBus_ProcessMsg(sysbus_msg_t* pThis)
{
  switch (pThis->device)
  {
  case SYSBUS_DEVICE_TYPE_CELLURA: // 蜂窝网模块
    SysBus_ProcessCelluraMsg(pThis);
    break;

  case SYSBUS_DEVICE_TYPE_PC_DEBUG: // 配置工具
    SysBus_ProcessPcDebugMsg(pThis);
    break;

  case SYSBUS_DEVICE_TYPE_CAN:  // CAN模块
    SysBus_ProcessCanMsg(pThis);
    break;

  case SYSBUS_DEVICE_TYPE_GPS:  // GPS模块
    SysBus_ProcessGpsMsg(pThis);
    break;

  case SYSBUS_DEVICE_TYPE_COLLECT: // 采集模块
    SysBus_ProcessCollectMsg(pThis);
    break;

  default:
    break;
  }
}

/******************************************************************************
 * iCloud模块任务函数
*******************************************************************************/
void AppThread_iCloud(void *argument)
{
  sysbus_msg_t msg;
  osStatus_t status;

  while (1)
  {
    status = SYSBUS_GetMbox(msg,10); // 等待消息(超时10ms)
    if (status == osOK) //对接收到的消息数据进行处理
    {
      SysBus_ProcessMsg(&msg);
    }
    HJEP_ProduceSendData();
    //GBEP_SendNetData();
    M2M_ProduceSendMsg(&m2m_context);
  }
}

/*************************************************************************
 *
*************************************************************************/
void iCloud_ServiceInit(void)
{
  if(mqid_SysBusMbox==NULL)
  {
    mqid_SysBusMbox = osMessageQueueNew(SIZE_OF_SYSTEM_BUS_QUEUE, sizeof(sysbus_msg_t), NULL); // 创建消息邮箱
  }
  
  Parm_ReadM2mAssetData();
  Parm_ReadLvcInfo();
  //SysCounterInit();
  COLT_ReadInternalWdtState();
  M2M_Initialize();
}

/*************************************************************************
 *
*************************************************************************/
void iCloud_ServiceStart(void)
{
  tid_iCloud = osThreadNew(AppThread_iCloud, NULL, &AppThreadAttr_iCloud);
}

