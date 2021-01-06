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
// ������Ϣ
sysbus_msg_t SbusMsg_PcDebug={SYSBUS_DEVICE_TYPE_PC_DEBUG,1,0,0,0,0};
uint8_t net_public_data_buffer[1460];

/******************************************************************************
 * RTOS���
 ******************************************************************************/
#define SIZE_OF_SYSTEM_BUS_QUEUE  10  // ���д�С

#define AppThreadPriority_iCloud  osPriorityLow   // osPriorityNormal7  
osThreadId_t tid_iCloud;
const osThreadAttr_t AppThreadAttr_iCloud =
{
  .priority = AppThreadPriority_iCloud,
  .attr_bits = osThreadDetached,
  .stack_size = 2048, // �ֽ�
};

osMessageQueueId_t mqid_SysBusMbox=NULL;

/******************************************************************************
 *
*******************************************************************************/
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
void SysBus_ProcessMsg(sysbus_msg_t* pThis)
{
  switch (pThis->device)
  {
  case SYSBUS_DEVICE_TYPE_PC_DEBUG: // ���ù���
    SysBus_ProcessPcDebugMsg(pThis);
    break;

  default:
    break;
  }
}

/******************************************************************************
 * iCloudģ��������
*******************************************************************************/
void AppThread_iCloud(void *argument)
{
  sysbus_msg_t msg;
  osStatus_t status;

  while (1)
  {
    status = SYSBUS_GetMbox(msg,10); // �ȴ���Ϣ(��ʱ10ms)
    if (status == osOK) //�Խ��յ�����Ϣ���ݽ��д���
    {
      SysBus_ProcessMsg(&msg);
    }
    //GBEP_SendNetData();
    //M2M_ProduceSendMsg(&m2m_context);
  }
}

/*************************************************************************
 *
*************************************************************************/
void iCloud_ServiceInit(void)
{
  if(mqid_SysBusMbox==NULL)
  {
    mqid_SysBusMbox = osMessageQueueNew(SIZE_OF_SYSTEM_BUS_QUEUE, sizeof(sysbus_msg_t), NULL); // ������Ϣ����
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

