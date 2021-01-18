/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Mcu.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-19
* @brief
******************************************************************************/
//-----ͷ�ļ�����-------------------------------------------------------------
#include "config.h"
#include "CanHW.h"

/******************************************************************************
 * RTOS
 ******************************************************************************/
#define AppThreadPriority_Can1Recv   osPriorityHigh4
osThreadId_t tid_Can1Recv;

const osThreadAttr_t AppThreadAttr_Can1Recv =
{
  .priority = AppThreadPriority_Can1Recv,
  .attr_bits = osThreadDetached,
  .stack_size = 512, // �ֽ�
};

#define AppThreadPriority_Can2Recv   osPriorityHigh3
osThreadId_t tid_Can2Recv;

const osThreadAttr_t AppThreadAttr_Can2Recv =
{
  .priority = AppThreadPriority_Can2Recv,
  .attr_bits = osThreadDetached,
  .stack_size = 512, // �ֽ�
};

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// ��������
lvc_context_t lvc_context = {
  .header = 0x55AA5AA5,     // У���ֽ�  
  .lock_cmd_flag = 0x00,    // ƽ̨�·��������Ͱ������ʶ
  .report_srv_flag = 0x00,  // �ϱ�ƽ̨��־
  .lock_command = 0x00,     // ��������:  0:����, 1~3:��������
  .bind_command = 0x00,     // ������,0:���κ�����, 0xaa:��, 0x55:���
  .lock_level1_sn = 0x00,   // ƽ̨�·���һ��������ˮ��
  .lock_level2_sn = 0x00,   // ƽ̨�·��Ķ���������ˮ��
  .unlock_sn = 0x00,        // ƽ̨�·��Ľ�����ˮ��
  .bind_sn = 0x00,          // ƽ̨�·������ü��ģʽ��ˮ��
  .ecu_rsp_lock_state = 0x00, // ECU����������״̬:1=����, 0=����
  .ecu_rsp_bind_state = 0x00, // ECU�����İ�״̬:1=��, 0=�����
};

// CAN����
can_context_t can_context = {
  .ep_valid_flag = 0, // ����������Ч��־: 0=��Ч, 1=��Ч
  .mil_lamp = 0,      // ���ϵ�״̬:0:δ����, 1=����
  .engine_speed = 0x00, // ������ת��

  .comm_state1 = 0,  // ͨ��״̬:0=�쳣, 1=����
  .recv_state1 = 0,  // ����״̬:0=δ�յ�����, 1=���յ�����
  .recv_timer1 = CAN1_RECV_TIMEOUT_SP, // ���ճ�ʱ������

  .comm_state2 = 0,  // ͨ��״̬:0=�쳣, 1=����
  .recv_state2 = 0,  // ����״̬:0=δ�յ�����, 1=���յ�����
  .recv_timer2 = CAN2_RECV_TIMEOUT_SP, // ���ճ�ʱ������

  .sleep_state = 0, // ����״̬,0=δ����, 1=����
};

obd_info_t obd_info;
dtc_context_t dtc_1939;
dtc_context_t dtc_27145;

can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];

can_msg_queue_t can1_msg_queue;
can_msg_queue_t can2_msg_queue;

#define ACC_ON_TIMEOUT_SP     30  //ͨ���쳣�ж�ʱ��
/**********************************************************************************
* MCUͨ��״̬�жϣ��˺���1sִ��һ��
***********************************************************************************/
void Can_CheckCommState(void)
{
  static uint8_t acc_current_state = 0;
  static uint8_t acc_previous_state = 0;
  static uint16_t acc_on_timer = ACC_ON_TIMEOUT_SP; // 30��
  static uint8_t reset_can_timer = 0;  // 5����
  static uint8_t reset_can_flag = 0;     // 0=������������CAN��1=��������������CAN

  acc_current_state = COLT_GetAccStatus();
  if (acc_current_state != acc_previous_state)
  {
    if (acc_current_state==0) // ACC�ر��¼�
    {
      acc_on_timer = ACC_ON_TIMEOUT_SP;
      reset_can_flag = 0;
    }
  }

  if (acc_current_state==1) // ACC��
  {
    if (acc_on_timer)
    {
      acc_on_timer--;
    }

    if ((acc_on_timer==0) && (can_context.recv_state1==0) && (can_context.recv_state2==0) && (reset_can_flag==0))
    {
      reset_can_flag = 1;
      reset_can_timer = 5;
      CAN_POWER_OFF();  // �շ����������״̬
      can_context.comm_state1 = CAN_NOK; //CANͨѶ�쳣
      PcDebug_SendString("Can Off!\n");
    }

    if (reset_can_timer)
    {
      reset_can_timer--;
      if (reset_can_timer==0)
      {
        CAN_Initialize();  // ���¶�CANģ���ϵ�ͳ�ʼ��
        PcDebug_SendString("Can Rest!\n");
      }
    }
  }

  acc_previous_state = acc_previous_state;
}

/**********************************************************************************
 * can�ӿ�û���յ����ݵ���ʱ(100ms����)
 ***********************************************************************************/
void iCan_DebounceRecvState(void)
{
  if (can_context.recv_timer1)
    can_context.recv_timer1--;
  else
  {
    can_context.recv_state1 = CAN_NOK; // δ�յ�CAN1����
  }

  if (can_context.recv_timer2)
    can_context.recv_timer2--;
  else
  {
    can_context.recv_state2 = CAN_NOK; // δ�յ�CAN2����
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DTC����
////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************************************************
* ��ȡ�Ƿ����µ�DTC��Ҫ�ϱ�
*************************************************************************/
uint8_t DTC_GetNewFlag(dtc_context_t* pThis)
{
  return pThis->new_flag;
}

/************************************************************************
* �����DTC��־
*************************************************************************/
void DTC_ClearNewFlag(dtc_context_t* pThis)
{
  pThis->new_flag = 0;
}

/******************************************************************************************
* �յ�CAN�����������������������б��Ƿ��иù����룬���������¸ù�����ļ�ʱ,
* ���û�У�����Ϊ���²����Ĺ����룬��ӵ�����������б�,������;
* ����б�������������ֱ���б��пյ�Ԫ
* ����: 0-û�������������������б�������1-�����˹�����
*******************************************************************************************/
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode)
{
  uint8_t i;

  if (dtcode == 0x00000000)
  {
    return 0;
  }

  dtcode &= 0x00FFFFFF;
  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i] == dtcode) // ��ʷ������
    {
      pThis->debounce_tmr[i] = DTC_DEBOUNCE_TIME_SP;
      return 0;
    }
  }

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++) // ����������
  {
    if (pThis->code[i] == 0)
    {
      pThis->code[i] = dtcode;
      pThis->debounce_tmr[i] = DTC_DEBOUNCE_TIME_SP;
      pThis->total_num++;
      //PcDebug_Printf("SaveNewDTC!\r\n");
      return 1;
    }
  }
  return 0;	// �б�����
}

/*****************************************************************************************
* �˺���ÿ1������һ�Σ��Թ������б��й�������ڵ�ʱ����е���ʱ������ʱ��0ʱ��
* ��Ϊ�ù������Ѿ���ʧ���Ըù���������
* DTC(Diagnostic Trouble Code)
******************************************************************************************/
void DTC_DebounceCode(dtc_context_t* pThis)
{
  uint8_t i;

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i])
    {
      if (pThis->debounce_tmr[i])
        pThis->debounce_tmr[i]--;
      else
      {
        pThis->code[i] = 0;
        pThis->total_num--;
        //PcDebug_Printf("ClearOldDTC!\r\n");
      }
    }
  }
}

/******************************************************************************************
* ��ȡ�������������
* pBuf - ָ�������������ݻ����ָ��
* ����: ��������������0��ʾû�й������������
******************************************************************************************/
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf)
{
  uint8_t i;
  uint8_t dtc_cnt=0;

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i])
    {
      *pBuf++ = pThis->code[i] & 0xFF;
      *pBuf++ = (pThis->code[i] >> 8) & 0xFF;
      *pBuf++ = (pThis->code[i] >> 16) & 0xFF;
      *pBuf++ = (pThis->code[i] >> 24) & 0xFF;
      dtc_cnt++;
    }
  }

  if (dtc_cnt == 0)
  {
    pThis->total_num = 0x00;
  }
  //PcDebug_Printf("GetDTC:%d!\r\n",dtc_cnt);
  return dtc_cnt;
}

//========================================================================================
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis)
{
  return pThis->total_num;
}

/**********************************************************************************
 * ������յ���CAN����(SAE J1939������)
**********************************************************************************/
uint8_t iCan_ProcessDtcMsg_J1939(uint32_t canId,uint8_t *pdata,uint8_t size)
{
  uint8_t i;
  uint8_t j = 0;
  uint8_t retval = 0x00;
  uint32_t dtc;
  static uint8_t dtc_total;
  static uint8_t mfault_packages = 0;  // ECM�ְ�����
  static uint8 mfault_index = 0;   // ECM���������֡����
  static uint8_t torque_flag = 0;  // �ο�Ť�ؽ��ձ�־0-�� 1-��
  static uint8_t dm1_valid_flag = 0; // �յ�����Ϲ㲥֡

  switch (canId)
  {
  case 0x18FECA00:  // ��������(0x18FECA00)
    //ep_data_buffer[EP_POS24_ADDRESS] = (pdata[0]>>6) & 0x03; // MIL��״̬(B1.bit7-8)

    dtc = pdata[2] + (uint32_t)(pdata[3]<<8) + (uint32_t)(pdata[4]<<16) + (uint32_t)(pdata[5]<<24);
    if ((dtc!=0xFFFFFFFF) && (dtc!=0x00000000))
    {
      DTC_SaveCode(&dtc_1939, dtc);
    }
    retval = 0x01;
    break;

  case 0x18ECFF00: // ����������жϹ��ϱ���(�㲥֡) (0x18ECFF00)
    if (pdata[5]==0xCA && pdata[6]==0xFE && pdata[7]==0x00) // ����������Э��PGN65226
    {
      dtc_total = (pdata[1] - 2) / 4; // ���ϴ�������
      mfault_packages = pdata[3];  //����֡CAN֡����
      mfault_index = 1;
      dm1_valid_flag = 1;
    }
    else if (pdata[5]==0xE3 && pdata[6]==0xFE && pdata[7]==0x00) // ����������ͨ��PGN65251
    {
      torque_flag = 1;
    }
    retval = 0x01;
    break;

  case 0x18EBFF00: // ������������ݱ���   (0x18EBFF00)
    if (torque_flag == 1 && pdata[0] == 3) // ����������package3
    {
      torque_flag = 0;   //���ս���
    }

    if (dm1_valid_flag == 0) break; // û���յ��������㲥֡

    if (pdata[0]==mfault_index && dm1_valid_flag==1) // �յ�����Ϸְ�
    {
      if (pdata[0] == 1) // ��֡����
      {
        //ep_data_buffer[EP_POS24_ADDRESS] = (pdata[1]>>6) & 0x03; // MIL��״̬(B1.bit7-8)
        memset(dtc_1939.buffer, 0xFF, DTC_BUFFER_SIZE); // ���DTC����
        memcpy(dtc_1939.buffer,&pdata[3],5);
        dtc_1939.index = 5;
      }
      else // ����֡����
      {
        if (dtc_1939.index < (DTC_BUFFER_SIZE-7))
        {
          memcpy(&dtc_1939.buffer[dtc_1939.index],&pdata[1],7);
          dtc_1939.index += 7;
        }
      }

      mfault_index++;
      if (mfault_index > mfault_packages) //���ͽ���
      {
        for (i=0; i<dtc_total; i++)
        {
          dtc = (uint32_t)(dtc_1939.buffer[j]) + (uint32_t)(dtc_1939.buffer[j+1]<<8) + (uint32_t)(dtc_1939.buffer[j+2]<<16) + (uint32_t)(dtc_1939.buffer[j+3]<<24);
          j += 4;

          if ((dtc!=0xFFFFFFFF) && (dtc!=0x00000000))
          {
            uint8_t val = DTC_SaveCode(&dtc_1939, dtc); // ���������

            if (val == 1) // �¹�����
            {
              dtc_1939.new_flag = 1;
            }
          }
        }

        mfault_index = 0;
        mfault_packages = 0;
        dm1_valid_flag = 0;
      }
    }
    retval = 0x01;
    break;

  default:
    break;
  }

  return retval;
}

//==��λ����ֵΪ��Ч==============================================================
void CAN_ResetObdToDefault(void)
{
  obd_info.mil_status = 0xfe;

  obd_info.diag_valid_flag = 0x00;
  obd_info.diag_supported_status = 0x00;
  obd_info.diag_supported_status = 0x00;

  obd_info.vin_valid_flag = 0;
  memset(obd_info.vin, 0x00, 17);

  obd_info.calid_valid_flag = 0;

  memset(obd_info.calid, 0xFF, 18);

  obd_info.cvn_valid_flag = 0;

  memset(obd_info.cvn, 0xFF, 18);

  obd_info.iupr_valid_flag = 0;

  memset(obd_info.iupr, 0xFF, 36);

  obd_info.dtc_cnt = 0x00;
  memset(obd_info.dtc, 0x00, sizeof(obd_info.dtc));
}

//==��λCAN֡�б�=================================================================
void CAN_ResetFrameTable(void)
{
  uint8_t it;

  for (it=0; it<MAX_CAN_FRAME_NUM; it++)
  {
    can_frame_table[it].id = 0x00;
    memset(can_frame_table[it].data, 0x00, 8);
  }
}

/**********************************************************************************
*Description	:MCUģ���ʱ�������˺���10msִ��һ��
**********************************************************************************/
#if 0
void  MCU_TimerCount_Delay(void)
{
  static uint8 usTime500ms = 4;
  static uint8 usTime1s = 10;

  if (usTime500ms)
  {
    if (!(--usTime500ms))
    {
      usTime500ms = 4;
      //SendCan1Message();//�˴����2������һ�εĺ���
    }
  }

  SendLockCan1Message_10ms();
  if (usTime1s)
  {
    if (!(--usTime1s))
    {
      usTime1s=10;
      SendCan1Message();	//100MS
    }
  }
}
#endif

/**********************************************************************************
* ����CAN����(��������/��������/���OBD����)(100ms����)
***********************************************************************************/
void CAN_ProduceSendMsg(void)
{

}

/******************************************************************************
* ���CAN��������
******************************************************************************/
void can_msg_queue_reset(can_msg_queue_t* pThis)
{
  pThis->head = 0;
  pThis->tail = 0;
}

/******************************************************************************
* ��һ�����ݴ��������
******************************************************************************/
void can_msg_queue_push(can_msg_queue_t* pThis, CanRxMsg* pMsg)
{
  uint8_t pos;

  pos = (pThis->head + 1) % CAN_MSG_QUEUE_MAX_SIZE;
  if (pos != pThis->tail) // ����״̬
  {
    // ���ƽṹ��
    pThis->msg_buff[pThis->head].StdId = pMsg->StdId;
    pThis->msg_buff[pThis->head].ExtId = pMsg->ExtId;
    pThis->msg_buff[pThis->head].IDE = pMsg->IDE;
    pThis->msg_buff[pThis->head].RTR = pMsg->RTR;
    pThis->msg_buff[pThis->head].DLC = pMsg->DLC;
    pThis->msg_buff[pThis->head].FMI = pMsg->FMI;
    pThis->msg_buff[pThis->head].Data[0] = pMsg->Data[0];
    pThis->msg_buff[pThis->head].Data[1] = pMsg->Data[1];
    pThis->msg_buff[pThis->head].Data[2] = pMsg->Data[2];
    pThis->msg_buff[pThis->head].Data[3] = pMsg->Data[3];
    pThis->msg_buff[pThis->head].Data[4] = pMsg->Data[4];
    pThis->msg_buff[pThis->head].Data[5] = pMsg->Data[5];
    pThis->msg_buff[pThis->head].Data[6] = pMsg->Data[6];
    pThis->msg_buff[pThis->head].Data[7] = pMsg->Data[7];

    pThis->head = pos; // ����ͷ��
  }
}

/******************************************************************************
* �Ӷ�����ȡһ������
******************************************************************************/
static void can_msg_queue_pop(can_msg_queue_t* pThis, can_msg_t* pMsg)
{
  if (pThis->tail != pThis->head) //�ǿ�״̬
  {
    if (pThis->msg_buff[pThis->tail].IDE == CAN_Id_Extended)
    {
      pMsg->id = pThis->msg_buff[pThis->tail].ExtId;
      pMsg->type = CAN_FRAME_TYPE_EXT;
    }
    else
    {
      pMsg->id = pThis->msg_buff[pThis->tail].StdId;
      pMsg->type = CAN_FRAME_TYPE_STD;
    }
    pMsg->len = pThis->msg_buff[pThis->tail].DLC ;
    pMsg->data[0] = pThis->msg_buff[pThis->tail].Data[0];
    pMsg->data[1] = pThis->msg_buff[pThis->tail].Data[1];
    pMsg->data[2] = pThis->msg_buff[pThis->tail].Data[2];
    pMsg->data[3] = pThis->msg_buff[pThis->tail].Data[3];
    pMsg->data[4] = pThis->msg_buff[pThis->tail].Data[4];
    pMsg->data[5] = pThis->msg_buff[pThis->tail].Data[5];
    pMsg->data[6] = pThis->msg_buff[pThis->tail].Data[6];
    pMsg->data[7] = pThis->msg_buff[pThis->tail].Data[7];

    pThis->tail = (pThis->tail + 1) % CAN_MSG_QUEUE_MAX_SIZE; // ����β��
  }
}

/******************************************************************************
* ��ȡ��������Ч���ݸ���
******************************************************************************/
static uint8_t can_msg_queue_size(can_msg_queue_t* pThis)
{
  return ((pThis->head + CAN_MSG_QUEUE_MAX_SIZE - pThis->tail) % CAN_MSG_QUEUE_MAX_SIZE);
}


/**********************************************************************************
* ������յ�������CAN1(�³�)�ı���
***********************************************************************************/
void CAN1_ProcessRecvMsg(can_msg_t* pMsg)
{
  uint8_t buff[8];
  uint8_t retval = 0x00;

  //==����ʽ�³�====================================================================
  retval = CAN_ProcessRecvDownMsg_AC(pMsg->id, pMsg->data, pMsg->len); // �³���������
  if (retval==0x01)
  {
    return;
  }

  retval = CAN_ProcessRecvEngineMsg_AC(pMsg->id, pMsg->data, pMsg->len); // �³�����������
  if (retval==0x01)
  {
    return;
  }
  
  //==ȫ�����³�====================================================================
  retval = CAN_ProcessRecvDownMsg_AG(pMsg->id, pMsg->data, pMsg->len); // �³���������
  if (retval==0x01)
  {
    return;
  }

  retval = CAN_ProcessRecvEngineMsg_AG(pMsg->id, pMsg->data, pMsg->len); // �³�����������
  if (retval==0x01)
  {
    return;
  }

  switch (pMsg->id)
  {
  case 0x1fffffff://���� ID  31 61 32 62 33 63 34 64
    if ((pMsg->data[0]==0x31)&&(pMsg->data[1]==0x61)&&(pMsg->data[2]==0x32)&&(pMsg->data[3]==0x62)&&(pMsg->data[4]==0x33)&&(pMsg->data[5]==0x63)&&(pMsg->data[6]==0x34)&&(pMsg->data[7]==0x64))
    {
      buff[0] = 101;
      buff[1] = 53;
      buff[2] = 102;
      buff[3] = 54;
      buff[4] = 103;
      buff[5] = 55;
      buff[6] = 104;
      buff[7] = 56;
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x1fffffff, 8, buff);
    }
    break;

  default:
    break;
  }
}

#define can1MsgQueueENTER_CRITICAL()  do{CAN_ITConfig(CAN1,CAN_IT_FMP0, DISABLE);CAN_ITConfig(CAN1,CAN_IT_FMP1, DISABLE);}while(0)
#define can1MsgQueueEXIT_CRITICAL()   do{CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);CAN_ITConfig(CAN1,CAN_IT_FMP1, ENABLE);}while(0)
/******************************************************************************
* ����MCUģ�������(����)
*******************************************************************************/
void AppThread_Can1Recv(void *argument)
{
  can_msg_t can_msg;

  while (1)
  {
    if (CAN_OK==CAN1_ReceiveMessage(2*OS_TICKS_PER_SEC)) // �ȴ�CAN1�ź���(����)
    {
      while (can_msg_queue_size(&can1_msg_queue) > 0) // if
      {
        can1MsgQueueENTER_CRITICAL(); // �رս����ж�
        can_msg_queue_pop(&can1_msg_queue, &can_msg);   // ȡһ������(���˺�)
        can1MsgQueueEXIT_CRITICAL();  // ���������ж�
        CAN1_ProcessRecvMsg(&can_msg);
      }
    }
  }
}

/**********************************************************************************
* ������յ�������CAN2(�ϳ�)�ı���
***********************************************************************************/
void CAN2_ProcessRecvMsg(can_msg_t* pMsg)
{
  uint8_t buff[8];
  uint8_t retval = 0x00;

  retval = CAN_ProcessRecvUpMsg(pMsg->id, pMsg->data, pMsg->len); // �ϳ�����
  if (retval==0x01)
  {
    return;
  }

  switch (pMsg->id)
  {
  case 0x1ffffffe://���� ID 31 61 32 62 33 63 34 64
    if ((pMsg->data[0]==0x31)&&(pMsg->data[1]==0x61)&&(pMsg->data[2]==0x32)&&(pMsg->data[3]==0x62)&&(pMsg->data[4]==0x33)&&(pMsg->data[5]==0x63)&&(pMsg->data[6]==0x34)&&(pMsg->data[7]==0x64))
    {
      buff[0] = 101;
      buff[1] = 53;
      buff[2] = 102;
      buff[3] = 54;
      buff[4] = 103;
      buff[5] = 55;
      buff[6] = 104;
      buff[7] = 56;
      CAN2_TransmitData(CAN_FRAME_TYPE_EXT, 0x1ffffffe, 8, buff);
    }
    break;

  default:
    break;
  }
}

#define can2MsgQueueENTER_CRITICAL()  do{CAN_ITConfig(CAN2,CAN_IT_FMP0, DISABLE);CAN_ITConfig(CAN2,CAN_IT_FMP1, DISABLE);}while(0)
#define can2MsgQueueEXIT_CRITICAL()   do{CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);CAN_ITConfig(CAN2,CAN_IT_FMP1, ENABLE);}while(0)
/******************************************************************************
* ����MCUģ�������(����)
*******************************************************************************/
void AppThread_Can2Recv(void *argument)
{
  can_msg_t can_msg;

  while (1)
  {
    if (CAN_OK==CAN2_ReceiveMessage(2*OS_TICKS_PER_SEC)) // �ȴ�CAN2�ź���(����)
    {
      while (can_msg_queue_size(&can2_msg_queue) > 0) // if
      {
        can2MsgQueueENTER_CRITICAL(); // �رս����ж�
        can_msg_queue_pop(&can2_msg_queue, &can_msg);   // ȡһ������(���˺�)
        can2MsgQueueEXIT_CRITICAL();  // ���������ж�
        CAN2_ProcessRecvMsg(&can_msg);
      }
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void Can_ServiceInit(void)
{
  can_msg_queue_reset(&can1_msg_queue);
  can_msg_queue_reset(&can1_msg_queue);
  zxsts_Initialize();
  CAN_GpioInitialize(); 
  CAN_Initialize();
}

/*************************************************************************
 *
*************************************************************************/
void Can_ServiceStart(void)
{
  tid_Can1Recv = osThreadNew(AppThread_Can1Recv, NULL, &AppThreadAttr_Can1Recv);
  tid_Can2Recv = osThreadNew(AppThread_Can2Recv, NULL, &AppThreadAttr_Can2Recv);
}

/**********************************************************************************
* 100msִ��һ��(������)
***********************************************************************************/
void Can_Do100msTasks(void)
{
  iCan_DebounceRecvState();
  zxsts_StateMachine();
}

/**********************************************************************************
* 1sִ��һ��(������)
***********************************************************************************/
void Can_Do1sTasks(void)
{
  Can_CheckCommState();
  DTC_DebounceCode(&dtc_1939);
  DTC_DebounceCode(&dtc_27145);
}

/**********************************************************************************
* ��ȡGPS�ն���MCU��ͨ��״̬
***********************************************************************************/
uint8_t CAN_GetCommState(uint8_t channel)
{
  if (CAN_CHANNEL1==channel)
  {
    return can_context.comm_state1;
  }
  else if (CAN_CHANNEL2==channel)
  {
    return can_context.comm_state2;
  }

  return CAN_NOK;
}

/**********************************************************************************
* ��ȡCAN����״̬,1:�յ���CAN1���ݣ�2:û���յ�CAN1����
***********************************************************************************/
uint8_t CAN_GetRecvState(uint8_t channel)
{
  if (CAN_CHANNEL1==channel)
  {
    return can_context.recv_state1;
  }
  else if (CAN_CHANNEL2==channel)
  {
    return can_context.recv_state2;
  }

  return CAN_NOK;
}

//==��ȡ����������Ч��=============================================================
uint8_t CAN_GetVinState(void)
{
  if ((m2m_asset_data.vin_valid_flag == 0x01) || (obd_info.vin_valid_flag==0x01))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//==��ȡ����������=================================================================
uint8_t CAN_GetEngineType(void)
{
  return m2m_asset_data.ecu_type;
}

//==��ȡ������ת��=================================================================
uint16_t CAN_GetEngineSpeed(void)
{
  return can_context.engine_speed;
}

//-----�ļ�Mcu.c����---------------------------------------------
