/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Mcu.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-19
* @brief
******************************************************************************/
//-----头文件调用-------------------------------------------------------------
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
  .stack_size = 512, // 字节
};

#define AppThreadPriority_Can2Recv   osPriorityHigh3
osThreadId_t tid_Can2Recv;

const osThreadAttr_t AppThreadAttr_Can2Recv =
{
  .priority = AppThreadPriority_Can2Recv,
  .attr_bits = osThreadDetached,
  .stack_size = 512, // 字节
};

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// 锁车对象
lvc_context_t lvc_context = {
  .header = 0x55AA5AA5,     // 校验字节  
  .lock_cmd_flag = 0x00,    // 平台下发的锁车和绑定命令标识
  .report_srv_flag = 0x00,  // 上报平台标志
  .lock_command = 0x00,     // 锁车级别:  0:解锁, 1~3:锁车级别
  .bind_command = 0x00,     // 绑定命令,0:无任何命令, 0xaa:绑定, 0x55:解绑
  .lock_level1_sn = 0x00,   // 平台下发的一级锁车流水号
  .lock_level2_sn = 0x00,   // 平台下发的二级锁车流水号
  .unlock_sn = 0x00,        // 平台下发的解锁流水号
  .bind_sn = 0x00,          // 平台下发的设置监控模式流水号
  .ecu_rsp_lock_state = 0x00, // ECU反馈的锁车状态:1=锁车, 0=解锁
  .ecu_rsp_bind_state = 0x00, // ECU反馈的绑定状态:1=绑定, 0=解除绑定
};

// CAN对象
can_context_t can_context = {
  .ep_valid_flag = 0, // 环保数据有效标志: 0=无效, 1=有效
  .mil_lamp = 0,      // 故障灯状态:0:未点亮, 1=点亮
  .engine_speed = 0x00, // 发动机转速

  .comm_state1 = 0,  // 通信状态:0=异常, 1=正常
  .recv_state1 = 0,  // 接收状态:0=未收到数据, 1=已收到数据
  .recv_timer1 = CAN1_RECV_TIMEOUT_SP, // 接收超时计数器

  .comm_state2 = 0,  // 通信状态:0=异常, 1=正常
  .recv_state2 = 0,  // 接收状态:0=未收到数据, 1=已收到数据
  .recv_timer2 = CAN2_RECV_TIMEOUT_SP, // 接收超时计数器

  .sleep_state = 0, // 休眠状态,0=未休眠, 1=休眠
};

obd_info_t obd_info;
dtc_context_t dtc_1939;
dtc_context_t dtc_27145;

can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];

can_msg_queue_t can1_msg_queue;
can_msg_queue_t can2_msg_queue;

#define ACC_ON_TIMEOUT_SP     30  //通信异常判定时间
/**********************************************************************************
* MCU通信状态判断，此函数1s执行一次
***********************************************************************************/
void Can_CheckCommState(void)
{
  static uint8_t acc_current_state = 0;
  static uint8_t acc_previous_state = 0;
  static uint16_t acc_on_timer = ACC_ON_TIMEOUT_SP; // 30秒
  static uint8_t reset_can_timer = 0;  // 5秒钟
  static uint8_t reset_can_flag = 0;     // 0=可以重新启动CAN，1=不可以重新启动CAN

  acc_current_state = COLT_GetAccStatus();
  if (acc_current_state != acc_previous_state)
  {
    if (acc_current_state==0) // ACC关闭事件
    {
      acc_on_timer = ACC_ON_TIMEOUT_SP;
      reset_can_flag = 0;
    }
  }

  if (acc_current_state==1) // ACC打开
  {
    if (acc_on_timer)
    {
      acc_on_timer--;
    }

    if ((acc_on_timer==0) && (can_context.recv_state1==0) && (can_context.recv_state2==0) && (reset_can_flag==0))
    {
      reset_can_flag = 1;
      reset_can_timer = 5;
      CAN_POWER_OFF();  // 收发器进入待机状态
      can_context.comm_state1 = CAN_NOK; //CAN通讯异常
      PcDebug_SendString("Can Off!\n");
    }

    if (reset_can_timer)
    {
      reset_can_timer--;
      if (reset_can_timer==0)
      {
        CAN_Initialize();  // 重新对CAN模块上电和初始化
        PcDebug_SendString("Can Rest!\n");
      }
    }
  }

  acc_previous_state = acc_previous_state;
}

/**********************************************************************************
 * can接口没有收到数据倒计时(100ms周期)
 ***********************************************************************************/
void iCan_DebounceRecvState(void)
{
  if (can_context.recv_timer1)
    can_context.recv_timer1--;
  else
  {
    can_context.recv_state1 = CAN_NOK; // 未收到CAN1数据
  }

  if (can_context.recv_timer2)
    can_context.recv_timer2--;
  else
  {
    can_context.recv_state2 = CAN_NOK; // 未收到CAN2数据
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DTC管理
////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************************************************
* 获取是否有新的DTC需要上报
*************************************************************************/
uint8_t DTC_GetNewFlag(dtc_context_t* pThis)
{
  return pThis->new_flag;
}

/************************************************************************
* 清除新DTC标志
*************************************************************************/
void DTC_ClearNewFlag(dtc_context_t* pThis)
{
  pThis->new_flag = 0;
}

/******************************************************************************************
* 收到CAN故障码后，先搜索激活故障码列表是否有该故障码，如果有则更新该故障码的计时,
* 如果没有，则认为是新产生的故障码，添加到激活故障码列表,并且置;
* 如果列表已满，则丢弃，直到列表有空单元
* 返回: 0-没有新增故障码或故障码列表已满，1-新增了故障码
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
    if (pThis->code[i] == dtcode) // 历史故障码
    {
      pThis->debounce_tmr[i] = DTC_DEBOUNCE_TIME_SP;
      return 0;
    }
  }

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++) // 新增故障码
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
  return 0;	// 列表已满
}

/*****************************************************************************************
* 此函数每1秒运行一次，对故障码列表中故障码存在的时间进行倒计时，倒计时到0时，
* 认为该故障码已经消失，对该故障码清零
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
* 获取故障诊断码数据
* pBuf - 指向故障诊断码数据缓存的指针
* 返回: 故障诊断码个数，0表示没有故障诊断码数据
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
 * 处理接收到的CAN报文(SAE J1939故障码)
**********************************************************************************/
uint8_t iCan_ProcessDtcMsg_J1939(uint32_t canId,uint8_t *pdata,uint8_t size)
{
  uint8_t i;
  uint8_t j = 0;
  uint8_t retval = 0x00;
  uint32_t dtc;
  static uint8_t dtc_total;
  static uint8_t mfault_packages = 0;  // ECM分包数量
  static uint8 mfault_index = 0;   // ECM多个故障码帧序列
  static uint8_t torque_flag = 0;  // 参考扭矩接收标志0-无 1-有
  static uint8_t dm1_valid_flag = 0; // 收到多故障广播帧

  switch (canId)
  {
  case 0x18FECA00:  // 单包故障(0x18FECA00)
    //ep_data_buffer[EP_POS24_ADDRESS] = (pdata[0]>>6) & 0x03; // MIL灯状态(B1.bit7-8)

    dtc = pdata[2] + (uint32_t)(pdata[3]<<8) + (uint32_t)(pdata[4]<<16) + (uint32_t)(pdata[5]<<24);
    if ((dtc!=0xFFFFFFFF) && (dtc!=0x00000000))
    {
      DTC_SaveCode(&dtc_1939, dtc);
    }
    retval = 0x01;
    break;

  case 0x18ECFF00: // 发动机多包判断故障报文(广播帧) (0x18ECFF00)
    if (pdata[5]==0xCA && pdata[6]==0xFE && pdata[7]==0x00) // 多包故障诊断协议PGN65226
    {
      dtc_total = (pdata[1] - 2) / 4; // 故障代码总数
      mfault_packages = pdata[3];  //故障帧CAN帧个数
      mfault_index = 1;
      dm1_valid_flag = 1;
    }
    else if (pdata[5]==0xE3 && pdata[6]==0xFE && pdata[7]==0x00) // 发动机配置通告PGN65251
    {
      torque_flag = 1;
    }
    retval = 0x01;
    break;

  case 0x18EBFF00: // 发动机多包内容报文   (0x18EBFF00)
    if (torque_flag == 1 && pdata[0] == 3) // 发动机配置package3
    {
      torque_flag = 0;   //接收结束
    }

    if (dm1_valid_flag == 0) break; // 没有收到多故障码广播帧

    if (pdata[0]==mfault_index && dm1_valid_flag==1) // 收到多故障分包
    {
      if (pdata[0] == 1) // 首帧解析
      {
        //ep_data_buffer[EP_POS24_ADDRESS] = (pdata[1]>>6) & 0x03; // MIL灯状态(B1.bit7-8)
        memset(dtc_1939.buffer, 0xFF, DTC_BUFFER_SIZE); // 清除DTC缓存
        memcpy(dtc_1939.buffer,&pdata[3],5);
        dtc_1939.index = 5;
      }
      else // 非首帧解析
      {
        if (dtc_1939.index < (DTC_BUFFER_SIZE-7))
        {
          memcpy(&dtc_1939.buffer[dtc_1939.index],&pdata[1],7);
          dtc_1939.index += 7;
        }
      }

      mfault_index++;
      if (mfault_index > mfault_packages) //传送结束
      {
        for (i=0; i<dtc_total; i++)
        {
          dtc = (uint32_t)(dtc_1939.buffer[j]) + (uint32_t)(dtc_1939.buffer[j+1]<<8) + (uint32_t)(dtc_1939.buffer[j+2]<<16) + (uint32_t)(dtc_1939.buffer[j+3]<<24);
          j += 4;

          if ((dtc!=0xFFFFFFFF) && (dtc!=0x00000000))
          {
            uint8_t val = DTC_SaveCode(&dtc_1939, dtc); // 存入故障码

            if (val == 1) // 新故障码
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

//==复位参数值为无效==============================================================
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

//==复位CAN帧列表=================================================================
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
*Description	:MCU模块计时函数，此函数10ms执行一次
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
      //SendCan1Message();//此处添加2秒运行一次的函数
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
* 发送CAN报文(包括锁车/解锁、绑定/解绑、OBD请求)(100ms周期)
***********************************************************************************/
void CAN_ProduceSendMsg(void)
{

}

/******************************************************************************
* 清空CAN队列数据
******************************************************************************/
void can_msg_queue_reset(can_msg_queue_t* pThis)
{
  pThis->head = 0;
  pThis->tail = 0;
}

/******************************************************************************
* 把一个数据存入队列中
******************************************************************************/
void can_msg_queue_push(can_msg_queue_t* pThis, CanRxMsg* pMsg)
{
  uint8_t pos;

  pos = (pThis->head + 1) % CAN_MSG_QUEUE_MAX_SIZE;
  if (pos != pThis->tail) // 非满状态
  {
    // 复制结构体
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

    pThis->head = pos; // 更新头部
  }
}

/******************************************************************************
* 从队列中取一个数据
******************************************************************************/
static void can_msg_queue_pop(can_msg_queue_t* pThis, can_msg_t* pMsg)
{
  if (pThis->tail != pThis->head) //非空状态
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

    pThis->tail = (pThis->tail + 1) % CAN_MSG_QUEUE_MAX_SIZE; // 更新尾巴
  }
}

/******************************************************************************
* 获取队列中有效数据个数
******************************************************************************/
static uint8_t can_msg_queue_size(can_msg_queue_t* pThis)
{
  return ((pThis->head + CAN_MSG_QUEUE_MAX_SIZE - pThis->tail) % CAN_MSG_QUEUE_MAX_SIZE);
}


/**********************************************************************************
* 处理接收到的来自CAN1(下车)的报文
***********************************************************************************/
void CAN1_ProcessRecvMsg(can_msg_t* pMsg)
{
  uint8_t buff[8];
  uint8_t retval = 0x00;

  //==汽车式下车====================================================================
  retval = CAN_ProcessRecvDownMsg_AC(pMsg->id, pMsg->data, pMsg->len); // 下车底盘数据
  if (retval==0x01)
  {
    return;
  }

  retval = CAN_ProcessRecvEngineMsg_AC(pMsg->id, pMsg->data, pMsg->len); // 下车发动机数据
  if (retval==0x01)
  {
    return;
  }
  
  //==全地面下车====================================================================
  retval = CAN_ProcessRecvDownMsg_AG(pMsg->id, pMsg->data, pMsg->len); // 下车底盘数据
  if (retval==0x01)
  {
    return;
  }

  retval = CAN_ProcessRecvEngineMsg_AG(pMsg->id, pMsg->data, pMsg->len); // 下车发动机数据
  if (retval==0x01)
  {
    return;
  }

  switch (pMsg->id)
  {
  case 0x1fffffff://测试 ID  31 61 32 62 33 63 34 64
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
* 处理MCU模块的任务(阻塞)
*******************************************************************************/
void AppThread_Can1Recv(void *argument)
{
  can_msg_t can_msg;

  while (1)
  {
    if (CAN_OK==CAN1_ReceiveMessage(2*OS_TICKS_PER_SEC)) // 等待CAN1信号量(阻塞)
    {
      while (can_msg_queue_size(&can1_msg_queue) > 0) // if
      {
        can1MsgQueueENTER_CRITICAL(); // 关闭接收中断
        can_msg_queue_pop(&can1_msg_queue, &can_msg);   // 取一个数据(过滤后)
        can1MsgQueueEXIT_CRITICAL();  // 开启接收中断
        CAN1_ProcessRecvMsg(&can_msg);
      }
    }
  }
}

/**********************************************************************************
* 处理接收到的来自CAN2(上车)的报文
***********************************************************************************/
void CAN2_ProcessRecvMsg(can_msg_t* pMsg)
{
  uint8_t buff[8];
  uint8_t retval = 0x00;

  retval = CAN_ProcessRecvUpMsg(pMsg->id, pMsg->data, pMsg->len); // 上车数据
  if (retval==0x01)
  {
    return;
  }

  switch (pMsg->id)
  {
  case 0x1ffffffe://测试 ID 31 61 32 62 33 63 34 64
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
* 处理MCU模块的任务(阻塞)
*******************************************************************************/
void AppThread_Can2Recv(void *argument)
{
  can_msg_t can_msg;

  while (1)
  {
    if (CAN_OK==CAN2_ReceiveMessage(2*OS_TICKS_PER_SEC)) // 等待CAN2信号量(阻塞)
    {
      while (can_msg_queue_size(&can2_msg_queue) > 0) // if
      {
        can2MsgQueueENTER_CRITICAL(); // 关闭接收中断
        can_msg_queue_pop(&can2_msg_queue, &can_msg);   // 取一个数据(过滤后)
        can2MsgQueueEXIT_CRITICAL();  // 开启接收中断
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
* 100ms执行一次(无阻塞)
***********************************************************************************/
void Can_Do100msTasks(void)
{
  iCan_DebounceRecvState();
  zxsts_StateMachine();
}

/**********************************************************************************
* 1s执行一次(无阻塞)
***********************************************************************************/
void Can_Do1sTasks(void)
{
  Can_CheckCommState();
  DTC_DebounceCode(&dtc_1939);
  DTC_DebounceCode(&dtc_27145);
}

/**********************************************************************************
* 获取GPS终端与MCU的通信状态
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
* 获取CAN接收状态,1:收到了CAN1数据，2:没有收到CAN1数据
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

//==获取环保数据有效性=============================================================
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

//==获取发动机类型=================================================================
uint8_t CAN_GetEngineType(void)
{
  return m2m_asset_data.ecu_type;
}

//==获取发动机转速=================================================================
uint16_t CAN_GetEngineSpeed(void)
{
  return can_context.engine_speed;
}

//-----文件Mcu.c结束---------------------------------------------
