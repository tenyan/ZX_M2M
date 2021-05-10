/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CanHW.c
 * @Engineer: TenYan
 * @Company:  徐工信息智能硬件部
 * @version:  V1.0
 * @Date:     2020-10-15
 * @brief:
 *******************************************************************************/
#include "CanHW.h"
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define CAN_BRT_250K     1
#define CAN_BRT_500K     2
#define CAN_BRT_1000K    3

#define USE_CAN1         1
#define CAN1_BAUDRATE    CAN_BRT_250K

#define USE_CAN2         1
#define CAN2_BAUDRATE    CAN_BRT_250K

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Can1ReceiveSem;  // CAN1接收信号量
osSemaphoreId_t sid_Can2ReceiveSem;  // CAN2接收信号量

extern can_msg_queue_t can1_msg_queue;
extern can_msg_queue_t can2_msg_queue;

//extern STUSleep stuSleep; //休眠结构体定义
extern can_frame_t can_frame_table[MAX_CAN_FRAME_NUM]; // CAN数据缓存

/******************************************************************************
*
*******************************************************************************/
void Can_AddFrameToBuffer(uint32_t id, uint8_t *data)
{
  uint8_t i;

  for (i=0; i<MAX_CAN_FRAME_NUM; i++)
  {
    if (can_frame_table[i].id==id) // 已存在
    {
      memcpy(can_frame_table[i].data, data, 8);
      return;
    }
    else
    {
      if (can_frame_table[i].id==0) // 空缓存且新ID
      {
        can_frame_table[i].id = id;
        memcpy(can_frame_table[i].data, data, 8);
        return;
      }
    }
  }
}

/*****************************************************************************
 * //PA2(CAN_PWR_EN) PA6(CAN1_LED) PA7(CAN2_LED)
 ****************************************************************************/
void CAN_GpioInitialize(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  CAN_POWER_OFF();
}

/*******************************************************************************
* CAN初始化函数
*******************************************************************************/
void CAN_Initialize(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  CAN_POWER_OFF();  // 关闭CAN接收器,先进行CAN控制器初始化
  
  can_msg_queue_reset(&can1_msg_queue);
  can_msg_queue_reset(&can2_msg_queue);
  
  if(NULL==sid_Can1ReceiveSem)
  {  sid_Can1ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(NULL==sid_Can2ReceiveSem)
  {  sid_Can2ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  //==CAN1 configuration========================================================
  // CAN Periph clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  // CAN GPIOs configuration
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1); // Connect PA12 to CAN1_Tx pin
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1); // Connect PA11 to CAN1_Rx pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // CAN register init
  CAN_DeInit(CAN1);   // 将外设CAN的全部寄存器重设为缺省值
  CAN_StructInit(&CAN_InitStructure); // 把CAN_InitStruct中的每一个参数按缺省值填入

  // CAN configuration
  CAN_InitStructure.CAN_TTCM = DISABLE; // 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳
  CAN_InitStructure.CAN_ABOM = ENABLE;  // 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出
  CAN_InitStructure.CAN_AWUM = DISABLE; // 自动唤醒禁止，有报文来的时候自动退出休眠
  CAN_InitStructure.CAN_NART = DISABLE; // 报文重传, 如果错误一直传到成功止，否则只传一次
  CAN_InitStructure.CAN_RFLM = DISABLE; // 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文
  CAN_InitStructure.CAN_TXFP = DISABLE;  // 发送优先级  0---由标识符决定  1---由发送请求顺序决定
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 模式
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器

  // 波特率计算方法
  // CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1)
  // 配置大方向: Tseg1>=Tseg2      Tseg2>=tq; Tseg2>=2TSJW
#if(CAN1_BAUDRATE==CAN_BRT_250K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;  // 时间段1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;  // 时间段2
  CAN_InitStructure.CAN_Prescaler = 15;     // 波特率预分频数
#elif(CAN1_BAUDRATE==CAN_BRT_500K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 12;
#elif(CAN1_BAUDRATE==CAN_BRT_1000K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  CAN_InitStructure.CAN_Prescaler = 3;
#else // 默认使用250K
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 15;
#endif
  CAN_Init(CAN1, &CAN_InitStructure);

  /* Enable CAN1 RX0 interrupt IRQ channel  */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN1,CAN_IT_FMP1, ENABLE);


  //==CAN2 configuration========================================================
  // CAN Periph clock enable
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

  // CAN GPIOs configuration
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); // Connect PB13 to CAN2_Tx pin
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); // Connect PB12 to CAN2_Rx pin
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // CAN register init
  CAN_DeInit(CAN2);   // 将外设CAN的全部寄存器重设为缺省值
  CAN_StructInit(&CAN_InitStructure); // 把CAN_InitStruct中的每一个参数按缺省值填入

  // CAN configuration
  CAN_InitStructure.CAN_TTCM = DISABLE; // 时间触发禁止, 时间触发：CAN硬件的内部定时器被激活，并且被用于产生时间戳
  CAN_InitStructure.CAN_ABOM = ENABLE;  // 自动离线禁止，自动离线：一旦硬件监控到128次11个隐性位，就自动退出离线状态。在这里要软件设定后才能退出
  CAN_InitStructure.CAN_AWUM = DISABLE; // 自动唤醒禁止，有报文来的时候自动退出休眠
  CAN_InitStructure.CAN_NART = DISABLE; // 报文重传, 如果错误一直传到成功止，否则只传一次
  CAN_InitStructure.CAN_RFLM = DISABLE; // 接收FIFO锁定, 1--锁定后接收到新的报文摘不要，0--接收到新的报文则覆盖前一报文
  CAN_InitStructure.CAN_TXFP = DISABLE;  // 发送优先级  0---由标识符决定  1---由发送请求顺序决定
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // 模式
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // 重新同步跳宽，只有can硬件处于初始化模式时才能访问这个寄存器

  // 波特率计算方法
  // CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1)
  // 配置大方向: Tseg1>=Tseg2      Tseg2>=tq; Tseg2>=2TSJW
#if(CAN2_BAUDRATE==CAN_BRT_250K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;  // 时间段1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;  // 时间段2
  CAN_InitStructure.CAN_Prescaler = 15;     // 波特率预分频数
#elif(CAN2_BAUDRATE==CAN_BRT_500K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 12;
#elif(CAN2_BAUDRATE==CAN_BRT_1000K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  CAN_InitStructure.CAN_Prescaler = 3;
#else // 默认使用250K
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 15;
#endif
  CAN_Init(CAN2, &CAN_InitStructure);  //波特率为：30MHz/15/(1+6+1)=0.25 即波特率为250KBPs

  /* Enable CAN2 RX0 interrupt IRQ channel  */
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  CAN_ITConfig(CAN2,CAN_IT_FMP0, ENABLE);
  CAN_ITConfig(CAN2,CAN_IT_FMP1, ENABLE);

  // 配置CAN过滤器
  CAN_FilterInitStructure.CAN_FilterNumber = 0; // 过滤器0
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // 屏敝模式
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32位
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; // 以下四个都为0, 表明不过滤任何id
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // 能够通过该过滤器的报文存到fifo0中,用CAN2_RX0_IRQn中断来处理
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber = 14; // 过滤器14
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // 能够通过该过滤器的报文存到fifo1中,用CAN2_RX1_IRQn中断来处理
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  CAN_POWER_ON();  // 开启CAN接收器
}

/*********************************************************************************
* CAN1发送数据
**********************************************************************************/
void CAN1_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata)
{
  CanTxMsg tx_msg;

  if (frame_type == CAN_FRAME_TYPE_EXT) // 扩展帧
  {
    tx_msg.ExtId = can_id;
    tx_msg.IDE = CAN_ID_EXT;
  }
  else // 标准帧
  {
    tx_msg.StdId = can_id;
    tx_msg.IDE = CAN_ID_STD;
  }

  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = len;
  tx_msg.Data[0] = pdata[0];
  tx_msg.Data[1] = pdata[1];
  tx_msg.Data[2] = pdata[2];
  tx_msg.Data[3] = pdata[3];
  tx_msg.Data[4] = pdata[4];
  tx_msg.Data[5] = pdata[5];
  tx_msg.Data[6] = pdata[6];
  tx_msg.Data[7] = pdata[7];

  CAN_Transmit(CAN1, &tx_msg);
}

/*********************************************************************************
 * 等待来自CAN1的数据帧: timeout(等待超时的时间)
**********************************************************************************/
uint8_t CAN1_ReceiveMessage(uint16_t timeout)
{
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Can1ReceiveSem, timeout);
  if (stat == osOK)
  {
    return CAN_OK;
  }
  else
  {
    return CAN_NOK;
  }
}

/*********************************************************************************
* CAN2发送数据
**********************************************************************************/
void CAN2_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata)
{
  CanTxMsg tx_msg;

  if (frame_type == CAN_FRAME_TYPE_EXT) // 扩展帧
  {
    tx_msg.ExtId = can_id;
    tx_msg.IDE = CAN_ID_EXT;
  }
  else // 标准帧
  {
    tx_msg.StdId = can_id;
    tx_msg.IDE = CAN_ID_STD;
  }

  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = len;
  tx_msg.Data[0] = pdata[0];
  tx_msg.Data[1] = pdata[1];
  tx_msg.Data[2] = pdata[2];
  tx_msg.Data[3] = pdata[3];
  tx_msg.Data[4] = pdata[4];
  tx_msg.Data[5] = pdata[5];
  tx_msg.Data[6] = pdata[6];
  tx_msg.Data[7] = pdata[7];

  CAN_Transmit(CAN2, &tx_msg);
}

/*********************************************************************************
 * 等待来自CAN1的数据帧: timeout(等待超时的时间)
**********************************************************************************/
uint8_t CAN2_ReceiveMessage(uint16_t timeout)
{
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Can2ReceiveSem, timeout);
  if (stat == osOK)
  {
    return CAN_OK;
  }
  else
  {
    return CAN_NOK;
  }
}

/******************************************************************************
 * 特定车辆CAN1帧处理(下车)
 ******************************************************************************/
#define NUM_OF_CAN1_FILTER_ID1   3
uint32_t Can1FilterId1[NUM_OF_CAN1_FILTER_ID1] = {0x18FF0800};

//==汽车式下车====================================================================
// 下车底盘CAN1帧id表
uint32_t ZxDownCanIdTbl_AC[] = {\
0x1CF23B21,0x1CF23F21,0x1CF21D21,0x1CF25E21,0x1CF22D21,0x1CF28B21,\
0x1CF25C21,0x1CF25D21,0x1CF23A21,0x1CF24B21,0x1CF25C21,\
0x1CF28C21,0x1CF40D21,0x1CF20F21,0x1CF21F21,0x1CF22F21,0x1CF29F21};
#define SIZE_OF_ZXDOWN_CAN_ID_TBL_AC  (sizeof(ZxDownCanIdTbl_AC)/sizeof(ZxDownCanIdTbl_AC[0]))

// 发动机CAN1帧id表(汽车式)
uint32_t ZxEngineCanIdTbl_AC[] = {\
0x0CF00300,0x0CF00400,0x0CF0233D,0x18E00021,0x18F00A00,0x18F00E51,0x18F00F52,0x18FCBD3D,0x18FD0700,\
0x18FD2000,0x18FD3E3D,0x18FD7C00,0x18FD9BA3,0x18FDB200,0x18FE56A3,0x18FE563D,0x18FECA00,0x18ECFF00,0x18EBFF00,0x18FEDF00,\
0x18FEDF3D,0x18FEE000,0x18FEE500,0x18FEE900,0x18FEEE00,0x18FEEF00,0x18FEF100,0x18FEF200,\
0x18FEF500,0x18FEF600,0x18FEFC17,0x18FEFF00,0x18FF1400,0x18FF203D,0x1FFFFFFF};
#define SIZE_OF_ZXENGINE_CAN_ID_TBL_AC  (sizeof(ZxEngineCanIdTbl_AC)/sizeof(ZxEngineCanIdTbl_AC[0]))

//==全地面下车====================================================================
// 下车底盘CAN1帧id表
uint32_t ZxDownCanIdTbl_AG[] = {\
0x20F,0x21D,0x21E,0x21F,0x22A,0x22B,0x22C,0x22D,0x22E,0x22F,\
0x23A,0x23B,0x23F,0x24B,0x24D,0x24E,0x24F,0x25A,0x25C,0x25D,0x25E,\
0x25F,0x26A,0x28C,0x28D,0x29F,0x2BB,0x2BC,0x2BD,0x2BE,0x2AB,\
0x2AC,0x2AD,0x2AE,0x2AF};
#define SIZE_OF_ZXDOWN_CAN_ID_TBL_AG  (sizeof(ZxDownCanIdTbl_AG)/sizeof(ZxDownCanIdTbl_AG[0]))

// 发动机CAN1帧id表(全地面)
uint32_t ZxEngineCanIdTbl_AG[] = {\
0x20A,0x20B,0x20C,0x20D,0x20E,0x21A,0x21B,0x21C,0x24A,0x40A,0x40C,0x40D,0x41A,\
0x41B,0x41C,0x41D,0x42A,0x42B,0x42C,0x46A,0x46B,0x46C,0x46D};
#define SIZE_OF_ZXENGINE_CAN_ID_TBL_AG  (sizeof(ZxEngineCanIdTbl_AG)/sizeof(ZxEngineCanIdTbl_AG[0]))

/**********************************************************************************
* 返回值:3=该帧只需要上传,1=需要上传且需特殊处理, 2=只需要特殊处理,0=直接丢弃
***********************************************************************************/
uint8_t CAN1_FilterFrame(CanRxMsg* pMsg)
{
  uint8_t i;
  uint32_t canId;

  if (pMsg->IDE==CAN_Id_Extended)
  {
    canId = pMsg->ExtId;
  }
  else
  {
    canId = pMsg->StdId;
  }

  // 上传且处理
  for (i=0; i<NUM_OF_CAN1_FILTER_ID1; i++)
  {
    if (canId==Can1FilterId1[i])
    {
      Can_AddFrameToBuffer(canId, pMsg->Data);
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // 发送信号量
      return 1;
    }
  }

  // 用户设置上传
  for (i=0; i<m2m_asset_data.canId_num; i++)
  {
    if (canId==m2m_asset_data.canId_tbl[i])
    {
      Can_AddFrameToBuffer(canId, pMsg->Data);
      return 3;
    }
  }

  // 只需处理
  //==汽车式=====================================================
  for (i=0; i<SIZE_OF_ZXDOWN_CAN_ID_TBL_AC; i++)
  {
    if (canId==ZxDownCanIdTbl_AC[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // 发送信号量
      return 2;
    }
  }

  for (i=0; i<SIZE_OF_ZXENGINE_CAN_ID_TBL_AC; i++)
  {
    if (canId==ZxEngineCanIdTbl_AC[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // 发送信号量
      return 2;
    }
  }

  //==全地面=====================================================
  for (i=0; i<SIZE_OF_ZXDOWN_CAN_ID_TBL_AG; i++)
  {
    if (canId==ZxDownCanIdTbl_AG[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // 发送信号量
      return 2;
    }
  }

  for (i=0; i<SIZE_OF_ZXENGINE_CAN_ID_TBL_AG; i++)
  {
    if (canId==ZxEngineCanIdTbl_AG[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // 发送信号量
      return 2;
    }
  }

  return 0;
}

/*******************************************************************************
 * CAN1 RX0 interrupts: 将收到的所有CAN数据放入缓存
 *******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0))
  {
    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
    CAN1_FilterFrame(&RxMessage); // 过滤CAN帧
    can_context.comm_state1 = CAN_OK;
    can_context.recv_state1 = CAN_OK;
    can_context.recv_timer1 = CAN1_RECV_TIMEOUT_SP;
  }
}

/*******************************************************************************
* CAN1 RX1 interrupts: 将收到的所有CAN数据放入缓存
*******************************************************************************/
void CAN1_RX1_IRQHandler(void)
{
  CanRxMsg RxMessage;
  
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP1))
  {
    CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);
    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP1);
    CAN1_FilterFrame(&RxMessage); // 过滤CAN帧
    can_context.comm_state1 = CAN_OK;
    can_context.recv_state1 = CAN_OK;
    can_context.recv_timer1 = CAN1_RECV_TIMEOUT_SP;
  }
}

/******************************************************************************
 * 特定车辆CAN2帧处理(上车)
 ******************************************************************************/
#define NUM_OF_CAN2_FILTER_ID1   2
uint32_t Can2FilterId1[NUM_OF_CAN2_FILTER_ID1] = {0x18FF0800};

// 上车CAN2帧id表
uint32_t ZxUpCanIdTbl[] = {\
0x181,0x183,0x185,0x188,0x191,0x193,0x195,0x1A3,0x1B1,0x1B2,\
0x1B3,0x1C3,0x1C5,0x1D1,0x1D5,0x1E3,0x1E9,0x1F3,0x203,0x263,0x264,0x273,0x283,\
0x285,0x288,0x291,0x293,0x295,0x2A3,0x2B3,0x2C3,0x2E3,0x2F3,\
0x343,0x35A,0x35B,0x35C,0x35D,0x35E,0x35F,0x363,0x364,0x36A,0x36B,\
0x36C,0x373,0x381,0x382,0x385,0x388,0x391,0x393,0x394,0x395,\
0x3A1,0x3B3,0x3C3,0x3D3,0x3E3,0x3E4,0x3E5,0x3F3,0x3F5,0x443,0x463,\
0x464,0x473,0x482,0x488,0x493,0x4A3,0x4B3,0x4C1,0x4C3,0x4D3,\
0x4E3,0x4E4,0x4F3,0x504,0x50E,0x561,0x562,0x564,0x565,0x571,0x572,0x573,0x574,0x575,0x1FFFFFFE};
#define SIZE_OF_ZXUP_CAN_ID_TBL  (sizeof(ZxUpCanIdTbl)/sizeof(ZxUpCanIdTbl[0]))

/**********************************************************************************
* 返回值:3=该帧只需要上传,1=需要上传且需特殊处理, 2=只需要特殊处理,0=直接丢弃
***********************************************************************************/
uint8_t CAN2_FilterFrame(CanRxMsg* pMsg)
{
  uint8_t i;
  uint32_t canId;

  if (pMsg->IDE==CAN_Id_Extended)
  {
    canId = pMsg->ExtId;
  }
  else
  {
    canId = pMsg->StdId;
  }

  // 上传且处理
  for (i=0; i<NUM_OF_CAN2_FILTER_ID1; i++)
  {
    if (canId==Can2FilterId1[i])
    {
      Can_AddFrameToBuffer(canId, pMsg->Data);
      can_msg_queue_push(&can2_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can2ReceiveSem); // 发送信号量
      return 1;
    }
  }

  // 用户设置上传
  for (i=0; i<m2m_asset_data.canId_num; i++)
  {
    if (canId==m2m_asset_data.canId_tbl[i])
    {
      Can_AddFrameToBuffer(canId, pMsg->Data);
      return 3;
    }
  }

  // 只需处理
  for (i=0; i<SIZE_OF_ZXUP_CAN_ID_TBL; i++)
  {
    if (canId==ZxUpCanIdTbl[i])
    {
      can_msg_queue_push(&can2_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can2ReceiveSem); // 发送信号量
      return 2;
    }
  }

  return 0;
}

/*******************************************************************************
* CAN2 RX0 interrupts: 将收到的所有CAN数据放入缓存
*******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0))
  {
    CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);
    CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
    CAN2_FilterFrame(&RxMessage); // 过滤CAN帧
    can_context.comm_state2 = CAN_OK;
    can_context.recv_state2 = CAN_OK;
    can_context.recv_timer2 = CAN2_RECV_TIMEOUT_SP;
  }
}

/*******************************************************************************
* CAN2 RX1 interrupts: 将收到的所有CAN数据放入缓存
*******************************************************************************/
void CAN2_RX1_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN2,CAN_IT_FMP1))//增加fi1接受327
  {
    CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);
    CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);
    CAN2_FilterFrame(&RxMessage); // 过滤CAN帧
    can_context.comm_state2 = CAN_OK;
    can_context.recv_state2 = CAN_OK;
    can_context.recv_timer2 = CAN2_RECV_TIMEOUT_SP;
  }
}

//-----文件McuHW.c结束---------------------------------------------

