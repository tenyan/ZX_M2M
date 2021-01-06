/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CanHW.c
 * @Engineer: TenYan
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
osSemaphoreId_t sid_Can1ReceiveSem;  // CAN1�����ź���
osSemaphoreId_t sid_Can2ReceiveSem;  // CAN2�����ź���

extern can_msg_queue_t can1_msg_queue;
extern can_msg_queue_t can2_msg_queue;

//extern STUSleep stuSleep; //���߽ṹ�嶨��
extern can_frame_t can_frame_table[MAX_CAN_FRAME_NUM]; // CAN���ݻ���

/******************************************************************************
*
*******************************************************************************/
void iCan_AddFrameToBuffer(uint32_t id, uint8_t *data)
{
  uint8_t i;

  for (i=0; i<MAX_CAN_FRAME_NUM; i++)
  {
    if (can_frame_table[i].id==id) // �Ѵ���
    {
      memcpy(can_frame_table[i].data, data, 8);
      return;
    }
    else
    {
      if (can_frame_table[i].id==0) // �ջ�������ID
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
void CAN_GpioInitialize()
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  CAN_POWER_ON();
  //CAN1_LED_ON();
  //CAN2_LED_ON();
}

/*******************************************************************************
* CAN��ʼ������
*******************************************************************************/
void CAN_Initialize(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef CAN_InitStructure;
  CAN_FilterInitTypeDef CAN_FilterInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  can_msg_queue_reset(&can1_msg_queue);
  can_msg_queue_reset(&can1_msg_queue);
  
  if(NULL==sid_Can1ReceiveSem)
  {  sid_Can1ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(NULL==sid_Can2ReceiveSem)
  {  sid_Can2ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  CAN_GpioInitialize();
  //CAN_CreatQueue(can1_recv_msg);
  //CAN_CreatQueue(can2_recv_msg);

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
  CAN_DeInit(CAN1);   // ������CAN��ȫ���Ĵ�������Ϊȱʡֵ
  CAN_StructInit(&CAN_InitStructure); // ��CAN_InitStruct�е�ÿһ��������ȱʡֵ����

  // CAN configuration
  CAN_InitStructure.CAN_TTCM = DISABLE; // ʱ�䴥����ֹ, ʱ�䴥����CANӲ�����ڲ���ʱ����������ұ����ڲ���ʱ���
  CAN_InitStructure.CAN_ABOM = ENABLE;  // �Զ����߽�ֹ���Զ����ߣ�һ��Ӳ����ص�128��11������λ�����Զ��˳�����״̬��������Ҫ����趨������˳�
  CAN_InitStructure.CAN_AWUM = DISABLE; // �Զ����ѽ�ֹ���б�������ʱ���Զ��˳�����
  CAN_InitStructure.CAN_NART = DISABLE; // �����ش�, �������һֱ�����ɹ�ֹ������ֻ��һ��
  CAN_InitStructure.CAN_RFLM = DISABLE; // ����FIFO����, 1--��������յ��µı���ժ��Ҫ��0--���յ��µı����򸲸�ǰһ����
  CAN_InitStructure.CAN_TXFP = DISABLE;  // �������ȼ�  0---�ɱ�ʶ������  1---�ɷ�������˳�����
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // ģʽ
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // ����ͬ������ֻ��canӲ�����ڳ�ʼ��ģʽʱ���ܷ�������Ĵ���

  // �����ʼ��㷽��
  // CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1)
  // ���ô���: Tseg1>=Tseg2      Tseg2>=tq; Tseg2>=2TSJW
#if(CAN1_BAUDRATE==CAN_BRT_250K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;  // ʱ���1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;  // ʱ���2
  CAN_InitStructure.CAN_Prescaler = 15;     // ������Ԥ��Ƶ��
#elif(CAN1_BAUDRATE==CAN_BRT_500K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 12;
#elif(CAN1_BAUDRATE==CAN_BRT_1000K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  CAN_InitStructure.CAN_Prescaler = 3;
#else // Ĭ��ʹ��250K
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
  CAN_DeInit(CAN2);   // ������CAN��ȫ���Ĵ�������Ϊȱʡֵ
  CAN_StructInit(&CAN_InitStructure); // ��CAN_InitStruct�е�ÿһ��������ȱʡֵ����

  // CAN configuration
  CAN_InitStructure.CAN_TTCM = DISABLE; // ʱ�䴥����ֹ, ʱ�䴥����CANӲ�����ڲ���ʱ����������ұ����ڲ���ʱ���
  CAN_InitStructure.CAN_ABOM = ENABLE;  // �Զ����߽�ֹ���Զ����ߣ�һ��Ӳ����ص�128��11������λ�����Զ��˳�����״̬��������Ҫ����趨������˳�
  CAN_InitStructure.CAN_AWUM = DISABLE; // �Զ����ѽ�ֹ���б�������ʱ���Զ��˳�����
  CAN_InitStructure.CAN_NART = DISABLE; // �����ش�, �������һֱ�����ɹ�ֹ������ֻ��һ��
  CAN_InitStructure.CAN_RFLM = DISABLE; // ����FIFO����, 1--��������յ��µı���ժ��Ҫ��0--���յ��µı����򸲸�ǰһ����
  CAN_InitStructure.CAN_TXFP = DISABLE;  // �������ȼ�  0---�ɱ�ʶ������  1---�ɷ�������˳�����
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal; // ģʽ
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;      // ����ͬ������ֻ��canӲ�����ڳ�ʼ��ģʽʱ���ܷ�������Ĵ���

  // �����ʼ��㷽��
  // CANbps= Fpclk/((BRP+1)*((Tseg1+1)+(Tseg2+1)+1)
  // ���ô���: Tseg1>=Tseg2      Tseg2>=tq; Tseg2>=2TSJW
#if(CAN2_BAUDRATE==CAN_BRT_250K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;  // ʱ���1
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;  // ʱ���2
  CAN_InitStructure.CAN_Prescaler = 15;     // ������Ԥ��Ƶ��
#elif(CAN2_BAUDRATE==CAN_BRT_500K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 12;
#elif(CAN2_BAUDRATE==CAN_BRT_1000K)
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_3tq;
  CAN_InitStructure.CAN_Prescaler = 3;
#else // Ĭ��ʹ��250K
  CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
  CAN_InitStructure.CAN_Prescaler = 15;
#endif
  CAN_Init(CAN2, &CAN_InitStructure);  //������Ϊ��30MHz/15/(1+6+1)=0.25 ��������Ϊ250KBPs

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

  // ����CAN������
  CAN_FilterInitStructure.CAN_FilterNumber = 0; // ������0
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // ����ģʽ
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; // 32λ
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; // �����ĸ���Ϊ0, �����������κ�id
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // �ܹ�ͨ���ù������ı��Ĵ浽fifo0��,��CAN2_RX0_IRQn�ж�������
  CAN_FilterInit(&CAN_FilterInitStructure);

  CAN_FilterInitStructure.CAN_FilterNumber = 14; // ������14
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; // �ܹ�ͨ���ù������ı��Ĵ浽fifo1��,��CAN2_RX1_IRQn�ж�������
  CAN_FilterInit(&CAN_FilterInitStructure);
}

/*********************************************************************************
* CAN1��������
**********************************************************************************/
void CAN1_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata)
{
  CanTxMsg tx_msg;

  if (frame_type == CAN_FRAME_TYPE_EXT) // ��չ֡
  {
    tx_msg.ExtId = can_id;
    tx_msg.IDE = CAN_ID_EXT;
  }
  else // ��׼֡
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
 * �ȴ�����CAN1������֡: timeout(�ȴ���ʱ��ʱ��)
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
* CAN2��������
**********************************************************************************/
void CAN2_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata)
{
  CanTxMsg tx_msg;

  if (frame_type == CAN_FRAME_TYPE_EXT) // ��չ֡
  {
    tx_msg.ExtId = can_id;
    tx_msg.IDE = CAN_ID_EXT;
  }
  else // ��׼֡
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
 * �ȴ�����CAN1������֡: timeout(�ȴ���ʱ��ʱ��)
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
 * �ض�����CAN1֡����(�³�)
 ******************************************************************************/
#define NUM_OF_CAN1_FILTER_ID1   1
uint32_t Can1FilterId1[NUM_OF_CAN1_FILTER_ID1] = {0x18FF0800};

// �³�����CAN1֡id��
uint32_t zxdown_can_id_tbl[] = {\
0x21D,0x21E,0x22A,0x22B,0x22C,0x22D,0x22E,0x23A,0x23B,0x24B,\
0x24D,0x24E,0x24F,0x25A,0x25C,0x25D,0x25E,0x25F,0x26A,0x28B,\
0x28C,0x2BB,0x2BC,0x2BD,0x2BE,0x2AB,0x2AC,0x2AD,0x2AE,0x2AF};
#define SIZE_OF_ZXDOWN_CAN_ID_TBL  (sizeof(zxdown_can_id_tbl)/sizeof(zxdown_can_id_tbl[0]))

// ������CAN1֡id��
uint32_t zxengine_can_id_tbl[] = {\
0x0CF00300,0x0CF00400,0x0CF0233D,0x18E00021,0x18F00E51,0x18F00F52,0x18FCBD3D,0x18FD0700,\
0x18FD2000,0x18FD3E3D,0x18FD7C00,0x18FD9BA3,0x18FDB200,0x18FE56A3,0x18FE563D,0x18FEDF00,\
0x18FEDF3D,0x18FEE000,0x18FEE500,0x18FEE900,0x18FEEE00,0x18FEEF00,0x18FEF100,0x18FEF200,\
0x18FEF600,0x18FEFC17,0x18FEFF00,0x18FF1400,0x18FF203D,0x1FFFFFFF};
#define SIZE_OF_ZXENGINE_CAN_ID_TBL  (sizeof(zxengine_can_id_tbl)/sizeof(zxengine_can_id_tbl[0]))

/**********************************************************************************
* ����ֵ:3=��ֻ֡��Ҫ�ϴ�,1=��Ҫ�ϴ��������⴦��, 2=ֻ��Ҫ���⴦��,0=ֱ�Ӷ���
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

  // �û������ϴ�
  //for (i=0; i<m2m_asset_data.canId_num; i++)
  //{
  //  if (canId==m2m_asset_data.canId_tbl[i])
  //  {
  //    Can_AddFrameToBuffer(id, rx_msg.Data);
  //    return 3;
  //  }
  //}

  // �ϴ��Ҵ���
  for (i=0; i<NUM_OF_CAN1_FILTER_ID1; i++)
  {
    if (canId==Can1FilterId1[i])
    {
      //Can_AddFrameToBuffer(id, rx_msg.Data);
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // �����ź���
      return 1;
    }
  }

  // ֻ�账��
  for (i=0; i<SIZE_OF_ZXDOWN_CAN_ID_TBL; i++)
  {
    if (canId==zxdown_can_id_tbl[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // �����ź���
      return 2;
    }
  }

  for (i=0; i<SIZE_OF_ZXENGINE_CAN_ID_TBL; i++)
  {
    if (canId==zxengine_can_id_tbl[i])
    {
      can_msg_queue_push(&can1_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can1ReceiveSem); // �����ź���
      return 2;
    }
  }

  return 0;
}

/*******************************************************************************
 * CAN1 RX0 interrupts: ���յ�������CAN���ݷ��뻺��
 *******************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0))
  {
    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage);
    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
    CAN1_FilterFrame(&RxMessage); // ����CAN֡
  }
}

/*******************************************************************************
* CAN1 RX1 interrupts: ���յ�������CAN���ݷ��뻺��
*******************************************************************************/
void CAN1_RX1_IRQHandler(void)
{
  CanRxMsg RxMessage;
  
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP1))
  {
    CAN_Receive(CAN1,CAN_FIFO1, &RxMessage);
    CAN_ClearITPendingBit(CAN1,CAN_IT_FMP1);
    CAN1_FilterFrame(&RxMessage); // ����CAN֡
  }
}

/******************************************************************************
 * �ض�����CAN2֡����(�ϳ�)
 ******************************************************************************/
#define NUM_OF_CAN2_FILTER_ID1   2
uint32_t Can2FilterId1[NUM_OF_CAN2_FILTER_ID1] = {0x18FF0800};
  
// �ϳ�CAN2֡id��
uint32_t zxup_can_id_tbl[] = {\
0x181,0x183,0x185,0x188,0x191,0x193,0x195,0x1A3,0x1B1,0x1B2,\
0x1B3,0x1C3,0x1C5,0x1D5,0x1E3,0x1E9,0x1F3,0x263,0x264,0x273,0x283,\
0x285,0x288,0x291,0x295,0x293,0x2A3,0x2B3,0x2C3,0x2E3,0x2F3,\
0x343,0x35A,0x35B,0x35C,0x35D,0x35E,0x35F,0x363,0x364,0x36A,0x36B,\
0x36C,0x373,0x381,0x382,0x385,0x388,0x391 0x393,0x394,0x395,\
0x3A1,0x3B3,0x3C3,0x3D3,0x3E3,0x3E4,0x3E5,0x3F3,0x3F5,0x443,0x463,\
0x464,0x473,0x482,0x488,0x493,0x4A3,0x4B3,0x4C1,0x4C3,0x4D3,\
0x4E3,0x4E4,0x4F3,0x504,0x561,0x564,0x565,0x1FFFFFFE};
#define SIZE_OF_ZXUP_CAN_ID_TBL  (sizeof(zxup_can_id_tbl)/sizeof(zxup_can_id_tbl[0]))

/**********************************************************************************
* ����ֵ:3=��ֻ֡��Ҫ�ϴ�,1=��Ҫ�ϴ��������⴦��, 2=ֻ��Ҫ���⴦��,0=ֱ�Ӷ���
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

  // �û������ϴ�
  //for (i=0; i<m2m_asset_data.canId_num; i++)
  //{
  //  if (canId==m2m_asset_data.canId_tbl[i])
  //  {
  //    Can_AddFrameToBuffer(id, rx_msg.Data);
  //    return 3;
  //  }
  //}

  // �ϴ��Ҵ���
  for (i=0; i<NUM_OF_CAN2_FILTER_ID1; i++)
  {
    if (canId==Can2FilterId1[i])
    {
      //Can_AddFrameToBuffer(id, rx_msg.Data);
      can_msg_queue_push(&can2_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can2ReceiveSem); // �����ź���
      return 1;
    }
  }

  // ֻ�账��
  for (i=0; i<SIZE_OF_ZXUP_CAN_ID_TBL; i++)
  {
    if (canId==zxup_can_id_tbl[i])
    {
      can_msg_queue_push(&can2_msg_queue, pMsg);
      osSemaphoreRelease(sid_Can2ReceiveSem); // �����ź���
      return 2;
    }
  }

  return 0;
}

/*******************************************************************************
* CAN2 RX0 interrupts: ���յ�������CAN���ݷ��뻺��
*******************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0))
  {
    CAN_Receive(CAN2,CAN_FIFO0, &RxMessage);
    CAN_ClearITPendingBit(CAN2,CAN_IT_FMP0);
    CAN2_FilterFrame(&RxMessage); // ����CAN֡
  }
}

/*******************************************************************************
* CAN2 RX1 interrupts: ���յ�������CAN���ݷ��뻺��
*******************************************************************************/
void CAN2_RX1_IRQHandler(void)
{
  CanRxMsg RxMessage;

  if (CAN_GetITStatus(CAN2,CAN_IT_FMP1))//����fi1����327
  {
    CAN_Receive(CAN2,CAN_FIFO1, &RxMessage);
    CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);
    CAN2_FilterFrame(&RxMessage); // ����CAN֡
  }
}

//-----�ļ�McuHW.c����---------------------------------------------

