/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-10-11
 * @brief:    GPSģ��ײ���������
 *******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "GpsHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart2ReceiveSem = NULL;  // ���ڽ����ź���
uint8_t usart2_dma_tx_buffer[USART2_TX_BUFFER_SIZE];

volatile uint16_t usart2_rx_size;
osSemaphoreId_t sid_Usart2TransmitSem = NULL;  // ���ڷ����ź���
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];
uint8_t usart2_dma_rx_buffer[USART2_RX_BUFFER_SIZE];

extern void nmea_msg_queue_push(uint8_t dat);

/*****************************************************************************
 * PC0--GPS_ON_OFF, PC1--GPS_POWER
 ****************************************************************************/
void GPS_GpioInitialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPS_OFF();
  DISABLE_GPS_PWR();
}

/*****************************************************************************
 * USART2��TX/RX��PA2(TXD)��PA3(RXD)
 ****************************************************************************/
void USART2_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS�ź�������
  if(sid_Usart2ReceiveSem==NULL)
  {  sid_Usart2ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart2TransmitSem==NULL)
  {  sid_Usart2TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // ����ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // ����DMAʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ����GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // ����USART2ʱ��

  // USART GPIOs configuration
  // Configure USART3_Tx(PA2) and USART3_Rx(PA3) pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  // Connect PA2 to USART2_Tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);  // Connect PA3 to USART2_Rx

  // Configure USART Tx and Rx as alternate function
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_DeInit(USART2);
  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
  USART_ClockInit(USART2, &USART_ClockInitStructure);

  // USART configuration
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART2, ENABLE);  // ����2ʹ��
  USART_ClearFlag(USART2, USART_FLAG_TC);  // ���������ɱ�־

  //����DMA
  DMA_DeInit(DMA1_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // ͨ��4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_dma_rx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // ���赽�ڴ�
  DMA_InitStructure.DMA_BufferSize = USART2_RX_BUFFER_SIZE;               // �����С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // �����ַ����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ��ַ��
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݴ�С1byte
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݴ�С1byte
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     // ���ȼ���
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // ����fifo
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);                             // ��ʼ��dma1��1
  DMA_Cmd(DMA1_Stream5, ENABLE);                                          // ����dma1��1
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);                          // ��������2 DMA����

  //����DMA����
  DMA_DeInit(DMA1_Stream6);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);     // �����ַ
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_dma_tx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_BufferSize = USART2_TX_BUFFER_SIZE;               // ����DMA�ڴ���ʱ�������ĳ���
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // ���費����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ�ȣ��ֽ�
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ�ȣ��ֽ�
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMAģʽ��һ�δ��䣬��ѭ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // ָ�����FIFOģʽ��ֱ��ģʽ������ָ����������ʹ��FIFOģʽ
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       // ָ����FIFO��ֵˮƽ
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             // ָ����Burstת�������ڴ洫��
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     // ָ����Burstת��������Χת��
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 // �����ڴ�-->����  �Ӵ洢����
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);                          // ��������DMA����
}

/* =====================����2�����жϷ�����========================== */
void USART2_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) // �����ж�
  {
    it = USART2->SR;
    it = USART2->DR;    // �ȶ�SR,���DR����������жϱ�־λ

    USART_ClearITPendingBit(USART2,USART_IT_IDLE);	// ��������жϱ�־
    DMA_Cmd(DMA1_Stream5, DISABLE);                 // �ر�DMA,��ֹ�������������
    usart2_rx_size = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5); // �����յ�����������
    for (it = 0; it < usart2_rx_size; it++)
    {
      nmea_msg_queue_push(usart2_dma_rx_buffer[it]); // ���յ������ݷ������
      usart2_rx_buffer[it] = usart2_dma_rx_buffer[it];
    }
    //memcpy(usart2_rx_buffer, usart2_dma_rx_buffer, usart2_rx_size);
    usart2_rx_buffer[usart2_rx_size] = '\0';          // ���һλ���㣬���ڴ����ַ���
    DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_BUFFER_SIZE); // ��������DMAͨ�������С
    DMA_Cmd(DMA1_Stream5, ENABLE); // ����DMA
    osSemaphoreRelease(sid_Usart2ReceiveSem);
  }
}

/* =====================����2DMA���ͽ����ж�========================== */
void DMA1_Stream6_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6))//�ж��Ƿ���յ��ж�
  {
    osSemaphoreRelease(sid_Usart2TransmitSem);
  }
  DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6);	//����жϱ�־
}


/* ================================================================== */
void USART2_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART2_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart2TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream6, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart2_dma_tx_buffer),data,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream6, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream6, ENABLE);                      // ����DMA����
}

/* ================================================================== */
void USART2_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// ��ȡ�ַ����ַ�����
  if ((size==0)||(size>USART2_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart2TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream6, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart2_dma_tx_buffer),pstr,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream6, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream6, ENABLE);                      // ����DMA����
}

//===================================================================
uint8_t USART2_ReceiveData(uint8_t **data, uint16_t* size)
{
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Usart2ReceiveSem, osWaitForever);
  if (stat == osOK)
  {
    *data = usart2_rx_buffer;
    *size = usart2_rx_size;
    return 1;
  }

  return 0;
}
