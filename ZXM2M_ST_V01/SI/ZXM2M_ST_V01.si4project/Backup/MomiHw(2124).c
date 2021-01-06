/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: MomiHw.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-20
* @brief     ���ļ�Ϊ4Gģ��Ӳ���������ͷ�ļ�
******************************************************************************/

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "MomiHw.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart3ReceiveSem = NULL;  // ���ڽ����ź���
uint8_t usart3_dma_tx_buffer[USART3_TX_BUFFER_SIZE];

volatile uint16_t usart3_rx_size;
osSemaphoreId_t sid_Usart3TransmitSem = NULL;  // ���ڷ����ź���
uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
uint8_t usart3_dma_rx_buffer[USART3_RX_BUFFER_SIZE];

extern void momi_msg_queue_push(uint8_t dat);

/*****************************************************************************
 * PA15-MODEM_POWER, PC9-MODEM_DTR, PB14-MODEM_PWRKEY, PC8-MODEM_RESET
 * PB15--MODEM_STATUS
 ****************************************************************************/
void Modem_GpioInitialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); // GPIOʱ��ʹ��

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  DISABLE_MODEM_PWR();
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  MODEM_PWRKEY_H();
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);  
  MODEM_DTR_L();
  MODEM_RESET_H();

  // MODEM�ϵ�״̬
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // GPIO_PuPd_NOPULL
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*****************************************************************************
 * USART3��TX/RX��PB10(TXD)��PB11(RXD)
 ****************************************************************************/
void USART3_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS�ź�������
  if(sid_Usart3ReceiveSem==NULL)
  {  sid_Usart3ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart3TransmitSem==NULL)
  {  sid_Usart3TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // ����ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //����DMAʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ����GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // ����USART1ʱ��

  // USART GPIOs configuration
  // Configure USART3_Tx(PC10) and USART3_Rx(PC11) pins
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_USART3);  // Connect PC10 to USART3_Tx
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_USART3);  // Connect PC11 to USART3_Rx

  // Configure USART Tx and Rx as alternate function
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // USART configuration
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART3, ENABLE);  // ����3ʹ��
  USART_ClearFlag(USART3, USART_FLAG_TC);  // ���������ɱ�־

  //����DMA
  DMA_DeInit(DMA1_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // ͨ��4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart3_dma_rx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // ���赽�ڴ�
  DMA_InitStructure.DMA_BufferSize = USART3_RX_BUFFER_SIZE;               // �����С
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
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);                             // ��ʼ��dma1��1
  DMA_Cmd(DMA1_Stream1, ENABLE);                                          // ����dma1��1
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                          // ��������3DMA����

  //����DMA����
  DMA_DeInit(DMA1_Stream3);
  DMA_InitStructure.DMA_Channel =DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart3_dma_tx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_BufferSize = USART3_TX_BUFFER_SIZE;               // �����С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // ���費����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // �ڴ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // �������ݿ�ȣ��ֽ�
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // �ڴ����ݿ�ȣ��ֽ�
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMAģʽ��һ�δ��䣬��ѭ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // ���ȼ�����
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 // �����ڴ�-->����  �Ӵ洢����
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3| DMA_FLAG_TEIF3);
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);                          // ��������DMA����
}

/* =====================����3�����жϷ�����========================== */
void USART3_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) // �����ж�
  {
    it = USART3->SR;
    it = USART3->DR;    // �ȶ�SR,���DR����������жϱ�־λ
    
    USART_ClearITPendingBit(USART3,USART_IT_IDLE);	// ��������жϱ�־
    DMA_Cmd(DMA1_Stream1, DISABLE);//�ر�DMA,��ֹ�������������
    usart3_rx_size = USART3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1); // �����յ�����������
    for (it = 0; it < usart3_rx_size; it++)
    {
      momi_msg_queue_push(usart3_dma_rx_buffer[it]); // ���յ������ݷ������
      usart3_rx_buffer[it] = usart3_dma_rx_buffer[it];
    }
    //memcpy(usart3_rx_buffer, usart3_dma_rx_buffer, usart3_rx_size);
    usart3_rx_buffer[usart3_rx_size] = '\0';         // ���һλ���㣬���ڴ����ַ���
    DMA_SetCurrDataCounter(DMA1_Stream1, USART3_RX_BUFFER_SIZE); // ��������DMAͨ��6�����С
    DMA_Cmd(DMA1_Stream1, ENABLE); // ����DMA
    osSemaphoreRelease(sid_Usart3ReceiveSem);
  }
}

/* =====================����3DMA���ͽ����ж�========================== */
void DMA1_Stream3_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3))//�ж��Ƿ���յ��ж�
  {
    osSemaphoreRelease(sid_Usart3TransmitSem);
  }
  DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3 | DMA_FLAG_TEIF3);	//����жϱ�־
}

/* ================================================================== */
void USART3_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart3_dma_tx_buffer),data,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // ����DMA����
}

/* ================================================================== */
void USART3_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// ��ȡ�ַ����ַ�����
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart3_dma_tx_buffer),pstr,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // ����DMA����
}

//===================================================================
uint8_t USART3_ReceiveData(uint8_t **data, uint16_t* size)
{  
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Usart3ReceiveSem, osWaitForever);
  if (stat == osOK)
  {
    *data = usart3_rx_buffer;
    *size = usart3_rx_size;
    return 1;
  }

  return 0;
}

//-----�ļ�MomiHW.c����---------------------------------------------
