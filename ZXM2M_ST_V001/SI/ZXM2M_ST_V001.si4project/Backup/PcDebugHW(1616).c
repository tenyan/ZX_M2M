/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebugHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-9-21
 * @brief:
 *******************************************************************************/

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "PcDebugHW.h"
#include "cmsis_os2.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart1ReceiveSem = NULL;  // ���Դ��ڽ����ź���
__align(8) uint8_t usart1_dma_tx_buffer[USART1_TX_BUFFER_SIZE];

volatile uint16_t usart1_rx_size;
osSemaphoreId_t sid_Usart1TransmitSem = NULL;  // ���Դ��ڷ����ź���
__align(8) uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
__align(8) uint8_t usart1_dma_rx_buffer[USART1_RX_BUFFER_SIZE];

/*****************************************************************************
 * USART1��TX/RX��PA9(TXD)��PA10(RXD)
 ****************************************************************************/
void USART1_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS�ź�������
  if(sid_Usart1ReceiveSem==NULL)
  {  sid_Usart1ReceiveSem = osSemaphoreNew(1, 1, NULL);}

  if(sid_Usart1TransmitSem==NULL)
  {  sid_Usart1TransmitSem = osSemaphoreNew(1, 1, NULL);}

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //����DMAʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ����GPIOʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // ����USART1ʱ��

  // USART GPIOs configuration
  // Configure USART1_Tx(PA9) and USART1_Rx(PA10) pins
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); // Connect PA9 to USART1_Tx
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1); // Connect PA10 to USART1_Rx

  // Configure USART Tx and Rx as alternate function
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);


  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         // ͨ������Ϊ����1�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // ʹ���ж�
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // ͨ������Ϊ����1�ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // ʹ���ж�
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);
  USART_Cmd(USART1, ENABLE);

  //����DMA����
  DMA_DeInit(DMA2_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                            // ͨ��4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart1_dma_rx_buffer;   // �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                   // ���赽�ڴ�
  DMA_InitStructure.DMA_BufferSize = USART1_RX_BUFFER_SIZE;                 // �����С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          // �����ַ����
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   // �ڴ��ַ��
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   // �������ݴ�С1byte
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           // �ڴ����ݴ�С1byte
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                           // ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                       // ���ȼ���
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                    // ����fifo
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream5, &DMA_InitStructure);                               // ��ʼ��dma2��5

  DMA_Cmd(DMA2_Stream5, ENABLE);  //����dma2��5
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // ��������1DMA����

  //����DMA����
  DMA_DeInit(DMA2_Stream7);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
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
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart1_dma_tx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_BufferSize = USART1_TX_BUFFER_SIZE;               // �����С
  DMA_Init(DMA2_Stream7,&DMA_InitStructure);

  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7| DMA_FLAG_TEIF7);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // ��������DMA����
}

//==�����жϷ�����====================================================
void USART1_IRQHandler(void)
{
  uint16_t i;

  i = i;
  if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
  {
    i = USART1->SR;
    i = USART1->DR;
    USART_ClearITPendingBit(USART1,USART_IT_IDLE);
    DMA_Cmd(DMA2_Stream5, DISABLE);  // DMAͨ����ֹ���رպ�ſ�����DMA����
    usart1_rx_size = USART1_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5); //�����յ�����������
    memcpy(usart1_rx_buffer, usart1_dma_rx_buffer, usart1_rx_size);
    usart1_rx_buffer[usart1_rx_size] = '\0';
    DMA_SetCurrDataCounter(DMA2_Stream5, USART1_RX_BUFFER_SIZE); // ��������DMAͨ��5�����С
    DMA_Cmd(DMA2_Stream5, ENABLE); // ����DMA
    osSemaphoreRelease(sid_Usart1ReceiveSem);
  }
}

//==DMA���ͽ����ж�===============================================
void DMA2_Stream7_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
  {
    osSemaphoreRelease(sid_Usart1TransmitSem);
  }
  DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7 | DMA_FLAG_TEIF7); // clear DMA flag
}

//===================================================================
void USART1_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART1_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart1TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA2_Stream7, DISABLE);                     // ��ֹDMAͨ��
  memcpy((uint8_t*)(usart1_dma_tx_buffer),data,size); // ��������
  DMA_SetCurrDataCounter(DMA2_Stream7, size);         // �޸�DMA������������
  DMA_Cmd(DMA2_Stream7, ENABLE);                      // ʹ��DMAͨ��
}

//===================================================================
void USART1_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	//��ȡ�ַ����ַ�����

  if ((size==0)||(size>USART1_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart1TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA2_Stream7, DISABLE);                     // ��ֹDMAͨ��
  memcpy((uint8_t*)(usart1_dma_tx_buffer),pstr,size); // ��������
  DMA_SetCurrDataCounter(DMA2_Stream7, size);         // �޸�DMA������������
  DMA_Cmd(DMA2_Stream7, ENABLE);                      // ʹ��DMAͨ��
}

//===================================================================
uint8_t USART1_ReceiveData(uint8_t **data, uint16_t* size)
{
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Usart1ReceiveSem, osWaitForever);
  if (stat == osOK)
  {
    *data = usart1_rx_buffer;
    *size = usart1_rx_size;
    return 1;
  }

  return 0;
}

//-----�ļ�PcHardware.c����---------------------------------------------
