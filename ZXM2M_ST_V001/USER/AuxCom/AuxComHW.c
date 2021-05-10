/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: AuxComHW.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version:  V1.0
* @Date:     2021-1-12
* @brief:    ���ļ�Ϊ4Gģ��Ӳ���������ͷ�ļ�
******************************************************************************/

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "AuxComHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart6TransmitSem = NULL;  // ���ڷ����ź���
uint8_t usart6_dma_tx_buffer[USART6_TX_BUFFER_SIZE];

volatile uint16_t usart6_rx_size;
osSemaphoreId_t sid_Usart6ReceiveSem = NULL;  // ���ڽ����ź���
uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];
uint8_t usart6_dma_rx_buffer[USART6_RX_BUFFER_SIZE];

/*****************************************************************************
 * USART6��TX/RX��PB10(TXD)��PB11(RXD)
 ****************************************************************************/
void USART6_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS�ź�������
  if(sid_Usart6ReceiveSem==NULL)
  {  sid_Usart6ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart6TransmitSem==NULL)
  {  sid_Usart6TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // ����ʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //����DMAʱ��
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // ����GPIOʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  // ����USART6ʱ��

  // USART GPIOs configuration
  // Configure USART6_Tx(PC6) and USART6_Rx(PC7) pins
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  // Connect PC6 to USART6_Tx
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);  // Connect PC7 to USART6_Rx

  // Configure USART Tx and Rx as alternate function
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // USART configuration
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART6, &USART_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART6, ENABLE);  // ����6ʹ��
  USART_ClearFlag(USART6, USART_FLAG_TC);  // ���������ɱ�־

  //����DMA
  DMA_DeInit(DMA2_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;                          // ͨ��5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6_dma_rx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // ���赽�ڴ�
  DMA_InitStructure.DMA_BufferSize = USART6_RX_BUFFER_SIZE;               // �����С
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
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);                             // ��ʼ��dma1��1
  DMA_Cmd(DMA2_Stream1, ENABLE);                                          // ����dma1��1
  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);                          // ��������6DMA����

  //����DMA����
  DMA_DeInit(DMA2_Stream6);
  DMA_InitStructure.DMA_Channel =DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6_dma_tx_buffer; // �ڴ��ַ
  DMA_InitStructure.DMA_BufferSize = USART6_TX_BUFFER_SIZE;               // �����С
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
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);

  DMA_ITConfig(DMA2_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
  USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);                          // ��������DMA����
}

/* =====================����6�����жϷ�����========================== */
void USART6_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) // �����ж�
  {
    it = USART6->SR;
    it = USART6->DR;    // �ȶ�SR,���DR����������жϱ�־λ
    it = it;
    
    USART_ClearITPendingBit(USART6,USART_IT_IDLE);	// ��������жϱ�־
    DMA_Cmd(DMA2_Stream1, DISABLE);//�ر�DMA,��ֹ�������������
    usart6_rx_size = USART6_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1); // �����յ�����������
    memcpy(usart6_rx_buffer, usart6_dma_rx_buffer, usart6_rx_size);
    usart6_rx_buffer[usart6_rx_size] = '\0';         // ���һλ���㣬���ڴ����ַ���
    DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_BUFFER_SIZE); // ��������DMAͨ�������С
    DMA_Cmd(DMA2_Stream1, ENABLE); // ����DMA
    osSemaphoreRelease(sid_Usart6ReceiveSem);
  }
}

/* =====================����3DMA���ͽ����ж�========================== */
void DMA2_Stream6_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6))//�ж��Ƿ���յ��ж�
  {
    osSemaphoreRelease(sid_Usart6TransmitSem);
  }
  DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6); //����жϱ�־
}

/* ================================================================== */
void USART6_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART6_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart6TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA2_Stream6, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart6_dma_tx_buffer),data,size); // ��������
  DMA_SetCurrDataCounter(DMA2_Stream6, size);         // �޸�DMA ������������
  DMA_Cmd(DMA2_Stream6, ENABLE);                      // ����DMA����
}

/* ================================================================== */
void USART6_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// ��ȡ�ַ����ַ�����
  if ((size==0)||(size>USART6_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart6TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA2_Stream6, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart6_dma_tx_buffer),pstr,size); // ��������
  DMA_SetCurrDataCounter(DMA2_Stream6, size);         // �޸�DMA ������������
  DMA_Cmd(DMA2_Stream6, ENABLE);                      // ����DMA����
}

//===================================================================
uint8_t USART6_ReceiveData(uint8_t **data, uint16_t* size)
{  
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Usart6ReceiveSem, osWaitForever);
  if (stat == osOK)
  {
    *data = usart6_rx_buffer;
    *size = usart6_rx_size;
    return 1;
  }

  return 0;
}

//-----�ļ�AuxComHW.c����---------------------------------------------

