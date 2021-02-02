/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebugHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-9-21
 * @brief:
 *******************************************************************************/

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "PcDebugHW.h"
#include "cmsis_os2.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart1ReceiveSem = NULL;  // 调试串口接收信号量
__align(8) uint8_t usart1_dma_tx_buffer[USART1_TX_BUFFER_SIZE];

volatile uint16_t usart1_rx_size;
osSemaphoreId_t sid_Usart1TransmitSem = NULL;  // 调试串口发送信号量
__align(8) uint8_t usart1_rx_buffer[USART1_RX_BUFFER_SIZE];
__align(8) uint8_t usart1_dma_rx_buffer[USART1_RX_BUFFER_SIZE];

/*****************************************************************************
 * USART1的TX/RX在PA9(TXD)和PA10(RXD)
 ****************************************************************************/
void USART1_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS信号量定义
  if(sid_Usart1ReceiveSem==NULL)
  {  sid_Usart1ReceiveSem = osSemaphoreNew(1, 1, NULL);}

  if(sid_Usart1TransmitSem==NULL)
  {  sid_Usart1TransmitSem = osSemaphoreNew(1, 1, NULL);}

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //开启DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 开启GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // 开启USART1时钟

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


  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         // 通道设置为串口1中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能中断
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;   // 通道设置为串口1中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x08; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // 使能中断
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART1, USART_IT_IDLE , ENABLE);
  USART_Cmd(USART1, ENABLE);

  //接收DMA配置
  DMA_DeInit(DMA2_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                            // 通道4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart1_dma_rx_buffer;   // 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                   // 外设到内存
  DMA_InitStructure.DMA_BufferSize = USART1_RX_BUFFER_SIZE;                 // 缓冲大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          // 外设地址不增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                   // 内存地址增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;   // 外设数据大小1byte
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;           // 内存数据大小1byte
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                           // 循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                       // 优先级高
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                    // 不开fifo
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream5, &DMA_InitStructure);                               // 初始化dma2流5

  DMA_Cmd(DMA2_Stream5, ENABLE);  //开启dma2流5
  USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); // 开启串口1DMA接收

  //发送DMA配置
  DMA_DeInit(DMA2_Stream7);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度：字节
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度：字节
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMA模式：一次传输，非循环
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级：高
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 // 方向：内存-->外设  从存储器读
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart1_dma_tx_buffer; // 内存地址
  DMA_InitStructure.DMA_BufferSize = USART1_TX_BUFFER_SIZE;               // 传输大小
  DMA_Init(DMA2_Stream7,&DMA_InitStructure);

  DMA_ITConfig(DMA2_Stream7, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7| DMA_FLAG_TEIF7);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  // 开启串口DMA发送
}

//==空闲中断服务函数====================================================
void USART1_IRQHandler(void)
{
  uint16_t i;

  i = i;
  if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
  {
    i = USART1->SR;
    i = USART1->DR;
    USART_ClearITPendingBit(USART1,USART_IT_IDLE);
    DMA_Cmd(DMA2_Stream5, DISABLE);  // DMA通道禁止，关闭后才可设置DMA参数
    usart1_rx_size = USART1_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5); //计算收到的数据数量
    memcpy(usart1_rx_buffer, usart1_dma_rx_buffer, usart1_rx_size);
    usart1_rx_buffer[usart1_rx_size] = '\0';
    DMA_SetCurrDataCounter(DMA2_Stream5, USART1_RX_BUFFER_SIZE); // 重新设置DMA通道5传输大小
    DMA_Cmd(DMA2_Stream5, ENABLE); // 开启DMA
    osSemaphoreRelease(sid_Usart1ReceiveSem);
  }
}

//==DMA发送结束中断===============================================
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
  osSemaphoreAcquire(sid_Usart1TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA2_Stream7, DISABLE);                     // 禁止DMA通道
  memcpy((uint8_t*)(usart1_dma_tx_buffer),data,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA2_Stream7, size);         // 修改DMA发送数据数量
  DMA_Cmd(DMA2_Stream7, ENABLE);                      // 使能DMA通道
}

//===================================================================
void USART1_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	//获取字符串字符数量

  if ((size==0)||(size>USART1_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart1TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA2_Stream7, DISABLE);                     // 禁止DMA通道
  memcpy((uint8_t*)(usart1_dma_tx_buffer),pstr,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA2_Stream7, size);         // 修改DMA发送数据数量
  DMA_Cmd(DMA2_Stream7, ENABLE);                      // 使能DMA通道
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

//-----文件PcHardware.c结束---------------------------------------------
