/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: AuxComHW.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version:  V1.0
* @Date:     2021-1-12
* @brief:    本文件为4G模块硬件驱动层的头文件
******************************************************************************/

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "AuxComHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart6TransmitSem = NULL;  // 串口发送信号量
uint8_t usart6_dma_tx_buffer[USART6_TX_BUFFER_SIZE];

volatile uint16_t usart6_rx_size;
osSemaphoreId_t sid_Usart6ReceiveSem = NULL;  // 串口接收信号量
uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];
uint8_t usart6_dma_rx_buffer[USART6_RX_BUFFER_SIZE];

/*****************************************************************************
 * USART6的TX/RX在PB10(TXD)和PB11(RXD)
 ****************************************************************************/
void USART6_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS信号量定义
  if(sid_Usart6ReceiveSem==NULL)
  {  sid_Usart6ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart6TransmitSem==NULL)
  {  sid_Usart6TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // 开启时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //开启DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 开启GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  // 开启USART6时钟

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART6, ENABLE);  // 串口6使能
  USART_ClearFlag(USART6, USART_FLAG_TC);  // 清除发送完成标志

  //接收DMA
  DMA_DeInit(DMA2_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;                          // 通道5
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6_dma_rx_buffer; // 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 外设到内存
  DMA_InitStructure.DMA_BufferSize = USART6_RX_BUFFER_SIZE;               // 缓冲大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设地址不增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存地址增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据大小1byte
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据大小1byte
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                         // 循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                     // 优先级高
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 不开fifo
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);                             // 初始化dma1流1
  DMA_Cmd(DMA2_Stream1, ENABLE);                                          // 开启dma1流1
  USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);                          // 开启串口6DMA接收

  //发送DMA配置
  DMA_DeInit(DMA2_Stream6);
  DMA_InitStructure.DMA_Channel =DMA_Channel_5;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart6_dma_tx_buffer; // 内存地址
  DMA_InitStructure.DMA_BufferSize = USART6_TX_BUFFER_SIZE;               // 传输大小
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
  DMA_Init(DMA2_Stream6, &DMA_InitStructure);

  DMA_ITConfig(DMA2_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
  USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);                          // 开启串口DMA发送
}

/* =====================串口6空闲中断服务函数========================== */
void USART6_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) // 空闲中断
  {
    it = USART6->SR;
    it = USART6->DR;    // 先读SR,后读DR可清除空闲中断标志位
    it = it;
    
    USART_ClearITPendingBit(USART6,USART_IT_IDLE);	// 清除空闲中断标志
    DMA_Cmd(DMA2_Stream1, DISABLE);//关闭DMA,防止处理其间有数据
    usart6_rx_size = USART6_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1); // 计算收到的数据数量
    memcpy(usart6_rx_buffer, usart6_dma_rx_buffer, usart6_rx_size);
    usart6_rx_buffer[usart6_rx_size] = '\0';         // 最后一位清零，便于处理字符串
    DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_BUFFER_SIZE); // 重新设置DMA通道传输大小
    DMA_Cmd(DMA2_Stream1, ENABLE); // 开启DMA
    osSemaphoreRelease(sid_Usart6ReceiveSem);
  }
}

/* =====================串口3DMA发送结束中断========================== */
void DMA2_Stream6_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6))//判断是否接收到中断
  {
    osSemaphoreRelease(sid_Usart6TransmitSem);
  }
  DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6); //清除中断标志
}

/* ================================================================== */
void USART6_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART6_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart6TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA2_Stream6, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart6_dma_tx_buffer),data,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA2_Stream6, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA2_Stream6, ENABLE);                      // 开启DMA发送
}

/* ================================================================== */
void USART6_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// 获取字符串字符数量
  if ((size==0)||(size>USART6_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart6TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA2_Stream6, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart6_dma_tx_buffer),pstr,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA2_Stream6, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA2_Stream6, ENABLE);                      // 开启DMA发送
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

//-----文件AuxComHW.c结束---------------------------------------------

