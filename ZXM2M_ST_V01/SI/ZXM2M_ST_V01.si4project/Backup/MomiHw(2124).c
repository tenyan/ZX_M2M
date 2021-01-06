/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: MomiHw.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-20
* @brief     本文件为4G模块硬件驱动层的头文件
******************************************************************************/

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "MomiHw.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart3ReceiveSem = NULL;  // 串口接收信号量
uint8_t usart3_dma_tx_buffer[USART3_TX_BUFFER_SIZE];

volatile uint16_t usart3_rx_size;
osSemaphoreId_t sid_Usart3TransmitSem = NULL;  // 串口发送信号量
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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE); // GPIO时钟使能

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

  // MODEM上电状态
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN; // GPIO_PuPd_NOPULL
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*****************************************************************************
 * USART3的TX/RX在PB10(TXD)和PB11(RXD)
 ****************************************************************************/
void USART3_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS信号量定义
  if(sid_Usart3ReceiveSem==NULL)
  {  sid_Usart3ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart3TransmitSem==NULL)
  {  sid_Usart3TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // 开启时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); //开启DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // 开启GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // 开启USART1时钟

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART3, ENABLE);  // 串口3使能
  USART_ClearFlag(USART3, USART_FLAG_TC);  // 清除发送完成标志

  //接收DMA
  DMA_DeInit(DMA1_Stream1);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // 通道4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart3_dma_rx_buffer; // 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 外设到内存
  DMA_InitStructure.DMA_BufferSize = USART3_RX_BUFFER_SIZE;               // 缓冲大小
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
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);                             // 初始化dma1流1
  DMA_Cmd(DMA1_Stream1, ENABLE);                                          // 开启dma1流1
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);                          // 开启串口3DMA接收

  //发送DMA配置
  DMA_DeInit(DMA1_Stream3);
  DMA_InitStructure.DMA_Channel =DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART3->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart3_dma_tx_buffer; // 内存地址
  DMA_InitStructure.DMA_BufferSize = USART3_TX_BUFFER_SIZE;               // 传输大小
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
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Stream3, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3| DMA_FLAG_TEIF3);
  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);                          // 开启串口DMA发送
}

/* =====================串口3空闲中断服务函数========================== */
void USART3_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET) // 空闲中断
  {
    it = USART3->SR;
    it = USART3->DR;    // 先读SR,后读DR可清除空闲中断标志位
    
    USART_ClearITPendingBit(USART3,USART_IT_IDLE);	// 清除空闲中断标志
    DMA_Cmd(DMA1_Stream1, DISABLE);//关闭DMA,防止处理其间有数据
    usart3_rx_size = USART3_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1); // 计算收到的数据数量
    for (it = 0; it < usart3_rx_size; it++)
    {
      momi_msg_queue_push(usart3_dma_rx_buffer[it]); // 将收到的数据放入队列
      usart3_rx_buffer[it] = usart3_dma_rx_buffer[it];
    }
    //memcpy(usart3_rx_buffer, usart3_dma_rx_buffer, usart3_rx_size);
    usart3_rx_buffer[usart3_rx_size] = '\0';         // 最后一位清零，便于处理字符串
    DMA_SetCurrDataCounter(DMA1_Stream1, USART3_RX_BUFFER_SIZE); // 重新设置DMA通道6传输大小
    DMA_Cmd(DMA1_Stream1, ENABLE); // 开启DMA
    osSemaphoreRelease(sid_Usart3ReceiveSem);
  }
}

/* =====================串口3DMA发送结束中断========================== */
void DMA1_Stream3_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream3,DMA_IT_TCIF3))//判断是否接收到中断
  {
    osSemaphoreRelease(sid_Usart3TransmitSem);
  }
  DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3 | DMA_FLAG_TEIF3);	//清除中断标志
}

/* ================================================================== */
void USART3_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart3_dma_tx_buffer),data,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // 开启DMA发送
}

/* ================================================================== */
void USART3_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// 获取字符串字符数量
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart3_dma_tx_buffer),pstr,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // 开启DMA发送
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

//-----文件MomiHW.c结束---------------------------------------------
