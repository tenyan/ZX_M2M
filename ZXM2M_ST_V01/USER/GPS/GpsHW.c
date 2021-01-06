/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-10-11
 * @brief:    GPS模块底层驱动函数
 *******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "GpsHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart2ReceiveSem = NULL;  // 串口接收信号量
uint8_t usart2_dma_tx_buffer[USART2_TX_BUFFER_SIZE];

volatile uint16_t usart2_rx_size;
osSemaphoreId_t sid_Usart2TransmitSem = NULL;  // 串口发送信号量
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
 * USART2的TX/RX在PA2(TXD)和PA3(RXD)
 ****************************************************************************/
void USART2_Initialize(uint32_t baudrate)
{
  USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // RTOS信号量定义
  if(sid_Usart2ReceiveSem==NULL)
  {  sid_Usart2ReceiveSem = osSemaphoreNew(1, 0, NULL);}

  if(sid_Usart2TransmitSem==NULL)
  {  sid_Usart2TransmitSem = osSemaphoreNew(1, 1, NULL);}

  // 开启时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // 开启DMA时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 开启GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // 开启USART2时钟

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x04; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
  USART_Cmd(USART2, ENABLE);  // 串口2使能
  USART_ClearFlag(USART2, USART_FLAG_TC);  // 清除发送完成标志

  //接收DMA
  DMA_DeInit(DMA1_Stream5);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;                          // 通道4
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_dma_rx_buffer; // 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                 // 外设到内存
  DMA_InitStructure.DMA_BufferSize = USART2_RX_BUFFER_SIZE;               // 缓冲大小
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
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);                             // 初始化dma1流1
  DMA_Cmd(DMA1_Stream5, ENABLE);                                          // 开启dma1流1
  USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);                          // 开启串口2 DMA接收

  //发送DMA配置
  DMA_DeInit(DMA1_Stream6);
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);     // 外设地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)usart2_dma_tx_buffer; // 内存地址
  DMA_InitStructure.DMA_BufferSize = USART2_TX_BUFFER_SIZE;               // 设置DMA在传输时缓冲区的长度
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;        // 外设不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                 // 内存递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // 外设数据宽度：字节
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;         // 内存数据宽度：字节
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           // DMA模式：一次传输，非循环
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                 // 优先级：高
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                  // 指定如果FIFO模式或直接模式将用于指定的流：不使能FIFO模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;       // 指定了FIFO阈值水平
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;             // 指定的Burst转移配置内存传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;     // 指定的Burst转移配置外围转移
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                 // 方向：内存-->外设  从存储器读
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);

  DMA_ITConfig(DMA1_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
  DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
  USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);                          // 开启串口DMA发送
}

/* =====================串口2空闲中断服务函数========================== */
void USART2_IRQHandler(void)
{
  uint16_t it;

  if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET) // 空闲中断
  {
    it = USART2->SR;
    it = USART2->DR;    // 先读SR,后读DR可清除空闲中断标志位

    USART_ClearITPendingBit(USART2,USART_IT_IDLE);	// 清除空闲中断标志
    DMA_Cmd(DMA1_Stream5, DISABLE);                 // 关闭DMA,防止处理其间有数据
    usart2_rx_size = USART2_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5); // 计算收到的数据数量
    for (it = 0; it < usart2_rx_size; it++)
    {
      nmea_msg_queue_push(usart2_dma_rx_buffer[it]); // 将收到的数据放入队列
      usart2_rx_buffer[it] = usart2_dma_rx_buffer[it];
    }
    //memcpy(usart2_rx_buffer, usart2_dma_rx_buffer, usart2_rx_size);
    usart2_rx_buffer[usart2_rx_size] = '\0';          // 最后一位清零，便于处理字符串
    DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_BUFFER_SIZE); // 重新设置DMA通道传输大小
    DMA_Cmd(DMA1_Stream5, ENABLE); // 开启DMA
    osSemaphoreRelease(sid_Usart2ReceiveSem);
  }
}

/* =====================串口2DMA发送结束中断========================== */
void DMA1_Stream6_IRQHandler(void)
{
  if (DMA_GetITStatus(DMA1_Stream6,DMA_IT_TCIF6))//判断是否接收到中断
  {
    osSemaphoreRelease(sid_Usart2TransmitSem);
  }
  DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6);	//清除中断标志
}


/* ================================================================== */
void USART2_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART2_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart2TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream6, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart2_dma_tx_buffer),data,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream6, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream6, ENABLE);                      // 开启DMA发送
}

/* ================================================================== */
void USART2_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// 获取字符串字符数量
  if ((size==0)||(size>USART2_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart2TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream6, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart2_dma_tx_buffer),pstr,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream6, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream6, ENABLE);                      // 开启DMA发送
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
