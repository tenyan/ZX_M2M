/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CollectHW.c
 * @Engineer: TenYan
 * @Company:  徐工信息智能硬件部
 * @version:  V1.0
 * @Date:     2020-10-10
 * @brief:    本文件为开关量采集功能模块综合层的实现文件
 *******************************************************************************/
#include "config.h"
#include "CollectHW.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define ADC_PIN_VBAT      GPIO_Pin_3  // PC3
#define ADC_PIN_VRAW      GPIO_Pin_2  // PC2

/******************************************************************************
 *   Data Types and Globals
 ******************************************************************************/
volatile uint8_t ADC_UPDATED_FLAG;

__IO uint16_t adc_dma_buffers[NUMBER_OF_ADC_CHANNEL];
adc_buf_type_t adc_buffers[NUMBER_OF_ADC_CHANNEL];

adc_result_type_t adc_result;

void adc_nb_sample_init(void);

/*************************************************************************
 * VRAW-PC2(ADC123_IN12),VBAT-PC3(ADC123_IN13)
*************************************************************************/
void ADC_Initialize(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /// Enable ADC1, DMA2 and GPIO clocks
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  DMA_DeInit(DMA2_Stream0);
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;        // DMA外设ADC基地址
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_dma_buffers; // adc_dma_buffers[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;            // 从外设读取发送到内存
  DMA_InitStructure.DMA_BufferSize = NUMBER_OF_ADC_CHANNEL;          // 顺序进行规则转换的ADC通道的数目 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // 外设地址不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;            // 内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // 16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                    // 模式为循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;                // DMA优先级高
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;             // 直接传输模式
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /// Configure ADC1 Channel10 and Channel11 pin as analog input
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure); // PC0和PC1

  /// ADC Common Init
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;     // 独立模式
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;  // //60M/4=15M  max value is 30MHz
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; // 采样周期
  ADC_CommonInit(&ADC_CommonInitStructure);  // 基本初始化

  /// ADC1 Init 
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;  // 多个通道扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; // 无外部触发
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;  // 触发模式
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // 右对齐方式  
  ADC_InitStructure.ADC_NbrOfConversion = NUMBER_OF_ADC_CHANNEL; // 通道数量  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_TempSensorVrefintCmd(ENABLE);  // 使能温度传感器和内部参考电压通道
  
  /// ADC1,规则采样顺序值,采样周期
  /// The temperature sensor, VREFINT and the VBAT channel are available only on the master ADC1 peripheral.
  ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_112Cycles); // VBAT_MEAS  ADC_SampleTime_112Cycles
  ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_112Cycles); // VRAW_MEAS
  ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 3, ADC_SampleTime_112Cycles);  // 内部温度
  ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 4, ADC_SampleTime_112Cycles);  // 参考电压=1.21V

  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); // Enable DMA request after last transfer (Single-ADC mode) 
  ADC_DMACmd(ADC1, ENABLE); // Enable ADC1 DMA
  ADC_Cmd(ADC1, ENABLE);    // Enable ADC1
  
  ADC_SoftwareStartConv(ADC1); // 使能软件转换功能

  adc_nb_sample_init();
}

/*************************************************************************
 *
*************************************************************************/
void adc_nb_sample_init(void)
{
  buf_vbat.av_nb_sample = ADC_CHANNEL_VBAT_NB_SAMPLES;
  buf_vraw.av_nb_sample = ADC_CHANNEL_VRAW_NB_SAMPLES;
  buf_inttemp.av_nb_sample = ADC_CHANNEL_INTTEMP_NB_SAMPLES;
  buf_vref.av_nb_sample = ADC_CHANNEL_VREF_NB_SAMPLES;
}

/*************************************************************************
 *
*************************************************************************/
__inline void adc_push_sample(adc_buf_type_t *buf, uint16_t value)
{
  uint8_t new_head = buf->head + 1;

  if (new_head >= buf->av_nb_sample) {
    new_head = 0;
  }
  buf->sum -= buf->values[new_head];
  buf->values[new_head] = value;
  buf->sum += value;
  buf->head = new_head;
}

/*************************************************************************
 * 100ms调用一次
*************************************************************************/
void adc_push_dma_sample(void)
{
  uint8_t i;

  for (i=0;i<NUMBER_OF_ADC_CHANNEL;i++)
  {
    adc_push_sample(&adc_buffers[i],adc_dma_buffers[i]);
  }
}

/******************************************************************************
 * PA0--RTC_WAKE_UP
 ******************************************************************************/
void RTC_WAKE_UP_GPIO_EXTI_Initialize(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // Enable GPIOA clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable SYSCFG clock

  // Configure PA0 pin as input floating 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //  外部已上拉GPIO_PuPd_NOPULL
  GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0); // Connect EXTI Line0 to PA0 pin

  // Configure EXTI Line0
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line0 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//============================================================================
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line0) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}

/******************************************************************************
 * PD3--OPEN_BOX_INT,开盖报警中断唤醒
 ******************************************************************************/
void Openbox_GPIO_EXTI_Initialize()
{
#if 0
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // Enable GPIOD clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable SYSCFG clock

  // Configure PD3 pin as input floating 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD,&GPIO_InitStructure);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD,EXTI_PinSource3); // Connect EXTI Line0 to PD3 pin

  // Configure EXTI Line3
  EXTI_InitStructure.EXTI_Line = EXTI_Line3;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line3 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif
}

//============================================================================
void EXTI3_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line3) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line3);
    Tbox_ResetSleepTime();
    IWDG_Feed();
  }
}

/******************************************************************************
 * PA8--RING_WAKE_UP  振铃PA8中断唤醒
 ******************************************************************************/
void RING_WAKE_GPIO_EXTI_Initialize(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // Enable GPIOD clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable SYSCFG clock

  // Configure PA8 pin as input floating 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA,&GPIO_InitStructure);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8); // Connect EXTI Line8 to PA8 pin

  // Configure EXTI Line8
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line12 Interrupt to EXTI9_5_IRQn lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/******************************************************************************
 * PC5--ACC_IN ACC中断唤醒
 ******************************************************************************/
void ACC_IN_GPIO_EXTI_Initialize(void)
{
  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); // Enable GPIOD clock
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable SYSCFG clock

  // Configure PC5 pin as input floating 
  GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC,&GPIO_InitStructure);
  
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource5); // Connect EXTI Line15 to PD15 pin

  // Configure EXTI Line5
  EXTI_InitStructure.EXTI_Line = EXTI_Line5;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // 下降沿
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable and set EXTI Line12 Interrupt to the lowest priority
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

//======================================================================
void EXTI9_5_IRQHandler(void)
{
  if (EXTI_GetITStatus(EXTI_Line5) != RESET) // ACC_IN
  {
    EXTI_ClearITPendingBit(EXTI_Line5);
    Tbox_ResetSleepTime();
    IWDG_Feed();
  }

#if 0
  if (EXTI_GetITStatus(EXTI_Line8) != RESET) // RING_WAKE_UP
  {
    EXTI_ClearITPendingBit(EXTI_Line8);
    Tbox_ResetSleepTime();
    IWDG_Feed();
  }
#endif
}

/******************************************************************************
 * 停机和待机模式的唤醒源
 * 1.外部中断线:EXTI0-15, EXTI16(PVD OUTPUT), EXTI17(RTC Alarm), 
 *   EXTI18(OTG FS Wakeup), EXTI19(Ethernet Wakeup), EXTI20(OTG HS Wakeup)
 *   EXTI21(RTC Tamper & timestamp), EXTI22(RTC Wakeup)
 * 2.其他内如信号:IWDG复位
 * 3.其他I/O引脚: WKUP引脚上的上升沿PA0和NRST引脚上的复位信号(独立引脚)
 ******************************************************************************/
void CollectHw_Init(void)
{
  ADC_Initialize();
  ACC_IN_GPIO_EXTI_Initialize();
  //Openbox_GPIO_EXTI_Initialize();
  //RING_WAKE_GPIO_EXTI_Initialize();
  RTC_WAKE_UP_GPIO_EXTI_Initialize();
}

//-----文件CollectHW.c结束---------------------------------------------
