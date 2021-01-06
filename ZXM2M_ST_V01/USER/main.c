/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: main.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-10-16
* @brief     Main program body.
* @Micro     STM32F205VET6: 120MHz,512KB FLASH,128KB RAM
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"
//#include "GpsHW.h"
//#include "CelluraHW.h"

/******************************************************************************
* Typedef
******************************************************************************/


/******************************************************************************
* Define
******************************************************************************/


/******************************************************************************
* Macros
******************************************************************************/


/******************************************************************************
* Data Types and Globals
******************************************************************************/


/******************************************************************************
* Task Define
******************************************************************************/
// 任务的属性设置
#define AppThreadPriority_Start   osPriorityLow
osThreadId_t tid_AppStart = NULL;

const osThreadAttr_t AppThreadAttr_Start =
{
  .attr_bits = osThreadDetached,
  .priority = osPriorityHigh4,
  .stack_size = 1024, // 字节
};

/********************************************************************************
*	函 数 名: AppThread_Start
*	功能说明: 启动任务，这里用作BSP驱动包处理。
*	形    参: 无
*	返 回 值: 无
* 优 先 级: osPriorityLow
********************************************************************************/
void AppThread_Start(void *argument)
{
  osDelay(50); // 上电延时50MS

  /* 初始化外设bsp_init() */
  PcDebug_ServiceInit();
  rtc_Initialize();
  sfud_init();
  Can_ServiceInit();
  iCloud_ServiceInit();
  //GPS_ServiceInit();
  Momi_ServiceInit();
  Collect_ServiceInit();
  tbox_initialize();
  
  /* 创建任务AppTaskCreate() */
  iCloud_ServiceStart();
  Collect_ServiceStart();
  PcDebug_ServiceStart();
  Can_ServiceStart();
  //GPS_ServiceStart();
  Momi_ServiceStart();
  
  while (1)
  {
    osDelay(10);
    tbox_state_machine();  //工作模式处理
  }
}

/*----------------------------------------------------------------------------
 * Main: Initialize and start the RTOS2 Kernel
 *---------------------------------------------------------------------------*/
int main (void)
{
  /* 时钟初始化 */
  SystemCoreClockUpdate();

  /* 系统(用户硬件)初始化 */
  system_Initialize();

  /* 内核初始化 */
  osKernelInitialize();

  /* 创建启动任务 */
  tid_AppStart = osThreadNew(AppThread_Start, NULL, &AppThreadAttr_Start);

  /* 开启多任务 */
  osKernelStart();

  while (1);
}

/******************************************************************************
*
******************************************************************************/
void gpio_Initialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); // GPIO时钟使能

//**************************  开关量输出信号  **********************************
  GPIO_InitStructure.GPIO_Pin   = GPIOA_OUTPUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  WIFI_LED_ON();
  CAN_POWER_OFF();
  ETH_LED_ON();
  MODEM_VBUS_OFF();
  GPS_LED_OFF();
  CAN1_LED_OFF();
  CAN2_LED_OFF();
  DISABLE_MODEM_PWR();

  GPIO_InitStructure.GPIO_Pin   = GPIOB_OUTPUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  POWER_LED_ON();
  MAIN_POWER_ON();
  FLASH_CS_L();
  BAT_CHARGE_OFF();
  //MODEM_PWRKEY_H();

  GPIO_InitStructure.GPIO_Pin   = GPIOC_OUTPUT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  MODEM_EMMC_OFF();
  DISABLE_GPS_PWR();
  //MODEM_RESET_H();
  //MODEM_DTR_L();

  //PB7: I2C1_SDA 和 PB6: I2C1_SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  I2C1_SCL_H();
  I2C1_SDA_H();

  //*****************************开关量输入信号   **********************************
  GPIO_InitStructure.GPIO_Pin   = GPIOA_INPUT_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIOB_INPUT_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin   = GPIOC_INPUT_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/******************************************************************************
* system_Initialize() sets up registers, timers, etc.
* 占先式优先级(pre-emption priority)
* --高占先式优先级的中断事件会打断当前的主程序/中断程序运行—抢断式优先响应，俗称中断嵌套。
*
* 副(响应)优先级(subpriority)
* --在占先式优先级相同的情况下，高副优先级的中断优先被响应；
* --在占先式优先级相同的情况下，如果有低副优先级中断正在执行，
*   高副优先级的中断要等待已被响应的低副优先级中断执行结束后
*   才能得到响应—非抢断式响应(不能嵌套)。
******************************************************************************/
void system_Initialize(void)
{
  //_enable_interrupts();	// 开启所有中断
  // STM32移植freeRTOS时的中断优先级配置应为NVIC_PriorityGroup_4
  // 优先级分组设置为4，可配置0-15级抢占式优先级，0级子优先级，即不存在子优先级
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 4位用于指定抢占式优先级
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  //使能电源和后备寄存器时钟
  PWR_BackupAccessCmd(ENABLE);   // 使能对后备寄存器的访问

  gpio_Initialize();
  IWDG_Init();
}

/********************************************************
 * 功能描述: 延时函数
 * 输    入：延时毫秒数
 * 输    出：无
********************************************************/
void delay_ms(uint32_t t)
{
  uint32_t i;

  while (t--)
  {
    for (i = 0; i<8030; i++)
    {
      __nop();
    }
    IWDG_Feed();
  }
}

/*******************************************************************************
 *
 *******************************************************************************/
void delay_us(uint32_t nTimer)
{
  volatile uint32_t i = 0;
  for (i=0; i<nTimer; i++)
  {
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    __NOP();
  }
  IWDG_Feed();
}

/******************************************************************************
 * 初始化独立看门狗
 * prer:分频数:0~7(只有低 3 位有效!)
 * 分频因子=4*2^prer.但最大值只能是 256!
 * rlr:重装载寄存器值:低 11 位有效.
 * 时间计算(大概):Tout=((128*625)/40 = 2000(ms).
 * IWDG 2s溢出
 ******************************************************************************/
void IWDG_Init(void)
{
  /* IWDG timeout equal to 2000ms.
  The timeout may varies due to LSI frequency dispersion, the
  LSE value is centred around 32 KHz */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // 使能对寄存器IWDG_PR和IWDG_RLR的写操作
  IWDG_SetPrescaler(IWDG_Prescaler_128);        // 设置IWDG预分频值:设置IWDG预分频值
  IWDG_SetReload(625);                          // 设置IWDG重装载值
  IWDG_ReloadCounter();                         // 按照IWDG重装载寄存器的值重装载IWDG计数器
  IWDG_Enable();                                // 使能IWDG
}

#if 0
//==喂独立看门狗==============================================================
void IWDG_Feed(void)
{
  IWDG_ReloadCounter();  // Reload IWDG counter
}
#endif

#if 0
/******************************************************************************
 * @brief  Configure the RTC peripheral by selecting the clock source.
 * 配置周期唤醒定时器,可把MCU从停止和待机模式下唤醒(EXTI22)
 * RTC属于电池供电域,常用于在没有外界中断的情况下，把MCU从低功耗模式唤醒
 * ->自唤醒模式(Auto-Wakeup)
 * RTC的三个时钟源中只有两个能达到以上目的:
 * 1.LSE(32.768KHz)属于电池供电域,即使Vdd掉电仍可工作
 * 2.LSI(32KHz)系统进入低功耗模式,只要Vdd还在,仍可工作
 * 唤醒源: 
 * 1.RTC报警(RTC AlarmA&B:EXTI17) 
 * 2.RTC唤醒(RTC Wakeup:EXTI22)
 * 3.RTC入侵(EXTI21)
******************************************************************************/
void iRTC_Initialize(void)
{
  // To wake up the device from the Stop mode with an RTC wakeup event, it is necessary to:
  // a) Configure the EXTI Line 22 to be sensitive to rising edges (Interrupt or Eventmodes)
  // b) Enable the RTC wakeup interrupt in the RTC_CR register
  // c) Configure the RTC to generate the RTC Wakeup event  

  RTC_InitTypeDef   RTC_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
  EXTI_InitTypeDef EXTI_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable the PWR clock
  PWR_BackupAccessCmd(ENABLE); // Allow access to RTC

  //==LSI used as RTC source clock============================
  RCC_LSICmd(ENABLE); // Enable the LSI OSC

  while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET) // Wait till LSI is ready
  {
  }

  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI); // Select the RTC Clock Source
  
  RCC_RTCCLKCmd(ENABLE); // Enable the RTC Clock

  RTC_WaitForSynchro(); // Wait for RTC APB registers synchronisation

  //==Calendar Configuration with LSI supposed at 32KHz==
  RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
  RTC_InitStructure.RTC_SynchPrediv = 0xFF; /* (32KHz / 128) - 1 = 0xFF*/
  RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
  RTC_Init(&RTC_InitStructure);
  
  //==Configure the RTC WakeUp Clock source: CK_SPRE (1Hz)====
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  RTC_SetWakeUpCounter(0x0);
  RTC_ITConfig(RTC_IT_WUT, ENABLE); // Enable the RTC Wakeup Interrupt

  //==EXTI configuration======================================
  EXTI_ClearITPendingBit(EXTI_Line22);
  EXTI_InitStructure.EXTI_Line = EXTI_Line22;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  
  //==Enable the RTC Wakeup Interrupt=========================
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // 中断占先等级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // 中断响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void iRTC_EnalbeWakeupCnt(void)
{
  RTC_WakeUpCmd(ENABLE); // 使能唤醒计数器
}

void iRTC_DisalbeWakeupCnt(void)
{
  RTC_WakeUpCmd(DISABLE); // 关闭唤醒计数器
}

extern uint32_t light_sleep_wkp_cnt;
/******************************************************************************
 * This function handles RTC Auto wake-up interrupt request.
 ******************************************************************************/
void RTC_WKUP_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
  {
    light_sleep_wkp_cnt++;
    RTC_ClearITPendingBit(RTC_IT_WUT);
    EXTI_ClearITPendingBit(EXTI_Line22);
  }
}

/******************************************************************************
 * This function handles RTC Alarms interrupt request.
 ******************************************************************************/
void RTC_Alarm_IRQHandler(void)
{
  if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
  {
    RTC_ClearITPendingBit(RTC_IT_ALRA);
    EXTI_ClearITPendingBit(EXTI_Line17);
  } 
}
#endif

//-----文件main.c结束---------------------------------------------
