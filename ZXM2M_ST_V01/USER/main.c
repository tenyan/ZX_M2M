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
// �������������
#define AppThreadPriority_Start   osPriorityLow
osThreadId_t tid_AppStart = NULL;

const osThreadAttr_t AppThreadAttr_Start =
{
  .attr_bits = osThreadDetached,
  .priority = osPriorityHigh4,
  .stack_size = 1024, // �ֽ�
};

/********************************************************************************
*	�� �� ��: AppThread_Start
*	����˵��: ����������������BSP����������
*	��    ��: ��
*	�� �� ֵ: ��
* �� �� ��: osPriorityLow
********************************************************************************/
void AppThread_Start(void *argument)
{
  osDelay(50); // �ϵ���ʱ50MS

  /* ��ʼ������bsp_init() */
  PcDebug_ServiceInit();
  rtc_Initialize();
  sfud_init();
  Can_ServiceInit();
  iCloud_ServiceInit();
  //GPS_ServiceInit();
  Momi_ServiceInit();
  Collect_ServiceInit();
  tbox_initialize();
  
  /* ��������AppTaskCreate() */
  iCloud_ServiceStart();
  Collect_ServiceStart();
  PcDebug_ServiceStart();
  Can_ServiceStart();
  //GPS_ServiceStart();
  Momi_ServiceStart();
  
  while (1)
  {
    osDelay(10);
    tbox_state_machine();  //����ģʽ����
  }
}

/*----------------------------------------------------------------------------
 * Main: Initialize and start the RTOS2 Kernel
 *---------------------------------------------------------------------------*/
int main (void)
{
  /* ʱ�ӳ�ʼ�� */
  SystemCoreClockUpdate();

  /* ϵͳ(�û�Ӳ��)��ʼ�� */
  system_Initialize();

  /* �ں˳�ʼ�� */
  osKernelInitialize();

  /* ������������ */
  tid_AppStart = osThreadNew(AppThread_Start, NULL, &AppThreadAttr_Start);

  /* ���������� */
  osKernelStart();

  while (1);
}

/******************************************************************************
*
******************************************************************************/
void gpio_Initialize(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE); // GPIOʱ��ʹ��

//**************************  ����������ź�  **********************************
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

  //PB7: I2C1_SDA �� PB6: I2C1_SCL
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  I2C1_SCL_H();
  I2C1_SDA_H();

  //*****************************�����������ź�   **********************************
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
* ռ��ʽ���ȼ�(pre-emption priority)
* --��ռ��ʽ���ȼ����ж��¼����ϵ�ǰ��������/�жϳ������С�����ʽ������Ӧ���׳��ж�Ƕ�ס�
*
* ��(��Ӧ)���ȼ�(subpriority)
* --��ռ��ʽ���ȼ���ͬ������£��߸����ȼ����ж����ȱ���Ӧ��
* --��ռ��ʽ���ȼ���ͬ������£�����е͸����ȼ��ж�����ִ�У�
*   �߸����ȼ����ж�Ҫ�ȴ��ѱ���Ӧ�ĵ͸����ȼ��ж�ִ�н�����
*   ���ܵõ���Ӧ��������ʽ��Ӧ(����Ƕ��)��
******************************************************************************/
void system_Initialize(void)
{
  //_enable_interrupts();	// ���������ж�
  // STM32��ֲfreeRTOSʱ���ж����ȼ�����ӦΪNVIC_PriorityGroup_4
  // ���ȼ���������Ϊ4��������0-15����ռʽ���ȼ���0�������ȼ����������������ȼ�
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); // 4λ����ָ����ռʽ���ȼ�
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  //ʹ�ܵ�Դ�ͺ󱸼Ĵ���ʱ��
  PWR_BackupAccessCmd(ENABLE);   // ʹ�ܶԺ󱸼Ĵ����ķ���

  gpio_Initialize();
  IWDG_Init();
}

/********************************************************
 * ��������: ��ʱ����
 * ��    �룺��ʱ������
 * ��    ������
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
 * ��ʼ���������Ź�
 * prer:��Ƶ��:0~7(ֻ�е� 3 λ��Ч!)
 * ��Ƶ����=4*2^prer.�����ֵֻ���� 256!
 * rlr:��װ�ؼĴ���ֵ:�� 11 λ��Ч.
 * ʱ�����(���):Tout=((128*625)/40 = 2000(ms).
 * IWDG 2s���
 ******************************************************************************/
void IWDG_Init(void)
{
  /* IWDG timeout equal to 2000ms.
  The timeout may varies due to LSI frequency dispersion, the
  LSE value is centred around 32 KHz */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // ʹ�ܶԼĴ���IWDG_PR��IWDG_RLR��д����
  IWDG_SetPrescaler(IWDG_Prescaler_128);        // ����IWDGԤ��Ƶֵ:����IWDGԤ��Ƶֵ
  IWDG_SetReload(625);                          // ����IWDG��װ��ֵ
  IWDG_ReloadCounter();                         // ����IWDG��װ�ؼĴ�����ֵ��װ��IWDG������
  IWDG_Enable();                                // ʹ��IWDG
}

#if 0
//==ι�������Ź�==============================================================
void IWDG_Feed(void)
{
  IWDG_ReloadCounter();  // Reload IWDG counter
}
#endif

#if 0
/******************************************************************************
 * @brief  Configure the RTC peripheral by selecting the clock source.
 * �������ڻ��Ѷ�ʱ��,�ɰ�MCU��ֹͣ�ʹ���ģʽ�»���(EXTI22)
 * RTC���ڵ�ع�����,��������û������жϵ�����£���MCU�ӵ͹���ģʽ����
 * ->�Ի���ģʽ(Auto-Wakeup)
 * RTC������ʱ��Դ��ֻ�������ܴﵽ����Ŀ��:
 * 1.LSE(32.768KHz)���ڵ�ع�����,��ʹVdd�����Կɹ���
 * 2.LSI(32KHz)ϵͳ����͹���ģʽ,ֻҪVdd����,�Կɹ���
 * ����Դ: 
 * 1.RTC����(RTC AlarmA&B:EXTI17) 
 * 2.RTC����(RTC Wakeup:EXTI22)
 * 3.RTC����(EXTI21)
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0E; // �ж�ռ�ȵȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // �ж���Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void iRTC_EnalbeWakeupCnt(void)
{
  RTC_WakeUpCmd(ENABLE); // ʹ�ܻ��Ѽ�����
}

void iRTC_DisalbeWakeupCnt(void)
{
  RTC_WakeUpCmd(DISABLE); // �رջ��Ѽ�����
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

//-----�ļ�main.c����---------------------------------------------
