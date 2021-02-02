/*****************************************************************************
* @FileName: tbox_machine.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-10-16
* @brief
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Typedef
******************************************************************************/

/******************************************************************************
* Define
******************************************************************************/


/******************************************************************************
* Macros
******************************************************************************/
#define TBOX_DELAY(ms)    do { osDelay(ms); } while(0)

/******************************************************************************
* Data Types and Globals
******************************************************************************/
// tbox state variable defines
tbox_mode_t tbox_mode;
volatile uint16_t tbox_10msec_timer;
uint8_t tbox_started_delay;
uint8_t tbox_state_transition;
tbox_state_t tbox_state;
tbox_iap_state_t tbox_iap_state;
tbox_sleep_state_t tbox_sleep_state;

uint8_t public_data_buffer[1460];

static uint8_t acc_off_event_flag = 0; // ACC由ON变为OFF标志位
static uint32_t acc_off_cumulated_timer = 0;
static uint32_t acc_off_reset_cumulated_timer = 0;
/******************************************************************************
 * 主电供电情况下,终端断电重启函数(1S执行一次)
 * 当ACC关闭持续时间达到休眠唤醒间隔时长时，通知AM1805对整机进行断电10秒钟
 * 然后重新上电;之后如果ACC仍持续为OFF状态，则每间隔24小时通知通知AM1805对
 * 整机进行断电10秒钟然后重新上电。
*******************************************************************************/
void Tbox_CheckPowerOffMachine(void)
{
  uint8_t current_acc_state;
  static uint8_t previous_acc_state = 0;

  current_acc_state = COLT_GetAccStatus();
  if(current_acc_state==0) // ACC关闭
  {
    if ((acc_off_cumulated_timer >= m2m_asset_data.sleep_time_sp*60) || (acc_off_reset_cumulated_timer >= 86400))
    {
      acc_off_cumulated_timer = 0;
      acc_off_reset_cumulated_timer = 0;
      
      PcDebug_SendString("Tbox:RtcPwrOff!\n");
      TBOX_DELAY(OS_TICKS_PER_SEC);
      
      am18x5_PswControl(1); // 通知AM1805断开终端外部和电池电源
      am18x5_SetSleepTime(10,2); // 通知AM1805(RTC芯片)断电10秒(关主电、关电池电)
      TBOX_DELAY(OS_TICKS_PER_SEC);
      MAIN_POWER_OFF();  // 关主电
      TBOX_DELAY(OS_TICKS_PER_SEC*3);
    }
  }

  if(previous_acc_state != current_acc_state)  // ACC状态跳变判断
  {
    if (current_acc_state==0) // ACC关闭
    {  acc_off_event_flag = 1;}
  }
  
  if ((current_acc_state==0) && (acc_off_event_flag==1))
  {  acc_off_cumulated_timer++;}
  else
  {  acc_off_cumulated_timer = 0;}

  if (current_acc_state==0) // ACC关闭
  {  acc_off_reset_cumulated_timer++;}
  else
  {  acc_off_reset_cumulated_timer = 0;}

  previous_acc_state = current_acc_state; // 更新状态
}

/****************************************************************************
 * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
 *         and select PLL as system clock source.
 * @param  None
 * @retval None
****************************************************************************/
void SYSCLKConfig_STOP(void)
{
  /* After wake-up from STOP reconfigure the system clock */
  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
    {}

  /* Enable PLL */
  RCC_PLLCmd(ENABLE);

  /* Wait till PLL is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {}

  /* Select PLL as system clock source */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

  /* Wait till PLL is used as system clock source */
  while (RCC_GetSYSCLKSource() != 0x08)
    {}
}

extern uint16_t adc_dma_buffers[NUMBER_OF_ADC_CHANNEL];
//uint32_t quit_sleep_timer_sp = 7200; // 2小时
uint32_t quit_sleep_timer_sp = 300; // 2小时
static uint32_t quit_sleep_timer; // 唤醒间隔定时器
//============================================================================
void Tbox_CheckSleepStatus(void)
{
  uint16_t vraw;    // 外电源电压,单位10mV
  static uint8_t acc_on_debounce_cnt = 0;
  static uint8_t voltage_full_debounce_cnt = 0;

  // 对ACC进行判断 防止中断无法正常唤醒
  if (ACC_IN()==0x00) // ACC开
  {
    if (acc_on_debounce_cnt > 5) // 5秒
      quit_sleep_timer = 0;
    else
      acc_on_debounce_cnt++;
  }
  else
  {
    acc_on_debounce_cnt = 0;
  }

  //对外电源电压进行判断 达到启动电压是唤醒
  vraw = (uint16_t) (adc_dma_buffers[ADC_CHANNEL_VRAW] * 121 / 125 + 27);  // 单位是0.01V
  if ((vraw>1300 && vraw<1510)|| (vraw>2650) || (vraw <500))
  {
    if (voltage_full_debounce_cnt > 5) // 5秒
      quit_sleep_timer = 0;
    else
      voltage_full_debounce_cnt++;
  }
  else
  {
    voltage_full_debounce_cnt = 0;
  }
  
  acc_off_cumulated_timer++;
  acc_off_reset_cumulated_timer++;
}

/*****************************************************************************
 * 复位休眠时间
 ******************************************************************************/
void Tbox_ResetSleepTime(void)
{
  quit_sleep_timer = 0;
}

//============================================================================
void tbox_power_on(void)
{
  rtc_date_t rtc_time;

  if (tbox_state_transition == TRUE)
  {
    tbox_state_transition = FALSE;
    tbox_started_delay = FALSE;
    tbox_mode = TBOX_MODE_WORKING;
  }

  if (tbox_started_delay == FALSE)
  {
    tbox_started_delay = TRUE;

    //==硬件使能=============================================
    MAIN_POWER_ON();     // 使能主电供电
    CAN_POWER_ON();
    //GPS_SetModuleState(GPS_MODULE_STATE_POWERON); // GPS模块开机
    Modem_SetState(MODEM_STATE_POWER_ON); // 4G模块开机
    
    rtc_time = RTC_GetBjTime(); // 读取RTC时间
    PcDebug_Printf("Tst:%d-%d-%d %d:%d:%d\n\r", rtc_time.year,rtc_time.month,rtc_time.day,rtc_time.hour,rtc_time.minute,rtc_time.second);
    rtcSoft_Init(&rtc_time);
    
    tbox_10msec_timer = 300; // 延时3S,用于自检
  }
  else
  {
    if (tbox_10msec_timer == 0)
    {
      tbox_state_transition = TRUE;
      tbox_state = TBOX_STATE_WORKING;
    }
  }
}

static uint32_t enter_sleep_timer_sp = 30000; // 5分钟
static uint32_t enter_sleep_timer;
//============================================================================
void tbox_working(void)
{ 
  if (tbox_state_transition == TRUE)
  {
    tbox_state_transition = FALSE;
    tbox_started_delay = FALSE;
    tbox_mode = TBOX_MODE_WORKING;
  }

  if (tbox_started_delay == FALSE)
  {
    tbox_started_delay = TRUE;
    enter_sleep_timer_sp = m2m_asset_data.wakeup_work_time_sp*100; // 10ms时基
    enter_sleep_timer = enter_sleep_timer_sp;
    tbox_10msec_timer = 100;
  }
  else
  {
    if (tbox_10msec_timer == 0)
    {
      if (COLT_GetAccStatus()==1) // ACC开
      {
        enter_sleep_timer = enter_sleep_timer_sp; // 重置休眠时间
      }

      if((CAN_GetRecvState(CAN_CHANNEL1)==CAN_OK)||(CAN_GetRecvState(CAN_CHANNEL2)==CAN_OK)) // CAN工作
      {
        enter_sleep_timer = enter_sleep_timer_sp; // 重置休眠时间
      }

      if(m2m_context.update.state > M2M_UPDATE_STATE_IDLE)  // 终端正在升级
      {
        enter_sleep_timer = enter_sleep_timer_sp; // 重置休眠时间
      }

      // 馈电检测和判断

      //========================================
      //if()  // 进入升级模式
      //{
      //  tbox_state_transition = TRUE;
      //  tbox_state = TBOX_STATE_IAP;
      //  return;
      //}

      //========================================
      if (enter_sleep_timer==0) // 进入休眠模式
      {
        tbox_state_transition = TRUE;
        tbox_state = TBOX_STATE_SLEEP;
        return;
      }
    }
  }
}

//uint32_t light_sleep_wkp_cnt = 0;
//============================================================================
void tbox_sleep(void)
{
  uint8_t it;
  
  if (tbox_state_transition == TRUE)
  {
    tbox_state_transition = FALSE;
    tbox_started_delay = FALSE;
    tbox_sleep_state = TBOX_SLEEP_STATE_INIT;
    tbox_mode = TBOX_MODE_SLEEP;
  }

  switch (tbox_sleep_state)
  {
  case TBOX_SLEEP_STATE_INIT: // 关闭硬件,确定进入那个休眠状态
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      //=关闭硬件====================================
      CAN_POWER_OFF();
      //GPS_SetModuleState(GPS_MODULE_STATE_SLEPT); // 关闭GPS模块
      tbox_10msec_timer = 100;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        if ((0==COLT_GetMainPowerStatus()) && (0==COLT_GetMainPowerLowStatus()))// 主电供电且主电不馈电
        {
          tbox_sleep_state = TBOX_SLEEP_STATE_LIGHT;
          tbox_started_delay = FALSE;
        }
        else
        {
          tbox_sleep_state = TBOX_SLEEP_STATE_DEEP;
          tbox_started_delay = FALSE;
        }
      }
    }
    break;

  case TBOX_SLEEP_STATE_LIGHT: // 轻度休眠(不断电,RTC一秒唤醒一次)
    /*
     * 在STM32开发中经常会用到独立看门狗（IWDG）和低功耗模式，看门狗是为了检测和解决由软件错误引起的故障，
     * 低功耗模式是为了在CPU不需要继续运行时进入到休眠模式用以节省电能。
     * 其中独立看门狗的时钟由独立的RC振荡器（STM32F10x一般为40kHz）提供，即使在主时钟出现故障时，也仍然有效，
     * 因此可以在停止和待机模式下工作。而且独立看门狗一旦启动，除了系统复位，它不能再被停止。
     * 但这样引发的一个问题是当MCU进入到低功耗模式后由于CPU停止运行无法喂狗，会导致系统频繁复位。
     */
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      PcDebug_SendString("Tbox:L-Sleep!\n");
      //=关闭硬件====================================
      Modem_SetState(MODEM_STATE_POWER_OFF); // 关闭4G模块
      for (it=0; it<60; it++) // 保证模块正常关机
      {
        TBOX_DELAY(OS_TICKS_PER_SEC);
        if(Modem_GetState()==MODEM_STATE_OFF) // 4G模块正常关闭
        {
          break;
        }
      }
      tbox_10msec_timer = 10;
      quit_sleep_timer = m2m_asset_data.sleep_time_sp * 60; // 两小时
      //quit_sleep_timer = 3 * 60; // 两小时
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        //PcDebug_Printf("L-SleepTime=%d\n",quit_sleep_timer);
        PcDebug_SendString("Tbox:L-Slept!\n");
        TBOX_DELAY(OS_TICKS_PER_SEC/10);
        BAT_CHARGE_OFF();

        osKernelLock();
        do
        {
          if (quit_sleep_timer)
          {
            quit_sleep_timer--;
          }
          Tbox_CheckSleepStatus();
          IWDG_Feed(); // 进入停机模式前喂狗
          am18x5_SetSleepTime(1,1);  // 1秒钟唤醒一次
          // 停止模式后IO口保持停止前的状态
          // PWR_STOPEntry_WFI=中断唤醒; PWR_STOPEntry_WFE=事件唤醒
          PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); // CPU进入停止模式,所有时钟(看门狗不停止)都停止(只有EXTI中断线和内部RTC闹钟能唤醒)
          // 等待唤醒(外部闹钟、CAN、开盖、ACC)
          IWDG_Feed(); // 唤醒后先喂狗
          SYSCLKConfig_STOP();  // 系统从停止模式被唤醒时,时钟使用HSI(8MHz)作为系统时钟
          am18x5_ClearIntFlag();
          if (quit_sleep_timer==0)
          {
            break;  // 休眠时间到
          }
        }while (quit_sleep_timer);
        osKernelUnlock();
        //PcDebug_Printf("L-SleepWkpCnt=%d\n",light_sleep_wkp_cnt);
        tbox_sleep_state = TBOX_SLEEP_STATE_EXIT;
        tbox_started_delay = FALSE;
      }
    }
    break;

  case TBOX_SLEEP_STATE_DEEP: // 深度休眠(断电,RTC工作,24小时唤醒一次)
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      PcDebug_SendString("Tbox:D-Sleep!\n");
      //=关闭硬件====================================
      BAT_CHARGE_OFF();
      Modem_SetState(MODEM_STATE_POWER_OFF); // 关闭4G模块
      for (it=0; it<60; it++) // 保证模块正常关机
      {
        TBOX_DELAY(OS_TICKS_PER_SEC);
        if(Modem_GetState()==MODEM_STATE_OFF) // 4G模块正常关闭
        {
          break;
        }
      }
      
      tbox_10msec_timer = 10;
      quit_sleep_timer = 86398;  // 24小时
      //quit_sleep_timer = 3 * 60; // 两小时
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        PcDebug_SendString("Tbox:D-Slept!\n");
        am18x5_PswControl(1);
        am18x5_SetSleepTime(quit_sleep_timer, 2);  // 设置闹钟,系统断电(关主电、关电池电)
        TBOX_DELAY(OS_TICKS_PER_SEC);
        MAIN_POWER_OFF();  // 关主电
        TBOX_DELAY(OS_TICKS_PER_SEC*3);
      }
    }
    break;

  case TBOX_SLEEP_STATE_EXIT: // 退出休眠,进入正常工作
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      //=唤醒硬件====================================
      PcDebug_SendString("Tbox:Wakeup!\n");

      IWDG_Feed();
      tbox_10msec_timer = 10;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        tbox_state_transition = TRUE;
        tbox_state = TBOX_STATE_POWERON;
      }
    }
    break;

  default:
    break;
  }
}

//============================================================================
void tbox_iap(void)
{
  static uint8_t rfu_status;
  
  if (tbox_state_transition == TRUE)
  {
    tbox_state_transition = FALSE;
    tbox_started_delay = FALSE;
    tbox_iap_state = TBOX_IAP_STATE_INIT;
    PcDebug_SendString("T-Box:Enter IAP!\n");
    tbox_mode = TBOX_MODE_IAP;
  }

  switch (tbox_iap_state)
  {
  case TBOX_IAP_STATE_INIT:
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      //=关闭硬件====================================
      IWDG_Feed();
      BAT_CHARGE_OFF();
      CAN_POWER_OFF();
      //GPS_SetModuleState(GPS_MODULE_STATE_SLEPT); // 关闭GPS模块
      Modem_SetState(MODEM_STATE_POWER_OFF); // 关闭4G模块
      tbox_10msec_timer = 2000;  // 等待20秒,保证外设关闭
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        tbox_started_delay = FALSE;
        tbox_iap_state = TBOX_IAP_STATE_VERIFY;
      }
    }
    break;
  
  case TBOX_IAP_STATE_VERIFY:
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      
      rfu_status = rfu_CheckNewFirmware(&rfu_context, rfu_data_buffer, RFU_BUFFER_SIZE);
      if (rfu_status == RFU_OK)
      {
        PcDebug_SendString("IAP:CrcOk!\n");
      }
      else
      {
        PcDebug_SendString("IAP:CrcErr!\n");
      }

      tbox_10msec_timer = 50;  // 等待0.5秒
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        tbox_started_delay = FALSE;
        if (rfu_status == RFU_OK)
        {
          tbox_iap_state = TBOX_IAP_STATE_IAP;
        }
        else
        {
          tbox_iap_state = TBOX_IAP_STATE_EXIT;
        }
      }
    }
    break;

  case TBOX_IAP_STATE_IAP:
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      PcDebug_SendString("IAP:Doing!\n");
      tbox_10msec_timer = 5;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        //taskENTER_CRITICAL(); // 进入临界区
        iap_main();
        //taskEXIT_CRITICAL();  // 退出临界区
      }
    }
    break;

  case TBOX_IAP_STATE_EXIT:
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      tbox_10msec_timer = 50;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        tbox_state = TBOX_STATE_POWERON; // 返回主界面
        tbox_state_transition = TRUE;
      }
    }
    break;

  case TBOX_IAP_STATE_REBOOT:
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      tbox_10msec_timer = 10;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        __set_FAULTMASK(1);     // 关闭所有中断
        NVIC_SystemReset();			// 复位
      }
    }
    break;
  }
}

/*****************************************************************************
* 处理GPS终端上电初始化、正常工作、休眠、固件升级
******************************************************************************/
void tbox_state_machine(void)
{
  static uint16_t divide_for_1s = 6000;

  switch (tbox_state)
  {
  case TBOX_STATE_POWERON: // 上电初始化
    tbox_power_on();
    break;

  case TBOX_STATE_WORKING: // 正常工作
    tbox_working();
    break;

  case TBOX_STATE_SLEEP: // 休眠
    tbox_sleep();
    break;

  case TBOX_STATE_IAP: // 固件升级
    tbox_iap();
    break;

  default:
    break;
  }

  if (tbox_10msec_timer) // 普通定时器计数
    tbox_10msec_timer--;

  if (enter_sleep_timer) // 休眠定时器计数
    enter_sleep_timer--;

  if(divide_for_1s) // 1S计时器(10ms)
    divide_for_1s--;
  else
  {
    divide_for_1s = 100;
    Tbox_CheckPowerOffMachine();
  }
}

/******************************************************************************
*
******************************************************************************/
void tbox_reset_parameter(void)
{

}

/******************************************************************************
*
******************************************************************************/
void tbox_initialize(void)
{
  tbox_reset_parameter();
  enter_sleep_timer_sp = (uint32_t)m2m_asset_data.wakeup_work_time_sp*100;
  tbox_state = TBOX_STATE_POWERON;
  tbox_state_transition = TRUE;
}

/******************************************************************************
* 设置系统工作状态
******************************************************************************/
void Tbox_SetMachineState(tbox_state_t state)
{
  tbox_state_transition = TRUE;
  tbox_state = state;
}


