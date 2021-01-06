/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Collect.C
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-4
* @brief
******************************************************************************/
//-----头文件调用-------------------------------------------------------------
#include "CollectHW.h"
#include "Collect.h"
#include "config.h"

/******************************************************************************
 * RTOS
 ******************************************************************************/
#define AppThreadPriority_Collect   osPriorityNormal
osThreadId_t tid_Collect;

const osThreadAttr_t AppThreadAttr_Collect =
{
  .priority = AppThreadPriority_Collect,
  .attr_bits = osThreadDetached,
  .stack_size = 2048, // 字节
};

/******************************************************************************
* Data Types and Globals
******************************************************************************/
volatile uint8_t divide_for_100ms;
volatile uint8_t divide_for_5second;
volatile uint8_t divide_for_1second;
volatile uint8_t divide_for_1minute;
volatile uint8_t divide_for_1hour;
volatile bittype timer_flag0;
colt_info_t colt_info;

/******************************************************************************
 * User Macros
 ******************************************************************************/
#define DIVIDER_FOR_100MS   9  /* The number of 10ms timer interrupts per 100 ms */
#define DIVIDER_FOR_5SECOND 4  /* The number of 1s timer interrupts per 5 second */
#define DIVIDER_FOR_1SECOND 9  /* The number of 100ms timer interrupts per 1 second */
#define DIVIDER_FOR_1MIN    59 /* The number of 1s timer interrupts per 1 minute */
#define DIVIDER_FOR_1HOUR   59 /* The number of 1min timer interrupts per 1 hour */

#define DO_10MS_FLAG        timer_flag0.b.bit0
#define DO_50MS_FLAG        timer_flag0.b.bit1
#define DO_100MS_FLAG       timer_flag0.b.bit2
#define DO_1SECOND_FLAG     timer_flag0.b.bit3
#define DO_5SECOND_FLAG     timer_flag0.b.bit4
#define DO_1MINUTE_FLAG     timer_flag0.b.bit5
#define DO_1HOUR_FLAG       timer_flag0.b.bit6
//#define x   timer_flag0.b.bit7

#define COLT_DELAY(ms)  do { osDelay(ms); } while(0)

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
extern void Do10msTasks(void);
extern void Do100msTasks(void);
extern void Do1sTasks(void);
extern void Do5sTasks(void);
extern void Do1minuteTasks(void);
extern void Do1hourTasks(void);
uint8_t COLT_GetAccStatus(void);

#define LIPO_BATTERY_LOW_VOLTAGE_SP    350 // 3.5V
#define BATTERY_LOW_DEBOUNCE_TIME_SP   5   // 5秒
#define BATTERY_HIGH_DEBOUNCE_TIME_SP  5   // 5秒
/*************************************************************************
 * 1秒调用一次
************************************************************************/
void COLT_CheckBatteryStatus(void)
{
  uint32_t average_value;
  static uint8_t battery_low_debounce_cnt = 0;
  static uint8_t battery_high_debounce_cnt = 0;

  average_value = buf_vbat.sum / buf_vbat.av_nb_sample;
  colt_info.vbat = (uint16_t) (average_value * 33 / 128);  // 单位是0.01V

  if (colt_info.vbat < LIPO_BATTERY_LOW_VOLTAGE_SP)
  {
    battery_high_debounce_cnt = 0;
    if (battery_low_debounce_cnt > BATTERY_LOW_DEBOUNCE_TIME_SP)
    {
      if((colt_info.switch3 & BIT(5)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_BAT_LOW;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 |= BIT(5); // 内部锂电池馈电
    }
    else
      battery_low_debounce_cnt++;
  }
  else
  {
    battery_low_debounce_cnt = 0;
    if (battery_high_debounce_cnt > BATTERY_HIGH_DEBOUNCE_TIME_SP)
    {
      if(colt_info.switch3 & BIT(5))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_BAT_NORMAL;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 &= ~BIT(5); // 内部锂电不馈电
    }
    else
      battery_high_debounce_cnt++;
  }
}

#define MAIN_POWER_LOW_VOLTAGE_HSP        2200 // 22V
#define MAIN_POWER_LOW_VOLTAGE_LSP        1700 // 17V
#define MAIN_POWER_LOW_DEBOUNCE_TIME_SP   10   // 5秒
#define MAIN_POWER_HIGH_DEBOUNCE_TIME_SP  5    // 5秒
#define MAIN_POWER_OFF_DEBOUNCE_TIME_SP   5    // 5秒
#define MAIN_POWER_ON_DEBOUNCE_TIME_SP    5    // 5秒
/*************************************************************************
 * 1秒调用一次
*************************************************************************/
void COLT_CheckMainPowerStatus(void)
{
  uint32_t average_value;
  static uint8_t main_power_low_debounce_cnt = 0;
  static uint8_t main_power_high_debounce_cnt = 0;

  average_value = buf_vraw.sum / buf_vraw.av_nb_sample;
  colt_info.vraw = (uint16_t) (average_value * 121 / 125)+67;  // 单位是0.01V
  if (colt_info.vraw < 500)
  {
    colt_info.vraw = 0; // 当采集到外电小于5V时,认为外电切断
  }

  if ((colt_info.vraw < MAIN_POWER_LOW_VOLTAGE_HSP) && (colt_info.vraw > MAIN_POWER_LOW_VOLTAGE_LSP)) // 大于17V小于22V
  {
    main_power_high_debounce_cnt = 0;
    if (main_power_low_debounce_cnt > MAIN_POWER_LOW_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch3 & BIT(1)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_LOW; // 电源电压低
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 |= BIT(1); // 外部电压低
    }
    else
      main_power_low_debounce_cnt++;
  }
  else
  {
    main_power_low_debounce_cnt = 0;
    if(main_power_high_debounce_cnt > MAIN_POWER_HIGH_DEBOUNCE_TIME_SP)
    {
      if (colt_info.switch3 & BIT(1))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_NORMAL;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 &= ~BIT(1); // 外部电压正常
    }
    else
      main_power_high_debounce_cnt++;
  }

  static uint8_t vraw_on_debounce_cnt = 0;
  static uint8_t vraw_off_debounce_cnt = 0;
  //==外电供电检测=================================================================
  if (colt_info.vraw > 700)
  {
    vraw_off_debounce_cnt = 0;
    if (vraw_on_debounce_cnt > MAIN_POWER_ON_DEBOUNCE_TIME_SP)
    {
      if(colt_info.switch3 & BIT(0))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_ON;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 &= ~BIT(0);	// 外电供电
    }
    else
      vraw_on_debounce_cnt++;
  }
  else
  {
    vraw_on_debounce_cnt = 0;
    if (vraw_off_debounce_cnt > MAIN_POWER_OFF_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch3 & BIT(0)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_OFF;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 |= BIT(0); 	// 电池供电
    }
    else
      vraw_off_debounce_cnt++;
  }
}

/*****************************************************************************
 * 只有ACC或者发动机开关为高时,才能对电池充电
******************************************************************************/
void COLT_ControlBatteryCharge(void)
{
  static uint16_t usTimer=0;

  if (tbox_state==TBOX_STATE_WORKING)
  {
    if ((colt_info.switch3 & BIT(0)) == 0x00) // 外电供电
    {
      usTimer = 0;

      if (colt_info.vbat > 415)
      {
        BAT_CHARGE_OFF(); // 不充电
        colt_info.switch3 &= ~BIT(2);
      }
      else if (colt_info.vbat < 400)
      {
        BAT_CHARGE_ON(); // 充电
        colt_info.switch3 |= BIT(2);
      }
    }
    else // 电池供电
    {
      if ((usTimer++ > 60) && (colt_info.vbat < 340)) //电池保护
      {
        usTimer = 0;
        //g_stuSystem.ucShutDownTime = 10;
      }

      BAT_CHARGE_OFF(); // 不充电
      colt_info.switch3 &= ~BIT(2);
    }
  }
  else
  {
    BAT_CHARGE_OFF(); // 不充电
    colt_info.switch3 &= ~BIT(2);
  }
}

//==检测ACC由开到关事件和由关到开事件,100ms调用一次==========================
void COLT_CheckAccOffEvent(void)
{
  static uint8_t current_state = 0;
  static uint8_t previous_state = 1;
  static uint8_t debounced_state = 0;
  static uint16_t debounce_tmr = 1200; // 2分钟

  // 读取ACC状态
  if(COLT_GetAccStatus() == 1)  // 开
    current_state = 0;
  else // 关
    current_state = 1;
  
  if (current_state != previous_state) // Checking for stability
  {
    debounce_tmr = 1200;  // 去耦时间
    previous_state = current_state;
    colt_info.acc_off_event_flag = 0;
  }
  else
  {
    if (debounce_tmr)
      debounce_tmr--;  // 去抖动
    else
    {
      if (current_state && !debounced_state) // 保存终端超时时间
      {
        colt_info.acc_off_event_flag = 1;
        PcDebug_SendString("ACC:Off!\n");
      }
      debounced_state = current_state;
    }
  }
}

#define ACC_OFF_DEBOUNCE_TIME_SP    20  // 2秒
#define ACC_ON_DEBOUNCE_TIME_SP     20  // 2秒
/********************************************************************************
 * 100ms调用一次
*********************************************************************************/
void COLT_CheckAccStatus(void)
{
  static uint8_t acc_on_debounce_cnt = 0;
  static uint8_t acc_off_debounce_cnt = 0;
  
  //==ACC开关信号================================================================
  if (ACC_IN()) // ACC关闭
  {
    acc_on_debounce_cnt = 0;
    if (acc_off_debounce_cnt > ACC_OFF_DEBOUNCE_TIME_SP)
    {
      if (colt_info.switch1 & BIT(1))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_ACC_OFF;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch1 &= ~BIT(1);  // 关闭=0
    }
    else
      acc_off_debounce_cnt++;
  }
  else
  {
    acc_off_debounce_cnt = 0;
    if (acc_on_debounce_cnt > ACC_ON_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch1 & BIT(1)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_ACC_ON;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch1 |= BIT(1);
    }
    else
      acc_on_debounce_cnt++;
  }
}

#define BOX_OPENED_DEBOUNCE_TIME_SP    30  // 3秒
#define BOX_CLOSED_DEBOUNCE_TIME_SP    30  // 3秒
/********************************************************************************
 * 100ms调用一次
*********************************************************************************/
void COLT_CheckBoxStatus(void)
{
  static uint8_t box_opened_debounce_cnt = 0;
  static uint8_t box_closed_debounce_cnt = 0;
  
  //==开盖输入信号================================================================
  if (BOX_IN()) // 未开盖(高电平)
  {
    box_opened_debounce_cnt = 0;
    if (box_closed_debounce_cnt > BOX_CLOSED_DEBOUNCE_TIME_SP)
    {
      if (colt_info.switch2 & BIT(3))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_BOX_CLOSED;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch2 &= ~BIT(3);  // 未开盖=0
    }
    else
      box_closed_debounce_cnt++;
  }
  else // 开盖(低电平)
  {
    box_closed_debounce_cnt = 0;
    if (box_opened_debounce_cnt > ACC_ON_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch2 & BIT(3)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_BOX_OPENED;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch2 |= BIT(3); // 开盖报警=1
    }
    else
      box_opened_debounce_cnt++;
  }
}

/*************************************************************************
 * This should be called by the main app every 1s.
*************************************************************************/
void COLT_GetMicroTemperature(void)
{
  uint16_t average_value;

  average_value = buf_inttemp.sum / buf_inttemp.av_nb_sample;
  // temp = (3.3/4096 * SampleValue-0.76)/0.0025 + 25;
  colt_info.int_temp = (average_value*155/512) - 279;
}

/********************************************************************************
 * 1s调用一次
*********************************************************************************/
void COLT_ManageLedStatus(void)
{
  if(tbox_state==TBOX_STATE_POWERON)
  {
    led_control[LED_GSM].blink = STEADY_ON;
    led_control[LED_GPS].blink = STEADY_OFF;
    led_control[LED_CAN].blink = STEADY_OFF;
  }
  else if(tbox_state==TBOX_STATE_WORKING)
  {
    //==红灯(电源、运行、GSM上线)=================================
    if(M2M_GetConnStatus() == M2M_TRUE)
    {
      led_control[LED_GSM].blink = FAST_CONTINUOUS;
    }
    else
    {
      led_control[LED_GSM].blink = SLOW_CONTINUOUS;
    }

    //==绿灯(GPS)=================================================
    if (GPS_GetPositioningStatus()==NMEA_TRUE) // 模块已定位
    {
      led_control[LED_GPS].blink = SLOW_CONTINUOUS;
    }
    else // 未定位
    {
      if(GPS_GetAntennaStatus()==GPS_NOK) // 天线故障
      {
        led_control[LED_GPS].blink = STEADY_ON;
      }
      else
      {
        led_control[LED_GPS].blink = STEADY_OFF;
      }
    }

    //==黄灯(CAN)=================================================
    if(CAN_GetRecvState(CAN_CHANNEL1)==CAN_OK && CAN_GetRecvState(CAN_CHANNEL2)==CAN_OK)
    {
      led_control[LED_CAN].blink = FAST_CONTINUOUS;
    }
    else if(CAN_GetRecvState(CAN_CHANNEL1)==CAN_NOK && CAN_GetRecvState(CAN_CHANNEL2)==CAN_NOK)
    {
      led_control[LED_CAN].blink = STEADY_OFF;
    }
    else
    {
      led_control[LED_CAN].blink = SLOW_CONTINUOUS;
    }
  }
  else if(tbox_state==TBOX_STATE_SLEEP)
  {
    led_control[LED_GSM].blink = STEADY_OFF;
    led_control[LED_GPS].blink = STEADY_OFF;
    led_control[LED_CAN].blink = STEADY_OFF;
  }
}

/**********************************************************************************
 *
*********************************************************************************/
//==获取开盖状态==============================================================
uint8_t COLT_GetBoxOpenStatus(void)
{
  return colt_info.open_box_flag;
}

//==获取电瓶电压(0.1V)==============================================================
uint16_t COLT_GetMainPowerVoltage(void)
{
  return (colt_info.vraw/10);
}

//==获取锂电池电压(0.1V)============================================================
uint16_t COLT_GetBatVoltage(void)
{
  return (colt_info.vbat/10);
}

//==获取锂电池低电压状态======================================================
uint8_t COLT_GetBatLowStatus(void)
{
  return (colt_info.switch3 & BIT(5))? 1:0; // 1=电压低,0=电压正常
}

//==获取电池充电状态==========================================================
uint8_t COLT_GetBatChargeStatus(void)
{
  return (colt_info.switch3 & BIT(2))? 1:0; // 1=充电,0=未充电
}

//==获取钥匙开关状态==========================================================
uint8_t COLT_GetAccStatus(void)
{
  return (colt_info.switch1 & BIT(1)) ? 1 : 0;  // 0=关闭,1=打开
}

//==获取主电源状态============================================================
uint8_t COLT_GetMainPowerStatus(void)
{
  return (colt_info.switch3 & BIT(0)) ? 1 : 0; // 0=外电供电；1=电池供电
}

//==获取主电源馈电状态========================================================
uint8_t COLT_GetMainPowerLowStatus(void)
{
  return (colt_info.switch3 & BIT(1)) ? 1 : 0; // 0=未馈电, 1=馈电
}

//=============================================================================
uint8_t COLT_GetSwitch1Status(void)
{
  return colt_info.switch1;
}

//=============================================================================
uint8_t COLT_GetSwitch2Status(void)
{
  return colt_info.switch2;
}

//=============================================================================
uint8_t COLT_GetSwitch3Status(void)
{
  return colt_info.switch3;
}

//==获取拖车状态===============================================================
uint8_t COLT_GetVehicleTowingStatus(void)
{
  return 0;
}

//==读取内部看门狗状态=========================================================
void COLT_ReadInternalWdtState(void)
{
  uint8_t tempVal;

  tempVal = FLASH_OB_GetUser();
  // Bit0: 0=软件控制使能片内看门狗IWDG;1=硬件使能片内看门狗
  if (0 != (0x01 & tempVal)) 
    colt_info.wdt_status = 0;
  else
    colt_info.wdt_status = 0xAA;
}

//==获取总工作时间=============================================================
uint32_t COLT_GetTotalWorkTime(void)
{
  return colt_info.total_work_time;
}

//设置总工作时间
void COLT_SetTotalWorkTime(uint32_t work_time)
{
  colt_info.total_work_time = work_time;
  BKP_SaveTotalWorkTimeInfo();
  Parm_SaveTotalWorkTimeInfo();
}

/**********************************************************************************
 * 从Nor Flash RTC RAM中读取总工作小时
*********************************************************************************/
void COLT_InitTotalWorkTime(void)
{
  uint32_t tempVal;

  tempVal = RTC_ReadBackupRegister(RTC_BKP_DR0); // 从内存读取校验头
  if (0x55AA5AA5 != tempVal) // 断电
  {
    Parm_ReadTotalWorkTimeInfo();
  }
  else
  {
    tempVal = RTC_ReadBackupRegister(RTC_BKP_DR1); // 从内存读取累计工作时间
    colt_info.total_work_time = tempVal;
  }
}

/**********************************************************************************
 * 统计工作小时:当保留开关为高电平时候累计,当由高电平
 * 变为低电平时候进行存储，正常工作时候6分钟存储一次
*********************************************************************************/
void COLT_CumulateTotalWorkTime(void)
{
  static uint8 ucCount60s = 60;        // 一分钟计时
  uint8 current_acc_state;
  static uint8_t previous_acc_state  = 0;
  static uint8 ucAccOffCount = 0;

  current_acc_state = COLT_GetAccStatus();
  if (current_acc_state==1)
  {
    colt_info.total_work_time++;
    if (!(--ucCount60s))
    {
      ucCount60s = 60;
      BKP_SaveTotalWorkTimeInfo();
    }
  }
  else
  {
    if (previous_acc_state == 1)//ACC ON----->ACC OFF
    {
      BKP_SaveTotalWorkTimeInfo();
      ucAccOffCount = 5;
    }
  }
  
  if (ucAccOffCount)
  {
    if (!(--ucAccOffCount))
    {
      Parm_ReadTotalWorkTimeInfo();
    }
  }
  previous_acc_state = current_acc_state;
}

/******************************************************************************
 * 设置系统复位延时时间
*******************************************************************************/
void CTL_SetRestartDelayTime(uint8_t time)
{
  colt_info.restart_timer = time;
}

/******************************************************************************
 * 设置系统关机延时时间
*******************************************************************************/
void CTL_SetPwroffDelayTime(uint8_t time)
{
  colt_info.powerr_off_timer = time;
}

/******************************************************************************
 * 周期性调用函数,1S调用一次,无阻塞
*******************************************************************************/
void CTL_Do1sTasks(void)
{
  //==系统复位===========================
  if(colt_info.restart_timer)
  {
    colt_info.restart_timer--;
    if(colt_info.restart_timer==0x00)
    {
      PcDebug_SendString("CTL:Reset!\n");
      COLT_DELAY(10); // 延时10MS,保证调试信息完整输出
      NVIC_SystemReset(); // 复位
    }
  }

  //==关机===============================
  if(colt_info.powerr_off_timer)  // 关机
  {
    colt_info.powerr_off_timer--;
    if(colt_info.powerr_off_timer==0x00)
    {
      MAIN_POWER_OFF();
      BAT_POWER_OFF();
      rtc_DisableAlarm();
      COLT_DELAY(100); // 延时100MS
      PcDebug_SendString("CTL:PowerOff!\n");
    }
  }
}

/******************************************************************************
* 1秒调用一次
******************************************************************************/
uint8_t COLT_GetRtcStatus(void)
{
  return colt_info.correct_rtc_flag;
}

void COLT_SetRtcStatus(uint8_t status)
{
  colt_info.correct_rtc_flag = 1;
}

void COLT_CorrectRtcStatus(void)
{
  //uint8_t RTC_CovertUtcToBjt(rtc_date_t *utc, rtc_date_t *bj);
  rtc_date_t rtc_bj;
  uint8_t tempVal;

  if(GPS_GetPositioningStatus()==1) // GPS已定位
  {
    if(COLT_GetRtcStatus()==0) // 未校时
    {
      tempVal = RTC_CovertUtcToBjt(&nav_data.utc,&rtc_bj);
      if(tempVal==TRUE)
      {
        COLT_SetRtcStatus(1);
        rtc_SetDateTime(&rtc_bj); // 校时
        PcDebug_SendString("CorrectRtc!\n");
      }
    }
  }
}

/******************************************************************************
*
******************************************************************************/
void COLT_Do100msTasks(void)
{
  adc_push_dma_sample();
  COLT_CheckBoxStatus();
  COLT_CheckAccStatus();
  COLT_CheckAccOffEvent();
}


/******************************************************************************
*
******************************************************************************/
void COLT_Do1sTasks(void)
{
  COLT_ManageLedStatus();
  COLT_CheckBatteryStatus();
  COLT_CheckMainPowerStatus();
  COLT_ControlBatteryCharge();
  COLT_CorrectRtcStatus();
}

/******************************************************************************
*
******************************************************************************/
void TIM_InitializeAllTimer(void)
{
  divide_for_100ms = DIVIDER_FOR_100MS;
  divide_for_1second = DIVIDER_FOR_1SECOND;
  divide_for_5second = DIVIDER_FOR_5SECOND;
  divide_for_1minute =DIVIDER_FOR_1MIN;
  divide_for_1hour = DIVIDER_FOR_1HOUR;

  timer_flag0.byte = 0x00;
}

/******************************************************************************
*
******************************************************************************/
void do_10ms(void)
{
  /**************** Do10msTasks();****************/
  DO_10MS_FLAG = 1;

  if (!divide_for_100ms)	// Schedule 100msec
  {
    DO_100MS_FLAG = 1;
    divide_for_100ms = DIVIDER_FOR_100MS;

    if (!divide_for_1second) 	// Schedule 1 sec
    {
      DO_1SECOND_FLAG = 1;
      divide_for_1second = DIVIDER_FOR_1SECOND;

      if (!divide_for_5second) 	// Schedule 5 sec
      {
        DO_5SECOND_FLAG = 1;
        divide_for_5second = DIVIDER_FOR_5SECOND;
      }
      else divide_for_5second--;

      if (!divide_for_1minute) // Schedule 1 minute
      {
        DO_1MINUTE_FLAG = 1;
        divide_for_1minute = DIVIDER_FOR_1MIN;

        if (!divide_for_1hour)
        {
          DO_1HOUR_FLAG = 1;
          divide_for_1hour = DIVIDER_FOR_1HOUR;
        }
        else divide_for_1hour--;
      }
      else divide_for_1minute--;
    }
    else divide_for_1second--;
  }
  else divide_for_100ms--;
}

/*******************************************************************************
 *	check_timer_tasks() Checks timer flags and executes routines
 *******************************************************************************/
void check_timer_tasks(void)
{
  //==========================================================
  if (DO_10MS_FLAG)
  {
    DO_10MS_FLAG = 0;
    Do10msTasks();
  }  // Do10msTasks();

  //==========================================================
  if (DO_100MS_FLAG)
  {
    DO_100MS_FLAG = 0;
    Do100msTasks();
  }  // Do100msTasks();

  //==========================================================
  if (DO_1SECOND_FLAG)
  {
    DO_1SECOND_FLAG = 0;
    Do1sTasks();
  }  // Do1sTasks();

  //==========================================================
  if (DO_5SECOND_FLAG)
  {
    DO_5SECOND_FLAG = 0;
    Do5sTasks();
  }  // Do5sTasks();

  //==========================================================
  if (DO_1MINUTE_FLAG)
  {
    DO_1MINUTE_FLAG = 0;
    Do1minuteTasks();
  }  // Do1minuteTasks();

  //==========================================================
  if (DO_1HOUR_FLAG)
  {
    DO_1HOUR_FLAG = 0;
    Do1hourTasks();
  }  // Do1hourTasks();
}

/******************************************************************************
* 基于时基无阻塞采集任务(采集外部开关量状态、电压和电池电压等)
*******************************************************************************/
void* pthread_Collect(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms
    do_10ms();
    check_timer_tasks();
  }
}

/******************************************************************************
*
******************************************************************************/
void Do10msTasks(void)
{
  led_state_machine();
  discrete_inputs_check();
  discrete_inputs_command();
}

/* ========================================================== */
void Do100msTasks(void)
{
  IWDG_Feed();
  COLT_Do100msTasks();
  Can_Do100msTasks();
}

/* ========================================================== */
void Do1sTasks(void)
{
  COLT_Do1sTasks();
  GPS_Do1sTasks();
  M2M_Do1sTasks();
  CTL_Do1sTasks();
  Can_Do1sTasks();
}

extern uint8_t rtc_GetRegister(uint8_t reg_addr);
/* ========================================================== */
void Do5sTasks(void)
{
  //uint8_t rtc_alarm_status;
  //rtc_date_t time;
  //time = RTC_GetBjTime();
  //PcDebug_Printf("Tst:%d-%d-%d %d:%d:%d\n\r", time.year,time.month,time.day,time.hour,time.minute,time.second);
  
  //if(RTC_ALARM_IN()==0) // RTC闹铃
  //{
  //  PcDebug_SendString("RtcAlarm!\n");
  //}
  
  //rtc_alarm_status = rtc_GetRegister(RTC_CONTROL_STATUS2_ADDR); // 判断闹钟中断
  //rtc_alarm_status &= 0x1F;
  //PcDebug_Printf("RtcAlarm=%x\n",rtc_alarm_status);
}

/* ========================================================== */
void Do1minuteTasks(void)
{
  PcDebug_Printf("Vraw=%d, Vbat=%d, Tmcu=%d\n",colt_info.vraw,colt_info.vbat,colt_info.int_temp);
}

/* ========================================================== */
void Do1hourTasks(void)
{
  COLT_SetRtcStatus(0); // 一小时校准一次时间
}

/*************************************************************************
 *
*************************************************************************/
void Collect_ServiceInit(void)
{
  CollectHw_Init();
  DIN_InitializeAllInputs();
  TIM_InitializeAllTimer();
  colt_info.acc_off_event_flag = 0;
  colt_info.correct_rtc_flag = 0;
  colt_info.m2m_online_flag = 0;
  colt_info.hjep_online_flag = 0;
  colt_info.gbep_online_flag = 0;
  colt_info.switch1 = 0;    // 用于报警开关、ACC开关、小时计开关、搭铁等状态标识
  colt_info.switch2 = 0;    // 用于继电器锁车、开盒报警、CAN、RS23通信状态标识
  colt_info.switch3 = 0;    // 用于供电方式、外电和内部电池状态、行驶速度、位置越界等状态标识
}

/*************************************************************************
 *
*************************************************************************/
void Collect_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_COLLECT_ID], NULL, pthread_Collect, NULL);
  usleep(10);
}

//-----文件Collect.c结束---------------------------------------------
