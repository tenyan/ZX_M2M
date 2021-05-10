/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Collect.C
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-4
* @brief
******************************************************************************/
//-----头文件调用-------------------------------------------------------------
#include "Collect.h"
#include "config.h"

/******************************************************************************
* Data Types and Globals
******************************************************************************/
volatile uint8_t divide_for_100ms;
volatile uint8_t divide_for_5second;
volatile uint8_t divide_for_1second;
volatile uint8_t divide_for_1minute;
volatile uint8_t divide_for_1hour;
volatile bittype timer_flag0;

colt_info_t colt_info = {
  .total_work_time = 0x00,  // 累计工作时间,单位:s
  .total_offline_time = 0x00, // 累计不上线时间
  .restart_timer = 0x00, // 终端重启
};

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

#define COLT_DELAY(ms)  do { msleep(ms); } while(0)

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
  //uint32_t average_value;
  static uint8_t battery_low_debounce_cnt = 0;
  static uint8_t battery_high_debounce_cnt = 0;

  //average_value = buf_vbat.sum / buf_vbat.av_nb_sample;
  //colt_info.vbat = (uint16_t) (average_value * 33 / 128);  // 单位是0.01V
  if (colt_info.vbat < LIPO_BATTERY_LOW_VOLTAGE_SP)
  {
    battery_high_debounce_cnt = 0;
    if (battery_low_debounce_cnt > BATTERY_LOW_DEBOUNCE_TIME_SP)
    {
      if((colt_info.switch3 & BIT(5)) == 0x00)
      {
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_BAT_LOW;
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_BAT_NORMAL;
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
  //uint32_t average_value;
  static uint8_t main_power_low_debounce_cnt = 0;
  static uint8_t main_power_high_debounce_cnt = 0;

  //average_value = buf_vraw.sum / buf_vraw.av_nb_sample;
  //colt_info.vraw = (uint16_t) (average_value * 121 / 125)+67;  // 单位是0.01V
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_LOW; // 电源电压低
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_NORMAL;
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_ON;
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_OFF;
        //SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 |= BIT(0); 	// 电池供电
    }
    else
      vraw_off_debounce_cnt++;
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
#if 0
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_ACC_OFF;
        //SYSBUS_PutMbox(SbusMsg_Collect);
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
        //SbusMsg_Collect.type = COLLECT_MSG_TYPE_ACC_ON;
        //SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch1 |= BIT(1);
    }
    else
      acc_on_debounce_cnt++;
  }
#endif
}

/********************************************************************************
 * 1s调用一次
*********************************************************************************/
void COLT_ManageLedStatus(void)
{
  //==进程运行状态指示灯====================================
  led_control[LED_USER].blink = SLOW_CONTINUOUS;

  //==网络状态指示灯========================================
  if(M2M_GetConnStatus() == M2M_TRUE)
  {
    led_control[LED_NET].blink = SLOW_CONTINUOUS;
  }
  else if(Modem_GetState()==MODEM_STATE_INIT)
  {
    led_control[LED_NET].blink = STEADY_ON;
  }
  else
  {
    led_control[LED_NET].blink = STEADY_OFF;
  }
}

/**********************************************************************************
* 获取GPS终端与MCU的通信状态
***********************************************************************************/
uint8_t CAN_GetCommState(uint8_t channel)
{
  if (CAN_CHANNEL1==channel)
  {
    return ( (colt_info.can_status & BIT(0))? 1: 0 ); // 1=收到, 0=未收到
  }
  else if (CAN_CHANNEL2==channel)
  {
    return ( (colt_info.can_status & BIT(1))? 1: 0 ); // 1=收到, 0=未收到
  }
  return CAN_NOK;
}

/**********************************************************************************
* 获取CAN接收状态,1:收到了CAN1数据，2:没有收到CAN1数据
***********************************************************************************/
uint8_t CAN_GetRecvState(uint8_t channel)
{
  if (CAN_CHANNEL1==channel)
  {
    return ( (colt_info.can_status & BIT(2))? 1: 0 ); // 1=正常, 0=异常
  }
  else if (CAN_CHANNEL2==channel)
  {
    return ( (colt_info.can_status & BIT(3))? 1: 0 ); // 1=正常, 0=异常
  }

  return CAN_NOK;
}

//==获取VIN码有效性=============================================================
uint8_t CAN_GetVinState(void)
{
  if ((zxtcw_context.vin_valid_flag == 0x01) || (obd_info.vin_valid_flag==0x01))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//==获取环保数据有效性=============================================================
uint8_t CAN_GetObdVinState(void)
{
  if (obd_info.vin_valid_flag==0x01)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//==获取环保数据有效性=============================================================
uint8_t CAN_GetUserVinState(void)
{
  if (zxtcw_context.vin_valid_flag == 0x01)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

//==获取环保数据有效性=============================================================
uint8_t CAN_GetEpDataState(void)
{
  return zxtcw_context.ep_valid_flag;
}

//==获取环保类型=================================================================
uint8_t CAN_GetEpType(void)
{
  return zxtcw_context.ep_type;
}

//==获取发动机类型=================================================================
uint8_t CAN_GetEngineType(void)
{
  return zxtcw_context.ecu_type;
}

//==获取发动机转速=================================================================
uint16_t CAN_GetEngineSpeed(void)
{
  return zxtcw_context.engine_speed;
}

//==获取MIL灯状态=================================================================
uint8_t CAN_GetMilLampState(void)
{
  return zxtcw_context.mil_lamp;
}

//==获取ECU绑定状态=================================================================
uint8_t CAN_GetEcuBindState(void)
{
  return zxtcw_context.lvc_binded_flag;
}

/**********************************************************************************
 *
*********************************************************************************/
//==获取ST工作状态状态==============================================================
uint8_t COLT_GetTboxState(void)
{
  return zxtcw_context.tbox_state;
}

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

//==获取协处理器版本===========================================================
uint16_t COLT_GetStVersion(void)
{
  return colt_info.st_version;
}

//==获取总工作时间=============================================================
uint32_t COLT_GetTotalWorkTime(void)
{
  return colt_info.total_work_time;
}

//设置总工作时间
void COLT_SetTotalWorkTime(uint32_t work_time)
{
#if 0
  colt_info.total_work_time = work_time;
  //BKP_SaveTotalWorkTimeInfo();
  Parm_SaveTotalWorkTimeInfo();
#endif
}

/**********************************************************************************
 * 从Nor Flash RTC RAM中读取总工作小时
*********************************************************************************/
void COLT_InitTotalWorkTime(void)
{
#if 0
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
#endif
}

/**********************************************************************************
 * 统计工作小时:当保留开关为高电平时候累计,当由高电平
 * 变为低电平时候进行存储，正常工作时候6分钟存储一次
*********************************************************************************/
void COLT_CumulateTotalWorkTime(void)
{
#if 0
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
#endif
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
      Modem_SetState(MODEM_STATE_RESET); // 重新开机
      //NVIC_SystemReset(); // 复位
    }
  }

  //==关机===============================
  if(colt_info.powerr_off_timer)  // 关机
  {
    colt_info.powerr_off_timer--;
    if(colt_info.powerr_off_timer==0x00)
    {
      COLT_DELAY(100); // 延时100MS
      PcDebug_SendString("CTL:PowerOff!\n");
    }
  }
}

/******************************************************************************
*
******************************************************************************/
void COLT_Do100msTasks(void)
{
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
  //discrete_inputs_check();
  //discrete_inputs_command();
}

/* ========================================================== */
void Do100msTasks(void)
{
  //WDG_Feed();
  COLT_Do100msTasks();
  Momi_AnalyzeAdSwitch();
  //Can_Do100msTasks();
  Net_CheckIsModemError();
}

/* ========================================================== */
void Do1sTasks(void)
{
  COLT_Do1sTasks();
  GPS_Do1sTasks();
  M2M_Do1sTasks();
  CTL_Do1sTasks();
  //Can_Do1sTasks();
}

extern uint8_t rtc_GetRegister(uint8_t reg_addr);
/* ========================================================== */
void Do5sTasks(void)
{
  //rtc_date_t time;
  //time = RTC_GetBjTime();
  //PcDebug_Printf("Tst:%d-%d-%d %d:%d:%d\n\r", time.year,time.month,time.day,time.hour,time.minute,time.second);
}

/* ========================================================== */
void Do1minuteTasks(void)
{
  // PcDebug_Printf("Vraw=%d, Vbat=%d, Tmcu=%d\n",colt_info.vraw,colt_info.vbat,colt_info.int_temp);

#if 0  // 测试代码
  //if(FSRV_GetState()==FSRV_STATE_IDLE)
  //{
  //  FSRV_SetState(FSRV_STATE_START);  // 启动外部程序升级
  //}
#endif
}

/* ========================================================== */
void Do1hourTasks(void)
{
  
}

/*************************************************************************
 *
*************************************************************************/
void Collect_ServiceInit(void)
{
  //CollectHw_Init();
  //DIN_InitializeAllInputs();
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
  pthread_attr_t thread_attr;
  int ret ,stacksize = DEFAULT_THREAD_STACK_SIZE; // thread堆栈设置为40KB

  pthread_attr_init(&thread_attr);
  ret = pthread_attr_setstacksize(&thread_attr,stacksize);
  if(ret!=0)
  {
    printf("Set StackSize Error!\n");
  }

  pthread_create(&pthreads[PTHREAD_COLLECT_ID], &thread_attr, pthread_Collect, NULL);
  usleep(10);
}

//-----文件Collect.c结束---------------------------------------------
