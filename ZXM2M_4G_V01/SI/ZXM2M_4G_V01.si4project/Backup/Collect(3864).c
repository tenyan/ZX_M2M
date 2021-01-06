/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Collect.C
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-4
* @brief
******************************************************************************/
//-----ͷ�ļ�����-------------------------------------------------------------
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
  .stack_size = 2048, // �ֽ�
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
#define BATTERY_LOW_DEBOUNCE_TIME_SP   5   // 5��
#define BATTERY_HIGH_DEBOUNCE_TIME_SP  5   // 5��
/*************************************************************************
 * 1�����һ��
************************************************************************/
void COLT_CheckBatteryStatus(void)
{
  uint32_t average_value;
  static uint8_t battery_low_debounce_cnt = 0;
  static uint8_t battery_high_debounce_cnt = 0;

  average_value = buf_vbat.sum / buf_vbat.av_nb_sample;
  colt_info.vbat = (uint16_t) (average_value * 33 / 128);  // ��λ��0.01V

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
      colt_info.switch3 |= BIT(5); // �ڲ�﮵������
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
      colt_info.switch3 &= ~BIT(5); // �ڲ�﮵粻����
    }
    else
      battery_high_debounce_cnt++;
  }
}

#define MAIN_POWER_LOW_VOLTAGE_HSP        2200 // 22V
#define MAIN_POWER_LOW_VOLTAGE_LSP        1700 // 17V
#define MAIN_POWER_LOW_DEBOUNCE_TIME_SP   10   // 5��
#define MAIN_POWER_HIGH_DEBOUNCE_TIME_SP  5    // 5��
#define MAIN_POWER_OFF_DEBOUNCE_TIME_SP   5    // 5��
#define MAIN_POWER_ON_DEBOUNCE_TIME_SP    5    // 5��
/*************************************************************************
 * 1�����һ��
*************************************************************************/
void COLT_CheckMainPowerStatus(void)
{
  uint32_t average_value;
  static uint8_t main_power_low_debounce_cnt = 0;
  static uint8_t main_power_high_debounce_cnt = 0;

  average_value = buf_vraw.sum / buf_vraw.av_nb_sample;
  colt_info.vraw = (uint16_t) (average_value * 121 / 125)+67;  // ��λ��0.01V
  if (colt_info.vraw < 500)
  {
    colt_info.vraw = 0; // ���ɼ������С��5Vʱ,��Ϊ����ж�
  }

  if ((colt_info.vraw < MAIN_POWER_LOW_VOLTAGE_HSP) && (colt_info.vraw > MAIN_POWER_LOW_VOLTAGE_LSP)) // ����17VС��22V
  {
    main_power_high_debounce_cnt = 0;
    if (main_power_low_debounce_cnt > MAIN_POWER_LOW_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch3 & BIT(1)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_MAIN_POWER_LOW; // ��Դ��ѹ��
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch3 |= BIT(1); // �ⲿ��ѹ��
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
      colt_info.switch3 &= ~BIT(1); // �ⲿ��ѹ����
    }
    else
      main_power_high_debounce_cnt++;
  }

  static uint8_t vraw_on_debounce_cnt = 0;
  static uint8_t vraw_off_debounce_cnt = 0;
  //==��繩����=================================================================
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
      colt_info.switch3 &= ~BIT(0);	// ��繩��
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
      colt_info.switch3 |= BIT(0); 	// ��ع���
    }
    else
      vraw_off_debounce_cnt++;
  }
}

/*****************************************************************************
 * ֻ��ACC���߷���������Ϊ��ʱ,���ܶԵ�س��
******************************************************************************/
void COLT_ControlBatteryCharge(void)
{
  static uint16_t usTimer=0;

  if (tbox_state==TBOX_STATE_WORKING)
  {
    if ((colt_info.switch3 & BIT(0)) == 0x00) // ��繩��
    {
      usTimer = 0;

      if (colt_info.vbat > 415)
      {
        BAT_CHARGE_OFF(); // �����
        colt_info.switch3 &= ~BIT(2);
      }
      else if (colt_info.vbat < 400)
      {
        BAT_CHARGE_ON(); // ���
        colt_info.switch3 |= BIT(2);
      }
    }
    else // ��ع���
    {
      if ((usTimer++ > 60) && (colt_info.vbat < 340)) //��ر���
      {
        usTimer = 0;
        //g_stuSystem.ucShutDownTime = 10;
      }

      BAT_CHARGE_OFF(); // �����
      colt_info.switch3 &= ~BIT(2);
    }
  }
  else
  {
    BAT_CHARGE_OFF(); // �����
    colt_info.switch3 &= ~BIT(2);
  }
}

//==���ACC�ɿ������¼����ɹص����¼�,100ms����һ��==========================
void COLT_CheckAccOffEvent(void)
{
  static uint8_t current_state = 0;
  static uint8_t previous_state = 1;
  static uint8_t debounced_state = 0;
  static uint16_t debounce_tmr = 1200; // 2����

  // ��ȡACC״̬
  if(COLT_GetAccStatus() == 1)  // ��
    current_state = 0;
  else // ��
    current_state = 1;
  
  if (current_state != previous_state) // Checking for stability
  {
    debounce_tmr = 1200;  // ȥ��ʱ��
    previous_state = current_state;
    colt_info.acc_off_event_flag = 0;
  }
  else
  {
    if (debounce_tmr)
      debounce_tmr--;  // ȥ����
    else
    {
      if (current_state && !debounced_state) // �����ն˳�ʱʱ��
      {
        colt_info.acc_off_event_flag = 1;
        PcDebug_SendString("ACC:Off!\n");
      }
      debounced_state = current_state;
    }
  }
}

#define ACC_OFF_DEBOUNCE_TIME_SP    20  // 2��
#define ACC_ON_DEBOUNCE_TIME_SP     20  // 2��
/********************************************************************************
 * 100ms����һ��
*********************************************************************************/
void COLT_CheckAccStatus(void)
{
  static uint8_t acc_on_debounce_cnt = 0;
  static uint8_t acc_off_debounce_cnt = 0;
  
  //==ACC�����ź�================================================================
  if (ACC_IN()) // ACC�ر�
  {
    acc_on_debounce_cnt = 0;
    if (acc_off_debounce_cnt > ACC_OFF_DEBOUNCE_TIME_SP)
    {
      if (colt_info.switch1 & BIT(1))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_ACC_OFF;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch1 &= ~BIT(1);  // �ر�=0
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

#define BOX_OPENED_DEBOUNCE_TIME_SP    30  // 3��
#define BOX_CLOSED_DEBOUNCE_TIME_SP    30  // 3��
/********************************************************************************
 * 100ms����һ��
*********************************************************************************/
void COLT_CheckBoxStatus(void)
{
  static uint8_t box_opened_debounce_cnt = 0;
  static uint8_t box_closed_debounce_cnt = 0;
  
  //==���������ź�================================================================
  if (BOX_IN()) // δ����(�ߵ�ƽ)
  {
    box_opened_debounce_cnt = 0;
    if (box_closed_debounce_cnt > BOX_CLOSED_DEBOUNCE_TIME_SP)
    {
      if (colt_info.switch2 & BIT(3))
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_BOX_CLOSED;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch2 &= ~BIT(3);  // δ����=0
    }
    else
      box_closed_debounce_cnt++;
  }
  else // ����(�͵�ƽ)
  {
    box_closed_debounce_cnt = 0;
    if (box_opened_debounce_cnt > ACC_ON_DEBOUNCE_TIME_SP)
    {
      if ((colt_info.switch2 & BIT(3)) == 0x00)
      {
        SbusMsg_Collect.type = COLLECT_MSG_TYPE_BOX_OPENED;
        SYSBUS_PutMbox(SbusMsg_Collect);
      }
      colt_info.switch2 |= BIT(3); // ���Ǳ���=1
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
 * 1s����һ��
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
    //==���(��Դ�����С�GSM����)=================================
    if(M2M_GetConnStatus() == M2M_TRUE)
    {
      led_control[LED_GSM].blink = FAST_CONTINUOUS;
    }
    else
    {
      led_control[LED_GSM].blink = SLOW_CONTINUOUS;
    }

    //==�̵�(GPS)=================================================
    if (GPS_GetPositioningStatus()==NMEA_TRUE) // ģ���Ѷ�λ
    {
      led_control[LED_GPS].blink = SLOW_CONTINUOUS;
    }
    else // δ��λ
    {
      if(GPS_GetAntennaStatus()==GPS_NOK) // ���߹���
      {
        led_control[LED_GPS].blink = STEADY_ON;
      }
      else
      {
        led_control[LED_GPS].blink = STEADY_OFF;
      }
    }

    //==�Ƶ�(CAN)=================================================
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
//==��ȡ����״̬==============================================================
uint8_t COLT_GetBoxOpenStatus(void)
{
  return colt_info.open_box_flag;
}

//==��ȡ��ƿ��ѹ(0.1V)==============================================================
uint16_t COLT_GetMainPowerVoltage(void)
{
  return (colt_info.vraw/10);
}

//==��ȡ﮵�ص�ѹ(0.1V)============================================================
uint16_t COLT_GetBatVoltage(void)
{
  return (colt_info.vbat/10);
}

//==��ȡ﮵�ص͵�ѹ״̬======================================================
uint8_t COLT_GetBatLowStatus(void)
{
  return (colt_info.switch3 & BIT(5))? 1:0; // 1=��ѹ��,0=��ѹ����
}

//==��ȡ��س��״̬==========================================================
uint8_t COLT_GetBatChargeStatus(void)
{
  return (colt_info.switch3 & BIT(2))? 1:0; // 1=���,0=δ���
}

//==��ȡԿ�׿���״̬==========================================================
uint8_t COLT_GetAccStatus(void)
{
  return (colt_info.switch1 & BIT(1)) ? 1 : 0;  // 0=�ر�,1=��
}

//==��ȡ����Դ״̬============================================================
uint8_t COLT_GetMainPowerStatus(void)
{
  return (colt_info.switch3 & BIT(0)) ? 1 : 0; // 0=��繩�磻1=��ع���
}

//==��ȡ����Դ����״̬========================================================
uint8_t COLT_GetMainPowerLowStatus(void)
{
  return (colt_info.switch3 & BIT(1)) ? 1 : 0; // 0=δ����, 1=����
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

//==��ȡ�ϳ�״̬===============================================================
uint8_t COLT_GetVehicleTowingStatus(void)
{
  return 0;
}

//==��ȡ�ڲ����Ź�״̬=========================================================
void COLT_ReadInternalWdtState(void)
{
  uint8_t tempVal;

  tempVal = FLASH_OB_GetUser();
  // Bit0: 0=�������ʹ��Ƭ�ڿ��Ź�IWDG;1=Ӳ��ʹ��Ƭ�ڿ��Ź�
  if (0 != (0x01 & tempVal)) 
    colt_info.wdt_status = 0;
  else
    colt_info.wdt_status = 0xAA;
}

//==��ȡ�ܹ���ʱ��=============================================================
uint32_t COLT_GetTotalWorkTime(void)
{
  return colt_info.total_work_time;
}

//�����ܹ���ʱ��
void COLT_SetTotalWorkTime(uint32_t work_time)
{
  colt_info.total_work_time = work_time;
  BKP_SaveTotalWorkTimeInfo();
  Parm_SaveTotalWorkTimeInfo();
}

/**********************************************************************************
 * ��Nor Flash RTC RAM�ж�ȡ�ܹ���Сʱ
*********************************************************************************/
void COLT_InitTotalWorkTime(void)
{
  uint32_t tempVal;

  tempVal = RTC_ReadBackupRegister(RTC_BKP_DR0); // ���ڴ��ȡУ��ͷ
  if (0x55AA5AA5 != tempVal) // �ϵ�
  {
    Parm_ReadTotalWorkTimeInfo();
  }
  else
  {
    tempVal = RTC_ReadBackupRegister(RTC_BKP_DR1); // ���ڴ��ȡ�ۼƹ���ʱ��
    colt_info.total_work_time = tempVal;
  }
}

/**********************************************************************************
 * ͳ�ƹ���Сʱ:����������Ϊ�ߵ�ƽʱ���ۼ�,���ɸߵ�ƽ
 * ��Ϊ�͵�ƽʱ����д洢����������ʱ��6���Ӵ洢һ��
*********************************************************************************/
void COLT_CumulateTotalWorkTime(void)
{
  static uint8 ucCount60s = 60;        // һ���Ӽ�ʱ
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
 * ����ϵͳ��λ��ʱʱ��
*******************************************************************************/
void CTL_SetRestartDelayTime(uint8_t time)
{
  colt_info.restart_timer = time;
}

/******************************************************************************
 * ����ϵͳ�ػ���ʱʱ��
*******************************************************************************/
void CTL_SetPwroffDelayTime(uint8_t time)
{
  colt_info.powerr_off_timer = time;
}

/******************************************************************************
 * �����Ե��ú���,1S����һ��,������
*******************************************************************************/
void CTL_Do1sTasks(void)
{
  //==ϵͳ��λ===========================
  if(colt_info.restart_timer)
  {
    colt_info.restart_timer--;
    if(colt_info.restart_timer==0x00)
    {
      PcDebug_SendString("CTL:Reset!\n");
      COLT_DELAY(10); // ��ʱ10MS,��֤������Ϣ�������
      NVIC_SystemReset(); // ��λ
    }
  }

  //==�ػ�===============================
  if(colt_info.powerr_off_timer)  // �ػ�
  {
    colt_info.powerr_off_timer--;
    if(colt_info.powerr_off_timer==0x00)
    {
      MAIN_POWER_OFF();
      BAT_POWER_OFF();
      rtc_DisableAlarm();
      COLT_DELAY(100); // ��ʱ100MS
      PcDebug_SendString("CTL:PowerOff!\n");
    }
  }
}

/******************************************************************************
* 1�����һ��
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

  if(GPS_GetPositioningStatus()==1) // GPS�Ѷ�λ
  {
    if(COLT_GetRtcStatus()==0) // δУʱ
    {
      tempVal = RTC_CovertUtcToBjt(&nav_data.utc,&rtc_bj);
      if(tempVal==TRUE)
      {
        COLT_SetRtcStatus(1);
        rtc_SetDateTime(&rtc_bj); // Уʱ
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
* ����ʱ���������ɼ�����(�ɼ��ⲿ������״̬����ѹ�͵�ص�ѹ��)
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
  
  //if(RTC_ALARM_IN()==0) // RTC����
  //{
  //  PcDebug_SendString("RtcAlarm!\n");
  //}
  
  //rtc_alarm_status = rtc_GetRegister(RTC_CONTROL_STATUS2_ADDR); // �ж������ж�
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
  COLT_SetRtcStatus(0); // һСʱУ׼һ��ʱ��
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
  colt_info.switch1 = 0;    // ���ڱ������ء�ACC���ء�Сʱ�ƿ��ء�������״̬��ʶ
  colt_info.switch2 = 0;    // ���ڼ̵������������б�����CAN��RS23ͨ��״̬��ʶ
  colt_info.switch3 = 0;    // ���ڹ��緽ʽ�������ڲ����״̬����ʻ�ٶȡ�λ��Խ���״̬��ʶ
}

/*************************************************************************
 *
*************************************************************************/
void Collect_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_COLLECT_ID], NULL, pthread_Collect, NULL);
  usleep(10);
}

//-----�ļ�Collect.c����---------------------------------------------
