/*****************************************************************************
* @FileName: tbox_machine.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
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

static uint8_t acc_off_event_flag = 0; // ACC��ON��ΪOFF��־λ
static uint32_t acc_off_cumulated_timer = 0;
static uint32_t acc_off_reset_cumulated_timer = 0;
/******************************************************************************
 * ���繩�������,�ն˶ϵ���������(1Sִ��һ��)
 * ��ACC�رճ���ʱ��ﵽ���߻��Ѽ��ʱ��ʱ��֪ͨAM1805���������жϵ�10����
 * Ȼ�������ϵ�;֮�����ACC�Գ���ΪOFF״̬����ÿ���24Сʱ֪֪ͨͨAM1805��
 * �������жϵ�10����Ȼ�������ϵ硣
*******************************************************************************/
void Tbox_CheckPowerOffMachine(void)
{
  uint8_t current_acc_state;
  static uint8_t previous_acc_state = 0;

  current_acc_state = COLT_GetAccStatus();
  if(current_acc_state==0) // ACC�ر�
  {
    if ((acc_off_cumulated_timer >= m2m_asset_data.sleep_time_sp*60) || (acc_off_reset_cumulated_timer >= 86400))
    {
      acc_off_cumulated_timer = 0;
      acc_off_reset_cumulated_timer = 0;
      
      PcDebug_SendString("Tbox:RtcPwrOff!\n");
      TBOX_DELAY(OS_TICKS_PER_SEC);
      
      am18x5_PswControl(1); // ֪ͨAM1805�Ͽ��ն��ⲿ�͵�ص�Դ
      am18x5_SetSleepTime(10,2); // ֪ͨAM1805(RTCоƬ)�ϵ�10��(�����硢�ص�ص�)
      TBOX_DELAY(OS_TICKS_PER_SEC);
      MAIN_POWER_OFF();  // ������
      TBOX_DELAY(OS_TICKS_PER_SEC*3);
    }
  }

  if(previous_acc_state != current_acc_state)  // ACC״̬�����ж�
  {
    if (current_acc_state==0) // ACC�ر�
    {  acc_off_event_flag = 1;}
  }
  
  if ((current_acc_state==0) && (acc_off_event_flag==1))
  {  acc_off_cumulated_timer++;}
  else
  {  acc_off_cumulated_timer = 0;}

  if (current_acc_state==0) // ACC�ر�
  {  acc_off_reset_cumulated_timer++;}
  else
  {  acc_off_reset_cumulated_timer = 0;}

  previous_acc_state = current_acc_state; // ����״̬
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
//uint32_t quit_sleep_timer_sp = 7200; // 2Сʱ
uint32_t quit_sleep_timer_sp = 300; // 2Сʱ
static uint32_t quit_sleep_timer; // ���Ѽ����ʱ��
//============================================================================
void Tbox_CheckSleepStatus(void)
{
  uint16_t vraw;    // ���Դ��ѹ,��λ10mV
  static uint8_t acc_on_debounce_cnt = 0;
  static uint8_t voltage_full_debounce_cnt = 0;

  // ��ACC�����ж� ��ֹ�ж��޷���������
  if (ACC_IN()==0x00) // ACC��
  {
    if (acc_on_debounce_cnt > 5) // 5��
      quit_sleep_timer = 0;
    else
      acc_on_debounce_cnt++;
  }
  else
  {
    acc_on_debounce_cnt = 0;
  }

  //�����Դ��ѹ�����ж� �ﵽ������ѹ�ǻ���
  vraw = (uint16_t) (adc_dma_buffers[ADC_CHANNEL_VRAW] * 121 / 125 + 27);  // ��λ��0.01V
  if ((vraw>1300 && vraw<1510)|| (vraw>2650) || (vraw <500))
  {
    if (voltage_full_debounce_cnt > 5) // 5��
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
 * ��λ����ʱ��
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

    //==Ӳ��ʹ��=============================================
    MAIN_POWER_ON();     // ʹ�����繩��
    CAN_POWER_ON();
    //GPS_SetModuleState(GPS_MODULE_STATE_POWERON); // GPSģ�鿪��
    Modem_SetState(MODEM_STATE_POWER_ON); // 4Gģ�鿪��
    
    rtc_time = RTC_GetBjTime(); // ��ȡRTCʱ��
    PcDebug_Printf("Tst:%d-%d-%d %d:%d:%d\n\r", rtc_time.year,rtc_time.month,rtc_time.day,rtc_time.hour,rtc_time.minute,rtc_time.second);
    rtcSoft_Init(&rtc_time);
    
    tbox_10msec_timer = 300; // ��ʱ3S,�����Լ�
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

static uint32_t enter_sleep_timer_sp = 30000; // 5����
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
    enter_sleep_timer_sp = m2m_asset_data.wakeup_work_time_sp*100; // 10msʱ��
    enter_sleep_timer = enter_sleep_timer_sp;
    tbox_10msec_timer = 100;
  }
  else
  {
    if (tbox_10msec_timer == 0)
    {
      if (COLT_GetAccStatus()==1) // ACC��
      {
        enter_sleep_timer = enter_sleep_timer_sp; // ��������ʱ��
      }

      if((CAN_GetRecvState(CAN_CHANNEL1)==CAN_OK)||(CAN_GetRecvState(CAN_CHANNEL2)==CAN_OK)) // CAN����
      {
        enter_sleep_timer = enter_sleep_timer_sp; // ��������ʱ��
      }

      if(m2m_context.update.state > M2M_UPDATE_STATE_IDLE)  // �ն���������
      {
        enter_sleep_timer = enter_sleep_timer_sp; // ��������ʱ��
      }

      // ��������ж�

      //========================================
      //if()  // ��������ģʽ
      //{
      //  tbox_state_transition = TRUE;
      //  tbox_state = TBOX_STATE_IAP;
      //  return;
      //}

      //========================================
      if (enter_sleep_timer==0) // ��������ģʽ
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
  case TBOX_SLEEP_STATE_INIT: // �ر�Ӳ��,ȷ�������Ǹ�����״̬
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      //=�ر�Ӳ��====================================
      CAN_POWER_OFF();
      //GPS_SetModuleState(GPS_MODULE_STATE_SLEPT); // �ر�GPSģ��
      tbox_10msec_timer = 100;
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        if ((0==COLT_GetMainPowerStatus()) && (0==COLT_GetMainPowerLowStatus()))// ���繩�������粻����
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

  case TBOX_SLEEP_STATE_LIGHT: // �������(���ϵ�,RTCһ�뻽��һ��)
    /*
     * ��STM32�����о������õ��������Ź���IWDG���͵͹���ģʽ�����Ź���Ϊ�˼��ͽ���������������Ĺ��ϣ�
     * �͹���ģʽ��Ϊ����CPU����Ҫ��������ʱ���뵽����ģʽ���Խ�ʡ���ܡ�
     * ���ж������Ź���ʱ���ɶ�����RC������STM32F10xһ��Ϊ40kHz���ṩ����ʹ����ʱ�ӳ��ֹ���ʱ��Ҳ��Ȼ��Ч��
     * ��˿�����ֹͣ�ʹ���ģʽ�¹��������Ҷ������Ź�һ������������ϵͳ��λ���������ٱ�ֹͣ��
     * ������������һ�������ǵ�MCU���뵽�͹���ģʽ������CPUֹͣ�����޷�ι�����ᵼ��ϵͳƵ����λ��
     */
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      PcDebug_SendString("Tbox:L-Sleep!\n");
      //=�ر�Ӳ��====================================
      Modem_SetState(MODEM_STATE_POWER_OFF); // �ر�4Gģ��
      for (it=0; it<60; it++) // ��֤ģ�������ػ�
      {
        TBOX_DELAY(OS_TICKS_PER_SEC);
        if(Modem_GetState()==MODEM_STATE_OFF) // 4Gģ�������ر�
        {
          break;
        }
      }
      tbox_10msec_timer = 10;
      quit_sleep_timer = m2m_asset_data.sleep_time_sp * 60; // ��Сʱ
      //quit_sleep_timer = 3 * 60; // ��Сʱ
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
          IWDG_Feed(); // ����ͣ��ģʽǰι��
          am18x5_SetSleepTime(1,1);  // 1���ӻ���һ��
          // ֹͣģʽ��IO�ڱ���ֹͣǰ��״̬
          // PWR_STOPEntry_WFI=�жϻ���; PWR_STOPEntry_WFE=�¼�����
          PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI); // CPU����ֹͣģʽ,����ʱ��(���Ź���ֹͣ)��ֹͣ(ֻ��EXTI�ж��ߺ��ڲ�RTC�����ܻ���)
          // �ȴ�����(�ⲿ���ӡ�CAN�����ǡ�ACC)
          IWDG_Feed(); // ���Ѻ���ι��
          SYSCLKConfig_STOP();  // ϵͳ��ֹͣģʽ������ʱ,ʱ��ʹ��HSI(8MHz)��Ϊϵͳʱ��
          am18x5_ClearIntFlag();
          if (quit_sleep_timer==0)
          {
            break;  // ����ʱ�䵽
          }
        }while (quit_sleep_timer);
        osKernelUnlock();
        //PcDebug_Printf("L-SleepWkpCnt=%d\n",light_sleep_wkp_cnt);
        tbox_sleep_state = TBOX_SLEEP_STATE_EXIT;
        tbox_started_delay = FALSE;
      }
    }
    break;

  case TBOX_SLEEP_STATE_DEEP: // �������(�ϵ�,RTC����,24Сʱ����һ��)
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      PcDebug_SendString("Tbox:D-Sleep!\n");
      //=�ر�Ӳ��====================================
      BAT_CHARGE_OFF();
      Modem_SetState(MODEM_STATE_POWER_OFF); // �ر�4Gģ��
      for (it=0; it<60; it++) // ��֤ģ�������ػ�
      {
        TBOX_DELAY(OS_TICKS_PER_SEC);
        if(Modem_GetState()==MODEM_STATE_OFF) // 4Gģ�������ر�
        {
          break;
        }
      }
      
      tbox_10msec_timer = 10;
      quit_sleep_timer = 86398;  // 24Сʱ
      //quit_sleep_timer = 3 * 60; // ��Сʱ
    }
    else
    {
      if (tbox_10msec_timer == 0)
      {
        PcDebug_SendString("Tbox:D-Slept!\n");
        am18x5_PswControl(1);
        am18x5_SetSleepTime(quit_sleep_timer, 2);  // ��������,ϵͳ�ϵ�(�����硢�ص�ص�)
        TBOX_DELAY(OS_TICKS_PER_SEC);
        MAIN_POWER_OFF();  // ������
        TBOX_DELAY(OS_TICKS_PER_SEC*3);
      }
    }
    break;

  case TBOX_SLEEP_STATE_EXIT: // �˳�����,������������
    if (tbox_started_delay == FALSE)
    {
      tbox_started_delay = TRUE;
      //=����Ӳ��====================================
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
      //=�ر�Ӳ��====================================
      IWDG_Feed();
      BAT_CHARGE_OFF();
      CAN_POWER_OFF();
      //GPS_SetModuleState(GPS_MODULE_STATE_SLEPT); // �ر�GPSģ��
      Modem_SetState(MODEM_STATE_POWER_OFF); // �ر�4Gģ��
      tbox_10msec_timer = 2000;  // �ȴ�20��,��֤����ر�
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

      tbox_10msec_timer = 50;  // �ȴ�0.5��
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
        //taskENTER_CRITICAL(); // �����ٽ���
        iap_main();
        //taskEXIT_CRITICAL();  // �˳��ٽ���
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
        tbox_state = TBOX_STATE_POWERON; // ����������
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
        __set_FAULTMASK(1);     // �ر������ж�
        NVIC_SystemReset();			// ��λ
      }
    }
    break;
  }
}

/*****************************************************************************
* ����GPS�ն��ϵ��ʼ�����������������ߡ��̼�����
******************************************************************************/
void tbox_state_machine(void)
{
  static uint16_t divide_for_1s = 6000;

  switch (tbox_state)
  {
  case TBOX_STATE_POWERON: // �ϵ��ʼ��
    tbox_power_on();
    break;

  case TBOX_STATE_WORKING: // ��������
    tbox_working();
    break;

  case TBOX_STATE_SLEEP: // ����
    tbox_sleep();
    break;

  case TBOX_STATE_IAP: // �̼�����
    tbox_iap();
    break;

  default:
    break;
  }

  if (tbox_10msec_timer) // ��ͨ��ʱ������
    tbox_10msec_timer--;

  if (enter_sleep_timer) // ���߶�ʱ������
    enter_sleep_timer--;

  if(divide_for_1s) // 1S��ʱ��(10ms)
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
* ����ϵͳ����״̬
******************************************************************************/
void Tbox_SetMachineState(tbox_state_t state)
{
  tbox_state_transition = TRUE;
  tbox_state = state;
}


