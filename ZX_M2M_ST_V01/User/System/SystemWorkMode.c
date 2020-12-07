/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: SystemWorkMode.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪSystem����ģ�鹤��ģʽ������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-05, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "SystemWorkMode.h"
#include "RTC.h"
#include "CollectInterfaceLayer.h"
//-----�ⲿ��������------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern uint8   g_ucRstCount;            //�ݶ�����������
extern uint16  g_usRstTime;          //�ݶ�����ʱ����
extern stuFirmwareUpdate FirmwareUpdate;
extern uint8 ucOpenTimes;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
//-----�ڲ���������------------------------------------------------------------


//-----�ڲ���������------------------------------------------------------------
STUSleep stuSleep;              //���߽ṹ�嶨��
extern uint16 ADC3ConvertedValue;
extern STU_SSData SSData;
extern STU_GSM_State g_stuGsmState;
//-----�ⲿ��������------------------------------------------------------------


/******************************************************************************
** ��������: Get_SYSWorkingMode
** ��������: 
** 
** ��    ��: 
** ��    ��: ��
** ��    ��: g_stuSystem.ucWorkingMode,�豸����״̬
**
** ��    ��: Lxf
** ��    ��: 2011-06-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 Get_SYSWorkingMode(void)
{
    return stuSleep.ucWorkingMode;
}

void SYS_GPSSleep(void)
{
  
    stuSleep.ucGpsSleepState = 1;
 //  	GPS_SleepGps();
}

void SYS_GPSWake(void)
{
    stuSleep.ucGpsSleepState = 0;
  //  GPS_WakeGps();
}
void SYS_GSMPwrofff(void)
{
	stuSleep.ucGsmSleepState = 1;
    g_stuSystem.ucOnline = 0;
	//GSM_PwrOff();
}

void SYS_GSMSleep(void)
{
    stuSleep.ucGsmSleepState = 1;
    g_stuSystem.ucOnline = 0;
	OSTimeDly(OS_TICKS_PER_SEC * 3); //��ֹ������������ʱ�豸�������ߵ���ʱʱ��  
	g_stuGsmState.ucRunState = 4;
	OSTimeDly(OS_TICKS_PER_SEC * 10);
 
}

void SYS_GSMWake(void)
{
    stuSleep.ucGsmSleepState = 0;     //GSm���߻��ѱ�־
//    GSM_Wake();
    stuSleep.ucFlag = 0;
	g_stuGsmState.ucRunState = 0;
    ucOpenTimes = 0;
	PC_SendDebugData((uint8 *)("GsmWake"), 7, DEBUG_ANYDATA);
}


void MainPowerCheck(void)
{
	uint8 i;
	uint32 uifTemp = 0;
	uint16 ADtemp[20]={0};
	uint16 usInputVoltage = 0;
	static uint8 ucAccCount = 5;
	static uint8 ucvolCount=5;

    //��ACC�����ж� ��ֹ�ж��޷���������
	if(!ReadSwitchACC())     //ACC Ϊ��
	{
	    if(ucAccCount)
	    {
            if(!(--ucAccCount))
            {
                stuSleep.usSleepTimes = 0;
				ucAccCount = 5;
			}
		}
	}
	else
	{
        ucAccCount = 5;
	}	
	
    //�����Դ��ѹ�����ж� �ﵽ������ѹ�ǻ���
    for(i=0;i<20;i++)
    {
    	ADtemp[i] = ADC3ConvertedValue;
	}
	uifTemp = DigitFilter1(ADtemp, 20, 2);
    usInputVoltage = (uint16)(uifTemp*0.09659); //�����ⲿ��ѹֵ
    if((usInputVoltage>130&&usInputVoltage<151)|| (usInputVoltage>265) || (usInputVoltage<50))
    {
	    if(ucvolCount)
	    {
            if(!(--ucvolCount))
            {
                stuSleep.usSleepTimes = 0;
				ucvolCount = 5;
			}
		}	 
	}
	else
	{
        ucvolCount = 5;     
	}		
//��ʱ
	g_stuSystem.uiResetTimer++;	
    if(g_stuSystem.ucAccFlag==1)
	    g_stuSystem.uiAccOffTimer++;
}

/****************************************************************************
  * @brief  Configures system clock after wake-up from STOP: enable HSE, PLL
  *         and select PLL as system clock source.
  * @param  None
  * @retval None
  ****************************************************************************/
//SystemInit();
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


/******************************************************************************
** ��������: CPU_PowerDown
** ��������: CPU����������ߺ���
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2016-06-30
**-----------------------------------------------------------------------------
*******************************************************************************/
void CPU_PowerDown(void)
{
	 //�ر���Χ�õ��豸
    Mcu_CanClose();          //�ر�CANģ���Դ
    ChargeOff(); 		     //�رյ�س��,��ֹ����ǰû�йر�
    SYS_GPSSleep();
    SYS_GSMSleep();
	if((0==GetPwrSupplyState()) && (0==GetPwrLowState()))//���繩�������粻����
	{
//		SYS_GSMSleep();
		OSTimeDly(OS_TICKS_PER_SEC * 10);
		
		stuSleep.usSleepTimes = g_stuSYSParamSet.usSleepWakeSlot*60;
		while(stuSleep.usSleepTimes)
		{
			IWdtFeed();
			stuSleep.usSleepTimes--;
			MainPowerCheck();
			RTCSetSleepTime(1, 1);    //1���ӻ���һ��			
			PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);
			SYSCLKConfig_STOP();
			IWdtFeed();
		 	am_clr_intflag();
			if(0==stuSleep.usSleepTimes)
				break;
		}
		IWdtFeed();
		stuSleep.ucReadRTCClockFlag = 1;
		OSTimeDly(OS_TICKS_PER_SEC);
		PC_SendDebugData((uint8 *)("CPU_WK"),6, DEBUG_ANYDATA);
		if(stuSleep.ucGsmSleepState)
		{
		    stuSleep.usWakeLastTime = WEAKLAST_TIME;   //���Ѻ�ά������5����
		    SYS_GPSWake();
		    SYS_GSMWake();            
		}
	}
	else	//���õ�ع������������
	{
		SYS_GSMPwrofff();
		OSTimeDly(OS_TICKS_PER_SEC * 10); 
		PSWControl(1);
		RTCSetSleepTime(86398, 2);   //24Сʱ
		MainPowOff();
		OSTimeDly(OS_TICKS_PER_SEC*3);
		stuSleep.ucReadRTCClockFlag = 1;
	//	BatPowOff();
	}
   // BLU_LED_OFF();    //�رյ�Դָʾ��       
}

void SYS_ResetSleeptime(void)
{
	stuSleep.usSleepTimes = 0;
}
/******************************************************************************
** ��������: SYS_WorkModeSleep_Count
** ��������: ���ն�����ǰʱ�����,����ά��ʱ��������м�ʱ����,(�ú���ÿ��ִ��һ��)
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-08-05
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkModeSleep_Count(void)
{

   // if(FirmwareUpdate.ucStep>0)      //Զ������ʱ����������
    if(stu_ExtMCUUpgrade.ucUpgradeStep>0||FirmwareUpdate.ucMainCPuUpgradeFlag>0)
        return;
	//�����ڼ�ģ��ȫ�����Ѻ��ʱ5����
	if((!(GetAccState())) && (2==GetCanRcvState(CAN_CHANNEL1))  && (0==MCU_GetVoltageState()))
    {	    
        if(stuSleep.usWakeLastTime&&(stuSleep.ucWorkingMode > WORK_MODEL_NORMAL))
        {//PC_SendDebugData((uint8 *)("r0"), 2, DEBUG_ANYDATA);
            if(!(--stuSleep.usWakeLastTime))  
            {
                stuSleep.ucFlag|=BIT(1);
            } 
			if(stuSleep.usWakeLastTime==5)
			{
         //       SSData.usTimer = 0;
			}
			
        }
		//ACC OFF ���������ǰ��ʱ
        if(stuSleep.usSleepBeforSlot)
        {
            if(!(--stuSleep.usSleepBeforSlot)) 
            {
                stuSleep.ucFlag|=BIT(2);
            }
        }
    }
    else            //��⵽�������ڹ���
    {
	    stuSleep.usWakeLastTime=0;
        stuSleep.ucFlag = 0;
        stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //���Ѻ����¸�ֵ		
    }

	if(stuSleep.ucWork_Heart_Count)
    {
        if(!(--stuSleep.ucWork_Heart_Count))  
			stuSleep.ucWork_heart_Flag = 1;
    }  

}

/******************************************************************************
** ��������: SYS_WorkModeInit
** ��������: �ն˹���ģʽ��ʼ��
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-06-30
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkModeInit(void)
{
    stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;    //����ģʽ����ֵ
    stuSleep.usWakeLastTime = 0;          //���߻��Ѻ��������ʱ��
    stuSleep.ucWork_heart_Flag = 0;                //�������ߵı�־��ֵΪ0
    stuSleep.ucWork_Heart_Count = 0;               //�������ߵĵȴ�ʱ�丳ֵΪ0
}

/******************************************************************************
** ��������: SYS_WorkMode_Exec
** ��������: �ն˹���ģʽ�л�������
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkMode_Exec(void)
{
	//�����ߡ����䡢��������¼�⵽Կ��Ϊ�ߵ�����Զ�����    
	if(((1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState()||GetAccState())&&(stuSleep.ucWorkingMode>WORK_MODEL_NORMAL))                
	{
        if(stuSleep.ucGsmSleepState)
		{
			PC_SendDebugData((uint8*)"WAKE2", 5, DEBUG_ANYDATA); 
			SYS_GPSWake();                     // ����GPSģ��
			SYS_GSMWake();                     // ����GSMģ��  
		}
		McuInit();  // ��ʼ��CAN������
	//	BatPowOn();
		MainPowOn();
		PSWControl(0);
		am_disable_alm();
		stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;
		stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //���Ѻ����¸�ֵ
		return;            
	}
	
  //  if(FirmwareUpdate.ucStep>0)      //Զ������ʱ����������
    if(stu_ExtMCUUpgrade.ucUpgradeStep>0||FirmwareUpdate.ucMainCPuUpgradeFlag>0)
        return;

	//���Ѻ��������5���Ӻ��ٽ�������
	if((stuSleep.ucFlag&BIT(1))&&g_stuSYSParamSet.usSleepBeforSlot)   
	{
		stuSleep.ucFlag&=~BIT(1);           //�����־λ
		if(stuSleep.ucWorkingMode==WORK_MODEL_SLEEP)           //����
		{  
			PC_SendDebugData((uint8*)"SLEEP4", 6, DEBUG_ANYDATA);   //test  �������
			CPU_PowerDown();       //�趨�������� �������ò���(��������)        
		}  
	    else 
        {
            PC_SendDebugData((uint8 *)"MODE=ERR", 8, DEBUG_ANYDATA);//���Խ��г�����        
        }
	}

    switch(stuSleep.ucWorkingMode)
	{
		case WORK_MODEL_NORMAL:           
			//ACC���߱�������Ϊ����ǿ�Ʋ�����
			if(GetAccState()||(1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState())
			{            
				stuSleep.ucFlag = 0;
				stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //���Ѻ����¸�ֵ
			}        

			//ACC�رպ�����ǰ��ʱʱ�䵽
			if((stuSleep.ucFlag&BIT(2))&&g_stuSYSParamSet.usSleepBeforSlot)
			{
				stuSleep.ucWorkingMode = WORK_MODEL_SLEEP;
				stuSleep.ucWork_Heart_Count = WORK_HEART_TIME;
				stuSleep.ucFlag&=~BIT(2);
		//		g_stuGsmState.ucRunState = 4;
        //        SSData.usTimer = 0;
			}            
			break;
		
		case WORK_MODEL_SLEEP:               
			if(stuSleep.ucWork_heart_Flag)
			{
				stuSleep.ucWork_heart_Flag = 0;
				PC_SendDebugData((uint8*)"SLEEP1", 6, DEBUG_ANYDATA);  
				CPU_PowerDown();                     //��CPU�����������                 
			}
			break;
		default:
			break;
	}            

}
//-----�ļ�SystemWorkMode.c����---------------------------------------------

