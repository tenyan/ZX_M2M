/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
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
 * 2019-03-16, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "SystemWorkMode.h"
//#include "RTC.h"
#include "CollectInterfaceLayer.h"
//-----�ⲿ��������------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern stuFirmwareUpdate FirmwareUpdate;
//-----�ڲ���������------------------------------------------------------------


//-----�ڲ���������------------------------------------------------------------
STUSleep stuSleep;              //���߽ṹ�嶨��
extern uint16 ADC3ConvertedValue;
extern STU_SSData SSData;
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
   	GPS_SleepGps();
}

void SYS_GPSWake(void)
{
    stuSleep.ucGpsSleepState = 0;
    GPS_WakeGps();
}
void SYS_GSMPwrofff(void)
{
	stuSleep.ucGsmSleepState = 1;
    g_stuSystem.ucOnline = 0;
	GSM_PwrOff();
}

void SYS_GSMSleep(void)
{
    stuSleep.ucGsmSleepState = 1;
    g_stuSystem.ucOnline = 0;
    GSM_Sleep();
}

void SYS_GSMWake(void)
{
    stuSleep.ucGsmSleepState = 0;     //GSm���߻��ѱ�־
    GSM_Wake();
    stuSleep.ucFlag = 0;
	PC_SendDebugData((uint8 *)("GsmWake"), 7, DEBUG_ANYDATA);
}

#if 0


/******************************************************************************
** ��������: SYS_WorkModeInit
** ��������: �ն˹���ģʽ��ʼ��
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2019-03-30
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
** ��    ��: 2019-03-01
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
		BatPowOn();
		MainPowOn();
		PSWControl(0);
		am_disable_alm();
		stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;
		stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //���Ѻ����¸�ֵ

		return;            
	}
	
    if(FirmwareUpdate.ucStep>0)      //Զ������ʱ����������
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
#endif

//-----�ļ�SystemWorkMode.c����---------------------------------------------

