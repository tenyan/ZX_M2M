/*
 * Copyright(c)2019, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
 * All right reserved 
 * 
 * �ļ�����: CollectCompositelayer.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�Ϊ�������ɼ�����ģ���ۺϲ��ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-18,lxf, �������ļ�
 *
 */
 
 //-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectHW.h"


 //-----�ⲿ��������------------------------------------------------------------
 extern struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;


 //-----�ڲ���������------------------------------------------------------------


 
 //-----�ڲ���������------------------------------------------------------------
 
 
 
 //-----�ⲿ��������------------------------------------------------------------
/*********************************************************************************************************
** ��������: Task10ms
** ��������: //10msִ��һ�ε�����
** �䡡	 �� : ��
** �䡡	 �� : ��
** ȫ�ֱ���: 
** ����ģ��: 
** ����	 �� : 
** �ա�	 �� : 2019��03 ��19 ��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Task10ms(void)    
{
	 MCU_TimerCount_NoDelay();//MCUģ���ʱ�������˺���10msִ��һ��
	 SYS_TimerCount_NoDelay();//Systemģ���ʱ����
	 //GSM_TimerCount_NODelay();//GSMģ���ʱ����
	 GPS_TimerCount_NoDelay();
}
 
/*********************************************************************************************************
** ��������: SwitchInit
** ��������: //10msִ��һ�ε�����
** �䡡  �� : ��
** �䡡  �� : ��
** ȫ�ֱ���: 
** ����ģ��: 
** ����  �� : 
** �ա�  �� : 2019��03 ��19 ��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void SwitchInit(void)
{
    STU_Systemstate.ucSwitch1=0;    // ���ڱ������ء�ACC���ء�Сʱ�ƿ��ء�������״̬��ʶ
	STU_Systemstate.ucSwitch2=0;    // ���ڼ̵������������б�����CAN��RS23ͨ��״̬��ʶ  
	STU_Systemstate.ucSwitch3=0;    // ���ڹ��緽ʽ�������ڲ����״̬����ʻ�ٶȡ�λ��Խ���״̬��ʶ 
}

#if 0
//�����ӳٵĶ�ʱִ�к���:10ms
void Collect_TimerCount_Delay()
{
	static uint8 ucCount50ms=50;		/* 50ms����*/
	static uint8 ucCount500ms=5;

	if(!(--ucCount500ms))
	{
        ucCount500ms = 5;
        //SYS_POWERLED_Display();		
	}
	if(!(--ucCount50ms))
    {
        ucCount50ms=50;
		//ADConvert();                	//��Դ�Ĳɼ�
	}
}
#endif
/*********************************************************************************************************
** ��������: pthread_Collect_Function
** ��������: ������������λ���Ȳɼ��������ɼ��ⲿ������״̬����ѹ�͵�ص�ѹ��
** �䡡  ��: ��
** ȫ�ֱ���: 
** ����ģ��: 
** ����	 �� : 
** �ա�	 �� : 2019��03 ��18 ��

**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void* pthread_Collect_Function(void *data)
{
    static uint8 ucCount100ms=10;                 /* 100ms����*/
	static uint8 ucCount500ms=50;                 /* 100ms����*/
    static uint8 ucCount1s=100;                   /* 1s����*/

    data = data;

	SwitchInit();
	InitWorktime();	
//	Watchdog_Init();
    SYS_ParamRead();
	while(1)				  
    {
        usleep(One_MilliSecond*10);       //10ms ��ʱ
		Task10ms();                       //10msִ��һ�ε�����
		if(!(--ucCount100ms))
        {
            ucCount100ms=10;
		
    		MCU_TimerCount_Delay();        //100msѭ��
            SYS_SendNetData();
		}
		
		if(!(--ucCount500ms))
		{
			ucCount500ms=50;             	//500ms����
	
		}
        if(!(--ucCount1s))                 	//ÿ1S�жԲɼ��ĵ�ѹ���д���һ��
        {
            ucCount1s=100;   
	
         // CountWorkTime();
		//	InputVoltageCheck();
		//	BatVoltageCheck();
	
			ReportSwitchState();
			Sys_GPSState_Change();   
			//McuFirmwareTimerOut();
			MCU_SendCorePlateData(0x02);
		//	A5Uart0_TimingSendClock();   //��ʱΪMCUУʱ
		//	Watchdog_Feed();        //ι��
        }
     }
}






//-----�ļ�CollectCompositelayer.c����---------------------------------------------


