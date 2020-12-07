/* 
 * Copyright(c)2019,Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmIntegrateLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ���ۺϲ��ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-05-01 by , �������ļ�
 *
 */

#include "config.h"
//#include "GsmInterfaceLayer.h" 
#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmIntegrateLayer.h"

//-----�ⲿ��������------------------------------------------------------------
//-----�ڲ�����������------------------------------------------------------------
STU_GSM_State g_stuGsmState;
uint8 ucOpenTimes = 0;   //��¼��������
//static uint8 ucAtErrCnt = 0;	//�ղ���AT��Ӧ����
//static uint8 ucSimErrTimes = 0; //��¼��ѯ����SIM���Ĵ���

//-----�ڲ���������------------------------------------------------------------

/******************************************************************************
** ��������: TaskUart3Recv
** ��������: ������յ����İ巢������������
**
** ��    ��: pvData,��չ����
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: lxf
** ��    ��: 2019-5-28
**-----------------------------------------------------------------------------
*******************************************************************************/
void TaskUart3Recv(void *pvData)
{
	uint8 *pData = NULL;
	uint16 len;
	
	pvData = pvData;
	GSM_GPIO_Pin_Init();
	GSM_Uart_Init();
	
    while(1)
	{	
	    if(ReadGsmUartData(&pData, &len))
		{
			A5_Deal_RecVA5_Data(pData, len);
		    PC_SendDebugData(pData, len, DEBUG_MCUMODULE);
		}
	}
}

void ModemPwrOn()
{
	uint8 i;
	uint8 j = 0;
	
	ModemPwrHigh();					//�ϵ�
 	OSTimeDly(OS_TICKS_PER_SEC*3);	//��ʱ3s   
	for(i=0; i<3; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC/2);	//500ms
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC*12);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem on"), 8, DEBUG_ANYDATA);

		for(j=0;j<15;j++)
		{
            OSTimeDly(OS_TICKS_PER_SEC);
    		if(1==Modem_Status())
    			return;			
		}
		
 	}
}

void ModemPwrOff()
{
	uint8 i;
	uint8 j = 0;
	
	for(i=0; i<2; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC*20);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem off"), 9, DEBUG_ANYDATA);

		for(j=0;j<25;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
    		if(0==Modem_Status())
    			return;			
		}
	}
	ModemPwrLow();					//�ص�
	OSTimeDly(OS_TICKS_PER_SEC*20); //
}

#if 0
//��������,ǰ��:GSMģ�����ϵ�
void ModemOpen()
{
	uint8 i;
	uint8 j = 0;	

	ModemPwrHigh();					//�ϵ�
    OSTimeDly(OS_TICKS_PER_SEC*3);	//��ʱ3s      
	for(i=0; i<5; i++)
//	for(i=0; i<1; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC/2);	//2s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC*12);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem Open"), 10, DEBUG_ANYDATA);

		for(j=0;j<15;j++)
		{
            OSTimeDly(OS_TICKS_PER_SEC);
    		if(1==Modem_Status())
    			return;			
		}
	}
}
#endif
void ModemClose(void)
{
	uint8 i=0;
	uint8 j=0;

	for(i=0; i<2; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//����
		PC_SendDebugData((uint8 *)("modem Close"), 11, DEBUG_ANYDATA);		
		OSTimeDly(OS_TICKS_PER_SEC*20);	//��ʱ1s

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
    		if(0==Modem_Status())
    			return;			
		}		
	}
}

void ModemClose_sleep(void)
{
//	uint8 i=0;
//	uint8 j=0;

//	for(i=0; i<1; i++)
//	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//����
		PC_SendDebugData((uint8 *)("modem Close"), 11, DEBUG_ANYDATA);		
		/*OSTimeDly(OS_TICKS_PER_SEC*25);	//��ʱ1s

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
    		if(0==Modem_Status())
    			return;			
		}	*/	
//	}
}

//ucflag=0,�ն˸�������ʹ��,��4Gģ���ϵ��ֱ�ӷ���������
//ucflag=1,��Ҫ��ģ��ػ����ٿ���ʹ��
void GSM_Modem_Start()
{

    ModemPwrHigh();             //GSMģ���ϵ�

   //����
	ucOpenTimes++;
	if(ucOpenTimes > 10)		//�����������ɹ���Ϣ10����
	{
		PC_SendDebugData((uint8 *)("GSM REST"), 8, DEBUG_ANYDATA);
		ModemPwrOff();
		OSTimeDly(OS_TICKS_PER_SEC*300);
		ucOpenTimes = 1;       //�³�����ģ����Ҫ�ȹػ�  ���Ը�ֵΪ1
		ModemPwrOn();
	}
	else if((ucOpenTimes%5)==0)  //ÿ5�ζϵ�һ��
	{
		PC_SendDebugData((uint8 *)("GSM REPWR"), 9, DEBUG_ANYDATA);
		ModemPwrOff();
		ModemPwrOn();
	}
	else                         //���¿���
	{
		PC_SendDebugData((uint8 *)("GSM OPEN AGAIN"), 14, DEBUG_ANYDATA);
		if(ucOpenTimes!=1)
	    	ModemClose();
		//ModemOpen();
		ModemPwrOn();
	}
}

void GSM_Run_Function(void)
{

    switch(g_stuGsmState.ucRunState)
    {
        case 0:    //����
            GSM_Modem_Start();
			g_stuGsmState.ucRunState = 1;
            break;
		case 1:
			
			break;
		case 2:    //�����ɹ�
		    break;
		case 3:    //����ͨ���쳣 ����ģ��  ���Կ���ͨ����λ�ſ���ģ������
            GSM_Modem_Start();
			g_stuGsmState.ucRunState = 1;
			break;
		case 4:    //ģ������   
		    //�������߶���   ֪ͨģ���������
         //   USB_VBUS_OFF();
			ModemDTRHigh();	
			ModemClose_sleep();
		    g_stuGsmState.ucRunState = 5;
	    	break; 
        default:
			break;		
	}
}


void TaskGsmService(void * pvData)
{

    pvData = pvData;

 //   InitGsmModule();            //��ʼ��GSMģ��
	
	while(1)
	{

		GSM_Run_Function();
		//������ͨ���жϺ�ģ��uart��ͨѶ�ж�ģ���Ƿ��������쳣��Ҫ����
		OSTimeDly(OS_TICKS_PER_SEC/10);   //100msִ��һ��
	}
}

