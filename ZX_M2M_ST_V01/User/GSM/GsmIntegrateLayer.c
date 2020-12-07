/* 
 * Copyright(c)2019,硬件研发部
 * All right reserved
 *
 * 文件名称: GsmIntegrateLayer.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块综合层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-05-01 by , 创建本文件
 *
 */

#include "config.h"
//#include "GsmInterfaceLayer.h" 
#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmIntegrateLayer.h"

//-----外部变量声明------------------------------------------------------------
//-----内部部变量定义------------------------------------------------------------
STU_GSM_State g_stuGsmState;
uint8 ucOpenTimes = 0;   //记录开机次数
//static uint8 ucAtErrCnt = 0;	//收不到AT响应计数
//static uint8 ucSimErrTimes = 0; //记录查询不到SIM卡的次数

//-----内部函数声明------------------------------------------------------------

/******************************************************************************
** 函数名称: TaskUart3Recv
** 功能描述: 处理接收到核心板发来的数据任务
**
** 输    入: pvData,扩展参数
** 输    出: 无
** 返    回: 无
**
** 作    者: lxf
** 日    期: 2019-5-28
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
	
	ModemPwrHigh();					//上电
 	OSTimeDly(OS_TICKS_PER_SEC*3);	//延时3s   
	for(i=0; i<3; i++)
	{
		ModemOpenHigh();         		//拉高
		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
		ModemOpenLow();					//拉低
		OSTimeDly(OS_TICKS_PER_SEC/2);	//500ms
		ModemOpenHigh();				//拉高
		OSTimeDly(OS_TICKS_PER_SEC*12);	//延时1s
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
		ModemOpenHigh();         		//拉高
		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
		ModemOpenLow();					//拉低
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//拉高
		OSTimeDly(OS_TICKS_PER_SEC*20);	//延时1s
		PC_SendDebugData((uint8 *)("modem off"), 9, DEBUG_ANYDATA);

		for(j=0;j<25;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
    		if(0==Modem_Status())
    			return;			
		}
	}
	ModemPwrLow();					//关电
	OSTimeDly(OS_TICKS_PER_SEC*20); //
}

#if 0
//开机操作,前提:GSM模块已上电
void ModemOpen()
{
	uint8 i;
	uint8 j = 0;	

	ModemPwrHigh();					//上电
    OSTimeDly(OS_TICKS_PER_SEC*3);	//延时3s      
	for(i=0; i<5; i++)
//	for(i=0; i<1; i++)
	{
		ModemOpenHigh();         		//拉高
		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
		ModemOpenLow();					//拉低
		OSTimeDly(OS_TICKS_PER_SEC/2);	//2s
		ModemOpenHigh();				//拉高
		OSTimeDly(OS_TICKS_PER_SEC*12);	//延时1s
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
		ModemOpenHigh();         		//拉高
		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
		ModemOpenLow();					//拉低
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//拉高
		PC_SendDebugData((uint8 *)("modem Close"), 11, DEBUG_ANYDATA);		
		OSTimeDly(OS_TICKS_PER_SEC*20);	//延时1s

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
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
		ModemOpenHigh();         		//拉高
		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
		ModemOpenLow();					//拉低
		OSTimeDly(OS_TICKS_PER_SEC*3);	//3s
		ModemOpenHigh();				//拉高
		PC_SendDebugData((uint8 *)("modem Close"), 11, DEBUG_ANYDATA);		
		/*OSTimeDly(OS_TICKS_PER_SEC*25);	//延时1s

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//延时1s
    		if(0==Modem_Status())
    			return;			
		}	*/	
//	}
}

//ucflag=0,终端刚启动是使用,给4G模块上电后直接发开机命令
//ucflag=1,需要对模块关机后再开机使用
void GSM_Modem_Start()
{

    ModemPwrHigh();             //GSM模块上电

   //开机
	ucOpenTimes++;
	if(ucOpenTimes > 10)		//反复开机不成功休息10分钟
	{
		PC_SendDebugData((uint8 *)("GSM REST"), 8, DEBUG_ANYDATA);
		ModemPwrOff();
		OSTimeDly(OS_TICKS_PER_SEC*300);
		ucOpenTimes = 1;       //下车重启模块需要先关机  所以赋值为1
		ModemPwrOn();
	}
	else if((ucOpenTimes%5)==0)  //每5次断电一次
	{
		PC_SendDebugData((uint8 *)("GSM REPWR"), 9, DEBUG_ANYDATA);
		ModemPwrOff();
		ModemPwrOn();
	}
	else                         //重新开机
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
        case 0:    //开机
            GSM_Modem_Start();
			g_stuGsmState.ucRunState = 1;
            break;
		case 1:
			
			break;
		case 2:    //开机成功
		    break;
		case 3:    //串口通信异常 重启模块  可以考虑通过复位脚控制模块重启
            GSM_Modem_Start();
			g_stuGsmState.ucRunState = 1;
			break;
		case 4:    //模块休眠   
		    //发送休眠动作   通知模块进入休眠
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

 //   InitGsmModule();            //初始化GSM模块
	
	while(1)
	{

		GSM_Run_Function();
		//过程中通过判断和模块uart口通讯判断模块是否正常，异常需要重启
		OSTimeDly(OS_TICKS_PER_SEC/10);   //100ms执行一次
	}
}

