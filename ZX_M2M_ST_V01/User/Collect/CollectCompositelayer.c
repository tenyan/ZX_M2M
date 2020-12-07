/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved 
 * 
 * 文件名称: CollectCompositelayer.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为开关量采集功能模块综合层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-08,, 创建本文件
 *
 */
 
 //-----头文件调用------------------------------------------------------------
#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectHW.h"


 //-----外部变量定义------------------------------------------------------------
 extern struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;
 extern STU_A5_Date stuA5Date;
 extern STU_A5Comm stuA5Comm;


 //-----内部变量定义------------------------------------------------------------


 
 //-----内部函数声明------------------------------------------------------------
 
 
 
 //-----外部函数定义------------------------------------------------------------
/*********************************************************************************************************
** 函数名称: Task10ms
** 功能描述: //10ms执行一次的任务
** 输　	 入 : 无
** 输　	 出 : 无
** 全局变量: 
** 调用模块: 
** 作　	 者 : 
** 日　	 期 : 2019年05 月10 日
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void Task10ms(void)    
{
	 MCU_TimerCount_NoDelay();//MCU模块计时函数，此函数10ms执行一次
	 SYS_TimerCount_NoDelay();//System模块计时函数
	 Ext_Mcu_UpgradeTimer();
}


/*********************************************************************************************************
** 函数名称: SwitchInit
** 功能描述: //10ms执行一次的任务
** 输　  入 : 无
** 输　  出 : 无
** 全局变量: 
** 调用模块: 
** 作　  者 : 
** 日　  期 : 2011年07 月19 日
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void SwitchInit(void)
{
    STU_Systemstate.ucSwitch1=0;    // 用于报警开关、ACC开关、小时计开关、搭铁等状态标识
	STU_Systemstate.ucSwitch2=0;    // 用于继电器锁车、开盒报警、CAN、RS23通信状态标识  
	STU_Systemstate.ucSwitch3=0;    // 用于供电方式、外电和内部电池状态、行驶速度、位置越界等状态标识 
}


//容许延迟的定时执行函数:10ms
void Collect_TimerCount_Delay()
{
	static uint8 ucCount50ms=50;		/* 50ms计数*/
	static uint8 ucCount500ms=5;

	if(!(--ucCount500ms))
	{
        ucCount500ms = 5;
      //  SYS_POWERLED_Display();		
	}
	if(!(--ucCount50ms))
    {
        ucCount50ms=50;
	}
}


 /*********************************************************************************************************
** 函数名称: TaskCollect
** 功能描述: 此任务用来采集外部电压和电池电压并对电压进行处理，还采集
                                开关量状态和继电器状态
** 输　  入 : 无
** 输　  出 : 无
** 全局变量: 
** 调用模块: 
** 作　	 者 : 
** 日　	 期 : 2019年05 月10 日

**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

void TaskCollect (void *pData)
{    
    static uint8 ucCount100ms=10;                 /* 100ms计数*/
	static uint8 ucCount500ms=50;                 /* 100ms计数*/
    static uint8 ucCount1s=100;                   /* 1s计数*/
  
    pData = pData;

    ADInit();                           	//A/D转换初始化
	SwitchInit();
	while(1)				  
    {
        OSTimeDly(OS_TICKS_PER_SEC/100) ;   //10ms进入任务一次
		Task10ms();                         //10ms执行一次的任务
		AccountSysDate();
		

		if(!(--ucCount100ms))
        {
            ucCount100ms=10;
			CheckSwitchState();         	//读取当前开关量状态 
			ADConvert();                	//电源的采集
            MCU_TimerCount_Delay();        //100ms循环
            GpsAntenaCheck();
		}
		
		if(!(--ucCount500ms))
		{
			ucCount500ms=50;             	//500ms计数
			IWdtFeed();					    //喂狗
		}
        if(!(--ucCount1s))                 	//每1S中对采集的电压进行处理一次
        {
            ucCount1s=100;  
			DealVoltage(); 
           // CountWorkTime();
			InputVoltageCheck();
			BatVoltageCheck();
			BatContronl();			
			McuFirmwareTimerOut();
			KCMcuFirmwareTimerOut();
			A5_CommErr_Judge();
		//	printf("Time: %d - %d - %d %d : %d : %d : %d  \n \r ", stuA5Date.ucYear,stuA5Date.ucMon,stuA5Date.ucDay,stuA5Date.ucHour,stuA5Date.ucMin,stuA5Date.ucSec,stuA5Date.usmSec);

		}
     }
}
 



//-----文件CollectCompositelayer.c结束---------------------------------------------



