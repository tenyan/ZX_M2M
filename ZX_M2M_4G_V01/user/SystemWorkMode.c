/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: SystemWorkMode.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为System功能模块工作模式处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16, by  lxf, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "SystemWorkMode.h"
//#include "RTC.h"
#include "CollectInterfaceLayer.h"
//-----外部变量定义------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern stuFirmwareUpdate FirmwareUpdate;
//-----内部变量定义------------------------------------------------------------


//-----内部函数声明------------------------------------------------------------
STUSleep stuSleep;              //休眠结构体定义
extern uint16 ADC3ConvertedValue;
extern STU_SSData SSData;
//-----外部函数定义------------------------------------------------------------


/******************************************************************************
** 函数名称: Get_SYSWorkingMode
** 功能描述: 
** 
** 输    入: 
** 输    出: 无
** 返    回: g_stuSystem.ucWorkingMode,设备工作状态
**
** 作    者: Lxf
** 日    期: 2011-06-26
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
    stuSleep.ucGsmSleepState = 0;     //GSm休眠唤醒标志
    GSM_Wake();
    stuSleep.ucFlag = 0;
	PC_SendDebugData((uint8 *)("GsmWake"), 7, DEBUG_ANYDATA);
}

#if 0


/******************************************************************************
** 函数名称: SYS_WorkModeInit
** 功能描述: 终端工作模式初始化
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2019-03-30
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkModeInit(void)
{
    stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;    //工作模式赋初值
    stuSleep.usWakeLastTime = 0;          //休眠唤醒后持续工作时间
    stuSleep.ucWork_heart_Flag = 0;                //允许休眠的标志赋值为0
    stuSleep.ucWork_Heart_Count = 0;               //进入休眠的等待时间赋值为0
}

/******************************************************************************
** 函数名称: SYS_WorkMode_Exec
** 功能描述: 终端工作模式切换处理函数
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2019-03-01
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkMode_Exec(void)
{
	//在休眠、运输、掉电情况下检测到钥匙为高的情况自动唤醒    
	if(((1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState()||GetAccState())&&(stuSleep.ucWorkingMode>WORK_MODEL_NORMAL))                
	{
        if(stuSleep.ucGsmSleepState)
		{
			PC_SendDebugData((uint8*)"WAKE2", 5, DEBUG_ANYDATA); 
			SYS_GPSWake();                     // 唤醒GPS模块
			SYS_GSMWake();                     // 唤醒GSM模块  
		}
		McuInit();  // 初始化CAN控制器
		BatPowOn();
		MainPowOn();
		PSWControl(0);
		am_disable_alm();
		stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;
		stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //唤醒后重新赋值

		return;            
	}
	
    if(FirmwareUpdate.ucStep>0)      //远程升级时不进入休眠
    		return;

	//唤醒后持续运行5分钟后再进入休眠
	if((stuSleep.ucFlag&BIT(1))&&g_stuSYSParamSet.usSleepBeforSlot)   
	{
		stuSleep.ucFlag&=~BIT(1);           //清除标志位
		if(stuSleep.ucWorkingMode==WORK_MODEL_SLEEP)           //休眠
		{  
			PC_SendDebugData((uint8*)"SLEEP4", 6, DEBUG_ANYDATA);   //test  调试输出
			CPU_PowerDown();       //设定唤醒周期 按照设置参数(唤醒周期)        
		}  
	    else 
        {
            PC_SendDebugData((uint8 *)"MODE=ERR", 8, DEBUG_ANYDATA);//可以进行出错处理        
        }
	}

    switch(stuSleep.ucWorkingMode)
	{
		case WORK_MODEL_NORMAL:           
			//ACC或者保留开关为高则强制不休眠
			if(GetAccState()||(1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState())
			{            
				stuSleep.ucFlag = 0;
				stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //唤醒后重新赋值
			}        

			//ACC关闭后休眠前计时时间到
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
				CPU_PowerDown();                     //主CPU进入掉电休眠                 
			}
			break;
		default:
			break;
	}            

}
#endif

//-----文件SystemWorkMode.c结束---------------------------------------------

