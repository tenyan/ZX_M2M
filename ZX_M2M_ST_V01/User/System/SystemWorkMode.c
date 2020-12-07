/*
 * Copyright(c)2019, 硬件研发部
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
 * 2019-05-05, by  lxf, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "SystemWorkMode.h"
#include "RTC.h"
#include "CollectInterfaceLayer.h"
//-----外部变量定义------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern uint8   g_ucRstCount;            //梯度重启计数器
extern uint16  g_usRstTime;          //梯度重启时间间隔
extern stuFirmwareUpdate FirmwareUpdate;
extern uint8 ucOpenTimes;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
//-----内部变量定义------------------------------------------------------------


//-----内部函数声明------------------------------------------------------------
STUSleep stuSleep;              //休眠结构体定义
extern uint16 ADC3ConvertedValue;
extern STU_SSData SSData;
extern STU_GSM_State g_stuGsmState;
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
	OSTimeDly(OS_TICKS_PER_SEC * 3); //防止主处理器升级时设备进入休眠的延时时间  
	g_stuGsmState.ucRunState = 4;
	OSTimeDly(OS_TICKS_PER_SEC * 10);
 
}

void SYS_GSMWake(void)
{
    stuSleep.ucGsmSleepState = 0;     //GSm休眠唤醒标志
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

    //对ACC进行判断 防止中断无法正常唤醒
	if(!ReadSwitchACC())     //ACC 为高
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
	
    //对外电源电压进行判断 达到启动电压是唤醒
    for(i=0;i<20;i++)
    {
    	ADtemp[i] = ADC3ConvertedValue;
	}
	uifTemp = DigitFilter1(ADtemp, 20, 2);
    usInputVoltage = (uint16)(uifTemp*0.09659); //真正外部电压值
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
//计时
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
** 函数名称: CPU_PowerDown
** 功能描述: CPU进入掉电休眠函数
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2016-06-30
**-----------------------------------------------------------------------------
*******************************************************************************/
void CPU_PowerDown(void)
{
	 //关闭外围用电设备
    Mcu_CanClose();          //关闭CAN模块电源
    ChargeOff(); 		     //关闭电池充电,防止休眠前没有关闭
    SYS_GPSSleep();
    SYS_GSMSleep();
	if((0==GetPwrSupplyState()) && (0==GetPwrLowState()))//主电供电且主电不馈电
	{
//		SYS_GSMSleep();
		OSTimeDly(OS_TICKS_PER_SEC * 10);
		
		stuSleep.usSleepTimes = g_stuSYSParamSet.usSleepWakeSlot*60;
		while(stuSleep.usSleepTimes)
		{
			IWdtFeed();
			stuSleep.usSleepTimes--;
			MainPowerCheck();
			RTCSetSleepTime(1, 1);    //1秒钟唤醒一次			
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
		    stuSleep.usWakeLastTime = WEAKLAST_TIME;   //唤醒后维持运行5分钟
		    SYS_GPSWake();
		    SYS_GSMWake();            
		}
	}
	else	//内置电池供电或主电馈电
	{
		SYS_GSMPwrofff();
		OSTimeDly(OS_TICKS_PER_SEC * 10); 
		PSWControl(1);
		RTCSetSleepTime(86398, 2);   //24小时
		MainPowOff();
		OSTimeDly(OS_TICKS_PER_SEC*3);
		stuSleep.ucReadRTCClockFlag = 1;
	//	BatPowOff();
	}
   // BLU_LED_OFF();    //关闭电源指示灯       
}

void SYS_ResetSleeptime(void)
{
	stuSleep.usSleepTimes = 0;
}
/******************************************************************************
** 函数名称: SYS_WorkModeSleep_Count
** 功能描述: 对终端休眠前时间变量,唤醒维持时间变量进行计时函数,(该函数每秒执行一次)
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-08-05
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_WorkModeSleep_Count(void)
{

   // if(FirmwareUpdate.ucStep>0)      //远程升级时不进入休眠
    if(stu_ExtMCUUpgrade.ucUpgradeStep>0||FirmwareUpdate.ucMainCPuUpgradeFlag>0)
        return;
	//休眠期间模块全部唤醒后计时5分钟
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
		//ACC OFF 后进入休眠前计时
        if(stuSleep.usSleepBeforSlot)
        {
            if(!(--stuSleep.usSleepBeforSlot)) 
            {
                stuSleep.ucFlag|=BIT(2);
            }
        }
    }
    else            //检测到车辆正在工作
    {
	    stuSleep.usWakeLastTime=0;
        stuSleep.ucFlag = 0;
        stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //唤醒后重新赋值		
    }

	if(stuSleep.ucWork_Heart_Count)
    {
        if(!(--stuSleep.ucWork_Heart_Count))  
			stuSleep.ucWork_heart_Flag = 1;
    }  

}

/******************************************************************************
** 函数名称: SYS_WorkModeInit
** 功能描述: 终端工作模式初始化
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-06-30
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
** 日    期: 2011-07-01
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
	//	BatPowOn();
		MainPowOn();
		PSWControl(0);
		am_disable_alm();
		stuSleep.ucWorkingMode = WORK_MODEL_NORMAL;
		stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;  //唤醒后重新赋值
		return;            
	}
	
  //  if(FirmwareUpdate.ucStep>0)      //远程升级时不进入休眠
    if(stu_ExtMCUUpgrade.ucUpgradeStep>0||FirmwareUpdate.ucMainCPuUpgradeFlag>0)
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
		//		g_stuGsmState.ucRunState = 4;
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
//-----文件SystemWorkMode.c结束---------------------------------------------

