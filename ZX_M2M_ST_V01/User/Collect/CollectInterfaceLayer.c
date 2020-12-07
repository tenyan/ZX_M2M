/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: CollectInterfaceLayer.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: A/D采样，开关量检测函数文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-08, , 创建本文件
 *
 */
//-----头文件调用------------------------------------------------------------

#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectModule.h"



//-----内部变量定义------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
uint16 Input_Voltage = 0;
uint8 switchr[3] = {0xaa,0xbb,0xcc};

struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;
struct STU_ADstruct STU_AD;



static STUSYSMsgBus f_KeyOFF={5, KeyOFF, 0, 0, 0, 0}; //钥匙开关ACC关闭
static STUSYSMsgBus f_KeyON={5, KeyON, 0, 0, 0, 0}; //钥匙开关ACC打开
/*
static STUSYSMsgBus f_GSM_AntennaOFF={5, GSM_AntennaOFF, 0, 0, 0, 0}; //GSM天线关闭
static STUSYSMsgBus f_GSM_AntennaON={5, GSM_AntennaON, 0, 0, 0, 0}; //GSM天线打开
*/


static STUSYSMsgBus f_BOXNormal={5, BOXNormal, 0, 0, 0, 0}; //开盒正常
static STUSYSMsgBus f_BOXAlarm ={5, BOXAlarm,  0, 0, 0, 0}; //开盒异常


static STUSYSMsgBus f_GET5V={5, GET5V, 0, 0, 0, 0}; //外部供电
static STUSYSMsgBus f_NO5V={5, NO5V, 0, 0, 0, 0}; //电池供电

//static STUSYSMsgBus f_PowerLow={5, PowerLow, 0, 0, 0, 0}; //电源馈电
//static STUSYSMsgBus f_PowerNormal={5, PowerNormal, 0, 0, 0, 0}; //电源回复正常
//static STUSYSMsgBus f_Battery_Charge={5, Battery_Charge, 0, 0, 0, 0}; //电源馈电
//static STUSYSMsgBus f_Battery_NOCharge={5, Battery_NOCharge, 0, 0, 0, 0}; //电源回复正常


//-----外部变量定义------------------------------------------------------------
extern STUSleep stuSleep;              //休眠结构体定义




/**********************************************************************************
** 函数名称: GetInput_Voltage
** 功能描述: 外部功能模块索取电瓶电压
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
uint16 GetInput_Voltage(void)
{
	return STU_Systemstate.usInput_Voltage;
}
/**********************************************************************************
** 函数名称: GetBAT_Voltage
** 功能描述: 外部功能模块索取电池电压
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 GetBAT_Voltage(void)
{
	return STU_Systemstate.ucBAT_Voltage;
}

/**********************************************************************************
** 函数名称: SaveWorktime
** 功能描述: 将32位数据分成4个8位字节,再保存在铁电里
** 输    入: uiworktime
** 输　  出: 
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void SaveWorktime(uint32 uiworktime)
{
	uint8 aucTemp[7];
	 
	aucTemp[0] = 0x12;
	aucTemp[1] = 0x34;
    aucTemp[2] = uiworktime&0xff;
    aucTemp[3] = (uiworktime>>8)&0xff;
    aucTemp[4] = (uiworktime>>16)&0xff;
    aucTemp[5] = (uiworktime>>24)&0xff;
	aucTemp[6] = SumCalc(aucTemp,6);
	
	WriteToFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);//20110303当钥匙开关由高变为低时候开始保存工作小时至存储器中
	WriteToFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
	PC_SendDebugData((uint8 *)("sw2e"), 4, DEBUG_ANYDATA);
}

//保存工作小时到RTC RAM
void SaveWorktimeToRTCRAM(uint32 uiworktime)
{
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0x12345678);
	RTC_WriteBackupRegister(RTC_BKP_DR1, STU_Systemstate.uiWorkingTime);
}
/**********************************************************************************
** 函数名称: InitWorktime
** 功能描述: 从EERPROM或RTC RAM中读取总工作小时
** 输　  出: 
** 全局变量: 
** 调用模块:
** 作　	 者 : HHM
** 日　	 期 : 2015年07 月9 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void InitWorktime()
{
	uint8 aucTemp[7];
	uint32 uiTemp;

	uiTemp = RTC_ReadBackupRegister(RTC_BKP_DR0);
	if(0x12345678!=uiTemp)//断电
	{
		ReadFromFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);
		if(aucTemp[0]==0x12 && aucTemp[1]==0x34 && aucTemp[6]==SumCalc(aucTemp,6))
		{
			STU_Systemstate.uiWorkingTime = StrToUint32(&aucTemp[2], MEM_MOD_LITLE);
		}
		else
		{
			PC_SendDebugData((uint8 *)("rd wkt1 err"), 11, DEBUG_ANYDATA);
			ReadFromFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
			if(aucTemp[0]==0x12 && aucTemp[1]==0x34 && aucTemp[6]==SumCalc(aucTemp,6))
			{
				STU_Systemstate.uiWorkingTime = StrToUint32(&aucTemp[2], MEM_MOD_LITLE);
			}
			else
			{
				PC_SendDebugData((uint8 *)("rd wkt2 err"), 11, DEBUG_ANYDATA);
				STU_Systemstate.uiWorkingTime = 0;
			}
		}
	}
	else
	{
		uiTemp = RTC_ReadBackupRegister(RTC_BKP_DR1);
		STU_Systemstate.uiWorkingTime = uiTemp;
	}
}


/**********************************************************************************
** 函数名称: CountWorkTime 工作小时统计单位为分钟
** 功能描述: 统计工作小时:当保留开关为高电平时候累计,当由高电平
**                            变为低电平时候进行存储，正常工作时候6分钟存储一次
** 输    入: 无参数
** 输　  出: 
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void CountWorkTime(void)
{
    static uint8 ucCount60s = 60;        // 一分钟计时
	uint8 acc;
	static uint8 acc_pre  = 0;
    static uint8 ucAccOffCount = 0;
	
	acc = GetAccState();
	if(acc==1)
	{
		STU_Systemstate.uiWorkingTime++;   
        if(!(--ucCount60s))
        {
            ucCount60s = 60;
			SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
        }
    }
    else
	{
	 	if(acc_pre == 1)//ACC ON----->ACC OFF
 		{
 			SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
 			ucAccOffCount = 5;
 		}
	}
    if(ucAccOffCount)
    {
        if(!(--ucAccOffCount))
        {
    		SaveWorktime(STU_Systemstate.uiWorkingTime);
		}
	}	
	acc_pre = acc;
}


/**********************************************************************************
** 函数名称: GetWorkingTime
** 功能描述: 外部功能模块索取电池电压
** 输    入: 无参数
** 输    出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
uint32 GetWorkingTime(void)
{
	return STU_Systemstate.uiWorkingTime;
}

//设置总工作时间
void SetWorkingTime(uint32 uiWorkTime)
{
	STU_Systemstate.uiWorkingTime = uiWorkTime;
	SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
 	SaveWorktime(STU_Systemstate.uiWorkingTime);
}


/**********************************************************************************
** 函数名称: CheckSwitchState
** 功能描述: 获取开关量当前状态每100ms检测一次
** 输    入: 无参数
** 输    出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void CheckSwitchState(void)
{
    static uint8 ucCountKye_Hig = 10;      	//管脚高持续1秒,Key-on为高电平
    static uint8 ucCountKye_Low = 10;        	//管脚低持续1秒,Key-on为低电平
    //static uint16 ucCountHulled_Hig = 50;	    //管脚高持续5秒,搭铁极高电平
	//static uint16 ucCountHulled_Low = 50;		//管脚低持续5秒,搭铁极低电平
    static uint16 ucCountGet5V_Hig = 50; //管脚高持续5秒,有5V输出高电平
    static uint16 ucCountGet5V_Low = 50; //管脚低持续5秒,无5V输出
    static uint8  GpsAntShot_Hig = 10;
	static uint8  GpsAntShot_Low = 10;
	static uint8  GpsAntOpen_Low = 10;
	static uint8  GpsAntOpen_Hig = 10;
//	static uint8  SIMCover_Hig=20;
//	static uint8  SIMCover_Low=20;
	
    if(ReadSwitchACC())  //判断ACC //ACC开关信号采集
	{

		ucCountKye_Hig=10;
		if(!--ucCountKye_Low)
		{
			ucCountKye_Low=10;
			STU_Systemstate.ucSwitch1 &= ~BIT(1);
		}
	}
	else
	{
	
		ucCountKye_Low=10;
		if(!--ucCountKye_Hig)
		{
			ucCountKye_Hig=10;
            STU_Systemstate.ucSwitch1 |= BIT(1);
		}
	}       
	 
	 if(STU_Systemstate.usInput_Voltage>70)    //外电供电检测
	 {
		 ucCountGet5V_Low = 50;
		 if(ucCountGet5V_Hig) 
		 {
			 if(!(--ucCountGet5V_Hig))
			 {
				 ucCountGet5V_Hig = 50;
				 STU_Systemstate.ucSwitch3 &= ~BIT(0);	// 外电供电有5V输出	
			 }
		 }
	 }
	 else  
	 {
		 ucCountGet5V_Hig = 50;
		 if(ucCountGet5V_Low)
		 {
			 if(!(--ucCountGet5V_Low))
			 {
				 ucCountGet5V_Low = 50;
				 STU_Systemstate.ucSwitch3 |= BIT(0); 	// 电池供电无5V输出
 
			 } 
		 }
	 }

 	//gps天线检测
	if(ReadGpsAntShot())
	{
		GpsAntShot_Low = 10;
		if(!(--GpsAntShot_Hig))
		{
			STU_Systemstate.DisassmbleSwitch |= BIT(5);
			GpsAntShot_Hig = 10;
		}
	}
	else
	{
		GpsAntShot_Hig = 10;
		if(!(--GpsAntShot_Low))
		{
			STU_Systemstate.DisassmbleSwitch &= ~BIT(5);
			GpsAntShot_Low = 10;
		}
	}
	if(ReadGpsAntOpen())
	{
		GpsAntOpen_Low = 10;
		if(!(--GpsAntOpen_Hig))
		{
			STU_Systemstate.DisassmbleSwitch |= BIT(6);
			GpsAntOpen_Hig = 10;
		}
	}
	else
	{
		GpsAntOpen_Hig = 10;
		if(!(--GpsAntOpen_Low))
		{
			STU_Systemstate.DisassmbleSwitch &= ~BIT(6);
			GpsAntOpen_Low = 10;
		}
	}

/*
	 //SIM卡盖检测
	if(ReadSIMCardState()) 
	{
		SIMCover_Hig=20;
		if(!--SIMCover_Low)
		{
			SIMCover_Low=20;
			STU_Systemstate.DisassmbleSwitch &= ~BIT(4);   //SIM卡正常
		}
	}
	else
	{
		SIMCover_Low=20;
		if(!--SIMCover_Hig)
		{
			SIMCover_Hig=20;
			STU_Systemstate.DisassmbleSwitch |= BIT(4);   //SIM卡拆除
		}
	}
	*/
		
}
/******************************************************************************
** 函数名称: ReportSwitchState
** 功能描述: 开关量检测有变化通知SYSTEM
** 输            入: 无参数
** 输　      出: 
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
void ReportSwitchState (void)
{
	//uint8 a[2] = {0xaa,0xaa};
	//uint8 b[2] = {0xbb,0xbb};

    /*钥匙开关量检测      */
  	if((STU_Systemstate.ucSwitch1&BIT(1))^(STU_SystemstatePre.ucSwitch1&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch1&BIT(1)) 
        {
            SYS_PutDataQ((uint8*)&f_KeyOFF);        //Key开关关
        }
        else 
        {
            SYS_PutDataQ((uint8*)&f_KeyON);         //Key开关开
        }
    }


		//外电切断状态上报
    if((STU_Systemstate.ucSwitch3&BIT(0))^(STU_SystemstatePre.ucSwitch3&BIT(0)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(0)) 
        {
            SYS_PutDataQ((uint8*)&f_NO5V);
        }
        else 
        {
           SYS_PutDataQ((uint8*)&f_GET5V); 
        }
    }

	//开盖报警状态上报
	if((STU_Systemstate.ucSwitch2 & BIT(3))^(STU_SystemstatePre.ucSwitch2 & BIT(3)))
    {
        if(STU_SystemstatePre.ucSwitch2 & BIT(3)) 
        {
            SYS_PutDataQ((uint8*)&f_BOXNormal);     //通知开盒2恢复正常          
        }
        else 
        {
            SYS_PutDataQ((uint8*)&f_BOXAlarm);      //通知开盒2变为异常    
        }
    }
	
	STU_SystemstatePre = STU_Systemstate;
} 
/******************************************************************************
** 函数名称:  GetACCState
** 功能描述:  外部函数获取钥匙开关当前状态
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetAccState()
{
	return (STU_Systemstate.ucSwitch1 & BIT(1)) ? 1 : 0;
}

/******************************************************************************
** 函数名称:  GetPwrSupplyState
** 功能描述:  获取主电源当前供电状态
** 输    入: 无参数
** 输　  出: 无参数
** 返    回: 0=外电源供电, 1=内置电池供电
** 作　	 者 : hhm
** 日　	 期 : 2016-7-6
*******************************************************************************/
uint8 GetPwrSupplyState()
{
	return (STU_Systemstate.ucSwitch3 & BIT(0)) ? 1 : 0;
}

/******************************************************************************
** 函数名称:  GetPwrLowState
** 功能描述:  获取主电源馈电状态
** 输    入: 无参数
** 输　  出: 无参数
** 返    回: 0=外电源不馈电, 1=外电源馈电
** 作　	 者 : hhm
** 日　	 期 : 2016-7-6
*******************************************************************************/
uint8 GetPwrLowState()
{
	return (STU_Systemstate.ucSwitch3 & BIT(1)) ? 1 : 0;
}

/******************************************************************************
** 函数名称:  GetBoxOpenState
** 功能描述:  获取开盖状态
** 输    入: 无参数
** 输　  出: 无参数
** 返    回: 0=开盖正常, 1=开盖报警
** 作　	 者 : hhm
** 日　	 期 : 2016-7-6
*******************************************************************************/
uint8 GetBoxOpenState()
{
	return (STU_Systemstate.ucSwitch2 & BIT(3))?1 : 0;
}

uint8 READ_OPENBOX2()
{
	return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
}
/******************************************************************************
** 函数名称:  CheckBoxState
** 功能描述:  获取开盒开关量当前状态每100ms检测一次
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
void CheckBoxState(void)
{
    static uint8 ucCountBOX2Normal = 10;      //开盒开关2正常
    static uint8 ucCountBOX2Alarm = 10;       //开盒开关2报警

 
    if(READ_OPENBOX2())
    {
   		 ucCountBOX2Alarm = 20;
		 if(ucCountBOX2Normal)
		 {
			 if(!(--ucCountBOX2Normal))
			 {
				 ucCountBOX2Normal = 20;
				 STU_Systemstate.ucSwitch2 &= ~BIT(3);   //开盒开关2正常 
			 }
		 }
    }
    else    
    {
   		 ucCountBOX2Normal = 20;
		 if(ucCountBOX2Alarm)
		 {
			 if(!(--ucCountBOX2Alarm))
			 {
				 ucCountBOX2Alarm = 20;
				 STU_Systemstate.ucSwitch2 |=  BIT(3);  //开盒开关2报警  
			 }
		 }
    }
 
}



/******************************************************************************
** 函数名称:  GetSwitch1State
** 功能描述:  外部函数获取开关量1当前状态
**            B0：报警开关    0=低电平,1=高电平；
**			  B1：钥匙开关ACC    0=关闭,1=打开；
**            B2：小时计开关 0=关闭,1=打开；
**            B3：外接电正极状态     0=断开,1=联通；
**            B4：外接电负极状态     0=联通,1=断开；
**            B5：搭铁极状态     0=联通,1=断开；
**            B6：GSM天线报警   0=无报警, 1=报警
**            B7：保留位
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch1State(void)
{
	return STU_Systemstate.ucSwitch1;
}

/******************************************************************************
** 函数名称:  GetSwitch2State
** 功能描述:  外部函数获取开关量2当前状态
**            B0：继电器1控制状态     0=未锁车;1=锁车
**            B1：继电器2控制状态     0=未锁车;1=锁车
**            B2：开盒报警1     0=正常;1=报警
**            B3：开盒报警2   0=正常;1=报警
**            B4：保留
**            B5：RS232通讯状态    0= 未通讯；1=通讯；
**            B6：RS232通讯仪表提示状态    0= 未通讯；1=通讯；
**            B7：CAN通讯状态    0= 未通讯；1=通讯；
** 输        入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch2State(void)
{
	return STU_Systemstate.ucSwitch2;
}

/******************************************************************************
** 函数名称:  GetSwitch3State
** 功能描述:  开关量采集3
**			  B0：供电状态 0=外电供电；1=电池供电；
**			  B1：外电馈电状态   0=不馈电；1=馈电
**			  B2：内部电池充电状态    0=未充电,1=充电；
**			  B3：车辆行驶速度报警 0=速度正常；1=超速报警；
**			  B4：车辆位置越界报警 0=不越界；1= 越界报警。
**			  B5：保留
**			  B6：保留
**			  B7：保留
** 输        入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch3State(void)
{
	return STU_Systemstate.ucSwitch3;
}


//获取GPS 开关量检测状态，
//返回值:	0=正常,1=异常
uint8 GetGpsSwitchState()
{
	if(!(STU_Systemstate.DisassmbleSwitch&BIT(5)) ||(STU_Systemstate.DisassmbleSwitch&BIT(6)))//报警
		return 1;
	else
		return 0;
}

/*****************************************************************************
 ** 函数名称: GetGpsAntShortState
 ** 功能描述: 获取GPS天线短路状态
 ** 输	  入: 无
 ** 输	  出: 无
 ** 返	  回: GPS天线短路状态, 1=短路,0=正常
 ** 作　  者 : hhm
 ** 日　  期 : 2016-7-6
 ******************************************************************************/
uint8 GetGpsAntShortState()
{
	return (!(STU_Systemstate.DisassmbleSwitch&BIT(5)))?1:0;
}

/*****************************************************************************
 ** 函数名称: GetGpsAntOpenState
 ** 功能描述: 获取GPS天线开路状态
 ** 输	  入: 无
 ** 输	  出: 无
 ** 返	  回: GPS天线开路状态, 1=开路,0=正常
 ** 作　  者 : hhm
 ** 日　  期 : 2016-7-6
 ******************************************************************************/
uint8 GetGpsAntOpenState()
{
	return (STU_Systemstate.DisassmbleSwitch&BIT(6))?1:0;
}

/*****************************************************************************
 ** 函数名称: GetBatChargeState
 ** 功能描述: 获取电池充电状态
 ** 输	  入: 无
 ** 输	  出: 无
 ** 返	  回: 电池充电状态, 1=充电,0=未充电
 ** 作　  者 : hhm
 ** 日　  期 : 2016-8-19
 ******************************************************************************/
uint8 GetBatChargeState(void)
{
	return (STU_Systemstate.ucSwitch3&BIT(2))?1:0;
}

/*****************************************************************************
 ** 函数名称: GetBatVolLowState
 ** 功能描述: 获取电池电压低状态
 ** 输	  入: 无
 ** 输	  出: 无
 ** 返	  回: 电池电压低状态, 1=电压低,0=电压正常
 ** 作　  者 : hhm
 ** 日　  期 : 2016-8-19
 ******************************************************************************/
uint8 GetBatVolLowState(void)
{
	return (STU_Systemstate.ucSwitch3&BIT(5))?1:0;
}


/******************************************************************************
** 函数名称: InputVoltageCheck
** 功能描述: 对外部电压的处理
**           如果PWR_STA为低电平，则认为外电供电，如果PWR_STA为
             高电平，  则电池供电，认定为电池供电;如果外部电压
**           大于9V但是小于雨设置低电压则报警:电源电压低

** 输    入: 无参数
** 输    出: 
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
void InputVoltageCheck(void)
{
	static uint16 usInputVolLowTime = 5;              // 外部电压持续低于报警电压时间暂定为5秒
	
    //if(STU_Systemstate.usInput_Voltage<=g_stuSYSParamSet.ucVoltageLowAlarm)   
    if(STU_Systemstate.usInput_Voltage<=220)
    {
        if(usInputVolLowTime)
        {
            if(!(--usInputVolLowTime))
            {
                STU_Systemstate.ucSwitch3|=BIT(1);      // 外部电压低
              // 	SYS_PutDataQ((uint8*)&f_PowerLow);      // 电源电压低        
            }
        }
    }
    else
    {
        usInputVolLowTime = 5;                        // 暂定为30秒
        if(STU_Systemstate.ucSwitch3&BIT(1))
        {
            STU_Systemstate.ucSwitch3&=~BIT(1);         // 外部电压正常
			//SYS_PutDataQ((uint8*)&f_PowerNormal);
        }           
    }
    
}

/******************************************************************************
** 函数名称: BatVoltageCheck
** 功能描述: 对內部电池电压的处理,电压低于3.5V持续5秒认为电压低
** 输    入: 无参数
** 输    出: 
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2011年07 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
void BatVoltageCheck(void)
{
	static uint16 ucBVLT = 5;              // 电压持续低于3.5报警电压时间暂定为5s
	static uint16 ucBVHT = 5;              // 电压持续高于3.5报警电压时间暂定为5s

	if(STU_Systemstate.ucBAT_Voltage<35)     
    {
    	ucBVHT = 5;
        if(ucBVLT>0)
        {
            if(!(--ucBVLT))
            {
                STU_Systemstate.ucSwitch3 |= BIT(5);      // 外部电压低
            }
        }
    }
    else
    {
    	ucBVLT = 5;
        if(ucBVHT>0)
        {
            if(!(--ucBVHT))
            {
                STU_Systemstate.ucSwitch3 &= ~BIT(5);      // 外部电压低
            }
        }     
    }
}


 /*****************************************************************************
 ** 函数名称: BatContronl
 ** 功能描述: 只有ACC或者发动机开关为高时才能对电池充电
 **
 ** 输	  入: 无
 ** 输	  出: 无
 ** 返	  回: 
 **
 ** 作　  者 : 
 ** 日　  期 : 2011年07 月19 日

 **----------------------------------------------------------------------------
 ******************************************************************************/
 void BatContronl(void)
 {
     static uint16 usTimer=0;                      
	 
	 if(!(STU_Systemstate.ucSwitch3 & BIT(0))) 
	 {
	 //	 if(STU_Systemstate.ucSwitch1 & BIT(1))
	 //	 {
		     usTimer = 0;
			 if(STU_Systemstate.ucBAT_Voltage<41)
			 {
				 ChargeOn(); 					//给电池充电
				 STU_Systemstate.ucSwitch3 |= BIT(2);	  
			 }
			 if(STU_Systemstate.ucBAT_Voltage>41)
			 {
				 ChargeOff(); 				   	//关闭电池充电
				 STU_Systemstate.ucSwitch3 &= ~BIT(2);
			 }
	// 	 }
	 }
	 else										//电池供电
	 {
     	if(STU_Systemstate.ucBAT_Voltage<35) //电池保护
     	{
     	    if(usTimer++>30)
     	    {
             	usTimer = 0;		 	
    		 	g_stuSystem.ucShutDownTime = 10;
     	    }
     	}
		else
		{
            usTimer = 0;
		}
		 
		ChargeOff(); 					   //关闭充电管脚
		STU_Systemstate.ucSwitch3 &= ~BIT(2);
	 }
	 
 }


 void WatchDogHard(void)
 { 
/*	 if((FIO2PIN&BIT(2)))
		 WDT_LOW();
	 else
		 WDT_HIGH();
		 */
 }



uint16 ADC2ConvertedValue;
uint16 ADC3ConvertedValue;
void ADInit(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC3->DR); //ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* DMA2 Stream0 channel3 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC2->DR); //ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC2ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream3, ENABLE);
  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);
  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_112Cycles);
 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC3);

   /* ADC2 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_112Cycles);
 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  /* Enable ADC2 DMA */
  ADC_DMACmd(ADC2, ENABLE);
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  ADC_SoftwareStartConv(ADC2);
}


/**********************************************************************************
** 函数名称: ADConvert
** 功能描述: AD转换，采用通道0和1
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/

void  ADConvert(void)
{
    static uint8 i=0;

	STU_AD.usVBAT[i]          = ADC2ConvertedValue;	//V_BAT
	STU_AD.usInput_Voltage[i] = ADC3ConvertedValue;	//V_POWER

	i++;
	if(i>=10)
		i = 0;
}



/**********************************************************************************
** 函数名称: DealVoltage
** 功能描述: 外部电压和电池电压处理，采集10 求平均值
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2011年07 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/

void  DealVoltage(void)
{
    uint32 uifTemp;
   
    uifTemp = DigitFilter1(STU_AD.usInput_Voltage, 10, 2);
    STU_Systemstate.usInput_Voltage = (uint16)(uifTemp*0.09659);//真正外部电压值
    if(STU_Systemstate.usInput_Voltage < 50) //因为外电切断后采集电压处仍有3V左右电压,所以当采集到外电压小于5V时认为外电切断赋值为0 
    {
		STU_Systemstate.usInput_Voltage = 0;
    }
    uifTemp = DigitFilter1(STU_AD.usVBAT, 10, 2);//简易数字滤波器，先排序，去掉最大和最小值，剩下取平均值
    STU_Systemstate.ucBAT_Voltage = (uint8)(uifTemp*0.02578);//真正电池电压值

}


uint8 BuildAD_Switch(uint8 *buf)
{
    uint16 usTemp = 0;
	
	usTemp = GetInput_Voltage();
    *buf++ = (usTemp>>8) & 0xff;
	*buf++ =  usTemp & 0xff;
	*buf++ = GetBAT_Voltage();
	*buf++ = GetSwitch1State();
	*buf++ = GetSwitch2State();
	*buf++ = GetSwitch3State();
	if(0xaa==g_stuSystem.ucWDTState)
		STU_Systemstate.DisassmbleSwitch |= BIT(0);
	else
		STU_Systemstate.DisassmbleSwitch &= ~BIT(0);
	*buf++ = STU_Systemstate.DisassmbleSwitch;
	*buf++ = stuSleep.ucWorkingMode;
	*buf++ = HW_VERSION&0xFF;      //存放协处理器的硬件版本号
	*buf++ = (uint8)(HW_VERSION>>8)&0xFF;

	return 10;	
}
#if 0
//通过4G模块传送的定位状态进行判断
//天线异常报警
void GpsAntenaCheck()
{
	static uint8  AntennaOkTime;	//天线正常状态计次
	static uint8  AntennaErrTime;	//天线故障状态计次
	static  uint8   ucAntennaState=1;   //天线正常异常锁:0正常,1异常

	if (m_stuOrient.ucGpsState&BIT(1)) // B1=1 GPS定位,0 GPS不定位
	{
		m_stuOrient.ucGpsState&=~BIT(0); 
	}
	else	//不定位则判断天线状态
	{
		if (m_stuOrient.ucGpsState & BIT(0))    //现在天线异常
		{
			if(!GetGpsSwitchState())			//天线及时恢复正常
			{
				AntennaOkTime++;
				AntennaErrTime=0;
				if (AntennaOkTime>Antenna_ok_time)
				{
//					GPS_Antenna_Ok=TRUE;			
					m_stuOrient.ucGpsState &= ~BIT(0); 
					AntennaOkTime=0;
				}
			}
			else 		//天线仍然故障	
			{
				AntennaOkTime=0;
			}
		}
		else		//现在状态是正常
		{
			if(!GetGpsSwitchState())              //天线仍然正常
			{
				AntennaErrTime=0;
			}
			else     //天线异常
			{
				AntennaErrTime++;
				AntennaOkTime=0;
				if (AntennaErrTime>Antenna_err_time)
				{
					m_stuOrient.ucGpsState|=BIT(0); 
					AntennaErrTime=0;
				}
			}
		}
	}    

	/*
	if(m_stuOrient.ucGpsState & BIT(0))                          
	{
	    if(0==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA WARNING\n", 16, DEBUG_GPSMODULE);
		    f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_ERROR;
		    SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 1;            //天线异常标识
    	}
	}		
	else                                       //清零天线异常计数器
	{
	    if(1==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA OK\n", 11, DEBUG_GPSMODULE);
			f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_OK;
			SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 0;
    	}
	}
	*/
}
#endif


/*-----文件CollectInterfaceLayer.c结束-----*/
