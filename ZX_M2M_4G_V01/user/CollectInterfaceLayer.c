/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
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
 * 202-11-08, lxf创建本文件
 *
 */
//-----头文件调用------------------------------------------------------------

#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectModule.h"

//-----外部变量定义------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern STU_SSData SSData;
//-----内部变量定义------------------------------------------------------------


struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;



/**********************************************************************************
** 函数名称: GetInput_Voltage
** 功能描述: 外部功能模块索取电瓶电压
** 输            入: 无参数
** 输　      出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者 : 
** 日　	 期 : 2019年07 月19 日

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
** 日　	 期 : 2019年03 月19 日

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
** 日　	 期 : 2019年03 月19 日

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
	
	//WriteToFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);//20110303当钥匙开关由高变为低时候开始保存工作小时至存储器中
	//WriteToFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
	PC_SendDebugData((uint8 *)("sw2e"), 4, DEBUG_ANYDATA);
}

//保存工作小时到RTC RAM
void SaveWorktimeToRTCRAM(uint32 uiworktime)
{
//	RTC_WriteBackupRegister(RTC_BKP_DR0, 0x12345678);
//	RTC_WriteBackupRegister(RTC_BKP_DR1, STU_Systemstate.uiWorkingTime);
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
#if 0
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
	#endif
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
** 日　	 期 : 2019年03 月19 日

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
** 日　	 期: 2019年03 月19 日

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


/******************************************************************************
** 函数名称: ReportSwitchState
** 功能描述: 开关量检测有变化通知SYSTEM
** 输    入: 无参数
** 输　  出: 
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2019年03 月19 日

**-----------------------------------------------------------------------------
*******************************************************************************/
void ReportSwitchState (void)
{
    uint8 ucFlag = 0;     //1=触发状态数据上报

    /*钥匙开关量检测      */
  	if((STU_Systemstate.ucSwitch1&BIT(1))^(STU_SystemstatePre.ucSwitch1&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch1&BIT(1)) 
        {
           // SYS_PutDataQ((uint8*)&f_KeyOFF);        //Key开关关
           ucFlag = 1;
        }
        else 
        {
         //   SYS_PutDataQ((uint8*)&f_KeyON);         //Key开关开
           ucFlag = 1;
        }
    }


		//外电切断状态上报
    if((STU_Systemstate.ucSwitch3&BIT(0))^(STU_SystemstatePre.ucSwitch3&BIT(0)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(0)) 
        {
         //   SYS_PutDataQ((uint8*)&f_NO5V);
             ucFlag = 1;
        }
        else 
        {
         //  SYS_PutDataQ((uint8*)&f_GET5V); 
             ucFlag = 1;
        }
    }
		//外电馈电状态上报
    if((STU_Systemstate.ucSwitch3&BIT(1))^(STU_SystemstatePre.ucSwitch3&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(1)) 
        {
             ucFlag = 1;
        }
        else 
        {
             ucFlag = 1;
        }
    }

	/*
	//开盖报警状态上报
	if((STU_Systemstate.ucSwitch2 & BIT(3))^(STU_SystemstatePre.ucSwitch2 & BIT(3)))
    {
        if(STU_SystemstatePre.ucSwitch2 & BIT(3)) 
        {
          //  SYS_PutDataQ((uint8*)&f_BOXNormal);     //通知开盒2恢复正常   
            ucFlag = 1;
        }
        else 
        {
         //   SYS_PutDataQ((uint8*)&f_BOXAlarm);      //通知开盒2变为异常    
            ucFlag = 1;
        }
    }
	*/
	if(ucFlag)
	{
		SSData.usTimer = 0;
		ucFlag = 0;
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
** 日　	 期 : 2019年03 月19 日

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
** 日　	 期 : 2019年03 月19 日

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
** 日　	 期 : 2019年03月19 日

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
** 日　	 期 : 2019年03 月19 日

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

#if 0

/**********************************************************************************
** 函数名称: GetBatDate
** 功能描述: 电池电压采集
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2019年03 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
int GetBatDate(void)
{
	int fd_bat,fd_scale;
	int ret;
    char rx_buf[128] = "";
    int ad_int,ad_data;
	double scale_double;
	

     fd_bat = open(DEV_TTY_BAT, O_RDWR|O_NOCTTY);
	 if (fd_bat < 0)
	 {
		//  printf("ERROR open %s ret=%d\n\r", DEV_TTY_BAT, fd_bat);
	//	 printf("ERROR open\n");
		  return 0;
	 }
	  ret =read(fd_bat, rx_buf, 127);//读原始转换值
	  //printf("ad=%s  fd=%d\n",rx_buf,fd_bat);
      ad_int = atoi(rx_buf);          
            
	  fd_scale = open(DEV_TTY_SCALE, O_RDWR|O_NOCTTY);
	  if (fd_scale < 0)
	  {
		//  printf("ERROR open %s ret=%d\n\r", DEV_TTY_SCALE, fd_scale);
	//	  printf("ERROR open\n");
		  return 0;
	  }
	   ret =read(fd_scale, rx_buf, 127);//读系数
	   //printf("SCALE=%s  fd=%d\n",rx_buf,fd_scale);			
       scale_double = atof(rx_buf); 
            
       ad_data=(int)(ad_int*scale_double);
	   //printf("ad data=%d\n",ad_data);
			
       close(fd_bat);
       close(fd_scale);

	return ad_data;
}

/**********************************************************************************
** 函数名称: GetInputDate
** 功能描述: 电池电压采集
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2019年03 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
int GetInputDate(void)
{
	int fd_input,fd_scale;
	int ret;
    char rx_buf[128] = "";
    int ad_int,ad_data;
	double scale_double;
	

     fd_input = open(DEV_TTY_INPUT, O_RDWR|O_NOCTTY);
	 if (fd_input < 0)
	 {
		 // printf("ERROR open %s ret=%d\n\r", DEV_TTY_INPUT, fd_input);
		//  printf("ERROR open\n");
		  return 0;
	 }
	  ret =read(fd_input, rx_buf, 127);//读原始转换值
	  //printf("ad=%s  fd=%d\n",rx_buf,fd_bat);
      ad_int = atoi(rx_buf);          
            
	  fd_scale = open(DEV_TTY_SCALE, O_RDWR|O_NOCTTY);
	  if (fd_scale < 0)
	  {
		 // printf("ERROR open %s ret=%d\n\r", DEV_TTY_SCALE, fd_scale);
		//  printf("ERROR open\n");
		  return 0;
	  }
	   ret =read(fd_scale, rx_buf, 127);//读系数
	   //printf("SCALE=%s  fd=%d\n",rx_buf,fd_scale);			
       scale_double = atof(rx_buf); 
            
       ad_data=(int)(ad_int*scale_double);
	   //printf("ad data=%d\n",ad_data);
			
       close(fd_input);
       close(fd_scale);

	return ad_data;
}
/**********************************************************************************
** 函数名称: ADConvert
** 功能描述: AD转换，采用通道0和1
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2019年03 月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void  ADConvert(void)
{
    static uint8 i=0;

	STU_AD.usVBAT[i]          = (uint16)GetBatDate();//ADC2ConvertedValue;	    //V_BAT
	usleep(10);
	STU_AD.usInput_Voltage[i] = (uint16)GetInputDate();//ADC3ConvertedValue;	//V_POWER

	i++;
	if(i>=10)
		i = 0;
}

/**********************************************************************************
** 函数名称: DealVoltage
** 功能描述: 外部电压和电池电压处理，采集10 求平均值
** 输    入: 无参数
** 输　  出: 无参数
** 全局变量: 
** 调用模块:
** 作　	 者: 
** 日　	 期: 2013年03月19 日

**--------------------------------------------------------------------------------
*********************************************************************************/
void  DealVoltage(void)
{
    uint32 uifTemp;
   
    uifTemp = DigitFilter1(STU_AD.usInput_Voltage, 10, 2);
    STU_Systemstate.usInput_Voltage = (uint16)(uifTemp*0.11989);//真正外部电压值
    if(STU_Systemstate.usInput_Voltage < 50) //因为外电切断后采集电压处仍有3V左右电压,所以当采集到外电压小于5V时认为外电切断赋值为0 
    {
		STU_Systemstate.usInput_Voltage = 0;
    }
    uifTemp = DigitFilter1(STU_AD.usVBAT, 10, 2);//简易数字滤波器，先排序，去掉最大和最小值，剩下取平均值
    STU_Systemstate.ucBAT_Voltage = (uint8)(uifTemp*0.032);//真正电池电压值

}
#endif
/*-----文件CollectInterfaceLayer.c结束-----*/
