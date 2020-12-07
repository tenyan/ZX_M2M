/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved 
 * 
 * 文件名称: GpsIntergrateLayer.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为GPS功能模块综合层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by  创建本文件
 *
 */
#include "config.h"
#include "GpsModule.h"
#include "GpsHardWareLayer.h"
#include "GpsProtocolLayer.h"
#include "GpsIntegrateLayer.h"
#include "GpsInterfaceLayer.h"

//-----外部变量定义------------------------------------------------------------
extern STU_SSData SSData;
extern uint16_t UART4Timer;
extern uint8_t UART4Buffer[];
extern uint8 GPS_Recv_Buff[GPS_RECEIVE_BUFF_LEN];
extern STUSystem g_stuSystem;
extern uint8 Speedlen ;
//-----内部变量定义------------------------------------------------------------
STU_Orient m_stuOrient;
STU_Gps GPS;

//-----内部函数声明------------------------------------------------------------

static void  ResetGps(void);

//-----外部函数实现------------------------------------------------------------
/******************************************************************************
** 函数名称: pthread_Gps_Function
** 功能描述: 读取、处理Gps模块数据线程,阻塞方式
** 
** 输    入: data
** 输    出: 无
** 返    回: 无
** 
** 作    者: lxf
** 日    期: 2019-03-25
**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_Gps_Function(void* data)
{

	uint16 usLen = 0;  //缓冲区接收到的数据长度    
	uint8 *p=NULL;
  
    InitGps(0);

	while(1)
    {       
     
        if(ReadGpsUartData(&p, &usLen))
        {     
           PC_SendDebugData(p,usLen,DEBUG_GPSMODULE);

           GPS.RecvBuffLen = usLen;
		   memcpy(GPS.RecvBuff,p,usLen);
		   usLen = 0;
        //   PrintTime();
		//   printf("read: %d , %s\n", GPS.RecvBuffLen,GPS.RecvBuff);
           m_stuOrient.ucGpsState &= ~BIT(6);           //GPS模块正常
           GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
           ReadFromGpsSerialBuff(GPS.RecvBuff, GPS.RecvBuffLen);
           ParseNMEA(); 		   
        }
		else
		{
          //  usleep(20*One_MilliSecond);
		}
      //  usleep(20*One_MilliSecond);
    }
}

void GPS_Function(uint8 *buff,uint16 uslen)
{
    PC_SendDebugData(buff,uslen,DEBUG_GPSMODULE);
    m_stuOrient.ucGpsState &= ~BIT(6);           //GPS模块正常
    GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
    ReadFromGpsSerialBuff(buff,uslen);
    ParseNMEA(); 
}

//-----内部函数实现------------------------------------------------------------

void GpsModuleErrCheck()
{
	if(0==GPS.SleepState)
	{
		if(0==GPS.RecvTimeOut--) 
		{
			GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
		    m_stuOrient.ucGpsState |= BIT(6);    //将模块置为异常
			m_stuOrient.ucGpsState &= ~BIT(1);   //GPS强制不定位
	    	ResetGps();                          //重启GPS模块	    	    
	    } 
	}
}
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
	
	if(m_stuOrient.ucGpsState & BIT(0))                          
	{
	    if(0==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA WARNING\n", 16, DEBUG_GPSMODULE);
		//    f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_ERROR;
		//    SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 1;            //天线异常标识
			SSData.usTimer = 0;
    	}
	}		
	else                                       //清零天线异常计数器
	{
	    if(1==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA OK\n", 11, DEBUG_GPSMODULE);
		//	f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_OK;
		//	SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 0;
		    SSData.usTimer = 0;
    	}
	}
}

//超速报警
void SpeedOverAlarm()
{
	static  uint8   ucSpeedOverTime=0;    //超速时间
	static  uint8   ucSpeedNormalTime=0;  //速度回归时间
	static  uint8   ucSpeed = 0;        //车辆超速锁:0不超速,1超速
	
	if(m_stuOrient.ucGpsState & BIT(1))
	{
	    if(0 != SYS_GetParam()->ucSpeedOver)        //为0时,不进行超速报警
		{
			if(m_stuOrient.usSpeed > (10*SYS_GetParam()->ucSpeedOver)) //超速报警
			{
				ucSpeedNormalTime = 0;
			    if(0==ucSpeed && ucSpeedOverTime++>5)
		    	{
		    	    ucSpeedOverTime = 0;
			    	ucSpeed = 1;
					m_stuOrient.ucGpsState |= BIT(7);
					PC_SendDebugData((uint8*)"OVERSPEED WARNING\n", 18, DEBUG_GPSMODULE);
				   // f_stuGpsWarning.ucKind = GPS_MSG_WARN_SPEED_VOER;
				  //  SYS_PutDataQ((void *)&f_stuGpsWarning);	
				    SSData.usTimer = 0;
		    	}
			}
			else                                                  //解除报警
			{
				ucSpeedOverTime = 0;
			    if(1==ucSpeed && ucSpeedNormalTime++>5) 
		    	{
		    	    ucSpeedNormalTime=0;
			    	ucSpeed = 0;
					m_stuOrient.ucGpsState &= ~BIT(7);
					//f_stuGpsWarning.ucKind = GPS_MSG_WARN_SPEED_OK;
				   // SYS_PutDataQ((void *)&f_stuGpsWarning);	
				    SSData.usTimer = 0;
		    	}
			}
		}
	}
}
/******************************************************************************
** 函数名称: ResetGps
** 功能描述: 重启GPS模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: 
** 日    期: 2019-03-28
**-----------------------------------------------------------------------------
** 修改记录
**-----------------------------------------------------------------------------
** 2019-03-07  初始版本
*******************************************************************************/
void ResetGps()
{
	GpsPower_CLR();
	OSTimeDly(200);
	GpsPower_SET();
	OSTimeDly(200);
	OSTimeDly(200);
	GPSRMCGGA();
	OSTimeDly(OS_TICKS_PER_SEC*2);
	PC_SendDebugData((uint8 *)("ResetGps"), 8, DEBUG_ANYDATA);
}
//容许延迟的定时执行函数:10ms
void GPS_TimerCount_Delay()
{
	static uint8 Cnt1s=0;
	
	if(Cnt1s)
		Cnt1s--;
	else
	{
		Cnt1s = 100;
		GpsModuleErrCheck();
		ReInitGps();
	}
}
//不容许延迟的定时执行函数:10ms
void GPS_TimerCount_NoDelay()
{
	static uint8 Cnt1s=0;
	
	if(Cnt1s)
		Cnt1s--;
	else
	{
		Cnt1s = 100;
		GpsAntenaCheck();
		SpeedOverAlarm();
	//	SaveDistanceToEEPROM_AccOff();
	}
}
//-----文件GpsIntegrateLayer.c结束---------------------------------------------


