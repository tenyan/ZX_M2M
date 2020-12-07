//文件名：Mcu_Can.h

#ifndef _MCU_CAN_
#define _MCU_CAN_

#ifdef  __cplusplus
extern "C" {
#endif

#include "config.h"
#include "McuHW.h"

STU_CommState m_stuCommState = {
	.ucCan1RcvErr = 0,					//Can1接收数据错误,0:没有收到数据,1:收到数据
	.usCan1RcvErrTime = 0,				//Can1没有收到数据计时	
	.ucCan1CommState = 0,					//Can1接收数据状态通信状态，1:通信异常,0:通信正常
	.ucRcvCan1DataFlag = 2,				//Can1接收数据状态 1:收到了CAN1数据，2:没有收到CAN1数据

	.ucCan2RcvErr = 0,					//Can2接收数据错误,0:没有收到数据,1:收到数据
	.usCan2RcvErrTime = 0,				//Can2没有收到数据计时	
	.ucCan2CommState = 1,					//Can2通信状态，1:通信异常,0:通信正常
	.ucRcvCan2DataFlag = 2,				//Can2接收数据状态 1:收到了CAN1数据，2:没有收到CAN1数据
};
STU_Date McuDataTime;					//数据采集的时间
uint16  McuDataTime_ms;					//数据采集时间的ms部分
STU_CanFrame CanData[MAX_CAN_FRAME_NUM];//CAN数据缓存 
STU_McuCmd m_stuMcuCmd = {
	.usSpeedLimit= 3500,
    .ucCmdCheckFlag = 0,
	
};

stuXWFaultCode g_stuXWFaultCodeUp,g_stuXWFaultCodeRecv; 
faultCode m_stuActiveFaultCodeList[MAX_ACTIVE_FAULTCODE_NUM];//当前激活的所有故障码列表，最多20个，去激活的故障码用0填充
static uint8 f_ucNewFaultCodeFlag;		//产生了新的DTC标志,0=无,1=有 
extern STUSYSParamSet g_stuSYSParamSet;
extern struct STU_Sysstruct STU_Systemstate;
extern STUMCUFirmware stu_McuFirmware;
extern stuFirmwareUpdate FirmwareUpdate;
extern STU_A5Comm stuA5Comm;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;


static  const  uint16  tbl[256]={ 
0x0000,  0x1021, 0x2042,  0x3063,  0x4084,  0x50a5,  0x60c6,  0x70e7, 
0x8108,  0x9129, 0xa14a,  0xb16b,  0xc18c,  0xd1ad,  0xe1ce,  0xf1ef,  
0x1231,  0x0210, 0x3273,  0x2252,  0x52b5,  0x4294,  0x72f7,  0x62d6, 
0x9339,  0x8318, 0xb37b,  0xa35a,  0xd3bd,  0xc39c,  0xf3ff,  0xe3de, 
0x2462,  0x3443, 0x0420,  0x1401,  0x64e6,  0x74c7,  0x44a4,  0x5485, 
0xa56a,  0xb54b, 0x8528,  0x9509,  0xe5ee,  0xf5cf,  0xc5ac,  0xd58d, 
0x3653,  0x2672, 0x1611,  0x0630,  0x76d7,  0x66f6,  0x5695,  0x46b4, 
0xb75b,  0xa77a, 0x9719,  0x8738,  0xf7df,  0xe7fe,  0xd79d,  0xc7bc, 
0x48c4,  0x58e5, 0x6886,  0x78a7,  0x0840,  0x1861,  0x2802,  0x3823, 
0xc9cc,  0xd9ed, 0xe98e,  0xf9af,  0x8948,  0x9969,  0xa90a,  0xb92b, 
0x5af5,  0x4ad4, 0x7ab7,  0x6a96,  0x1a71,  0x0a50,  0x3a33,  0x2a12, 
0xdbfd,  0xcbdc, 0xfbbf,  0xeb9e,  0x9b79,  0x8b58,  0xbb3b,  0xab1a, 
0x6ca6,  0x7c87, 0x4ce4,  0x5cc5,  0x2c22,  0x3c03,  0x0c60,  0x1c41, 
0xedae,  0xfd8f, 0xcdec,  0xddcd,  0xad2a,  0xbd0b,  0x8d68,  0x9d49, 
0x7e97,  0x6eb6, 0x5ed5,  0x4ef4,  0x3e13,  0x2e32,  0x1e51,  0x0e70, 
0xff9f,  0xefbe, 0xdfdd,  0xcffc,  0xbf1b,  0xaf3a,  0x9f59,  0x8f78, 
0x9188,  0x81a9, 0xb1ca,  0xa1eb,  0xd10c,  0xc12d,  0xf14e,  0xe16f,  
0x1080,  0x00a1, 0x30c2,  0x20e3,  0x5004,  0x4025,  0x7046,  0x6067, 
0x83b9,  0x9398, 0xa3fb,  0xb3da,  0xc33d,  0xd31c,  0xe37f,  0xf35e, 
0x02b1,  0x1290, 0x22f3,  0x32d2,  0x4235,  0x5214,  0x6277,  0x7256, 
0xb5ea,  0xa5cb, 0x95a8,  0x8589,  0xf56e,  0xe54f,  0xd52c,  0xc50d, 
0x34e2,  0x24c3, 0x14a0,  0x0481,  0x7466,  0x6447,  0x5424,  0x4405, 
0xa7db,  0xb7fa, 0x8799,  0x97b8,  0xe75f,  0xf77e,  0xc71d,  0xd73c, 
0x26d3,  0x36f2, 0x0691,  0x16b0,  0x6657,  0x7676,  0x4615,  0x5634, 
0xd94c,  0xc96d, 0xf90e,  0xe92f,  0x99c8,  0x89e9,  0xb98a,  0xa9ab, 
0x5844,  0x4865, 0x7806,  0x6827,  0x18c0,  0x08e1,  0x3882,  0x28a3, 
0xcb7d,  0xdb5c, 0xeb3f,  0xfb1e,  0x8bf9,  0x9bd8,  0xabbb,  0xbb9a, 
0x4a75,  0x5a54, 0x6a37,  0x7a16,  0x0af1,  0x1ad0,  0x2ab3,  0x3a92, 
0xfd2e,  0xed0f, 0xdd6c,  0xcd4d,  0xbdaa,  0xad8b,  0x9de8,  0x8dc9, 
0x7c26,  0x6c07, 0x5c64,  0x4c45,  0x3ca2,  0x2c83,  0x1ce0,  0x0cc1, 
0xef1f,  0xff3e, 0xcf5d,  0xdf7c,  0xaf9b,  0xbfba,  0x8fd9,  0x9ff8,  
0x6e17,  0x7e36, 0x4e55,  0x5e74,  0x2e93,  0x3eb2,  0x0ed1,  0x1ef0  
};

STU_MCUSend_GPSData stuMcuSend_Gpsdata;   //向控制器发送定位信息
#if 0
/*
*********************************************************************************************************
*Function name	:McuSaveToMemory
*Description	:保存Mcu模块相关参数到存储器，并对变量初始化
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2016-9-22
*Modified		:
*********************************************************************************************************
*/

void McuSaveToMemory()
{
	uint8 aucTemp[20];

	aucTemp[0] = 0xaa;
	aucTemp[1] =  m_stuMcuCmd.LockControlFlag;
	aucTemp[2] =  m_stuMcuCmd.ucRespSerFlag;
	aucTemp[3] =  m_stuMcuCmd.usLockOneSq & 0xff;
	aucTemp[4] = (m_stuMcuCmd.usLockOneSq>>8) & 0xff;
	aucTemp[5] =  m_stuMcuCmd.usLockSecSq & 0xff;
	aucTemp[6] = (m_stuMcuCmd.usLockSecSq>>8) & 0xff;
	aucTemp[7] =  m_stuMcuCmd.usUnLockSq & 0xff;
	aucTemp[8] = (m_stuMcuCmd.usUnLockSq>>8) & 0xff;
	aucTemp[9] =  m_stuMcuCmd.usMonitorSq & 0xff;
	aucTemp[10] =(m_stuMcuCmd.usMonitorSq>>8) & 0xff;
	
	aucTemp[19] = SumCalc(aucTemp, 19);
	//CPU_IntDis();//0413
	WriteToFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp);
	// CPU_IntEn();
	OSTimeDly(OS_TICKS_PER_SEC/100);
	WriteToFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp);
}
/*
*********************************************************************************************************
*Function name	:McuReadFromMemory
*Description	:从存储器中读出Mcu模块相关参数，并对变量初始化
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2016-9-22
*Modified		:
*********************************************************************************************************
*/
void McuReadFromMemory()
{
	uint8 aucTemp1[20];
	uint8 sum1 = 0;
	uint8 aucTemp2[20];
	uint8 sum2 = 0;
	uint8 m_aucDataSave[20];		//保存到铁电和flash的数据缓存，
	uint8 flag = 0;
	
	ReadFromFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp1);
	sum1 = SumCalc(aucTemp1, 19);
	
    ReadFromFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp2);
	sum2 = SumCalc(aucTemp2,19);

	if((aucTemp1[0]==0xAA) && (sum1==aucTemp1[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp1[0],20);
		if((aucTemp2[0]!=0xAA) || (sum2!=aucTemp2[19]))
		{
			PC_SendDebugData((uint8 *)("MCUSAVE 2 ERR"), 13, DEBUG_ANYDATA);
			WriteToFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp1);
		}
	}
	else if((aucTemp2[0]==0xAA) && (sum2==aucTemp2[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp2[0],20);
		PC_SendDebugData((uint8 *)("MCUSAVE 1 ERR"), 13, DEBUG_ANYDATA);
		WriteToFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp2);
	}
	else
	{
		PC_SendDebugData((uint8 *)("MCUSAVE 1_2 ERR"), 15, DEBUG_ANYDATA);
	}
	
	if(1==flag)
	{
		m_stuMcuCmd.LockControlFlag = m_aucDataSave[1];
		m_stuMcuCmd.ucRespSerFlag   = m_aucDataSave[2];
		m_stuMcuCmd.usLockOneSq		= m_aucDataSave[3] + (m_aucDataSave[4]<<8);	
		m_stuMcuCmd.usLockSecSq		= m_aucDataSave[5] + (m_aucDataSave[6]<<8);	
		m_stuMcuCmd.usUnLockSq		= m_aucDataSave[7] + (m_aucDataSave[8]<<8);
		m_stuMcuCmd.usMonitorSq 	= m_aucDataSave[9] + (m_aucDataSave[10]<<8);
	}
}

/**********************************************************************************
** 函数名称: void MCU_GPSStateSend(void)
** 功能描述  用于定时向MCU发送GPS自身状态数据函数
** 输        入: 无参数
** 输　      出: 
** 全局变量: 
** 调用模块:
** 作　     者 :
** 日　     期 : 2016-12-25
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 MCU_CmdLockSend(void)
{
	uint8 Sendbuff[8]={0,0,0,0,0,0,0,0};
	uint8 sendbuff2[8] = {0,0,0,0,0,0,0,0};
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	uint16 password = 0;
	uint8 ucOpenModeFlag = 0;
	static uint16 seed=1;

    static uint8 lockcntflag = 0;
	
    if(m_stuMcuCmd.LockControlFlag&BIT(0))
    {
        if(m_stuMcuCmd.LockControlFlag&BIT(1)) 
        {
            Sendbuff[1] = 35;    //开启监控模式
            ucOpenModeFlag = 1;
        }
        else
            Sendbuff[1] = 36;    //关闭监控模式        
    }

    if(lockcntflag==0)
   	{
    	if(m_stuMcuCmd.LockControlFlag&BIT(2))
    	{
        	if(m_stuMcuCmd.LockControlFlag&BIT(3))  //一级锁车
        	{
            	Sendbuff[0] = 31;
        	}
        	else                          //一级解锁
        	{
            	Sendbuff[0] = 32;
        	}
      	}
		else
		{
       //     Sendbuff[0] = ucLockTemp;
		}
		lockcntflag=1;
		//ucLockTemp = Sendbuff[0];
    }
	else
	{
    	if(m_stuMcuCmd.LockControlFlag&BIT(4))
    	{
            if(m_stuMcuCmd.LockControlFlag&BIT(5))  //二级锁车
            {
                Sendbuff[0]=33;
            }
            else                             //二级解锁
            {
                Sendbuff[0]=34;
            }
    	}
		else
		{
       //     Sendbuff[0] = ucLockTemp;
		}
     //   ucLockTemp = Sendbuff[0];		
       	lockcntflag=0;
	}

	if(m_stuMcuCmd.LockControlFlag&0x15 )
	{	
       
		srand(seed);		//产生随机数
	    A = rand();
	    seed = A;
		
		A_L = A & 0xff;
		A_H = (A>>8) & 0xff;
		password = tbl[A_H];
		C = ~(password ^ A);

		sendbuff2[0]= Sendbuff[0];
		sendbuff2[1] = Sendbuff[1];
		
		sendbuff2[4] = A_H;
		sendbuff2[5] = A_L;
		sendbuff2[6] = (C>>8) & 0xff;
		sendbuff2[7] = C & 0xff;
        
		
		if(ucOpenModeFlag==1)
		{
            Sendbuff[3] = (uint8)(m_stuMcuCmd.uiKey&0xFF);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiKey>>8);
    	    Sendbuff[5] = (uint8)(m_stuMcuCmd.uiKey>>16);
    	    Sendbuff[6] = (uint8)(m_stuMcuCmd.uiKey>>24);
		}
		else
		{
            Sendbuff[3] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiFs>>8);
    	    Sendbuff[5] = (uint8)(m_stuMcuCmd.uiFs>>16);
    	    Sendbuff[6] = (uint8)(m_stuMcuCmd.uiFs>>24);
		}
     	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe1af4, 8, Sendbuff);
		OSTimeDly(5);
		CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0af4, 8, sendbuff2);
		return 1;
	}
	return 0;
}
#endif

/**********************************************************************************
** 函数名称: void MCU_GPSStateSend(void)
** 功能描述  用于定时向MCU发送GPS自身状态数据函数
** 输        入: 无参数
** 输　      出: 
** 全局变量: 
** 调用模块:
** 作　     者 :
** 日　     期 : 2016-12-25
**--------------------------------------------------------------------------------
*********************************************************************************/
#if 0
void MCU_CmdOldLockSend(void)
{
	uint8 Sendbuff[8]={0,0,0,0,0,0,0,0};
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	uint16 password = 0;
	static uint16 seed=1;
   
	srand(seed);		//产生随机数
    A = rand();
    seed = A;
	
	A_L = A & 0xff;
	A_H = (A>>8) & 0xff;
	password = tbl[A_H];
	C = ~(password ^ A);

	Sendbuff[0] = stuMcuSend_Gpsdata.aControlData[0];
	Sendbuff[1] = stuMcuSend_Gpsdata.aControlData[1];
	Sendbuff[4] = A_H;
	Sendbuff[5] = A_L;
	Sendbuff[6] = (C>>8) & 0xff;
	Sendbuff[7] = C & 0xff;
 	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0af4, 8, Sendbuff);
}
#endif
uint8 MCU_SendNewLockCmd(uint8 *ptr)
{
    uint8 ucCanChannel;
	uint8 ucFF;
    uint32 uiCanID = 0;
	uint8 ucCanDatalen = 0;
	uint8 aCanData[8] = {0};
	
	ucCanChannel = ptr[0];
	if(ucCanChannel>1)
		return FALSE;
	
	ucFF = ptr[1];
	if(ucFF>1)
		return FALSE;	

    uiCanID = (uint32)(ptr[2]<<24) + (uint32)(ptr[3]<<16) +(uint32)(ptr[4]<<8) + ptr[5];
    ucCanDatalen = ptr[6];
    if(ucCanDatalen > 8)
		return FALSE;
	
    memcpy(aCanData,&ptr[7],8);
 //   memcpy(stuMcuSend_Gpsdata.aControlData,&aCanData[0],2);
	
	if(aCanData[0]==35)    //开启监控模式  采用密钥
	{
        aCanData[1] = (uint8)(m_stuMcuCmd.uiKey&0xFF);
	    aCanData[2] = (uint8)(m_stuMcuCmd.uiKey>>8);
	    aCanData[3] = (uint8)(m_stuMcuCmd.uiKey>>16);
	    aCanData[4] = (uint8)(m_stuMcuCmd.uiKey>>24);
	}
	else
	{
        aCanData[1] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
	    aCanData[2] = (uint8)(m_stuMcuCmd.uiFs>>8);
	    aCanData[3] = (uint8)(m_stuMcuCmd.uiFs>>16);
	    aCanData[4] = (uint8)(m_stuMcuCmd.uiFs>>24);
	}

 //   stuMcuSend_Gpsdata.ucCount = 2;  //发送旧协议命令

    CanWrite(ucCanChannel, ucFF, uiCanID, ucCanDatalen, aCanData);	
	return TRUE;
}

#if 0
/**********************************************************************************
** 函数名称: void MCU_GPSStateSend(void)
** 功能描述  用于定时向MCU发送GPS自身状态数据函数
** 输        入: 无参数
** 输　      出: 
** 全局变量: 
** 调用模块:
** 作　     者 : Lxf
** 日　     期 : 2012-8-13
**--------------------------------------------------------------------------------
*********************************************************************************/
void MCU_GPSSendWorkState(void)
{

	uint8 arr[8];
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	static uint16 seed=1;
	uint16 password = 0;
	static uint8 heart_flag=0;
	
    if(heart_flag==0)
    {
        arr[0]=0x55;
		heart_flag=1;
    }
	else
    {
		 arr[0]=0xaa;
		 heart_flag=0;
	}	 	

    if(GetSwitch3State()&BIT(0))   //电源
    {
        arr[1]=13;                 //电池供电
    }
    else 
    {
        arr[1]=12;                 //外电源供电
    }
    
    if(GPS_GetAnteState())      //gps天线
    {
        arr[2]=15;              //GPS天线故障  
    }
    else
    {
        arr[2]=14;              //GPS天线正常
    }

#if 0	
	if(0==Get_SIMCardState()||!(STU_Systemstate.DisassmbleSwitch&BIT(4)))
	{
        arr[3] = 16;        //SIM卡正常
	}
	else
	{
        arr[3] = 17;       //SIM卡故障
	}
#endif
    arr[3] = 16;           //SIM卡正常

	srand(seed);	              //产生随机数
    A = rand();
    seed = A;
	
	A_L = A & 0xff;
	A_H = (A>>8) & 0xff;
	password = tbl[A_H];
	C = ~(password ^ A);
	
	arr[4] = A_H;
	arr[5] = A_L;
	arr[6] = (C>>8) & 0xff;
	arr[7] = C & 0xff;
//	if(stuA5Comm.ucCommflag==0)
    CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe08f4, 8, arr);
}

#endif
//GPS终端心跳 收到MCU心跳后进行校验发送
void MCU_GPSSendWorkState_check(uint8 *ptr)
{

	uint8 buff[8];
	static uint8 heart_flag=0;
    uint32 uis;

    uint8 n1 = 3,n2 = 2,n3 = 0,n4 = 2;

	uis = ptr[2]+(ptr[4]<<8)+(m_stuMcuCmd.uiKey&0x00FF0000)+((m_stuMcuCmd.uiKey&0xFF)<<24);
   
    m_stuMcuCmd.uiFs = (uis%17)*(uis%19)*n4+(uis%29)*(uis%57)*n3+(uis%25)*(uis%1352)*n2+(uis%5273)*n1;
   
	
    if(heart_flag==0)
    {
        buff[0]=0x55;
		heart_flag=1;
    }
	else
    {
		 buff[0]=0xaa;
		 heart_flag=0;
	}	 	

    if(GetSwitch3State()&BIT(0))   //电源
    {
        buff[1]=13;                 //电池供电
    }
    else 
    {
        buff[1]=12;                 //外电源供电
    }

	
    if(GPS_GetAnteState())      //gps天线
    {
        buff[2]=15;              //GPS天线故障  
    }
    else
    {
        buff[2]=14;              //GPS天线正常
    }

#if 0	
	if(0==Get_SIMCardState()||!(STU_Systemstate.DisassmbleSwitch&BIT(4)))
	{
        arr[3] = 16;        //SIM卡正常
	}
	else
	{
        arr[3] = 17;       //SIM卡故障
	}
#endif
    buff[3] = 16;           //SIM卡正常
    buff[4] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
	buff[5] = (uint8)(m_stuMcuCmd.uiFs>>8);
	buff[6] = (uint8)(m_stuMcuCmd.uiFs>>16);
	buff[7] = (uint8)(m_stuMcuCmd.uiFs>>24);
//	if(stuA5Comm.ucCommflag==0)
    CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB01B1, 8, buff);
}

#if 0
//GPS向MCU发送位置状态 海拔高度等
void MCU_GPSSendPostionState(void)
{
	uint8 arr[8] = {0};
	uint8 ucTemp =0;
	uint16 usHeight = 0;
	
	arr[0] = (GPS_GetState()& BIT(1))?1:0;	
    if(1==GPS_GetLatitudeHemisphere())
     	ucTemp |= BIT(0);
	else
		ucTemp&=~BIT(0);
    if(1==GPS_GetLongitudeHemisphere())
     	ucTemp |= BIT(1);
	else
		ucTemp&=~BIT(1);
    if(GPS_GetHeight())
    {
     	ucTemp |= BIT(7);
        usHeight = GPS_GetHeight();
	}
	else
	{
		ucTemp&=~BIT(7);
		usHeight = 0-GPS_GetHeight();
	}	
	
	arr[1] = ucTemp;	
	arr[2] = usHeight & 0xff;
	arr[3] = (usHeight>>8) & 0xff;
	arr[4] = GPS_GetSatellitenums();

    if(!(GPS_GetState() & BIT(1)))   //不定位
    {
		arr[1]= 0;
        arr[2]= 0;
		arr[3]= 0;
        arr[4]= 0;
		arr[5]= 0;
        arr[6]= 0;
		arr[7]= 0;
	}
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff6, 8, arr);
}

//GPS向MCU发送位置信息 经纬度
void MCU_GPSSendPostionInformation(void)
{
	uint8 arr[8] = {0};
	uint8 *p;

	if(GPS_GetState() & BIT(1))      //定位
	{
		
		p = GPS_GetLongitude_Original();
		arr[0] = 0xff & DecToUint16(p,3);
		arr[1] = 0xff & DecToUint16(p+3,2);
		arr[2] = 0xff & DecToUint16(p+6,2);
		arr[3] = 0xff & DecToUint16(p+8,2);
		p = GPS_GetLatitude_Original();
		arr[4] = 0xff & DecToUint16(p,2);
		arr[5] = 0xff & DecToUint16(p+2,2);
		arr[6] = 0xff & DecToUint16(p+5,2);
		arr[7] = 0xff & DecToUint16(p+7,2);		
	}
	else                    //不定位 0
	{
        arr[0]= 0;
		arr[1]= 0;
        arr[2]= 0;
		arr[3]= 0;
        arr[4]= 0;
		arr[5]= 0;
        arr[6]= 0;
		arr[7]= 0;
	}
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff7, 8, arr);
}
#endif
void MCU_GPSSendPostionState(void)
{
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB03B1, 8, stuMcuSend_Gpsdata.aGpsstate);
}

void MCU_GPSSendPostionInformation(void)
{
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB04B1, 8, stuMcuSend_Gpsdata.aGpsData);
}
/**********************************************************************************
** 函数名称: MCU_SendCan1Data
** 功能描述: 该函数每100ms执行一次
             每间隔2S检测一次当前存储的网络命令，如果有命令则向仪表发送
**           命令分为3中 1.监控模式命令 2.一级锁车命令 3.二级锁车命令 三种命令
**           轮循向仪表发送，直到收到仪表响应后才删除该命令停止发送
** 输        入: 无参数
** 输　      出: 
** 全局变量: 
** 调用模块:
** 作　     者 : lxf
** 日　     期 : 2016年12月25日                        
**--------------------------------------------------------------------------------
*********************************************************************************/
void MCU_SendCan1Data(void)
{
    static uint8 SendFlag=0;           //发送GPS状态数据与其他命令的选择标志
                                       //0=发送状态数据0x55;1=发送定位属性,2=发送经纬度
  
    if(GetAccState()||(1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState())         //收不到MCU数据，则不发送命令
    {
        if(m_stuMcuCmd.usSendMCULockTimer)
			m_stuMcuCmd.usSendMCULockTimer--;
		if(m_stuMcuCmd.usSendGPSDataTimer)
			m_stuMcuCmd.usSendGPSDataTimer--;
		
		if(!m_stuMcuCmd.usSendMCULockTimer) 
		{
			m_stuMcuCmd.usSendMCULockTimer = MCU_GPSCMD_SEND_TIME;      //定时为1秒钟
         /*
			if(1==MCU_CmdLockSend())      //有数据发
			{
    			if(0==m_stuMcuCmd.usSendGPSDataTimer)
    				m_stuMcuCmd.usSendGPSDataTimer = 1;
			}
			*/
   	 	}
		if(!m_stuMcuCmd.usSendGPSDataTimer)
		{
	        if(0==SendFlag)
			{
	       // 	MCU_GPSSendWorkState();
				SendFlag = 1;
			}
			else if(1==SendFlag)
			{
				MCU_GPSSendPostionState();
				SendFlag = 2;
			}
			else if(2==SendFlag)
			{
				MCU_GPSSendPostionInformation();
				SendFlag = 0;
				m_stuMcuCmd.usSendGPSDataTimer = MCU_GPSSTATE_SEND_TIME; //定时5秒钟
			}
		}
    }
}

//清除CAN帧地址内容
void MCU_ClearCanData(void)
{
    uint8 i = 0;
	
	for(i=0; i<MAX_CAN_FRAME_NUM; i++)
	{
	    CanData[i].id = 0;
	}
    g_stuXWFaultCodeRecv.ucFaultCodeNum = 0;
	g_stuXWFaultCodeUp.ucFaultCodeNum = 0;
	g_stuXWFaultCodeRecv.ucVehicleType = 0;
	for(i=0;i<MAX_CAN_FaultCode_NUM;i++)
	{
        g_stuXWFaultCodeRecv.CanData[i].id = 0;
		g_stuXWFaultCodeUp.CanData[i].id = 0;
	}	
}
/*********************************************************************************************************
*Function name	:MCUCommStateJudge
*Description	:MCU通信状态判断，此函数1s执行一次
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-7-9
*Modified		:
********************************************************************************************************/
void MCUCommStateJudge(void)
{
	static uint16 usACCOnTime = TIME_COMM_UNNORMAL;          // 钥匙开关为高持续时间
	static uint8 ucACCPre;                                   // 前一次钥匙开关的状态
	uint8 ucACCCurrent;                                      // 当前钥匙开关的状态
	static uint8 ucPWRCloseTimer = 0;            //5秒钟
	static uint8 ucCanRstflag = 0;               //0=可以重新启动CAN，1=不可以重新启动CAN
	
	ucACCCurrent =	GetAccState();    // 获取当前钥匙开关状态 
	if(ucACCCurrent != ucACCPre)
	{
		if(!ucACCCurrent)             // 由高到低的变化
		{
			usACCOnTime = TIME_COMM_UNNORMAL;
			ucCanRstflag = 0;
		}
	}

	if(ucACCCurrent)                  // 如果钥匙开关为高
	{
		if(usACCOnTime)
			usACCOnTime--;

		if(!usACCOnTime && m_stuCommState.ucRcvCan1DataFlag == 2&&ucCanRstflag==0)
		{
		    ucCanRstflag = 1;
			m_stuCommState.ucCan1CommState = 1;   //CAN通讯异常
            PC_SendDebugData((uint8 *)("CAN OFF"), 7, DEBUG_ANYDATA);
            POWEROFF_CAN();      //关闭CAN模块电源
            ucPWRCloseTimer = 5;
		}
		if(ucPWRCloseTimer)
		{
		    if(!(--ucPWRCloseTimer))
            {
                McuInit();      //关电计时5秒 重新对CAN模块上电并进行初始化
                PC_SendDebugData((uint8 *)("CAN REST"), 8, DEBUG_ANYDATA);
		    }
		}	
	}
	
	if(m_stuCommState.ucRcvCan1DataFlag==1)
	{
        ucCanRstflag = 0;
	}
	
    ucACCPre = ucACCCurrent;	
}

//统计can接口没有收到数据计时
//该函数10ms运行一次
void CanRcvDataTimer()
{
	if(++m_stuCommState.usCan1RcvErrTime > MAX_CAN1_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan1DataFlag = 2;
		MCU_ClearCanData();     //CAN通讯中断后，清除原有保留CAN数据
	}
	if(++m_stuCommState.usCan2RcvErrTime > MAX_CAN2_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan2DataFlag = 2;
	}
}


/*
*********************************************************************************************************
*Function name	:GetMcuFaultCode
*Description	:获取故障诊断码数据
*Arguments  	:faultCodeData	:指向故障诊断码数据数据缓存的指针
*Returns    	:故障诊断码个数，0表示没有故障诊断码数据
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetMcuFaultCode(uint8 *faultCodeData)
{
	uint8 i;
	uint8 faultCodeNum=0;

	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode)
		{
			*faultCodeData     = m_stuActiveFaultCodeList[i].faultCode & 0xff;
			*(faultCodeData+1) = (m_stuActiveFaultCodeList[i].faultCode >> 8) & 0xff;
			*(faultCodeData+2) = (m_stuActiveFaultCodeList[i].faultCode >> 16) & 0xff;
			*(faultCodeData+3) = (m_stuActiveFaultCodeList[i].faultCode >> 24) & 0xff;
		    faultCodeData += 4;
			faultCodeNum++;
		}
	}
	return faultCodeNum;
}

/*
*********************************************************************************************************
*Function name	:IsNewMcuFaultCode
*Description	:获取是否有新的DTC需要上报
*Arguments  	:
*Returns    	:0=无,1=有
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 IsNewMcuFaultCode(void)
{
	return f_ucNewFaultCodeFlag;
}
/*
*********************************************************************************************************
*Function name	:ClearNewMcuFaultCodeFlag
*Description	:清除有新的DTC需要上报标志,此操作在成功上报给平台后执行
*Arguments  	:
*Returns    	:
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
void ClearNewMcuFaultCodeFlag()
{
	f_ucNewFaultCodeFlag = 0;
}
/******************************************************************************
** 函数名称: GetCanRcvState
** 功能描述: 获取CAN接收状态
** 输    入:
** 输    出: 无
** 返    回: 1=接收,2=停止
** 作    者: hhm
** 日    期: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
//获取CAN接收状态,1:收到了CAN1数据，2:没有收到CAN1数据
uint8 GetCanRcvState(uint8 channel)
{
	if(CAN_CHANNEL1==channel)
		return m_stuCommState.ucRcvCan1DataFlag;
	else if(CAN_CHANNEL2==channel)
		return m_stuCommState.ucRcvCan2DataFlag;
	else
		return 0;
}

void AddCanFrameToBuff(uint32 id, uint8 *data)
{
	uint8 i;
	uint8 j=0xff;
	uint8 flag = 0;


	for(i=0; i<MAX_CAN_FRAME_NUM; i++)    //?此处个数应该采用设定个数才合理
	{
		if(CanData[i].id==id)
		{
			memcpy(CanData[i].aucData, data, 8);
			flag = 1;
			break;
		}
		else
		{
			if((CanData[i].id==0) && (j==0xff))//???
			{
				j = i;
			}
		}
	}
			
	
	if(1==flag)
		return;
	else
	{
		if(j!=0xff)
		{
			CanData[j].id = id;
			memcpy(CanData[j].aucData, data, 8);  //???
		}
	}
}
/*
*********************************************************************************************************
*Function name	:AddToFaultCodeList
*Description	:接收到来自CAN1的故障码后，先搜索激活故障码列表是否有该故障码，如果有则更新该故障码的计时,
*				 如果没有，则认为是新产生的故障码，添加到激活故障码列表,并且置;如果列表已满，则
*				 丢弃，直到列表有空单元
*Arguments  	:faultCode	:来自CAN1的故障码
				:ucSource   :故障码的来源，0:发动机ecm，0x31:控制器
*Returns    	:0:没有新增故障码或故障码列表已满，1:新增了故障码		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
uint8 AddToFaultCodeList(uint32 faultCode, uint8 ucSource)
{
	uint8 i,j;
	uint8 ucSameFaultcodeFlag = 0;	//是否有相同故障码标志，1:有不相同故障码，0:无不同故障码
	
	if(!faultCode)
	{
		return 0;
	}
	faultCode = (faultCode & 0x00ffffff)| ((ucSource<<24)&0xff000000);//添加来源
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode  == faultCode)
		{
			m_stuActiveFaultCodeList[i].activeTimes = TIME_FAULTCODE_ACTIVE;//更新计时
			return 0;
		}
		else
		{
			ucSameFaultcodeFlag = 1;
		}
	}	
	if(ucSameFaultcodeFlag==1)	//新增故障码
	{
		for(j=0; j<MAX_ACTIVE_FAULTCODE_NUM; j++)
		{
			if(m_stuActiveFaultCodeList[j].faultCode == 0)
			{
				m_stuActiveFaultCodeList[j].faultCode = faultCode;
				m_stuActiveFaultCodeList[j].activeTimes = TIME_FAULTCODE_ACTIVE;
				return 1;
			}
		}
		return 0;			//列表已满
	}
	return 0;
}

/*
*********************************************************************************************************
*Function name	:FaultcodeCheck
*Description	:此函数每1秒运行一次，对故障码列表中故障码存在的时间进行倒计时，倒计时到0时，认为该故障码
*                已经消失，对该故障码清零
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
void FaultcodeCheck(void)
{
	uint8 i;
	uint8 flag = 0;

	if(!GetAccState())
		return;
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode)
		{
			if(m_stuActiveFaultCodeList[i].activeTimes)
				m_stuActiveFaultCodeList[i].activeTimes--;
			else
			{
				//PC_SendDebugData((uint8 *)( "D"), 1, DEBUG_ANYDATA);
				m_stuActiveFaultCodeList[i].faultCode = 0;
				flag = 1;
			}
		}
	}
	
}

#define SP_CANID1_NUM	1
uint32 SpecialCanId1[SP_CANID1_NUM] = {0x1ADC05C1};   //需要特殊处理且上报

#define SP_CANID2_NUM	5
//只需要特殊处理，不需要上报
uint32 SpecialCanId2[SP_CANID2_NUM] = {0x0591,0x0592,0x0612,0x1fffffff,0x1ffffffe};

//返回值:3=该帧只需要上传,1=需要上传且需特殊处理, 2=只需要特殊处理,0=直接丢弃
uint8 CanFrameFilter(uint32 id)
{
	uint8 i;	

	for(i=0; i<SP_CANID1_NUM; i++)
	{
		if(id==SpecialCanId1[i])
		{
			return 1;
		}
	}
	
	for(i=0; i<SP_CANID2_NUM; i++)
	{
		if(id==SpecialCanId2[i])
		{
			return 2;
		}
	}
	for(i=0; i<g_stuSYSParamSet.ucCanIdNum; i++)
	{
		if(id==g_stuSYSParamSet.auiCanId[i])
		{
			return 3;
		}
	}
	
	return 0;
}

/*
*********************************************************************************************************
*Function name	:DealCan1Message
*Description	:处理接收到的来自CAN1的报文
*Arguments  	:msg	:来自CAN1的报文
*Returns    	:none		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
void DealCan1Message(MessageDetail msg)
{
	uint8 arr[8];
	uint8 buff[16];
	uint32 id = 0;
//	uint8 i=0;
//    uint16 usVehicleModel=0;
    uint16 usPercent = 0;
//	static uint8 ucCheckCount=0;   //校验异常次数

	memcpy(arr, msg.CANRE, 8);
	id = msg.CANID;
	
    if(msg.FF==STD_FRAME)
        stu_McuFirmware.ucCANFrameFormat = STD_FRAME;  //标准帧
	else
		stu_McuFirmware.ucCANFrameFormat = EXT_FRAME;  //扩展帧	

	KCMCU_RecVCan_ACK(id,arr);
	
    switch(id)
    {   

	    case 0x591:

            if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev)
			{
			    switch(arr[0])
			    {
			        case 0xA4: 		//判断MCU是否应答启动块下载命令
						if(arr[4]==0x27)
						{
                            stu_McuFirmware.ucMcuRespflag = 1;
                            stu_McuFirmware.ucLoadStep = 2;
						}
                        else if(arr[4]==0x02)
                        {
						}						
						break;
					case 0xA2:     //判断MCU是否成功接收数据包
					    if(arr[1]==0x27)
					    {					        
                            stu_McuFirmware.usReadFlashSN++;
							if(stu_McuFirmware.usReadFlashSN==stu_McuFirmware.usTotalPackets)
							{
                                stu_McuFirmware.ucLoadStep = 3;  //数据包发送完毕，通知MCU升级固件
    							stu_McuFirmware.ucMcuRespflag = 1;
							}
							else
							{
    							stu_McuFirmware.ucLoadStep = 3;  //发送块下载结束命令
    							stu_McuFirmware.ucMcuRespflag = 1;
							}
						}
						else if(arr[1]==0x02)
						{
						}
						else
						{
                            stu_McuFirmware.ucMcuRespflag = 0;
						}
						break;
					case 0xA1:     //MCU应答块下载结束命令 ,开始下一个数据块下载
                        if(stu_McuFirmware.usReadFlashSN==stu_McuFirmware.usTotalPackets)
                        {
    					    stu_McuFirmware.ucLoadStep = 4;
                        }
						else
						{
    					    stu_McuFirmware.ucLoadStep = 1;							
						}
					//终端停止发送
						break;
					case 0x4B:     //控制器应答升级结果
					    if(arr[4]==0||arr[4]==8)
					    {
							stu_McuFirmware.ucUploadResult = 1;
					    }
						else
						{
							stu_McuFirmware.ucUploadResult = 0;
						}
					    stu_McuFirmware.ucLoadStep = 7;  //MCU升级结果应答
						break;
    			    default:
    			  	    break;
				}	
                
				//如果失败 重新发送
			}
			break;
		case 0x0612:        //控制器收包进度
            if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev)
            {
    			if(arr[0]==0x2B&&arr[1]==0x18&&arr[2]==0xA0)
    			{
                    usPercent = arr[4] + (uint16)(arr[5]<<8);  //当usPercent=10000时表示收包完成
    			}
			}
			break;
        case 0x1ADC05C1:    //MCU当前锁车情况
            MCU_GPSSendWorkState_check(arr);
			break;
			
	#if 0		
		case 0x18fe0bf4:    //此处添加锁车、监控模式、安装模式处理代码。
		case 0x18fe1bf4:
			
           if(id==0x18fe0bf4)
		       m_stuMcuCmd.ucCmdCheckFlag = 0;
		   else
		   {
		       m_stuMcuCmd.ucCmdCheckFlag = arr[3];    //校验信息
		       if(m_stuMcuCmd.ucCmdCheckFlag==52)  //0x34
		       {
		           if(ucCheckCount<3)
		           {
                       ucCheckCount++;
					   break;
		           }
                   else
				   	   ucCheckCount = 0;
			   }
			   else
			   {
                   ucCheckCount = 0;
			   }
		   }
		       
           switch(arr[0])
           {
           	  case 41:      //一级锁车应答
	
                if((m_stuMcuCmd.LockControlFlag&BIT(2))&&(m_stuMcuCmd.LockControlFlag&BIT(3)))
                {
                     m_stuMcuCmd.LockControlFlag &=~BIT(2);
                     m_stuMcuCmd.ucRespSerFlag |= BIT(0);
                     McuSaveToMemory();    //保存命令
                }
			  	break;
			  case 42:      //一级解锁应答

                if((m_stuMcuCmd.LockControlFlag&BIT(2))&&!(m_stuMcuCmd.LockControlFlag&BIT(3)))  //判定当前是否有一级解锁命令发出
                {
                	m_stuMcuCmd.ucRespSerFlag |= BIT(2);
                    m_stuMcuCmd.LockControlFlag &=~BIT(2);
                    McuSaveToMemory();     //保存命令
                }
			  	break;
			  case 43:     //二级锁车应答

                if((m_stuMcuCmd.LockControlFlag&BIT(4))&&(m_stuMcuCmd.LockControlFlag&BIT(5)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(1);
                    m_stuMcuCmd.LockControlFlag &=~BIT(4);
                    McuSaveToMemory();     //保存命令     
                }
			  	break;
			  case 44:      //二级解锁应答

                if((m_stuMcuCmd.LockControlFlag&BIT(4))&&!(m_stuMcuCmd.LockControlFlag&BIT(5)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(3);
                    m_stuMcuCmd.LockControlFlag &=~BIT(4);
                    McuSaveToMemory();     //保存命令   
                }
			  	break;
			  default:
			  	break;
           }
		   switch(arr[1])
           {
           	  case 45:      //打开监控模式应答
			  	if((m_stuMcuCmd.LockControlFlag&BIT(0))&&(m_stuMcuCmd.LockControlFlag&BIT(1)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(4);
                    m_stuMcuCmd.LockControlFlag &=~BIT(0);
                    McuSaveToMemory();     //保存命令                                            
                }  
			  	break;
			  case 46:      //关闭监控模式应答
			  	if((m_stuMcuCmd.LockControlFlag&BIT(0))&&!(m_stuMcuCmd.LockControlFlag&BIT(1)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(5);
                    m_stuMcuCmd.LockControlFlag &=~BIT(0);                       
                    McuSaveToMemory();     //保存命令					  
                }   
			  	break;			  
			  default:
			  	break;
           }
		   break;

	
		case 0x18fe24f3:
            usVehicleModel = arr[3]+(uint16)(arr[4]<<8);
			if(usVehicleModel>=450)
				g_stuXWFaultCodeRecv.ucVehicleType = 2; //大吨位
			else
				g_stuXWFaultCodeRecv.ucVehicleType = 1; //小吨位
			break;
		case 0x18FE27F3:    //中小挖
			if(g_stuXWFaultCodeRecv.ucVehicleType!=1)
				return;
			
            if(arr[0]<=1&&arr[4]<=1)
            {
                g_stuXWFaultCodeRecv.ucFaultCodeNum = 1;
				g_stuXWFaultCodeRecv.CanData[0].id = 0x18FE27F3;
				memcpy(g_stuXWFaultCodeRecv.CanData[0].aucData,arr,8);
				g_stuXWFaultCodeUp.ucFaultCodeNum = 1;
				g_stuXWFaultCodeUp.CanData[0].id = 0x18FE27F3;
				memcpy(g_stuXWFaultCodeUp.CanData[0].aucData,arr,8);
			}
			else if(arr[0]<=15&&arr[4]<=15)
			{
                if(arr[0]>arr[4])
                {
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[1];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE27F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
					if(arr[0]==arr[1])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[1];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE27F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}
					}
				}
				else
				{
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[5];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE27F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
                    if(arr[4]==arr[5])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[5];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE27F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}					
                    }
				}
			}
			break;			
		case 0x18FE25F3: //故障码  大挖
		    if(g_stuXWFaultCodeRecv.ucVehicleType!=2)
				return;
			
            if(arr[0]<=1&&arr[4]<=1)
            {
                g_stuXWFaultCodeRecv.ucFaultCodeNum = 1;
				g_stuXWFaultCodeRecv.CanData[0].id = 0x18FE25F3;
				memcpy(g_stuXWFaultCodeRecv.CanData[0].aucData,arr,8);
				g_stuXWFaultCodeUp.ucFaultCodeNum = 1;
				g_stuXWFaultCodeUp.CanData[0].id = 0x18FE25F3;
				memcpy(g_stuXWFaultCodeUp.CanData[0].aucData,arr,8);
			}
			else if(arr[0]<=15&&arr[4]<=15)
			{
                if(arr[0]>arr[4])
                {
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[1];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE25F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
					if(arr[0]==arr[1])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[1];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE25F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}
					}
				}
				else
				{
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[5];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE25F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
                    if(arr[4]==arr[5])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[5];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE25F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}					
                    }
				}
			}
			break;
    	#endif
		
		case 0x1fffffff://测试 ID
			if((arr[0]==0x31)&&(arr[1]==0x61)&&(arr[2]==0x32)&&(arr[3]==0x62)&&(arr[4]==0x33)&&(arr[5]==0x63)&&(arr[6]==0x34)&&(arr[7]==0x64))
	        {
				buff[0]=101;
				buff[1]=53;
				buff[2]=102;
				buff[3]=54;
				buff[4]=103;
				buff[5]=55;
				buff[6]=104;
				buff[7]=56;
				CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1fffffff, 8, buff);
	        }
			break;

		case 0x1ffffffe://测试 ID
			if((arr[0]==0x31)&&(arr[1]==0x61)&&(arr[2]==0x32)&&(arr[3]==0x62)&&(arr[4]==0x33)&&(arr[5]==0x63)&&(arr[6]==0x34)&&(arr[7]==0x64))
	        {
				buff[0]=101;
				buff[1]=53;
				buff[2]=102;
				buff[3]=54;
				buff[4]=103;
				buff[5]=55;
				buff[6]=104;
				buff[7]=56;
				CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x1ffffffe, 8, buff);
			}
			break;

		default:
			break;
    }
}



/*********************************************************************************************************
*Function name	: MCU_TimerCount_Delay
*Description	:MCU模块计时函数，此函数100ms执行一次
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2015-8-7
*Modified		:
*********************************************************************************************************/
void  MCU_TimerCount_Delay(void)
{
	static uint8 usTime1s = 10;

	if(usTime1s)
	{
		usTime1s--;
	}
	else
	{
		usTime1s = 10;
        SYS_PowerOff_Reset();
	}
    MCU_SendCan1Data();
}


/*
*********************************************************************************************************
*Function name	:McuInit
*Description	:对Mcu模块相关变量初始化
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-6-9
*Modified		:
*********************************************************************************************************
*/
void McuInit()
{
	m_stuCommState.ucRcvCan1DataFlag = 2;
	McuHwInit();
	Mcu_CanOpen();
	//POWEROFF_485();
	m_stuMcuCmd.uiKey = 0x18FE0AF4;
}

#if 0
/*
*********************************************************************************************************
*Function name	:MCUCommStateCheck
*Description	:检查MCU通信状态是否改变，此函数1s执行一次
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-7-9
*Modified		:
*********************************************************************************************************
*/
void MCUCommStateCheck()
{
	static uint8 ucCan1CommStatePre;			//前一次的can通信状态
	static uint8 ucCan2CommStatePre;			//前一次的can通信状态
	static STUSYSMsgBus sysMsg;
	static uint8 can1rcvstatepre = 1;
	static uint8 can2rcvstatepre = 1;
	
	//if(!GetAccState())
		//return;
	//can1通信状态变化监测
	if(m_stuCommState.ucCan1CommState != ucCan1CommStatePre)
	{
		if(m_stuCommState.ucCan1CommState)
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN1COMM_ERR ;
		}
		else
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN1COMM_OK;
		}
		sysMsg.ucSrcDevice = 3;
		
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	ucCan1CommStatePre = m_stuCommState.ucCan1CommState;

	//can2通信状态变化监测
	if(m_stuCommState.ucCan2CommState != ucCan2CommStatePre)
	{
		if(m_stuCommState.ucCan2CommState)
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN2COMM_ERR;
		}
		else
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN2COMM_OK ;
		}
		sysMsg.ucSrcDevice = 3;
		
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	ucCan2CommStatePre = m_stuCommState.ucCan2CommState;
	

	//can1接收状态变化监测
	if((1==m_stuCommState.ucRcvCan1DataFlag)&&(2==can1rcvstatepre))//can1接收通信开始
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_START;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	else if((2==m_stuCommState.ucRcvCan1DataFlag)&&(1==can1rcvstatepre))//can1接收通信中断
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_STOP;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	can1rcvstatepre = m_stuCommState.ucRcvCan1DataFlag;

	//can2接收状态变化监测
	if((1==m_stuCommState.ucRcvCan2DataFlag)&&(2==can2rcvstatepre))//can2接收通信开始
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN2_RCV_START;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	else if((2==m_stuCommState.ucRcvCan2DataFlag)&&(1==can2rcvstatepre))//can1接收通信中断
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_STOP;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	can2rcvstatepre = m_stuCommState.ucRcvCan2DataFlag;
}
#endif

/*********************************************************************************************************
*Function name	:MCU_TimerCount_NoDelay
*Description	:MCU模块计时函数，此函数10ms执行一次
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2019-6-9
*Modified		:
*********************************************************************************************************/
void  MCU_TimerCount_NoDelay(void)
{
	static uint8  ucTime1s;
	
	CanRcvDataTimer();
	/*
    if(stuMcuSend_Gpsdata.ucCount)
    {
        stuMcuSend_Gpsdata.ucCount--;
		if(!stuMcuSend_Gpsdata.ucCount)
		{
            MCU_CmdOldLockSend();   //发送旧指令
		}
	}
	*/
	if(ucTime1s)
	{
	    ucTime1s--;
	}
	else
	{
		ucTime1s = 100;	
		MCUCommStateJudge();    // 此处添加1秒运行一次的函数
		//MCUCommStateCheck();
		//CanMcuCommManage();
		//FaultcodeCheck();
	}	

}

/*
*********************************************************************************************************
*Function name	:GetCanCommState
*Description	:获取GPS终端与MCU的通信状态
*Arguments  	:none
*Returns    	:  	00=通信正常
                	10=通讯异常
*Author			:hhm
*Date			:2016-8-19
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetCanCommState(uint8 channel)
{
	if(CAN_CHANNEL1==channel)
	{
		return m_stuCommState.ucCan1CommState;
	}
	else if(CAN_CHANNEL2==channel)
	{
		return m_stuCommState.ucCan2CommState;
	}
	else
		return 0;
}

/*
*********************************************************************************************************
*Function name	:GetCanBusDeviceState
*Description	:获取CAN总线上设备的通信状态
*Arguments  	:none
*Returns    	:B0:1=显示器异常，0=显示器正常，
*                B1:1=控制器异常，0=控制器正常，
*                B2:1=ECM异常，0=ECM正常

*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetCanBusDeviceState()
{
	return 0;
}


void Mcu_CanOpen()
{
	m_stuCommState.ucSleepState = 0;
	POWERON_CAN();
}

void Mcu_CanClose()
{
	m_stuCommState.ucSleepState = 1;
	POWEROFF_CAN();

}

//获取Can模块的休眠状态,1=休眠，０＝未休眠
uint8 Mcu_GetCanSleepState(void)
{
	return m_stuCommState.ucSleepState;
}

//返回值 1=蓄电瓶电压达到开机电压;0=蓄电瓶电压为未达到开机电压
BOOL MCU_GetVoltageState(void)
{
    if((GetInput_Voltage()>130&&GetInput_Voltage()<151)||GetInput_Voltage()>265)
        return 1;
    else
		return 0;   
}


/*
*********************************************************************************************************
*Function name	:TaskMCU
*Description	:处理MCU模块的任务
*Arguments  	:pdata	:传递给任务的数据
*Returns    	:none		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
void TaskMCU(void *pdata)
{
	extern MessageDetail MessageCAN1_0;      // 引用CAN0通道帧变量
	extern MessageDetail MessageCAN1_1;
    extern MessageDetail MessageCAN2_0;      // 引用CAN0通道帧变量
    extern MessageDetail MessageCAN2_1;
	
	pdata = pdata;
	
//	McuReadFromMemory();
    stu_McuFirmware.ucCANFrameFormat = STD_FRAME;

	while(1)
	{
		if(CAN1_DATA_OK==WaitForCAN1Message(2*OS_TICKS_PER_SEC))
		{
			if(MessageCAN1_0.CANID!=0)
			{
				DealCan1Message(MessageCAN1_0);
				MessageCAN1_0.CANID = 0; 
			}
			if(MessageCAN1_1.CANID!=0)
			{
				DealCan1Message(MessageCAN1_1);
				MessageCAN1_1.CANID = 0;
			}
			if(MessageCAN2_0.CANID!=0)
			{
				DealCan1Message(MessageCAN2_0);
				MessageCAN2_0.CANID = 0; 
			}
			if(MessageCAN2_1.CANID!=0)
			{
				DealCan1Message(MessageCAN2_1);
				MessageCAN2_1.CANID = 0;
			}				
		}
	}	
}



#ifdef  __cplusplus
}
#endif

#endif
