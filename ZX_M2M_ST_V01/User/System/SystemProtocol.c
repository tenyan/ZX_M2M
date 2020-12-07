/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: SystemProtocol.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为System功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-02, by  , 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------

#include "config.h"
#include "SystemProtocol.h"
#include "SystemCommand.h"

#define SYS_RECEIVDATAQ_SIZE	10							//System模块接收数据的消息队列	
OS_EVENT    *m_SYS_ReceivDataQ;               				//System模块接收数据的消息队列
void        *m_SYS_ReceivDataQBuff[SYS_RECEIVDATAQ_SIZE];	//消息对列深度为6
STUSystem g_stuSystem;
STU_SYSCounter SysCounter;
uint8   g_ucRstCount=0;            			//梯度重启计数器
uint16  g_usRstTime=GSM_RESTTIME;       	//梯度重启时间间隔
uint32  g_uiSysTime=0;       	            //存储器操作时间计时器


STUSYSParamSet g_stuSYSParamSet = {
	.aucDeviceID = {0xA0,0xC1,0x10,0x09,0,0,6},			//终端的ID
	.ucAPNLen = 5,
	.aucAPN = "CMNET",
	.aucUser = "hhm",						//M2M平台登录用户名
	.ucUserLen = 3,							//M2M平台登录用户名长度
	.aucPassword = {1,2,3},					//M2M平台登录密码
	.ucPasswordLen = 3,						//M2M平台登录密码长度
	.aucSmsCenterNum = "15150578385",		//短信中心号码
	.ucSmsCenterNumLen = 11,				//短信中心号码长度
	.aucHostIP = {58,218,196,200},    		//主中心IP地址
	.aucSpareHostIP = {218,80,94,178},		//副中心IP地址
	.usHostPort = 10004,              		//主中心udp端口
	.usSpareHostPort = 6601,             	//副中心tcp端口
	.ucSpareHostProtocolType = 1,			//副中心承载协议类型：0：UDP协议,1: TCP协议
	.ucHostProtocolType = 1,				//主中心承载协议类型：0：UDP协议,1: TCP协议
	.uiHeartbeatInterval=240,				//心跳间隔，单位：秒0x0000-不发送心跳,默认心跳间隔为30秒
	.usCanBrt = 0,							//本地CAN总线波特率
	.usCanFmt = 1,							//本地CAN报文格式
	//CAN ID 过滤配置，4字节一组
	.auiCanId = {
	             0x1ADC01C1,0x1ADC02C1,0x1ADC03C1,0x1ADC04C1,0x1ADC05C1,0x1ADC06C1,0x1ADC07C1,0x1ADC08C1,0x1ADC09C1,
				 0x1ADC0AC1,0x1ADC0BC1,0x1ADC0CC1,0x1ADC21C1,0x1ADC22C1,0x1ADC23C1,0x1ADC24C1,0x1ADC25C1,0x1ADC26C1,
				 0x1ADC27C1,0x1ADC28C1,0x1ADC29C1,0x1ADA23A1,
				 0x0CF00300,0x0CF00400,0x18FFF800,0x18FEDB00,0x18FED900,0x18FFE200,0x18FFF900,0x18FEEE00,0x18FEEF00,
				 0x18FEF200,0x18FEF500,0x18FEF600,0x18FEF700,0x18FEE900,0x18FED000,0x18FED200,0x18FF0300,0x18FFDC00,
				 0x18FEEB00,0x18FEDA00,0x18FF7A00,0x18FEB100,0x18E8E400,0x1CECFF00,0x1CEBFF00,0x18FECA00,0x18FECB00,				 
				 0x18FEE500,0x18FEDF00,0x18FE5600,0x18FD7C00,0x18FD2000,0x18FD3E00},
	.ucCanIdNum=55,							//can id 个数
	.usSleepBeforSlot = 300,       			//进入休眠时间,单位:s
	.usSleepWakeSlot =  115,
	.ucSSRepSlot = 180,
	.ucCanErrTime = 120,					//CAN故障判断时间
	.ucCanOKTime = 10,						//CAN恢复正常判断时间
	.ucPwrOffTime = 10,						//终端断电时间条件
	.ucPwrOnTime = 1,						//终端上电时间条件
	.ucPwrLowVol = 110,						//外部电源低电压报警阈值，单位：1%
	.ucPwrLowTime = 10,						//外部电源低电压报警的时间参数，单位：1s
	.ucBatLowVol = 35,						//内部电源低电压报警阈值，单位：1%
	.ucBatLowTime = 10,						//外部电源低电压报警的时间参数
	.ucGpsAntErrTime = 30,					//终端天线故障报警的时间参数，单位：1s
	.ucGpsAntOKTime = 30,					//终端天线故障报警的解除时间参数，单位：1s
	.ucGpsModuleErrTime = 30,				//终端GPS模块故障报警的时间参数
	.ucGpsModuleOKTime = 10,				//终端GPS模块故障报警解除的时间参数，单位：1s	
	.ucSpeedOver = 60,						//表示超速报警阈值，单位：1KM/H
    .ucSpeedOverLastTime = 20,				//超速报警的时间参数，单位：1s
    .ucTransportCar = 2,					//非自主移动（拖车）报警距离阈值，单位：1KM

	.ucDeviceWorkTimeRepCfg = 0x00,			//设备工作时间段统计配置参数
	.ucWorkDataRepModel = 0,				//工作参数（工况）数据单条上传模式
	.ucWorkDataRepInterval = 120,			//工作参数（工况）传输参数。时间，单位：10秒
	.ucPosiInforRepModel = 0,				//位置信息单条上传模式
	.ucPosiInforRepInterval = 60,			//位置信息上传间隔
};


uint8 aMsgSendData[1024];


uint16 f_usUpLoadCmdSn;				//上行命令的流水号

STU_SSData SSData = {
	.usTimer = 0,
	.ucTimeToRepFlag = 1,
	.ucRepeats = 0,
	.ucRepFlag = 0,
}; 
#if 0
stuWorkDataRep WorkDataRep = {
	.ucTrackModel = 0xfe,
};
#endif
stuConnect Connect;
stuPositionRep PositionTrack = {
	.ucTrackModel = 0xfe,
	.usTrackScope = 0,
};
stuHeartBeat HeartBeat; 
stuAlarmRep AlarmRep;
stuDTCRep DTCRep; 
stuFirmwareUpdate FirmwareUpdate = {
	.ucStep = 0,
};

stuSerAddr g_stuSerAddr;
//-----外部变量定义------------------------------------------------------------
extern STU_McuCmd m_stuMcuCmd;
extern uint8 Public_Buf[];
extern  STUSleep stuSleep; 
stuMcuResp McuResp;
extern STU_CanFrame CanData[];
extern stuXWFaultCode g_stuXWFaultCodeUp;
extern stuXWFaultCode g_stuXWFaultCodeRecv; 
extern STUMCUFirmware stu_McuFirmware;
extern STUKCMCUDownload stu_KCMCUDownload;


//从用户可选字节中读取看门狗状态
void ReadWatchDogState()
{
	uint8 ucTemp;
	
	ucTemp = FLASH_OB_GetUser();
	if(0!=(0x01 & ucTemp))
		g_stuSystem.ucWDTState = 0;
	else
		g_stuSystem.ucWDTState = 0xaa;
}

void SysCounterInit()
{
	SysCounter.usAlarmReportCounter = 0;
	SysCounter.usDailyReportCounter = 0;

	g_stuSystem.ucOnline = FALSE;
	g_stuSystem.ucSerAddrChangeFlag = 0;
	g_stuSystem.ucAccFlag = 0;
}
/******************************************************************************
** 函数名称: SYS_PutDataQ
** 功能描述: 调用OSQPost()发送消息队列
** 
** 输    入: cmd 发送数据指针
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_PutDataQ(void* ptr)
{
	uint8 ucTemp;
	
    ucTemp = OSQPost(m_SYS_ReceivDataQ, ptr);
	if(OS_ERR_NONE!=ucTemp)
		PC_SendDebugData((uint8 *)("GPRS POST ERR"), 13, DEBUG_ANYDATA);
}

//-----外部函数定义------------------------------------------------------------
/******************************************************************************
** 函数名称: SumDataCheck
** 功能描述: 对数据包累计和校验的核对函数
** 
** 输    入: add 指向校验数据包起始数据的指针,
             len 需要校验数据的长度
** 输    出: 无
** 返    回: 校验通过返回TRUE,不通过返回FALSE
**
** 作    者: Lxf
** 日    期: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8_t Check_DataSum(uint8_t *padd,uint16_t uslen)
{
    uint8_t ucSum;
    uint8_t i;

    ucSum = 0;
    for(i=0;i<uslen;i++)
    {
        ucSum+=*(padd+i);
    }
   	
    if(*(padd+uslen)==ucSum)
    {
        return TRUE;
    }
    return FALSE;
}


/******************************************************************************
** 函数名称: SYS_CreateReceivDataQ
** 功能描述: 创建消息邮箱m_SYS_ReceivDataQ,深度为6
** 输    入: cmd 发送数据指针
** 输    出: 无
** 返    回: 无
** 作    者: Lxf
** 日    期: 2011-06-22
*******************************************************************************/
void SYS_CreateReceivDataQ(void)
{
    m_SYS_ReceivDataQ = OSQCreate(&m_SYS_ReceivDataQBuff[0], SYS_RECEIVDATAQ_SIZE);
}

#if 0
void ClearNoUploadGprsTimer(void)
{
	HeartBeat.usNoUploadGprsTimer = 0;
}
#endif

/******************************************************************************
** 函数名称: BuildMsgHead
** 功能描述: 构造报文头
** 输    入:
** 输    出: 无
** 返    回: 报文头长度
** 作    者: hhm
** 日    期: 2016-08-18
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 BuildMsgHead(uint8 *buf,uint8 ucMsgType,uint16 usMsgBodyLen, uint8 ucFlag, uint16 usSn)
{
	uint16 usTemp;
	
	*buf++ = ucMsgType;
	memcpy(buf, g_stuSYSParamSet.aucDeviceID, 7);
	buf += 7;
	*buf++ = ucFlag;
	*buf++ = (usSn>>8) & 0xff;
	*buf++ =  usSn & 0xff;
	usTemp = usMsgBodyLen + 1;//+1为校验字
	*buf++ = (usTemp>>8) & 0xff;
	*buf++ =  usTemp & 0xff;
	return 13;
}

/******************************************************************************
** 函数名称: BuildState
** 功能描述: 构造状态位
** 输    入:
** 输    出: 无
** 返    回: 状态位
** 作    者: hhm
** 日    期: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 BuildState(void)
{
	uint32 uiTemp = 0;

    if(1==GPS_GetOrientState())
		uiTemp |= BIT(31);

	if(0!=stuSleep.ucWorkingMode)
		uiTemp |= BIT(28);
	if(0xaa==g_stuSystem.ucWDTState)	//BIT(27)【终端健康状态】 0：正常 1：存在告警异常
		uiTemp |= BIT(27);
	if(2==GetCanRcvState(CAN_CHANNEL1))	//【关联设备工作状态】 0：工作中 1：未工作
		uiTemp |= BIT(26);
	if(GetCanCommState(CAN_CHANNEL1))//【关联设备健康状态】0：正常 1：存在故障异常
		uiTemp |= BIT(25);
	if(1==GetAccState())				//ACC状态
		uiTemp |= BIT(24);
	if(GetPwrSupplyState())
		uiTemp |= BIT(23);
	if(GetPwrLowState())
		uiTemp |= BIT(22);
	if(GetBatChargeState())
		uiTemp |= BIT(21);
	if(GetBatVolLowState())
		uiTemp |= BIT(20);
	//if(GPS_GetGpsState())
	//	uiTemp |= BIT(19);
	if(GetGpsAntOpenState())
		uiTemp |= BIT(18);
	if(GetGpsAntShortState())
		uiTemp |= BIT(17);
	if(GetCanCommState(CAN_CHANNEL1))	//【终端总线通信中断标志】0：未中断 1：中断
		uiTemp |= BIT(16);									
										//15~8预留
//	if(GPS_GetSpeedoverState())			//【超速标志】0：未超速 1：已超速
//		uiTemp |= BIT(7);
										//1：拖车报警	
	if(GetBoxOpenState())				//【终端开盖标识】0：当前未开盖 1：当前开盖
		uiTemp |= BIT(0);
										
	return uiTemp;
}


uint8 BuildPositionInfor(uint8 *buf)
{
#if 1
	uint8 ucTemp = 0;
	uint32 uiTemp = 0;
	STU_Date date;
	/*
	if(0==GPS_GetLongitudeHemisphere())
		ucTemp |= 0x01;
	if(0==GPS_GetLatitudeHemisphere())
		ucTemp |= 0x10;
	*/
	*buf++ = ucTemp;
	
	//uiTemp = GPS_GetLatitude();
    *buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;
	//uiTemp = GPS_GetLongitude();
    *buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;

//	*buf++ = (GPS_GetSpeed()/10) & 0xff;
//	*buf++ = (GPS_GetForDirect()/2)&0xff;
//  *buf++ = (GPS_GetHeight()>>8)&0xff;
//	*buf++ =  GPS_GetHeight()&0xff;			//固定部分20+1字节
    buf += 4;
	
//	if(GPS_GetOrientState())
//		date = GPS_GetUtcTime();
//	else
		date = GetRTCTime_UTC();
	*buf++ =  date.ucYear;
	*buf++ =  date.ucMon;
	*buf++ =  date.ucDay;
	*buf++ =  date.ucHour;
	*buf++ =  date.ucMin;
	*buf++ =  date.ucSec;
#endif
    return 19;
}
#if 0
/******************************************************************************
** 函数名称: SYS_SendHeartBeatData
** 功能描述: 向平台发送PING心跳
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-7
*******************************************************************************/
void SYS_SendHeartBeatData()
{
	uint16 usSendLen;
	
    f_usUpLoadCmdSn++;
   	
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_PING_REQ, 0, 0, f_usUpLoadCmdSn);
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);                
}

/******************************************************************************
** 函数名称: HeartbeatRepManage
** 功能描述: 管理向平台发送PING心跳,此函数一秒运行一次
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-8-22
*******************************************************************************/
void HeartbeatRepManage()
{
	if(0!=HeartBeat.usTimer)
		HeartBeat.usTimer--;
	
	if(1==HeartBeat.ucRepFlag)
	{
		if(HeartBeat.ucRespondTimer>0)
		{
			HeartBeat.ucRespondTimer--;
		}
		else
		{
			if(HeartBeat.ucRepeats>=HEARTBEAT_MAX_REPEATS)
			{
				g_stuSystem.ucResetModem = 1;
				HeartBeat.ucRepeats = 0;
				PC_SendDebugData((uint8 *)("hbtrep failed"), 13, DEBUG_ANYDATA);
			}
			HeartBeat.ucRepFlag = 0;
		}
	}
}

/******************************************************************************
** 函数名称: SYS_LandonServer
** 功能描述: 向平台发送连接请求
** 输    入:
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-08-18
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendConnectData()
{
	uint16 usSendLen;
	uint8 *p, *P2;
	uint16 usTemp;
	
    f_usUpLoadCmdSn++;

	p = &aMsgSendData[MSG_HEAD_LEN];
   	*p++ = 0;					//协议名
	*p++ = 4;
	memcpy(p, "XM2M", 4);
	p += 4;
	*p++ = 1;					//协议版本
	*p++ = 0x10;	//TLV100D-当前软件版本号
	*p++ = 0x0d;
	*p++ = 0;
	*p++ = SW_VERSION_LEN;
	memcpy(p, SW_VERSION, SW_VERSION_LEN);
	p += SW_VERSION_LEN;
	*p++ = 0x01;//TLV-0x0111 ICCID
	*p++ = 0x11;
	*p++ = 0x00;
	*p++ = 20;
	P2 = GSM_GetICCID();
	memcpy(p, P2, 20);
	p += 20;
	*p++ = 0;	//连接标识
	usTemp = 36+SW_VERSION_LEN;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CONN_REQ, usTemp, 0, f_usUpLoadCmdSn);
	usSendLen += usTemp;
	*p = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);    
}


/******************************************************************************
** 函数名称: BasicWorkDataRepManage
** 功能描述: 管理向平台发送基本工作数据,此函数一秒运行一次
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-8-22
*******************************************************************************/
void SSDataRepManage()
{
	if(0!=SSData.usTimer)
		SSData.usTimer--;
	
	if(0==SSData.usTimer)
	{
		SSData.ucTimeToRepFlag = 1;
		if((0==PositionTrack.ucTrackModel) && (PositionTrack.usTrackScope>0))
		{
			if(PositionTrack.usTrackScope>=PositionTrack.ucTrackInterval)
				PositionTrack.usTrackScope -= PositionTrack.ucTrackInterval;
			else
				PositionTrack.usTrackScope = 0;
		}
		
		if((0x00==PositionTrack.ucTrackModel) && (0!=PositionTrack.usTrackScope))
		{
			SSData.usTimer = PositionTrack.ucTrackInterval;
		}
		else if(g_stuSYSParamSet.ucSSRepSlot>0)
		{
			SSData.usTimer = g_stuSYSParamSet.ucSSRepSlot;
		}
		else
		{
			SSData.usTimer = 30;
		}
	}
	
	if(1==SSData.ucRepFlag)
	{
		if(SSData.ucRespondTimer>0)
		{
			SSData.ucRespondTimer--;
		}
		else
		{
			if(SSData.ucRepeats>=SSDATA_MAX_REPEATS)
			{
				g_stuSystem.ucResetModem = 1;
				SSData.ucRepeats = 0;
				PC_SendDebugData((uint8 *)("ssrep failed"), 12, DEBUG_ANYDATA);
			}
			SSData.ucRepFlag = 0;
		}
	}
}
#endif
uint16 BuildSS(uint8 *buf)
{
	uint8 ucTemp;
	uint16 usTemp;
	uint32 uiTemp;
	uint16 usMsgBodyLen;
	uint8 ucTlvNmr = 0;
	uint8 *p = buf;
	uint8 *p2;
	uint8 num = 0;
	uint8 i;
	uint8 n=0;
    uint8 atemp[8]={0,0,0,0,0,0,0,0};
	uint32 uiID;
	uint8 ucFaultCodeNum = 0;
	
	*buf++ = 0x00;				//数据类型长度
	*buf++ = 0x02;
	*buf++ = 'S';
	*buf++ = 'S';
	buf++;						//数据内容长度
	buf++;
	buf++;						//状态同步TLV个数

	usMsgBodyLen = 7;

	*buf++ = 0x30;				//TLV1-状态位（0x3000）
	*buf++ = 0x00;
	*buf++ = 0;
	*buf++ = 4;
	uiTemp = BuildState();
	*buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;
	usMsgBodyLen += 8;
	ucTlvNmr++;

	*buf++ = 0x21;				//位置信息单包（0x2101）
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//主电源电压值（0x3004）
	*buf++ = 0x04;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetInput_Voltage();
	*buf++ =  (usTemp>>8) & 0xff;
	*buf++ =   usTemp     & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;

	*buf++ = 0x30;				//终端内置电池电压（0x3005）
	*buf++ = 0x05;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetBAT_Voltage();
	*buf++ =  0;
	*buf++ =   usTemp & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//GSM信号强度
	*buf++ = 0x07;
	*buf++ = 0;
	*buf++ = 1;
	//*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPS卫星颗数
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
//	*buf++ =  GPS_GetSatellitenums();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//ACC ON累计时间 （0x3016）
	*buf++ = 0x16;
	*buf++ = 0x00;
	*buf++ = 0x04;
	uiTemp = GetWorkingTime();
	*buf++ = (uiTemp>>24) & 0xff;
	*buf++ = (uiTemp>>16) & 0xff;
	*buf++ = (uiTemp>>8)  & 0xff;
	*buf++ =  uiTemp      & 0xff;
	usMsgBodyLen += 8;
	ucTlvNmr++;

	*buf++ = 0x30;//PPP（端对端协议）状态(0x3017)
	*buf++ = 0x17;
	*buf++ = 0;
	*buf++ = 1;
//	if(1==GSM_GetGsmModuleState())
		ucTemp =4;
//	else if(1==GSM_GetSimState())
		ucTemp = 6;
//	else
		ucTemp = 0;
	*buf++ = ucTemp;	
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;//GSM注册状态(0x3018)
	*buf++ = 0x18;
	*buf++ = 0;
	*buf++ = 1;
//	if(1==GSM_GetGsmRegState())
		ucTemp =1;
//	else 
		ucTemp = 0;
	*buf++ = ucTemp;
	usMsgBodyLen += 5;
	ucTlvNmr++;
	
	*buf++ = 0x30;//GPRS注册状态(0x3019)
	*buf++ = 0x19;
	*buf++ = 0;
	*buf++ = 1;
//	if(1==GSM_GetGprsState())
		ucTemp =1;
//	else 
		ucTemp = 0;
	*buf++ = ucTemp;	
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;//与平台连接状态(0x301A)
	*buf++ = 0x1A;
	*buf++ = 0;
	*buf++ = 1;
	if(1==g_stuSystem.ucOnline)
		ucTemp =1;
	else 
		ucTemp = 0;
	*buf++ = ucTemp;
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//Cellular ID，终端设备所在的小区标识
	*buf++ = 0x06;
	*buf++ = 0;
	*buf++ = 4;
//	uiTemp = GSM_GetCreg();
	*buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;
	usMsgBodyLen += 8;
	ucTlvNmr++;

	if(1==GetCanRcvState(CAN_CHANNEL1))
	{
		*buf++ = 0x21;//工作参数单包：CAN总线数据（0x2103）
		*buf++ = 0x03;
		p2 = buf;
		buf +=3;	  //TLV(0x2103)长度2byte + CAN数据包数1byte
		for(i=0; i<MAX_CAN_FRAME_NUM; i++)
		{
			if(CanData[i].id != 0)
			{
				*buf++ = (CanData[i].id>>24) & 0xff;
				*buf++ = (CanData[i].id>>16) & 0xff;
				*buf++ = (CanData[i].id>>8)  & 0xff;
				*buf++ =  CanData[i].id      & 0xff;
			//	CanData[i].id = 0;
				memcpy(buf, CanData[i].aucData, 8);
				buf+=8;
				num++;
			}
		}

		usTemp = num*12 + 1;
		
		*p2++  = (usTemp>>8) & 0xff;   //TLV(0x2103)长度
		*p2++  =  usTemp     & 0xff;
		*p2 = num;	//can帧个数
		if(0!=num)
		{
		    usMsgBodyLen += usTemp+4;
		    ucTlvNmr++;
		}
		
		//故障代码
		*buf++ = 0x30;
		*buf++ = 0xF0;
		p2 = buf;
		buf +=3;	  //TLV(0x30F0)长度2byte + CAN数据包数1byte
		ucFaultCodeNum = 0;      //清0
		for(n=0;n<g_stuXWFaultCodeUp.ucFaultCodeNum;n++)
		{
            if(g_stuXWFaultCodeUp.CanData[n].id!=0)
            {
				*buf++ = (g_stuXWFaultCodeUp.CanData[n].id>>24) & 0xff;
				*buf++ = (g_stuXWFaultCodeUp.CanData[n].id>>16) & 0xff;
				*buf++ = (g_stuXWFaultCodeUp.CanData[n].id>>8)  & 0xff;
				*buf++ =  g_stuXWFaultCodeUp.CanData[n].id & 0xff;
			
				memcpy(buf, g_stuXWFaultCodeUp.CanData[n].aucData, 8);
				buf+=8;
				ucFaultCodeNum++;				
			}
		}
		
		if(ucFaultCodeNum==0&&num)
		{
		    if(g_stuXWFaultCodeRecv.ucVehicleType==2)
				uiID = 0x18FE25F3;
			else
				uiID = 0x18FE27F3;
			*buf++ = (uiID>>24) & 0xff;
			*buf++ = (uiID>>16) & 0xff;
			*buf++ = (uiID>>8)  & 0xff;
			*buf++ =  uiID & 0xff;
		   
			memcpy(buf, atemp, 8);
			buf+=8;
			ucFaultCodeNum++;	            
		}
		usTemp = ucFaultCodeNum*12 + 1;
		
		*p2++  = (usTemp>>8) & 0xff;   //TLV(0x2103)长度
		*p2++  =  usTemp     & 0xff;
		*p2 = ucFaultCodeNum;	//can帧个数
		if(0!=ucFaultCodeNum)
		{
		    usMsgBodyLen += usTemp+4;
		    ucTlvNmr++;
		}			
	}

	p += 4;
	usTemp = usMsgBodyLen - 6;
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}
#if 0
/******************************************************************************
** 函数名称: SYS_SendSS
** 功能描述: 终端基本状态同步数据上传
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-8-24
*******************************************************************************/
void SYS_SendSS()
{
	uint16 usMsgBodyLen, usSendLen;
	
    f_usUpLoadCmdSn++;
	SSData.usRepSn = f_usUpLoadCmdSn;
	usMsgBodyLen = BuildSS(&aMsgSendData[MSG_HEAD_LEN]);
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_PUSH_DATA, usMsgBodyLen, 0, f_usUpLoadCmdSn);
	usSendLen += usMsgBodyLen;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);      
}

/******************************************************************************
** 函数名称: ConnectManage
** 功能描述: 管理向平台发送连接指令,此函数一秒运行一次
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-9
*******************************************************************************/
void ConnectManage()
{
	if(1==Connect.ucRepFlag)
	{
		if(Connect.ucRespondTimer>0)
		{
			Connect.ucRespondTimer--;
		}
		else
		{
			if(Connect.ucRepeats>=CONNECT_MAX_REPEATS)
			{
				g_stuSystem.ucResetModem = 1;
				Connect.ucRepeats = 0;
				PC_SendDebugData((uint8 *)("connet failed"), 13, DEBUG_ANYDATA);
			}
			Connect.ucRepFlag = 0;
		}
	}
}

/******************************************************************************
** 函数名称: AlarmDataRepManage()
** 功能描述: 管理向平台发送工作数据,此函数一秒运行一次
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-13
*******************************************************************************/
void AlarmDataRepManage()
{
	//uint8 i;
	/*
	if(0xff==AlarmRep.ucRepIndex)
	{
		for(i=0; i<ALARM_MAX_NUM; i++)
		{
			if((0==AlarmRep.Alarm[i].ucState) && (0!=AlarmRep.Alarm[i].ucType))
			{
				AlarmRep.ucRepIndex = i;
			}
		}
	}
	*/
	if(1==AlarmRep.ucRepFlag)
	{
		if(AlarmRep.ucRespondTimer>0)
		{
			AlarmRep.ucRespondTimer--;
		}
		else
		{
			if(AlarmRep.ucRepeats>=ALARM_MAX_REPEATS)
			{
				g_stuSystem.ucResetModem = 1;
				AlarmRep.ucRepeats = 0;
				AlarmRep.ucRespondTimer = ALARM_RESPOND_TIMEOUT;	
				PC_SendDebugData((uint8 *)("alrep failed"), 12, DEBUG_ANYDATA);
			}
			AlarmRep.ucRepFlag = 0;
		}
	}
}


/******************************************************************************
** 函数名称: AddToAlarmList
** 功能描述: 当报警产生或消失时，添加到报警列表中来
** 输    入: type:报警类型,如下:
            0x01--设备非自主移动报警（功能提醒）
			0x02--出围栏（功能提醒）
			0x03--设备超速报警（功能提醒）
			0x04--终端与设备的总线通信故障(终端硬件报警)
			0x05--GPS模块故障(终端硬件报警)
			0x06--GPS天线故障(终端硬件报警)
			0x07--终端外部电源低电压 (终端硬件报警)
			0x08--终端外部电源断电(终端硬件报警)
			0x09--终端内部电池低电压(终端硬件报警)
			0x0A--SIM卡换卡报警（终端硬件报警）
			0x0B--GPS信号强度弱(终端硬件报警)
			0x0C--GPRS信号强度弱(终端硬件报警)
			0x0D--GSM/GPRS模块故障

			 flag:产生与消失标识,0=产生, 1=消失
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-9-23
*******************************************************************************/
void AddToAlarmList(uint8 type, uint8 flag)
{
	uint8 ucTemp;
	
	if(0==flag)
		ucTemp = type | BIT(8);
	else
		ucTemp = type;
	if((ucTemp==AlarmRep.Alarm[type].ucType) && (1==AlarmRep.Alarm[type].ucState))
		return;
	AlarmRep.Alarm[type].ucType = ucTemp;
	AlarmRep.Alarm[type].ucState = 0;
	AlarmRep.ucNewAlarmFlag = 1;
}


/******************************************************************************
** 函数名称: SYS_SendAlarmData
** 功能描述: 向平台发送报警数据
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-9
*******************************************************************************/
void SYS_SendAlarmData()
{
	uint8 ucTemp = 0;
	uint16 usSendLen;
	uint8 i;
	uint8 *p;
	uint8 *buf;

	
    f_usUpLoadCmdSn++;
	AlarmRep.usRepSn = f_usUpLoadCmdSn;
	aMsgSendData[MSG_HEAD_LEN] = 0;
	aMsgSendData[MSG_HEAD_LEN+1] = 2;
	aMsgSendData[MSG_HEAD_LEN+2] = 'D';
	aMsgSendData[MSG_HEAD_LEN+3] = 'A';
		//数据内容长度MSBs
		//数据内容长度LSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+6];
	*buf++ = 0x30;
	*buf++ = 0x0d;
	p = buf;		//数据长度
	buf += 2;
	for(i=0; i<ALARM_MAX_NUM; i++)
	{
		if((0==AlarmRep.Alarm[i].ucState) && (0!=AlarmRep.Alarm[i].ucType))
		{
			*buf++ = AlarmRep.Alarm[i].ucType;
			ucTemp++;
		}
	}
	if(0==ucTemp)
		return;
	*p++ = 0;
	*p++ = ucTemp;
	aMsgSendData[MSG_HEAD_LEN+4] = 0;
	aMsgSendData[MSG_HEAD_LEN+5] = ucTemp+10;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_ALERT, ucTemp+10, 0, f_usUpLoadCmdSn);
	usSendLen += ucTemp+10;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);      
}


/******************************************************************************
** 函数名称: DTCRepManage()
** 功能描述: 管理向平台发送DTC数据,此函数一秒运行一次
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-13
*******************************************************************************/
void DTCRepManage()
{
	
	if(1==DTCRep.ucRepFlag)
	{
		if(DTCRep.ucRespondTimer>0)
		{
			DTCRep.ucRespondTimer--;
		}
		else
		{
			if(DTCRep.ucRepeats>=DTC_MAX_REPEATS)
			{
				g_stuSystem.ucResetModem = 1;
				DTCRep.ucRepeats = 0;
				DTCRep.ucRespondTimer = DTC_RESPOND_TIMEOUT;
				PC_SendDebugData((uint8 *)("dtcrep failed"), 13, DEBUG_ANYDATA);
			}
			DTCRep.ucRepFlag = 0;
		}
	}
}


/******************************************************************************
** 函数名称: SYS_SendAlarmData_DTC
** 功能描述: 向平台发送报警数据
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-9
*******************************************************************************/
void SYS_SendDTCData()
{
	uint8 ucTemp = 0;
	uint16 usSendLen;
	uint8 *p, *p2;
	uint8 *buf;

	
    f_usUpLoadCmdSn++;
	AlarmRep.usRepSn = f_usUpLoadCmdSn;
	aMsgSendData[MSG_HEAD_LEN] = 0;
	aMsgSendData[MSG_HEAD_LEN+1] = 3;
	aMsgSendData[MSG_HEAD_LEN+2] = 'D';
	aMsgSendData[MSG_HEAD_LEN+3] = 'T';
	aMsgSendData[MSG_HEAD_LEN+4] = 'C';
		//数据内容长度MSBs
		//数据内容长度LSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+7];
	*buf++ = 0x30;
	*buf++ = 0xEF;
	p = buf;		//数据长度
	buf += 2;
	p2= buf;		//DTC个数
	buf += 1;
	ucTemp = GetMcuFaultCode(buf);
	if(0==ucTemp)
		return;
	*p2 = ucTemp;
	*p++ = 0;
	*p++ = ucTemp*4+1;
	
	aMsgSendData[MSG_HEAD_LEN+5] = 0;
	aMsgSendData[MSG_HEAD_LEN+6] = ucTemp*4+12;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_ALERT, ucTemp*4+12, 0, f_usUpLoadCmdSn);
	usSendLen += ucTemp*4+12;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);      
}

/******************************************************************************
** 函数名称: SYS_SendMcuRespData
** 功能描述: 向平台发送MCU对平台命令响应的数据
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-9
*******************************************************************************/

void SYS_SendMcuRespData()
{
	uint8 *pBuf = &aMsgSendData[MSG_HEAD_LEN];
	uint32 uiTemp;
	uint16 usSq, usSendLen;
	
	if(m_stuMcuCmd.ucRespSerFlag & BIT(0))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(0);
		usSq = m_stuMcuCmd.usLockOneSq;
	}
	else if(m_stuMcuCmd.ucRespSerFlag & BIT(1))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(1);
		usSq =  m_stuMcuCmd.usLockSecSq;
	}
	else if(m_stuMcuCmd.ucRespSerFlag & BIT(2))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(2);
		usSq = m_stuMcuCmd.usUnLockSq;
	}
	else if(m_stuMcuCmd.ucRespSerFlag & BIT(3))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(3);
		usSq = m_stuMcuCmd.usUnLockSq;
	}
	else if(m_stuMcuCmd.ucRespSerFlag & BIT(4))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(4);
		usSq = m_stuMcuCmd.usMonitorSq;
	}
	else if(m_stuMcuCmd.ucRespSerFlag & BIT(5))
	{
		m_stuMcuCmd.ucRespSerFlag &= ~BIT(5);
		usSq = m_stuMcuCmd.usMonitorSq;
	}
	else
	{
		return;
	}
	McuSaveToMemory();
	*pBuf++ = (usSq>>8) & 0xff;
	*pBuf++ =  usSq     & 0xff;
	*pBuf++ = 0x00;
	*pBuf++ = 0x02;
	*pBuf++ = 'R';
	*pBuf++ = 'C';
	*pBuf++ = 0x00;
	*pBuf++ = 0x05;
	if(m_stuMcuCmd.ucCmdCheckFlag==52)   //校验失败
	    *pBuf++ = 1;
	else
	    *pBuf++ = 0;
	uiTemp = BuildState();
	*pBuf++ = (uiTemp>>24) & 0xff;
	*pBuf++ = (uiTemp>>16) & 0xff;
	*pBuf++ = (uiTemp>>8)  & 0xff;
	*pBuf++ =  uiTemp      & 0xff;

	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, 13, 0, usSq);
	usSendLen += 13;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	usSendLen += 1;
	GSM_SendGprs(aMsgSendData, usSendLen, 0);
}
/******************************************************************************
** 函数名称: SYS_SendMcuRespData
** 功能描述: 向平台发送MCU对平台命令响应的数据
** 输    入: 无
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-9
*******************************************************************************/
void McuRespManage()
{
	if(McuResp.ucRespondTimer)
		McuResp.ucRespondTimer--;
}

/******************************************************************************
** 函数名称: SYS_NetDataSend
** 功能描述: 向GSM模块发送心跳及MCU数据
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-07-23
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendNetData(void)
{	
    if(1==g_stuSystem.ucResetModem)       //需要重新启动GSM模块
    {
        g_stuSystem.ucResetModem = 0;
		Connect.ucSucceedFlag = 0;
		g_stuSystem.ucOnline = 0;
        GsmLedOff();                         //关闭GSM指示灯
        GSM_Reset();                        //重启GSM模块
        return;
    }

	if(GPRS_LINK_STATE_CLOSED==GSM_GetLinkState(0))
	{
		if(1==Connect.ucSucceedFlag)
			Connect.ucSucceedFlag = 0;
		if(1==g_stuSystem.ucOnline)
			g_stuSystem.ucOnline = 0;
	}
	
	if(GPRS_LINK_STATE_READY != GSM_GetLinkState(0))
	{
		return;
	}

	//远程升级
	if(FirmwareUpdate.ucStep > 0)
	{
		UpgradeManage();
		return;
	}

	//向平台发送连接指令
	if(1!=Connect.ucSucceedFlag)
	{
		if(1!=Connect.ucRepFlag)
		{
			if(0==Connect.ucRespondTimer)
			{
				Connect.ucRespondTimer = CONNECT_RESPOND_TIMEOUT;
				Connect.ucRepFlag = 1;
				Connect.ucRepeats++;
				SYS_SendConnectData();
			}
		}
		return;
	}

	//向平台发送状态同步数据
	if((1==SSData.ucTimeToRepFlag) && (0==SSData.ucRepFlag))
	{
		SSData.ucTimeToRepFlag = 0;
		
		SSData.ucRepeats++;
		SSData.ucRepFlag = 1;
		SSData.ucRespondTimer = SSDATA_RESPOND_TIMEOUT;
		SYS_SendSS();
		return;
	}	
	

	//向平台发送心跳通讯指令
	if(g_stuSYSParamSet.uiHeartbeatInterval > 0)
	{
	    if(0==HeartBeat.usTimer)
		{
			HeartBeat.usTimer = g_stuSYSParamSet.uiHeartbeatInterval;
			HeartBeat.ucRepFlag = 1;
			HeartBeat.ucRespondTimer = HEARTBEAT_RESPOND_TIMEOUT;
			SYS_SendHeartBeatData();
			return;
		}
	}
	
	if((0!=m_stuMcuCmd.ucRespSerFlag) && (0==McuResp.ucRespondTimer))
	{
		McuResp.ucRespondTimer = 3;
		SYS_SendMcuRespData();
		return;
	}	

}
#endif

/******************************************************************************
** 函数名称: SYS_LedDisplay
** 功能描述: 对设备指示灯的控制和显示,该函数需要每秒钟执行一次
** GPS指示灯:
   GSM指示灯:
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-07-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_LedDisplay(void)
{
//	static BOOL ucOnline = FALSE;
    static uint8 ucPositionFlag = TRUE;
	static uint8 ucCan1Flag = 0;
	static uint8 ucCan2Flag = 0;

	//GPS指示灯控制
	if(stuSleep.ucGpsSleepState)
    {
       	GpsLedOff();                     //休眠GPS指示灯灭
    }
	else if(GPS_GetAnteState())  
	{
        GpsLedOn();                      //GPS天线未接(长亮)
	}
	else
	{
	    if(1==GPS_GetOrientState())
	    {
	        if(ucPositionFlag==TRUE)
	        {
	            ucPositionFlag = FALSE;
	            GpsLedOff();                    //灯灭
	        }
	        else
	        {
	            ucPositionFlag = TRUE;
	            GpsLedOn();                    //灯亮
	        }          
	    }
		else
		{
			GpsLedOff();                       //GPS正常 (长灭)
		}
	}

#if 0
	//CAN1指示灯控制
	if(1==Mcu_GetCanSleepState())
	{
		CAN1_LED_OFF();
	}
	else if(1==GetCanRcvState(CAN_CHANNEL1))
	{
		if(ucCan1Flag==0)
		{
			ucCan1Flag = 1;
			CAN1_LED_OFF();
		}
		else
		{
			ucCan1Flag = 0;
			CAN1_LED_ON();
		}
	}
	else if(0==GetCanCommState(CAN_CHANNEL1))
	{
		CAN1_LED_OFF();
	}
	else
	{
		CAN1_LED_ON();
	}
#endif
	//CAN1指示灯控制
	if(1==Mcu_GetCanSleepState())
	{
		CAN1_LED_OFF();
	}
	else if(1==GetCanRcvState(CAN_CHANNEL1))
	{
		if(ucCan1Flag==0)
		{
			ucCan1Flag = 1;
			CAN1_LED_OFF();
		}
		else
		{
			ucCan1Flag = 0;
			CAN1_LED_ON();
		}
	}
	else
	{
       CAN1_LED_ON();
	}

	//CAN2指示灯控制
	if(1==Mcu_GetCanSleepState())
	{
		CAN2_LED_OFF();
	}
	else if(1==GetCanRcvState(CAN_CHANNEL2))
	{
		if(ucCan2Flag==0)
		{
			ucCan2Flag = 1;
			CAN2_LED_OFF();
		}
		else
		{
			ucCan2Flag = 0;
			CAN2_LED_ON();
		}
	}
	else
	{
        CAN2_LED_ON();
	}	

	//WiFi
	WiFi_LEN_OFF();
	//ETH
	ETHER_LED_OFF();
	POWER_LED_ON();
}


#if 0
/*外电源指示灯函数,该函数每0.5秒执行一次
通过蓝牙指示灯快闪（闪烁周期4秒，每次闪烁0.5秒）指示终端主电源掉电*/
void SYS_POWERLED_Display(void)
{
    static uint8 ucOffcount = 8;
	static uint8 ucBluFlag = 0;  //0-灯灭,1=灯亮
    if(0==GetPwrSupplyState())
    {
        BLU_LED_OFF();           //主电源供电 长灭
	}
	else
	{
        if(ucBluFlag)
        {
            ucBluFlag = 0;
            BLU_LED_OFF();
			ucOffcount = 8;
		}
		else
		{
            if(ucOffcount)
            {
                if(!(--ucOffcount))
                {
                    ucBluFlag = 1;
                    BLU_LED_ON();
				}
			}        
		}
	}
}
#endif
/******************************************************************************
** 函数名称: SYS_Reset
** 功能描述: 系统复位
** 输    入: delay--延时复位时间
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Reset(uint8 delay)
{
	g_stuSystem.ucRstTime = delay;
}

/******************************************************************************
** 函数名称: SYS_Shutdown
** 功能描述: 系统关机
** 输    入: delay--延时复位时间
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Shutdown(uint8 delay)
{
	g_stuSystem.ucShutDownTime = delay;
}


/******************************************************************************
** 函数名称: SYS_TimerCount
** 功能描述: System模块计时函数,调用基数为10ms,同时包含50ms、100ms、1s共4种计数方式
** 
** 输    入: 
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_TimerCount_NoDelay(void)
{

    static uint16 us2s = 200;
    static  uint8 uc1s = 100;
	
    //GoToUpdate();   
	/*
    if(uc50ms)             //50ms基数
    {
        if(!(--uc50ms))
        {
            uc50ms = 5;
        }
    }

    if(uc100ms)            //100ms基数
    {
        if(!(--uc100ms))
        {
            uc100ms = 10;
        }
    }    
	*/
	if(us2s)          //基数2秒
    {
        if(!(--us2s))
        {
            us2s = 200;
		//	DealUnNormData();
        }
    } 
    if(uc1s)               //1s基数
    {
        if(!(--uc1s))
        {
            uc1s = 100;

            if(g_stuSystem.ucRstTime)           //使系统复位
            {
                if(!(--g_stuSystem.ucRstTime))
                {
                   while(1);
                }
            }
			
            if(g_stuSystem.ucShutDownTime)  //关机
            {
                if(!(--g_stuSystem.ucShutDownTime))  
                {
                	MainPowOff();
				//	BatPowOff();      //关机
					PSWControl(1);
					PC_SendDebugData((uint8 *)("Power Off"), 9, DEBUG_ANYDATA);
				}
            }

			//心跳发送计时
            if(HeartBeat.usNoUploadGprsTimer<MAX_NO_UPLOAD_GPRS_TIME)
            {
                HeartBeat.usNoUploadGprsTimer++; 
			}
			/*
			if(SysCounter.usDailyReportCounter!=0)
				SysCounter.usDailyReportCounter--;
			*/
            SYS_LedDisplay();                    //LED 指示灯显示  
		//	ConnectManage();
		//	SSDataRepManage();
		//	HeartbeatRepManage();

        //  AlarmDataRepManage();
		//	DTCRepManage();
        //  	UpgradeTimer();
		//	SerAddrChangeGsmRst();
			SYS_WorkModeSleep_Count();
        //  McuRespManage();	
		//	SIMCardStateJudge();
        }
    }
}

void SYS_TimerCount_Delay(void)
{

}
#if 0
/******************************************************************************
** 函数名称: DealGpsMessage
** 功能描述: 
** 
** 输    入: PtrTxt:指针
             usPtrTxtLen:整包数据的长度,
             ucKind:消息数据类型 ,报警或者恢复
** 输    出: 无
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据，
**
** 作    者: Lxf
** 日    期: 2011-07-31
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 DealGpsMessage(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind)
{

    switch(ucKind)
	{
		case GPS_MSG_WARN_SPEED_VOER:
			AddToAlarmList(0x03, 0);
            SSData.usTimer = 0;
			break;
		case GPS_MSG_WARN_SPEED_OK:
			AddToAlarmList(0x03, 1);
            SSData.usTimer = 0;
			break;
		case GPS_MSG_WARN_ANTENNA_ERROR:
			AddToAlarmList(0x06, 0);
            SSData.usTimer = 0;
			break;
		case GPS_MSG_WARN_ANTENNA_OK:
			AddToAlarmList(0x06, 1);
            SSData.usTimer = 0;
			break;
		default:
			break;
	}
    return 0;
}

/******************************************************************************
** 函数名称: SYS_CollectModule_ReceiveData_Execution
** 功能描述: 
** 
** 输    入: PtrTxt:指针
             usPtrTxtLen:整包数据的长度,
             ucKind:消息数据类型 ,报警或者恢复
** 输    出: 无
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据，
**
** 作    者: Lxf
** 日    期: 2011-07-31
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 SYS_CollectModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind)
{
    switch(ucKind)
    {
    	case KeyON:
			SSData.usTimer = 0;
	//		WorkDataRep.usTimer = 7;
			break;
		case KeyOFF:
			SSData.usTimer = 0;
	//		WorkDataRep.usTimer = 2;
			break;	
        case BOXAlarm:
			AddToAlarmList(0x0E, 0);
			SSData.usTimer = 0;
			break;
		case BOXNormal:
			AddToAlarmList(0x0E, 1);
			SSData.usTimer = 0;
			break;	
		case PowerLow: 
			AddToAlarmList(0x07, 0);
			SSData.usTimer = 0;
        	break;
		case PowerNormal:
			AddToAlarmList(0x07, 1);
			SSData.usTimer = 0;
        	break;
 		case GET5V:
			AddToAlarmList(0x08, 1);
			SSData.usTimer = 0;
			break;
		case NO5V:
			AddToAlarmList(0x08, 0);
			SSData.usTimer = 0;
			break;
		default:
			break;
    }
    return 0;
}

/******************************************************************************
** 函数名称: SYS_MCUModule_ReceiveData_Execution
** 功能描述: 
** 
** 输    入: PtrTxt:指针
             usPtrTxtLen:整包数据的长度,包括MCUCMD_ID 长度+MCUCMD内容长度
             ucKind:消息数据类型 1:can通信变化;2:RS232通信变化;
             3:对服务器发送给MCU的命令的回复;4:故障码发生变化;5:MCU数据变化
** 输    出: 无
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据，
**
** 作    者: Lxf
** 日    期: 2011-07-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 SYS_MCUModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind)
{
    switch(ucKind)
	{
		case MCU_MSG_KIND_CAN1COMM_ERR:  //MCU状态改变,处理MCU故障代码触发信息
	        AddToAlarmList(0x04, 0);
			SSData.usTimer = 0;
			break;
		case MCU_MSG_KIND_CAN1COMM_OK:   //MCU状态改变,处理MCU故障代码触发信息
            SSData.usTimer = 1;
	        AddToAlarmList(0x04, 1);
	//		WorkDataRep.usTimer = 7;		//can通信开始触发工作参数
			break;	
		case MCU_MSG_KIND_CAN1_RCV_START:
	//		WorkDataRep.usTimer = 7;		//can通信开始触发工作参数
	        SSData.usTimer = 7;
			break;
//		case  MCU_MSG_KIND_CAN2_RCV_START:
			//AddToAlarmList(5, 1);
		//	WorkDataRep.usTimer = 0;		//can通信开始触发工作参数
//			break;
		case  MCU_MSG_KIND_CAN1_RCV_STOP:
		case  MCU_MSG_KIND_CAN2_RCV_STOP:	
			//AddToAlarmList(5, 0);
			break;
		default:
			break;
	}
	return 0;
}
#endif

/******************************************************************************
** 函数名称: TaskSYS
** 功能描述: System模块任务函数
** 
** 输    入: 
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void TaskSYS(void *pdata)
{
    uint8   err;
    uint16  usDataLen; 
    PSTUSYSMsgBus pSTUSYS_MsgBus;


    pdata = pdata;
	SYS_ParamRead();
    SYS_CreateReceivDataQ();              //创建消息邮箱
  //  SysCounterInit();
	ReadWatchDogState();
	
    while(1)
    {	     
        pSTUSYS_MsgBus = (PSTUSYSMsgBus)OSQPend(m_SYS_ReceivDataQ, 10, &err);
        if (err == OS_ERR_NONE)             //对接收到的消息数据进行处理
        {           
            usDataLen = 0;                        
            switch(pSTUSYS_MsgBus->ucSrcDevice)
            {  
            #if 0
                case SRCDEVICE_ID_GSM:    //GSM来源数据/设置程序
                    if(GSM_MSG_GPRS_RECV==pSTUSYS_MsgBus->ucKind)//GPRS消息、短信消息
                    {
						usDataLen = DealSerCmd(pSTUSYS_MsgBus->pMsgPacket, pSTUSYS_MsgBus->usSize,pSTUSYS_MsgBus->ucSrcDevice,pSTUSYS_MsgBus->ucKind);
                    }
					else if(GSM_MSG_SMS_RECV==pSTUSYS_MsgBus->ucKind)
					{
						#if 0
						OSTimeDly(200);
						usDataLen = SYS_SMS_CommandAll_Execution_Universal(pSTUSYS_MsgBus->pMsgPacket, 
							             pSTUSYS_MsgBus->usSize, pSTUSYS_MsgBus->ucSrcDevice,
							             pSTUSYS_MsgBus->ucKind);
						#endif
					}
 					else if(GSM_MSG_RING_RECV==pSTUSYS_MsgBus->ucKind)//电话
                    {
                        //g_stuSystem.ucUploadType = HEARTBEAT_PHONECALL;
                        //g_stuSystem.stuParamTem.uiHeartbeatSlot =1;
                    }
                    else if(GSM_MSG_ONLINE==pSTUSYS_MsgBus->ucKind)//上线
                    {
                        //g_stuSystem.stuHearBeat.ucGprsConnect = 1;
                       // g_stuSystem.ucUploadType = HEARTBEAT_TIME;
                        //g_stuSystem.stuParamTem.uiHeartbeatSlot = 0;
                    }
					else if(GSM_MSG_FTP_RECV==pSTUSYS_MsgBus->ucKind)
					{
						if(pSTUSYS_MsgBus->usSize > 0)
						{
						//	SaveFirmwareUpdatePack();
						}
					}
                    break;  
					#endif
                case SRCDEVICE_ID_SETTOOL:
					 usDataLen = DealSerCmd(pSTUSYS_MsgBus->pMsgPacket, pSTUSYS_MsgBus->usSize,pSTUSYS_MsgBus->ucSrcDevice,pSTUSYS_MsgBus->ucKind);
                    break;  
	
                default:
                    break;
            }
        }  
        A5_MCUSendCANDataToA5();
 		SYS_WorkMode_Exec();          //工作模式处理
    }
}


/******************************************************************************
** 函数名称: SYS_PowerOff_Reset
** 功能描述: 终端断电重启函数，每1S执行一次
**           当ACC关闭持续时间达到休眠唤醒间隔时长时，通知AM1805对整机进行断电10秒钟
             然后重新上电;之后如果ACC仍持续为OFF状态，则每间隔24小时通知通知AM1805对
             整机进行断电10秒钟然后重新上电。
           
** 输    入: 
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2017-03-24
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_PowerOff_Reset(void)
{
    static uint8 ucAccPre = 0;
//	static uint8 ucAccFlag = 0;    //ACC由ON变为OFF标志位

	if((g_stuSystem.uiAccOffTimer>=g_stuSYSParamSet.usSleepWakeSlot*60||
		g_stuSystem.uiResetTimer>=86400)&&!GetAccState())
	{     
        PC_SendDebugData((uint8 *)("RTCPWROFF"), 9, DEBUG_ANYDATA);

		PSWControl(1);            //通知AM1805断开终端外部和电池电源
        RTCSetSleepTime(10, 2);   //通知AM1805(RTC芯片)断电10秒钟
		MainPowOff(); 
		OSTimeDly(OS_TICKS_PER_SEC*3);
	//	BatPowOff();
    	g_stuSystem.uiAccOffTimer = 0;
		g_stuSystem.uiResetTimer = 0;
	}

    if(GetAccState()!=ucAccPre)
    {
        if(!GetAccState())
			g_stuSystem.ucAccFlag = 1;
	}
    if(!GetAccState()&&g_stuSystem.ucAccFlag==1)
    {
    	g_stuSystem.uiAccOffTimer++;
	}
	else
	{
    	g_stuSystem.uiAccOffTimer = 0;

	}

	if(!GetAccState())
	{
    	g_stuSystem.uiResetTimer++;
	}
	else
	{
    	g_stuSystem.uiResetTimer = 0;
	}
	ucAccPre = GetAccState();
}
#if 0
//GPS由不定位到定位触发数据上报
void Sys_GPSState_Change(void)
{
    static uint8 ucGPSStatePre = 0;
    static uint8 ucACCPre = 0;
	static uint8 ucSendCount = 3;

	if(ucACCPre!=GetAccState())
	{
        if(GetAccState())
			ucSendCount = 0;
		else
			ucSendCount = 3;
	}
	
    if(ucGPSStatePre!=GPS_GetOrientState())
    {
        if((1==GPS_GetOrientState())&&ucSendCount<5)
        {
            ucSendCount++;
			SSData.usTimer = 3;              //触发数据上报
            PC_SendDebugData((uint8 *)("GPS_DingW"), 9, DEBUG_ANYDATA);
		}
	}
	
    ucGPSStatePre = GPS_GetOrientState();
	ucACCPre = GetAccState();
}
#endif
/******************************************************************************
** 函数名称: TaskSysTimer
** 功能描述: 此任务1秒执行一次，对于定时时间精度要求不是很高的函数可以在此执行，
             本任务中执行的函数可以有系统延时
** 输　     入 : 无
** 输　     出 : 无
** 全局变量: 
** 调用模块: 
** 作　	 者 : hhm
** 日　	 期 : 2014年11月24日

**-----------------------------------------------------------------------------
*******************************************************************************/
void TaskSysTimer (void *pData)
{ 
    pData = pData;
    ///test////////////////
/*
	stu_McuFirmware.ucRcvPackflag = 1;
	FirmwareUpdate.ucdev = 1;
	stu_McuFirmware.ucLoadStep = 1; 
	*/
	///////////////////////

	
	while(1)
	{
        if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev==1)
            Mcu_FirmwareDownload_Function();   //威卡控制器升级
		else if(stu_KCMCUDownload.ucRcvPackflag&&FirmwareUpdate.ucdev==4)
		    KCMCU_ProgramUpdate_Function();    //川崎控制器升级
		
	    if(stu_McuFirmware.ucRcvPackflag)
		    OSTimeDly(50);   //50ms
		else if(stu_KCMCUDownload.ucRcvPackflag)
		{
		    if(stu_KCMCUDownload.ucRepeatSendCount)
			    OSTimeDly(20);  //10ms
			else
				OSTimeDly(5);
		}
        else
			OSTimeDly(OS_TICKS_PER_SEC);  //1S			
	}
}


//-----文件SystemProtocol.c结束---------------------------------------------

