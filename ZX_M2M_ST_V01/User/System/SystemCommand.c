/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: SystemCommand.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为System功能模块协议层数据解析的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-06-16, by  lxf, 创建本文件
 *
 */
//-----头文件调用------------------------------------------------------------

#include "config.h"
#include "SystemCommand.h"
#include "SystemProtocol.h"
#include "IAP.h"

uint8 Public_Buf[1500];

//-----外部变量定义------------------------------------------------------------

extern STUSystem g_stuSystem;
extern STU_SYSCounter SysCounter;
extern uint8   g_ucRstCount;            //梯度重启计数器
extern uint16  g_usRstTime;          //梯度重启时间间隔
//extern STUSYSMsgBus f_stuRepeatRecvGprs;
//extern stuWorkDataRep WorkDataRep;
extern stuPositionRep PositionTrack;
extern stuHeartBeat HeartBeat; 
extern stuAlarmRep AlarmRep;
extern stuDTCRep DTCRep;
extern stuAccRep AccRep;
extern uint16 f_usUpLoadCmdSn;
extern uint8 aMsgSendData[];
extern STU_McuCmd m_stuMcuCmd;
extern stuFirmwareUpdate FirmwareUpdate;
extern STUSleep stuSleep; 
extern stuConnect Connect;
extern STU_SSData SSData;
extern STUSYSParamSet g_stuSYSParamSet;
extern stuSerAddr g_stuSerAddr;
extern STUSleep stuSleep;
extern STUMCUFirmware stu_McuFirmware;
//-----内部变量定义------------------------------------------------------------
STUUnNomalData stuUnnormData;
volatile uint8 flag_update=0;
volatile uint32 update_timer=0;


//-----内部函数声明------------------------------------------------------------
void AllParamSave(void);
//-----外部函数定义------------------------------------------------------------


//1:时间已经校准, 0:时间未校准
uint8 IsCorrectTimeOk()
{
	return g_stuSystem.ucCorrectTimeOk;
}

void SetCorrectTimeOk()
{
	g_stuSystem.ucCorrectTimeOk = 1;
}
#if 1

/*********************************************************************************************************
** 函数名称: LKYW_EscapeData
** 功能描述: 将接受到服务器的数据进行转义还原
            ESCESC0x7B7B ------>0x7B7B  ESCESC0x7D7D---->7D7D
** 输　     入: Ptr,需要进行转义数据指向的指针地址  
                len 需要转义的数据的长度
** 输　     出: 转义后的长度
** 全局变量: 
** 调用模块: 
**
** 作　     者: 
** 日　     期: 2011年9 月5 日
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
uint16 RestoreTranslateData(uint8 *p,uint16 len)
{
    uint16 i=0;

	for(i=0; i<len-3; i++)
    {
        if((p[i]==0x1b)&&(p[i+1]==0x1b)&&(p[i+2]==0x7b)&&(p[i+3]==0x7b))
        {
            memmove(&p[i],&p[i+2],len-2-i);
            len -= 2;
        }
        else if((p[i]==0x1b)&&(p[i+1]==0x1b)&&(p[i+2]==0x7d)&&(p[i+3]==0x7d))
        {
            memmove(&p[i],&p[i+2],len-2-i);
            len -= 2; 
        }                    
		else 
		{
			
		}
    }
    return len;
}

/**********************************************************************************
** 函数名称: TranslateData
** 功能描述: 对数据包进行转译0x7B7B------>ESCESC和0x7B7B  7D7D------>ESCESC和0x7D7D
** 输    入: src:待转译的数据包
** 输　  出: des=转译后的数据包 
** 返    回: 转译后的数据包长度
** 作　  者: hhm
** 日　  期: 2014年9月25日
**--------------------------------------------------------------------------------
*********************************************************************************/
uint16 TranslateData( uint8* src,uint16 srcLen)
{
	uint16 i;
	uint16 desLen = 0;

	uint8 *pDes;
	uint8 *pSrc;
	//uint8 temp[MAX_GPRS_SEND_LEN];
	uint8 *temp = Public_Buf;
	
	pSrc = src;
	pDes = &temp[0];
	desLen = srcLen;
	for(i = 0; i < srcLen; i++) 
	{
		if((0x7B==*pSrc) && (0x7B==*(pSrc+1)))			//0x7B7B------>ESCESC和0x7B7B
		{
			*pDes++ = 0x1B;
			*pDes++ = 0x1B;
			*pDes++ = 0x7B;
			*pDes++ = 0x7B;
			pSrc += 2;;
			desLen += 2;
		}
		else if((0x7d==*pSrc)&&(0x7d==*(pSrc+1)) )		//0x7D7D------>ESCESC和0x7D7D
		{
			*pDes++ = 0x1B;	
			*pDes++ = 0x1B;
			*pDes++ = 0x7D;	
			*pDes++ = 0x7D;
			pSrc += 2;
			desLen += 2;
		}
		else
		{
			*pDes++ = *pSrc++;
		}
	}
	memcpy(src, temp, desLen);
	return desLen;
}
#endif

/******************************************************************************
** 函数名称: DealSerCmd_DefSerRespondGps
** 功能描述: 处理平台对终端命令的标准回复
** 输    入: head--服务器下发指令的包头信息
             PtrTxt,下发数据包指针;
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-7-9
******************************************************************************/
uint16 DealSerCmd_DefSerRespondGps(uint8 *PtrTxt,STUSYSPactHeader head)
{
	uint8 *p = PtrTxt;
	uint16 sn;
	uint8 cmd;
	uint8 ret;
	
	p += MSG_HEAD_LEN;
	sn = (*p<<8) + *(p+1);
	p += 2;
	cmd = *p++;
	ret = *p;
	if(0!=ret)
		return 0;
	switch(cmd)
	{
		case MSG_TYPE_PUSH_DATA:
			if(sn==SSData.usRepSn)
			{
				SSData.ucRepeats = 0;
				SSData.ucRepFlag = 0;
			}
			#if 0
			else if(sn==WorkDataRep.usRepSn)
			{
				WorkDataRep.ucRepeats = 0;
				WorkDataRep.ucRepFlag = 0;
			}
			#endif
			else
			{
			}
			break;
		case MSG_TYPE_ALERT:
			if(sn==AlarmRep.usRepSn)
			{
				AlarmRep.ucRepeats = 0;
				AlarmRep.ucRepFlag = 0;
				AlarmRep.ucNewAlarmFlag = 0;
			}
			else if(sn==DTCRep.usRepSn)
			{
				DTCRep.ucRepeats = 0;
				DTCRep.ucRepFlag = 0;
				ClearNewMcuFaultCodeFlag();
			}
			else
			{
			}
			break;
		default:
			break;
	}
	return 0;
}

/******************************************************************************
** 函数名称: DealSerCmd_Connect
** 功能描述: 处理服务器下发的连接响应指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: PtrTxt,响应数据包指针
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-18
******************************************************************************/
uint16 DealSerCmd_Connect(uint8 *PtrTxt,STUSYSPactHeader head)
{
	uint8 *p = PtrTxt;
	
	p += MSG_HEAD_LEN;
	if(0==*p)
	{
		Connect.ucRespondTimer = 0;
		Connect.ucRepeats = 0;
		Connect.ucRepFlag = 0;
		Connect.ucSucceedFlag = 1;
		g_stuSystem.ucOnline = 1;
		if(SSData.ucTimeToRepFlag !=1)
		    SSData.usTimer = 0;          //20170324 lxf
	}
	return 0;
}
/******************************************************************************
** 函数名称: ParsePara
** 功能描述: 解析参数
** 输    入: id=参数标识
             len=参数长度
             value=参数内容
             src=参数来源,0=来自服务器或测试客户端命令,1=来自参数存储器
** 输    出: PtrTxt,响应数据包指针
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-7-7
******************************************************************************/
BOOLEAN ParsePara(uint16 id, uint16 len, uint8 *value, uint8 src)
{
	uint8 ucTemp;
	uint32 uiTemp;
	
	switch(id)
	{
		case 0x0000://终端产品唯一编号（终端设备序列号），唯一标识该M2M终端设备。
			if(7!=len)
				return FALSE;
			memcpy(g_stuSYSParamSet.aucDeviceID, value, 7);
			ucTemp = FLASH_OB_GetUser();
			if(0!=(0x01 & ucTemp))
			{
				FLASH_OB_Unlock();
				FLASH_OB_UserConfig(OB_IWDG_HW, OB_STOP_NoRST, OB_STDBY_NoRST);
				FLASH_OB_Launch();
				FLASH_OB_Lock();
			}			
			break;
		case 0x0002://M2M平台网络接入点名称（APN）
			if((len>32) || (len==0))
				return FALSE;
			g_stuSYSParamSet.ucAPNLen = len;
			memcpy(g_stuSYSParamSet.aucAPN, value, len);
			break;
		case 0x0003://M2M平台登录用户名
			g_stuSYSParamSet.ucUserLen = len;
			memcpy(g_stuSYSParamSet.aucUser, value, len);
			break;
		case 0x0004://M2M平台登录密码
			g_stuSYSParamSet.ucPasswordLen= len;
			memcpy(g_stuSYSParamSet.aucPassword, value, len);
			break;
		case 0x0005://短信中心号码
			g_stuSYSParamSet.ucSmsCenterNumLen = len;
			memcpy(g_stuSYSParamSet.aucSmsCenterNum, value, len);
			break;	
		case 0x0006://主中心IP地址
			if(4!=len)
				return FALSE;
			memcpy(g_stuSYSParamSet.aucHostIP, value, 4);
			memcpy(g_stuSerAddr.aucIp, g_stuSYSParamSet.aucHostIP, 4);
			if(0==src)
				g_stuSystem.ucSerAddrChangeFlag = 1;
			break;	
		case 0x0007://副中心IP地址
			if(4!=len)
				return FALSE;
			memcpy(g_stuSYSParamSet.aucSpareHostIP, value, 4);
			if(0==src)
				g_stuSystem.ucSerAddrChangeFlag = 1;
			break;
		case 0x0008://主中心端口
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usHostPort = (*value<<8) + *(value+1);
			g_stuSerAddr.usPort = g_stuSYSParamSet.usHostPort;
			if(0==src)
				g_stuSystem.ucSerAddrChangeFlag = 1;
			break;
		case 0x0009://副中心端口
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usSpareHostPort = (*value<<8) + *(value+1);
			if(0==src)
				g_stuSystem.ucSerAddrChangeFlag = 1;
			break;
		case 0x000A://心跳间隔，单位：秒
			if(4!=len)
				return FALSE;
			g_stuSYSParamSet.uiHeartbeatInterval = (*(value)<<24) + (*(value+1)<<16) + (*(value+2)<<8) + *(value+3);
			break;	

		case 0x000B://最大登录重复次数参数
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucMaxLandonRepeats = *value;
			break;	
		case 0x000C://登录失败重试间隔参数
			if(8!=len)
				return FALSE;
			g_stuSYSParamSet.uiLandonFailedMinRepeatsInterval = (*(value  )<<24) + (*(value+1)<<16) + (*(value+2)<<8) + *(value+3);
			g_stuSYSParamSet.uiLandonFailedMaxRepeatsInterval = (*(value+4)<<24) + (*(value+5)<<16) + (*(value+6)<<8) + *(value+7);
			break;	
			
			
		case 0x0106://本地CAN总线波特率
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usCanBrt= (*value<<8) + *(value+1);
			break;	
		case 0x0107://本地CAN报文格式
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usCanFmt = (*value<<8) + *(value+1);
			break;		
		case 0x0108://CAN ID 过滤配置
			if((len<4) && (len%4!=0))
				return FALSE;
			g_stuSYSParamSet.ucCanIdNum = len/4;
			for(ucTemp=0; ucTemp<g_stuSYSParamSet.ucCanIdNum; ucTemp++)
			{
				g_stuSYSParamSet.auiCanId[ucTemp]=(*value<<24)+(*(value+1)<<16)+(*(value+2)<<8)+*(value+3);
				value += 4;	
			}
			break;	
		case 0x010B://进入休眠时间，单位：秒
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usSleepBeforSlot = (*value<<8) + *(value+1);
			break;		

		case 0x010C://休眠期间定时唤醒间隔，单位：分	
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usSleepWakeSlot = (*value<<8) + *(value+1);
			break;

		case 0x010D://终端基本状态同步数据自动发送间隔,单位：秒
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucSSRepSlot = *value;
			break;	

		case 0x0110://SIM卡号，不足高位补0,
			if(6!=len)
				return FALSE;
			memcpy(g_stuSYSParamSet.aucSim, value, 6);
			break;

		case 0x0113://主中心域名
			if(len<3)
				return FALSE;
			g_stuSYSParamSet.ucHostDnLen = len;
			memcpy(g_stuSYSParamSet.aucHostDn, value, len);
			break;	

		case 0x0114://副中心域名
			if(len<3)
				return FALSE;
			g_stuSYSParamSet.ucSpareDnLen= len;
			memcpy(g_stuSYSParamSet.aucSpareDn, value, len);
			break;		

		case 0x0115://DNS
			if(len<3)
				return FALSE;
			memcpy(g_stuSYSParamSet.aucDns, value, 4);
			break;			

		case 0x0117://外电源额定电压,单位：0.1V
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usMainPwrRateVol= (*value<<8) + *(value+1);
			break;

		case 0x0118://终端电池额定电压,单位：0.1V
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.usBatRateVol = (*value<<8) + *(value+1);
			break;	

		case 0x0119://主中心承载协议类型：0：UDP协议,1: TCP协议
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucHostProtocolType = *value;
			g_stuSerAddr.ucProtocolType = g_stuSYSParamSet.ucHostProtocolType;
			if(0==src)
				g_stuSystem.ucSerAddrChangeFlag = 1;
			break;	

		case 0x011A://副中心承载协议类型：0：UDP协议,1: TCP协议
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucSpareHostProtocolType = *value;
			break;		
			
		case 0x0201://控制器与终端总线通信中断报警参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucCanErrTime = *value;
			g_stuSYSParamSet.ucCanOKTime = *(value+1);
			break;		
		case 0x0202://终端外部电源断电报警参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucPwrOffTime = *value;
			g_stuSYSParamSet.ucPwrOnTime = *(value+1);
			break;			
		case 0x0203://终端外部电源低电压报警参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucPwrLowVol = *value;
			g_stuSYSParamSet.ucPwrLowTime= *(value+1);
			break;		
		case 0x0204://终端内部电源（电池）低电压报警（TLV-0x300D-0x09报警）参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucBatLowVol= *value;
			g_stuSYSParamSet.ucBatLowTime= *(value+1);
			break;		
		case 0x0205://终端GPS天线故障报警（TLV-0x300D-0x06报警）参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucGpsAntErrTime = *value;
			g_stuSYSParamSet.ucGpsAntOKTime  = *(value+1);
			break;		
		case 0x0206://终端GPS定位模块故障报警（TLV-0x300D-0x05报警）参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucGpsModuleErrTime= *value;
			g_stuSYSParamSet.ucGpsModuleOKTime= *(value+1);
			break;		
		case 0x020A://超速报警（TLV-0x300D-0x03）报警参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucSpeedOver = *value;
			g_stuSYSParamSet.ucSpeedOverLastTime = *(value+1);
			break;		
		case 0x020B://拖车报警（TLV-0x300D-0x01）报警参数
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucTransportCar = *value;
			break;
		/*	
		case 0x0300://ACC ON/OFF(设备开关机)上传配置参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucTransportCar = *value;
			break;	
		*/
		case 0x2000://设备工作时间段统计配置参数
			if(1!=len)
				return FALSE;
			g_stuSYSParamSet.ucDeviceWorkTimeRepCfg = *value;
			break;	
		case 0x2001://第1字节表示工作参数（工况）数据单条上传模式,第2字节当第1字节为0x00时，表示工作参数（工况）传输参数
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucWorkDataRepModel = *value;
			g_stuSYSParamSet.ucWorkDataRepInterval= *(value+1);
			break;		
		case 0x2002://第1字节表示位置信息单条上传模式,第2字节表示位置信息传输间隔
			if(2!=len)
				return FALSE;
			g_stuSYSParamSet.ucPosiInforRepModel = *value;
			g_stuSYSParamSet.ucPosiInforRepInterval = *(value+1);
			break;
		case 0x3016://终端ACC ON 累计时间	
			if(4!=len)
				return FALSE;
			uiTemp = (*value<<24)+(*(value+1)<<16)+(*(value+2)<<8)+*(value+3);
			SetWorkingTime(uiTemp);
			break;
			
		default:
			return FALSE;
	//		break;
	}
	return TRUE;
}

uint8 *GSM_GetICCID(void)
{
    uint8 aucCCID[20]={0};
    return (uint8*)&aucCCID[0];
}
/******************************************************************************
** 函数名称: DealSerCmd_SetPara
** 功能描述: 处理服务器下发的"设置参数"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: PtrTxt,响应数据包指针
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-22
******************************************************************************/
uint16 DealSerCmd_SetPara(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint16 usSendLen,usTemp;
	uint8 ucTlvNum;
	uint16 usParaId = 0;
	uint16 ucParaLen;
	uint8 flag = 0;
	uint8 *p =  pCmdBody;
	uint16 ausFaildParaId[50]; 
	uint8 ucFaildParaIdNum = 0;
	uint8 ucIndex = 0;
	uint8 *pBuf;
	uint8 i=0;

	if(usCmdBodyLen<5)
		return 0;
	ucTlvNum = *p;
	p++;
	while(ucTlvNum)
	{
		usParaId = (*p<<8) + *(p+1);
		p += 2;
		ucParaLen = (*p<<8) + *(p+1);
		p += 2;
		if(FALSE == ParsePara(usParaId, ucParaLen, p, 0))
		{
			flag = 1;
			ausFaildParaId[ucIndex] = usParaId;
			ucIndex++;
			ucFaildParaIdNum++;
		}
		ucTlvNum--;
		p += ucParaLen;
	}
	AllParamSave();
	aMsgSendData[MSG_HEAD_LEN] = (head.usSq>>8) & 0xff;
	aMsgSendData[MSG_HEAD_LEN+1] = head.usSq & 0xff;
	aMsgSendData[MSG_HEAD_LEN+2] = 0;
	aMsgSendData[MSG_HEAD_LEN+3] = 2;
	aMsgSendData[MSG_HEAD_LEN+4] = 'P';
	aMsgSendData[MSG_HEAD_LEN+5] = 'W';
	if(0==flag)
	{
		aMsgSendData[MSG_HEAD_LEN+6] = 0;
		aMsgSendData[MSG_HEAD_LEN+7] =  1;
		aMsgSendData[MSG_HEAD_LEN+8] = flag;
		usTemp = 1;
	}
	else
	{
		usTemp = 2 + ucFaildParaIdNum*2;
		aMsgSendData[MSG_HEAD_LEN+6] = (usTemp>>8) & 0xff;
		aMsgSendData[MSG_HEAD_LEN+7] =  usTemp     & 0xff;
		aMsgSendData[MSG_HEAD_LEN+8] = flag;
		aMsgSendData[MSG_HEAD_LEN+9] = ucFaildParaIdNum;
		pBuf = &aMsgSendData[MSG_HEAD_LEN+10];
		while(ucFaildParaIdNum)
		{
			*pBuf++ = (ausFaildParaId[i]>>8) & 0xff;
			*pBuf++ =  ausFaildParaId[i] & 0xff;
			i++;
			ucFaildParaIdNum--;
		}
	}
	usSendLen = 8 + usTemp;
	usTemp = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, usSendLen, 0, head.usSq);
	usSendLen += usTemp;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	if(SRCDEVICE_ID_SETTOOL==head.ucSrc)
		PC_SendToPCData2(aMsgSendData, usSendLen+1);
	else if(SRCDEVICE_ID_A5_COM1==head.ucSrc)
		A5_SendToA5Data(aMsgSendData, usSendLen+1);
	//else
	//	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);  
	return usSendLen+1;
}

uint16 PackParaData(uint16 usParaId, uint8 *pBuf)
{
	uint32 uiTemp;
	uint16 usTemp;
	uint16 usParaLen=0;
	uint8 i;
	uint8 *pTemp;
	
	switch(usParaId)
	{
		case 0x0000:		//终端产品唯一编号（终端设备序列号），唯一标识该M2M终端设备
			*pBuf++ = 0x00;
			*pBuf++ = 0x00;
			*pBuf++ = 0;
			*pBuf++ = 7;
			memcpy(pBuf,g_stuSYSParamSet.aucDeviceID, 7);
			pBuf += 7;
			usParaLen += 11;
			break;
		case 0x0001:	//只读，终端设备软件版本号
			*pBuf++ = 0x00;
			*pBuf++ = 0x01;
			*pBuf++ = 0;
			*pBuf++ = SW_VERSION_LEN;
			memcpy(pBuf,SW_VERSION, SW_VERSION_LEN);
			pBuf += SW_VERSION_LEN;
			usParaLen += 4+SW_VERSION_LEN;
			break;
		case 0x0002:	//M2M平台网络接入点名称（APN）
			*pBuf++ = 0x00;
			*pBuf++ = 0x02;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucAPNLen;
			memcpy(pBuf,g_stuSYSParamSet.aucAPN, g_stuSYSParamSet.ucAPNLen);
			pBuf += g_stuSYSParamSet.ucAPNLen;
			usParaLen += g_stuSYSParamSet.ucAPNLen+4;
			break;
		case 0x0003://M2M平台登录用户名
			*pBuf++ = 0x00;
			*pBuf++ = 0x03;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucUserLen;
			memcpy(pBuf,g_stuSYSParamSet.aucUser, g_stuSYSParamSet.ucUserLen);
			pBuf += g_stuSYSParamSet.ucUserLen;
			usParaLen += g_stuSYSParamSet.ucUserLen+4;
			break;
		case 0x0004://M2M平台登录密码
			*pBuf++ = 0x00;
			*pBuf++ = 0x04;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucPasswordLen;
			memcpy(pBuf,g_stuSYSParamSet.aucPassword, g_stuSYSParamSet.ucPasswordLen);
			pBuf += g_stuSYSParamSet.ucPasswordLen;
			usParaLen += g_stuSYSParamSet.ucPasswordLen+4;
			break;
		case 0x0005://短信中心号码
			*pBuf++ = 0x00;
			*pBuf++ = 0x05;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucSmsCenterNumLen;
			memcpy(pBuf,g_stuSYSParamSet.aucSmsCenterNum, g_stuSYSParamSet.ucSmsCenterNumLen);
			pBuf += g_stuSYSParamSet.ucSmsCenterNumLen;
			usParaLen += g_stuSYSParamSet.ucSmsCenterNumLen+4;
			break;
		case 0x0006://主中心IP地址
			*pBuf++ = 0x00;
			*pBuf++ = 0x06;
			*pBuf++ = 0;
			*pBuf++ = 4;
			memcpy(pBuf,g_stuSYSParamSet.aucHostIP, 4);
			pBuf += 4;
			usParaLen += 8;
			break;
		case 0x0007://副中心IP地址
			*pBuf++ = 0x00;
			*pBuf++ = 0x07;
			*pBuf++ = 0;
			*pBuf++ = 4;
			memcpy(pBuf,g_stuSYSParamSet.aucSpareHostIP, 4);
			pBuf += 4;
			usParaLen += 8;
			break;
		case 0x0008://主中心端口
			*pBuf++ = 0x00;
			*pBuf++ = 0x08;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usHostPort>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usHostPort     & 0xff;
			usParaLen += 6;
			break;
		case 0x0009://副中心端口
			*pBuf++ = 0x00;
			*pBuf++ = 0x09;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usSpareHostPort>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usSpareHostPort     & 0xff;
			usParaLen += 6;
			break;
		case 0x000a:	//心跳间隔
			*pBuf++ = 0x00;
			*pBuf++ = 0x0a;
			*pBuf++ = 0;
			*pBuf++ = 4;
			*pBuf++ = (g_stuSYSParamSet.uiHeartbeatInterval>>24) & 0xff;
			*pBuf++ = (g_stuSYSParamSet.uiHeartbeatInterval>>16) & 0xff;
			*pBuf++ = (g_stuSYSParamSet.uiHeartbeatInterval>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.uiHeartbeatInterval     & 0xff;
			usParaLen += 8;
			break;	
		case 0x0106://本地CAN总线波特率
			*pBuf++ = 0x01;
			*pBuf++ = 0x06;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usCanBrt>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usCanBrt     & 0xff;
			usParaLen += 6;
			break;
		case 0x0107://本地CAN总线波特率
			*pBuf++ = 0x01;
			*pBuf++ = 0x07;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usCanFmt>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usCanFmt     & 0xff;
			usParaLen += 6;
			break;
			
		case 0x0108://CAN ID 过滤配置
			*pBuf++ = 0x01;
			*pBuf++ = 0x08;
			usTemp = g_stuSYSParamSet.ucCanIdNum*4;
			*pBuf++ = (usTemp>>8) & 0xff;
			*pBuf++ =  usTemp & 0xff;
			usParaLen += 4;
			for(i=0; i<g_stuSYSParamSet.ucCanIdNum; i++)
			{
				*pBuf++ = (g_stuSYSParamSet.auiCanId[i]>>24) & 0xff;
				*pBuf++ = (g_stuSYSParamSet.auiCanId[i]>>16) & 0xff;
				*pBuf++ = (g_stuSYSParamSet.auiCanId[i]>>8)  & 0xff;
				*pBuf++ =  g_stuSYSParamSet.auiCanId[i]      & 0xff;
				usParaLen += 4;
			}
			break;
		case 0x010B://进入休眠时间
			*pBuf++ = 0x01;
			*pBuf++ = 0x0B;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usSleepBeforSlot>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usSleepBeforSlot     & 0xff;
			usParaLen += 6;
			break;
		case 0x010C://休眠期间定时唤醒间隔，单位：分
			*pBuf++ = 0x01;
			*pBuf++ = 0x0C;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usSleepWakeSlot>>8) & 0xff;
			*pBuf++ =  g_stuSYSParamSet.usSleepWakeSlot     & 0xff;
			usParaLen += 6;
			break;

		case 0x010D://终端基本状态同步数据自动发送间隔,单位：秒
			*pBuf++ = 0x01;
			*pBuf++ = 0x0D;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucSSRepSlot;
			usParaLen += 5;
			break;	
			
		case 0x0110://SIM卡号，不足高位补0,	
			*pBuf++ = 0x01;
			*pBuf++ = 0x10;
			*pBuf++ = 0;
			*pBuf++ = 6;
			memcpy(pBuf, g_stuSYSParamSet.aucSim, 6);
			pBuf += 6;
			usParaLen += 10;
			break;
		case 0x0111://ICCID码
			pTemp = GSM_GetICCID();
			*pBuf++ = 0x01;
			*pBuf++ = 0x11;
			*pBuf++ = 0;
			*pBuf++ = 20;
			memcpy(pBuf, pTemp, 20);
			pBuf += 20;
			usParaLen += 24;
			break;			

		case 0x0113://主中心域名	
			*pBuf++ = 0x01;
			*pBuf++ = 0x13;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucHostDnLen;
			memcpy(pBuf, g_stuSYSParamSet.aucHostDn, g_stuSYSParamSet.ucHostDnLen);
			pBuf += g_stuSYSParamSet.ucHostDnLen;
			usParaLen += g_stuSYSParamSet.ucHostDnLen+4;
			break;

		case 0x0114://副中心域名	
			*pBuf++ = 0x01;
			*pBuf++ = 0x14;
			*pBuf++ = 0;
			*pBuf++ = g_stuSYSParamSet.ucSpareDnLen;
			memcpy(pBuf, g_stuSYSParamSet.aucSpareDn, g_stuSYSParamSet.ucSpareDnLen);
			pBuf += g_stuSYSParamSet.ucSpareDnLen;
			usParaLen += g_stuSYSParamSet.ucSpareDnLen+4;
			break;	

		case 0x0115://DNS	
			*pBuf++ = 0x01;
			*pBuf++ = 0x15;
			*pBuf++ = 0;
			*pBuf++ = 4;
			memcpy(pBuf, g_stuSYSParamSet.aucDns, 4);
			pBuf += 4;
			usParaLen += 8;
			break;		

		case 0x0116://硬件版本号，如V1.5表示为0x0105
			*pBuf++ = 0x01;
			*pBuf++ = 0x16;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (HW_VERSION>>8)& 0xff;
			*pBuf++ =  HW_VERSION    & 0xff;
			usParaLen += 6;
			break;	

		case 0x0117://外电源额定电压单位：0.1V
			*pBuf++ = 0x01;
			*pBuf++ = 0x17;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usMainPwrRateVol>>8)& 0xff;
			*pBuf++ =  g_stuSYSParamSet.usMainPwrRateVol    & 0xff;
			usParaLen += 6;
			break;	

		case 0x0118://终端电池额定电压单位：0.1V
			*pBuf++ = 0x01;
			*pBuf++ = 0x18;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = (g_stuSYSParamSet.usBatRateVol>>8)& 0xff;
			*pBuf++ =  g_stuSYSParamSet.usBatRateVol    & 0xff;
			usParaLen += 6;
			break;		

		case 0x0119://主中心承载协议类型：0：UDP协议,1: TCP协议
			*pBuf++ = 0x01;
			*pBuf++ = 0x19;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucHostProtocolType;
			usParaLen += 5;
			break;		

		case 0x011A://副中心承载协议类型：0：UDP协议,1: TCP协议
			*pBuf++ = 0x01;
			*pBuf++ = 0x1A;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucSpareHostProtocolType;
			usParaLen += 5;
			break;			
			
		case 0x0201://控制器与终端总线通信中断报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x01;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucCanErrTime;
			*pBuf++ = g_stuSYSParamSet.ucCanOKTime;
			usParaLen += 6;
			break;
		case 0x0202://终端外部电源断电报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x02;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucPwrOffTime;
			*pBuf++ = g_stuSYSParamSet.ucPwrOnTime;
			usParaLen += 6;
			break;
		
		case 0x0203://终端外部电源低电压报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x03;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucPwrLowVol;
			*pBuf++ = g_stuSYSParamSet.ucPwrLowTime;
			usParaLen += 6;
			break;

		case 0x0204://终端内部电源（电池）低电压报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x04;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucBatLowVol;
			*pBuf++ = g_stuSYSParamSet.ucBatLowTime;
			usParaLen += 6;
			break;
		case 0x0205://终端GPS天线故障报警（TLV-0x300D-0x06报警）参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x05;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucGpsAntErrTime;
			*pBuf++ = g_stuSYSParamSet.ucGpsAntOKTime;
			usParaLen += 6;
			break;
		case 0x0206://终端GPS天线故障报警（TLV-0x300D-0x06报警）参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x06;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucGpsModuleErrTime;
			*pBuf++ = g_stuSYSParamSet.ucGpsModuleOKTime;
			usParaLen += 6;
			break;
		case 0x020A://超速报警报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x0A;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucSpeedOver;
			*pBuf++ = g_stuSYSParamSet.ucSpeedOverLastTime;
			usParaLen += 6;
			break;
		case 0x020B://拖车报警报警参数
			*pBuf++ = 0x02;
			*pBuf++ = 0x0B;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucTransportCar;
			usParaLen += 5;
			break;
		#if 0	
		case 0x0300://ACC ON/OFF(设备开关机)上传配置参数 	
			*pBuf++ = 0x02;
			*pBuf++ = 0x06;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucTransportCar;
			usParaLen += 5;
			break;
		#endif
		case 0x2000://ACC ON/OFF(设备开关机)上传配置参数 	
			*pBuf++ = 0x20;
			*pBuf++ = 0x00;
			*pBuf++ = 0;
			*pBuf++ = 1;
			*pBuf++ = g_stuSYSParamSet.ucDeviceWorkTimeRepCfg;
			usParaLen += 5;
			break;
		case 0x2001://第1字节表示工作参数（工况）数据单条上传模式,当第1字节为0x00时，表示工作参数（工况）传输参数
			*pBuf++ = 0x20;
			*pBuf++ = 0x01;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucWorkDataRepModel;
			*pBuf++ = g_stuSYSParamSet.ucWorkDataRepInterval;
			usParaLen += 6;
			break;
		case 0x2002://第1字节表示位置信息单条上传模式,第2字节表示位置信息传输间隔
			*pBuf++ = 0x20;
			*pBuf++ = 0x02;
			*pBuf++ = 0;
			*pBuf++ = 2;
			*pBuf++ = g_stuSYSParamSet.ucPosiInforRepModel;
			*pBuf++ = g_stuSYSParamSet.ucPosiInforRepInterval;
			usParaLen += 6;
			break;
		case 0x3016:	//工作小时
			uiTemp = GetWorkingTime();
			*pBuf++ = 0x30;
			*pBuf++ = 0x16;
			*pBuf++ = 0;
			*pBuf++ = 4;
			*pBuf++ = (uiTemp>>24) & 0xff;
			*pBuf++ = (uiTemp>>16) & 0xff;
			*pBuf++ = (uiTemp>>8)  & 0xff;
			*pBuf++ =  uiTemp      & 0xff;
			usParaLen += 8;
			break;
			
		default:
			break;	
			
	}		
	return usParaLen;
}

/******************************************************************************
** 函数名称: DealSerCmd_QueryPara
** 功能描述: 处理服务器下发的"参数查询指令"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-7-8
******************************************************************************/
uint16 DealSerCmd_QueryPara(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint8 *p = pCmdBody;
	uint16 usSendLen, usTemp;
	uint8 ucTlvNum;
	uint8 ucTlvNumRet=0;	//返回的TLV个数
	uint16 usParaId = 0;
	uint16 usParaLen = 0;
	uint8 *pBuf = &aMsgSendData[MSG_HEAD_LEN+9];
	
	ucTlvNum = *p++;
	while(0!=ucTlvNum)
	{
		usParaId = (*p<<8) + *(p+1);
		p += 2;
		usTemp = PackParaData(usParaId, pBuf);
		if(usTemp>0)
		{
			ucTlvNumRet++;
			pBuf += usTemp;
			usParaLen += usTemp;
		}
		ucTlvNum--;
	}

	aMsgSendData[MSG_HEAD_LEN]   = (head.usSq>>8) & 0xff;
	aMsgSendData[MSG_HEAD_LEN+1] =  head.usSq     & 0xff;
	aMsgSendData[MSG_HEAD_LEN+2]   = 0x00;
	aMsgSendData[MSG_HEAD_LEN+3] = 0x02;
	aMsgSendData[MSG_HEAD_LEN+4] = 'P';
	aMsgSendData[MSG_HEAD_LEN+5] = 'R';
	usCmdBodyLen = usParaLen+1;
	aMsgSendData[MSG_HEAD_LEN+6] = (usCmdBodyLen>>8) & 0xff;
	aMsgSendData[MSG_HEAD_LEN+7] =  usCmdBodyLen     & 0xff;
	aMsgSendData[MSG_HEAD_LEN+8] = ucTlvNumRet;
	usCmdBodyLen = usCmdBodyLen + 8;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, usCmdBodyLen, 0, head.usSq);
	usSendLen +=  usCmdBodyLen;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	usSendLen += 1;
	if(SRCDEVICE_ID_SETTOOL==head.ucSrc)
		PC_SendToPCData2(aMsgSendData, usSendLen);
	else if(SRCDEVICE_ID_A5_COM1==head.ucSrc)
		A5_SendToA5Data(aMsgSendData, usSendLen);
	
//	else
//		GSM_SendGprs(aMsgSendData, usSendLen, 0);  
	return usSendLen;
}

#define PARA_NUM	28
void AllParamSave()
{
	uint8 *p = Public_Buf;
	uint16 usTemp;
	uint16 len = 0;
	uint8 i;
   	uint16 paraid[PARA_NUM] = {
	0x0000,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,0x000a,
	0x000b,0x000c,0x000d,0x0106,0x0107,0x0108,0x010b,0x010c,0x010d,0x0110,
	0x0113,0x0114,0x0115,0x0116,0x0117,0x0118,0x0119,0x011a
	};
	*p++ = 0xaa;
	p++;		//预留参数长度,<1024
	p++;
	for(i=0; i<PARA_NUM; i++)
	{
		usTemp = PackParaData(paraid[i], p);
		len += usTemp;
		p += usTemp; 
	}
	Public_Buf[1] = (len>>8) & 0xff;
	Public_Buf[2] =  len     & 0xff;
	*p = SumCalc(Public_Buf, len+3);
	len += 4;
	WriteToFlash(FLASH_PAGEADDR_PARAMET1, 0, len, Public_Buf);//保存到flash第一个存储区
	WriteToFlash(FLASH_PAGEADDR_PARAMET2, 0, len, Public_Buf);//保存到flash第二个存储区
	//PC_SendDebugData(Public_Buf, len, DEBUG_ANYDATA);
}

void SYS_ParamRead(void)
{
	
	uint8 *aParamData = Public_Buf;
    uint16 usParamLen;
	uint8 ucTlvNum = PARA_NUM;
	uint16 usParaId;
	uint8 *p;
	uint16 usParaLen;
	uint8 flag = 0;
	
	//读取地址1 长度37-100超出视为异常  
    ReadFromFlash(FLASH_PAGEADDR_PARAMET1,0,1024,aParamData);    //从存储器中读取参数的标志和长度 
    usParamLen = (aParamData[1]<<8)+aParamData[2];     //参数长度
	//PC_SendDebugData(Public_Buf, usParamLen+4, DEBUG_ANYDATA);
    if((aParamData[0]==0xaa) && (usParamLen<=1021))       	//读取的数据标志正确
    {
        if(aParamData[usParamLen+3]==SumCalc(aParamData, usParamLen+3))      //判定读取参数校验和是否正确
        {
           	p = &Public_Buf[3];
			while(ucTlvNum)
			{
				usParaId = (*p<<8) + *(p+1);
				p += 2;
				usParaLen = (*p<<8) + *(p+1);
				p += 2;
				if(ParsePara(usParaId, usParaLen, p, 0))
				{
					p += usParaLen;
				}
				else
				{
					PC_SendDebugData((uint8 *)(&usParaId), 2, DEBUG_ANYDATA); 
					break;
				}
				ucTlvNum--;
			}            
			flag = 1;
		}
    }

	//读取地址2
	if(0==flag)
	{
		PC_SendDebugData((uint8 *)("ReadParaErr1"), 12, DEBUG_ANYDATA);    //地址1参数错误
		ReadFromFlash(FLASH_PAGEADDR_PARAMET2,0,1024,aParamData);    //从存储器中读取参数的标志和长度 
	    usParamLen = (aParamData[1]<<8)+aParamData[2];     //参数长度
	    if((aParamData[0]==0xaa) && (usParamLen<=1021))       	//读取的数据标志正确
	    {
	        if(aParamData[usParamLen+3]==SumCalc(aParamData, usParamLen+3))      //判定读取参数校验和是否正确
	        {
	           	p = &Public_Buf[3];
				while(ucTlvNum)
				{
					usParaId = (*p<<8) + *(p+1);
					p += 2;
					usParaLen = (*p<<8) + *(p+1);
					p += 2;
					if(ParsePara(usParaId, usParaLen, p, 0))
					{
						p += usParaLen;
					}
					else
					{
						PC_SendDebugData((uint8 *)(&usParaId), 2, DEBUG_ANYDATA); 
						break;
					}	
					ucTlvNum--;
				}            //对所有参数进行初始化 
				WriteToFlash(FLASH_PAGEADDR_PARAMET1, 0, 1024, Public_Buf);
				flag = 1;
			}
	    }
		if(0==flag)
		{
			PC_SendDebugData((uint8 *)("ReadParaErr2"), 12, DEBUG_ANYDATA);    //地址1参数错误
		}
	}
	
	memcpy(g_stuSerAddr.aucIp, g_stuSYSParamSet.aucHostIP, 4);
	g_stuSerAddr.usPort = g_stuSYSParamSet.usHostPort;
	g_stuSerAddr.ucProtocolType = g_stuSYSParamSet.ucHostProtocolType;
	//test
//	g_stuSYSParamSet.usSleepBeforSlot = 60;  //test liuxf
	stuSleep.usSleepBeforSlot = g_stuSYSParamSet.usSleepBeforSlot;
}

/******************************************************************************
** 函数名称: SysParaToFactory
** 功能描述: 将系统配置参数恢复出厂设置
** 输    入:  
** 输    出: 
** 返    回: 
** 作    者: hhm
** 日    期: 2016-8-23
******************************************************************************/
void SysParaToFactory()
{
	uint8 temp[1] = {0xff};

	WriteToFlash(FLASH_PAGEADDR_PARAMET1, 0, 1, temp);
	WriteToFlash(FLASH_PAGEADDR_PARAMET2, 0, 1, temp);
}

#if 0
/******************************************************************************
** 函数名称: DealSerCmd_WorkDataTrack
** 功能描述: 查询/关注设备当前工作参数命令
** 输    入:  
** 输    出: 
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-23
******************************************************************************/
uint16 DealSerCmd_WorkDataTrack(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint8 *p = pCmdBody;
	uint16 usSendLen;
	
	WorkDataRep.ucTrackModel = *p++;
	WorkDataRep.ucTrackInterval = *p++;
	WorkDataRep.usTrackScope = 60 * (*p++);
	
	aMsgSendData[MSG_HEAD_LEN]   = (head.usSq>>8) & 0xff;
	aMsgSendData[MSG_HEAD_LEN+1] =  head.usSq     & 0xff;
	aMsgSendData[MSG_HEAD_LEN+2]   = 0x00;
	aMsgSendData[MSG_HEAD_LEN+3] = 0x02;
	aMsgSendData[MSG_HEAD_LEN+4] = 'W';
	aMsgSendData[MSG_HEAD_LEN+5] = 'P';
	
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, 6, 0, head.usSq);
	usSendLen += 6;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen, 0);
	return 0;
}

#endif
/******************************************************************************
** 函数名称: DealSerCmd_PositionTrack
** 功能描述: 位置追踪命令
** 输    入:  
** 输    出: 
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-23
******************************************************************************/
uint16 DealSerCmd_PositionTrack(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint16 usSendLen;
	uint8 *p = pCmdBody;
	uint16 usMsgBodyLen;
	
    if(SRCDEVICE_ID_SETTOOL!=head.ucSrc)
	{
		PositionTrack.ucTrackModel = *p++;
		if(0xff==PositionTrack.ucTrackModel)
		{
			PositionTrack.ucTrackModel = 0;
			PositionTrack.ucTrackInterval = 1;
			PositionTrack.usTrackScope = 1;
		}
		else
		{
			PositionTrack.ucTrackInterval = *p++;
			PositionTrack.usTrackScope = 60 * (*p++);
		}
		if(0==PositionTrack.ucTrackModel)
			SSData.usTimer = 1;
	 }
	
	aMsgSendData[MSG_HEAD_LEN]   = (head.usSq>>8) & 0xff;
	aMsgSendData[MSG_HEAD_LEN+1] =  head.usSq     & 0xff;
	aMsgSendData[MSG_HEAD_LEN+2] = 0x00;
	aMsgSendData[MSG_HEAD_LEN+3] = 0x02;
	aMsgSendData[MSG_HEAD_LEN+4] = 'L';
	aMsgSendData[MSG_HEAD_LEN+5] = 'T';
	
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, 6, 0, head.usSq);
	usSendLen += 6;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);

	if(SRCDEVICE_ID_SETTOOL==head.ucSrc)
	{
		PC_SendToPCData2(aMsgSendData, usSendLen+1);
		OSTimeDly(OS_TICKS_PER_SEC/50);
		usMsgBodyLen = BuildSS(&aMsgSendData[MSG_HEAD_LEN]);
		usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_PUSH_DATA, usMsgBodyLen, 0, head.usSq);
		usSendLen += usMsgBodyLen;
		aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
		PC_SendToPCData2(aMsgSendData, usSendLen+1);      
	}
	else
	{
	//	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);
	}
	
	return 0;
}


/******************************************************************************
** 函数名称: DealSerCmd_RemoteCtrl
** 功能描述: 处理远程控制指令--复位,初始化
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-24
******************************************************************************/
uint16 DealSerCmd_RemoteCtrl(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint8 *p = pCmdBody;
	uint16 usTemp,usLen,usSendLen;
	uint32 uiTemp;
	uint8 ucTemp;
	uint8 ucFlag = 0;
	uint8 *pBuf = &aMsgSendData[MSG_HEAD_LEN];
	uint8 ucRespFlag = 0;	//响应服务器标志,0=响应,1=不用响应(针对需要发送命令给MCU的情况)

	usTemp = (*p<<8) + *(p+1);
	p += 2;
	usLen = (*p<<8) + *(p+1);
	p += 2;
	switch(usTemp)
	{
		case 0x4fff://调试模式设置
			if(1==usLen)
				g_stuSystem.ucDebugPrint = *p;
			else
				ucFlag = 1;
			break;
		case 0x4000://终端重启控制
			if(0==usLen)
			{
				ucTemp = FLASH_OB_GetUser();
				if(0!=(0x01 & ucTemp))
				{
					FLASH_OB_Unlock();
					FLASH_OB_UserConfig(OB_IWDG_HW, OB_STOP_NoRST, OB_STDBY_NoRST);
					FLASH_OB_Launch();
					FLASH_OB_Lock();
				}
				SYS_Reset(5);

			}
			else
				ucFlag = 1;
			break;
		case 0x4008://终端立即关机	
			if(0==usLen)
				g_stuSystem.ucShutDownTime = 5;
			else
				ucFlag = 1;
			break;
		case 0x4001://终端设备参数初始化	
			if(0==usLen)
			{
			//	SysParaToFactory();
				SYS_Reset(5);
			}
			else
				ucFlag = 1;
			break;
		case 0x4004:     //标准锁车指令	
			ucTemp = *p;
			if(0==ucTemp)		//解锁
			{
				m_stuMcuCmd.LockControlFlag |= BIT(2);
				m_stuMcuCmd.LockControlFlag &= ~BIT(3);
				m_stuMcuCmd.LockControlFlag |= BIT(4);
				m_stuMcuCmd.LockControlFlag &= ~BIT(5);
				m_stuMcuCmd.usUnLockSq = head.usSq;
			//	McuSaveToMemory();
			}
			else if(1==ucTemp)	//一级锁车:怠速运行:1200rpm
			{
				m_stuMcuCmd.LockControlFlag |= BIT(2);
				m_stuMcuCmd.LockControlFlag |= BIT(3);
				m_stuMcuCmd.usLockOneSq = head.usSq;
			//	McuSaveToMemory();
				
			}
			else if(2==ucTemp)  //二级锁车:禁止启动
			{
				m_stuMcuCmd.LockControlFlag |= BIT(4);
				m_stuMcuCmd.LockControlFlag |= BIT(5);
				m_stuMcuCmd.usLockSecSq = head.usSq;
			//	McuSaveToMemory();
			}
			else
			{
				return 0;
			}
		    if(1==GetCanRcvState(CAN_CHANNEL1))
		    {
				ucRespFlag = 1;	
		//		WorkDataRep.usTimer = 8;
		        SSData.usTimer = 8;
			}
			else
			{
				ucFlag = 1;
			}			
			break;
		case 0x4009:         //监控模式	
			ucTemp = *p;
			if(0==ucTemp)    //监控模式关闭
			{
				m_stuMcuCmd.LockControlFlag |= BIT(0);
				m_stuMcuCmd.LockControlFlag &= ~BIT(1);
				m_stuMcuCmd.LockControlFlag &= 0x03;
				m_stuMcuCmd.usMonitorSq = head.usSq;
			//	McuSaveToMemory();
			}
			else if(1==ucTemp) //监控模式开启
			{
				m_stuMcuCmd.LockControlFlag |= BIT(0);
				m_stuMcuCmd.LockControlFlag |= BIT(1);
				m_stuMcuCmd.usMonitorSq = head.usSq;
			//	McuSaveToMemory();
			}
			else 
			{
				return 0;
			}			
			if(1==GetCanRcvState(CAN_CHANNEL1))
			{
                //WorkDataRep.usTimer = 8;
				SSData.usTimer = 8;
				ucRespFlag = 1;
			}
			else
			{
				ucFlag = 1;
			}
			break;			
		default:
			break;
	}
    if(1==ucRespFlag)
		return 0;
	
	*pBuf++ = (head.usSq>>8) & 0xff;
	*pBuf++ =  head.usSq     & 0xff;
	*pBuf++ = 0x00;
	*pBuf++ = 0x02;
	*pBuf++ = 'R';
	*pBuf++ = 'C';
	*pBuf++ = 0x00;
	*pBuf++ = 0x05;
	*pBuf++ = ucFlag;
//	uiTemp = BuildState();
	*pBuf++ = (uiTemp>>24) & 0xff;
	*pBuf++ = (uiTemp>>16) & 0xff;
	*pBuf++ = (uiTemp>>8)  & 0xff;
	*pBuf++ =  uiTemp      & 0xff;

	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CMD_RESP, 13, 0, head.usSq);
	usSendLen += 13;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	usSendLen += 1;
	if(SRCDEVICE_ID_SETTOOL==head.ucSrc)
		PC_SendToPCData2(aMsgSendData, usSendLen);
//	else
//		GSM_SendGprs(aMsgSendData, usSendLen, 0);
	return usSendLen;
}




#if 0
/******************************************************************************
** 函数名称: DealSerCmd_FirmwareUpdate
** 功能描述: 处理服务器下发的"升级固件指令"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-7-19
******************************************************************************/
uint16 DealSerCmd_FirmwareUpdate(uint8 *PtrTxt,STUSYSPactHeader head)
{
	uint8 *p = PtrTxt;
	uint8 type;
	uint16 usTemp;
	uint8 flag = 0;
	uint8 *p1, *p2;
	uint8 ucTemp;
	
	p += PROTOCOL_HEAD_LEN;
	type = *p++;
	
	usTemp = head.usMsgBodyLen - 1;
	if((usTemp<200) && (usTemp>0))
	{
		memset(FirmwareUpdate.aucUrl, 0, usTemp+1);
		memcpy(FirmwareUpdate.aucUrl,p,usTemp);
		p2 = FirmwareUpdate.aucUrl;
		p1 = SearchString(p2, usTemp, "//", 2);
		if(p1)
		{
			ucTemp = p1-p2;
			usTemp -= ucTemp+2;
			if(usTemp>28)
			{	
				p1 += 2;
				p2 = SearchString(p1, usTemp, ":", 1);
				if(p2)
				{
					ucTemp = p2 - p1;
					FirmwareUpdate.ucUserNameLen = ucTemp;
					memcpy(FirmwareUpdate.aucUserName, p1, ucTemp); 
					usTemp -= ucTemp+1;
					if(usTemp>22)
					{
						p2 += 1;
						p1 = SearchString(p2, usTemp, "@", 1);
						if(p1)
						{
							ucTemp = p1 - p2;
							FirmwareUpdate.ucPasswordLen= ucTemp;
							memcpy(FirmwareUpdate.aucPassword, p2, ucTemp); 
							usTemp -= ucTemp+1;
							if(usTemp>16)
							{
								p1 += 1;
								p2 = SearchString(p1, usTemp, ":", 1);
								if(p2)
								{
									ucTemp = p2-p1;
									FirmwareUpdate.ucSerAddrLen = ucTemp;
									memcpy(FirmwareUpdate.aucSerAddr, p1, ucTemp); 
									usTemp -= ucTemp+1;
									if(usTemp>10)
									{
										p2 += 1;
										p1 = SearchString(p2, usTemp, "/", 1);
										if(p1)
										{
											ucTemp = p1-p2;
											FirmwareUpdate.ucSerPortLen = ucTemp;
											memcpy(FirmwareUpdate.aucSerPort, p2, ucTemp); 
											usTemp -= ucTemp;
											if(usTemp>6)
											{
												p2 = strrchr(p1,'/');
												if(p2)
												{
													ucTemp = p2 - p1 +1;
													FirmwareUpdate.ucPathLen = ucTemp;
													memcpy(FirmwareUpdate.aucPath, p1, ucTemp);
													usTemp -= ucTemp;
													
													FirmwareUpdate.ucFileNameLen = usTemp;
													memcpy(FirmwareUpdate.aucFileName, p2+1, usTemp);
													//GSM_OpenFtp();
												}
												else
												{
													PC_SendDebugData((uint8 *)("Path Err"), 8, DEBUG_ANYDATA);
													flag = 1;
												}
											}
											else
											{
												PC_SendDebugData((uint8 *)("e01"), 3, DEBUG_ANYDATA);
												flag = 1;
											}
										}
										else
										{
											PC_SendDebugData((uint8 *)("e02"), 3, DEBUG_ANYDATA);
											flag = 1;
										}
									}
									else
									{
										PC_SendDebugData((uint8 *)("e03"), 3, DEBUG_ANYDATA);
										flag = 1;
									}
								}
								else
								{
									PC_SendDebugData((uint8 *)("e04"), 3, DEBUG_ANYDATA);
									flag = 1;
								}
							}
							else
							{
								PC_SendDebugData((uint8 *)("e05"), 3, DEBUG_ANYDATA);
								flag = 1;
							}
						}
						else
						{
							PC_SendDebugData((uint8 *)("e06"), 3, DEBUG_ANYDATA);
							flag = 1;
						}
					}
					else
					{
						PC_SendDebugData((uint8 *)("e07"), 3, DEBUG_ANYDATA);
						flag = 1;
					}
				}
				else
				{
					PC_SendDebugData((uint8 *)("e08"), 3, DEBUG_ANYDATA);
					flag = 1;
				}
			}
			else
			{
				PC_SendDebugData((uint8 *)("e09"), 3, DEBUG_ANYDATA);
				flag = 1;
			}
		}
		else
		{
			PC_SendDebugData((uint8 *)("e10"), 3, DEBUG_ANYDATA);
			flag = 1;
		}
	}
	else
	{
		PC_SendDebugData((uint8 *)("e11"), 3, DEBUG_ANYDATA);
		flag = 1;
	}
	//usTemp = GpsRespondSerDefCmd(head, flag);
	OSTimeDly(OS_TICKS_PER_SEC);
	if(0==flag)
		GSM_OpenFtp();
	return usTemp;
}
#endif


/******************************************************************************
** 函数名称: DealSerCmd_CmdReq
** 功能描述: 处理服务器下发的"命令请求"
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-8-22
******************************************************************************/
uint16 DealSerCmd_CmdReq(uint8 * PtrTxt, STUSYSPactHeader head)
{
	uint8 *p = PtrTxt;
	uint16 usCmdTypeLen;
	uint8 *pCmdType;
	uint16 usCmdBodyLen;
	uint8 *pCmdBody;
	
	p += MSG_HEAD_LEN;
	usCmdTypeLen = (*p<<8) + *(p+1);
	if((usCmdTypeLen>0) && (usCmdTypeLen<=head.usMsgBodyLen))
	{
		p += 2;
 		pCmdType = p;
		p += usCmdTypeLen;
		usCmdBodyLen = (*p<<8) + *(p+1);
		p += 2;
		pCmdBody = p;
		
		if(0==memcmp(pCmdType, "PW", 2))
		{
			return DealSerCmd_SetPara(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else if(0==memcmp(pCmdType, "PR", 2))
		{
			return DealSerCmd_QueryPara(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else if(0==memcmp(pCmdType, "LT", 2))
		{
			return DealSerCmd_PositionTrack(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		#if 0
		else if(0==memcmp(pCmdType, "WP", 2))
		{
			return DealSerCmd_WorkDataTrack(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		#endif
		else if(0==memcmp(pCmdType, "RC", 2))
		{
			return DealSerCmd_RemoteCtrl(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else
		{
		
		}
	}
	return 0;
}
#if 0


/******************************************************************************
** 函数名称: DealSerCmd_UpgradeNote
** 功能描述: 处理服务器下发的"'UN'远程固件升级通知"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 DealSerCmd_UpgradeNote(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint16 usSendLen;
	uint8 *p =  pCmdBody;
	uint16 usParaLen,usParaId;
	uint16 cmdbodylen;
	
	if(FirmwareUpdate.ucStep>0)
		return 0;
	cmdbodylen = (*p<<8) + *(p+1);
	p += 2;
	FirmwareUpdate.uctype = *p++;
	FirmwareUpdate.ucdev= *p++;

	//1002升级服务器IP
	usParaId = (*p<<8) + *(p+1);
	p += 2;
	usParaLen = (*p<<8) + *(p+1);
	p += 2;
	if((usParaId==0x1002) && (usParaLen==4))
		memcpy(FirmwareUpdate.aucSerIp, p, 4);
	else
		return 0;
	p += 4;

	//1003升级服务器端口号
	usParaId = (*p<<8) + *(p+1);
	p += 2;
	usParaLen = (*p<<8) + *(p+1);
	p += 2;
	if((usParaId==0x1003) && (usParaLen==2))
		FirmwareUpdate.usSerPort = (*p<<8) + *(p+1);
	else
		return 0;
	p += 2;

	//100E 升级服务器协议类型
	usParaId = (*p<<8) + *(p+1);
	p += 2;
	usParaLen = (*p<<8) + *(p+1);
	p += 2;
	if((usParaId==0x100E) && (usParaLen==1))
		FirmwareUpdate.ucSerProtocolType = *p;
	else
		return 0;
	p += 1;

	//100C升级固件名称
	usParaId = (*p<<8) + *(p+1);
	p += 2;
	usParaLen = (*p<<8) + *(p+1);
	p += 2;
	if((usParaId==0x100c) && (usParaLen>0) && (usParaLen<50))
	{
		if((FirmwareUpdate.ucdev==0)&&(memcmp(p, "LRC", 3)!=0))   //判断终端升级时固件名称起始字符
		{
			return 0;
		}
		else
		{
    		FirmwareUpdate.ucSWNameLen = usParaLen;
    		memcpy(FirmwareUpdate.aucSWName, p, usParaLen);
		}
	}
	else
		return 0;
	p += usParaLen;

	//TLV-1005升级固件版本号
	usParaId = (*p<<8) + *(p+1);
	p += 2;
	usParaLen = (*p<<8) + *(p+1);
	p += 2;
	if((usParaId==0x1005) && (usParaLen>0) && (usParaLen<10))
	{
		FirmwareUpdate.ucSWVersionLen = usParaLen;
		memcpy(FirmwareUpdate.aucSWVersion, p, usParaLen);
	}
	else
		return 0;
	p += usParaLen;

	p = &aMsgSendData[MSG_HEAD_LEN];
	*p++ = 0x00;
	*p++ = 0x02;
	*p++ = 'U';
	*p++ = 'N';
	*p++ = 0;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE_ACK, 5, 0, head.usSq);
	usSendLen += 5;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	usSendLen += 1;
	//GSM_SendGprs(aMsgSendData, usSendLen, 0);
	
	OSTimeDly(OS_TICKS_PER_SEC/10);
	if((0!=memcmp(FirmwareUpdate.aucSerIp, g_stuSerAddr.aucIp, 4)) || 
	   (FirmwareUpdate.usSerPort!=g_stuSerAddr.usPort)             ||
	   (FirmwareUpdate.ucSerProtocolType!=g_stuSerAddr.ucProtocolType)
	  )
	{
		memcpy(g_stuSerAddr.aucIp, FirmwareUpdate.aucSerIp, 4); 
	    g_stuSerAddr.usPort = FirmwareUpdate.usSerPort;
	    g_stuSerAddr.ucProtocolType = FirmwareUpdate.ucSerProtocolType;
	//	GSM_LinkReConnet(0);
	}
	FirmwareUpdate.usTimeoutCnt = 60;	//1分钟没有连接上就算失败
	FirmwareUpdate.ucStep = 1;
	return 0;
}

/******************************************************************************
** 函数名称: DealSerCmd_UpgradeQuery
** 功能描述: 处理服务器下发的"'UQ'响应终端固件升级请求(平台->终端)"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 DealSerCmd_UpgradeQuery(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint8 *p =  pCmdBody;
	
	if(0 != *p++)
		return 0;
	FirmwareUpdate.uiSWSize = (*p<<24) + (*(p+1)<<16)+ (*(p+2)<<8)+ *(p+3);
	p += 4;
	FirmwareUpdate.uiCrc = (*p<<24) + (*(p+1)<<16)+ (*(p+2)<<8)+ *(p+3);
	p += 4;
	FirmwareUpdate.usPackets = (*p<<8) + *(p+1);
	p += 2;

	FirmwareUpdate.ucRepeats = 0;
	FirmwareUpdate.usTimeoutCnt = 5;
	FirmwareUpdate.ucStep = 4;
	
	return 0;
}
#endif
/******************************************************************************
** 函数名称: SYS_SendUpgradeQuery
** 功能描述: 向平台发送远程固件升级的请求
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 SYS_SendUpgradeRequest()
{
	extern uint16 f_usUpLoadCmdSn;				//上行命令的流水号
	uint16 usSendLen;
	uint8 ucTemp;
	uint8 *p, *pBak;
	uint16 usTemp;
	
	p = &aMsgSendData[MSG_HEAD_LEN];
	
	*p++ = 0x00;
	*p++ = 0x02;
	*p++ = 'U';
	*p++ = 'Q';

	pBak = p;	//命令内容长度
	p += 2;
	*p++ = (FIRMWARE_PACKET_LEN>>8) & 0xff;
	*p++ =  FIRMWARE_PACKET_LEN     & 0xff;
	usSendLen = 8;

	//TLV-100C升级固件名称
	*p++ = 0x10;
	*p++ = 0x0c;
	*p++ = 0x00;
	*p++ = FirmwareUpdate.ucSWNameLen;
	memcpy(p, FirmwareUpdate.aucSWName, FirmwareUpdate.ucSWNameLen);
	p += FirmwareUpdate.ucSWNameLen;
	usSendLen += FirmwareUpdate.ucSWNameLen+4;

	//TLV-1005升级固件版本号
	*p++ = 0x10;
	*p++ = 0x05;
	*p++ = 0x00;
	*p++ = FirmwareUpdate.ucSWVersionLen;
	memcpy(p, FirmwareUpdate.aucSWVersion, FirmwareUpdate.ucSWVersionLen);
	p += FirmwareUpdate.ucSWVersionLen;
	usSendLen += FirmwareUpdate.ucSWVersionLen+4;

	//TLV-100D当前固件版本号
	*p++ = 0x10;
	*p++ = 0x0D;
	*p++ = 0x00;
	*p++ = SW_VERSION_LEN;
	memcpy(p, SW_VERSION, SW_VERSION_LEN);
	p += SW_VERSION_LEN;
	usSendLen += SW_VERSION_LEN+4;

	usTemp = 14 + FirmwareUpdate.ucSWNameLen + FirmwareUpdate.ucSWVersionLen + SW_VERSION_LEN;
	*pBak++ = (usTemp>>8) & 0xff;
	*pBak   =  usTemp & 0xff;
	
	f_usUpLoadCmdSn++;
	ucTemp= BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, usSendLen, 0, f_usUpLoadCmdSn);
	usSendLen += ucTemp;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	usSendLen += 1;
//	GSM_SendGprs(aMsgSendData, usSendLen, 0);
	return usSendLen;
}

/******************************************************************************
** 函数名称: SYS_SendUpgradeLoad
** 功能描述: 'UL'终端请求下载升级包(终端'平台)
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 SYS_SendUpgradeLoadPacketRequest()
{
	extern uint16 f_usUpLoadCmdSn;				//上行命令的流水号
	uint8 ucTemp;
	uint8 *p,*pBak;
	uint16 usSendLen;
	
	p = &aMsgSendData[MSG_HEAD_LEN];
	
	*p++ = 0x00;
	*p++ = 0x02;
	*p++ = 'U';
	*p++ = 'L';
	pBak = p;//命令内容长度 2byte
	p += 2;
	*p++ = (FirmwareUpdate.usRequestPacketSn>>8) & 0xff;
	*p++ =  FirmwareUpdate.usRequestPacketSn     & 0xff;

	*p++ = (FirmwareUpdate.usPackets>>8) &0xff;
	*p++ =  FirmwareUpdate.usPackets     &0xff;
	usSendLen = 10;
	//100C升级固件名称
	*p++ = 0x10;
	*p++ = 0x0c;
	*p++ = 0;
	*p++ = FirmwareUpdate.ucSWNameLen;
	memcpy(p, FirmwareUpdate.aucSWName, FirmwareUpdate.ucSWNameLen);
	p += FirmwareUpdate.ucSWNameLen;
	usSendLen += FirmwareUpdate.ucSWNameLen+4;
	//1005升级固件版本号
	*p++ = 0x10;
	*p++ = 0x05;
	*p++ = 0;
	*p++ = FirmwareUpdate.ucSWVersionLen;
	memcpy(p, FirmwareUpdate.aucSWVersion, FirmwareUpdate.ucSWVersionLen);
	p += FirmwareUpdate.ucSWVersionLen;
	usSendLen += FirmwareUpdate.ucSWVersionLen+4;
	
	*pBak++ = 0; //命令内容长度 2byte
	*pBak =	FirmwareUpdate.ucSWNameLen + FirmwareUpdate.ucSWVersionLen + 12;
	
	f_usUpLoadCmdSn++;
	ucTemp= BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, usSendLen, 0, f_usUpLoadCmdSn);
	ucTemp += usSendLen;
	aMsgSendData[ucTemp] = SumCalc(aMsgSendData, ucTemp);
	ucTemp += 1;
//	GSM_SendGprs(aMsgSendData, ucTemp, 0);
	return ucTemp;
}


unsigned int crctab[256] =
{
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3, 
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F, 
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D, 
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433, 
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01, 
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F, 
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD, 
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683, 
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1, 
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7, 
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B, 
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79, 
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713, 
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777, 
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45, 
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9, 
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

uint32 GetCrc32(void)
{
	uint32 k, crc;
    uint16 m=0, n=0;
	uint32 nDataLen = 0;
	uint8 *atempbff=Public_Buf;

	nDataLen = (FirmwareUpdate.usPackets-1)*FIRMWARE_PACKET_LEN+FirmwareUpdate.usLastPacketLen;
	crc = 0XFFFFFFFF;
   
	for(k=0; k<nDataLen; k++)
	{
	    if(0==k%FIRMWARE_PACKET_LEN)
	    {
	    	IWdtFeed();
			if(FirmwareUpdate.ucdev==3)  //终端升级
    		    ReadFromFlash(FLASH_PAGEADDR_UPGRADE+m*(FIRMWARE_PACKET_LEN/256),0,FIRMWARE_PACKET_LEN, &atempbff[0]);  //读取数据
            else                         //控制器或者仪表升级
    		    ReadFromFlash(FLASH_Firmware_MCU+m*(FIRMWARE_PACKET_LEN/256),0,FIRMWARE_PACKET_LEN, &atempbff[0]);  //读取数据
		}	
		crc = (crc >> 8) ^ crctab[(crc & 0xFF) ^ atempbff[n]];
		n++;
		if(n==FIRMWARE_PACKET_LEN)
		{
			n = 0;
			m++;
		}
	}

	crc ^= 0xFFFFFFFF;

	return crc;
}

#if 0
/******************************************************************************
** 函数名称: DealSerCmd_UpgradeQuery
** 功能描述: 处理服务器下发的"'UL'响应终端请求下载升级包(平台'终端)"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 DealSerCmd_UpgradeLoad(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	uint8 *p =  pCmdBody;
	uint16 usPackSn, usTotalPacks, usPackLen;
	uint32 uiTemp;
	uint16 Cmdbdlen;

	Cmdbdlen = (*p<<8) + *(p+1);
	if(Cmdbdlen<4)
		return 0;
	p += 2;
	usPackSn = (*p<<8) + *(p+1);
	p += 2; 
	usTotalPacks = (*p<<8) + *(p+1);
	p += 2;
	usPackLen = (*p<<8) + *(p+1);
	p += 2;
	if(usPackSn != FirmwareUpdate.usRequestPacketSn)
		return 0;
	if(usTotalPacks!= FirmwareUpdate.usPackets)
		return 0;
	if(usPackLen == 0)
		return 0;

	if(FirmwareUpdate.ucdev==0)
	    WriteToFlash(FLASH_PAGEADDR_UPGRADE +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
	else
	    WriteToFlash(FLASH_Firmware_MCU +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);

	FirmwareUpdate.usRequestPacketSn++;
	if(FirmwareUpdate.usRequestPacketSn==FirmwareUpdate.usPackets)//传输结束
	{
		FirmwareUpdate.usLastPacketLen = usPackLen;
		uiTemp = GetCrc32();
		if(uiTemp==FirmwareUpdate.uiCrc)
		{
			FirmwareUpdate.ucRet = 0;
			stu_McuFirmware.ucRcvPackflag = 1;
			stu_McuFirmware.ucLoadStep = 1;
			stu_McuFirmware.usReadFlashSN = 0;
			if(FirmwareUpdate.ucdev)
			    stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;

			PC_SendDebugData((uint8 *)("FM Load OK"), 10, DEBUG_ANYDATA);
		}
		else
		{
			FirmwareUpdate.ucRet = 1;
			stu_McuFirmware.ucRcvPackflag = 0;
            stu_McuFirmware.ucLoadStep = 0;

			PC_SendDebugData((uint8 *)("FM CRC ERR"), 10, DEBUG_ANYDATA);
		}
		FirmwareUpdate.ucStep = 6;
		FirmwareUpdate.usRequestPacketSn = 0;
		return 0;
	}

	FirmwareUpdate.ucRepeats = 0;
	FirmwareUpdate.usTimeoutCnt = 5;
	FirmwareUpdate.ucStep = 4;
	
	return 0;
}
#endif
/******************************************************************************
** 函数名称: SYS_SendUpgradeRet
** 功能描述: 向平台上报升级结果
** 输    入: 
** 输    出: 无
** 返    回: 
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
void SYS_SendUpgradeRet()
{
	uint8 *p = &aMsgSendData[MSG_HEAD_LEN];
	uint16 usTemp;
	
	*p++ = 0;
	*p++ = 2;
	*p++ = 'U';
	*p++ = 'R';
	*p++ = FirmwareUpdate.ucRet;

	usTemp = BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, 5, 0, f_usUpLoadCmdSn);
	usTemp += 5;
	aMsgSendData[usTemp] = SumCalc(aMsgSendData, usTemp);
//	GSM_SendGprs(aMsgSendData, usTemp+1, 0);
}

#if 0
/******************************************************************************
** 函数名称: DealSerCmd_UpgradeRet
** 功能描述: 处理服务器响应的"'UR'响应终端上报升级结果(平台->终端)"指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 DealSerCmd_UpgradeRet(STUSYSPactHeader head,uint16 usCmdTypeLen,uint8 *pCmdType,uint16 usCmdBodyLen,uint8 *pCmdBody)
{
	//CPU_SR cpu_sr;
	uint8 *p = pCmdBody;
 
	if((0==*p) && (0==FirmwareUpdate.ucRet))
	{
		FirmwareUpdate.ucStep = 0;
		if(stu_McuFirmware.ucLoadStep==7&&FirmwareUpdate.ucdev)
		{
            stu_McuFirmware.ucLoadStep = 0;
			stu_McuFirmware.ucRcvPackflag = 0;
			return 0;
		}
		CPU_IntDis();//0413
		//CPU_CRITICAL_ENTER();
		//__disable_irq();
    	IAP_Start();            /*Enable interrupt and reset system    */   
		//__enable_irq();
		//CPU_CRITICAL_EXIT();
    	CPU_IntEn();
	}
	return 0;
}
#endif
/******************************************************************************
** 函数名称: ReConnectWhenUpgradeFaild
** 功能描述: 当升级失败时，重新连接到原服务器
** 输    入: 无 
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2016-11-21
******************************************************************************/
void ReConnectWhenUpgradeFaild()
{
	if((0!=memcmp(FirmwareUpdate.aucSerIp, g_stuSerAddr.aucIp, 4)) || 
	   (FirmwareUpdate.usSerPort!=g_stuSerAddr.usPort) ||
	   (FirmwareUpdate.ucSerProtocolType!=g_stuSerAddr.ucProtocolType))
	
	memcpy(g_stuSerAddr.aucIp, g_stuSYSParamSet.aucHostIP, 4);
	g_stuSerAddr.usPort = g_stuSYSParamSet.usHostPort;
	g_stuSerAddr.ucProtocolType = g_stuSYSParamSet.ucHostProtocolType;
	//GSM_LinkReConnet(0);
}

#if 0
/******************************************************************************
** 函数名称: UpgradeManage
** 功能描述: 管理远程固件升级的步骤
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
void UpgradeManage()
{
	if(0==FirmwareUpdate.ucStep)
		return;
	switch(FirmwareUpdate.ucStep)
	{
		case 1:
		//	if(GPRS_LINK_STATE_READY==GSM_GetLinkState(0))
				FirmwareUpdate.ucStep = 2;
		//	break;
		case 2:
			SYS_SendUpgradeRequest();
			FirmwareUpdate.usTimeoutCnt = 5;
			FirmwareUpdate.ucRepeats++;
			FirmwareUpdate.ucStep = 3;
			break;
		case 3:	
			break;
		case 4:	
			SYS_SendUpgradeLoadPacketRequest();
			FirmwareUpdate.usTimeoutCnt = 10;
			FirmwareUpdate.ucRepeats++;
			FirmwareUpdate.ucStep = 5;
			break;	
		case 5:
			break;
		case 6:
			SYS_SendUpgradeRet();
			FirmwareUpdate.usTimeoutCnt = 5;
			FirmwareUpdate.ucRepeats++;
			FirmwareUpdate.ucStep = 7;
			break;
		default:
			break;
	}
}

void UpgradeTimer()
{
	if(0==FirmwareUpdate.ucStep)
		return;

	if(FirmwareUpdate.usTimeoutCnt>0)
		FirmwareUpdate.usTimeoutCnt--;
	else
	{
		if(1==FirmwareUpdate.ucStep)
		{
			FirmwareUpdate.ucStep = 0;
			ReConnectWhenUpgradeFaild();
		}
		else if(3==FirmwareUpdate.ucStep)
		{
			if(FirmwareUpdate.ucRepeats>5)
			{
				FirmwareUpdate.ucStep = 0;
				FirmwareUpdate.ucRepeats = 0;
				ReConnectWhenUpgradeFaild();
			}
			else
			{
				FirmwareUpdate.ucStep = 2;
			}
		}
		else if(5==FirmwareUpdate.ucStep)
		{
			if(FirmwareUpdate.ucRepeats>5)
			{
				FirmwareUpdate.ucStep = 0;
				FirmwareUpdate.ucRepeats = 0;
				ReConnectWhenUpgradeFaild();
			}
			else
			{
				FirmwareUpdate.ucStep = 4;
			}
		}
		else if(7==FirmwareUpdate.ucStep)
		{
			if(FirmwareUpdate.ucRepeats>5)
			{
				FirmwareUpdate.ucStep = 0;
				FirmwareUpdate.ucRepeats = 0;
				ReConnectWhenUpgradeFaild();
			}
			else
			{
				FirmwareUpdate.ucStep = 6;
			}
		}
	}
}

/******************************************************************************
** 函数名称: DealSerCmd_Upgrade
** 功能描述: 处理服务器下发的远程固件升级相关的指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 无
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-9-19
******************************************************************************/
uint16 DealSerCmd_Upgrade(uint8 *PtrTxt,STUSYSPactHeader head)
{
	uint8 *p = PtrTxt;
	uint16 usCmdTypeLen;
	uint8 *pCmdType;
	uint16 usCmdBodyLen = 0;
	uint8 *pCmdBody;
	
	p += MSG_HEAD_LEN;
	usCmdTypeLen = (*p<<8) + *(p+1);
	if((usCmdTypeLen>0) && (usCmdTypeLen<=head.usMsgBodyLen))
	{
		p += 2;
 		pCmdType = p;
		p += usCmdTypeLen;
		/*
		usCmdBodyLen = (*p<<8) + *(p+1);
		p += 2;
		*/
		pCmdBody = p;
		
		if(0==memcmp(pCmdType, "UN", 2))
		{
			return DealSerCmd_UpgradeNote(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else if(0==memcmp(pCmdType, "UQ", 2))
		{
			return DealSerCmd_UpgradeQuery(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else if(0==memcmp(pCmdType, "UL", 2))
		{
			return DealSerCmd_UpgradeLoad(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else if(0==memcmp(pCmdType, "UR", 2))
		{
			return DealSerCmd_UpgradeRet(head, usCmdTypeLen, pCmdType, usCmdBodyLen, pCmdBody);
		}
		else
		{
		}
	}
	return 0;
}
#endif


/******************************************************************************
** 函数名称: DealSerCmd
** 功能描述: 处理服务器下发的所有指令
** 输    入: PtrTxt,下发数据包指针; 
** 输    出: 
** 返    回: 响应数据包长度
** 作    者: hhm
** 日    期: 2016-7-7
******************************************************************************/
uint16 DealSerCmd(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind)
{
    STUSYSPactHeader STUPactHeader;
    uint8 ucLrc;
    uint8 *cmd;
    uint16 usLen;
	
	usLen = usPtrTxtLen;
    cmd = PtrTxt;

	if(usLen<14)//包头+包尾17字节
	{
		PC_SendDebugData((uint8 *)("ERR:011"), 7, DEBUG_ANYDATA);
		PC_SendDebugData(cmd, usLen, DEBUG_ANYDATA);
		return 0;
	}
	#if 0
	//判断是否连包
	for(j=0; j<usPtrTxtLen-1; j++)
	{
		if(cmd[j] == 0x7E)
		{
			if(cmd[j+1] == 0x7E)//连包
			{
				stuUnnormData.ucDealFlag = 0x03;
				stuUnnormData.ucLen1 = j+1;
				memcpy(stuUnnormData.aucUnpackData1, cmd, stuUnnormData.ucLen1);
				stuUnnormData.ucLen2 = usPtrTxtLen - stuUnnormData.ucLen1;
				memcpy(stuUnnormData.aucUnpackData2, &cmd[j+1], stuUnnormData.ucLen2);
				PC_SendDebugData((uint8 *)("ERR:016"), 7, DEBUG_ANYDATA);
				return 0;
			}
		}
	}
	#endif

	ucLrc = SumCalc(cmd, usLen-1);
    if(ucLrc!=PtrTxt[usLen-1])  
    {
    	PC_SendDebugData((uint8 *)("ERR:013"), 7, DEBUG_ANYDATA);
		return 0;
    }
	usLen -= 1;	//去校验
	STUPactHeader.ucMsgType = *cmd++;
	memcpy(STUPactHeader.aucId, cmd, 7);		//终端ID
	cmd += 7;
	STUPactHeader.ucFlag = *cmd++;
	STUPactHeader.usSq = (*cmd++)<<8;			//报文流水号
	STUPactHeader.usSq += *cmd++;
	STUPactHeader.usMsgBodyLen = (*cmd++)<<8;	//消息长度
    STUPactHeader.usMsgBodyLen += *cmd++;
	STUPactHeader.usMsgBodyLen -= 1;			//消息体长度为剩余长度 - 1字节校验字
	if(usPtrTxtLen != STUPactHeader.usMsgBodyLen+14)	//数据长度冲突
	{
		PC_SendDebugData((uint8 *)("ERR:014"), 7, DEBUG_ANYDATA);
		return 0;
	}
	STUPactHeader.ucSrc = ucSrcAddr;
	
	switch(STUPactHeader.ucMsgType)
	{
		case MSG_TYPE_MESSAGEACK:
			 return DealSerCmd_DefSerRespondGps(PtrTxt, STUPactHeader);
		case MSG_TYPE_REGIST_RESP:
			break;
		case MSG_TYPE_CONN_RESP:
			return DealSerCmd_Connect(PtrTxt, STUPactHeader);
		case MSG_TYPE_CMD_REQ:
			return DealSerCmd_CmdReq(PtrTxt, STUPactHeader);
		case MSG_TYPE_PING_RESP:
			HeartBeat.ucRepeats = 0;
			HeartBeat.ucRepFlag = 0;
			HeartBeat.ucRespondTimer = HEARTBEAT_RESPOND_TIMEOUT;
			break;	
		case MSG_TYPE_UPDATE:
		case MSG_TYPE_UPDATE_ACK:
		//	DealSerCmd_Upgrade(PtrTxt, STUPactHeader);
			break;
		default:
			break;
	}
	return 0;
}

#if 0
/******************************************************************************
** 函数名称: 
** 功能描述: 查询设备当前运行状态命令响应函数
** 
** 输    入: PtrTxt:指针，指向消息头数据地址;
             usPtrTxtLen:整包数据的长度,从GPS命令ID开始到消息内容最后字节结束总数据长度
             ucSrcAddr:消息命令来源地址
** 输    出: 
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据。
             该长度=GPS命令ID长度(2Byte)+命令体长度(可能会为0).
             如果不需要回复的命令，该长度为0
**
** 作    者: Lxf
** 日    期: 2011-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 SYS_GPS_RespQueAllState(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr)
{
    uint32 WorkTime;
    uint16 InputVoltage;
    uint32 Distance;
	uint32 Creg;
	uint8 ucTemp;
  	uint16 usLen;             
    uint16 usGps_CmdLen;
    
    *PtrTxt++=(uint8)(0x0189&0xFF);
    *PtrTxt++=(uint8)(0x0189>>8);
    usGps_CmdLen = *PtrTxt++;
    usGps_CmdLen+=(uint16)((*PtrTxt++)<<8);
    PtrTxt-=2;

    if(usGps_CmdLen!=FALSE)
        return FALSE;

    //usLen  = 传输数据的长度
    usLen = SYS_GPSLOCATION_DATA_LEN+1;           //GPS终端基本信息+上线状态
    *PtrTxt++=usLen&0xFF;
    *PtrTxt++=(uint8)(usLen>>8);
    
    PtrTxt+=MSG_HEADER_DATA_LEN;

     //Get GPS state
    memcpy(PtrTxt,(uint8*)GPS_GetOrient(),21);
    PtrTxt+=21;

    WorkTime = GetWorkingTime();                 // 工作小时数，用于不带CAN信息判断时

  //  WorkTime = GetMcuWorkTime();                   // 工作小时数，用于带CAN信息判断时

    memcpy(PtrTxt,(uint8*)&WorkTime,4);
    PtrTxt+=4;

	ucTemp = GSM_GetCsq();
	ucTemp = ucTemp<<3;
	if(1==GSM_GetSimState())
		ucTemp |= BIT(0);
	if(1==GSM_GetGprsState())
		ucTemp |= BIT(1);
	if(1==GSM_GetCgatt())
		ucTemp |= BIT(2);
	
    *PtrTxt++=ucTemp;                      //GSM state;
    *PtrTxt++=GetBAT_Voltage();                    //内部电压值
    InputVoltage = GetInput_Voltage();

    memcpy(PtrTxt,(uint8*)&InputVoltage,2);        //外部电压值
    PtrTxt+=2;
    *PtrTxt++=0;                                   //保留模拟量低字节
    *PtrTxt++=0;                                   //保留模拟量高字节
    *PtrTxt++=0;                    				//GPS内部温度

    Distance = GPS_GetDistance();
    memcpy(PtrTxt,(uint8*)&Distance,4);            //里程
    PtrTxt+=4;
    *PtrTxt++=0;                                   //横向倾角低字节
    *PtrTxt++=0;                                   //横向倾角高字节
    *PtrTxt++=0;                                   //纵向倾角低字节
    *PtrTxt++=0;                                   //纵向倾角高字节
    *PtrTxt++=GetSwitch1State();                   //开关量采集1
    *PtrTxt++=(GetSwitch2State()&0x0F)|(GetCanCommState(CAN_CHANNEL1)&0xC0); //开关量采集2
		      
    *PtrTxt++=(GetSwitch3State()&0x0F)|(GetCanBusDeviceState()<<6);    //开关量采集3
              
    *PtrTxt++=stuSleep.ucWorkingMode;           //工作模式

    *(PtrTxt-1) |= 0x08;                           //携带LAC数据
	
	*(PtrTxt-1) &= 0xDF;                      

	
	Creg=GSM_GetCreg();
	memcpy(PtrTxt, &Creg, 4);            //增加信息码和小区信息


    PtrTxt+=4;
    //Get GSM state
    //if(g_stuSystem.stuHearBeat.ucGprsOnLine==TRUE)
    if(g_stuSystem.ucOnline==TRUE)
    {
        *PtrTxt = 14;
    }
    else
    {
        //*PtrTxt= GSM_GetAtStep();
        *PtrTxt=0;
    }
    
    return usLen+2+14;

   
}

uint8 SYS_GPS_RespSystemTest(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr)
{
    uint8 ucTestFlag;
    uint16 usGps_CmdLen;
    uint8 atemp[50];
    uint8 ucATlen;
    
    *PtrTxt++=0x8E;
    *PtrTxt++=0x01;
    usGps_CmdLen = *PtrTxt++;
    usGps_CmdLen+=(uint16)((*PtrTxt++)<<8);

    
    PtrTxt+=MSG_HEADER_DATA_LEN;    

    ucTestFlag = *PtrTxt++;
    
    if(ucTestFlag==1)                     //复位系统
    {
		g_stuSystem.ucRstTime = 20;
    }
    else if(ucTestFlag==3)
    {
    }
    else if(ucTestFlag==4)                //关机系统
    {
       // UART0Write((uint8 *)"Gj-time",7);
        g_stuSystem.ucShutDownTime = SYS_SHUTDOWN_TIME;   // 10秒后关机
    }
    else if(ucTestFlag==5)                //AT测试命令
    {

    }
    else if(ucTestFlag==6)
    {

    }
    else if(ucTestFlag==7)
    {
        ucATlen = *PtrTxt++;
        memcpy(&atemp[0],PtrTxt,ucATlen);
        atemp[ucATlen] = 0x0D;
        ucATlen = ucATlen+1;
        GSM_SendToGsm(&atemp[0], ucATlen);        
    }
    else
    {
        return FALSE;
    }

    return usGps_CmdLen+2+14;
}



/******************************************************************************
** 函数名称: SYS_GPS_RespDebug
** 功能描述: 接收调试命令的响应函数,根据命令信息通过调试口输出相应的调试信息
** 
** 输    入: PtrTxt:指针，指向消息头数据地址;
             usPtrTxtLen:整包数据的长度,从GPS命令ID开始到消息内容最后字节结束总数据长度
             ucSrcAddr:消息命令来源地址
** 输    出: 
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据。
             该长度=GPS命令ID长度(2Byte)+命令体长度(可能会为0).
             如果不需要回复的命令，该长度为0
**
** 作    者: Lxf
** 日    期: 2011-06-27
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 SYS_GPS_RespDebug(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr)
{
    uint16 usGps_CmdLen;
    
    *PtrTxt++=(uint8)(0x0183&0xFF);
    *PtrTxt++=(uint8)(0x0183>>8);
    usGps_CmdLen = *PtrTxt++;
    usGps_CmdLen+=(uint16)((*PtrTxt++)<<8);


    if(usGps_CmdLen!=1)
        return FALSE;


    //获取调试模式
    PtrTxt+=MSG_HEADER_DATA_LEN;

    g_stuSystem.ucDebugPrint = *PtrTxt;
	return usGps_CmdLen+2+14;    
}
#endif

#if 0

uint8 SYS_GPS_UPDATEFLASH(uint8_t* PtrTxt,STUSYSPactHeader head)
{
    uint16_t usGps_CmdLen;
    uint16_t index;
    //接收远程升级命令
    //构造应答数据
    //保存ip、port及文件名
    //应答平台后启动远程升级

	//SaveWorktime(0);//临时代码
    
    PtrTxt[1]=(RESPID_UPDATEFLASH&0xFF);
    PtrTxt[2]=(RESPID_UPDATEFLASH>>8);
    PtrTxt[3]=1;
    PtrTxt[4]=0;
    PtrTxt[20]=0;//成功
    
    memcpy(stu_tcp.filename,&PtrTxt[21],4);
    //IP地址
    memcpy(stu_tcp.net,&PtrTxt[26],PtrTxt[25]);
    stu_tcp.net[PtrTxt[25]]=0;
    //PORT端口
    memcpy(stu_tcp.port,&PtrTxt[27+PtrTxt[25]],PtrTxt[26+PtrTxt[25]]);
    stu_tcp.port[PtrTxt[26+PtrTxt[25]]]=0;
    stu_tcp.sum=0;
    stu_tcp.flag = 0xf9;//0xf8;
    for(index=0;index<63;index++)
        stu_tcp.sum+=stu_tcp.net[index];
    ip_write(&stu_tcp);
    
    flag_update=1;
    update_timer=200;
	
    return 21;
}

//服务器地址改变后重启GSM模块,该函数1s执行一次
void SerAddrChangeGsmRst()
{
	static uint8 cnt = 10;

	if(1==g_stuSystem.ucSerAddrChangeFlag)
	{
		if(cnt)
		{
			cnt--;
		}
		else
		{
			g_stuSystem.ucSerAddrChangeFlag = 0;
			GSM_Reset();
		}
	}
	else
	{
		cnt = 10;
	}
}
#endif

/******************************************************************************
** 函数名称: SYS_GetParam
** 功能描述: 获取设备参数配置的借口函数
** 
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
PSTUSYSParamSet SYS_GetParam(void)
{
    return &g_stuSYSParamSet;
}



//处理GSM连包数据，2秒执行一次
void DealUnNormData(void)
{
	#if 0
	if(stuUnnormData.ucDealFlag && g_stuSystem.ucOnline)
	{
		if(stuUnnormData.ucDealFlag & BIT(0))
		{
			stuUnnormData.ucDealFlag &= ~BIT(0);
			f_stuRepeatRecvGprs.pMsgPacket = stuUnnormData.aucUnpackData1;
			f_stuRepeatRecvGprs.usSize = stuUnnormData.ucLen1;
		}
		else if(stuUnnormData.ucDealFlag & BIT(1))
		{
			stuUnnormData.ucDealFlag &=  ~BIT(1);
			f_stuRepeatRecvGprs.pMsgPacket = stuUnnormData.aucUnpackData2;
			f_stuRepeatRecvGprs.usSize = stuUnnormData.ucLen2;
		}
		SYS_PutDataQ((void *)&f_stuRepeatRecvGprs);
	}
	#endif
}




#if 0
/////////////////////////////////////////////////////////////////////////////////////////////////////
//2016-1-26 szj 以下函数属于新添 功能:增加短信文本格式解析和回复
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
//查找指定位置是否存在此字符
extern uint8 filter_flag;//0不启用过滤器 1启用过滤器
extern STUGsmState f_stuGsmState;
extern uint8 aMsgSendData[];

uint8 Search_Char(uint8* PtrTxt,uint8 uchar,uint8 index)
{
	uint8 *buf = PtrTxt+index; 

    if(*buf==uchar)
		return 1;
	else
		return 0;
	 
}


//设置域名和端口号
//注意 此处的dp_len 是包含域名 冒号 端口号的总长度
void Set_Server_ADDR(uint8* Pdns_port,uint8 dp_len,uint16 Pport)
{
	uint8  ucTempLen = 0;
    uint16 old_len=0;
	
	//设置端口号
	aSYSParamData[21]=Pport;//端口号低字节
	aSYSParamData[22]=Pport>>8;//端口号高字节

	//设置域名:端口号 由于apn、中心号码长度、域名长度都是可变的，所以在设置域名时 要找到域名长度的位置
	ucTempLen=aSYSParamData[41];				//获取apn的长度
	ucTempLen=aSYSParamData[ucTempLen+41+1];	//获取中心长度
	ucTempLen=41+aSYSParamData[41]+1+ucTempLen+1;//获取域名长度的位序
	
	old_len=aSYSParamData[ucTempLen];				//备份原域名的长度
	aSYSParamData[ucTempLen]=dp_len;					//先修改域名长度
	memcpy(&aSYSParamData[ucTempLen+1], Pdns_port, dp_len);//再修改域名

	aSYSParamData[2]=aSYSParamData[2]+dp_len-old_len;//修改整体参数长度

    ucTempLen = aSYSParamData[2];
    aSYSParamData[ucTempLen+3] = SumCalc(&aSYSParamData[2], ucTempLen+1);
    SYS_ParamInit(&aSYSParamData[3], 0);                        //对所有参数进行初始化,        
    SYS_ParamWrite();
}

//获取字符与字符之间的长度 注意:在选择ptrtxt 地址时  第一个字符不能作为起始地址 
uint8 GetLen_from_char(uint8* PtrTxt,uint8 uslen,uint8 uchar1,uint8 uchar2)
{
	uint8* buff=PtrTxt;
	uint8 tempbuff[2]={0};
	uint8 i;
	for(i=0;i<uslen;i++)
	{
		if(tempbuff[0]==0)//先找第一个字符
		{
			if(buff[i]==uchar1)//字符1
				tempbuff[0]=i;	//记录序号 
		}
		else
		{
			if(tempbuff[1]==0)
			{
				if(buff[i]==uchar2)//字符2
					tempbuff[1]=i;//记录序号
			}
		}
	}
	if(tempbuff[1]<=tempbuff[0])//没查到 uchar1和uchar2之间有数据
		return 0;
	else
	{
		if(tempbuff[1]-tempbuff[0] > uslen)
			return 0;
		else
			return (tempbuff[1]-tempbuff[0]-1);//返回字符与字符间的数据长度
	}
}
//服务器地址设定 此处设置完成后需要重启GSM模块
uint8 SYS_SMS_Resp_DNS_PORT(uint8* PtrTxt,uint8 usLen)
{
	uint8* buff=PtrTxt;
	uint8 i=0,count=0;
	uint8 DNS_LEN=0,PORT_LEN=0;
	uint16 port_hex=0;
	//char str[20]={0};
	
	for(i=0;i<usLen;i++)
	{
		if(buff[i]==',')
			count++;
	}
	if(count!=1)//说明数据格式不正确 返回失败
	{
		PC_SendDebugData((uint8 *)"格式错误\r", 9, DEBUG_GSMMODULE);
		return 0;
	}
	//先检测短信内容是否超长度 或者有没有数据
	//寻找两个字符间的字符长度
	DNS_LEN=GetLen_from_char(buff,usLen,':',',');//获取域名长度
	PORT_LEN=GetLen_from_char(buff,usLen,',','#');//获取端口长度
	if((DNS_LEN>32)||(DNS_LEN==1))
		return 0;
	if((PORT_LEN>6)||(PORT_LEN==1))	//数据无 或超过6位
		return 0;
	buff=&PtrTxt[DNS_LEN+4];//指到','之后的一个字符
	for(i=0;i<PORT_LEN;i++)
	{
		if((buff[i]>='0')&&(buff[i]<='9'))//端口号必须是数字
		{
			//将字符转换成hex
			port_hex=port_hex*10+(buff[i]-'0');
		}
		else
			return 0;
	}
	//sprintf(str,"port=%5d\r\n",port_hex);
	//PC_SendDebugData((uint8 *)str, sizeof(str), DEBUG_GSMMODULE);
	//端口号数据大小长度限制 不得超过0xffff;
	if(port_hex>65535)
		return 0;
	buff=PtrTxt;
	//需要将','装换成':'
	*(buff+DNS_LEN+3)=':';
	Set_Server_ADDR(buff+3,DNS_LEN+PORT_LEN+1,(uint16)port_hex);//整体拷贝域名端口号 和':'
	//需要复位GSM模块
	g_stuSystem.ucSerAddrChangeFlag = 1;
	return 1;
}
//回复全部CANID上报设定 取消滤波器 短信回复内容: #0:2,OK# #0:2,ERR#
uint8 SYS_SMS_Resp_Can_All(uint8* PtrTxt,uint8 usLen)
{
	//PC_SendDebugData((uint8 *)"恢复默认\n", 9, DEBUG_GSMMODULE);//此处为了调试

	//检测数据内容是否为"can"
	if(0==Search_Char(PtrTxt,'c',3))
		return 0;
	if(0==Search_Char(PtrTxt,'a',4))
		return 0;
	if(0==Search_Char(PtrTxt,'n',5))
		return 0;
	//CanIdSave_To_All();//恢复默认采集全部ID
	//McuInit();			//重新初始化can设置 关闭过滤器

	//if(filter_flag)//滤波器已启动
		return 0;
	//else		   //全部ID上报
		//return 1;
}
//复位终端设定 此处需要先回复 后终端复位 
uint8 SYS_SMS_Resp_Reset_CPU(uint8* PtrTxt,uint8 usLen)
{
	g_stuSystem.ucRstTime = 30;           //复位 系统
	return 1;
}
//查询终端参数1
uint8 SYS_SMS_Resp_Param_1(uint8* PtrTxt,uint8 usLen)
{
	double item=0;
	uint32 integer=0;//整数
	uint16 dot=0;
	//CSQ小于10的 十位补0
	//sprintf((char)Sms_buff,"#a:CSQ%02d,LINKok,LON%3d",)//格式化输出
	//由于定长度了
	aMsgSendData[0] ='#';//短信头
	aMsgSendData[1] ='a';//查询命令码
	aMsgSendData[2] =':';//分隔符
	aMsgSendData[3] ='C';
	aMsgSendData[4] ='S';
	aMsgSendData[5] ='Q';
	aMsgSendData[6] =':';//分隔符
	aMsgSendData[7] =f_stuGsmState.ucCSQ/10+'0';//信号强度
	aMsgSendData[8] =f_stuGsmState.ucCSQ%10+'0';
	aMsgSendData[9] =',';//分隔符
	aMsgSendData[10]='L';
	aMsgSendData[11]='I';
	aMsgSendData[12]='N';
	aMsgSendData[13]='K';
	aMsgSendData[14]=':';//分隔符
	//if(f_bOnlineFlag==1)//GSM链接正常
	if(1==IsGprsReady())
	{
		aMsgSendData[15]='o';
		aMsgSendData[16]='k';
	}
	else//GSM链接异常
	{
		aMsgSendData[15]='e';
		aMsgSendData[16]='r';
	}
	aMsgSendData[17]=',';//分隔符
	aMsgSendData[18]='L';
	aMsgSendData[19]='O';
	aMsgSendData[20]='N';
	aMsgSendData[21]=':';//分隔符
	
	
	item=GPS_GetOrient()->lLongitude/60000.0;	//获取经度

	integer=(uint32)item;//获取整数部分
	dot=(uint16)(item*1000-integer*1000);//获取小数部分
	aMsgSendData[22]=integer/100 + '0';//经度整数3位
	aMsgSendData[23]=(integer%100)/10 + '0';
	aMsgSendData[24]=integer%10 + '0';
	
	aMsgSendData[25]='.';//分隔符
	
	aMsgSendData[26]=dot/100 + '0';//经度小数点后3位
	aMsgSendData[27]=(dot%100)/10 + '0';
	aMsgSendData[28]=dot%10 + '0';

	item=GPS_GetOrient()->lLatitude/60000.0;	   //获取纬度
	
	integer=(uint32)item;//获取整数部分
	dot=(uint16)(item*1000-integer*1000);//获取小数部分
	aMsgSendData[34]=integer/10 + '0';//纬度整数2位
	aMsgSendData[35]=integer%10 + '0';
	
	aMsgSendData[36]='.';//分隔符
	
	aMsgSendData[37]=dot/100 + '0';//纬度小数点后3位
	aMsgSendData[38]=(dot%100)/10 + '0';
	aMsgSendData[39]=dot%10 + '0';
	//sprintf((char *)(&aMsgSendData[22]),"%06.3f",item);//格式化输出
	//sprintf((char *)(&aMsgSendData[34]),"%05.2f",item);//格式化输出
	
	#if 0
	else
	{
		aMsgSendData[22]='0';//经度整数3位
		aMsgSendData[23]='0';
		aMsgSendData[24]='0';
		aMsgSendData[25]='.';//分隔符
		aMsgSendData[26]='0';//经度小数点后3位
		aMsgSendData[27]='0';
		aMsgSendData[28]='0';

		aMsgSendData[34]='0';//纬度整数2位
		aMsgSendData[35]='0';
		aMsgSendData[36]='.';//分隔符
		aMsgSendData[37]='0';//纬度小数点后3位
		aMsgSendData[38]='0';
		aMsgSendData[39]='0';
	}
	#endif
	aMsgSendData[29]=',';//分隔符
	aMsgSendData[30]='L';
	aMsgSendData[31]='A';
	aMsgSendData[32]='T';
	aMsgSendData[33]=':';//分隔符
	
	aMsgSendData[40]=',';//分隔符
	aMsgSendData[41]='A';
	aMsgSendData[42]='N';
	aMsgSendData[43]='T';
	aMsgSendData[44]=':';//分隔符
	if(GPS_GetAnteState()==0)//获取GPS天线状态 正常
	{
		aMsgSendData[45]='o';
		aMsgSendData[46]='k';
	}
	else//获取GPS天线状态 异常
	{
		aMsgSendData[45]='e';
		aMsgSendData[46]='r';
	}
	aMsgSendData[47]=',';//分隔符
	aMsgSendData[48]='O';
	aMsgSendData[49]='R';
	aMsgSendData[50]='I';
	aMsgSendData[51]=':';//分隔符
	if(1==GPS_GetOrientState())//获取GPS定位状态	定位
	{
		aMsgSendData[52]='o';
		aMsgSendData[53]='k';
	}
	else//获取GPS定位状态 不定位
	{
		aMsgSendData[52]='e';
		aMsgSendData[53]='r';
	}
	aMsgSendData[54]='#';//短信尾

	return 55;
}
/******************************************************************************
** 函数名称: SYS_SMS_CommandAll_Execution_Universal
** 功能描述: 接受的短信文本进行命令码解析
** 
** 输    入: PtrTxt:指针
             usPtrTxtLen:整包数据的长度,
             ucKind:消息数据类型 ,报警或者恢复
** 输    出: 无
** 返    回: 命令执行完成后会返回相应的长度，长度为无符号16位数据，
**
** 作    者: szj
** 日    期: 2016-01-26
**-----------------------------------------------------------------------------
*******************************************************************************/
//先检查此短信是否符合通信协议规定方式 
uint8 SMS_CheckData(uint8* PtrTxt,uint8 usPtrTxtLen,uint8* cmd)
{
	if(0==Search_Char(PtrTxt,'#',0))//先比较短信头和尾 # #
		return 0;
	if(0==Search_Char(PtrTxt,'#',usPtrTxtLen-1))
		return 0;
	if(0==Search_Char(PtrTxt,':',2))//第二个字符为:
		return 0;
	//至此检查基本上结束
	//获取字符格式的命令码，检查是否符合协议标准
	if ((PtrTxt[1] >= '0') && (PtrTxt[1] <= '9')) 
	{ 
		*cmd = PtrTxt[1] - '0'; 
	} 
	else if ((PtrTxt[1] >= 'A') && (PtrTxt[1] <= 'F'))    /*A....F*/ 
	{ 
		*cmd = PtrTxt[1] - 0x37; 
	} 
	else if((PtrTxt[1] >= 'a') && (PtrTxt[1] <= 'f'))      /*a....f */
	{ 
		*cmd = PtrTxt[1] - 0x57; 
	} 
	else
	{
		return 0;
	}
	return 1;
	
}
//短信通用回复组包
uint8 Build_SMS_Universal(uint8 flag,uint8 cmd)
{
	if(flag==0)//设置失败
	{
		aMsgSendData[0]='#';
		aMsgSendData[1]='0';
		aMsgSendData[2]=':';
		aMsgSendData[3]=cmd+'0';
		aMsgSendData[4]=',';
		aMsgSendData[5]='E';
		aMsgSendData[6]='R';
		aMsgSendData[7]='R';
		aMsgSendData[8]='#';
		return 9;
	}
	else if(flag==1)//设置成功
	{
		aMsgSendData[0]='#';
		aMsgSendData[1]='0';
		aMsgSendData[2]=':';
		aMsgSendData[3]=cmd+'0';
		aMsgSendData[4]=',';
		aMsgSendData[5]='O';
		aMsgSendData[6]='K';
		aMsgSendData[7]='#';
		return 8;
	}
	else
		return 0;
}
uint16 SYS_SMS_CommandAll_Execution_Universal(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind)
{
	uint8 SmsCmd=0;
	uint8 noerr=0;

	if(0)//此处应该添加是否设置了中心号码的判断
		return 0;
	
	//先检查此短信是否符合通信协议规定方式 
	noerr=SMS_CheckData(PtrTxt,usPtrTxtLen,&SmsCmd);
	if(noerr==1)//获取无误 进行命令码解析
	{
		switch(SmsCmd)
		{
			//以下是设置命令码
			case 0x00:
					return 0;
			case 0x01:
					return Build_SMS_Universal(SYS_SMS_Resp_DNS_PORT(PtrTxt, usPtrTxtLen) ,0x01);//设置服务器地址 需要回复短信
			case 0x02:
					return Build_SMS_Universal(SYS_SMS_Resp_Can_All(PtrTxt, usPtrTxtLen)  ,0x02);//恢复全部ID上传 需要回复短信
			case 0x03:
					return Build_SMS_Universal(SYS_SMS_Resp_Reset_CPU(PtrTxt, usPtrTxtLen),0x03);//复位终端 需要回复短信
			//以下是查询参数命令码 
			case 0x0a:
					return SYS_SMS_Resp_Param_1(PtrTxt,usPtrTxtLen);//查询参数1 需要回复短信
			default:
					return 0;
		}
	}
	else
		return 0;
	
}

#endif
//-----文件SystemCommand.c结束---------------------------------------------


