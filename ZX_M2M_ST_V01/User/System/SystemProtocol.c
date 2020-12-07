/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: SystemProtocol.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪSystem����ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-02, by  , �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------

#include "config.h"
#include "SystemProtocol.h"
#include "SystemCommand.h"

#define SYS_RECEIVDATAQ_SIZE	10							//Systemģ��������ݵ���Ϣ����	
OS_EVENT    *m_SYS_ReceivDataQ;               				//Systemģ��������ݵ���Ϣ����
void        *m_SYS_ReceivDataQBuff[SYS_RECEIVDATAQ_SIZE];	//��Ϣ�������Ϊ6
STUSystem g_stuSystem;
STU_SYSCounter SysCounter;
uint8   g_ucRstCount=0;            			//�ݶ�����������
uint16  g_usRstTime=GSM_RESTTIME;       	//�ݶ�����ʱ����
uint32  g_uiSysTime=0;       	            //�洢������ʱ���ʱ��


STUSYSParamSet g_stuSYSParamSet = {
	.aucDeviceID = {0xA0,0xC1,0x10,0x09,0,0,6},			//�ն˵�ID
	.ucAPNLen = 5,
	.aucAPN = "CMNET",
	.aucUser = "hhm",						//M2Mƽ̨��¼�û���
	.ucUserLen = 3,							//M2Mƽ̨��¼�û�������
	.aucPassword = {1,2,3},					//M2Mƽ̨��¼����
	.ucPasswordLen = 3,						//M2Mƽ̨��¼���볤��
	.aucSmsCenterNum = "15150578385",		//�������ĺ���
	.ucSmsCenterNumLen = 11,				//�������ĺ��볤��
	.aucHostIP = {58,218,196,200},    		//������IP��ַ
	.aucSpareHostIP = {218,80,94,178},		//������IP��ַ
	.usHostPort = 10004,              		//������udp�˿�
	.usSpareHostPort = 6601,             	//������tcp�˿�
	.ucSpareHostProtocolType = 1,			//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��
	.ucHostProtocolType = 1,				//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��
	.uiHeartbeatInterval=240,				//�����������λ����0x0000-����������,Ĭ���������Ϊ30��
	.usCanBrt = 0,							//����CAN���߲�����
	.usCanFmt = 1,							//����CAN���ĸ�ʽ
	//CAN ID �������ã�4�ֽ�һ��
	.auiCanId = {
	             0x1ADC01C1,0x1ADC02C1,0x1ADC03C1,0x1ADC04C1,0x1ADC05C1,0x1ADC06C1,0x1ADC07C1,0x1ADC08C1,0x1ADC09C1,
				 0x1ADC0AC1,0x1ADC0BC1,0x1ADC0CC1,0x1ADC21C1,0x1ADC22C1,0x1ADC23C1,0x1ADC24C1,0x1ADC25C1,0x1ADC26C1,
				 0x1ADC27C1,0x1ADC28C1,0x1ADC29C1,0x1ADA23A1,
				 0x0CF00300,0x0CF00400,0x18FFF800,0x18FEDB00,0x18FED900,0x18FFE200,0x18FFF900,0x18FEEE00,0x18FEEF00,
				 0x18FEF200,0x18FEF500,0x18FEF600,0x18FEF700,0x18FEE900,0x18FED000,0x18FED200,0x18FF0300,0x18FFDC00,
				 0x18FEEB00,0x18FEDA00,0x18FF7A00,0x18FEB100,0x18E8E400,0x1CECFF00,0x1CEBFF00,0x18FECA00,0x18FECB00,				 
				 0x18FEE500,0x18FEDF00,0x18FE5600,0x18FD7C00,0x18FD2000,0x18FD3E00},
	.ucCanIdNum=55,							//can id ����
	.usSleepBeforSlot = 300,       			//��������ʱ��,��λ:s
	.usSleepWakeSlot =  115,
	.ucSSRepSlot = 180,
	.ucCanErrTime = 120,					//CAN�����ж�ʱ��
	.ucCanOKTime = 10,						//CAN�ָ������ж�ʱ��
	.ucPwrOffTime = 10,						//�ն˶ϵ�ʱ������
	.ucPwrOnTime = 1,						//�ն��ϵ�ʱ������
	.ucPwrLowVol = 110,						//�ⲿ��Դ�͵�ѹ������ֵ����λ��1%
	.ucPwrLowTime = 10,						//�ⲿ��Դ�͵�ѹ������ʱ���������λ��1s
	.ucBatLowVol = 35,						//�ڲ���Դ�͵�ѹ������ֵ����λ��1%
	.ucBatLowTime = 10,						//�ⲿ��Դ�͵�ѹ������ʱ�����
	.ucGpsAntErrTime = 30,					//�ն����߹��ϱ�����ʱ���������λ��1s
	.ucGpsAntOKTime = 30,					//�ն����߹��ϱ����Ľ��ʱ���������λ��1s
	.ucGpsModuleErrTime = 30,				//�ն�GPSģ����ϱ�����ʱ�����
	.ucGpsModuleOKTime = 10,				//�ն�GPSģ����ϱ��������ʱ���������λ��1s	
	.ucSpeedOver = 60,						//��ʾ���ٱ�����ֵ����λ��1KM/H
    .ucSpeedOverLastTime = 20,				//���ٱ�����ʱ���������λ��1s
    .ucTransportCar = 2,					//�������ƶ����ϳ�������������ֵ����λ��1KM

	.ucDeviceWorkTimeRepCfg = 0x00,			//�豸����ʱ���ͳ�����ò���
	.ucWorkDataRepModel = 0,				//�������������������ݵ����ϴ�ģʽ
	.ucWorkDataRepInterval = 120,			//�������������������������ʱ�䣬��λ��10��
	.ucPosiInforRepModel = 0,				//λ����Ϣ�����ϴ�ģʽ
	.ucPosiInforRepInterval = 60,			//λ����Ϣ�ϴ����
};


uint8 aMsgSendData[1024];


uint16 f_usUpLoadCmdSn;				//�����������ˮ��

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
//-----�ⲿ��������------------------------------------------------------------
extern STU_McuCmd m_stuMcuCmd;
extern uint8 Public_Buf[];
extern  STUSleep stuSleep; 
stuMcuResp McuResp;
extern STU_CanFrame CanData[];
extern stuXWFaultCode g_stuXWFaultCodeUp;
extern stuXWFaultCode g_stuXWFaultCodeRecv; 
extern STUMCUFirmware stu_McuFirmware;
extern STUKCMCUDownload stu_KCMCUDownload;


//���û���ѡ�ֽ��ж�ȡ���Ź�״̬
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
** ��������: SYS_PutDataQ
** ��������: ����OSQPost()������Ϣ����
** 
** ��    ��: cmd ��������ָ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_PutDataQ(void* ptr)
{
	uint8 ucTemp;
	
    ucTemp = OSQPost(m_SYS_ReceivDataQ, ptr);
	if(OS_ERR_NONE!=ucTemp)
		PC_SendDebugData((uint8 *)("GPRS POST ERR"), 13, DEBUG_ANYDATA);
}

//-----�ⲿ��������------------------------------------------------------------
/******************************************************************************
** ��������: SumDataCheck
** ��������: �����ݰ��ۼƺ�У��ĺ˶Ժ���
** 
** ��    ��: add ָ��У�����ݰ���ʼ���ݵ�ָ��,
             len ��ҪУ�����ݵĳ���
** ��    ��: ��
** ��    ��: У��ͨ������TRUE,��ͨ������FALSE
**
** ��    ��: Lxf
** ��    ��: 2011-06-22
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
** ��������: SYS_CreateReceivDataQ
** ��������: ������Ϣ����m_SYS_ReceivDataQ,���Ϊ6
** ��    ��: cmd ��������ָ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: Lxf
** ��    ��: 2011-06-22
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
** ��������: BuildMsgHead
** ��������: ���챨��ͷ
** ��    ��:
** ��    ��: ��
** ��    ��: ����ͷ����
** ��    ��: hhm
** ��    ��: 2016-08-18
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
	usTemp = usMsgBodyLen + 1;//+1ΪУ����
	*buf++ = (usTemp>>8) & 0xff;
	*buf++ =  usTemp & 0xff;
	return 13;
}

/******************************************************************************
** ��������: BuildState
** ��������: ����״̬λ
** ��    ��:
** ��    ��: ��
** ��    ��: ״̬λ
** ��    ��: hhm
** ��    ��: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 BuildState(void)
{
	uint32 uiTemp = 0;

    if(1==GPS_GetOrientState())
		uiTemp |= BIT(31);

	if(0!=stuSleep.ucWorkingMode)
		uiTemp |= BIT(28);
	if(0xaa==g_stuSystem.ucWDTState)	//BIT(27)���ն˽���״̬�� 0������ 1�����ڸ澯�쳣
		uiTemp |= BIT(27);
	if(2==GetCanRcvState(CAN_CHANNEL1))	//�������豸����״̬�� 0�������� 1��δ����
		uiTemp |= BIT(26);
	if(GetCanCommState(CAN_CHANNEL1))//�������豸����״̬��0������ 1�����ڹ����쳣
		uiTemp |= BIT(25);
	if(1==GetAccState())				//ACC״̬
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
	if(GetCanCommState(CAN_CHANNEL1))	//���ն�����ͨ���жϱ�־��0��δ�ж� 1���ж�
		uiTemp |= BIT(16);									
										//15~8Ԥ��
//	if(GPS_GetSpeedoverState())			//�����ٱ�־��0��δ���� 1���ѳ���
//		uiTemp |= BIT(7);
										//1���ϳ�����	
	if(GetBoxOpenState())				//���ն˿��Ǳ�ʶ��0����ǰδ���� 1����ǰ����
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
//	*buf++ =  GPS_GetHeight()&0xff;			//�̶�����20+1�ֽ�
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
** ��������: SYS_SendHeartBeatData
** ��������: ��ƽ̨����PING����
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-7
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
** ��������: HeartbeatRepManage
** ��������: ������ƽ̨����PING����,�˺���һ������һ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-8-22
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
** ��������: SYS_LandonServer
** ��������: ��ƽ̨������������
** ��    ��:
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-08-18
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendConnectData()
{
	uint16 usSendLen;
	uint8 *p, *P2;
	uint16 usTemp;
	
    f_usUpLoadCmdSn++;

	p = &aMsgSendData[MSG_HEAD_LEN];
   	*p++ = 0;					//Э����
	*p++ = 4;
	memcpy(p, "XM2M", 4);
	p += 4;
	*p++ = 1;					//Э��汾
	*p++ = 0x10;	//TLV100D-��ǰ����汾��
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
	*p++ = 0;	//���ӱ�ʶ
	usTemp = 36+SW_VERSION_LEN;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CONN_REQ, usTemp, 0, f_usUpLoadCmdSn);
	usSendLen += usTemp;
	*p = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);    
}


/******************************************************************************
** ��������: BasicWorkDataRepManage
** ��������: ������ƽ̨���ͻ�����������,�˺���һ������һ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-8-22
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
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x02;
	*buf++ = 'S';
	*buf++ = 'S';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV����

	usMsgBodyLen = 7;

	*buf++ = 0x30;				//TLV1-״̬λ��0x3000��
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

	*buf++ = 0x21;				//λ����Ϣ������0x2101��
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//����Դ��ѹֵ��0x3004��
	*buf++ = 0x04;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetInput_Voltage();
	*buf++ =  (usTemp>>8) & 0xff;
	*buf++ =   usTemp     & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;

	*buf++ = 0x30;				//�ն����õ�ص�ѹ��0x3005��
	*buf++ = 0x05;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetBAT_Voltage();
	*buf++ =  0;
	*buf++ =   usTemp & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//GSM�ź�ǿ��
	*buf++ = 0x07;
	*buf++ = 0;
	*buf++ = 1;
	//*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPS���ǿ���
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
//	*buf++ =  GPS_GetSatellitenums();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//ACC ON�ۼ�ʱ�� ��0x3016��
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

	*buf++ = 0x30;//PPP���˶Զ�Э�飩״̬(0x3017)
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

	*buf++ = 0x30;//GSMע��״̬(0x3018)
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
	
	*buf++ = 0x30;//GPRSע��״̬(0x3019)
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

	*buf++ = 0x30;//��ƽ̨����״̬(0x301A)
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

	*buf++ = 0x30;				//Cellular ID���ն��豸���ڵ�С����ʶ
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
		*buf++ = 0x21;//��������������CAN�������ݣ�0x2103��
		*buf++ = 0x03;
		p2 = buf;
		buf +=3;	  //TLV(0x2103)����2byte + CAN���ݰ���1byte
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
		
		*p2++  = (usTemp>>8) & 0xff;   //TLV(0x2103)����
		*p2++  =  usTemp     & 0xff;
		*p2 = num;	//can֡����
		if(0!=num)
		{
		    usMsgBodyLen += usTemp+4;
		    ucTlvNmr++;
		}
		
		//���ϴ���
		*buf++ = 0x30;
		*buf++ = 0xF0;
		p2 = buf;
		buf +=3;	  //TLV(0x30F0)����2byte + CAN���ݰ���1byte
		ucFaultCodeNum = 0;      //��0
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
		
		*p2++  = (usTemp>>8) & 0xff;   //TLV(0x2103)����
		*p2++  =  usTemp     & 0xff;
		*p2 = ucFaultCodeNum;	//can֡����
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
** ��������: SYS_SendSS
** ��������: �ն˻���״̬ͬ�������ϴ�
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-8-24
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
** ��������: ConnectManage
** ��������: ������ƽ̨��������ָ��,�˺���һ������һ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-9
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
** ��������: AlarmDataRepManage()
** ��������: ������ƽ̨���͹�������,�˺���һ������һ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-13
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
** ��������: AddToAlarmList
** ��������: ��������������ʧʱ����ӵ������б�����
** ��    ��: type:��������,����:
            0x01--�豸�������ƶ��������������ѣ�
			0x02--��Χ�����������ѣ�
			0x03--�豸���ٱ������������ѣ�
			0x04--�ն����豸������ͨ�Ź���(�ն�Ӳ������)
			0x05--GPSģ�����(�ն�Ӳ������)
			0x06--GPS���߹���(�ն�Ӳ������)
			0x07--�ն��ⲿ��Դ�͵�ѹ (�ն�Ӳ������)
			0x08--�ն��ⲿ��Դ�ϵ�(�ն�Ӳ������)
			0x09--�ն��ڲ���ص͵�ѹ(�ն�Ӳ������)
			0x0A--SIM�������������ն�Ӳ��������
			0x0B--GPS�ź�ǿ����(�ն�Ӳ������)
			0x0C--GPRS�ź�ǿ����(�ն�Ӳ������)
			0x0D--GSM/GPRSģ�����

			 flag:��������ʧ��ʶ,0=����, 1=��ʧ
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-9-23
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
** ��������: SYS_SendAlarmData
** ��������: ��ƽ̨���ͱ�������
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-9
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
		//�������ݳ���MSBs
		//�������ݳ���LSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+6];
	*buf++ = 0x30;
	*buf++ = 0x0d;
	p = buf;		//���ݳ���
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
** ��������: DTCRepManage()
** ��������: ������ƽ̨����DTC����,�˺���һ������һ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-13
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
** ��������: SYS_SendAlarmData_DTC
** ��������: ��ƽ̨���ͱ�������
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-9
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
		//�������ݳ���MSBs
		//�������ݳ���LSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+7];
	*buf++ = 0x30;
	*buf++ = 0xEF;
	p = buf;		//���ݳ���
	buf += 2;
	p2= buf;		//DTC����
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
** ��������: SYS_SendMcuRespData
** ��������: ��ƽ̨����MCU��ƽ̨������Ӧ������
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-9
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
	if(m_stuMcuCmd.ucCmdCheckFlag==52)   //У��ʧ��
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
** ��������: SYS_SendMcuRespData
** ��������: ��ƽ̨����MCU��ƽ̨������Ӧ������
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-9
*******************************************************************************/
void McuRespManage()
{
	if(McuResp.ucRespondTimer)
		McuResp.ucRespondTimer--;
}

/******************************************************************************
** ��������: SYS_NetDataSend
** ��������: ��GSMģ�鷢��������MCU����
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-23
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendNetData(void)
{	
    if(1==g_stuSystem.ucResetModem)       //��Ҫ��������GSMģ��
    {
        g_stuSystem.ucResetModem = 0;
		Connect.ucSucceedFlag = 0;
		g_stuSystem.ucOnline = 0;
        GsmLedOff();                         //�ر�GSMָʾ��
        GSM_Reset();                        //����GSMģ��
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

	//Զ������
	if(FirmwareUpdate.ucStep > 0)
	{
		UpgradeManage();
		return;
	}

	//��ƽ̨��������ָ��
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

	//��ƽ̨����״̬ͬ������
	if((1==SSData.ucTimeToRepFlag) && (0==SSData.ucRepFlag))
	{
		SSData.ucTimeToRepFlag = 0;
		
		SSData.ucRepeats++;
		SSData.ucRepFlag = 1;
		SSData.ucRespondTimer = SSDATA_RESPOND_TIMEOUT;
		SYS_SendSS();
		return;
	}	
	

	//��ƽ̨��������ͨѶָ��
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
** ��������: SYS_LedDisplay
** ��������: ���豸ָʾ�ƵĿ��ƺ���ʾ,�ú�����Ҫÿ����ִ��һ��
** GPSָʾ��:
   GSMָʾ��:
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_LedDisplay(void)
{
//	static BOOL ucOnline = FALSE;
    static uint8 ucPositionFlag = TRUE;
	static uint8 ucCan1Flag = 0;
	static uint8 ucCan2Flag = 0;

	//GPSָʾ�ƿ���
	if(stuSleep.ucGpsSleepState)
    {
       	GpsLedOff();                     //����GPSָʾ����
    }
	else if(GPS_GetAnteState())  
	{
        GpsLedOn();                      //GPS����δ��(����)
	}
	else
	{
	    if(1==GPS_GetOrientState())
	    {
	        if(ucPositionFlag==TRUE)
	        {
	            ucPositionFlag = FALSE;
	            GpsLedOff();                    //����
	        }
	        else
	        {
	            ucPositionFlag = TRUE;
	            GpsLedOn();                    //����
	        }          
	    }
		else
		{
			GpsLedOff();                       //GPS���� (����)
		}
	}

#if 0
	//CAN1ָʾ�ƿ���
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
	//CAN1ָʾ�ƿ���
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

	//CAN2ָʾ�ƿ���
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
/*���Դָʾ�ƺ���,�ú���ÿ0.5��ִ��һ��
ͨ������ָʾ�ƿ�������˸����4�룬ÿ����˸0.5�룩ָʾ�ն�����Դ����*/
void SYS_POWERLED_Display(void)
{
    static uint8 ucOffcount = 8;
	static uint8 ucBluFlag = 0;  //0-����,1=����
    if(0==GetPwrSupplyState())
    {
        BLU_LED_OFF();           //����Դ���� ����
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
** ��������: SYS_Reset
** ��������: ϵͳ��λ
** ��    ��: delay--��ʱ��λʱ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Reset(uint8 delay)
{
	g_stuSystem.ucRstTime = delay;
}

/******************************************************************************
** ��������: SYS_Shutdown
** ��������: ϵͳ�ػ�
** ��    ��: delay--��ʱ��λʱ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Shutdown(uint8 delay)
{
	g_stuSystem.ucShutDownTime = delay;
}


/******************************************************************************
** ��������: SYS_TimerCount
** ��������: Systemģ���ʱ����,���û���Ϊ10ms,ͬʱ����50ms��100ms��1s��4�ּ�����ʽ
** 
** ��    ��: 
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_TimerCount_NoDelay(void)
{

    static uint16 us2s = 200;
    static  uint8 uc1s = 100;
	
    //GoToUpdate();   
	/*
    if(uc50ms)             //50ms����
    {
        if(!(--uc50ms))
        {
            uc50ms = 5;
        }
    }

    if(uc100ms)            //100ms����
    {
        if(!(--uc100ms))
        {
            uc100ms = 10;
        }
    }    
	*/
	if(us2s)          //����2��
    {
        if(!(--us2s))
        {
            us2s = 200;
		//	DealUnNormData();
        }
    } 
    if(uc1s)               //1s����
    {
        if(!(--uc1s))
        {
            uc1s = 100;

            if(g_stuSystem.ucRstTime)           //ʹϵͳ��λ
            {
                if(!(--g_stuSystem.ucRstTime))
                {
                   while(1);
                }
            }
			
            if(g_stuSystem.ucShutDownTime)  //�ػ�
            {
                if(!(--g_stuSystem.ucShutDownTime))  
                {
                	MainPowOff();
				//	BatPowOff();      //�ػ�
					PSWControl(1);
					PC_SendDebugData((uint8 *)("Power Off"), 9, DEBUG_ANYDATA);
				}
            }

			//�������ͼ�ʱ
            if(HeartBeat.usNoUploadGprsTimer<MAX_NO_UPLOAD_GPRS_TIME)
            {
                HeartBeat.usNoUploadGprsTimer++; 
			}
			/*
			if(SysCounter.usDailyReportCounter!=0)
				SysCounter.usDailyReportCounter--;
			*/
            SYS_LedDisplay();                    //LED ָʾ����ʾ  
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
** ��������: DealGpsMessage
** ��������: 
** 
** ��    ��: PtrTxt:ָ��
             usPtrTxtLen:�������ݵĳ���,
             ucKind:��Ϣ�������� ,�������߻ָ�
** ��    ��: ��
** ��    ��: ����ִ����ɺ�᷵����Ӧ�ĳ��ȣ�����Ϊ�޷���16λ���ݣ�
**
** ��    ��: Lxf
** ��    ��: 2011-07-31
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
** ��������: SYS_CollectModule_ReceiveData_Execution
** ��������: 
** 
** ��    ��: PtrTxt:ָ��
             usPtrTxtLen:�������ݵĳ���,
             ucKind:��Ϣ�������� ,�������߻ָ�
** ��    ��: ��
** ��    ��: ����ִ����ɺ�᷵����Ӧ�ĳ��ȣ�����Ϊ�޷���16λ���ݣ�
**
** ��    ��: Lxf
** ��    ��: 2011-07-31
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
** ��������: SYS_MCUModule_ReceiveData_Execution
** ��������: 
** 
** ��    ��: PtrTxt:ָ��
             usPtrTxtLen:�������ݵĳ���,����MCUCMD_ID ����+MCUCMD���ݳ���
             ucKind:��Ϣ�������� 1:canͨ�ű仯;2:RS232ͨ�ű仯;
             3:�Է��������͸�MCU������Ļظ�;4:�����뷢���仯;5:MCU���ݱ仯
** ��    ��: ��
** ��    ��: ����ִ����ɺ�᷵����Ӧ�ĳ��ȣ�����Ϊ�޷���16λ���ݣ�
**
** ��    ��: Lxf
** ��    ��: 2011-07-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 SYS_MCUModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind)
{
    switch(ucKind)
	{
		case MCU_MSG_KIND_CAN1COMM_ERR:  //MCU״̬�ı�,����MCU���ϴ��봥����Ϣ
	        AddToAlarmList(0x04, 0);
			SSData.usTimer = 0;
			break;
		case MCU_MSG_KIND_CAN1COMM_OK:   //MCU״̬�ı�,����MCU���ϴ��봥����Ϣ
            SSData.usTimer = 1;
	        AddToAlarmList(0x04, 1);
	//		WorkDataRep.usTimer = 7;		//canͨ�ſ�ʼ������������
			break;	
		case MCU_MSG_KIND_CAN1_RCV_START:
	//		WorkDataRep.usTimer = 7;		//canͨ�ſ�ʼ������������
	        SSData.usTimer = 7;
			break;
//		case  MCU_MSG_KIND_CAN2_RCV_START:
			//AddToAlarmList(5, 1);
		//	WorkDataRep.usTimer = 0;		//canͨ�ſ�ʼ������������
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
** ��������: TaskSYS
** ��������: Systemģ��������
** 
** ��    ��: 
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-06-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void TaskSYS(void *pdata)
{
    uint8   err;
    uint16  usDataLen; 
    PSTUSYSMsgBus pSTUSYS_MsgBus;


    pdata = pdata;
	SYS_ParamRead();
    SYS_CreateReceivDataQ();              //������Ϣ����
  //  SysCounterInit();
	ReadWatchDogState();
	
    while(1)
    {	     
        pSTUSYS_MsgBus = (PSTUSYSMsgBus)OSQPend(m_SYS_ReceivDataQ, 10, &err);
        if (err == OS_ERR_NONE)             //�Խ��յ�����Ϣ���ݽ��д���
        {           
            usDataLen = 0;                        
            switch(pSTUSYS_MsgBus->ucSrcDevice)
            {  
            #if 0
                case SRCDEVICE_ID_GSM:    //GSM��Դ����/���ó���
                    if(GSM_MSG_GPRS_RECV==pSTUSYS_MsgBus->ucKind)//GPRS��Ϣ��������Ϣ
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
 					else if(GSM_MSG_RING_RECV==pSTUSYS_MsgBus->ucKind)//�绰
                    {
                        //g_stuSystem.ucUploadType = HEARTBEAT_PHONECALL;
                        //g_stuSystem.stuParamTem.uiHeartbeatSlot =1;
                    }
                    else if(GSM_MSG_ONLINE==pSTUSYS_MsgBus->ucKind)//����
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
 		SYS_WorkMode_Exec();          //����ģʽ����
    }
}


/******************************************************************************
** ��������: SYS_PowerOff_Reset
** ��������: �ն˶ϵ�����������ÿ1Sִ��һ��
**           ��ACC�رճ���ʱ��ﵽ���߻��Ѽ��ʱ��ʱ��֪ͨAM1805���������жϵ�10����
             Ȼ�������ϵ�;֮�����ACC�Գ���ΪOFF״̬����ÿ���24Сʱ֪֪ͨͨAM1805��
             �������жϵ�10����Ȼ�������ϵ硣
           
** ��    ��: 
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2017-03-24
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_PowerOff_Reset(void)
{
    static uint8 ucAccPre = 0;
//	static uint8 ucAccFlag = 0;    //ACC��ON��ΪOFF��־λ

	if((g_stuSystem.uiAccOffTimer>=g_stuSYSParamSet.usSleepWakeSlot*60||
		g_stuSystem.uiResetTimer>=86400)&&!GetAccState())
	{     
        PC_SendDebugData((uint8 *)("RTCPWROFF"), 9, DEBUG_ANYDATA);

		PSWControl(1);            //֪ͨAM1805�Ͽ��ն��ⲿ�͵�ص�Դ
        RTCSetSleepTime(10, 2);   //֪ͨAM1805(RTCоƬ)�ϵ�10����
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
//GPS�ɲ���λ����λ���������ϱ�
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
			SSData.usTimer = 3;              //���������ϱ�
            PC_SendDebugData((uint8 *)("GPS_DingW"), 9, DEBUG_ANYDATA);
		}
	}
	
    ucGPSStatePre = GPS_GetOrientState();
	ucACCPre = GetAccState();
}
#endif
/******************************************************************************
** ��������: TaskSysTimer
** ��������: ������1��ִ��һ�Σ����ڶ�ʱʱ�侫��Ҫ���Ǻܸߵĺ��������ڴ�ִ�У�
             ��������ִ�еĺ���������ϵͳ��ʱ
** �䡡     �� : ��
** �䡡     �� : ��
** ȫ�ֱ���: 
** ����ģ��: 
** ����	 �� : hhm
** �ա�	 �� : 2014��11��24��

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
            Mcu_FirmwareDownload_Function();   //��������������
		else if(stu_KCMCUDownload.ucRcvPackflag&&FirmwareUpdate.ucdev==4)
		    KCMCU_ProgramUpdate_Function();    //�������������
		
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


//-----�ļ�SystemProtocol.c����---------------------------------------------

