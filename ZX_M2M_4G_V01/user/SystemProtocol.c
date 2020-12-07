/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
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
 * 2020-11-30, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------

#include "config.h"
#include "SystemProtocol.h"
#include "SystemCommand.h"

/********************************�ڲ���������**************************************/

/********************************ȫ�ֱ�������**************************************/
STUSystem g_stuSystem;
STU_SYSCounter SysCounter;
uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
uint16 f_usUpLoadCmdSn;				//�����������ˮ��

STU_SSData SSData = {
	.usTimer = 0,
	.ucTimeToRepFlag = 1,
	.ucRepeats = 0,
	.ucRepFlag = 0,
}; 
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
STU_SYS_LEDState g_stuSysLedState;
STUZXTimer g_stuZX_Timer;
//STUZXECUControl g_stuZX_ECUControl;

//�����ն˱�ţ�A0101011100001
STUSYSParamSet g_stuSYSParamSet = {
	//.aucDeviceID = {0xA0,0xC1,0x10,0x09,0,0,6},			//�ն˵�ID
	.aucDeviceID = {0xA0,0x10,0x10,0x11,0x10,0x00,0x01},			//�ն˵�ID
	.ucAPNLen = 5,
	.aucAPN = "cmmtm",
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
	.uiHeartbeatInterval= 240,				//�����������λ����0x0000-����������,Ĭ���������Ϊ30��
	.usCanBrt = 0,							//����CAN���߲�����
	.usCanFmt = 1,							//����CAN���ĸ�ʽ
	//CAN ID �������ã�4�ֽ�һ��
	.auiCanId = { //������
	             0x1ADC01C1,0x1ADC03C1,0x1ADC04C1,0x1ADC05C1,0x1ADC06C1,0x1ADC07C1,0x1ADC08C1,0x1ADC09C1,
				 0x1ADC21C1,0x1ADC22C1,0x1ADC23C1,0x1ADC24C1,0x1ADC27C1,0x1ADC28C1,0x1ADC29C1,0x1ADC30C1,
                 //�Ǳ�
				 0x1ADA30A1, 
				 //������ ��ʮ��+����˹
				 0x0CF00300,0x0CF00400,0x18FFF800,0x18FEDB00,0x18FED900,0x18FFE200,0x18FFF900,0x18FEEE00,
				 0x18FEEF00,0x18FEF200,0x18FEF500,0x18FEF600,0x18FEF700,0x18FEE900,0x18FED000,0x18FED200,
				 0x18FF0300,0x18FFDC00,0x18FEEB00,0x18FEDA00,0x18FF7A00,0x18FEB100,0x18E8E400,0x18FEE500,
				 0x18FEDF00,0x18FE5600,0x18FD7C00,0x18FD2000,0x18FD3E00
				 },
	.ucCanIdNum = 46,			//can id ����
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

/********************************�ⲿ��������**************************************/
extern STU_McuCmd m_stuMcuCmd;
extern uint8 Public_Buf[];
extern  STUSleep stuSleep; 
stuMcuResp McuResp;
extern STU_CanFrame CanData[];
extern STUMCUFirmware stu_McuFirmware;
extern pthread_mutex_t gGprsDataSendMutex;	/* ����GPRS������ */
extern struct STU_Sysstruct STU_Systemstate;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
extern STU_MultiFrame g_stuMultiFrame;
extern uint8 ucOpenTimes;                  //��¼��������
extern uint8 eng_LampStatus;
extern uint16 g_usSTMCU_Ver;

/**********************************************************************************/

#if 0
void SysCounterInit()
{
	SysCounter.usAlarmReportCounter = 0;
	SysCounter.usDailyReportCounter = 0;
	g_stuSystem.ucOnline = FALSE;
	g_stuSystem.ucSerAddrChangeFlag = 0;
	g_stuSystem.ucAccFlag = 0;
}
#endif

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


void ClearNoUploadGprsTimer(void)
{
	HeartBeat.usNoUploadGprsTimer = 0;
}

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
	//if(GetSwitch2State(void))
		//uiTemp |= BIT(30);
	//if(GPS_GetLatitudeHemisphere())
		//uiTemp |= BIT(29);
	if(0!=stuSleep.ucWorkingMode)
		uiTemp |= BIT(28);
	
	//if(0xaa==g_stuSystem.ucWDTState)	//BIT(27)���ն˽���״̬�� 0������ 1�����ڸ澯�쳣
    if(STU_Systemstate.DisassmbleSwitch&BIT(0))
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
	if(GPS_GetGpsState())
		uiTemp |= BIT(19);
	if(GetGpsAntOpenState())
		uiTemp |= BIT(18);
	if(GetGpsAntShortState())
		uiTemp |= BIT(17);
	if(GetCanCommState(CAN_CHANNEL1))	//���ն�����ͨ���жϱ�־��0��δ�ж� 1���ж�
		uiTemp |= BIT(16);									
										//15~8Ԥ��
	if(GPS_GetSpeedoverState())			//�����ٱ�־��0��δ���� 1���ѳ���
		uiTemp |= BIT(7);
										//1���ϳ�����	
	if(GetBoxOpenState())				//���ն˿��Ǳ�ʶ��0����ǰδ���� 1����ǰ����
		uiTemp |= BIT(0);
										
	return uiTemp;
}

/******************************************************************************
** ��������: Build_ZX_State (TLV-0xA501)
** ��������: ��������ͨ��״̬λ
** ��    ��:
** ��    ��: ��
** ��    ��: ���ݳ���
** ��    ��: lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 Build_ZX_State(uint8 *ptr)
{

    uint8 ucByte1 = 0,ucByte2 = 0,ucByte3 = 0,ucByte4 = 0;
	
	*ptr++ = 0xA5;				
	*ptr++ = 0x01;
	*ptr++ = 0;
	*ptr++ = 4;

    if(GetCanRcvState(CAN_CHANNEL2)!=1)
		ucByte1 |= BIT(0);
    if(GetCanRcvState(CAN_CHANNEL1)!=1)
		ucByte1 |= BIT(1);
	//�ϳ�����������״̬	
	//�³�����������״̬
	//B4-B4  ����λ

	//���һ������ݿ���״̬
	//�����������ݿ���״̬
	//���ݻ������ݿ���״̬
	//B3-B7  ����λ

	//GPS��״̬
	//ECU��״̬
	//GPS����״̬
	//ECU����״̬

	//Byte4 B0-B3  ����ԭ��
	//bit0:1-���ģ�0-����
	//bit1:1-SIM�� 0-����
	//bit2:1-��ʱ��0-����
	//bit3:1-GPS���ߣ�0-����
	//VIN��ƽ̨���ù���
	//C1��ȡ״̬
	//VIN��ȡ״̬
	//B7  ����λ

    *ptr++ = ucByte1;
    *ptr++ = ucByte2;
    *ptr++ = ucByte3;
    *ptr++ = ucByte4;
										
	return 8;
}

/******************************************************************************
** ��������: Build_SleepTimer_Counter (TLV-0x301E)
** ��������: �����ն�����ʱ��ͳ������
** ��    ��:
** ��    ��: ��
** ��    ��: ʱ��ͳ������ ������
** ��    ��: lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 Build_SleepTimer_Counter(uint8 *ptr)
{
    *ptr++ = 0x30;
	*ptr++ = 0x1E;
	*ptr++ = 0;
	*ptr++ = 8;
	
    
	return 12;
}


/******************************************************************************
** ��������: BuildPositionInfor (TLV-x2101)
** ��������: ����λ����Ϣ����
** ��    ��:
** ��    ��: ��
** ��    ��: 
** ��    ��: hhm
** ��    ��: 2016-07-02
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 BuildPositionInfor(uint8 *buf)
{
	uint8 ucTemp = 0;
	uint32 uiTemp;
	STU_Date date;
	
	if(0==GPS_GetLongitudeHemisphere())
		ucTemp |= 0x01;
	if(0==GPS_GetLatitudeHemisphere())
		ucTemp |= 0x10;
	*buf++ = ucTemp;
	
	uiTemp = GPS_GetLatitude();
    *buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;
	uiTemp = GPS_GetLongitude();
    *buf++ = (uiTemp>>24)&0xff;
	*buf++ = (uiTemp>>16)&0xff;
	*buf++ = (uiTemp>>8) &0xff;
	*buf++ =  uiTemp     &0xff;

	*buf++ = (GPS_GetSpeed()/10) & 0xff;
	*buf++ = (GPS_GetForDirect()/2)&0xff;
	*buf++ = (GPS_GetHeight()>>8)&0xff;
	*buf++ =  GPS_GetHeight()&0xff;			//�̶�����20+1�ֽ�

	
	if(GPS_GetOrientState())
		date = GPS_GetUtcTime();
	else
		date = GetRTCTime_UTC();
	
	*buf++ =  date.ucYear;
	*buf++ =  date.ucMon;
	*buf++ =  date.ucDay;
	*buf++ =  date.ucHour;
	*buf++ =  date.ucMin;
	*buf++ =  date.ucSec;

	return 19;
}

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
//    if(g_stuSystem.usCanFaultcodeTimer)
//		g_stuSystem.usCanFaultcodeTimer--;
    if(g_stuZX_Timer.usTCWSendTimer)
		g_stuZX_Timer.usTCWSendTimer--;
	
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
				PC_SendDebugData((uint8 *)("hbtrep failed"), 13, DEBUG_GPRS);
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
	*p++ = 2;					//Э��汾(���Ͱ汾)=2 2020-11-26
	*p++ = 0x10;	            //TLV100D-��ǰ����汾��
	*p++ = 0x0d;
	*p++ = 0;
	*p++ = SW_VERSION_LEN;
	memcpy(p, SW_VERSION, SW_VERSION_LEN);
	p += SW_VERSION_LEN;
	*p++ = 0x01;               //TLV-0x0111 ICCID
	*p++ = 0x11;
	*p++ = 0x00;
	*p++ = 20;
	P2 = GSM_GetICCID();
	memcpy(p, P2, 20);
	p += 20;

	//Э�������汾  TLV-0x100F
    *p++ = 0x10;
	*p++ = 0x0F;
	*p++ = 0;
	*p++ = 4;
	*p++ = 0;
	*p++ = 0;
	*p++ = (uint8)(g_usSTMCU_Ver>>8);
	*p++ = (uint8)(g_usSTMCU_Ver&0xFF);
	
	*p++ = 0;	//���ӱ�ʶ
	usTemp = 36+SW_VERSION_LEN+8;
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
				PC_SendDebugData((uint8 *)("ssrep failed"), 12, DEBUG_GPRS);
			}
			else
			{
                SSData.ucTimeToRepFlag = 1;
			}
			SSData.ucRepFlag = 0;
		}
	}
}


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
//	uint8 n=0;
//    uint8 atemp[8]={0,0,0,0,0,0,0,0};
//	uint32 uiID;
//	uint8 ucFaultCodeNum = 0;
	
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
	*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPS���ǿ���
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GPS_GetSatellitenums();
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
	if(1==GSM_GetGsmModuleState())
		ucTemp =4;
	else if(1==GSM_GetSimState())
		ucTemp = 6;
	else
		ucTemp = 0;
	*buf++ = ucTemp;	
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;//GSMע��״̬(0x3018)
	*buf++ = 0x18;
	*buf++ = 0;
	*buf++ = 1;
	if(1==GSM_GetGsmRegState())
		ucTemp =1;
	else 
		ucTemp = 0;
	*buf++ = ucTemp;
	usMsgBodyLen += 5;
	ucTlvNmr++;
	
	*buf++ = 0x30;//GPRSע��״̬(0x3019)
	*buf++ = 0x19;
	*buf++ = 0;
	*buf++ = 1;
	if(1==GSM_GetGprsState())
		ucTemp =1;
	else 
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
	uiTemp = GSM_GetCreg();
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
		else
		{
            buf -= 5;
		}

    //TLV13-������Сʱ�ƣ�0xA001��
        if(g_stuMultiFrame.ucControllerHoursLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x01;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucControllerHoursLen;
    		memcpy(buf,&g_stuMultiFrame.aControllerHours[0],g_stuMultiFrame.ucControllerHoursLen);
            buf += g_stuMultiFrame.ucControllerHoursLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucControllerHoursLen;
        	ucTlvNmr++;
        }
    
    //TLV14-�������汾��Ϣ��0xA002��
        if(g_stuMultiFrame.ucControllerVerLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x02;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucControllerVerLen;
    		memcpy(buf,&g_stuMultiFrame.aControllerVer[0],g_stuMultiFrame.ucControllerVerLen);
            buf += g_stuMultiFrame.ucControllerVerLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucControllerVerLen;
        	ucTlvNmr++;
        }
	}

	if(1==GetCanRcvState(CAN_CHANNEL2))
	{	
    //TLV15-Cummins����������汾�ţ�0xA003��
        if(g_stuMultiFrame.ucCumminsEngineVerLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x03;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucCumminsEngineVerLen;
    		memcpy(buf,&g_stuMultiFrame.aCumminsEngineVer[0],g_stuMultiFrame.ucCumminsEngineVerLen);
            buf += g_stuMultiFrame.ucCumminsEngineVerLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucCumminsEngineVerLen;
        	ucTlvNmr++;
        }
    
    //TLV16-ISUZU��������Ϣ��0xA004��
        if(g_stuMultiFrame.ucISUZUEngineInfoLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x04;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucISUZUEngineInfoLen;
    		memcpy(buf,&g_stuMultiFrame.aISUZUEngineInfo[0],g_stuMultiFrame.ucISUZUEngineInfoLen);
            buf += g_stuMultiFrame.ucISUZUEngineInfoLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucISUZUEngineInfoLen;
        	ucTlvNmr++;
        }
    
    //TLV17-ISUZU����������汾��(0xA005)
        if(g_stuMultiFrame.ucISUZUEngineVerLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x05;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucISUZUEngineVerLen;
    		memcpy(buf,&g_stuMultiFrame.aISUZUEngineVer[0],g_stuMultiFrame.ucISUZUEngineVerLen);
            buf += g_stuMultiFrame.ucISUZUEngineVerLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucISUZUEngineVerLen;
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

/******************************************************************************
** ��������: Build_ZX_TCB
** ��������: �������ͻ�����Ϣ���ݸ�ʽ

** ��    ��: ��
** ��    ��: ��
** ��    ��: ���ݳ���
**
** ��    ��: Lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Build_ZX_TCB(uint8 *buf)
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
    uint8 ucTlvLen = 0;
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'B';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV����

	usMsgBodyLen = 8;


	*buf++ = 0x21;				//λ����Ϣ������0x2101��
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
//TLV2-�ɼ�Э����Ϣ��0xA504��
//TLV3-�ϳ�ϵͳ�汾��0xA505��
//TLV4-�³�ϵͳ�汾��0xA506��
//TLV5-����Ƶ��ͳ��1��0xA5C5��
//TLV6-����Ƶ��ͳ��2��0xA5C6��
//TLV7-��ȫͳ�ƣ�0xA5C7��

            
	ucTlvLen = Build_SleepTimer_Counter(buf);  //����ʱ��ͳ�����ݣ�0x301E��
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;

	p += 4;
	usTemp = usMsgBodyLen - 6;
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}

/******************************************************************************
** ��������: Build_ZX_TCS
** ��������: �������ͻ���״̬ͬ������

** ��    ��: ��
** ��    ��: ��
** ��    ��: ���ݳ���
**
** ��    ��: Lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Build_ZX_TCS(uint8 *buf)
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
    uint8 ucTlvLen = 0;
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'S';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV����

	usMsgBodyLen = 8;

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
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
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
	*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPS���ǿ���
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GPS_GetSatellitenums();
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

	ucTlvLen = GSM_Get4GLac_Cell(buf);  //Cellular ID���ն��豸���ڵ�С����ʶ(TLV-0x301F)
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;

	*buf++ = 0x30;//PPP���˶Զ�Э�飩״̬(0x3017)
	*buf++ = 0x17;
	*buf++ = 0;
	*buf++ = 1;
	if(1==GSM_GetGsmModuleState())
		ucTemp =4;
	else if(1==GSM_GetSimState())
		ucTemp = 6;
	else
		ucTemp = 0;
	*buf++ = ucTemp;	
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;//GSMע��״̬(0x3018)
	*buf++ = 0x18;
	*buf++ = 0;
	*buf++ = 1;
	if(1==GSM_GetGsmRegState())
		ucTemp =1;
	else 
		ucTemp = 0;
	*buf++ = ucTemp;
	usMsgBodyLen += 5;
	ucTlvNmr++;
	
	*buf++ = 0x30;//GPRSע��״̬(0x3019)
	*buf++ = 0x19;
	*buf++ = 0;
	*buf++ = 1;
	if(1==GSM_GetGprsState())
		ucTemp =1;
	else 
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

	ucTlvLen = Build_ZX_State(buf);        //����ͨ��״̬λ��(TLV-0xA501)
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;	
                               
	ucTlvLen = Build_SleepTimer_Counter(buf);  //����ʱ��ͳ�����ݣ�0x301E��
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;

	p += 4;
	usTemp = usMsgBodyLen - 6;
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}

/******************************************************************************
** ��������: Build_ZX_TCW
** ��������: �������͹����ɼ�����

** ��    ��: ��
** ��    ��: ��
** ��    ��: ���ݳ���
**
** ��    ��: Lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Build_ZX_TCW(uint8 *buf)
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

    uint8 ucTlvLen = 0;
	uint16 usCan1DataLen = 0;
	uint16 usCan2DataLen = 0;
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'W';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV����
	usMsgBodyLen = 8;


	*buf++ = 0x21;				//λ����Ϣ������0x2101��
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
	ucTlvNmr++;
	
	if(1==GetCanRcvState(CAN_CHANNEL1))
	{
      //��Ҫ��ѭ��ɨ��CAN1��Ҫ�ϱ���TLV�ĳ���
	
	    //TLV13-������Сʱ�ƣ�0xA001��
        if(g_stuMultiFrame.ucControllerHoursLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x01;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucControllerHoursLen;
    		memcpy(buf,&g_stuMultiFrame.aControllerHours[0],g_stuMultiFrame.ucControllerHoursLen);
            buf += g_stuMultiFrame.ucControllerHoursLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucControllerHoursLen;
        	ucTlvNmr++;
        }  
		//usCan1DataLen = ;
	}

	if(1==GetCanRcvState(CAN_CHANNEL2))
	{	
      //��Ҫ��ѭ��ɨ��CAN2��Ҫ�ϱ���TLV�ĳ���
    //TLV15-Cummins����������汾�ţ�0xA003��
        if(g_stuMultiFrame.ucCumminsEngineVerLen)
        {
        	*buf++ = 0xA0;
        	*buf++ = 0x03;
        	*buf++ = 0;
        	*buf++ = g_stuMultiFrame.ucCumminsEngineVerLen;
    		memcpy(buf,&g_stuMultiFrame.aCumminsEngineVer[0],g_stuMultiFrame.ucCumminsEngineVerLen);
            buf += g_stuMultiFrame.ucCumminsEngineVerLen;
			usMsgBodyLen += 4+g_stuMultiFrame.ucCumminsEngineVerLen;
        	ucTlvNmr++;
        }    
		//usCan2DataLen = ;
	}

    if(usCan1DataLen==0&&usCan2DataLen==0)
		return 0;
	
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
#endif
/******************************************************************************
** ��������: SYS_SendZXTC_Data
** ��������: ����TC�����ϴ�
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2020-11-26
*******************************************************************************/
void SYS_SendZXTC_Data(uint8 ucZX_SendFlag)
{
	uint16 usMsgBodyLen, usSendLen;
	
    f_usUpLoadCmdSn++;
	SSData.usRepSn = f_usUpLoadCmdSn;
	if(ucZX_SendFlag==ZX_SEND_FLAG_TCS)
	    usMsgBodyLen = Build_ZX_TCS(&aMsgSendData[MSG_HEAD_LEN]);
	else if(ucZX_SendFlag==ZX_SEND_FLAG_TCB)
	    usMsgBodyLen = Build_ZX_TCB(&aMsgSendData[MSG_HEAD_LEN]);
    else if(ucZX_SendFlag==ZX_SEND_FLAG_TCW)
	    usMsgBodyLen = Build_ZX_TCW(&aMsgSendData[MSG_HEAD_LEN]);
	else if(ucZX_SendFlag==ZX_SEND_FLAG_TCD)
	    usMsgBodyLen = Build_ZX_TCD(&aMsgSendData[MSG_HEAD_LEN]);
    if(usMsgBodyLen==0)
		return;	
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_PUSH_DATA, usMsgBodyLen, 0, f_usUpLoadCmdSn);
	usSendLen += usMsgBodyLen;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);      
}

//��������
uint16 Build_CanFaultcode(uint8 *buf)
{
	uint8 ucTemp;
	uint16 usTemp;
	uint32 uiTemp;
	uint16 usMsgBodyLen;
	uint8 ucTlvNmr = 0;
	uint8 *p = buf;
	uint8 *p2;

	uint16 usfaultcodelen = 0;
	uint8 *pFC, *pFC2;

    if(GetCanRcvState(CAN_CHANNEL1)!=1&&GetCanRcvState(CAN_CHANNEL2)!=1)  //��·����Ҫ�ж�??
        return FALSE;
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x02;
	*buf++ = 'F';
	*buf++ = 'C';
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


//������,�Ǳ�
	if(GetCanRcvState(CAN_CHANNEL1)==1)  //�жϿ���������CAN֡
	{
    	*buf++ = 0xA8;				//CAN������ϴ���
    	*buf++ = 0x00;
		p2 = buf;
		buf++;
		buf++;
		*buf++ = 1;    //��������������
        *buf++ = (uint8)(Control_CANFrame_ID>>24);
        *buf++ = (uint8)(Control_CANFrame_ID>>16);
        *buf++ = (uint8)(Control_CANFrame_ID>>8);
        *buf++ = (uint8)(Control_CANFrame_ID&0xFF);
		*buf++ = g_stuMultiFrame.ucControlCanDataNum;
		if(g_stuMultiFrame.ucControlCanDataNum)
		{
		    memcpy(buf,&g_stuMultiFrame.aControlData,g_stuMultiFrame.ucControlCanDataNum*8);
            buf += g_stuMultiFrame.ucControlCanDataNum*8;
		}
		//+6=��������������1Byte+ID��4Byte+֡���ݸ���1Byte
		usfaultcodelen += 6+g_stuMultiFrame.ucControlCanDataNum*8;
		*p2++ = (uint8)(usfaultcodelen>>8);
		*p2++ = (uint8)(usfaultcodelen&0xFF);
    	usMsgBodyLen += usfaultcodelen+4;       //����TL��4Byte����
    	ucTlvNmr++;			
	}

//������������
	if(GetCanRcvState(CAN_CHANNEL2)==1)    //�жϿ���������CAN֡
	{
        *buf++ = 0xA8;
    	*buf++ = 0x04;
    	pFC = buf;		//���ݳ���
    	buf += 2;
    	pFC2= buf;		//DTC����
    	buf += 1;
    	*buf++=eng_LampStatus; //���ϵ�״̬
    	ucTemp = GetMcuFaultCode(buf);
    	buf += ucTemp*5;
    	*pFC2 = ucTemp;
    	*pFC++ = 0;
    	if(ucTemp)
    	{
    	    *pFC++ = ucTemp*5+1+1;       //���ӹ��ϵ�״̬
    		usMsgBodyLen += ucTemp*5+5+1;
    	}
    	else
    	{
    	    *pFC++ = ucTemp*5+1;       //dtc==0 �򲻷��͹��ϵ�״̬
    	    usMsgBodyLen += ucTemp*5+5;
    	}	
    	ucTlvNmr++;
	}
	
	p += 4;
	usTemp = usMsgBodyLen - 6;      //ȥ��������ͷ��7���ֽڳ��� (Э����DTC���������ֽڶ�Ӧ�ĳ���)
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}

/******************************************************************************
** ��������: Build_ZX_TCD
** ��������: �������͹�����

** ��    ��: ��
** ��    ��: ��
** ��    ��: ���ݳ���
**
** ��    ��: Lxf
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Build_ZX_TCD(uint8 *buf)
{
	uint8 ucTemp;
	uint16 usTemp;
	uint32 uiTemp;
	uint16 usMsgBodyLen;
	uint8 ucTlvNmr = 0;
	uint8 *p = buf;
	uint8 *p2;

	uint16 usfaultcodelen = 0;
	uint8 *pFC, *pFC2;

    if(GetCanRcvState(CAN_CHANNEL1)!=1&&GetCanRcvState(CAN_CHANNEL2)!=1)  //��·����Ҫ�ж�??
        return FALSE;
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x02;
	*buf++ = 'T';
	*buf++ = 'D';
	*buf++ = 'C';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV����

	usMsgBodyLen = 8;


	*buf++ = 0x21;				//λ����Ϣ������0x2101��
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;

//������,�Ǳ�
	if(GetCanRcvState(CAN_CHANNEL2)==1)  //�ж��ϳ�����״̬
	{
     //0xA507   �ϳ�������
    	*buf++ = 0xA5;				//CAN������ϴ���
    	*buf++ = 0x07;
		p2 = buf;
		buf++;
		buf++;
		*buf++ = 1;    //��������������
        *buf++ = (uint8)(Control_CANFrame_ID>>24);
        *buf++ = (uint8)(Control_CANFrame_ID>>16);
        *buf++ = (uint8)(Control_CANFrame_ID>>8);
        *buf++ = (uint8)(Control_CANFrame_ID&0xFF);
		*buf++ = g_stuMultiFrame.ucControlCanDataNum;
		if(g_stuMultiFrame.ucControlCanDataNum)
		{
		    memcpy(buf,&g_stuMultiFrame.aControlData,g_stuMultiFrame.ucControlCanDataNum*8);
            buf += g_stuMultiFrame.ucControlCanDataNum*8;
		}
		//+6=��������������1Byte+ID��4Byte+֡���ݸ���1Byte
		usfaultcodelen += 6+g_stuMultiFrame.ucControlCanDataNum*8;
		*p2++ = (uint8)(usfaultcodelen>>8);
		*p2++ = (uint8)(usfaultcodelen&0xFF);
    	usMsgBodyLen += usfaultcodelen+4;       //����TL��4Byte����
    	ucTlvNmr++;			
	}

//������������
	if(GetCanRcvState(CAN_CHANNEL1)==1)    //�ж��³�����״̬
	{
	//0xA508-A509
        *buf++ = 0xA8;
    	*buf++ = 0x04;
    	pFC = buf;		//���ݳ���
    	buf += 2;
    	pFC2= buf;		//DTC����
    	buf += 1;
    	*buf++=eng_LampStatus; //���ϵ�״̬
    	ucTemp = GetMcuFaultCode(buf);
    	buf += ucTemp*5;
    	*pFC2 = ucTemp;
    	*pFC++ = 0;
    	if(ucTemp)
    	{
    	    *pFC++ = ucTemp*5+1+1;       //���ӹ��ϵ�״̬
    		usMsgBodyLen += ucTemp*5+5+1;
    	}
    	else
    	{
    	    *pFC++ = ucTemp*5+1;       //dtc==0 �򲻷��͹��ϵ�״̬
    	    usMsgBodyLen += ucTemp*5+5;
    	}	
    	ucTlvNmr++;
	}
	
	p += 4;
	usTemp = usMsgBodyLen - 6;      //ȥ��������ͷ��7���ֽڳ��� (Э����DTC���������ֽڶ�Ӧ�ĳ���)
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}
#if 0
void SYS_SendCanFaultcode(void)
{
	uint16 usMsgBodyLen, usSendLen;
	
    f_usUpLoadCmdSn++;
	SSData.usRepSn = f_usUpLoadCmdSn;
	usMsgBodyLen = Build_CanFaultcode(&aMsgSendData[MSG_HEAD_LEN]);
    if(usMsgBodyLen==0)
		return;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_ALERT, usMsgBodyLen, 0, f_usUpLoadCmdSn);
	usSendLen += usMsgBodyLen;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0); 
}


/******************************************************************************
** ��������: SYS_SendSleepNote
** ��������: ��ƽ̨����������ʾָ��
** ��    ��:
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-07-14
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendSleepNote()
{
	uint16 usSendLen;
	
    f_usUpLoadCmdSn++;
	//aMsgSendData[0] = 		//��Ϣ����
	//aMsgSendData[1] =        	//
    memcpy(&aMsgSendData[2], g_stuSYSParamSet.aucDeviceID, 5);
    aMsgSendData[7] = PROTOCOL_VERSION;      
    aMsgSendData[8] = SUPPLY_CODE;
    aMsgSendData[9] = TERMINAL_TYPE;       
    aMsgSendData[10] = CUSTOMER_CODE;
    aMsgSendData[11] = (uint8)(f_usUpLoadCmdSn>>8);      //������ˮ��
    aMsgSendData[12] = (uint8)(f_usUpLoadCmdSn&0xFF);
    aMsgSendData[13] = UP_CMD_ID_UPLOADSLEEPNOTE;			//����ID
    usSendLen = BuildBasicRepPack(&aMsgSendData[14]);
	usSendLen += 14;
	aMsgSendData[0] = (usSendLen>>8) & 0xff;//��Ϣ����
	aMsgSendData[1] =  usSendLen     & 0xff;       	
	aMsgSendData[usSendLen] = Lrc(aMsgSendData, usSendLen);
	aMsgSendData[usSendLen+1] = 0x0d;
	aMsgSendData[usSendLen+2] = 0x0a;
	GSM_SendGprs(aMsgSendData, usSendLen+3, 0);    
}
#endif


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
				PC_SendDebugData((uint8 *)("connet failed"), 13, DEBUG_GPRS);
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
				PC_SendDebugData((uint8 *)("alrep failed"), 12, DEBUG_GPRS);
			}
			AlarmRep.ucRepFlag = 0;
		}
	}
}

#if 0
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
#endif

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
				PC_SendDebugData((uint8 *)("dtcrep failed"), 13, DEBUG_GPRS);
			}
			DTCRep.ucRepFlag = 0;
		}
	}
}

#if 0
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
#endif
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
** ��������: ��ƽ̨������������(��¼\����\״̬ͬ��\����������Ϣ\Զ������)
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2019-0-18
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendNetData(void)
{	

    if(1==g_stuSystem.ucResetModem)       //��Ҫ��������GSMģ��
    {

		PC_SendDebugData((uint8 *)("GSM RST2"), 8, DEBUG_GPRS);
		GSM_Variable_Init();
        GSM_SetModemWorkingState(8);                     //����GSMģ��
 
        g_stuSystem.ucResetModem = 0;
		Connect.ucSucceedFlag = 0;
		g_stuSystem.ucOnline = 0;
        GSM_LED_Off();                      //�ر�GSMָʾ��
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
				//add lxf 20190328  
				SSData.ucRepFlag = 0;
				SSData.ucRepeats = 0;
			}
		}
		return;
	}

	//��ƽ̨���������ն˻���״̬ͬ������TCS
	if((1==SSData.ucTimeToRepFlag) && (0==SSData.ucRepFlag))
	{
		SSData.ucTimeToRepFlag = 0;
		
		SSData.ucRepeats++;
		SSData.ucRepFlag = 1;
		SSData.ucRespondTimer = SSDATA_RESPOND_TIMEOUT;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCS);
		return;
	}	
	
    if(g_stuSystem.ucOnline)
		ucOpenTimes = 0;

	if((0!=m_stuMcuCmd.ucRespSerFlag) && (0==McuResp.ucRespondTimer))
	{
		McuResp.ucRespondTimer = 3;
		SYS_SendMcuRespData();
		return;
	}
	/*
    if(CanConfig_Data_Send())
		return;
    */

	//��ƽ̨�������ͻ�����Ϣ����TCB
    if(g_stuZX_Timer.ucTCBSendFlag)
    {
        g_stuZX_Timer.ucTCBSendFlag = 0;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCB);
	}
	//��ƽ̨�������͹����ɼ�����TCW
	if(!g_stuZX_Timer.usTCWSendTimer)
	{
        g_stuZX_Timer.usTCWSendTimer = g_stuSYSParamSet.uiHeartbeatInterval;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCW);
	}
		
	//��ƽ̨�������͹�����TCD
    if(!g_stuZX_Timer.usTCDSendTimer)
    {
		g_stuZX_Timer.usTCWSendTimer= ZXTCD_SEND_TIMER;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCD);
    }
	
    //��ƽ̨��������ͨѶָ��
	if(g_stuSYSParamSet.uiHeartbeatInterval > 0)
	{
	    if(0==HeartBeat.usTimer)
		{
			//HeartBeat.usTimer = g_stuSYSParamSet.uiHeartbeatInterval;
			HeartBeat.usTimer = HEART_BEAT_TIMER;
			HeartBeat.ucRepFlag = 1;
			//HeartBeat.ucRepeats++;
			HeartBeat.ucRespondTimer = HEARTBEAT_RESPOND_TIMEOUT;
			SYS_SendHeartBeatData();
			return;
		}
	}	
}


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
	static BOOL ucOnline = FALSE;
	static BOOL ucUserLed = FALSE;
//static uint8 ucPositionFlag = TRUE;
//	static uint8 ucCan1Flag = 0;
	//static uint8 ucCan2Flag = 0;
/*
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
			GpsLedOff();                        //GPS���� (����)

		}
	}
	*/
    //����ָʾ�ƿ���  
    if(stuSleep.ucGsmSleepState)
	{
		 GSM_LED_Off();    
	}
    else if(g_stuSystem.ucOnline)
    {
        if(ucOnline==TRUE)
        {
            ucOnline = FALSE;
            GSM_LED_On();                       //GSM����
        }
        else
        {
            ucOnline = TRUE;
            GSM_LED_Off();                       //GSM����
        }
    }  
    else                                      //���߹�����
    {
        if(GSM_GetAtStep())
        {
            GSM_LED_On();                       //GSM����
        }
    }

    if(ucUserLed==TRUE)
    {
        ucUserLed = FALSE;
        USER_LED_On();                       //GSM����
    }
    else
    {
        ucUserLed = TRUE;
        USER_LED_Off();                      //GSM����
    }
}

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
    //static  uint8 uc50ms=5;
    //static  uint8 uc100ms=10;
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
			//DealUnNormData();
        }
    } 
    if(uc1s)               //1s����
    {
        if(!(--uc1s))
        {
            uc1s = 100;

			
            Ext_Mcu_Timeout_Function();
            if(g_stuSystem.ucRstTime)           //ʹϵͳ��λ
            {
                if(!(--g_stuSystem.ucRstTime))
                {
                    GSM_Modem_RST();
                }
            }
			
            if(g_stuSystem.ucShutDownTime)  //�ػ�
            {
                if(!(--g_stuSystem.ucShutDownTime))  
                {
                /*
                	MainPowOff();
					PSWControl(1);
					PC_SendDebugData((uint8 *)("Power Off"), 9, DEBUG_ANYDATA);
					*/
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
			ConnectManage();
			SSDataRepManage();
			HeartbeatRepManage();	
		
         //   AlarmDataRepManage();
		//	DTCRepManage();
          	UpgradeTimer();
			SerAddrChangeGsmRst();
		//	SYS_WorkModeSleep_Count();
          	McuRespManage();	
			SIMCardStateJudge();
        }
    }
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

	if((g_stuSystem.uiAccOffTimer>=g_stuSYSParamSet.usSleepWakeSlot*60||
		g_stuSystem.uiResetTimer>=86400)&&!GetAccState())
	{     
        PC_SendDebugData((uint8 *)("RTCPWROFF"), 9, DEBUG_ANYDATA);

		PSWControl(1);            //֪ͨAM1805�Ͽ��ն��ⲿ�͵�ص�Դ
        RTCSetSleepTime(10, 2);   //֪ͨAM1805(RTCоƬ)�ϵ�10����
		MainPowOff(); 
		OSTimeDly(OS_TICKS_PER_SEC*3);
		//BatPowOff();
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
#endif
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
            PC_SendDebugData((uint8 *)("GPS_DingW"), 9, DEBUG_GPSMODULE);
		}
	}
	
    ucGPSStatePre = GPS_GetOrientState();
	ucACCPre = GetAccState();
}

/******************************************************************************
** ��������: pthread_TaskSysTimer_Function
** ��������: 10ms��ʱ�߳�
** �䡡     �� : ��
** �䡡     �� : ��
** ȫ�ֱ���: 
** ����ģ��: 
** ����	 �� : 
** �ա�	 �� : 2019��9��7��

**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_TaskSysTimer_Function(void *data)
{ 
    data = data;
	
	while(1)
	{
	    if(stu_ExtMCUUpgrade.ucUpgradeStep)
            usleep(One_MilliSecond*10);       //10ms ��ʱ
        else
			sleep(1);                         //1s ��ʾ
    //10ms ��ʱ��
        if(stu_ExtMCUUpgrade.ucTimer)
    		stu_ExtMCUUpgrade.ucTimer--;
		
        Ext_Mcu_Upgrade_Function();

    #if 0
        if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev)
            Mcu_FirmwareDownload_Function();
		
	    if(stu_McuFirmware.ucRcvPackflag)
		    OSTimeDly(50);   //50ms
        else
			OSTimeDly(OS_TICKS_PER_SEC);  //1S	
	 #endif
	}
}


//-----�ļ�SystemProtocol.c����---------------------------------------------

