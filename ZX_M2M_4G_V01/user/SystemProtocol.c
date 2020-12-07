/*
 * Copyright(c)2020, ½­ËÕĞì¹¤ĞÅÏ¢¼¼Êõ¹É·İÓĞÏŞ¹«Ë¾-ÖÇÄÜÓ²¼şÊÂÒµ²¿
 * All right reserved
 *
 * ÎÄ¼şÃû³Æ: SystemProtocol.c
 * °æ±¾ºÅ  : V1.0
 * ÎÄ¼ş±êÊ¶:
 * ÎÄ¼şÃèÊö: ±¾ÎÄ¼şÎªSystem¹¦ÄÜÄ£¿éĞ­Òé²ã´¦ÀíµÄÎÄ¼ş
 *
 *-----------------------------------------------------------------------------
 * ĞŞ¸Ä¼ÇÂ¼
 *-----------------------------------------------------------------------------
 *
 * 2020-11-30, by  lxf, ´´½¨±¾ÎÄ¼ş
 *
 */

//-----Í·ÎÄ¼şµ÷ÓÃ------------------------------------------------------------

#include "config.h"
#include "SystemProtocol.h"
#include "SystemCommand.h"

/********************************ÄÚ²¿±äÁ¿¶¨Òå**************************************/

/********************************È«¾Ö±äÁ¿¶¨Òå**************************************/
STUSystem g_stuSystem;
STU_SYSCounter SysCounter;
uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
uint16 f_usUpLoadCmdSn;				//ÉÏĞĞÃüÁîµÄÁ÷Ë®ºÅ

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

//²âÊÔÖÕ¶Ë±àºÅ£ºA0101011100001
STUSYSParamSet g_stuSYSParamSet = {
	//.aucDeviceID = {0xA0,0xC1,0x10,0x09,0,0,6},			//ÖÕ¶ËµÄID
	.aucDeviceID = {0xA0,0x10,0x10,0x11,0x10,0x00,0x01},			//ÖÕ¶ËµÄID
	.ucAPNLen = 5,
	.aucAPN = "cmmtm",
	.aucUser = "hhm",						//M2MÆ½Ì¨µÇÂ¼ÓÃ»§Ãû
	.ucUserLen = 3,							//M2MÆ½Ì¨µÇÂ¼ÓÃ»§Ãû³¤¶È
	.aucPassword = {1,2,3},					//M2MÆ½Ì¨µÇÂ¼ÃÜÂë
	.ucPasswordLen = 3,						//M2MÆ½Ì¨µÇÂ¼ÃÜÂë³¤¶È
	.aucSmsCenterNum = "15150578385",		//¶ÌĞÅÖĞĞÄºÅÂë
	.ucSmsCenterNumLen = 11,				//¶ÌĞÅÖĞĞÄºÅÂë³¤¶È
	.aucHostIP = {58,218,196,200},    		//Ö÷ÖĞĞÄIPµØÖ·
	.aucSpareHostIP = {218,80,94,178},		//¸±ÖĞĞÄIPµØÖ·
	.usHostPort = 10004,              		//Ö÷ÖĞĞÄudp¶Ë¿Ú
	.usSpareHostPort = 6601,             	//¸±ÖĞĞÄtcp¶Ë¿Ú
	.ucSpareHostProtocolType = 1,			//¸±ÖĞĞÄ³ĞÔØĞ­ÒéÀàĞÍ£º0£ºUDPĞ­Òé,1: TCPĞ­Òé
	.ucHostProtocolType = 1,				//Ö÷ÖĞĞÄ³ĞÔØĞ­ÒéÀàĞÍ£º0£ºUDPĞ­Òé,1: TCPĞ­Òé
	.uiHeartbeatInterval= 240,				//ĞÄÌø¼ä¸ô£¬µ¥Î»£ºÃë0x0000-²»·¢ËÍĞÄÌø,Ä¬ÈÏĞÄÌø¼ä¸ôÎª30Ãë
	.usCanBrt = 0,							//±¾µØCAN×ÜÏß²¨ÌØÂÊ
	.usCanFmt = 1,							//±¾µØCAN±¨ÎÄ¸ñÊ½
	//CAN ID ¹ıÂËÅäÖÃ£¬4×Ö½ÚÒ»×é
	.auiCanId = { //¿ØÖÆÆ÷
	             0x1ADC01C1,0x1ADC03C1,0x1ADC04C1,0x1ADC05C1,0x1ADC06C1,0x1ADC07C1,0x1ADC08C1,0x1ADC09C1,
				 0x1ADC21C1,0x1ADC22C1,0x1ADC23C1,0x1ADC24C1,0x1ADC27C1,0x1ADC28C1,0x1ADC29C1,0x1ADC30C1,
                 //ÒÇ±í
				 0x1ADA30A1, 
				 //·¢¶¯»ú ÎåÊ®Áå+¿µÃ÷Ë¹
				 0x0CF00300,0x0CF00400,0x18FFF800,0x18FEDB00,0x18FED900,0x18FFE200,0x18FFF900,0x18FEEE00,
				 0x18FEEF00,0x18FEF200,0x18FEF500,0x18FEF600,0x18FEF700,0x18FEE900,0x18FED000,0x18FED200,
				 0x18FF0300,0x18FFDC00,0x18FEEB00,0x18FEDA00,0x18FF7A00,0x18FEB100,0x18E8E400,0x18FEE500,
				 0x18FEDF00,0x18FE5600,0x18FD7C00,0x18FD2000,0x18FD3E00
				 },
	.ucCanIdNum = 46,			//can id ¸öÊı
	.usSleepBeforSlot = 300,       			//½øÈëĞİÃßÊ±¼ä,µ¥Î»:s
	.usSleepWakeSlot =  115,
	.ucSSRepSlot = 180,
	.ucCanErrTime = 120,					//CAN¹ÊÕÏÅĞ¶ÏÊ±¼ä
	.ucCanOKTime = 10,						//CAN»Ö¸´Õı³£ÅĞ¶ÏÊ±¼ä
	.ucPwrOffTime = 10,						//ÖÕ¶Ë¶ÏµçÊ±¼äÌõ¼ş
	.ucPwrOnTime = 1,						//ÖÕ¶ËÉÏµçÊ±¼äÌõ¼ş
	.ucPwrLowVol = 110,						//Íâ²¿µçÔ´µÍµçÑ¹±¨¾¯ãĞÖµ£¬µ¥Î»£º1%
	.ucPwrLowTime = 10,						//Íâ²¿µçÔ´µÍµçÑ¹±¨¾¯µÄÊ±¼ä²ÎÊı£¬µ¥Î»£º1s
	.ucBatLowVol = 35,						//ÄÚ²¿µçÔ´µÍµçÑ¹±¨¾¯ãĞÖµ£¬µ¥Î»£º1%
	.ucBatLowTime = 10,						//Íâ²¿µçÔ´µÍµçÑ¹±¨¾¯µÄÊ±¼ä²ÎÊı
	.ucGpsAntErrTime = 30,					//ÖÕ¶ËÌìÏß¹ÊÕÏ±¨¾¯µÄÊ±¼ä²ÎÊı£¬µ¥Î»£º1s
	.ucGpsAntOKTime = 30,					//ÖÕ¶ËÌìÏß¹ÊÕÏ±¨¾¯µÄ½â³ıÊ±¼ä²ÎÊı£¬µ¥Î»£º1s
	.ucGpsModuleErrTime = 30,				//ÖÕ¶ËGPSÄ£¿é¹ÊÕÏ±¨¾¯µÄÊ±¼ä²ÎÊı
	.ucGpsModuleOKTime = 10,				//ÖÕ¶ËGPSÄ£¿é¹ÊÕÏ±¨¾¯½â³ıµÄÊ±¼ä²ÎÊı£¬µ¥Î»£º1s	
	.ucSpeedOver = 60,						//±íÊ¾³¬ËÙ±¨¾¯ãĞÖµ£¬µ¥Î»£º1KM/H
    .ucSpeedOverLastTime = 20,				//³¬ËÙ±¨¾¯µÄÊ±¼ä²ÎÊı£¬µ¥Î»£º1s
    .ucTransportCar = 2,					//·Ç×ÔÖ÷ÒÆ¶¯£¨ÍÏ³µ£©±¨¾¯¾àÀëãĞÖµ£¬µ¥Î»£º1KM

	.ucDeviceWorkTimeRepCfg = 0x00,			//Éè±¸¹¤×÷Ê±¼ä¶ÎÍ³¼ÆÅäÖÃ²ÎÊı
	.ucWorkDataRepModel = 0,				//¹¤×÷²ÎÊı£¨¹¤¿ö£©Êı¾İµ¥ÌõÉÏ´«Ä£Ê½
	.ucWorkDataRepInterval = 120,			//¹¤×÷²ÎÊı£¨¹¤¿ö£©´«Êä²ÎÊı¡£Ê±¼ä£¬µ¥Î»£º10Ãë
	.ucPosiInforRepModel = 0,				//Î»ÖÃĞÅÏ¢µ¥ÌõÉÏ´«Ä£Ê½
	.ucPosiInforRepInterval = 60,			//Î»ÖÃĞÅÏ¢ÉÏ´«¼ä¸ô
};

/********************************Íâ²¿±äÁ¿¶¨Òå**************************************/
extern STU_McuCmd m_stuMcuCmd;
extern uint8 Public_Buf[];
extern  STUSleep stuSleep; 
stuMcuResp McuResp;
extern STU_CanFrame CanData[];
extern STUMCUFirmware stu_McuFirmware;
extern pthread_mutex_t gGprsDataSendMutex;	/* ·¢ËÍGPRSÊı¾İËø */
extern struct STU_Sysstruct STU_Systemstate;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
extern STU_MultiFrame g_stuMultiFrame;
extern uint8 ucOpenTimes;                  //¼ÇÂ¼¿ª»ú´ÎÊı
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

//-----Íâ²¿º¯Êı¶¨Òå------------------------------------------------------------
/******************************************************************************
** º¯ÊıÃû³Æ: SumDataCheck
** ¹¦ÄÜÃèÊö: ¶ÔÊı¾İ°üÀÛ¼ÆºÍĞ£ÑéµÄºË¶Ôº¯Êı
** 
** Êä    Èë: add Ö¸ÏòĞ£ÑéÊı¾İ°üÆğÊ¼Êı¾İµÄÖ¸Õë,
             len ĞèÒªĞ£ÑéÊı¾İµÄ³¤¶È
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Ğ£ÑéÍ¨¹ı·µ»ØTRUE,²»Í¨¹ı·µ»ØFALSE
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-06-22
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
** º¯ÊıÃû³Æ: BuildMsgHead
** ¹¦ÄÜÃèÊö: ¹¹Ôì±¨ÎÄÍ·
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ±¨ÎÄÍ·³¤¶È
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-08-18
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
	usTemp = usMsgBodyLen + 1;//+1ÎªĞ£Ñé×Ö
	*buf++ = (usTemp>>8) & 0xff;
	*buf++ =  usTemp & 0xff;
	return 13;
}

/******************************************************************************
** º¯ÊıÃû³Æ: BuildState
** ¹¦ÄÜÃèÊö: ¹¹Ôì×´Ì¬Î»
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ×´Ì¬Î»
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-07-01
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
	
	//if(0xaa==g_stuSystem.ucWDTState)	//BIT(27)¡¾ÖÕ¶Ë½¡¿µ×´Ì¬¡¿ 0£ºÕı³£ 1£º´æÔÚ¸æ¾¯Òì³£
    if(STU_Systemstate.DisassmbleSwitch&BIT(0))
	    uiTemp |= BIT(27);
	if(2==GetCanRcvState(CAN_CHANNEL1))	//¡¾¹ØÁªÉè±¸¹¤×÷×´Ì¬¡¿ 0£º¹¤×÷ÖĞ 1£ºÎ´¹¤×÷
		uiTemp |= BIT(26);
	if(GetCanCommState(CAN_CHANNEL1))//¡¾¹ØÁªÉè±¸½¡¿µ×´Ì¬¡¿0£ºÕı³£ 1£º´æÔÚ¹ÊÕÏÒì³£
		uiTemp |= BIT(25);
	if(1==GetAccState())				//ACC×´Ì¬
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
	if(GetCanCommState(CAN_CHANNEL1))	//¡¾ÖÕ¶Ë×ÜÏßÍ¨ĞÅÖĞ¶Ï±êÖ¾¡¿0£ºÎ´ÖĞ¶Ï 1£ºÖĞ¶Ï
		uiTemp |= BIT(16);									
										//15~8Ô¤Áô
	if(GPS_GetSpeedoverState())			//¡¾³¬ËÙ±êÖ¾¡¿0£ºÎ´³¬ËÙ 1£ºÒÑ³¬ËÙ
		uiTemp |= BIT(7);
										//1£ºÍÏ³µ±¨¾¯	
	if(GetBoxOpenState())				//¡¾ÖÕ¶Ë¿ª¸Ç±êÊ¶¡¿0£ºµ±Ç°Î´¿ª¸Ç 1£ºµ±Ç°¿ª¸Ç
		uiTemp |= BIT(0);
										
	return uiTemp;
}

/******************************************************************************
** º¯ÊıÃû³Æ: Build_ZX_State (TLV-0xA501)
** ¹¦ÄÜÃèÊö: ¹¹ÔìÖØĞÍÍ¨ÓÃ×´Ì¬Î»
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Êı¾İ³¤¶È
** ×÷    Õß: lxf
** ÈÕ    ÆÚ: 2020-11-26
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
	//ÉÏ³µ·¢¶¯»ú¹¤×÷×´Ì¬	
	//ÏÂ³µ·¢¶¯»ú¹¤×÷×´Ì¬
	//B4-B4  ±£ÁôÎ»

	//¹ú¼Ò»·±£Êı¾İ¿ªÆô×´Ì¬
	//±±¾©»·±£Êı¾İ¿ªÆô×´Ì¬
	//º¼Öİ»·±£Êı¾İ¿ªÆô×´Ì¬
	//B3-B7  ±£ÁôÎ»

	//GPS°ó¶¨×´Ì¬
	//ECU°ó¶¨×´Ì¬
	//GPSËø³µ×´Ì¬
	//ECUËø³µ×´Ì¬

	//Byte4 B0-B3  Ëø³µÔ­Òò
	//bit0:1-ÖĞĞÄ£¬0-Õı³£
	//bit1:1-SIM£¬ 0-Õı³£
	//bit2:1-³¬Ê±£¬0-Õı³£
	//bit3:1-GPSÌìÏß£¬0-Õı³£
	//VINÂëÆ½Ì¨ÉèÖÃ¹¦ÄÜ
	//C1»ñÈ¡×´Ì¬
	//VIN»ñÈ¡×´Ì¬
	//B7  ±£ÁôÎ»

    *ptr++ = ucByte1;
    *ptr++ = ucByte2;
    *ptr++ = ucByte3;
    *ptr++ = ucByte4;
										
	return 8;
}

/******************************************************************************
** º¯ÊıÃû³Æ: Build_SleepTimer_Counter (TLV-0x301E)
** ¹¦ÄÜÃèÊö: ¹¹ÔìÖÕ¶ËĞİÃßÊ±¼äÍ³¼ÆÊı¾İ
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Ê±¼äÍ³¼ÆÊı¾İ ¡¢³¤¶È
** ×÷    Õß: lxf
** ÈÕ    ÆÚ: 2020-11-26
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
** º¯ÊıÃû³Æ: BuildPositionInfor (TLV-x2101)
** ¹¦ÄÜÃèÊö: ¹¹ÔìÎ»ÖÃĞÅÏ¢Êı¾İ
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: 
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-07-02
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
	*buf++ =  GPS_GetHeight()&0xff;			//¹Ì¶¨²¿·Ö20+1×Ö½Ú

	
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
** º¯ÊıÃû³Æ: SYS_SendHeartBeatData
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍPINGĞÄÌø
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-7
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
** º¯ÊıÃû³Æ: HeartbeatRepManage
** ¹¦ÄÜÃèÊö: ¹ÜÀíÏòÆ½Ì¨·¢ËÍPINGĞÄÌø,´Ëº¯ÊıÒ»ÃëÔËĞĞÒ»´Î
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-8-22
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
** º¯ÊıÃû³Æ: SYS_LandonServer
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍÁ¬½ÓÇëÇó
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-08-18
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendConnectData()
{
	uint16 usSendLen;
	uint8 *p, *P2;
	uint16 usTemp;
	
    f_usUpLoadCmdSn++;

	p = &aMsgSendData[MSG_HEAD_LEN];
   	*p++ = 0;					//Ğ­ÒéÃû
	*p++ = 4;
	memcpy(p, "XM2M", 4);
	p += 4;
	*p++ = 2;					//Ğ­Òé°æ±¾(ÖØĞÍ°æ±¾)=2 2020-11-26
	*p++ = 0x10;	            //TLV100D-µ±Ç°Èí¼ş°æ±¾ºÅ
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

	//Ğ­´¦ÀíÆ÷°æ±¾  TLV-0x100F
    *p++ = 0x10;
	*p++ = 0x0F;
	*p++ = 0;
	*p++ = 4;
	*p++ = 0;
	*p++ = 0;
	*p++ = (uint8)(g_usSTMCU_Ver>>8);
	*p++ = (uint8)(g_usSTMCU_Ver&0xFF);
	
	*p++ = 0;	//Á¬½Ó±êÊ¶
	usTemp = 36+SW_VERSION_LEN+8;
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_CONN_REQ, usTemp, 0, f_usUpLoadCmdSn);
	usSendLen += usTemp;
	*p = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);    
}


/******************************************************************************
** º¯ÊıÃû³Æ: BasicWorkDataRepManage
** ¹¦ÄÜÃèÊö: ¹ÜÀíÏòÆ½Ì¨·¢ËÍ»ù±¾¹¤×÷Êı¾İ,´Ëº¯ÊıÒ»ÃëÔËĞĞÒ»´Î
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-8-22
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
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x02;
	*buf++ = 'S';
	*buf++ = 'S';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı

	usMsgBodyLen = 7;

	*buf++ = 0x30;				//TLV1-×´Ì¬Î»£¨0x3000£©
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

	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//Ö÷µçÔ´µçÑ¹Öµ£¨0x3004£©
	*buf++ = 0x04;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetInput_Voltage();
	*buf++ =  (usTemp>>8) & 0xff;
	*buf++ =   usTemp     & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;

	*buf++ = 0x30;				//ÖÕ¶ËÄÚÖÃµç³ØµçÑ¹£¨0x3005£©
	*buf++ = 0x05;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetBAT_Voltage();
	*buf++ =  0;
	*buf++ =   usTemp & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//GSMĞÅºÅÇ¿¶È
	*buf++ = 0x07;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPSÎÀĞÇ¿ÅÊı
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GPS_GetSatellitenums();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//ACC ONÀÛ¼ÆÊ±¼ä £¨0x3016£©
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

	*buf++ = 0x30;//PPP£¨¶Ë¶Ô¶ËĞ­Òé£©×´Ì¬(0x3017)
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

	*buf++ = 0x30;//GSM×¢²á×´Ì¬(0x3018)
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
	
	*buf++ = 0x30;//GPRS×¢²á×´Ì¬(0x3019)
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

	*buf++ = 0x30;//ÓëÆ½Ì¨Á¬½Ó×´Ì¬(0x301A)
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

	*buf++ = 0x30;				//Cellular ID£¬ÖÕ¶ËÉè±¸ËùÔÚµÄĞ¡Çø±êÊ¶
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
		*buf++ = 0x21;//¹¤×÷²ÎÊıµ¥°ü£ºCAN×ÜÏßÊı¾İ£¨0x2103£©
		*buf++ = 0x03;
		p2 = buf;
		buf +=3;	  //TLV(0x2103)³¤¶È2byte + CANÊı¾İ°üÊı1byte
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
		
		*p2++  = (usTemp>>8) & 0xff;   //TLV(0x2103)³¤¶È
		*p2++  =  usTemp     & 0xff;
		*p2 = num;	//canÖ¡¸öÊı
		if(0!=num)
		{
		    usMsgBodyLen += usTemp+4;
		    ucTlvNmr++;
		}
		else
		{
            buf -= 5;
		}

    //TLV13-¿ØÖÆÆ÷Ğ¡Ê±¼Æ£¨0xA001£©
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
    
    //TLV14-¿ØÖÆÆ÷°æ±¾ĞÅÏ¢£¨0xA002£©
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
    //TLV15-Cummins·¢¶¯»úÈí¼ş°æ±¾ºÅ£¨0xA003£©
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
    
    //TLV16-ISUZU·¢¶¯»úĞÅÏ¢£¨0xA004£©
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
    
    //TLV17-ISUZU·¢¶¯»úÈí¼ş°æ±¾ºÅ(0xA005)
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
** º¯ÊıÃû³Æ: Build_ZX_TCB
** ¹¦ÄÜÃèÊö: ´´½¨ÖØĞÍ»ù±¾ĞÅÏ¢Êı¾İ¸ñÊ½

** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Êı¾İ³¤¶È
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2020-11-26
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
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'B';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı

	usMsgBodyLen = 8;


	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
//TLV2-²É¼¯Ğ­ÒéĞÅÏ¢£¨0xA504£©
//TLV3-ÉÏ³µÏµÍ³°æ±¾£¨0xA505£©
//TLV4-ÏÂ³µÏµÍ³°æ±¾£¨0xA506£©
//TLV5-¶¯×÷Æµ´ÎÍ³¼Æ1£¨0xA5C5£©
//TLV6-¶¯×÷Æµ´ÎÍ³¼Æ2£¨0xA5C6£©
//TLV7-°²È«Í³¼Æ£¨0xA5C7£©

            
	ucTlvLen = Build_SleepTimer_Counter(buf);  //ĞİÃßÊ±¼äÍ³¼ÆÊı¾İ£¨0x301E£©
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
** º¯ÊıÃû³Æ: Build_ZX_TCS
** ¹¦ÄÜÃèÊö: ´´½¨ÖØĞÍ»ù±¾×´Ì¬Í¬²½Êı¾İ

** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Êı¾İ³¤¶È
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2020-11-26
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
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'S';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı

	usMsgBodyLen = 8;

	*buf++ = 0x30;				//TLV1-×´Ì¬Î»£¨0x3000£©
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

	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
	usMsgBodyLen += 23;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//Ö÷µçÔ´µçÑ¹Öµ£¨0x3004£©
	*buf++ = 0x04;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetInput_Voltage();
	*buf++ =  (usTemp>>8) & 0xff;
	*buf++ =   usTemp     & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;

	*buf++ = 0x30;				//ÖÕ¶ËÄÚÖÃµç³ØµçÑ¹£¨0x3005£©
	*buf++ = 0x05;
	*buf++ = 0;
	*buf++ = 2;
	usTemp = GetBAT_Voltage();
	*buf++ =  0;
	*buf++ =   usTemp & 0xff;
	usMsgBodyLen += 6;
	ucTlvNmr++;
	
	*buf++ = 0x30;				//GSMĞÅºÅÇ¿¶È
	*buf++ = 0x07;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GSM_GetCsq();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//GPSÎÀĞÇ¿ÅÊı
	*buf++ = 0x08;
	*buf++ = 0;
	*buf++ = 1;
	*buf++ =  GPS_GetSatellitenums();
	usMsgBodyLen += 5;
	ucTlvNmr++;

	*buf++ = 0x30;				//ACC ONÀÛ¼ÆÊ±¼ä £¨0x3016£©
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

	ucTlvLen = GSM_Get4GLac_Cell(buf);  //Cellular ID£¬ÖÕ¶ËÉè±¸ËùÔÚµÄĞ¡Çø±êÊ¶(TLV-0x301F)
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;

	*buf++ = 0x30;//PPP£¨¶Ë¶Ô¶ËĞ­Òé£©×´Ì¬(0x3017)
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

	*buf++ = 0x30;//GSM×¢²á×´Ì¬(0x3018)
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
	
	*buf++ = 0x30;//GPRS×¢²á×´Ì¬(0x3019)
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

	*buf++ = 0x30;//ÓëÆ½Ì¨Á¬½Ó×´Ì¬(0x301A)
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

	ucTlvLen = Build_ZX_State(buf);        //ÖØĞÍÍ¨ÓÃ×´Ì¬Î»£¬(TLV-0xA501)
    buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
    ucTlvNmr++;	
                               
	ucTlvLen = Build_SleepTimer_Counter(buf);  //ĞİÃßÊ±¼äÍ³¼ÆÊı¾İ£¨0x301E£©
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
** º¯ÊıÃû³Æ: Build_ZX_TCW
** ¹¦ÄÜÃèÊö: ´´½¨ÖØĞÍ¹¤¿ö²É¼¯Êı¾İ

** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Êı¾İ³¤¶È
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2020-11-26
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
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x03;
	*buf++ = 'T';
	*buf++ = 'C';
	*buf++ = 'W';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı
	usMsgBodyLen = 8;


	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTlvLen = BuildPositionInfor(buf);
	buf += ucTlvLen;
	usMsgBodyLen += ucTlvLen;
	ucTlvNmr++;
	
	if(1==GetCanRcvState(CAN_CHANNEL1))
	{
      //ĞèÒª×öÑ­»·É¨ÃèCAN1ĞèÒªÉÏ±¨µÄTLVµÄ³¤¶È
	
	    //TLV13-¿ØÖÆÆ÷Ğ¡Ê±¼Æ£¨0xA001£©
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
      //ĞèÒª×öÑ­»·É¨ÃèCAN2ĞèÒªÉÏ±¨µÄTLVµÄ³¤¶È
    //TLV15-Cummins·¢¶¯»úÈí¼ş°æ±¾ºÅ£¨0xA003£©
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
** º¯ÊıÃû³Æ: SYS_SendSS
** ¹¦ÄÜÃèÊö: ÖÕ¶Ë»ù±¾×´Ì¬Í¬²½Êı¾İÉÏ´«
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-8-24
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
** º¯ÊıÃû³Æ: SYS_SendZXTC_Data
** ¹¦ÄÜÃèÊö: ÖØĞÍTCÊı¾İÉÏ´«
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2020-11-26
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

//¹ÊÕÏÂë´ò°ü
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

    if(GetCanRcvState(CAN_CHANNEL1)!=1&&GetCanRcvState(CAN_CHANNEL2)!=1)  //Á½Â·¶¼ĞèÒªÅĞ¶Ï??
        return FALSE;
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x02;
	*buf++ = 'F';
	*buf++ = 'C';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı

	usMsgBodyLen = 7;

	*buf++ = 0x30;				//TLV1-×´Ì¬Î»£¨0x3000£©
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

	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;


//¿ØÖÆÆ÷,ÒÇ±í
	if(GetCanRcvState(CAN_CHANNEL1)==1)  //ÅĞ¶Ï¿ØÖÆÆ÷·¢ËÍCANÖ¡
	{
    	*buf++ = 0xA8;				//CAN¶à°ü¹ÊÕÏ´úÂë
    	*buf++ = 0x00;
		p2 = buf;
		buf++;
		buf++;
		*buf++ = 1;    //¶à°ü¹ÊÕÏÖÖÀà¸öÊı
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
		//+6=¶à°ü¹ÊÕÏÖÖÀà¸öÊı1Byte+IDµÄ4Byte+Ö¡Êı¾İ¸öÊı1Byte
		usfaultcodelen += 6+g_stuMultiFrame.ucControlCanDataNum*8;
		*p2++ = (uint8)(usfaultcodelen>>8);
		*p2++ = (uint8)(usfaultcodelen&0xFF);
    	usMsgBodyLen += usfaultcodelen+4;       //¼ÓÉÏTLµÄ4Byte³¤¶È
    	ucTlvNmr++;			
	}

//·¢¶¯»ú¹ÊÕÏÂë
	if(GetCanRcvState(CAN_CHANNEL2)==1)    //ÅĞ¶Ï¿ØÖÆÆ÷·¢ËÍCANÖ¡
	{
        *buf++ = 0xA8;
    	*buf++ = 0x04;
    	pFC = buf;		//Êı¾İ³¤¶È
    	buf += 2;
    	pFC2= buf;		//DTC¸öÊı
    	buf += 1;
    	*buf++=eng_LampStatus; //¹ÊÕÏµÆ×´Ì¬
    	ucTemp = GetMcuFaultCode(buf);
    	buf += ucTemp*5;
    	*pFC2 = ucTemp;
    	*pFC++ = 0;
    	if(ucTemp)
    	{
    	    *pFC++ = ucTemp*5+1+1;       //Ôö¼Ó¹ÊÕÏµÆ×´Ì¬
    		usMsgBodyLen += ucTemp*5+5+1;
    	}
    	else
    	{
    	    *pFC++ = ucTemp*5+1;       //dtc==0 Ôò²»·¢ËÍ¹ÊÕÏµÆ×´Ì¬
    	    usMsgBodyLen += ucTemp*5+5;
    	}	
    	ucTlvNmr++;
	}
	
	p += 4;
	usTemp = usMsgBodyLen - 6;      //È¥µôº¯Êı¿ªÍ·µÄ7¸ö×Ö½Ú³¤¶È (Ğ­ÒéÖĞDTCºóÃæÁ½¸ö×Ö½Ú¶ÔÓ¦µÄ³¤¶È)
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;
	
	return usMsgBodyLen;
}

/******************************************************************************
** º¯ÊıÃû³Æ: Build_ZX_TCD
** ¹¦ÄÜÃèÊö: ´´½¨ÖØĞÍ¹ÊÕÏÂë©

** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: Êı¾İ³¤¶È
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2020-11-26
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

    if(GetCanRcvState(CAN_CHANNEL1)!=1&&GetCanRcvState(CAN_CHANNEL2)!=1)  //Á½Â·¶¼ĞèÒªÅĞ¶Ï??
        return FALSE;
	
	*buf++ = 0x00;				//Êı¾İÀàĞÍ³¤¶È
	*buf++ = 0x02;
	*buf++ = 'T';
	*buf++ = 'D';
	*buf++ = 'C';
	buf++;						//Êı¾İÄÚÈİ³¤¶È
	buf++;
	buf++;						//×´Ì¬Í¬²½TLV¸öÊı

	usMsgBodyLen = 8;


	*buf++ = 0x21;				//Î»ÖÃĞÅÏ¢µ¥°ü£¨0x2101£©
	*buf++ = 0x01;
	*buf++ = 0x00;
	*buf++ = 19;
	ucTemp = BuildPositionInfor(buf);
	buf += ucTemp;
	usMsgBodyLen += 23;
	ucTlvNmr++;

//¿ØÖÆÆ÷,ÒÇ±í
	if(GetCanRcvState(CAN_CHANNEL2)==1)  //ÅĞ¶ÏÉÏ³µ½ÓÊÕ×´Ì¬
	{
     //0xA507   ÉÏ³µ¹ÊÕÏÂë
    	*buf++ = 0xA5;				//CAN¶à°ü¹ÊÕÏ´úÂë
    	*buf++ = 0x07;
		p2 = buf;
		buf++;
		buf++;
		*buf++ = 1;    //¶à°ü¹ÊÕÏÖÖÀà¸öÊı
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
		//+6=¶à°ü¹ÊÕÏÖÖÀà¸öÊı1Byte+IDµÄ4Byte+Ö¡Êı¾İ¸öÊı1Byte
		usfaultcodelen += 6+g_stuMultiFrame.ucControlCanDataNum*8;
		*p2++ = (uint8)(usfaultcodelen>>8);
		*p2++ = (uint8)(usfaultcodelen&0xFF);
    	usMsgBodyLen += usfaultcodelen+4;       //¼ÓÉÏTLµÄ4Byte³¤¶È
    	ucTlvNmr++;			
	}

//·¢¶¯»ú¹ÊÕÏÂë
	if(GetCanRcvState(CAN_CHANNEL1)==1)    //ÅĞ¶ÏÏÂ³µ½ÓÊÕ×´Ì¬
	{
	//0xA508-A509
        *buf++ = 0xA8;
    	*buf++ = 0x04;
    	pFC = buf;		//Êı¾İ³¤¶È
    	buf += 2;
    	pFC2= buf;		//DTC¸öÊı
    	buf += 1;
    	*buf++=eng_LampStatus; //¹ÊÕÏµÆ×´Ì¬
    	ucTemp = GetMcuFaultCode(buf);
    	buf += ucTemp*5;
    	*pFC2 = ucTemp;
    	*pFC++ = 0;
    	if(ucTemp)
    	{
    	    *pFC++ = ucTemp*5+1+1;       //Ôö¼Ó¹ÊÕÏµÆ×´Ì¬
    		usMsgBodyLen += ucTemp*5+5+1;
    	}
    	else
    	{
    	    *pFC++ = ucTemp*5+1;       //dtc==0 Ôò²»·¢ËÍ¹ÊÕÏµÆ×´Ì¬
    	    usMsgBodyLen += ucTemp*5+5;
    	}	
    	ucTlvNmr++;
	}
	
	p += 4;
	usTemp = usMsgBodyLen - 6;      //È¥µôº¯Êı¿ªÍ·µÄ7¸ö×Ö½Ú³¤¶È (Ğ­ÒéÖĞDTCºóÃæÁ½¸ö×Ö½Ú¶ÔÓ¦µÄ³¤¶È)
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
** º¯ÊıÃû³Æ: SYS_SendSleepNote
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍĞİÃßÌáÊ¾Ö¸Áî
** Êä    Èë:
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-07-14
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendSleepNote()
{
	uint16 usSendLen;
	
    f_usUpLoadCmdSn++;
	//aMsgSendData[0] = 		//ÏûÏ¢³¤¶È
	//aMsgSendData[1] =        	//
    memcpy(&aMsgSendData[2], g_stuSYSParamSet.aucDeviceID, 5);
    aMsgSendData[7] = PROTOCOL_VERSION;      
    aMsgSendData[8] = SUPPLY_CODE;
    aMsgSendData[9] = TERMINAL_TYPE;       
    aMsgSendData[10] = CUSTOMER_CODE;
    aMsgSendData[11] = (uint8)(f_usUpLoadCmdSn>>8);      //ÃüÁîÁ÷Ë®ºÅ
    aMsgSendData[12] = (uint8)(f_usUpLoadCmdSn&0xFF);
    aMsgSendData[13] = UP_CMD_ID_UPLOADSLEEPNOTE;			//ÃüÁîID
    usSendLen = BuildBasicRepPack(&aMsgSendData[14]);
	usSendLen += 14;
	aMsgSendData[0] = (usSendLen>>8) & 0xff;//ÏûÏ¢³¤¶È
	aMsgSendData[1] =  usSendLen     & 0xff;       	
	aMsgSendData[usSendLen] = Lrc(aMsgSendData, usSendLen);
	aMsgSendData[usSendLen+1] = 0x0d;
	aMsgSendData[usSendLen+2] = 0x0a;
	GSM_SendGprs(aMsgSendData, usSendLen+3, 0);    
}
#endif


/******************************************************************************
** º¯ÊıÃû³Æ: ConnectManage
** ¹¦ÄÜÃèÊö: ¹ÜÀíÏòÆ½Ì¨·¢ËÍÁ¬½ÓÖ¸Áî,´Ëº¯ÊıÒ»ÃëÔËĞĞÒ»´Î
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-9
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
** º¯ÊıÃû³Æ: AlarmDataRepManage()
** ¹¦ÄÜÃèÊö: ¹ÜÀíÏòÆ½Ì¨·¢ËÍ¹¤×÷Êı¾İ,´Ëº¯ÊıÒ»ÃëÔËĞĞÒ»´Î
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-13
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
** º¯ÊıÃû³Æ: AddToAlarmList
** ¹¦ÄÜÃèÊö: µ±±¨¾¯²úÉú»òÏûÊ§Ê±£¬Ìí¼Óµ½±¨¾¯ÁĞ±íÖĞÀ´
** Êä    Èë: type:±¨¾¯ÀàĞÍ,ÈçÏÂ:
            0x01--Éè±¸·Ç×ÔÖ÷ÒÆ¶¯±¨¾¯£¨¹¦ÄÜÌáĞÑ£©
			0x02--³öÎ§À¸£¨¹¦ÄÜÌáĞÑ£©
			0x03--Éè±¸³¬ËÙ±¨¾¯£¨¹¦ÄÜÌáĞÑ£©
			0x04--ÖÕ¶ËÓëÉè±¸µÄ×ÜÏßÍ¨ĞÅ¹ÊÕÏ(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x05--GPSÄ£¿é¹ÊÕÏ(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x06--GPSÌìÏß¹ÊÕÏ(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x07--ÖÕ¶ËÍâ²¿µçÔ´µÍµçÑ¹ (ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x08--ÖÕ¶ËÍâ²¿µçÔ´¶Ïµç(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x09--ÖÕ¶ËÄÚ²¿µç³ØµÍµçÑ¹(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x0A--SIM¿¨»»¿¨±¨¾¯£¨ÖÕ¶ËÓ²¼ş±¨¾¯£©
			0x0B--GPSĞÅºÅÇ¿¶ÈÈõ(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x0C--GPRSĞÅºÅÇ¿¶ÈÈõ(ÖÕ¶ËÓ²¼ş±¨¾¯)
			0x0D--GSM/GPRSÄ£¿é¹ÊÕÏ

			 flag:²úÉúÓëÏûÊ§±êÊ¶,0=²úÉú, 1=ÏûÊ§
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-9-23
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
** º¯ÊıÃû³Æ: SYS_SendAlarmData
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍ±¨¾¯Êı¾İ
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-9
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
		//Êı¾İÄÚÈİ³¤¶ÈMSBs
		//Êı¾İÄÚÈİ³¤¶ÈLSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+6];
	*buf++ = 0x30;
	*buf++ = 0x0d;
	p = buf;		//Êı¾İ³¤¶È
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
** º¯ÊıÃû³Æ: DTCRepManage()
** ¹¦ÄÜÃèÊö: ¹ÜÀíÏòÆ½Ì¨·¢ËÍDTCÊı¾İ,´Ëº¯ÊıÒ»ÃëÔËĞĞÒ»´Î
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-13
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
** º¯ÊıÃû³Æ: SYS_SendAlarmData_DTC
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍ±¨¾¯Êı¾İ
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-9
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
		//Êı¾İÄÚÈİ³¤¶ÈMSBs
		//Êı¾İÄÚÈİ³¤¶ÈLSB
	
	buf = &aMsgSendData[MSG_HEAD_LEN+7];
	*buf++ = 0x30;
	*buf++ = 0xEF;
	p = buf;		//Êı¾İ³¤¶È
	buf += 2;
	p2= buf;		//DTC¸öÊı
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
** º¯ÊıÃû³Æ: SYS_SendMcuRespData
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍMCU¶ÔÆ½Ì¨ÃüÁîÏìÓ¦µÄÊı¾İ
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-9
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
	if(m_stuMcuCmd.ucCmdCheckFlag==52)   //Ğ£ÑéÊ§°Ü
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
** º¯ÊıÃû³Æ: SYS_SendMcuRespData
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍMCU¶ÔÆ½Ì¨ÃüÁîÏìÓ¦µÄÊı¾İ
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-9
*******************************************************************************/
void McuRespManage()
{
	if(McuResp.ucRespondTimer)
		McuResp.ucRespondTimer--;
}

/******************************************************************************
** º¯ÊıÃû³Æ: SYS_NetDataSend
** ¹¦ÄÜÃèÊö: ÏòÆ½Ì¨·¢ËÍÉÏĞĞÊı¾İ(µÇÂ¼\ĞÄÌø\×´Ì¬Í¬²½\³µÁ¾¹ÊÕÏĞÅÏ¢\Ô¶³ÌÉı¼¶)
** 
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2019-0-18
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_SendNetData(void)
{	

    if(1==g_stuSystem.ucResetModem)       //ĞèÒªÖØĞÂÆô¶¯GSMÄ£¿é
    {

		PC_SendDebugData((uint8 *)("GSM RST2"), 8, DEBUG_GPRS);
		GSM_Variable_Init();
        GSM_SetModemWorkingState(8);                     //ÖØÆôGSMÄ£¿é
 
        g_stuSystem.ucResetModem = 0;
		Connect.ucSucceedFlag = 0;
		g_stuSystem.ucOnline = 0;
        GSM_LED_Off();                      //¹Ø±ÕGSMÖ¸Ê¾µÆ
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

	//Ô¶³ÌÉı¼¶
	if(FirmwareUpdate.ucStep > 0)
	{
		UpgradeManage();
		return;
	}

	//ÏòÆ½Ì¨·¢ËÍÁ¬½ÓÖ¸Áî
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

	//ÏòÆ½Ì¨·¢ËÍÖØĞÍÖÕ¶Ë»ù±¾×´Ì¬Í¬²½Êı¾İTCS
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

	//ÏòÆ½Ì¨·¢ËÍÖØĞÍ»ù±¾ĞÅÏ¢Êı¾İTCB
    if(g_stuZX_Timer.ucTCBSendFlag)
    {
        g_stuZX_Timer.ucTCBSendFlag = 0;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCB);
	}
	//ÏòÆ½Ì¨·¢ËÍÖØĞÍ¹¤¿ö²É¼¯Êı¾İTCW
	if(!g_stuZX_Timer.usTCWSendTimer)
	{
        g_stuZX_Timer.usTCWSendTimer = g_stuSYSParamSet.uiHeartbeatInterval;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCW);
	}
		
	//ÏòÆ½Ì¨·¢ËÍÖØĞÍ¹ÊÕÏÂëTCD
    if(!g_stuZX_Timer.usTCDSendTimer)
    {
		g_stuZX_Timer.usTCWSendTimer= ZXTCD_SEND_TIMER;
		SYS_SendZXTC_Data(ZX_SEND_FLAG_TCD);
    }
	
    //ÏòÆ½Ì¨·¢ËÍĞÄÌøÍ¨Ñ¶Ö¸Áî
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
** º¯ÊıÃû³Æ: SYS_LedDisplay
** ¹¦ÄÜÃèÊö: ¶ÔÉè±¸Ö¸Ê¾µÆµÄ¿ØÖÆºÍÏÔÊ¾,¸Ãº¯ÊıĞèÒªÃ¿ÃëÖÓÖ´ĞĞÒ»´Î
** GPSÖ¸Ê¾µÆ:
   GSMÖ¸Ê¾µÆ:
** Êä    Èë: ÎŞ
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-07-17
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
	//GPSÖ¸Ê¾µÆ¿ØÖÆ
	if(stuSleep.ucGpsSleepState)
    {
       	GpsLedOff();                     //ĞİÃßGPSÖ¸Ê¾µÆÃğ
    }
	else if(GPS_GetAnteState())  
	{
        GpsLedOn();                      //GPSÌìÏßÎ´½Ó(³¤ÁÁ)
	}
	else
	{
	    if(1==GPS_GetOrientState())
	    {
	        if(ucPositionFlag==TRUE)
	        {
	            ucPositionFlag = FALSE;
	            GpsLedOff();                    //µÆÃğ
	        }
	        else
	        {
	            ucPositionFlag = TRUE;
	            GpsLedOn();                    //µÆÁÁ
	        }          
	    }
		else
		{
			GpsLedOff();                        //GPSÕı³£ (³¤Ãğ)

		}
	}
	*/
    //ÉÏÏßÖ¸Ê¾µÆ¿ØÖÆ  
    if(stuSleep.ucGsmSleepState)
	{
		 GSM_LED_Off();    
	}
    else if(g_stuSystem.ucOnline)
    {
        if(ucOnline==TRUE)
        {
            ucOnline = FALSE;
            GSM_LED_On();                       //GSMµÆÁÁ
        }
        else
        {
            ucOnline = TRUE;
            GSM_LED_Off();                       //GSMµÆÁÁ
        }
    }  
    else                                      //ÉÏÏß¹ı³ÌÖĞ
    {
        if(GSM_GetAtStep())
        {
            GSM_LED_On();                       //GSMµÆÁÁ
        }
    }

    if(ucUserLed==TRUE)
    {
        ucUserLed = FALSE;
        USER_LED_On();                       //GSMµÆÁÁ
    }
    else
    {
        ucUserLed = TRUE;
        USER_LED_Off();                      //GSMµÆÁÁ
    }
}

/******************************************************************************
** º¯ÊıÃû³Æ: SYS_Reset
** ¹¦ÄÜÃèÊö: ÏµÍ³¸´Î»
** Êä    Èë: delay--ÑÓÊ±¸´Î»Ê±¼ä
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Reset(uint8 delay)
{
	g_stuSystem.ucRstTime = delay;
}

/******************************************************************************
** º¯ÊıÃû³Æ: SYS_Shutdown
** ¹¦ÄÜÃèÊö: ÏµÍ³¹Ø»ú
** Êä    Èë: delay--ÑÓÊ±¸´Î»Ê±¼ä
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
** ×÷    Õß: hhm
** ÈÕ    ÆÚ: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_Shutdown(uint8 delay)
{
	g_stuSystem.ucShutDownTime = delay;
}


/******************************************************************************
** º¯ÊıÃû³Æ: SYS_TimerCount
** ¹¦ÄÜÃèÊö: SystemÄ£¿é¼ÆÊ±º¯Êı,µ÷ÓÃ»ùÊıÎª10ms,Í¬Ê±°üº¬50ms¡¢100ms¡¢1s¹²4ÖÖ¼ÆÊı·½Ê½
** 
** Êä    Èë: 
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-06-22
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
    if(uc50ms)             //50ms»ùÊı
    {
        if(!(--uc50ms))
        {
            uc50ms = 5;
        }
    }

    if(uc100ms)            //100ms»ùÊı
    {
        if(!(--uc100ms))
        {
            uc100ms = 10;
        }
    }    
	*/
	if(us2s)          //»ùÊı2Ãë
    {
        if(!(--us2s))
        {
            us2s = 200;
			//DealUnNormData();
        }
    } 
    if(uc1s)               //1s»ùÊı
    {
        if(!(--uc1s))
        {
            uc1s = 100;

			
            Ext_Mcu_Timeout_Function();
            if(g_stuSystem.ucRstTime)           //Ê¹ÏµÍ³¸´Î»
            {
                if(!(--g_stuSystem.ucRstTime))
                {
                    GSM_Modem_RST();
                }
            }
			
            if(g_stuSystem.ucShutDownTime)  //¹Ø»ú
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

			//ĞÄÌø·¢ËÍ¼ÆÊ±
            if(HeartBeat.usNoUploadGprsTimer<MAX_NO_UPLOAD_GPRS_TIME)
            {
                HeartBeat.usNoUploadGprsTimer++; 
			}
			/*
			if(SysCounter.usDailyReportCounter!=0)
				SysCounter.usDailyReportCounter--;
			*/
            SYS_LedDisplay();                    //LED Ö¸Ê¾µÆÏÔÊ¾  
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
** º¯ÊıÃû³Æ: DealGpsMessage
** ¹¦ÄÜÃèÊö: 
** 
** Êä    Èë: PtrTxt:Ö¸Õë
             usPtrTxtLen:Õû°üÊı¾İµÄ³¤¶È,
             ucKind:ÏûÏ¢Êı¾İÀàĞÍ ,±¨¾¯»òÕß»Ö¸´
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÃüÁîÖ´ĞĞÍê³Éºó»á·µ»ØÏàÓ¦µÄ³¤¶È£¬³¤¶ÈÎªÎŞ·ûºÅ16Î»Êı¾İ£¬
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-07-31
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
** º¯ÊıÃû³Æ: SYS_CollectModule_ReceiveData_Execution
** ¹¦ÄÜÃèÊö: 
** 
** Êä    Èë: PtrTxt:Ö¸Õë
             usPtrTxtLen:Õû°üÊı¾İµÄ³¤¶È,
             ucKind:ÏûÏ¢Êı¾İÀàĞÍ ,±¨¾¯»òÕß»Ö¸´
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÃüÁîÖ´ĞĞÍê³Éºó»á·µ»ØÏàÓ¦µÄ³¤¶È£¬³¤¶ÈÎªÎŞ·ûºÅ16Î»Êı¾İ£¬
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-07-31
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
** º¯ÊıÃû³Æ: SYS_MCUModule_ReceiveData_Execution
** ¹¦ÄÜÃèÊö: 
** 
** Êä    Èë: PtrTxt:Ö¸Õë
             usPtrTxtLen:Õû°üÊı¾İµÄ³¤¶È,°üÀ¨MCUCMD_ID ³¤¶È+MCUCMDÄÚÈİ³¤¶È
             ucKind:ÏûÏ¢Êı¾İÀàĞÍ 1:canÍ¨ĞÅ±ä»¯;2:RS232Í¨ĞÅ±ä»¯;
             3:¶Ô·şÎñÆ÷·¢ËÍ¸øMCUµÄÃüÁîµÄ»Ø¸´;4:¹ÊÕÏÂë·¢Éú±ä»¯;5:MCUÊı¾İ±ä»¯
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÃüÁîÖ´ĞĞÍê³Éºó»á·µ»ØÏàÓ¦µÄ³¤¶È£¬³¤¶ÈÎªÎŞ·ûºÅ16Î»Êı¾İ£¬
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2011-07-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 SYS_MCUModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind)
{
    switch(ucKind)
	{
		case MCU_MSG_KIND_CAN1COMM_ERR:  //MCU×´Ì¬¸Ä±ä,´¦ÀíMCU¹ÊÕÏ´úÂë´¥·¢ĞÅÏ¢
	        AddToAlarmList(0x04, 0);
			SSData.usTimer = 0;
			break;
		case MCU_MSG_KIND_CAN1COMM_OK:   //MCU×´Ì¬¸Ä±ä,´¦ÀíMCU¹ÊÕÏ´úÂë´¥·¢ĞÅÏ¢
            SSData.usTimer = 1;
	        AddToAlarmList(0x04, 1);
	//		WorkDataRep.usTimer = 7;		//canÍ¨ĞÅ¿ªÊ¼´¥·¢¹¤×÷²ÎÊı
			break;	
		case MCU_MSG_KIND_CAN1_RCV_START:
	//		WorkDataRep.usTimer = 7;		//canÍ¨ĞÅ¿ªÊ¼´¥·¢¹¤×÷²ÎÊı
	        SSData.usTimer = 7;
			break;
//		case  MCU_MSG_KIND_CAN2_RCV_START:
			//AddToAlarmList(5, 1);
		//	WorkDataRep.usTimer = 0;		//canÍ¨ĞÅ¿ªÊ¼´¥·¢¹¤×÷²ÎÊı
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
** º¯ÊıÃû³Æ: SYS_PowerOff_Reset
** ¹¦ÄÜÃèÊö: ÖÕ¶Ë¶ÏµçÖØÆôº¯Êı£¬Ã¿1SÖ´ĞĞÒ»´Î
**           µ±ACC¹Ø±Õ³ÖĞøÊ±¼ä´ïµ½ĞİÃß»½ĞÑ¼ä¸ôÊ±³¤Ê±£¬Í¨ÖªAM1805¶ÔÕû»ú½øĞĞ¶Ïµç10ÃëÖÓ
             È»ºóÖØĞÂÉÏµç;Ö®ºóÈç¹ûACCÈÔ³ÖĞøÎªOFF×´Ì¬£¬ÔòÃ¿¼ä¸ô24Ğ¡Ê±Í¨ÖªÍ¨ÖªAM1805¶Ô
             Õû»ú½øĞĞ¶Ïµç10ÃëÖÓÈ»ºóÖØĞÂÉÏµç¡£
           
** Êä    Èë: 
** Êä    ³ö: ÎŞ
** ·µ    »Ø: ÎŞ
**
** ×÷    Õß: Lxf
** ÈÕ    ÆÚ: 2017-03-24
**-----------------------------------------------------------------------------
*******************************************************************************/
void SYS_PowerOff_Reset(void)
{
    static uint8 ucAccPre = 0;

	if((g_stuSystem.uiAccOffTimer>=g_stuSYSParamSet.usSleepWakeSlot*60||
		g_stuSystem.uiResetTimer>=86400)&&!GetAccState())
	{     
        PC_SendDebugData((uint8 *)("RTCPWROFF"), 9, DEBUG_ANYDATA);

		PSWControl(1);            //Í¨ÖªAM1805¶Ï¿ªÖÕ¶ËÍâ²¿ºÍµç³ØµçÔ´
        RTCSetSleepTime(10, 2);   //Í¨ÖªAM1805(RTCĞ¾Æ¬)¶Ïµç10ÃëÖÓ
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
//GPSÓÉ²»¶¨Î»µ½¶¨Î»´¥·¢Êı¾İÉÏ±¨
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
			SSData.usTimer = 3;              //´¥·¢Êı¾İÉÏ±¨
            PC_SendDebugData((uint8 *)("GPS_DingW"), 9, DEBUG_GPSMODULE);
		}
	}
	
    ucGPSStatePre = GPS_GetOrientState();
	ucACCPre = GetAccState();
}

/******************************************************************************
** º¯ÊıÃû³Æ: pthread_TaskSysTimer_Function
** ¹¦ÄÜÃèÊö: 10ms¼ÆÊ±Ïß³Ì
** Êä¡¡     Èë : ÎŞ
** Êä¡¡     ³ö : ÎŞ
** È«¾Ö±äÁ¿: 
** µ÷ÓÃÄ£¿é: 
** ×÷¡¡	 Õß : 
** ÈÕ¡¡	 ÆÚ : 2019Äê9ÔÂ7ÈÕ

**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_TaskSysTimer_Function(void *data)
{ 
    data = data;
	
	while(1)
	{
	    if(stu_ExtMCUUpgrade.ucUpgradeStep)
            usleep(One_MilliSecond*10);       //10ms ÑÓÊ±
        else
			sleep(1);                         //1s ÑİÊ¾
    //10ms ¼ÆÊ±Æ÷
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


//-----ÎÄ¼şSystemProtocol.c½áÊø---------------------------------------------

