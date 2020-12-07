/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmProtocolLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����:
���ļ�ΪGSMģ��Э����ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-06-09 by , �������ļ�
 *
 */
#include "config.h"
#include "GsmProtocolLayer.h"
#include "GsmIntegrateLayer.h"
#include "GsmInterfaceLayer.h"
//#include "GsmHardWareLayer.h"
#include "SystemProtocol.h"
//-----�ⲿ��������------------------------------------------------------------
extern STU_GsmOperate GsmOperate; 
extern STU_SmsSend f_stuSmsSend;
extern STU_GprsSendDataBuff GprsSendDataBuff[MAX_GPRS_LINK];
extern uint8 Public_Buf[];
extern stuFirmwareUpdate FirmwareUpdate;
extern stuSerAddr g_stuSerAddr;
//-----�ڲ���������------------------------------------------------------------
STUGsmState f_stuGsmState = {
	.ucModemSleepState = 0,
};
static uint8 f_ucAtSend;			//���͵�ATָ��
STU_GsmAtCmdState GsmAtCmdState; 
//static uint8 f_ucCipstartLinknum = 0xff;//��ǰ����AT+CIPSTART��ͨ����
//static uint8 f_ucCipcloseLinknum = 0xff;//��ǰ����AT+CIPCOSE��ͨ����
//static uint8 f_ucCipsendLinknum = 0xff;//��ǰ����AT+CIPSEND��ͨ����
//static STU_SmsRcv f_stuSmsRcv;

//static STUSYSMsgBus f_stuRecvGprsMsg={SRCDEVICE_ID_GSM, GSM_MSG_GPRS_RECV, 0, 0, 0, 0, 0}; 
//static STUSYSMsgBus f_stuRecvFtpMsg={SRCDEVICE_ID_GSM, GSM_MSG_FTP_RECV, 0, 0, 0, 0, 0};

//-----�ڲ���������------------------------------------------------------------
static uint8  CalcSmsPduLen(uint8 ucSrclen,uint8 ucSimLen);
static void   PackSmsPDU(uint8 *pucData, uint16 *pusLen);

/*
STU_SerAddress main_ser = {
	"61.147.119.201",
	14,	
	9003,
	0
};

STU_SerAddress customer_ser = {
	"61.147.119.201",
	14,	
	9002,
	1
};
uint8 GetMainSerAddr(PSTU_SerAddress ser)
{
	ser = &main_ser;
}
uint8 GetCustomSerAddr(PSTU_SerAddress ser)
{
	ser = &customer_ser;
}
*/


void SIMCardStateJudge(void)
{
    static uint8 ucCount = SIMCARD_Satate_Timer;    //3min
    
    if(0==GSM_GetSimState())
    {
        ucCount = SIMCARD_Satate_Timer;
		f_stuGsmState.ucSIMCardError = 0;
	}
	else
	{
        if(ucCount)
        {
            if(!(--ucCount))
            {
            	PC_SendDebugData((uint8 *)("SIMCardErr"), 10, DEBUG_AT);

                f_stuGsmState.ucSIMCardError = 1;
			}
		}
	}
}

//��ȡSIM��״̬ ��������ʱ�ж� 0-����,1-���
uint8 Get_SIMCardState(void)
{
    return f_stuGsmState.ucSIMCardError;
}

//-----�ڲ���������------------------------------------------------------------
uint8  GetAtSend()
{
	return f_ucAtSend;
}

void SetAtSend(uint8 at)
{
	f_ucAtSend = at;
}



/******************************************************************************
** ��������: ResetGsm
** ��������: ����GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-08-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void ResetGsm()
{
	uint8 i;
	
 //  	GsmOperate.GsmOperate = 1;
   	for(i=0; i<MAX_GPRS_LINK; i++)
		f_stuGsmState.ucLinkState[i] = GPRS_LINK_STATE_CLOSED;
//	f_stuGsmState.ucWirelessNet = 0;
	f_stuGsmState.ucCGATT = 0;
	f_stuGsmState.ucCSQ = 0;
	f_stuGsmState.ucCPIN = 0;
	f_stuGsmState.ucSmsRdy = 0;
	f_stuGsmState.ucGsmRegState = 0;
}

#if 0
/******************************************************************************
** ��������: WakeGsm
** ��������: GSMģ�黽�Ѻ���
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-21
**-----------------------------------------------------------------------------
*******************************************************************************/
void WakeGsm()
{
	PC_SendDebugData((uint8 *)("GSM RST3"), 8, DEBUG_ANYDATA);
    ResetGsm();
}

/******************************************************************************
** ��������: SleepGsm
** ��������: GSMģ�����ߺ���
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-09
**-----------------------------------------------------------------------------
*******************************************************************************/
void SleepGsm()
{
    uint8 i,j;

	f_stuGsmState.ucModemSleepState =MODEM_SLEEP_STATE_SLEEPING;

	GsmAtCmdState.ucIPShut = 0;
	for(i=0; i<2; i++)
	{
		SetAtSend(ATSEND_SLEEP_IPSHUT);
		WriteGsmUartData((uint8*)"at+cipshut\r", 11);//�ر�urc
		for(j=0; j<65; j++)
		{
			OSTimeDly(OS_TICKS_PER_SEC);//�ȴ�20ms
			if(1==GsmAtCmdState.ucIPShut)
				break;
		}
		if(1==GsmAtCmdState.ucIPShut)
				break;
	}
	SetAtSend(ATSEND_IDLE);
	
	GsmAtCmdState.ucCREG_SET= 0;
	for(i=0; i<3; i++)
	{
		SetAtSend(ATSEND_CONNECT_CREG_SET);
		WriteGsmUartData((uint8*)"at+creg=0\r", 10);
		OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
		if(1==GsmAtCmdState.ucCREG_SET)
			break;
	}
	SetAtSend(ATSEND_IDLE);

	
	GsmAtCmdState.ucCSCLK = 0;
	for(i=0; i<3; i++)
	{
		SetAtSend(ATSEND_SLEEP_CSCLK);
		WriteGsmUartData((uint8*)"AT+CSCLK=1\r", 11); 
		OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
		if(1==GsmAtCmdState.ucCSCLK)
			break;
	}
	SetAtSend(ATSEND_IDLE);
   			
	ModemDTRHigh();
	
	f_stuGsmState.ucModemSleepState =MODEM_SLEEP_STATE_SLEPT;
}



/******************************************************************************
** ��������: SendSms
** ��������: ���Ͷ���Ϣ������
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-24
**-----------------------------------------------------------------------------
*******************************************************************************/
void SendSms()
{
    uint8 aucTemp[300]={0};
	uint8 i;
	uint16 ucPackLen = 0;
	uint8* pucData = NULL;
	
	
	if(f_stuSmsSend.flag != 1)
		return;

    if(!strncmp((char*)f_stuSmsSend.aucDesNumber, "10", 2))  //10��ͷ���벻��86
	{
		ucPackLen=CalcSmsPduLen(f_stuSmsSend.usDataLen, f_stuSmsSend.ucNumberLen);
	}
	else
	{
		ucPackLen=CalcSmsPduLen(f_stuSmsSend.usDataLen, 2+f_stuSmsSend.ucNumberLen);
	}
	

	sprintf((char*)aucTemp, "AT+CMGS=%d\r", ucPackLen);
	GsmAtCmdState.ucCMGS_Cmd = 0;
	SetAtSend(ATSEND_CMGS_CMD);
	WriteGsmUartData(aucTemp, strlen((char*)aucTemp));
	for(i=0; i<3; i++)
	{
		OSTimeDly(1);
		if(1==GsmAtCmdState.ucCMGS_Cmd)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(1==GsmAtCmdState.ucCMGS_Cmd)
	{
		memcpy(aucTemp, f_stuSmsSend.aucSmsSendBuff, f_stuSmsSend.usDataLen);
		pucData = aucTemp;
		PackSmsPDU(pucData, &ucPackLen);//������ת��PDU��ʽ,���ݳ��ȷ�ǰ���ֽ�
		GsmAtCmdState.ucCMGS_Data = 0;
		SetAtSend(ATSEND_CMGS_DATA);
		WriteGsmUartData(aucTemp, ucPackLen);
		aucTemp[0] = 0x1a;
		WriteGsmUartData(aucTemp, 1);
	}
	SetAtSend(ATSEND_IDLE);
}




/******************************************************************************
** ��������: DialPhoneNumber
** ��������: ����绰
**
** ��    ��: number=�绰����, len=�绰���볤��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
void DialPhoneNumber(uint8 *number, uint8 len)
{
/*	uint8 buff[19];

	if(len > 16)
		return;
	memcpy(buff, "ATD", 3);
	memcpy(&buff[3], number, len);
	buff[len+3] = ';';
	SendGsmUartData(buff, len+4);*/
}



/******************************************************************************
** ��������: SendGprs
** ��������: ����GPRS���ݴ�����
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void SendGprs()
{
	static uint8 failcnt[3] = {0,0,0};//����ʧ�ܼ���,����6�ξ�����
    uint8 aucTemp[20];
	uint8 i,j,k;
	
    for(i=0; i<MAX_GPRS_LINK; i++)
	{
		if((1==GprsSendDataBuff[i].flag))
		{
			for(j=0; j<2; j++)//���ɹ�����ط�����
			{
				sprintf((char*)aucTemp, "AT+CIPSEND=%d,%d\r",GprsSendDataBuff[i].ucLinkNum,GprsSendDataBuff[i].usLen);
				GsmAtCmdState.ucCIPSEND_Cmd = 0;
				f_ucCipsendLinknum = i;
				SetAtSend(ATSEND_CIPSEND_CMD);
				WriteGsmUartData(aucTemp, strlen((char*)aucTemp));
				for(k=0; k<200; k++)
				{
					OSTimeDly(OS_TICKS_PER_SEC/200);
					if(1==GsmAtCmdState.ucCIPSEND_Cmd)
						break;
				}
				f_ucCipsendLinknum = 0xff;
				SetAtSend(ATSEND_IDLE);
				if(1==GsmAtCmdState.ucCIPSEND_Cmd)
				{
					f_ucCipsendLinknum = i;
					GsmAtCmdState.ucCIPSEND_Data = 0;
					SetAtSend(ATSEND_CIPSEND_DATA);
					
					GprsSendDataBuff[i].buff[GprsSendDataBuff[i].usLen] = 0x1A;
					WriteGsmUartData(GprsSendDataBuff[i].buff, GprsSendDataBuff[i].usLen+1);
					PC_SendDebugData(GprsSendDataBuff[i].buff, GprsSendDataBuff[i].usLen+1, DEBUG_GPRS);
					for(k=0; k<250; k++)
					{
						OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
						if(1==GsmAtCmdState.ucCIPSEND_Data)
							break;
					}
					f_ucCipsendLinknum = i;
					SetAtSend(ATSEND_IDLE);
					if(1==GsmAtCmdState.ucCIPSEND_Data)
					{
						GprsSendDataBuff[i].flag = 0;
						break;
					}
					else
					{
						failcnt[i]++;
					}
				}
				else
				{
					failcnt[i]++;
				}
			}
			GprsSendDataBuff[i].flag = 0;
			f_stuGsmState.ucLinkState[i] = GPRS_LINK_STATE_READY;
		}
	}
	#if 0
	if((failcnt[0]>6) || (failcnt[1]>6) ||(failcnt[2]>6))
	{
		failcnt[0] = 0;
		failcnt[1] = 0;
		failcnt[2] = 0;
		GsmOperate.GsmOperate = 1;
	}
	#endif
}




/******************************************************************************
** ��������: OpenLink
** ��������: ��һ��GRPS����
**			 �ݲ�֧��IP��DN�Ķ�̬�л����ӳ���
** ��    ��: LinkNum=���Ӻ�
             Addr:ָ���������ַ��ָ��
             AddrLen:��������ַ����
             Port:�������˿�
             ProcolType:Э������, 1:TCP, 2:UDP, ����:��ֹ
** ��    ��: ��
** ��    ��: 0=�ɹ�,1=ʧ��
**
** ��    ��: hhm
** ��    ��: 2014-09-22
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 OpenLink(uint8 LinkNum, uint8 *Addr, uint8 AddrLen, uint32 Port, uint8 ProcolType)
{
	uint8 i,j;
	uint8 buff[70]={0};
	uint8 arr[16] = {0};
	
	if(LinkNum >= MAX_GPRS_LINK)
		return 1;
	#if 0
	if(ProcolType==1)
		sprintf(buff, "at+cipstart=%d,\"tcp\",\"%.*s\",\"%d\"\r", LinkNum,AddrLen,Addr,Port);
	else if(ProcolType==2)
		sprintf(buff, "at+cipstart=%d,\"udp\",\"%.*s\",\"%d\"\r", LinkNum,AddrLen,Addr,Port);
	else
		return 1;
	#endif
	
	sprintf(arr, "%d.%d.%d.%d", *Addr, *(Addr+1), *(Addr+2), *(Addr+3));
	if(ProcolType==1)
	{
		sprintf(buff, "at+cipstart=%d,\"tcp\",\"%s\",\"%d\"\r", LinkNum,arr,Port);
	}
	else if(ProcolType==2)
	{
		sprintf(buff, "at+cipstart=%d,\"udp\",\"%s\",\"%d\"\r", LinkNum,arr,Port);
	}
	
	//for(i=0; i<2; i++)//�ط�û��
	//{
		f_ucCipstartLinknum = LinkNum;
		SetAtSend(ATSEND_CIPSTART);
		GsmAtCmdState.ucCIPStart = 0;
		WriteGsmUartData(buff, strlen(buff));	
		for(j=0; j<80; j++)
		{
			OSTimeDly(OS_TICKS_PER_SEC/4);
			if(1==GsmAtCmdState.ucCIPStart)
				break;
		}
		if(1==GsmAtCmdState.ucCIPStart)
		{
			f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_READY;
			//break;
		}
		else
		{
			//ResetGsm();
		}
	//}
	f_ucCipstartLinknum = 0xff;
	SetAtSend(ATSEND_IDLE);
	if(f_stuGsmState.ucLinkState[LinkNum]==GPRS_LINK_STATE_READY)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

void CloseLink(uint8 LinkNum)
{
	uint8 buff[16];
	uint8 i,j;
	
	sprintf(buff, "AT+CIPCLOSE=%d,1\r", LinkNum);
	for(i=0; i<3; i++)
	{
		f_ucCipcloseLinknum = LinkNum;
		SetAtSend(ATSEND_CIPCLOSE);
		GsmAtCmdState.ucCIPClose = 0;
		WriteGsmUartData(buff, 16);	
		for(j=0; j<4; j++)
		{
			OSTimeDly(OS_TICKS_PER_SEC/4);
			if(1==GsmAtCmdState.ucCIPClose)
				break;
		}
		if(1==GsmAtCmdState.ucCIPClose)
		{
			f_stuGsmState.ucLinkState[LinkNum] = 0;
				break;
		}
	}
	f_ucCipcloseLinknum = 0xff;
	SetAtSend(ATSEND_IDLE);
}






/******************************************************************************
** ��������: SaveSimSeq
** ��������: ����SIM�����кű��溯��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2012-07-04
**-----------------------------------------------------------------------------
*******************************************************************************/
void SaveSimSeq(void)
{

}

/******************************************************************************
** ��������: ReadSimSeq
** ��������: ����SIM�����кŶ�ȡ����
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2012-07-04
**-----------------------------------------------------------------------------
*******************************************************************************/
void ReadSimSeq(void)
{

}


/******************************************************************************
** ��������: InitSimNum
** ��������: FLASH����SIM���к�
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-10-28
**-----------------------------------------------------------------------------
*******************************************************************************/
void InitSimSeq(void)
{
	ReadSimSeq();
}




void ModemPwrOn()
{
	uint8 i;
	uint8 j = 0;
	
	ModemPwrHigh();					//�ϵ�
 	OSTimeDly(OS_TICKS_PER_SEC*3);	//��ʱ3s   
	for(i=0; i<5; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*2);	//2s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem on"), 8, DEBUG_ANYDATA);

		for(j=0;j<6;j++)
		{
            OSTimeDly(OS_TICKS_PER_SEC);
    		if(0==Modem_Status())
    			return;			
		}
		
 	}
}

void ModemPwrOff()
{
	uint8 i;
	uint8 j = 0;
	
	for(i=0; i<2; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*2);	//1s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem off"), 9, DEBUG_ANYDATA);

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
    		if(1==Modem_Status())
    			return;			
		}
	}
	ModemPwrLow();					//�ص�
	OSTimeDly(OS_TICKS_PER_SEC*20); //
}

//��������,ǰ��:GSMģ�����ϵ�
void ModemOpen()
{
	uint8 i;
	uint8 j = 0;
	
	//if(0==CheckModemPwr())
	//{
		ModemPwrHigh();					//�ϵ�
 		OSTimeDly(OS_TICKS_PER_SEC*3);	//��ʱ3s      
	//}
//	for(i=0; i<5; i++)
	for(i=0; i<1; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*2);	//2s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem on"), 8, DEBUG_ANYDATA);

		for(j=0;j<5;j++)
		{
            OSTimeDly(OS_TICKS_PER_SEC);
    		if(0==Modem_Status())
    			return;			
		}
	}
}

void ModemClose()
{
	uint8 i;
	uint8 j=0;

	for(i=0; i<2; i++)
	{
		ModemOpenHigh();         		//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		ModemOpenLow();					//����
		OSTimeDly(OS_TICKS_PER_SEC*2);	//2s
		ModemOpenHigh();				//����
		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
		PC_SendDebugData((uint8 *)("modem Close"), 9, DEBUG_ANYDATA);

		for(j=0;j<32;j++)
		{
    		OSTimeDly(OS_TICKS_PER_SEC);	//��ʱ1s
    		if(1==Modem_Status())
    			return;			
		}		
	}
}
#endif
/******************************************************************************
** ��������: IsSmsReady
** ��������: �鿴���ŷ����Ƿ��Ѿ�׼��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ׼�����˷���1,δ׼���÷���0
**
** ��    ��: 
** ��    ��: 2011-06-24
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL IsSmsReady(void)
{
	if(f_stuGsmState.ucSmsRdy && !f_stuSmsSend.flag)
		return 1;
    else 
		return 0;
}

/******************************************************************************
** ��������: CalcSmsPduLen
** ��������: ��ԭʼ���ݳ���ת��ΪPDU��ʽ����
**
** ��    ��: ucSrclen,��Ϣ��ucSimLen
** ��    ��: ��
** ��    ��: PDU��ʽ������
**
** ��    ��: 
** ��    ��: 2011-07-13
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 CalcSmsPduLen(uint8 ucSrclen,uint8 ucSimLen)
{
    if(ucSimLen%2)
        return ucSimLen/2+9+ucSrclen;
    else
        return ucSimLen/2+8+ucSrclen;
}

/******************************************************************************
** ��������: PackPdu
** ��������: ��ԭʼ���ݴ����PDU��ʽ��
**
** ��    ��: pucData,�ӹ�ǰ����;usLen,�ӹ�ǰ���ݳ���
** ��    ��: pucData,�ӹ�������;usLen,�ӹ������ݳ���
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2012-06-10
**-----------------------------------------------------------------------------
*******************************************************************************/
void PackSmsPDU(uint8 *pucData, uint16 *pusLen)
{
    uint8 ucSimLen = 0;
	uint8 ucSrcLen=*pusLen;
    uint8 aucTemp[300];
	uint8 *pucPoint=aucTemp;
	
    memcpy(pucPoint, "001100", 6);                          //ʹ��Ĭ�����ĺ���
	pucPoint += 6;	

	if(1==g_ucNumFlag)      //�̻�����
	{
	    ucSimLen = 11;
	}
	else                   //���ĺ���
	{
		ucSimLen=f_stuSmsSend.ucNumberLen;
	}

	if(1==g_ucNumFlag)
	{
    	Uint8ToHex2(ucSimLen+2, pucPoint);                      //Ŀ���ַ���ָ���
		pucPoint += 2;
	}
    else
	{
		if(strncmp(f_stuSmsSend.aucDesNumber, "10", 2))
		{
	    	Uint8ToHex2(ucSimLen+2, pucPoint);                      //Ŀ���ַ���ָ���
			pucPoint += 2;
		}
		else
		{
	    	Uint8ToHex2(ucSimLen, pucPoint);                      //Ŀ���ַ���ָ���
			pucPoint += 2;
		}
	}
	
	Uint8ToHex2(0x91, pucPoint);                            //Ŀ���ַ��ʽ
	pucPoint += 2;

    if(1==g_ucNumFlag)
	{
		Uint8ToHex2(0x86, pucPoint);                            //���ʺŶ�
		pucPoint += 2;
	}
	else
	{
		if(strncmp(f_stuSmsSend.aucDesNumber, "10", 2))
		{
			Uint8ToHex2(0x86, pucPoint);                            //���ʺŶ�
			pucPoint += 2;
		}
	}

    //ע��:86һ���������һ��ͨ��BCDB����������,�������
   
	memcpy(pucPoint, f_stuSmsSend.aucDesNumber, f_stuSmsSend.ucNumberLen);     //Ŀ�����
	
	if(ucSimLen%2)                                         //��ַ����Ϊ��������F
	{
	    pucPoint[ucSimLen++]='F';
	}	

    if(1==g_ucNumFlag)
	{
		ConvertBCDB(pucPoint-2, ucSimLen+2);                    //ת��ΪPDU��ʽ��ַ��ʽ
		pucPoint += ucSimLen;
	}
	else
	{
		if(strncmp(f_stuSmsSend.aucDesNumber, "10", 2))
		{
			ConvertBCDB(pucPoint-2, ucSimLen+2);                    //ת��ΪPDU��ʽ��ַ��ʽ
			pucPoint += ucSimLen;
		}
		else
		{
			ConvertBCDB(pucPoint, ucSimLen);                    //ת��ΪPDU��ʽ��ַ��ʽ
			pucPoint += ucSimLen;
		}
	}
	
	memcpy(pucPoint, "000400", 6);                          //�㵽��,UCS2����,��Ч��5����
	pucPoint += 6;	
	Uint8ToHex2(ucSrcLen, pucPoint);                        //�û����ݳ���
	pucPoint += 2;
	
	DataToHexbuff(pucPoint, (uint8*)pucData, ucSrcLen);     //�û�����
	pucPoint += 2*ucSrcLen;

    *pusLen = pucPoint-aucTemp;                             //ת����PDU��ʽ����

	memcpy(&pucData[2], aucTemp, *pusLen);

	pucData[0] = (uint8)*pusLen;

}


/******************************************************************************
** ��������: UnPackCREG
** ��������: ����CPINָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE;ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2020-06-29
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
BOOL UnPackCREG(uint8 *pucSrc, uint16 usSrcLen)
{
    uint8 *pucPtr = NULL;
	uint8 *pucPtr2 = NULL;
	uint16 usLen = 0;
	
    if(NULL==pucSrc || usSrcLen<10)
	{
	    return FALSE;
	}

/*
	pucPtr = SearchString((uint8 *)pucSrc, usSrcLen, "+CREG:", 6);
	if(NULL == pucPtr)
		return FALSE;
	
	pucPtr += 6;

	pucPtr2=GetStringMiddle(pucPtr, usSrcLen-6, '\"', 1, '\"', 2, &usLen);
	if(NULL != pucPtr2)
	{
	//    ASCIITOHex((uint8*)pucPtr2, (uint8*)&f_stuGsmState.usCREGLAC, usLen);
	}
	else
	{
		return FALSE;
	}
	if(NULL != (pucPtr2=GetStringMiddle((uint8*)pucPtr, usSrcLen-6, '\"', 3, '\"', 4, &usLen)))
	{
	//    ASCIITOHex((uint8*)pucPtr2, (uint8*)&f_stuGsmState.usCREGCI, usLen);
	}
	else
	{
		return FALSE;
	}
*/
	if(ATSEND_CONNECT_CREG==GetAtSend())
	{
		GsmAtCmdState.ucCREG = 1;
	}
	return TRUE;
}
#endif

BOOL UnPackCREG(uint8 *pucSrc, uint16 usSrcLen)
{
    uint8 *pucPtr = NULL;
	uint8 *pucPtr2 = NULL;
	uint16 usLen = 0;
	
    if(NULL==pucSrc || usSrcLen<10)
	{
	    return FALSE;
	}

//at+creg? +CREG: 2,1,"51D0","1F73"

//at+creg? +CREG: 2,1,FFFE,502C91F
	pucPtr = SearchString((uint8 *)pucSrc, usSrcLen, "+CREG:", 6);
	if(NULL == pucPtr)
		return FALSE;
	pucPtr += 6;//ָ��ո�

	pucPtr2=GetStringMiddle(pucPtr, usSrcLen-6, ',', 2, ',', 3, &usLen);
	if(NULL != pucPtr2)//ָ��F
	{
	 //   ASCIITOHex((uint8*)pucPtr2, (uint8*)&f_stuGsmState.usCREGLAC, usLen);
	}
	else
	{
		return FALSE;
	}
	if(NULL != (pucPtr2=GetStringMiddle((uint8*)pucPtr, usSrcLen-6, ',', 3, '\r', 1, &usLen)))
	{
	//    ASCIITOHex((uint8*)pucPtr2, (uint8*)&f_stuGsmState.usCREGCI, usLen);
	}
	else
	{
		return FALSE;
	}

	if(ATSEND_CONNECT_CREG==GetAtSend())
	{
		GsmAtCmdState.ucCREG = 1;
	}
	return TRUE;
}

/******************************************************************************
** ��������: UnPackCPIN
** ��������: ����CPINָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE; ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2011-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCPIN(const uint8 *pucSrc, uint16 usSrcLen)
{
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}
	
    if(SearchString((uint8 *)pucSrc, usSrcLen, "+CPIN: READY", 12))
 		return TRUE;
	else
		return FALSE;
}

/******************************************************************************
** ��������: UnPackCIMI
** ��������: ����CIMIָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE,ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2011-10-09
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCIMI(const uint8 *pucSrc, uint16 usSrcLen)
{
    uint8  *pucPtr1 = NULL;
	uint8  *pucPtr2 = NULL;
	uint8  ucLength = 0;
	
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}

	pucPtr1 = SearchString((uint8*)pucSrc, usSrcLen, "460", 3);  //���Ҵ���:�й�
	if(NULL != pucPtr1)
	{
	    pucPtr2 = SearchString(pucPtr1, usSrcLen-2, "\015\012", 2);
		if(NULL == pucPtr2)
			return FALSE;
		
		ucLength = pucPtr2 - pucPtr1;
		if(ucLength >20)
			return FALSE;
		strncpy((char*)f_stuGsmState.aucCIMI, (char*)pucPtr1, ucLength);
		return TRUE;
	}
	return FALSE;
}

/******************************************************************************
** ��������: UnPackCCID
** ��������: ����CCIDָ��ؽ��
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE,ʧ�ܷ���FALSE
** ��    ��: hhm
** ��    ��: 2016-7-26
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCCID(const uint8 *pucSrc, uint16 usSrcLen)
{
    uint8  *pucPtr1 = NULL;
	uint8  *pucPtr2 = NULL;
	uint8  ucLength = 0;
	
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}
	
	pucPtr1 = SearchString((uint8*)pucSrc, usSrcLen, "89", 2);    //���Ҵ���:�й�
//	pucPtr1 = SearchString((uint8*)pucSrc, usSrcLen, "8986", 4);  //���Ҵ���:�й�
	if(NULL != pucPtr1)
	{
	    pucPtr2 = SearchString(pucPtr1, usSrcLen-2, "\r\n", 2);
		ucLength = pucPtr2 - pucPtr1;
		if(ucLength >20)
			return FALSE;
		strncpy((char*)f_stuGsmState.aucCCID, (char*)pucPtr1, ucLength);
		return TRUE;
	}
	return FALSE;
}


/******************************************************************************
** ��������: UnPackCSQ
** ��������: ����CSQָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE,ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2011-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCSQ(const uint8 *pucSrc, uint16 usSrcLen)
{
    uint8  *pucPtr1 = NULL;
	uint8  *pucPtr2 = NULL;
	uint8  ucLength = 0;

    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}

    pucPtr1 = SearchString((uint8 *)pucSrc, usSrcLen, "CSQ: ", 5);
	if(pucPtr1 != NULL)
	{
		pucPtr2 = SearchString((uint8 *)pucSrc, usSrcLen, ",", 1) ;
		if(!pucPtr2)
			return FALSE;
		ucLength = pucPtr2 - pucPtr1 - 5;
		if(ucLength<3 && ucLength>0)
		{
		    f_stuGsmState.ucCSQ = DecToUint8(pucPtr1+5, ucLength);
			//printf("CSQ= %d\n" ,f_stuGsmState.ucCSQ);
			if(f_stuGsmState.ucCSQ > 31)
			{
				f_stuGsmState.ucCSQ = 0;
				return FALSE;
			}
		}
		else
		{
		    f_stuGsmState.ucCSQ = 0;
			return FALSE;
		}
	}
	else
	{
	     f_stuGsmState.ucCSQ = 0;
		 return FALSE;
	}
	return TRUE;
	
}

/******************************************************************************
** ��������: UnPackCGATT
** ��������: ����CPINָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE;ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2011-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCGATT(const uint8 *pucSrc, uint16 usSrcLen)
{
	uint8  *pucPtr = NULL;

    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}
	
	pucPtr = SearchString((uint8 *)pucSrc, usSrcLen, "CGATT: ", 7);
	if(!pucPtr)
		return FALSE;
	
	if(*(pucPtr+7) == '1')
	{
 	    f_stuGsmState.ucCGATT = 1;
 	}
	else
	{
	    f_stuGsmState.ucCGATT = 0;
		return FALSE;
	}
	return TRUE;
}

/******************************************************************************
** ��������: UnPackCPSI
** ��������: ����CPSIָ��ؽ��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE;ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2020-11-30
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackCPSI(const uint8 *pucSrc, uint16 usSrcLen)
{
	uint8  *pucPtr = NULL;
    uint8  ucsublen = 0;
	uint8  *ptemp = NULL;
	uint8  ucHexFlag1 = 0;
	uint8  ucHexFlag2 = 0;
	uint16 usTemp = 0;
	
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}
	
	pucPtr = SearchString((uint8 *)pucSrc, usSrcLen, "CPSI:", 5);
	if(!pucPtr)
		return FALSE;

    usSrcLen -= 6;
	ptemp = GetStringMiddle(pucPtr,usSrcLen, ',',3,',',4, &ucsublen);   //��λ״̬
    if(ptemp&&ucsublen)
    {
        ucHexFlag1 = *ptemp++;
		ucHexFlag2 = *ptemp++;
        if(ucHexFlag1=='0'&&ucHexFlag2=='x')
        {
            usTemp = (uint16)(hex2uint(ptemp)<<8);
			ptemp += 2;
        	f_stuGsmState.uiLAC = usTemp + hex2uint(ptemp);       	//��ȡLAC       
		}
	}
	else
		return FALSE;

	pucPtr = GetStringMiddle(ptemp,usSrcLen, ',',4,',',5, &ucsublen);   //��λ״̬
    if(ptemp&&ucsublen)
    {
        f_stuGsmState.uiCEll_ID = atoi((char*)pucPtr);       
	}
	else 
		return FALSE;

	return TRUE;
}



#if 0
/******************************************************************************
** ��������: UnPackGprs
** ��������: ��ȡGPRS���ݰ���������
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: 
** ��    ��: �ɹ�����1; ʧ�ܷ���1
**
** ��    ��: 
** ��    ��: 2011-06-21
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackGprs(const uint8 *pucSrc, uint16 usSrcLen)
{
    uint8  *pucPtr1 = NULL;
	uint8  *pucPtr2 = NULL;
	uint16  usLength = 0;
	uint8 ucLengBit;//���ݳ��ȵ�λ��
	uint8 ucLink = 0;
	uint8 i=0;
	uint8 flag = 0;
	
	
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}

    pucPtr1 = SearchString((uint8 *)pucSrc, usSrcLen, "+RECEIVE,", 9);
	if(pucPtr1 != 0)
	{
		pucPtr1 += 9;
		ucLink = *pucPtr1-'0';
		if(ucLink>=MAX_GPRS_LINK)
			return FALSE;
		pucPtr1 += 2;
		pucPtr2 = pucPtr1;
		for(i=0; i<4; i++)
		{
			if(*(++pucPtr1) == ':')
			{
				flag = 1;
				break;
			}
		}
		if(flag==1)
		{
			ucLengBit = pucPtr1 - pucPtr2;
			if(ucLengBit<5 && ucLengBit>0)
			{
			   usLength = DecToUint16(pucPtr2, ucLengBit);
				if(usLength <= usSrcLen-11-ucLengBit-1)
				{
					f_stuRecvGprsMsg.usSize = usLength;
					f_stuRecvGprsMsg.pMsgPacket = pucPtr1+3;//
					f_stuRecvGprsMsg.ucResv = ucLink;
					SYS_PutDataQ((void *)&f_stuRecvGprsMsg);
					PC_SendDebugData((uint8*)(pucPtr1+3), usLength, DEBUG_GPRS);
					return TRUE;
				}
			}
		}
	}
	return FALSE;
	
}
BOOL UnPackCIFSR(const uint8 *pucSrc, uint16 usSrcLen)
{
	uint8 ucDots = 0;
	uint8 *ptr;
	uint16 i;

	ptr = (uint8 *)pucSrc;
	for(i=0; i<usSrcLen; i++)
	{
		if(*ptr++ == '.')
			ucDots++;
	}
	if(3==ucDots)
		return TRUE;
	else
		return FALSE;
}
/******************************************************************************
** ��������: UnPackIpClose
** ��������: ����GSMģ���Զ��ϱ������ӶϿ�״̬
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen,Դ���ݳ���            
** ��    ��: 
** ��    ��: �ɹ�����1; ʧ�ܷ���1
**
** ��    ��: hhm
** ��    ��: 2011-09-4
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackIpClose(const uint8 *pucSrc, uint16 usSrcLen)
{
	uint8 ret = 0;
	
	if(SearchString((uint8 *)pucSrc, usSrcLen, "0, CLOSED", 9))
	{
		f_stuGsmState.ucLinkState[0]= 0;
		ret |= BIT(0);
	}
	if(SearchString((uint8 *)pucSrc, usSrcLen, "1, CLOSED", 9))
	{
		f_stuGsmState.ucLinkState[1]= 0;
		ret |= BIT(1);
	}

	if(ret)
		return TRUE;
	else
		return FALSE;
}
/******************************************************************************
** ��������: UnpackSms
** ��������: ��ȡSMS���ݰ�������
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen, Դ���ݳ���
** ��    ��: pusDesLen,Ŀ�����ݳ���
** ��    ��: �ɹ�����TRUE;ʧ�ܷ���FALSE
**
** ��    ��: 
** ��    ��: 2011-06-21
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL UnPackSms(const uint8 *pucSrc, uint16 usSrcLen)
{
    uint8  ucMasFlag = 0;
    uint16 ucLen = 0;
	//uint8  ucPackLen=0;                         //��ȥ���ĺ���İ���
	uint8  ucPackHeadLen=0;                     //���ĺ��볤��
	uint8  ucIndex=0;                           //����ֵ
	uint8  ucType=0;                            //���ݱ��뷽ʽ
	uint8  *pucPoint1 = (uint8*)pucSrc;
    uint8  *pucPoint2 = 0;
	uint8  *pucPoint3 = (uint8*)pucSrc;
	//uint8  RoyalSms = 0;
//	uint8  ucaComNum[]="13973100157";         	//
//	uint8  aucGimisSimNum[]="13809046648";		//
	uint8  aucSrcNumber[15];						//���յ��Ķ��ŵ�Դ����
//	uint8  ucSrcNumberLen = 0;
	uint8  aucTemp[12];
	
    if(NULL==pucSrc || 0==usSrcLen)
	{
	    return FALSE;
	}

	pucPoint1 = GetStringMiddle((uint8*)pucSrc, usSrcLen, ' ', 1, ',', 1, &ucLen);
	if(NULL==pucPoint1)
	{
	    return FALSE;
	}
	ucIndex = Dec3ToUint8(pucPoint1, ucLen);    //��ȡ����������

	pucPoint1 = SearchString((uint8*)pucSrc+3, usSrcLen, "\015\012", 2);//3Ϊ�����
	if(NULL==pucPoint1)
	{
	    return FALSE;
	}
	ucPackHeadLen = Dec2ToUint8(pucPoint1+2);   //ȡPDU��ͷ����(���ĺ���)
	pucPoint2 = pucPoint1+2;                    //��¼��ͷ��ʼ��ַ

	pucPoint1 = GetStringMiddle((uint8*)pucSrc+3, usSrcLen, ',', 3, '\015', 1, &ucLen);
	if(NULL==pucPoint1)
	{
	    return FALSE;
	}
	//ucPackLen = Dec3ToUint8(pucPoint1, ucLen);  //ȡPDU����

    pucPoint1 = SearchString((uint8*)pucPoint2, usSrcLen, "\r\n", 2);
	if(NULL==pucPoint1)                         //�ж�������ʱ,��������û�н�����
	{
	    pucPoint1 = (uint8*)pucSrc+usSrcLen;
	}

	ucLen = pucPoint1-pucPoint2;
	
    pucPoint2 += 2*(ucPackHeadLen+1);           //��ָ��λ������PDU���ݴ�
    pucPoint2 += 2;                             //PDU type

	ucLen = Hex2ToUint8(pucPoint2);             //Դ��ַ����
	pucPoint2 += 2;                             //��ַ����
	pucPoint2 += 2;                             //��ַ����

    ucLen = ucLen%2 ? ucLen+1 : ucLen;          //���Ȳ���
	ConvertBCDB(pucPoint2, ucLen);              //����ת���������洢��ʽ

    if(strncmp((char*)pucPoint2, "86", 2))//û��"86"
    {
	    ucMasFlag = 1;              //mas
	    memcpy(aucSrcNumber, pucPoint2, 15);
    }
	else
	{
	    ucMasFlag = 0;              //��������
		memcpy(aucSrcNumber, pucPoint2+2, 11);
	}
	/*
    if(1==ucMasFlag)
	{
		if(15==SYS_GetParam()->ucSmsCenterLen)
		{
		    if(!strncmp(aucSrcNumber, (char*)SYS_GetParam()->ucSmsCenter, 15))
	    	{
	 		   RoyalSms = 1;
		   	}
		}
	}
	else
	{
	    if(!strncmp(aucSrcNumber, aucGimisSimNum, strlen(aucGimisSimNum)))
		{
			RoyalSms = 1;
			//PC_SendDebugData((uint8 *)("SMSA"), 4, DEBUG_ANYDATA);
    	}
		else if(!strncmp(aucSrcNumber, ucaComNum, strlen(ucaComNum)))
		{
			RoyalSms = 1;
			//PC_SendDebugData((uint8 *)("SMSB"), 4, DEBUG_ANYDATA);
		}
	 	else if(!strncmp(aucSrcNumber, (char*)SYS_GetParam()->ucSmsCenter,SYS_GetParam()->ucSmsCenterLen))
 		{
 			RoyalSms = 1;
			//PC_SendDebugData((uint8 *)("SMSC"), 4, DEBUG_ANYDATA);
 		}
 	}
	*/
	

	
    ///-------------------���ն�����־
    /*if(g_stuSmsRep.ucSimLen>6)
    {
        if(g_stuSystem.ucWorkingMode>0)
        {
            SaveLog(0x08, 2, 0);     //���Ż���
        }
        uiPara1 = (uint32)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-6]<<24)+(uint32)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-5]<<16)
            +(uint32)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-4]<<8)+(uint32)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-3]);
        uiPara2 = (uint16)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-2]<<8)+(uint16)(g_stuSmsRep.aucSimNum[g_stuSmsRep.ucSimLen-2]);
        SaveLog(0x0116, uiPara1, uiPara2);   // ��־ test
    }*/
	//-------------------------
    pucPoint2 += ucLen;

	pucPoint2 += 2;                            //PIDЭ���־
	
	ucType = Hex2ToUint8(pucPoint2);           //DCS���ݱ��뷽ʽ
	pucPoint2 += 2;                            //DCS���ݱ��뷽ʽ

	pucPoint2 += 14;                           //ʱ���

	if(0x01==ucType)
	{
	   
	    ucLen=7*Hex2ToUint8(pucPoint2);
		if(ucLen%8)
		{
		    ucLen = ucLen/8+1;
		}
		else
		{
		    ucLen = ucLen/8;
		}
		//�ݲ������������
		//HexToDatabuff(&aucRecvSmsBuff[2], pucPoint2+2, ucLen*2);
	}
	else if(0x04==ucType)
	{
	    ucLen = Hex2ToUint8(pucPoint2);
		HexToDatabuff(pucPoint3, pucPoint2+2, ucLen*2);

		f_stuSmsRcv.ucNumberLen = (ucMasFlag==1)?15:11;
		memcpy(f_stuSmsRcv.aucSrcNumber,  aucSrcNumber, f_stuSmsRcv.ucNumberLen);

		f_stuSmsRcv.flag = 1;
		//f_stuRecvSmsMsg.pMsgPacket = pucPoint3;
		//f_stuRecvSmsMsg.usSize =  ucLen;
		//SYS_PutDataQ((void *)&f_stuRecvSmsMsg);
	}
	else
	{
	   
	}

	sprintf(aucTemp, "AT+CMGD=%d\r", ucIndex);
	WriteGsmUartData((uint8*)aucTemp, strlen(aucTemp));
	return TRUE;
}

BOOL UnPackCMGL(uint8 *pucData, uint16 usLen)
{
	if(SearchString(pucData, usLen, "+CMGL:", 6))
	{
		return UnPackSms(pucData, usLen);
	}
	else
	{
		return FALSE;
	}
}

BOOL UnPackCMT(uint8 *pucData, uint16 usLen)
{
	if(SearchString(pucData, usLen, "+CMT:", 5))
	{
		WriteGsmUartData((uint8 *)"AT+CMGL=4\r", 10);
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL UnPackPDPDEACT(uint8 *pucData, uint16 usLen)
{
	uint8 i;
	
	if(SearchString(pucData, usLen, "+PDP: DEACT", 11))
	{
		if(1==f_stuGsmState.ucWirelessNet)
		{
			GsmOperate.GsmOperate = 1;
			PC_SendDebugData((uint8 *)("GSM RST5"), 8, DEBUG_ANYDATA);
		}
		for(i=0; i<MAX_GPRS_LINK; i++)
			f_stuGsmState.ucLinkState[i] = GPRS_LINK_STATE_CLOSED;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL UnPackIpsend(uint8 *pucData, uint16 usLen)
{
	uint8 arr[12];
	
	if(f_ucCipsendLinknum >= MAX_GPRS_LINK)
		return FALSE;
	
	sprintf(arr,"%d, SEND OK",f_ucCipsendLinknum);
	if(SearchString((uint8*)pucData, usLen, arr, 10))
	{
		GsmAtCmdState.ucCIPSEND_Data = 1;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

//��ȡ���յ��Ķ��ŵĺ���
void Gsm_GetSmsSrcNumber(uint8 *number, uint8 *len)
{
	number = f_stuSmsRcv.aucSrcNumber;
	len = &f_stuSmsRcv.ucNumberLen;
}
//Ӧ�ó����ڴ�����϶��ź���ô˺���,�ͷŶ��Ż���, ͬʱ����GSM���������Ŵ�����Ͽ��Զ�ȡ��һ������
void Gsm_ReleaseSmsBuff()
{
	f_stuSmsRcv.flag = 0;
}
/******************************************************************************
** ��������: UnPackCMGS
** ��������: �������Ͷ��ŷ��صĽ��,���ڷ��Ͷ��ŷ��ؽ����ʱ��Ƚϳ�,��������ʽ�ȴ�
             ��������г�ʱ��ʱ,+CMGS:��Ϊ�Զ���������䴦��
**
** ��    ��: pucSrc,Դ����ָ��; usSrcLen, Դ���ݳ���
** ��    ��: ��
** ��    ��: �ɹ�����TRUE;ʧ�ܷ���FALSE
**
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/

BOOL UnPackCMGS(uint8 *pucData, uint16 usLen)
{
	if(SearchString((uint8*)pucData, usLen, "+CMGS:", 6))
	{
		if(SearchString((uint8*)pucData, usLen, "OK", 2))
		{
			if(2==f_stuSmsSend.flag)
			{
				f_stuSmsSend.flag = 0;
			}
			else
			{
				f_stuSmsSend.flag = 0;
			}
		}
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/******************************************************************************
** ��������: SendSmsTimeoutCount
** ��������: �������˶��ź�ú�����ʼ����,1sִ��һ��,ֱ����ʱ,�÷���ʧ��
** ��    ��:��
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
void SendSmsTimeoutCount()
{
	static uint8 sec10 = 10;//10s��ʱ
	
	if(2==f_stuSmsSend.flag)
	{
		if(0==(--sec10))
		{
			f_stuSmsSend.flag = 0;
		}
	}
	else if(sec10 != 10)
	{
		sec10 = 10;
	}	
		
}
#endif
#if 0
/******************************************************************************
** ��������: QueryState
** ��������: GSMģ��״̬��ѯ����,�ú���1������һ��
**
** ��    ��: ��
** ��    ��: �� 
** ��    ��:��
** ��    ��: hhm
** ��    ��: 2014-09-3
**-----------------------------------------------------------------------------
*******************************************************************************/


void QueryGsmState()
{
	static uint8 state = 0;
	static uint8 sim_err_times = 0;
	static uint8 cgatt_err_times = 0;
	
	switch (state)
	{
		case 0:
			GsmAtCmdState.ucCPIN = 0;
	   		SetAtSend(ATSEND_CONNECT_CPIN);
			WriteGsmUartData((uint8 *)"at+cpin?\r", 9);	
			OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
			if(1==GsmAtCmdState.ucCPIN)
			{
				sim_err_times = 0;
				f_stuGsmState.ucCPIN = 1;
			}
			else
			{
				if(sim_err_times++ > 3)
				{
					sim_err_times = 0;
					f_stuGsmState.ucCPIN = 0;
					PC_SendDebugData((uint8 *)("GSM RST4"), 8, DEBUG_ANYDATA);
					ResetGsm();
				}
			}
			SetAtSend(ATSEND_IDLE);
			state = 1;
			break;
		case 1:
			GsmAtCmdState.ucCSQ = 0;
			SetAtSend(ATSEND_CONNECT_CSQ);
	   		WriteGsmUartData((uint8 *)"at+csq\r", 7);	
			OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
			SetAtSend(ATSEND_IDLE);
			state = 2;
			break;
		case 2:
			GsmAtCmdState.ucCGATT = 0;
			SetAtSend(ATSEND_CONNECT_CGATT);
	   		WriteGsmUartData((uint8 *)"at+cgatt?\r", 10);	
			OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
			if(1==GsmAtCmdState.ucCGATT)
			{
				cgatt_err_times = 0;
				f_stuGsmState.ucCGATT = 1;
			}
			else
			{
				if(cgatt_err_times++ > 3)
				{
					cgatt_err_times = 0;
					f_stuGsmState.ucCGATT = 0;
				}
			}
			SetAtSend(ATSEND_IDLE);
			state = 3;
			break;
		case 3:
			if((0==f_stuSmsRcv.flag) && (0==f_stuSmsSend.flag))
			{
				GsmAtCmdState.ucCMGL = 0;
				SetAtSend(ATSEND_READSMS_CMGL);
		   		WriteGsmUartData((uint8 *)"AT+CMGL=4\r", 10);
				OSTimeDly(OS_TICKS_PER_SEC/50);//�ȴ�20ms
				SetAtSend(ATSEND_IDLE);
			}
			state = 0;
			break;
		default:
			break;
	}
}
#endif
#if 0
void TTSInit()
{
	m_stuTTS.BusyFlag  = 0;
	m_stuTTS.PlayFlag  = 0;
	m_stuTTS.PlayIndex = 0xff;
}

void PlayTTS()
{
	uint8 i=0;
	uint8 index = 0xff;
	uint8 buff[1024];
	uint16 len = 0;
	uint8 *ptr;
	uint8 ucTemp;

	if(m_stuTTS.BusyFlag == 1)
		return;
	for(i=0; i<32; i++)
	{
		if(m_stuTTS.PlayFlag & BIT(i))
		{
			m_stuTTS.PlayFlag &= ~BIT(i);
			index = i;
			break;
		}
	}
	m_stuTTS.PlayIndex = index;
	if(0xff ==index)
		return;

	ptr = buff;
	memcpy(ptr, "AT+CTTS=2,\"", 11);
	ptr += 11;
	switch(index)
	{
		case TTS_PLAY_TEXT_SPEEDOVER:
			SetVirtualKeyValue(KEY_VIR_VAL_SOD);
			memcpy(ptr, "���ѳ���", 8);
			ptr += 8;
			len = 8;
			break;
		case TTS_PLAY_TEXT_GNSSANTANNAFAULT:
			memcpy(ptr, "��λ���߹���", 12);
			ptr += 12;
			len = 12;
			break;
		case TTS_PLAY_TEXT_ICCARDIN:
			memcpy(ptr, "��ʻԱ����", 10);
			ptr += 10;
			len = 10;
			break;
		case TTS_PLAY_TEXT_ICCARDOUT:
			memcpy(ptr, "��ʻԱ�ǳ�", 10);
			ptr += 10;
			len = 10;
			break;
		case TTS_PLAY_TEXT_TIMEOUTDRIVE:
			SetVirtualKeyValue(KEY_VIR_VAL_TOD);
			memcpy(ptr, "��ʱ��ʻ", 8);
			ptr += 8;
			len = 8;
			break;
		case TTS_PLAY_TEXT_ICCARDINSERTNOTE:
			SetVirtualKeyValue(KEY_VIR_VAL_ICOUT);
			memcpy(ptr, "�����IC��", 10);
			ptr += 10;
			len = 10;
			break;	
		case TTS_PLAY_TEXT_SPEEDUNNORMAL:
			SetVirtualKeyValue(KEY_VIR_VAL_SAB);
			memcpy(ptr, "�ٶ��쳣", 8);
			ptr += 8;
			len = 8;
			break;		
		/*	
		case TTS_PLAY_TEXT_ICCARDERR:
			memcpy(ptr, "IC������", 8);
			ptr += 8;
			len = 8;
			break;
		*/
		case TTS_PLAY_TEXT_PLATFORM_TEXTDOWNLOARD:
			memcpy(ptr, TextInfor.TextInformation, TextInfor.TextLen);
			ptr += TextInfor.TextLen;
			len = TextInfor.TextLen;
			break;
		default:
			break;
	}
	memcpy(ptr,"\"\r", 2);
	GsmAtCmdState.ucCTTS= 0;
	SetAtSend(ATSEND_CTTS_PLAY);
	WriteGsmUartData(buff, len+11+2);
	for(i=0; i<100; i++)
	{
		OSTimeDly(OS_TICKS_PER_SEC/100);
		if(1==GsmAtCmdState.ucCTTS)
			break;
	}
	
	SetAtSend(ATSEND_IDLE);
	m_stuTTS.PlayOverTimer = 10;
	m_stuTTS.BusyFlag = 1;
}
/*
BOOL UnPackCTTS(const uint8 *pucSrc, uint16 usSrcLen)
{
	if(SearchString((uint8*)pucSrc, usSrcLen, "+CTTS:0", 7))
	{
		if(1==m_stuTTS.BusyFlag)
		{
			m_stuTTS.BusyFlag = 0;
			m_stuTTS.PlayFlag &= BIT(m_stuTTS.PlayIndex);
		}
		return TRUE;
	}
	return FALSE;
}
*/

void TTSTimerCount()
{
	if(1==m_stuTTS.BusyFlag)
	{
		if(m_stuTTS.PlayOverTimer)
		{
			m_stuTTS.PlayOverTimer--;
		}
		else
		{
			m_stuTTS.BusyFlag = 0;
			m_stuTTS.PlayIndex = 0xff;
		}
	}
}

//��ȡ��ǰ����������������
//����:TRUE =�ɹ���ȡ��������, ��֤��index��
//     FALSE=û�л�ȡ����Ч������
BOOL GetTTSPlayIndex(uint8 * index)
{
	if(0xff == m_stuTTS.PlayIndex)
	{
		return FALSE;
	}
	else
	{
		*index = m_stuTTS.PlayIndex;
		return TRUE;
	}
}

/******************************************************************************
** ��������: OpenFtpConnect
** ��������: ����һ��ftp����
** ��    ��: ��
** ��    ��: ��
** ��    ��: 0=�����ɹ�,1=����ʧ��
** ��    ��: hhm
** ��    ��: 2016-7-20
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 OpenFtpConnect()
{
	uint8 i;

	GsmAtCmdState.ucSAPBR_SET= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_SAPBR_SET);
   		WriteGsmUartData((uint8 *)"AT+SAPBR=3,2,\"Contype\",\"GPRS\"\r", 30);	
		OSTimeDly(OS_TICKS_PER_SEC);//�ȴ�20ms
		if(1==GsmAtCmdState.ucSAPBR_SET)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucSAPBR_SET)
		return 1;

	GsmAtCmdState.ucSAPBR_OPEN= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_SAPBR_OPEN);
   		WriteGsmUartData((uint8 *)"AT+SAPBR =1,2\r", 14);	
		OSTimeDly(OS_TICKS_PER_SEC);//�ȴ�20ms
		if(1==GsmAtCmdState.ucSAPBR_OPEN)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucSAPBR_OPEN)
		return 1;
	
	GsmAtCmdState.ucFTPCID= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPCID);
   		WriteGsmUartData((uint8 *)"AT+FTPCID=2\r", 12);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPCID)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPCID)
		return 1;

	GsmAtCmdState.ucFTPTYPE= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPTYPE);
   		WriteGsmUartData((uint8 *)"AT+FTPTYPE=\"I\"\r", 15);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPTYPE)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPTYPE)
		return 1;

	GsmAtCmdState.ucFTPSERV = 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPSERV);
		memcpy(Public_Buf, "AT+FTPSERV=\"", 12);
		memcpy(&Public_Buf[12],FirmwareUpdate.aucSerAddr, FirmwareUpdate.ucSerAddrLen);
		memcpy(&Public_Buf[12+FirmwareUpdate.ucSerAddrLen], "\"\r", 2);
   		WriteGsmUartData(Public_Buf, 14+FirmwareUpdate.ucSerAddrLen);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPSERV)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPSERV)
		return 1;

	GsmAtCmdState.ucFTPUN = 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPUN);
		memcpy(Public_Buf, "AT+FTPUN=\"", 10);
		memcpy(&Public_Buf[10],FirmwareUpdate.aucUserName, FirmwareUpdate.ucUserNameLen);
		memcpy(&Public_Buf[10+FirmwareUpdate.ucUserNameLen], "\"\r", 2);
   		WriteGsmUartData(Public_Buf, 12+FirmwareUpdate.ucUserNameLen);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPUN)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPUN)
		return 1;

	GsmAtCmdState.ucFTPPW= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPPW);
		memcpy(Public_Buf, "AT+FTPPW=\"", 10);
		memcpy(&Public_Buf[10],FirmwareUpdate.aucPassword, FirmwareUpdate.ucPasswordLen);
		memcpy(&Public_Buf[10+FirmwareUpdate.ucPasswordLen], "\"\r", 2);
   		WriteGsmUartData(Public_Buf, 12+FirmwareUpdate.ucPasswordLen);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPPW)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPPW)
		return 1;

	GsmAtCmdState.ucFTPGETNAME= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPGETNAME);
		memcpy(Public_Buf, "AT+FTPGETNAME=\"", 15);
		memcpy(&Public_Buf[15],FirmwareUpdate.aucFileName, FirmwareUpdate.ucFileNameLen);
		memcpy(&Public_Buf[15+FirmwareUpdate.ucFileNameLen], "\"\r", 2);
   		WriteGsmUartData(Public_Buf, 17+FirmwareUpdate.ucFileNameLen);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPGETNAME)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPGETNAME)
		return 1;

	GsmAtCmdState.ucFTPGETPATH= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPGETPATH);
		memcpy(Public_Buf, "AT+FTPGETPATH=\"", 15);
		memcpy(&Public_Buf[15],FirmwareUpdate.aucPath, FirmwareUpdate.ucPathLen);
		memcpy(&Public_Buf[15+FirmwareUpdate.ucPathLen], "\"\r", 2);
   		WriteGsmUartData(Public_Buf, 17+FirmwareUpdate.ucPathLen);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPGETPATH)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPGETPATH)
		return 1;

	GsmAtCmdState.ucFTPGET_OpenSession= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPGET_OPENSESSION);
   		WriteGsmUartData((uint8 *)"AT+FTPGET=1\r", 12);	
		OSTimeDly(OS_TICKS_PER_SEC*3);
		if(1==GsmAtCmdState.ucFTPGET_OpenSession)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPGET_OpenSession)
		return 1;
	else
		GsmOperate.FtpOperate = 2;

	return 0;
}

/******************************************************************************
** ��������: CloseFtpConnect
** ��������: �ر�һ��ftp����
** ��    ��: ��
** ��    ��: ��
** ��    ��: 0=�����ɹ�,1=����ʧ��
** ��    ��: hhm
** ��    ��: 2016-7-20
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 CloseFtpConnect()
{
	uint8 i;
	
	GsmAtCmdState.ucFTPQUIT= 0;
	for(i=0; i<5; i++)
	{
		SetAtSend(ATSEND_FTPQUIT);
   		WriteGsmUartData((uint8 *)"AT+FTPQUIT\r", 11);	
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPQUIT)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPQUIT)
		return 1;	
	else
		return 0;
}
/******************************************************************************
** ��������: GetFtpData
** ��������: �ڽ���һ��ftp���Ӻ���������
** ��    ��: ��
** ��    ��: ��
** ��    ��: 0=�����ɹ�,1=����ʧ��
** ��    ��: hhm
** ��    ��: 2016-7-20
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetFtpData(uint16 len)
{
	uint8 i;
	uint8 buf[20] = {0};
	
	GsmAtCmdState.ucFTPGET= 0;
	
	SetAtSend(ATSEND_FTPGET);
	sprintf(buf,"AT+FTPGET=2,%d\r",len);
	//WriteGsmUartData((uint8 *)"AT+FTPGET=2,1024\r", 17);
	WriteGsmUartData(buf, strlen(buf));
	for(i=0; i<5; i++)
	{
		OSTimeDly(OS_TICKS_PER_SEC/5);//�ȴ�20ms
		if(1==GsmAtCmdState.ucFTPGET)
			break;
	}
	SetAtSend(ATSEND_IDLE);
	if(0==GsmAtCmdState.ucFTPGET)
		return 1;
	else
		return 0;
}

//���ftp���ݻ���
void GSM_ClrFtpDataBuf()
{
	m_usFtpDataLen = 0;
}
#endif




