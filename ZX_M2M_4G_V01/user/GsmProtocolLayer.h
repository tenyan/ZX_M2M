/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmProtocolLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-06-09 by , �������ļ�
 *
 */

#ifndef GSMPROTOCOLLAYER_H_20110609_8AA4FBBE_5B1A_4627_A73C_BD105AFB6F8E
#define GSMPROTOCOLLAYER_H_20110609_8AA4FBBE_5B1A_4627_A73C_BD105AFB6F8E

//-----��������----------------------------------------------------------------
#define  RECV_SMS_BUFF_LEN 				512
#define  LENOFSIMSEQ					18

//*********************���͵�ATָ��********************************************
#define ATSEND_IDLE						0
#define ATSEND_CONNECT_AT				1
#define ATSEND_CONNECT_ATE0				2
#define ATSEND_CONNECT_CMEE				3
#define ATSEND_CONNECT_CPIN				4
#define ATSEND_CONNECT_CIMI				5
#define ATSEND_CONNECT_CPSI				6
#define ATSEND_CONNECT_CSQ				7
#define ATSEND_CONNECT_CREG_SET			8
#define ATSEND_CONNECT_CREG				9
#define ATSEND_CONNECT_CGATT			10
#define ATSEND_CONNECT_APN		    	11      //���� APN
#define ATSEND_CONNECT_NETACT_SET		12
#define ATSEND_CONNECT_NETACT			13
#define ATSEND_CONNECT_CFUN0			14
#define ATSEND_CONNECT_CFUN1			15
#define ATSEND_CONNECT_CGPADDR          16
#define ATSEND_CONNECT_WiFIOFF          17
#define ATSEND_CONNECT_WiFION           18
#define ATSEND_CONNECT_CRESET           0xFF    //��λģ��

#define ATSEND_READSMS_CMGL 			15
#define ATSEND_SLEEP_CSCLK				16
#define ATSEND_SLEEP_IPSHUT				17
#define ATSEND_CMGS_CMD					18
#define ATSEND_CMGS_DATA				19

#define ATSEND_CIPSTART         		20//���Ӵ�����
#define ATSEND_CIPCLOSE        		    21//���ӹر�����
#define ATSEND_CIPSEND_CMD				22//���ӷ�������--����
#define ATSEND_CIPSEND_DATA				23//���ӷ�������--����

#define ATSEND_CTTS_PLAY				24//TTS����

#define ATSEND_FTPCID					25
#define ATSEND_FTPSERV					26
#define ATSEND_FTPUN					27
#define ATSEND_FTPPW					28
#define ATSEND_FTPGETNAME				29
#define ATSEND_FTPGETPATH				30
#define ATSEND_FTPGET					31
#define ATSEND_FTPTYPE					32
#define ATSEND_FTPGET_OPENSESSION		33
#define ATSEND_FTPQUIT					34
#define ATSEND_CONNECT_CCID				35

#define ATSEND_SAPBR_SET				36
#define ATSEND_SAPBR_OPEN				37

//-----�ṹ����----------------------------------------------------------------

typedef struct _STU_GSM_ATCMD_STATE
{
	uint8 ucAt;
	uint8 ucATE0;
	uint8 ucCMEE;
	uint8 ucCPIN;
	uint8 ucCIMI;
	uint8 ucCCID;
	uint8 ucCNMI;
	uint8 ucCSQ;
	uint8 ucCGATT;
	uint8 ucCREG_SET;
	uint8 ucCREG;
	uint8 ucCIPMUX;
	uint8 ucCSTT;
	uint8 ucCIICR;
	uint8 ucCIFSR;
	uint8 ucCIPStart;
	uint8 ucCIPClose;
	uint8 ucCMGL;	
	uint8 ucCSCLK;
	uint8 ucIPShut;
	uint8 ucCIPSEND_Cmd;
	uint8 ucCIPSEND_Data;
	uint8 ucCMGS_Cmd;
	uint8 ucCMGS_Data;
	uint8 ucCTTS;
	uint8 ucFTPCID;
	uint8 ucFTPTYPE;
	uint8 ucFTPSERV;
	uint8 ucFTPUN;
	uint8 ucFTPPW;
	uint8 ucFTPGETNAME;
	uint8 ucFTPGETPATH;
	uint8 ucFTPGET_OpenSession;
	uint8 ucFTPGET;
	uint8 ucFTPQUIT;
	uint8 ucSAPBR_SET;
	uint8 ucSAPBR_OPEN;
}STU_GsmAtCmdState;


//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
uint8 DealGsmRcvData(uint8 *pucData, uint16 usLen);
void  ModemPwrOn(void);
void  ModemPwrOff(void);
void  ModemOpen(void);

void  OpenGsm(void);
void  CloseGsm(void);

void  WakeGsm(void);
void  SleepGsm(void);
void  ResetGsm(void);

uint8 OpenLink(uint8 LinkNum, uint8 *Addr, uint8 AddrLen, uint32 Port, uint8 ProcolType);
void  CloseLink(uint8 LinkNum);
BOOL  IsSmsReady(void);
void  DialPhoneNumber(uint8 *number, uint8 len);
void  AnserPhoneNumber(void);
BOOL  IsSmsReady(void);
uint8  GetAtSend(void);
void  SendSmsTimeoutCount(void);
void TTSTimerCount(void);
void PlayTTS(void);

#endif


