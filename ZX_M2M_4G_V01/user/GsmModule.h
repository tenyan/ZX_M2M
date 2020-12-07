/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmModule.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ�����ӿ�ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-06-09 by  �������ļ�
 *
 */

#ifndef GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138
#define GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138

#include "GsmHardWareLayer.h"
//-----��������----------------------------------------------------------------
#define GSM_RECV_BUFF_MAX_SIZE        1500
#define GSM_SEND_BUFF_MAX_SIZE        1500
#define GSM_AT_RECV_BUFF_MAX_SIZE     1000


#define TASK_GSM_RECV_STK_SIZE        400  //GSMģ���������ջ��С
#define TASK_GSM_RECV_ID              4    //GSMģ���������ID
#define TASK_GSM_RECV_PRIO            4    //GSMģ������������ȼ�

#define TASK_GSM_SERVICE_STK_SIZE     400  //GSMģ��������ջ��С
#define TASK_GSM_SERVICE_ID           5    //GSMģ��������ID
#define TASK_GSM_SERVICE_PRIO         5    //GSMģ�����������ȼ�

#define GPRS_SEND_BUFF_LEN			  1536   //�����ݰ���С1.5k

#define SMS_SEND_BUFF_LEN			  256	//����Ķ��ŵĳ���
#define MAX_GPRS_LINK				  1	   //���GRPS��������,����SIM800,��ֵ���ܴ���6	

#define GPRS_LINK_BUBIAO			  0    //��������
#define GPRS_LINK_BACKUP			  1    //���걸������
#define GPRS_LINK_CUSTOMER			  2	   //���ƿͻ�����
#define GPRS_LINK_ALL			      0xff //��������

#define GSM_MSG_GPRS_RECV             1    //�յ�GPRS��
#define GSM_MSG_RING_RECV             2    //�յ�RING
#define GSM_MSG_SMS_RECV              3    //�յ�SMS
#define GSM_MSG_SMS_SEND_OK           4    //����SMS�ɹ�
#define GSM_MSG_SMS_SEND_ERR          5    //����SMSʧ��
#define GSM_MSG_ONLINE                6    //GSM���߱�ʶ,֪ͨ�ⲿ�ɷ�GPRS��
#define GSM_MSG_FTP_RECV              7    //�յ�ftp���ݰ�


#define MODEM_STATE_CLOSED				0
#define MODEM_STATE_CLOSING				1
#define MODEM_STATE_OPEN				2
#define MODEM_STATE_OPENING				3

#define MODEM_SLEEP_STATE_WAKE			0
#define MODEM_SLEEP_STATE_SLEPT			1
#define MODEM_SLEEP_STATE_SLEEPING		2
#define MODEM_SLEEP_STATE_SHUTDOWN		3

#define GPRS_LINK_STATE_CLOSED			0
#define GPRS_LINK_STATE_READY			1
#define GPRS_LINK_STATE_BUSY			2

/********�������������Ŷ���***************/
#define TTS_PLAY_TEXT_SPEEDOVER					0	//��������
#define TTS_PLAY_TEXT_GNSSANTANNAFAULT			1	//��λ���߹���
#define TTS_PLAY_TEXT_ICCARDIN					2	//IC������
#define TTS_PLAY_TEXT_ICCARDOUT			        3	//IC���γ�
#define TTS_PLAY_TEXT_ICCARDERR			        4	//IC������
#define TTS_PLAY_TEXT_TIMEOUTDRIVE				5	//��ʱ��ʻ
#define TTS_PLAY_TEXT_ICCARDINSERTNOTE			6	//���Ѽ�ʻԱ����IC��
#define TTS_PLAY_TEXT_SPEEDUNNORMAL				7	//�ٶ��쳣
#define TTS_PLAY_TEXT_PLATFORM_TEXTDOWNLOARD    8	//ƽ̨�·����ı�����
/******************************************/
#define SIMCARD_Satate_Timer                 180    //�ж�SIM����ʱʱ��
#define One_MilliSecond                      1000   //1ms

//-----�ṹ����----------------------------------------------------------------

typedef struct _STU_GSMSTATE_
{
	uint8 ucModemState;		 //GSMģ��״̬:0=����,1=����,2=����socket,3=�����ɹ�,
	                            //4=�رղ���,5=�ر�socket,6=������,7=����
	uint8 ucQMIWwanEnable;   //1=֪ͨģ���������粦��,0=�ر�ģ�鲦��
	uint8 ucQMIWwanState;    //ģ�鲦�Ž��:1=�ɹ�,0=ʧ��
	uint8 ucModemSleepState; //GSMģ������״̬:0=����, 1=������,2=��������
    uint8  ucCSQ;            //�ź�ǿ��
	uint8  ucCPIN;           //SIM����ǰ״̬ 1=��⵽SIM��,0=δ��⵽SIM��
	uint8 ucSimErr;			 //SIM������״̬ 1=����, 0=����
	uint8 ucModuleState;	 //GSMģ�����״̬:1=����, 0=����
	uint8 ucSmsRdy;			 //���Ͷ��ŵ������Ƿ�߱�
	uint8  ucCGATT;          //���總�����
	uint16 usCREGLAC;        //λ������Ϣ
	uint16 usCREGCI;         //С����Ϣ
	uint8  ucSIMEX;          //SIM�����γ���ʶ
	uint8  aucCIMI[20];      //SIM����Ϣ
	uint8  aucCCID[20];      //SIM��CCID
	uint8  ucNETERR;         //�������
	uint8 ucGsmRegState;	 //GSMע��״̬: 1=��ע��,0=δע��
	uint8 ucWirelessNet;	 //GSM��������״̬:1=��׼����,0=û��׼����
	uint8 ucLinkState[MAX_GPRS_LINK];//��GPRS���ӣ�GPRS_LINK_STATE_CLOSED=�Ͽ�,GPRS_LINK_STATE_READY=��������
	                                 //            GPRS_LINK_STATE_BUSY=æ
	uint8 ucFtpState;		 //ftp״̬,0=δ����, 1=�Ѿ���      
	uint8 ucSIMCardError;    //ģ����SIM��״̬  ������ʱ���� 0=����,1=���

	uint32 uiLAC;           //λ������Ϣ
	uint32 uiCEll_ID;       //С����Ϣ
	
}STUGsmState,*PSTUGsmState;

//��������ַ
typedef struct _SER_ADDRESS_
{
	uint8 Addr[50];	//������IP������
	uint8  AddrLen;		//��������,������50�ַ�
	uint16 Port;		//�˿ں�
	uint8  ProcolType;	//0=TCP,1=UDP
}STU_SerAddress, *PSTU_SerAddress;

//-----�ⲿ����----------------------------------------------------------------
extern uint8      g_ucNumFlag; 

//-----�ⲿ����----------------------------------------------------------------
BOOL   GSM_SendToGsm(uint8 *pucData, uint16 usLen);

uint8  GSM_SendMsg(uint8 *pucData, uint16 usLen, uint8* DesNumber, uint8 NumLen);
int32  GSM_SendGprs(uint8 *pucData, uint16 usLen, uint8 ucLinkNum);

void   GSM_Wake(void);
void   GSM_Sleep(void);
void   GSM_Reset(void);
void  GSM_PwrOff(void);
void GSM_LinkReConnet(uint8 LinkNum);
void   GSM_TimerCount(void);

uint8 GSM_GetSimState(void);
uint32 GSM_GetCreg(void);
uint8  GSM_GetAtStep(void);
uint8  *GSM_GetCIMI(void);
uint8 *GSM_GetICCID(void);
uint8  GSM_GetSimExFlag(void);
uint8 GSM_GetCsq(void);
uint8 GSM_GetCgatt(void);
uint8 GSM_GetCpin(void);
uint8 GSM_GetGprsState(void);
uint8 GSM_GetGsmRegState(void);
uint8 GSM_GetGsmModuleState(void);
uint8 GSM_GetLinkState(uint8 ucLinkNum);
void Gsm_GetSmsSrcNumber(uint8 *number, uint8 *len);
void Gsm_ReleaseSmsBuff(void);
BOOL TTS_Play(uint8 TextIndex);
//BOOL GetTTSPlayIndex(uint8 * index);
void GSM_GetFtpData(uint16 len);
void GSM_OpenFtp(void);
void GSM_CloseFtp(void);
void SIMCardStateJudge(void);
uint8 Get_SIMCardState(void);
int32 Gsm_CreatSocket(void);
int32 OpenGsmAT_DEV_TTY(void);
void* pthread_gsm_init(void* data);
void* pthread_gsm_read_AT(void *data);
//void* pthread_quectel_CM(char *argv[]);
void* pthread_gsm_RecvData(void *data);

void ModemClose(void);
void ModemOpen(void);
void ModemPwrOff(void);
void ModemPwrOn(void);
void SetAtSend(uint8 at);
BOOL UnPackCPIN(const uint8 *pucSrc, uint16 usSrcLen);
BOOL UnPackCIMI(const uint8 *pucSrc, uint16 usSrcLen);
BOOL UnPackCCID(const uint8 *pucSrc, uint16 usSrcLen);
BOOL UnPackCSQ(const uint8 *pucSrc, uint16 usSrcLen);
BOOL UnPackCGATT(const uint8 *pucSrc, uint16 usSrcLen);
BOOL UnPackCPSI(const uint8 *pucSrc, uint16 usSrcLen);
void GSM_SetModemWorkingState(uint8 ucModemState);
void GSM_DataSendMutexInit(void);
BOOL UnPackCREG(uint8 *pucSrc, uint16 usSrcLen);
void GSM_Variable_Init(void);
void GSM_Modem_RST(void);

#endif



