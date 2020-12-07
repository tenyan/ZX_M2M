/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmInterfaceLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ��ӿڲ��ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by , �������ļ�
 *
 */
#include "config.h"
//#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmInterfaceLayer.h"
#include "GsmIntegrateLayer.h"
#include <signal.h>



//-----�ⲿ��������------------------------------------------------------------
extern STU_GsmOperate GsmOperate; 
extern STUGsmState f_stuGsmState;
extern STU_GsmOperate GsmOperate;
extern STUGsmState f_stuGsmState;
uint8  g_ucNumFlag=0;               //���Ͷ���ʱ���ú����ʶ,1��ʾ�̻�����
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern stuSerAddr g_stuSerAddr;
pthread_mutex_t gGprsDataSendMutex;	/* ����GPRS������ */

//-----�ڲ���������------------------------------------------------------------

STU_GprsSendDataBuff GprsSendDataBuff[MAX_GPRS_LINK];

STU_SmsSend f_stuSmsSend;//���淢�Ͷ�����ص���Ϣ

STU_TTS m_stuTTS;	//����TTS�����Ϣ

//GSMģ�������Ϣ�����ṩ�Ķ�����

//static STUSYSMsgBus f_stuGsmLink={SRCDEVICE_ID_GSM, GSM_MSG_ONLINE, 0, 0, 0, 0, 0};


//-----�ڲ���������------------------------------------------------------------

int gGsm_socketfd = -1;

/*
unsigned char sendline[MAXLINE] = {0x07,0xA0,0x10,0x10,0x11,0x10,0x00,0x01,0x00,
	0x00,0x09,0x00,0x01,0xF3};    //len=14
int n;
char buff[1024]={0};
char bufftemp[1024]={0};
int len = 0;
int i = 0;
*/

/***********************************************************************************************************
* ��������: SIGPIPE�źŴ�����
* �������: ��
* �������: ��
* �� �� ֵ: ��
* ˵    ��: �����������϶���������صĺ������Ҳ���������������²���SIGPIPE�źţ�������������źŴ�������
*           ���źŻ�ֱ�ӽ���main���̡�
***********************************************************************************************************/
static void Gsm_SigpipeHandle(void)
{
    printf("There has a SIGPIPE signal.\n");

    f_stuGsmState.ucLinkState[0] = GPRS_LINK_STATE_CLOSED;
	g_stuSystem.ucResetModem = 1;
	close(gGsm_socketfd);
	gGsm_socketfd = -1;
	
#if 0
    gTcpOkFlag = 0;
    
    /* 1����ppp���� */
    if (Sim_PppOn() < 0)
    {
        LOG_ERROR("PPP on failed.\n");
        return;
    }

    /* 2������SOCKET���ӡ����ʧ�ܣ�������һ�� */
    if (Sim_CreatSocket() < 0)
    {
        LOG_ERROR("Creat socket failed.\n");
        return;
    }
    
    gTcpOkFlag = 1;
	#endif	
}

void GSM_DataSendMutexInit(void)
{
    pthread_mutex_init(&gGprsDataSendMutex, NULL);   //
}
void Gsm_Sigpipe(void)
{
    struct sigaction action;
    action.sa_handler = Gsm_SigpipeHandle;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGPIPE, &action, NULL);
}

//�����Ͽ�����ʱ����Ҫclose(gGsm_socketfd);
//����0-�ɹ�,-1=ʧ��
/* ���� ����send��recv��ʱʱ��Ϊ5s ??*/
int32 Gsm_CreatSocket(void)
{
    int Result=0;

    struct sockaddr_in sockaddr;
   // char *servInetAddr = "58.218.196.200";

//    pthread_mutex_init(&gGprsDataSendMutex, NULL);   //

    f_stuGsmState.ucLinkState[0] = GPRS_LINK_STATE_CLOSED;
    if ((gGsm_socketfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        perror("socket(2) error");
        return -1;
    }

	printf("Gsm_CreatSocket: %d\n", gGsm_socketfd);
    memset(&sockaddr,0,sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons(g_stuSerAddr.usPort);
  //  sockaddr.sin_port = htons(10004);     //port
	//sockaddr.sin_addr=inet_addr("192.168.0.1");/*"192.168.0.1"ת���� Ϊ0x0100A8C0*/
    //inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);  //IP��ַ 
    sockaddr.sin_addr.s_addr = g_stuSerAddr.aucIp[0]+(uint32)(g_stuSerAddr.aucIp[1]<<8) 
                         +(uint32)(g_stuSerAddr.aucIp[2]<<16)+(uint32)(g_stuSerAddr.aucIp[3]<<24); //{58,218,196,200}

	#if 0
    /* ����connectΪ��������ʽ */
    iFlag = fcntl(gGsm_socketfd, F_GETFL, 0);
    iFlag |= O_NONBLOCK;
    if (-1 == fcntl(gGsm_socketfd, F_SETFL, iFlag))
    {
      //  LOG_PERROR("fcntl");
        printf("fcntl_No_ZuSe_Err\n");
    }	
	#endif
    Gsm_Sigpipe();

    Result = connect(gGsm_socketfd,(struct sockaddr*)&sockaddr,sizeof(sockaddr));
	if(Result < 0 )
    {
        printf("connect error %s errno: %d\n",strerror(errno),errno);
		return -1;
    }

	//Gsm_Sigpipe();
  //  signal(SIGPIPE, (__sighandler_t)Sim_SigpipeHandle);

	#if 0
    if (-1 == fcntl(gGsm_socketfd, F_SETFL, 0))
    {
        printf("fcntl_ZuSe_Err\n");
        perror("socket(2) error");
    }	
	#endif
	return 0;	
}


/******************************************************************************
** ��������: GSM_SendGprs
** ��������: GSMģ��GPRS���ݷ��ͽӿ�
**
** ��    ��: pucData=����������ָ��; usLen=���������ݳ���,ucLinkNu=ͨ�����,Ĭ��Ϊ0
** ��    ��: ��
** ��    ��: 0 :�ɹ�����
             -1:����ʧ��
** ��    ��: lxf
** ��    ��: 2019-03-16
**-----------------------------------------------------------------------------
*******************************************************************************/
int32 GSM_SendGprs(uint8 *pucData, uint16 usLen, uint8 ucLinkNum)
{

	uint8 BuffTemp[GSM_RECV_BUFF_MAX_SIZE]={0};
	
    if((NULL==pucData) || (0==usLen) || (usLen>GSM_SEND_BUFF_MAX_SIZE) || (ucLinkNum>2))
	{
		PC_SendDebugData((uint8 *)("GPRS Send Para Err"), 18, DEBUG_AT);
	    return -1;
	}

   /* if(GPRS_LINK_STATE_READY != GSM_GetLinkState(0))
	{
		return -1;
	}
	*/
	if (GPRS_LINK_STATE_READY == GSM_GetLinkState(0))  //��ط����жϵ�ǰ����״̬
	{
        pthread_mutex_lock(&gGprsDataSendMutex);//������

	    if(send(gGsm_socketfd, pucData, usLen, 0) < 0)
	    {
	        perror("send(2) error");
        	pthread_mutex_unlock(&gGprsDataSendMutex); //����������
		    return -1;
	    }
    	pthread_mutex_unlock(&gGprsDataSendMutex);     //����������
        if(SYS_GetDubugStatus()&BIT(0))
    		DataToHexbuff(BuffTemp, pucData, usLen);
	//	PrintTime();
	//	printf("Send,Len= %d; %s\n",usLen, BuffTemp);	
	    PC_SendDebugData(BuffTemp, usLen*2, DEBUG_AT);
        PC_SendDebugData(pucData, usLen, DEBUG_GPRS);

	}
	else
		return -1;

    return 0;

}

/******************************************************************************
** ��������: GSM_Sleep
** ��������: ����GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-15
***----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_Sleep(void)
{
	GsmOperate.GsmSleepOperate = 1;
}

/******************************************************************************
** ��������: GSM_Reset
** ��������: ����GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_Reset(void)
{
	PC_SendDebugData((uint8 *)("GSM RST2"), 8, DEBUG_AT);
    ResetGsm();
}

/******************************************************************************
** ��������: GSM_Pwroff
** ��������: �ر�GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_PwrOn(void)
{
	GsmOperate.GsmEnable = 1;
   //	ResetGsm();
}

/******************************************************************************
** ��������: GSM_Pwroff
** ��������: �ر�GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_PwrOff(void)
{
	GsmOperate.GsmOperate = 2;
}

/******************************************************************************
** ��������: GSM_Wake
** ��������: �ⲿ����GSMģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-07-21
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_Wake(void)
{
	GsmOperate.GsmEnable = 1;
	GsmOperate.GsmSleepOperate = 2;
}

/******************************************************************************
** ��������: GSM_SendToGsm
** ��������: �ⲿ������GSMģ�鷢����ATָ��ӿ�
**
** ��    ��: pucData,����������;usLen,���������ݳ���
** ��    ��: ��
** ��    ��: �ɹ�����TRUE,ʧ��(Խ��)����FALSE
**
** ��    ��: 
** ��    ��: 2011-08-08
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL GSM_SendToGsm(uint8 *pucData, uint16 usLen)
{
   // return WriteGsmUartData(pucData, usLen);
   return 0;
}

/******************************************************************************
** ��������: GSM_SendMsg
** ��������: GSMģ����ŷ��ͽӿ�
**
** ��    ��: pucData,����������ָ��; usLen,���������ݳ���
** ��    ��: ��
** ��    ��: 0=���������״̬
             1=��������
             2=�����GSMģ��δ׼����,��ǰ��һ���������ڷ���
**
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_SendMsg(uint8 *pucData, uint16 usLen, uint8* DesNumber, uint8 NumLen)
{
    if(NULL==pucData || 0==usLen || DesNumber==NULL || NumLen==0)
	{
	    return 1;
	}
	if(!IsSmsReady())
	{
	    return 2;
	}
	
	f_stuSmsSend.ucNumberLen = NumLen;
	memcpy(f_stuSmsSend.aucDesNumber, DesNumber, NumLen);
	memcpy(f_stuSmsSend.aucSmsSendBuff, pucData, usLen);
	f_stuSmsSend.flag = 1;
	
	return 0;
}

/******************************************************************************
** ��������: GSM_SendGprs
** ��������: GSMģ��GPRS���ݷ��ͽӿ�
**
** ��    ��: pucData=����������ָ��; usLen=���������ݳ���,ucLinkNu=ͨ�����
** ��    ��: ��
** ��    ��: 0:�ɹ�����
             1:���������
             2:����δ������æ
** ��    ��: hhm
** ��    ��: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
uint8 GSM_SendGprs(uint8 *pucData, uint16 usLen, uint8 ucLinkNum)
{
	uint8 i;
	
    if((NULL==pucData) || (0==usLen) || (usLen>GPRS_SEND_BUFF_LEN) || (ucLinkNum>2))
	{
		PC_SendDebugData((uint8 *)("GPRS Send Para Err"), 18, DEBUG_ANYDATA);
	    return 1;
	}
    for(i=0; i<200; i++)		//���������æ�����ȴ�2s
    {
		if(GPRS_LINK_STATE_READY == GSM_GetLinkState(ucLinkNum))                  //GPRS���ݷ�������������
		   	break;
		else
			OSTimeDly(OS_TICKS_PER_SEC/100);
    }
	if(GPRS_LINK_STATE_READY != GSM_GetLinkState(ucLinkNum))
	{
		PC_SendDebugData((uint8 *)("GPRS Send Busy"), 14, DEBUG_ANYDATA);
	 	return 2;
	}
	memcpy(GprsSendDataBuff[ucLinkNum].buff, pucData, usLen);
	GprsSendDataBuff[ucLinkNum].ucLinkNum = ucLinkNum;
	GprsSendDataBuff[ucLinkNum].usLen = usLen;
	GprsSendDataBuff[ucLinkNum].flag = 1;
	
	f_stuGsmState.ucLinkState[ucLinkNum] = GPRS_LINK_STATE_BUSY;

	return 0;
}
#endif



/******************************************************************************
** ��������: GSM_LinkOpen
** ��������: ��һ������
** ��    ��: ���ӱ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: hhm
** ��    ��: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkOpen(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 1;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** ��������: GSM_LinkClose
** ��������: �ر�һ������
** ��    ��: ���ӱ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: hhm
** ��    ��: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkClose(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 2;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** ��������: GSM_LinkReConnet
** ��������: ��������һ������
** ��    ��: ���ӱ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: hhm
** ��    ��: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkReConnet(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 3;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** ��������: GSM_GetLinkState
** ��������: ��ȡ������״̬
**
** ��    ��: ���ӱ��
** ��    ��: ��
** ��    ��: GPRS_LINK_STATE_CLOSED = �Ͽ�,
             GPRS_LINK_STATE_READY  = ��������,���Է�������
	         GPRS_LINK_STATE_BUSY   = æ
** 
** ��    ��: hhm
** ��    ��: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetLinkState(uint8 ucLinkNum)
{
	uint8 ret = 0xff;
	
	if(ucLinkNum>=MAX_GPRS_LINK)
		return 0xff;
	ret = f_stuGsmState.ucLinkState[ucLinkNum];
	
	return 	ret;
}


/******************************************************************************
** ��������: GSM_GetCpin
** ��������: ��ȡSIM����Ϣ
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��ȡcpin״̬
**
** ��    ��: 
** ��    ��: 2011-10-09
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCpin(void)
{
    return f_stuGsmState.ucCPIN;
}

/******************************************************************************
** ��������: GSM_GetCIMI
** ��������: ��ȡSIM����Ϣ
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: SIM����Ϣ,���ֽ�ΪSIM����Ϣ����
**
** ��    ��: 
** ��    ��: 2011-10-09
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GSM_GetCIMI(void)
{
    return f_stuGsmState.aucCIMI;
}

/******************************************************************************
** ��������: GSM_GetICCID
** ��������: ��ȡSIM��ICCID��Ϣ
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: 
**
** ��    ��: hhm
** ��    ��: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GSM_GetICCID(void)
{
    return f_stuGsmState.aucCCID;
}
/******************************************************************************
** ��������: GSM_GetSimExFlag
** ��������: ��ȡSIM�Ƿ��������ʶ
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: 1,������; 0,δ����
**
** ��    ��: 
** ��    ��: 2011-10-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetSimExFlag(void)
{
    return f_stuGsmState.ucSIMEX;
}

/******************************************************************************
** ��������: GSM_GetSimState
** ��������: ��ȡSIM���Ƿ����
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: 1=����; 0,����
**
** ��    ��: 
** ��    ��: 2011-10-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetSimState(void)
{
    return f_stuGsmState.ucSimErr;
}


/******************************************************************************
** ��������: GSM_GetCreg
** ��������: ��ȡС����Ϣ��λ����
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: С����Ϣ��λ����
**
** ��    ��: 
** ��    ��: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 GSM_GetCreg()
{
    return f_stuGsmState.usCREGLAC<<16 | f_stuGsmState.usCREGCI;
}

/******************************************************************************
** ��������: GSM_Get4GLac_Cell   (TLV-0x301F)
** ��������: ��ȡ4Gģ��С����Ϣ��λ����
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: С����Ϣ��λ����
**
** ��    ��: 
** ��    ��: 2020-11-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_Get4GLac_Cell(uint8 *ptr)
{
    *ptr++ = 0x30;
	*ptr++ = 0x1F;
	*ptr++ = 0;
	*ptr++ = 8;
    *ptr++ = (uint8)(f_stuGsmState.uiLAC>>24);
    *ptr++ = (uint8)(f_stuGsmState.uiLAC>>16);
    *ptr++ = (uint8)(f_stuGsmState.uiLAC>>8);
    *ptr++ = (uint8)(f_stuGsmState.uiLAC&0XFF);
    *ptr++ = (uint8)(f_stuGsmState.uiCEll_ID>>24);
    *ptr++ = (uint8)(f_stuGsmState.uiCEll_ID>>16);
    *ptr++ = (uint8)(f_stuGsmState.uiCEll_ID>>8);
    *ptr++ = (uint8)(f_stuGsmState.uiCEll_ID&0XFF);

	return 12;	
}
/******************************************************************************
** ��������: GSM_GetCsq
** ��������: ��ȡ�ź�ǿ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: С����Ϣ��λ����
**
** ��    ��: 
** ��    ��: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCsq()
{
    return f_stuGsmState.ucCSQ;
}


/******************************************************************************
** ��������: GSM_GetCgatt
** ��������: ��ȡ���總��״̬
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: С����Ϣ��λ����
**
** ��    ��: 
** ��    ��: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCgatt()
{
  return f_stuGsmState.ucCGATT;
}

/******************************************************************************
** ��������: GSM_GetGprsState
** ��������: ��ȡGPRS����״̬
**
** ��    ��: ��
** ��    ��: 
** ��    ��: 1=��׼����,0=û��׼����
** ��    ��: hhm
** ��    ��: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGprsState()
{
  return f_stuGsmState.ucWirelessNet;
}

/******************************************************************************
** ��������: GSM_GetGsmRegState
** ��������: ��ȡGSMע��״̬
**
** ��    ��: ��
** ��    ��: 
** ��    ��: 1=��ע��,0=δע��
** ��    ��: hhm
** ��    ��: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGsmRegState()
{
  return f_stuGsmState.ucGsmRegState;
}

/******************************************************************************
** ��������: GSM_GetGsmModuleState
** ��������: ��ȡGSMע��״̬
**
** ��    ��: ��
** ��    ��: 
** ��    ��: GSMģ�����״̬:1=����, 0=����
** ��    ��: hhm
** ��    ��: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGsmModuleState()
{
  return f_stuGsmState.ucModuleState;
}

/******************************************************************************
** ��������: GSM_GetAtStep
** ��������: ��ȡAT״̬
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: 
**
** ��    ��: 
** ��    ��: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8  GSM_GetAtStep(void)
{
	return GetAtSend();
}

/******************************************************************************
** ��������: GSM_GetFtpData
** ��������: ��ȡftp����
** ��    ��: ��
** ��    ��: ��
** ��    ��: 
** ��    ��: hhm
** ��    ��: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_GetFtpData(uint16 len)
{
	GsmOperate.usFtpGetLen = len;
	GsmOperate.FtpOperate = 2;
}

/******************************************************************************
** ��������: GSM_OpenFtp
** ��������: ����ftp����
** ��    ��: ��
** ��    ��: ��
** ��    ��: 
** ��    ��: hhm
** ��    ��: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_OpenFtp()
{
	GsmOperate.FtpOperate = 1;
}

/******************************************************************************
** ��������: GSM_CloseFtp
** ��������: �ر�ftp����
** ��    ��: ��
** ��    ��: ��
** ��    ��: 
** ��    ��: hhm
** ��    ��: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_CloseFtp()
{
	GsmOperate.FtpOperate = 3;
}
/******************************************************************************
** ��������: TTS_Play
** ��������: ���������ӿ�
**
** ��    ��: ��Ҫ�������������,ȡֵΪ:TTS_FLAY_xxxx
** ��    ��: ��
** ��    ��: TRUE=�ɹ�, FALSE=ʧ��
**
** ��    ��: hhm
** ��    ��: 2014-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL TTS_Play(uint8 TextIndex)
{
	if(TextIndex>15)
		return FALSE;
    m_stuTTS.PlayFlag |= BIT(TextIndex);
	return TRUE;
}

/******************************************************************************
** ��������: InitGsmModule
** ��������: modem��ʼ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-07-08
**-----------------------------------------------------------------------------
*******************************************************************************/
void InitGsmModule()
{
  	GsmOperate.GsmEnable  = 1;
	GsmOperate.GsmOperate = 1;
}

/******************************************************************************
** ��������: GSM_TimerCount
** ��������: GSMģ��������ӿ�
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-07-04
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_TimerCount(void)
{
	static uint8 sec1 = 100;

	if(0==sec1--)
	{
		sec1 = 100;

	//	SendSmsTimeoutCount();
		#if 0
		TTSTimerCount();
		#endif
	}
}



