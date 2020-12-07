/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GsmInterfaceLayer.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块接口层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by , 创建本文件
 *
 */
#include "config.h"
//#include "GsmHardWareLayer.h"
#include "GsmProtocolLayer.h"
#include "GsmInterfaceLayer.h"
#include "GsmIntegrateLayer.h"
#include <signal.h>



//-----外部变量定义------------------------------------------------------------
extern STU_GsmOperate GsmOperate; 
extern STUGsmState f_stuGsmState;
extern STU_GsmOperate GsmOperate;
extern STUGsmState f_stuGsmState;
uint8  g_ucNumFlag=0;               //发送短信时所用号码标识,1表示固化号码
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern stuSerAddr g_stuSerAddr;
pthread_mutex_t gGprsDataSendMutex;	/* 发送GPRS数据锁 */

//-----内部变量定义------------------------------------------------------------

STU_GprsSendDataBuff GprsSendDataBuff[MAX_GPRS_LINK];

STU_SmsSend f_stuSmsSend;//保存发送短信相关的信息

STU_TTS m_stuTTS;	//保存TTS相关信息

//GSM模块对外消息队列提供的队列项

//static STUSYSMsgBus f_stuGsmLink={SRCDEVICE_ID_GSM, GSM_MSG_ONLINE, 0, 0, 0, 0, 0};


//-----内部函数声明------------------------------------------------------------

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
* 功能描述: SIGPIPE信号处理函数
* 输入参数: 无
* 输出参数: 无
* 返 回 值: 无
* 说    明: 服务器出故障而代码中相关的函数在找不到服务器的情况下产生SIGPIPE信号，如果不做设置信号处理函数，
*           该信号会直接结束main进程。
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
    
    /* 1、打开ppp拨号 */
    if (Sim_PppOn() < 0)
    {
        LOG_ERROR("PPP on failed.\n");
        return;
    }

    /* 2、创建SOCKET链接。如果失败，再重试一次 */
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

//主动断开网络时还需要close(gGsm_socketfd);
//返回0-成功,-1=失败
/* 可以 设置send和recv超时时间为5s ??*/
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
	//sockaddr.sin_addr=inet_addr("192.168.0.1");/*"192.168.0.1"转换后 为0x0100A8C0*/
    //inet_pton(AF_INET,servInetAddr,&sockaddr.sin_addr);  //IP地址 
    sockaddr.sin_addr.s_addr = g_stuSerAddr.aucIp[0]+(uint32)(g_stuSerAddr.aucIp[1]<<8) 
                         +(uint32)(g_stuSerAddr.aucIp[2]<<16)+(uint32)(g_stuSerAddr.aucIp[3]<<24); //{58,218,196,200}

	#if 0
    /* 设置connect为非阻塞方式 */
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
** 函数名称: GSM_SendGprs
** 功能描述: GSM模块GPRS数据发送接口
**
** 输    入: pucData=待发送数据指针; usLen=待发送数据长度,ucLinkNu=通道编号,默认为0
** 输    出: 无
** 返    回: 0 :成功返回
             -1:发送失败
** 作    者: lxf
** 日    期: 2019-03-16
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
	if (GPRS_LINK_STATE_READY == GSM_GetLinkState(0))  //这地方需判断当前在线状态
	{
        pthread_mutex_lock(&gGprsDataSendMutex);//互斥锁

	    if(send(gGsm_socketfd, pucData, usLen, 0) < 0)
	    {
	        perror("send(2) error");
        	pthread_mutex_unlock(&gGprsDataSendMutex); //互斥锁解锁
		    return -1;
	    }
    	pthread_mutex_unlock(&gGprsDataSendMutex);     //互斥锁解锁
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
** 函数名称: GSM_Sleep
** 功能描述: 休眠GSM模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-09-15
***----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_Sleep(void)
{
	GsmOperate.GsmSleepOperate = 1;
}

/******************************************************************************
** 函数名称: GSM_Reset
** 功能描述: 重启GSM模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_Reset(void)
{
	PC_SendDebugData((uint8 *)("GSM RST2"), 8, DEBUG_AT);
    ResetGsm();
}

/******************************************************************************
** 函数名称: GSM_Pwroff
** 功能描述: 关闭GSM模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_PwrOn(void)
{
	GsmOperate.GsmEnable = 1;
   //	ResetGsm();
}

/******************************************************************************
** 函数名称: GSM_Pwroff
** 功能描述: 关闭GSM模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-09-15
**-----------------------------------------------------------------------------
*******************************************************************************/
void  GSM_PwrOff(void)
{
	GsmOperate.GsmOperate = 2;
}

/******************************************************************************
** 函数名称: GSM_Wake
** 功能描述: 外部唤醒GSM模块
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: 
** 日    期: 2011-07-21
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_Wake(void)
{
	GsmOperate.GsmEnable = 1;
	GsmOperate.GsmSleepOperate = 2;
}

/******************************************************************************
** 函数名称: GSM_SendToGsm
** 功能描述: 外部程序向GSM模块发任意AT指令接口
**
** 输    入: pucData,待发送数据;usLen,待发送数据长度
** 输    出: 无
** 返    回: 成功返回TRUE,失败(越界)返回FALSE
**
** 作    者: 
** 日    期: 2011-08-08
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL GSM_SendToGsm(uint8 *pucData, uint16 usLen)
{
   // return WriteGsmUartData(pucData, usLen);
   return 0;
}

/******************************************************************************
** 函数名称: GSM_SendMsg
** 功能描述: GSM模块短信发送接口
**
** 输    入: pucData,待发送数据指针; usLen,待发送数据长度
** 输    出: 无
** 返    回: 0=进入待发送状态
             1=参数错误
             2=网络或GSM模块未准备好,或当前有一条短信正在发送
**
** 作    者: hhm
** 日    期: 2014-12-1
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
** 函数名称: GSM_SendGprs
** 功能描述: GSM模块GPRS数据发送接口
**
** 输    入: pucData=待发送数据指针; usLen=待发送数据长度,ucLinkNu=通道编号
** 输    出: 无
** 返    回: 0:成功返回
             1:参数错误空
             2:链接未建立或忙
** 作    者: hhm
** 日    期: 2014-09-17
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
    for(i=0; i<200; i++)		//如果该链接忙，最多等待2s
    {
		if(GPRS_LINK_STATE_READY == GSM_GetLinkState(ucLinkNum))                  //GPRS数据发送条件不成熟
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
** 函数名称: GSM_LinkOpen
** 功能描述: 打开一个链接
** 输    入: 链接编号
** 输    出: 无
** 返    回: 无
** 
** 作    者: hhm
** 日    期: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkOpen(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 1;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** 函数名称: GSM_LinkClose
** 功能描述: 关闭一个链接
** 输    入: 链接编号
** 输    出: 无
** 返    回: 无
** 
** 作    者: hhm
** 日    期: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkClose(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 2;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** 函数名称: GSM_LinkReConnet
** 功能描述: 重新连接一个链接
** 输    入: 链接编号
** 输    出: 无
** 返    回: 无
** 
** 作    者: hhm
** 日    期: 2014-09-17
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_LinkReConnet(uint8 LinkNum)
{
	GsmOperate.LinkOperate[LinkNum] = 3;
	f_stuGsmState.ucLinkState[LinkNum] = GPRS_LINK_STATE_CLOSED;
}
/******************************************************************************
** 函数名称: GSM_GetLinkState
** 功能描述: 获取各链接状态
**
** 输    入: 链接编号
** 输    出: 无
** 返    回: GPRS_LINK_STATE_CLOSED = 断开,
             GPRS_LINK_STATE_READY  = 已连接上,可以发送数据
	         GPRS_LINK_STATE_BUSY   = 忙
** 
** 作    者: hhm
** 日    期: 2014-09-17
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
** 函数名称: GSM_GetCpin
** 功能描述: 获取SIM卡信息
**
** 输    入: 无
** 输    出: 无
** 返    回: 获取cpin状态
**
** 作    者: 
** 日    期: 2011-10-09
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCpin(void)
{
    return f_stuGsmState.ucCPIN;
}

/******************************************************************************
** 函数名称: GSM_GetCIMI
** 功能描述: 获取SIM卡信息
**
** 输    入: 无
** 输    出: 无
** 返    回: SIM卡信息,首字节为SIM卡信息长度
**
** 作    者: 
** 日    期: 2011-10-09
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GSM_GetCIMI(void)
{
    return f_stuGsmState.aucCIMI;
}

/******************************************************************************
** 函数名称: GSM_GetICCID
** 功能描述: 获取SIM卡ICCID信息
**
** 输    入: 无
** 输    出: 无
** 返    回: 
**
** 作    者: hhm
** 日    期: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GSM_GetICCID(void)
{
    return f_stuGsmState.aucCCID;
}
/******************************************************************************
** 函数名称: GSM_GetSimExFlag
** 功能描述: 获取SIM是否更换过标识
**
** 输    入: 无
** 输    出: 无
** 返    回: 1,更换过; 0,未更换
**
** 作    者: 
** 日    期: 2011-10-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetSimExFlag(void)
{
    return f_stuGsmState.ucSIMEX;
}

/******************************************************************************
** 函数名称: GSM_GetSimState
** 功能描述: 获取SIM卡是否故障
**
** 输    入: 无
** 输    出: 无
** 返    回: 1=故障; 0,正常
**
** 作    者: 
** 日    期: 2011-10-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetSimState(void)
{
    return f_stuGsmState.ucSimErr;
}


/******************************************************************************
** 函数名称: GSM_GetCreg
** 功能描述: 获取小区信息及位置码
**
** 输    入: 无
** 输    出: 无
** 返    回: 小区信息及位置码
**
** 作    者: 
** 日    期: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 GSM_GetCreg()
{
    return f_stuGsmState.usCREGLAC<<16 | f_stuGsmState.usCREGCI;
}

/******************************************************************************
** 函数名称: GSM_Get4GLac_Cell   (TLV-0x301F)
** 功能描述: 获取4G模块小区信息及位置码
**
** 输    入: 无
** 输    出: 无
** 返    回: 小区信息及位置码
**
** 作    者: 
** 日    期: 2020-11-26
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
** 函数名称: GSM_GetCsq
** 功能描述: 获取信号强度
**
** 输    入: 无
** 输    出: 无
** 返    回: 小区信息及位置码
**
** 作    者: 
** 日    期: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCsq()
{
    return f_stuGsmState.ucCSQ;
}


/******************************************************************************
** 函数名称: GSM_GetCgatt
** 功能描述: 获取网络附着状态
**
** 输    入: 无
** 输    出: 无
** 返    回: 小区信息及位置码
**
** 作    者: 
** 日    期: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetCgatt()
{
  return f_stuGsmState.ucCGATT;
}

/******************************************************************************
** 函数名称: GSM_GetGprsState
** 功能描述: 获取GPRS网络状态
**
** 输    入: 无
** 输    出: 
** 返    回: 1=已准备好,0=没有准备好
** 作    者: hhm
** 日    期: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGprsState()
{
  return f_stuGsmState.ucWirelessNet;
}

/******************************************************************************
** 函数名称: GSM_GetGsmRegState
** 功能描述: 获取GSM注册状态
**
** 输    入: 无
** 输    出: 
** 返    回: 1=已注册,0=未注册
** 作    者: hhm
** 日    期: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGsmRegState()
{
  return f_stuGsmState.ucGsmRegState;
}

/******************************************************************************
** 函数名称: GSM_GetGsmModuleState
** 功能描述: 获取GSM注册状态
**
** 输    入: 无
** 输    出: 
** 返    回: GSM模块故障状态:1=故障, 0=正常
** 作    者: hhm
** 日    期: 2016-9-6
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GSM_GetGsmModuleState()
{
  return f_stuGsmState.ucModuleState;
}

/******************************************************************************
** 函数名称: GSM_GetAtStep
** 功能描述: 获取AT状态
**
** 输    入: 无
** 输    出: 无
** 返    回: 
**
** 作    者: 
** 日    期: 2011-08-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8  GSM_GetAtStep(void)
{
	return GetAtSend();
}

/******************************************************************************
** 函数名称: GSM_GetFtpData
** 功能描述: 获取ftp数据
** 输    入: 无
** 输    出: 无
** 返    回: 
** 作    者: hhm
** 日    期: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_GetFtpData(uint16 len)
{
	GsmOperate.usFtpGetLen = len;
	GsmOperate.FtpOperate = 2;
}

/******************************************************************************
** 函数名称: GSM_OpenFtp
** 功能描述: 建立ftp连接
** 输    入: 无
** 输    出: 无
** 返    回: 
** 作    者: hhm
** 日    期: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_OpenFtp()
{
	GsmOperate.FtpOperate = 1;
}

/******************************************************************************
** 函数名称: GSM_CloseFtp
** 功能描述: 关闭ftp连接
** 输    入: 无
** 输    出: 无
** 返    回: 
** 作    者: hhm
** 日    期: 2016-7-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GSM_CloseFtp()
{
	GsmOperate.FtpOperate = 3;
}
/******************************************************************************
** 函数名称: TTS_Play
** 功能描述: 语音播报接口
**
** 输    入: 需要播报的语句的序号,取值为:TTS_FLAY_xxxx
** 输    出: 无
** 返    回: TRUE=成功, FALSE=失败
**
** 作    者: hhm
** 日    期: 2014-08-01
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
** 函数名称: InitGsmModule
** 功能描述: modem初始化
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: 
** 日    期: 2011-07-08
**-----------------------------------------------------------------------------
*******************************************************************************/
void InitGsmModule()
{
  	GsmOperate.GsmEnable  = 1;
	GsmOperate.GsmOperate = 1;
}

/******************************************************************************
** 函数名称: GSM_TimerCount
** 功能描述: GSM模块计数器接口
**
** 输    入: 无
** 输    出: 无
** 返    回: 无
**
** 作    者: 
** 日    期: 2011-07-04
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



