/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GsmModule.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块对外接口头文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-06-09 by  创建本文件
 *
 */

#ifndef GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138
#define GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138

#include "GsmHardWareLayer.h"
//-----常量定义----------------------------------------------------------------
#define GSM_RECV_BUFF_MAX_SIZE        1500
#define GSM_SEND_BUFF_MAX_SIZE        1500
#define GSM_AT_RECV_BUFF_MAX_SIZE     1000


#define TASK_GSM_RECV_STK_SIZE        400  //GSM模块接收任务栈大小
#define TASK_GSM_RECV_ID              4    //GSM模块接收任务ID
#define TASK_GSM_RECV_PRIO            4    //GSM模块接收任务优先级

#define TASK_GSM_SERVICE_STK_SIZE     400  //GSM模块主任务栈大小
#define TASK_GSM_SERVICE_ID           5    //GSM模块主任务ID
#define TASK_GSM_SERVICE_PRIO         5    //GSM模块主任务优先级

#define GPRS_SEND_BUFF_LEN			  1536   //大数据包大小1.5k

#define SMS_SEND_BUFF_LEN			  256	//缓存的短信的长度
#define MAX_GPRS_LINK				  1	   //最大GRPS链接数量,对于SIM800,该值不能大于6	

#define GPRS_LINK_BUBIAO			  0    //部标连接
#define GPRS_LINK_BACKUP			  1    //部标备用链接
#define GPRS_LINK_CUSTOMER			  2	   //定制客户链接
#define GPRS_LINK_ALL			      0xff //所有链接

#define GSM_MSG_GPRS_RECV             1    //收到GPRS包
#define GSM_MSG_RING_RECV             2    //收到RING
#define GSM_MSG_SMS_RECV              3    //收到SMS
#define GSM_MSG_SMS_SEND_OK           4    //发送SMS成功
#define GSM_MSG_SMS_SEND_ERR          5    //发送SMS失败
#define GSM_MSG_ONLINE                6    //GSM上线标识,通知外部可发GPRS包
#define GSM_MSG_FTP_RECV              7    //收到ftp数据包


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

/********语音播报语句序号定义***************/
#define TTS_PLAY_TEXT_SPEEDOVER					0	//车辆超速
#define TTS_PLAY_TEXT_GNSSANTANNAFAULT			1	//定位天线故障
#define TTS_PLAY_TEXT_ICCARDIN					2	//IC卡插入
#define TTS_PLAY_TEXT_ICCARDOUT			        3	//IC卡拔出
#define TTS_PLAY_TEXT_ICCARDERR			        4	//IC卡故障
#define TTS_PLAY_TEXT_TIMEOUTDRIVE				5	//超时驾驶
#define TTS_PLAY_TEXT_ICCARDINSERTNOTE			6	//提醒驾驶员插入IC卡
#define TTS_PLAY_TEXT_SPEEDUNNORMAL				7	//速度异常
#define TTS_PLAY_TEXT_PLATFORM_TEXTDOWNLOARD    8	//平台下发的文本播报
/******************************************/
#define SIMCARD_Satate_Timer                 180    //判断SIM卡延时时间
#define One_MilliSecond                      1000   //1ms

//-----结构定义----------------------------------------------------------------

typedef struct _STU_GSMSTATE_
{
	uint8 ucModemState;		 //GSM模块状态:0=启动,1=拨号,2=创建socket,3=创建成功,
	                            //4=关闭拨号,5=关闭socket,6=快重连,7=休眠
	uint8 ucQMIWwanEnable;   //1=通知模块启动网络拨号,0=关闭模块拨号
	uint8 ucQMIWwanState;    //模块拨号结果:1=成功,0=失败
	uint8 ucModemSleepState; //GSM模块休眠状态:0=唤醒, 1=已休眠,2=正在休眠
    uint8  ucCSQ;            //信号强度
	uint8  ucCPIN;           //SIM卡当前状态 1=检测到SIM卡,0=未检测到SIM卡
	uint8 ucSimErr;			 //SIM卡故障状态 1=故障, 0=正常
	uint8 ucModuleState;	 //GSM模块故障状态:1=故障, 0=正常
	uint8 ucSmsRdy;			 //发送短信的条件是否具备
	uint8  ucCGATT;          //网络附着情况
	uint16 usCREGLAC;        //位置码信息
	uint16 usCREGCI;         //小区信息
	uint8  ucSIMEX;          //SIM卡曾拔出标识
	uint8  aucCIMI[20];      //SIM卡信息
	uint8  aucCCID[20];      //SIM卡CCID
	uint8  ucNETERR;         //网络故障
	uint8 ucGsmRegState;	 //GSM注册状态: 1=已注册,0=未注册
	uint8 ucWirelessNet;	 //GSM无线网络状态:1=已准备好,0=没有准备好
	uint8 ucLinkState[MAX_GPRS_LINK];//各GPRS链接，GPRS_LINK_STATE_CLOSED=断开,GPRS_LINK_STATE_READY=已连接上
	                                 //            GPRS_LINK_STATE_BUSY=忙
	uint8 ucFtpState;		 //ftp状态,0=未连接, 1=已就绪      
	uint8 ucSIMCardError;    //模块检测SIM卡状态  经过延时处理 0=正常,1=拆除

	uint32 uiLAC;           //位置码信息
	uint32 uiCEll_ID;       //小区信息
	
}STUGsmState,*PSTUGsmState;

//服务器地址
typedef struct _SER_ADDRESS_
{
	uint8 Addr[50];	//服务器IP或域名
	uint8  AddrLen;		//域名长度,不超过50字符
	uint16 Port;		//端口号
	uint8  ProcolType;	//0=TCP,1=UDP
}STU_SerAddress, *PSTU_SerAddress;

//-----外部变量----------------------------------------------------------------
extern uint8      g_ucNumFlag; 

//-----外部函数----------------------------------------------------------------
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



