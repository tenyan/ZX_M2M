/*
 * Copyright(c)2019, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: SystemModule.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为System功能模块所有对外接口层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16, by lxf, 创建本文件
 *
 */

#ifndef _SYSTEMModule_H
#define _SYSTEMModule_H

#include "SystemProtocol.h"
#include "Mcu.h"


//-----常量定义----------------------------------------------------------------
//消息数据来源设备编码
#define SRCDEVICE_ID_GSM                   	1            //服务器GPRS
#define SRCDEVICE_ID_MCU                   	3            //MCU模块
#define SRCDEVICE_ID_GPS                   	4            //GPS模块
#define SRCDEVICE_ID_COLLECT               	5            //采集模块
#define SRCDEVICE_ID_SETTOOL               	6            //设置程序
#define SRCDEVICE_ID_5CGPS                	0x0B
#define SRCDEVICE_ID_5FGPS               	0x0E
#define SRCDEVICE_ID_5FCGPS                	0x12
#define SRCDEVICE_ID_3GGPS                	0x02



//设备参数获取索引号INDEX

//#define  INDEX_SPARE_SEVER_IP_PORT        0x0F         //备用服务器IP及Port 6字节
#define  INDEX_USER_NUMBER                1            //客户厂商代码
#define  INDEX_GPS_DEVICEID               2            //GPS终端ID号 4字节
#define  INDEX_SEVER_IP_PORT              3            //服务器IP及Port 6字节
#define  INDEX_ANP                        4            //APN     APN长度+APN内容   变长
#define  INDEX_SMS_CENTER                 5            //短信猫号码   短信猫号码长度+短信猫号码内容
#define  INDEX_SEVER_DNAME                6            //服务器域名   服务器域名长度+服务器域名
#define  INDEX_SEVER_DISTANCE             7            //总里程


//#define  ZXTCB_SEND_TIMER               
//#define  ZXTCW_SEND_TIMER
#define  ZXTCD_SEND_TIMER                  60      //默认60秒
#define  HEART_BEAT_TIMER                  180     //终端心跳间隔

#define  ZX_SEND_FLAG_TCS                  1
#define  ZX_SEND_FLAG_TCB                  2
#define  ZX_SEND_FLAG_TCW                  3
#define  ZX_SEND_FLAG_TCD                  4

#define  DATA_SaveTOMemory_MAX_SIZE        100             
//-----结构定义----------------------------------------------------------------
typedef struct _STU_tcp_
{
    uint8_t net[32];
    uint8_t port[8];
    uint8_t filename[22];
	uint8_t flag;
	uint8_t sum;
}STUSYStcp;

typedef struct _STU_SYSParamSet_
{
	uint8 aucDeviceID[7];				//终端的ID
	uint8 ucAPNLen;						//APN长度
	uint8 aucAPN[32];					//APN
	uint8 aucUser[32];					//M2M平台登录用户名
	uint8 ucUserLen;					//M2M平台登录用户名长度
	uint8 aucPassword[32];				//M2M平台登录密码
	uint8 ucPasswordLen;				//M2M平台登录密码长度
	uint8 aucSmsCenterNum[20];			//短信中心号码
	uint8 ucSmsCenterNumLen;			//短信中心号码长度
	uint8 aucHostIP[4];                 //主中心IP地址
	uint8 aucSpareHostIP[4];           	//副中心IP地址
	uint16 usHostPort;              	//主中心端口
	uint16 usSpareHostPort;             //副中心端口
	uint8  ucHostProtocolType;			//主中心承载协议类型：0：UDP协议,1: TCP协议
	uint8  ucSpareHostProtocolType;		//副中心承载协议类型：0：UDP协议,1: TCP协议
	uint32 uiHeartbeatInterval;			//心跳间隔，单位：秒0x0000-不发送心跳,默认心跳间隔为30秒
	uint8  ucMaxLandonRepeats;			//最大登录重复次数
	uint32 uiLandonFailedMinRepeatsInterval;//登录失败最小重试间隔
	uint32 uiLandonFailedMaxRepeatsInterval;//登录失败最大重试间隔
	uint16 usSmsRcvTimeout;				//短信接收超时时间，单位：秒
	uint16 usCanBrt;					//本地CAN总线波特率
	uint16 usCanFmt;					//本地CAN报文格式
	uint32 auiCanId[90];	            //CAN ID 过滤配置，4字节一组
	uint8  ucCanIdNum;					//can id 个数
	uint16 usSleepBeforSlot;           	//进入休眠时间,单位:s
	uint16 usSleepWakeSlot;				//休眠期间定时唤醒间隔，单位：分
	uint8  ucSSRepSlot;					//终端基本状态同步数据自动发送间隔单位：秒
	uint8  aucSim[6];					//SIM卡号，不足高位补0,
	uint8  ucHostDnLen;					//主中心域名长度
	uint8  aucHostDn[32];				//主中心域名
	uint8  ucSpareDnLen;				//副中心域名长度
	uint8  aucSpareDn[32];				//副中心域名
	uint8  aucDns[4];					//dns
	uint16 usHwVersion;					//硬件版本号，如V1.5表示为0x0105
	uint16 usMainPwrRateVol;			//外电源额定电压单位：0.1V
	uint16 usBatRateVol;				//终端电池额定电压单位：0.1V	
	
	uint8  ucCanErrTime;				//CAN故障判断时间
	uint8  ucCanOKTime;					//CAN恢复正常判断时间
	uint8  ucPwrOffTime;				//终端断电时间条件
	uint8  ucPwrOnTime;					//终端上电时间条件
	uint8  ucPwrLowVol;					//外部电源低电压报警阈值，单位：1%
	uint8  ucPwrLowTime;				//外部电源低电压报警的时间参数，单位：1s
	uint8  ucBatLowVol;					//内部电源低电压报警阈值，单位：1%
	uint8  ucBatLowTime;				//外部电源低电压报警的时间参数
	uint8  ucGpsAntErrTime;				//终端天线故障报警的时间参数，单位：1s
	uint8  ucGpsAntOKTime;				//终端天线故障报警的解除时间参数，单位：1s
	uint8  ucGpsModuleErrTime;			//终端GPS模块故障报警的时间参数
	uint8  ucGpsModuleOKTime;			//终端GPS模块故障报警解除的时间参数，单位：1s	
	uint8  ucSpeedOver;					//表示超速报警阈值，单位：1KM/H
    uint8  ucSpeedOverLastTime;			//超速报警的时间参数，单位：1s
    uint8  ucTransportCar;				//非自主移动（拖车）报警距离阈值，单位：1KM

	uint8  ucDeviceWorkTimeRepCfg;		//设备工作时间段统计配置参数
	uint8  ucWorkDataRepModel;			//工作参数（工况）数据单条上传模式	0x00：默认，等时间间隔上传
										//								    0x01：其他（如以某工况参数的变频函数为频率发送）
										//                                  0xFF：不以单条上传模式传输

	uint8  ucWorkDataRepInterval;		//工作参数（工况）传输参数。时间，单位：1秒
	uint8  ucPosiInforRepModel;			//位置信息单条上传模式
	uint8  ucPosiInforRepInterval;		//位置信息上传间隔
}STUSYSParamSet,*PSTUSYSParamSet;
//-----结构定义----------------------------------------------------------------


typedef struct _STU_SYSMsgBus_
{
    uint8 ucSrcDevice;	//消息来源设备  1=服务器GPRS；2=备用; 3=MCU模块; 
                        //4=GPS模块；5=采集模块;  6=设置程序；
    uint8 ucKind;       //消息类型
    uint8 ucPrior;      //消息优先级  (暂时保留 填0)
    uint8 ucResv;       //暂时保留 填0
    uint16 usSize;      //消息长度,该长度不包含上面4个参数及校验数据的长度，   长度可以为0，
    uint8* pMsgPacket;  //消息包指针
    uint8 ucCheck;      //消息校验值  采用累加和后取低字节方式校验，从ucSrcDevice开始累计至校验和之前

}STUSYSMsgBus, *PSTUSYSMsgBus;



typedef struct _STU_System_
{
    uint8 ucOnline;                 //上线标志,登录成功表示上线，0=未上线,1=上线
    uint8 ucDebugPrint;             //调试状状态  
									//Bit3	1：输出SYS相关调试信息，0：不输出
									//Bit2	1：输出MCU CAN相关调试信息，0：不输出
									//Bit1	1：输出GPS相关调试信息，0：不输出
									//Bit0	1：输出Modem相关调试信息，0：不输出
    uint8 ucShutDownTime;           //设备收到关机命令后等待关机的时间，定位5秒
    uint32 uiPingID;                //发送ping数据时索引号
    uint8  ucRstTime;               //当服务器或者短信下发复位命令时，需要延时20秒后复位系统，确保短信可以删除掉
    uint8 ucSerAddrChangeFlag;		//服务器地址(IP或域名、端口号)改变标志, 1=改变,0=未改变
    uint8 ucResetModem;				//复位GSM模块,1=复位, 0=不复位
	uint8 ucCorrectTimeOk;			//1:时间已经校准, 0:时间未校准

	uint32 uiAccOffTimer;           //ACC关闭后计时器 当时间>=主电休眠间隔时, 通知RTC整机断电10秒钟
	uint32 uiResetTimer;            //从GPS初始化后开始计时 时间>=24小时时, 通知RTC整机断电
    uint8 ucAccFlag;                //ACC由ON变为OFF标志位
	uint8 ucWDTState;				//看门狗状态,0xaa=开启,其他=未开启
	uint16 usCanFaultcodeTimer;     //车辆故障代码上报计数器
	
}STUSystem,*PSTUSystem;


typedef struct _STU_SYS_LEDState_
{
    uint8 ucLed_GPRS;  //GPRS工作状态 0-未上线,1-上线
	uint8 ucLed_GPS;   //GPS工作状态  0-未定位,1-定位
	uint8 ucLed_WiFi;  //WiFi工作状态
	uint8 ucLed_ETH;   //ETH工作状态
}STU_SYS_LEDState;

typedef struct _STU_ZX_Timer
{
    uint8 ucTCBSendFlag;
	uint16 usTCWSendTimer;
	uint16 usTCDSendTimer;
	uint32 uiSleepTotalTime;        //终端休眠总时间
	uint32 uiSleepCurrentTime;      //终端本次休眠时间
}STUZXTimer;

#if 0
typedef struct _STU_ZX_ECUControl_
{
    uint8 ucECURemoveBindingFlag;   //密码解绑标志 0-不需要解绑 1-需要解绑
	uint8 aucECURemoveBinding[4];   //解绑密码
    uint8 ucControlCode;            //命令控制                                      
                                    //B0: 0=不需要发送绑定/解绑命令;1=需要发送绑定/解绑命令;
                                    //B1: 0=ECU解绑; 1=ECU绑定
                                    //B2: 0=不需要发送锁车\解锁命令;1=需要发送锁车\解锁
                                    //B3: 0=解锁命令;1=执行锁车
    uint8 ucBindingFlag;            //获取ECU绑定状态:1-ECU绑定,0-ECU解绑
    uint8 ucECULockState;           //ECU反馈的锁车状态 0-未锁车,1-锁车                                    
    uint8 ucSendCANheartbeatFlag;   //向ECU发送CAN心跳标志 0-发送, 1-不发送
    uint8 ucSendLockCount;          //潍柴向ECU发送锁车次数 默认为3次
	uint8 ucEngineUpEnable;         //B0:杭州DB功能 0-关闭,1-开启 默认关闭;B1:北京DB功能 0-关闭,1-开启 默认关闭
	uint8 ucVinValidFlag;           //发动机VIN码控制字:0-vin平台设置关闭,1-vin平台设置激活 
	uint8 ucVinLen;                 //发动机VIN长度
	uint8 aucVin[VIN_BUFFER_SIZE];	
}STUZXECUControl;
#endif

//-----外部变量----------------------------------------------------------------
//extern STUSYSParamSet g_stuSYSParamSet;
//-----外部函数----------------------------------------------------------------
uint8_t Check_DataSum(uint8_t *padd,uint16_t len);  //累加和校验判定函数

//-----外部函数----------------------------------------------------------------
uint16 SYS_GPS_CommandAll_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
uint16 SYS_GetGPS_CollectData(uint8_t* PtrTxt);
void   SYS_ParamRead(void);
void   SYS_ParamWrite(void);
void   GoToUpdate(void);
void   SYS_PutDataQ(void* ptr);
void   SYS_TimerCount_NoDelay(void);
void   SYS_TimerCount_Delay(void);
void   TaskSYS(void *pdata);
void* pthread_TaskSysTimer_Function(void *data);
PSTUSYSParamSet SYS_GetParam(void);     //获取设备参数的接口函数
void ClearNoUploadGprsTimer(void);
uint8 IsCorrectTimeOk(void);
void SetCorrectTimeOk(void);
uint16 RestoreTranslateData(uint8 *src,uint16 srcLen);
uint16 TranslateData( uint8* src,uint16 srcLen);	
void SYS_ResetSleeptime(void);
void SYS_SendNetData(void);
#endif
