/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GsmIntegrateLayer.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块综合层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by  创建本文件
 *
 */

#ifndef GSMINTERGRATELAYER_H_20110609_5B055A0D_2784_442E_A1B3_4E9B7914E18B
#define GSMINTERGRATELAYER_H_20110609_5B055A0D_2784_442E_A1B3_4E9B7914E18B

#define  GSM_SERVICE_NUMBER   	10    //gsm 服务列表的长度
//-----常量定义----------------------------------------------------------------
#define SMS_SER_NONE 	0//无sms 服务
#define SMS_SER_IDLE 	1//sms服务空闲,可以发送sms
#define SMS_SER_BUSY 	2//sms服务正忙,不能发送sms 




//-----结构定义----------------------------------------------------------------
//GSM 模块操作结构体
typedef struct _STU_GSM_OPERATE_
{
	uint8 GsmEnable;						//GSM模块使能,0=禁止,1=使能
	uint8 GsmOperate;						//GSM操作，0=空闲,1=开机,2=关机
	uint8 GsmSleepOperate;					//休眠操作,0=空闲,1=休眠,2=唤醒

	uint8 LinkOperate[3];					//3个链接的操作,0=空闲,1=连接或保持,2=断开,3=重连
	uint8 FtpOperate;						//ftp操作, 0=空闲,1=连接或保持,2=请求数据,3=断开
	uint16 usFtpGetLen;						//请求ftp数据的长度
}STU_GsmOperate;

#if 0
//拨打电话结构体
typedef struct _STU_DIAL_NUMBER_
{
	uint8 Request;						//0:无拨号请求,1:有拨号请求
	uint8 Number[15];					//电话号码
	uint8 Len;							//号码长度
	uint8 State;						//拨号状态  0:空闲,1=正在拨号,2=已接通
}STU_DialNumber;

//接听电话结构体
typedef struct _STU_ANSER_NUMBER_
{
	uint8 IncomeFlag;					//来电标志,0:无来电,1:有来电
	uint8 Number[15];					//电话号码
	uint8 Len;							//号码长度
	uint8 State;						//接听状态  0:空闲,1:正在接听,2=已挂断,
}STU_AnserNumber;
#endif
//SMS 结构体
typedef struct _STU_SMS_
{
	uint8 SmsSerState;	//SMS 服务状态, SMS_SER_IDLE, SMS_SER_BUSY, SMS_SER_NONE 
	uint8 Index;		//当前正在发送的sms在发送缓存中的索引.
	uint8 SendState;	//短信发送状态, 1=已发送, 但未成功;2=已发送成功
}STU_Sms;

//语音播报结构体
typedef struct _STU_TTS_
{
	uint8 BusyFlag;		//忙标志, 1=正在播报, 0=空闲
	uint32 PlayFlag;	//播报标志, bit0~bit31分别对应第1句至第32句, 1=播报对应语句, 0=不播报
	uint8 PlayIndex;	//当前正在播报的语句的序号,取值0~31
	uint16 PlayOverTimer;//等待播放结束计时器
} STU_TTS;

#define GSMSERVICE_TYPE_OPEN				1
#define GSMSERVICE_TYPE_CLOSE				2
#define GSMSERVICE_TYPE_RESET				3
#define GSMSERVICE_TYPE_WAKE				4
#define GSMSERVICE_TYPE_SLEEP				5
#define GSMSERVICE_TYPE_OPENLINK			6
#define GSMSERVICE_TYPE_CLOSELINK			7
#define GSMSERVICE_TYPE_SENDGPRS			8
#define GSMSERVICE_TYPE_SENDSMS				9
#define GSMSERVICE_TYPE_RCVSMS				10
#define GSMSERVICE_TYPE_DIALPHONENUMBER		11
#define GSMSERVICE_TYPE_ANSERPHONENUMBER	12
#define GSMSERVICE_TYPE_QUERYSTATE			13
//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------

void QueryGsmState(void);

#endif

































