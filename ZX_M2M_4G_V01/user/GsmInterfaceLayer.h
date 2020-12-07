/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GsmInterfaceLayer.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块接口层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by, 创建本文件
 *
 */

#ifndef GSMINTERFACE_H_20110609_7ABB65AE_41C0_48D1_912F_B2DD60D08089
#define GSMINTERFACE_H_20110609_7ABB65AE_41C0_48D1_912F_B2DD60D08089

#include "GsmProtocolLayer.h"  //解决编译不通过问题,以后再想办法拿掉

//-----常量定义----------------------------------------------------------------


#define SMS_NUMBER_TYPE_MAS					1	//短信号码类型--MAS机
#define SMS_NUMBER_TYPE_MOBILE				0	//短信号码类型--普通手机号
#define SMS_NUMBER_LEN_MAS					15	//短信号码长度--MAS机
#define SMS_NUMBER_LEN_MOBILE				11	//短信号码长度--普通手机号

//-----结构定义----------------------------------------------------------------
typedef struct _GPRS_SEND_DATA_QUEUE_
{
	uint8 ucLinkNum;
	uint16 usLen;
	uint8 buff[GPRS_SEND_BUFF_LEN];
	uint8 priority;//优先级:1=最高优先级,2=次高
	uint8 flag;  //是否发送完成, 0=已完成, 1=待发送
}STU_GprsSendDataBuff;

typedef struct _STU_SMS_SEND
{
	uint8 aucDesNumber[15];
	uint8 ucNumberLen;
	uint8 aucSmsSendBuff[SMS_SEND_BUFF_LEN];//发送短信数据缓存
	uint16 usDataLen;
	uint8 flag;  //是否发送完成, 0=已完成, 1=待发送, 2=已发送,但未完成
}STU_SmsSend;

typedef struct _STU_SMS_RCV
{
	uint8 ucNumberLen;
	uint8 aucSrcNumber[15];
	uint16 usDataLen;
	uint8 aucSmsRcvBuff[SMS_SEND_BUFF_LEN];//发送短信数据缓存
	uint8 flag;			//该短信处理完毕,可以容许再次读短信并保存在这里, 0=处理完毕, 1=待处理
}STU_SmsRcv;
//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------

void   InitGsmModule(void);
void   InitSimSeq(void);


void   SendSms(void);
void   RecvSms(void);

void   SendGprs(void);
void   RecvGprs(void);
void   Connect(void);




#endif


