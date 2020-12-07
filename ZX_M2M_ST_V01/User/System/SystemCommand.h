/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: SystemCommand.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为System功能模块协议层数据解析的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, 创建本文件
 *
 */

#ifndef _SYSTEMCOMMAND_H
#define _SYSTEMCOMMAND_H
//-----常量定义----------------------------------------------------------------
#define  MSG_HEADER_ALLDATA_LEN             16         	//协议消息头长度为16
#define  MSG_HEADER_DATA_LEN                12         	//协议消息头总长度-GPS命令ID和命令长度(4字节)
/********************************上行命令ID********************************************/
#define UP_CMD_ID_GETSERVERPARA				0x80		//获取中心服务器通信参数指令
#define UP_CMD_ID_UPLOADHEARTBEAT			0x81		//终端心跳通讯指令
#define UP_CMD_ID_RESPSERDEFCMD				0x82		//终端命令应答指令
#define UP_CMD_ID_UPLOADPOSITIONINFOR		0x83		//上传位置信息指令
#define UP_CMD_ID_UPLOADPARA				0x84		//上传终端参数指令
#define UP_CMD_ID_LANDON					0x85		//登录指令
#define UP_CMD_ID_UPLOADWORKDATA			0x87		//上传工作参数指令
#define UP_CMD_ID_UPLOADSLEEPNOTE			0x89		//发送休眠提示指令
#define UP_CMD_ID_UPLOADALARMDATA			0x8A		//上传报警指令
#define UP_CMD_ID_UPLOADACCSTATE			0x8D		//上传开关机信息
/**************************************************************************************/

/********************************消息类型定义********************************************/
#define MSG_TYPE_MESSAGEACK					0x00		//通用报文响应
#define MSG_TYPE_CONN_REQ					0x01		//连接请求
#define MSG_TYPE_CONN_RESP					0x02		//连接响应
#define MSG_TYPE_PUSH_DATA					0x03		//终端向服务端发送数据或服务端向终端发送数据
#define MSG_TYPE_ALERT						0x04		//终端向服务端发送提醒、告警等特殊消息
#define MSG_TYPE_CMD_REQ					0x05		//终端向服务端发送命令请求，或服务端向终端发送命令请求
#define MSG_TYPE_CMD_RESP					0x06		//接收端对命令的响应
#define MSG_TYPE_PING_REQ					0x07		//终端对服务端发送的心跳请求
#define MSG_TYPE_PING_RESP					0x08		//服务端对终端心跳的响应
#define MSG_TYPE_DISCONNECT					0x09		//终端断开连接
#define MSG_TYPE_UPDATE						0x0A		//升级通知
#define MSG_TYPE_UPDATE_ACK					0x0B		//升级响应
#define MSG_TYPE_REGIST_REQ					0x0C		//注册请求
#define MSG_TYPE_REGIST_RESP				0x0D		//注册响应
#define MSG_TYPE_DEREG_REQ					0x0E		//注销请求
#define MSG_TYPE_DEREG_RESP					0x0F		//注销响应
/********************************下行命令ID********************************************/
#define DOWN_CMD_ID_SERVERPARA				0x01		//配置中心服务器通信参数指令
#define DOWN_CMD_ID_RESPGPSDEFCMD			0x02		//中心命令应答指令
#define DOWN_CMD_ID_GETPOSITIONINFOR		0x03		//定位指令
#define DOWN_CMD_ID_SETPARA					0x04		//设定参数
#define DOWN_CMD_ID_FIRMWAREUPDATE			0x06		//升级固件指令
#define DOWN_CMD_ID_QUERYPARA				0x07		//查询参数
#define DOWN_CMD_ID_QUERYWORKDATA			0x08		//查询工作参数
#define DOWN_CMD_ID_SETWORKDATAREP			0x09		//工作参数上传设置
#define DOWN_CMD_ID_ATTENTION				0x0c		//关注终端位置/工况指令
#define DOWN_CMD_ID_TRACK					0x0d		//追踪指令
#define DOWN_CMD_ID_CTRL					0x0f		//终端控制
/**************************************************************************************/
#define  SYS_SHUTDOWN_TIME                  10         	//收到关机命令后10秒后自动关机
#define  SYS_GPSLOCATION_DATA_LEN           48

#define MSG_HEAD_LEN						13			//消息头长度
typedef struct _STU_SYSPactHeader_
{
   uint16 usMsgLen;			//消息剩余长度,即报文体和校验字的总长度。剩余长度不包含用于编码的剩余长度字段本身的字节数。
   uint8 aucId[7];			//终端ID
   uint8 ucMsgType;			//消息类型
   uint8 ucFlag;			//厂家编号
   uint16 usSq;				//命令序号
   uint16 usMsgBodyLen;		//消息体长度,为消息长度
   uint8 ucSrc;				//消息来源
}STUSYSPactHeader,*PSTUSYSPactHeader;



//对于一些需要计时的变量统一管理
typedef struct _SYS_Counter_
{
	uint16 usDailyReportCounter;	//上传日报信息间隔计时
	uint16 usAlarmReportCounter;	//上传报警信息间隔计时
}STU_SYSCounter;

typedef struct _UnNomalGSMData
{
	uint8 aucUnpackData1[250];
	uint8 ucLen1;
	uint8 aucUnpackData2[250];//[0]:数据长度
	uint8 ucLen2;
	uint8 ucDealFlag;	//b0=1第一包处理完成，b1=1:第二包处理完成
}STUUnNomalData;

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
uint16 SYS_GPS_CommandAll_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr, uint8 ucKind);
//uint16 SYS_CollectModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
uint16 SYS_GPSModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
//uint16 SYS_MCUModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
//uint16 SYS_SMS_CommandAll_Execution_Universal(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
void SYS_ParamRead(void);
uint16 EscapeMassage( uint8* src,uint16 srcLen);
void DealUnNormData(void);
void SaveEcuPara(void);
void ReadEcuPara(void);
void ReadEcuFaultAddrFromFlash(void);
void SerAddrChangeGsmRst(void);
void UpgradeManage(void);
void UpgradeTimer(void);
uint16 DealSerCmd(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
#endif




























