/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: SystemProtocol.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为System功能模块协议层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16, by lxf, 创建本文件
 *
 */

#ifndef _SYSTEMPROTOCOL_H
#define _SYSTEMPROTOCOL_H

//-----常量定义----------------------------------------------------------------
#define	PROTOCOL_VERSION		0x01	//使用的协议版本号		
#define	SUPPLY_CODE				0x00	//供应商编号
#define	TERMINAL_TYPE			0x00	//终端型号
#define CUSTOMER_CODE			0X00	//使用方编号


#define CONNECT_MAX_REPEATS			2	//工作参数上传最大重复次数
#define CONNECT_RESPOND_TIMEOUT		8	//工作参数上传后等待响应超时的最长时间,单位:s
//向平台发送登录指令管理的结构体
typedef struct _Connect{
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器
	uint8  ucSucceedFlag;				//登录成功标志,1=成功,0=失败
}stuConnect,*pstuConnect;

//向平台发送状态同步数据
#define SSDATA_MAX_REPEATS		3	//基本工作数据上传最大重复次数
#define SSDATA_RESPOND_TIMEOUT	10	//基本工作数据上传后等待响应超时的最长时间,单位:s
#define SSDATA_REP_INTERVAL		30	//基本工作数据上报间隔
typedef struct _STU_SSData_
{
    uint16 usTimer;						//上报间隔计时器
    uint8 ucTimeToRepFlag;				//到了上报的时间标志,1=有效
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器
	uint16 usRepSn;						//上报的流水号
}STU_SSData;


#define MAX_NO_UPLOAD_GPRS_TIME		30	//没有上传任何GPRS数据的最长时间	
#define HEARTBEAT_MAX_REPEATS		2	//心跳上传最大重复次数
#define HEARTBEAT_RESPOND_TIMEOUT	8	//心跳参数上传后等待响应超时的最长时间,单位:s
//向平台发送心跳的结构体
typedef struct _STU_HeartBeat_
{
    uint16 usNoUploadGprsTimer;     	//没有向平台发送任何数据计时
    uint16 usTimer;						//上报间隔计时器
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器
}stuHeartBeat;

#define WORKDATAREP_MAX_REPEATS		2	//工作参数上传最大重复次数
#define WORKDATAREP_RESPOND_TIMEOUT	8	//工作参数上传后等待响应超时的最长时间,单位:s
//向平台上传设备工作参数管理的结构体
typedef struct _WorkDataRep{
	uint16 usTimer;						//上报间隔计时器
	uint8 ucTrackModel;					//0x00：等时间间隔追踪
										//0xFF：单次查询

	uint8 ucTrackInterval;				//上传间隔：
										//追踪模式为0x00时，表示时间，单位：1秒
										//追踪模式为：0xFF时，该值无效。

	uint16 usTrackScope;				//上传有效时间：
										//追踪模式为0x00时，表示时间，单位：1s
										//该数值为0x00时，表示停止关注。
										//追踪模式为0xFF，该值无效。
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器									
	uint16 usRepSn;						//上报的流水号									
}stuWorkDataRep,*pstuWorkDataRep;

//向平台上传位置信息管理的结构体
typedef struct _PositionRep{
	uint16 usTimer;						//上报间隔计时器
	uint8  ucTrackModel;				//追踪模式,0x00:等时间间隔追踪,0x01:等距离间隔追踪,0xFF:单次追踪
	uint8  ucTrackInterval;				//追踪命令设置的追踪间隔,
										//追踪模式为0x00时,表示时间，单位：1秒
										//追踪模式为0x01时,表示距离，单位：0.1千米
										//追踪模式为0xFF时,该值无效。
	uint16 usTrackScope;					//追踪有效区间：
										//追踪模式为0x00时，表示时间，单位：1s
										//追踪模式为0x01时，表示距离，单位：1千米
										//追踪模式为0xFF，该值无效
										//该数值为0x00时，表示关闭追踪功能。
}stuPositionRep,*pstuPositionRep;

#define ALARM_MAX_REPEATS			2	//报警上传最大重复次数
#define ALARM_RESPOND_TIMEOUT		8	//报警上传后等待响应超时的最长时间,单位:s
#define ALARM_MAX_NUM				20	//报警个数
//向平台上传报警信息
typedef struct _Alarm{
	uint8 ucType;						//报警类型
	uint8 ucState;						//上报状态,0=未上报，1=已上报
	//uint8 aucData[22];					//报警携带的数据
}stuAlarm,*pstuAlarm;

typedef struct _AlarmRep{
	stuAlarm Alarm[ALARM_MAX_NUM];
	uint8 ucRepIndex;					//当前正在上报的报警索引,0xff表示没有需要上报的报警
	uint8 ucRepeats;					//重复上报次数
	uint8 ucRepFlag;					//已上报标志,1=已上报
	uint8 ucRespondTimer;				//等待平台回复计时器
	uint8 ucNewAlarmFlag;				//新的报警事情发生,1=有,0=无
	uint16 usRepSn;						//上报的流水号
	
}stuAlarmRep,*pstuAlarmRep;

#define DTC_MAX_REPEATS			2	//DTC上传最大重复次数
#define DTC_RESPOND_TIMEOUT		8	//DTC上传后等待响应超时的最长时间,单位:s
typedef struct _DTCRep{
	uint8 ucRepeats;					//重复上报次数
	uint8 ucRepFlag;					//已上报标志,1=已上报
	uint8 ucRespondTimer;				//等待平台回复计时器
	uint16 usRepSn;						//上报的流水号
	
}stuDTCRep,*pstuDTCRep;

#define ACCSTATE_MAX_REPEATS		2	//工作参数上传最大重复次数
#define ACCSTATE_RESPOND_TIMEOUT	8	//工作参数上传后等待响应超时的最长时间,单位:s

//开关机信息上报结构体
typedef struct _AccRep{
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=等待上报,2=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器
	uint8  ucSucceedFlag;				//休眠指令发送成功标志,1=成功,0=失败
	uint8 aucData[23];					//上报的数据
}stuAccRep,pstuAccRep;


//开关机信息上报结构体
typedef struct _McuResp{
	uint8  ucRespondTimer;				//等待平台回复计时器

}stuMcuResp,pstuMcuResp;

//休眠指令上报结构体
typedef struct _SleepRep{
	uint8  ucRepeats;					//重复上报次数
	uint8  ucRepFlag;					//已上报标志,1=已上报
	uint8  ucRespondTimer;				//等待平台回复计时器
	uint8  ucSucceedFlag;				//休眠指令发送成功标志,1=成功,0=失败
}stuSleepRep,pstuSleepRep;

//固件升级结构体
#define FIRMWARE_PACKET_LEN			1024//固件升级包的长度
typedef struct _FirmwareUpdate{
	uint8 uctype;            //升级方式 0-询问升级,1-强制升级
	uint8 ucdev;			//升级的目标设备0x00:终端,0x01:控制器,0x02:显示器,0x03:协处理器
	uint8 aucSerIp[4];		//服务器IP地址
	uint16 usSerPort;		//服务器端口号
	uint8 ucSerProtocolType;	//协议类型
	uint8 ucSWNameLen;		//升级固件名称长度
	uint8 aucSWName[50];	//升级固件名称
	uint8 ucSWVersionLen;	//升级固件版本号长度
	uint8 aucSWVersion[10];	//升级固件版本号
	uint32 uiSWSize;		//升级文件的大小
	uint32 uiCrc;			//整个升级文件CRC32校验码
	uint16 usPackets;		//升级文件总包数
	uint16 usRequestPacketSn;//请求的包序号
	uint16 usLastPacketLen;		//最后一包数据的长度
	uint16 usTimeoutCnt;	//升级超时计数器,如果10分钟内没有升级完成则放弃
	uint8 ucRepeats;		//发送命令重复次数
	uint8 ucStep;			/*升级步骤，0=空闲,
	                                    1=收到升级通知正连接到升级服务器,
	                                    2=连接升级服务器成功,请求升级,
	                                    3=请求升级已发送,等待服务器响应
	                                    4=请求升级成功,请求下载升级包,
	                                    5=请求下载升级包命令已发送,等待服务器响应
	                                    6=升级包下载完成,正向平台上报下包结果,
	                                    7=向平台上报下包结果命令已发送,等待平台响应
	                        */
	uint8 ucRet;			/*下包结果0=成功,1=失败*/                        
	uint8 ucUpgradeRequestTimer;		//升级请求超时计时器
	uint8 ucUpgradeRequesRepeats;		//升级请求重复次数
	uint8 ucPacketRequestTimer;			//下载升级包请求超时计时器
	uint8 ucPacketRequesRepeats;		//下载升级包请求重复次数
	uint8 ucSendRetTimer;				//发送升级结果超时计时器
	uint8 ucSendRetRepeats;				//发送升级结果重复次数
}stuFirmwareUpdate;



typedef struct _SerAddr{
	uint8 aucIp[4];
	uint16 usPort;
	uint8  ucDnLen;
	uint8  aucDn[32];
	uint8  ucProtocolType;
}stuSerAddr;


uint8 BuildMsgHead(uint8 *buf,uint8 ucMsgType,uint16 usMsgBodyLen, uint8 ucFlag, uint16 usSn);
uint16 BuildSS(uint8 *buf);
void SYS_Reset(uint8 delay);
uint32 BuildState(void);
void SYS_POWERLED_Display(void);
void SYS_PowerOff_Reset(void);
void Sys_GPSState_Change(void);
uint16 Build_ZX_TCD(uint8 *buf);

#endif








