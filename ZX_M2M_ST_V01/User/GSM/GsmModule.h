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
 * 2019-05-09 by , 创建本文件
 *
 */

#ifndef GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138
#define GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138

#include "GsmHardWareLayer.h"
//-----常量定义----------------------------------------------------------------
#define TASK_GSM_RECV_STK_SIZE        300  //GSM模块接收任务栈大小
#define TASK_GSM_RECV_ID              4    //GSM模块接收任务ID
#define TASK_GSM_RECV_PRIO            4    //GSM模块接收任务优先级

#define TASK_GSM_SERVICE_STK_SIZE     100  //GSM模块主任务栈大小
#define TASK_GSM_SERVICE_ID           7    //GSM模块主任务ID
#define TASK_GSM_SERVICE_PRIO         7    //GSM模块主任务优先级

#define SIMCARD_Satate_Timer                 180    //判断SIM卡延时时间
#define Recv_4GMODULE_TIMEOUT                180    //接收主处理器串口信息最大超时时间

#define SEND_TO_A5_TIMER                     10
#define SEND_TO_A5_BUFF_LEN                  1024
#define SEND_TO_A5_BUFF_NUM                  5      //CAN存储缓存数
#define CAN_FRAME_MAX_NUM                    72     //没个CAN缓存可以存放的最大帧数
#define CAN_SEND_MIN_TIMER                   125    //向4G模块发送最小间隔
#define CAN_SEND_MAX_COUNTER                 8      //1秒发送最大包数


typedef struct _STU_A5Comm_
{
    uint8 ucSendTimer;
	uint8 aSenDataBuffFlag[SEND_TO_A5_BUFF_NUM];  //0-未有待发数据,1-接收完成,2-正在接收CAN数据,0xFF-接收超时被发送抢断
    uint8 aCANDatabuff[SEND_TO_A5_BUFF_NUM][SEND_TO_A5_BUFF_LEN];  //B0-Tag,B1-B2=Len,V=CAN数据 1byteCan帧个数 一个缓存最大可以存储72个CAN帧
    uint8 ucSN;                //流水号
	uint16 usmSec;             //数据说明：B15-帧格式0-扩展帧,1-标准帧B14-B11：帧数据长度B10-B0：偏移时间(毫秒)
    uint8 ucCommflag;          //核心板与MCU通讯状态标志 0-未通讯 ，1-通讯
    uint8 ucCommErrTime;       //通讯中断计时器 初始0秒
}STU_A5Comm;

//日期时间结构体
typedef struct _STU_A5_Date_
{
  uint8  ucYear;             //年
  uint8  ucMon;              //月
  uint8  ucDay;              //日
  uint8  ucHour;             //时
  uint8  ucMin;              //分
  uint8  ucSec;              //秒
  uint16 usmSec;             //毫秒
  uint16 usmSectemp;         //毫秒临时变量
}STU_A5_Date, *PSTU_A5_Date;

typedef struct _STU_GSM_State_
{
    uint8 ucRunState;          //4G 模块开机状态 0-未开机,1-已做开机操作,2-开机成功,3-模块串口通信异常,
                                                //4-模块休眠,5-已进入休眠
    uint8 ucRecvTimeErr;       //串口接收4G模块数据中断计时器
    
}STU_GSM_State;

typedef struct _STU_SYS_LEDState_
{
    uint8 ucLed_GPRS;  //GPRS工作状态 0-未上线,1-上线
	uint8 ucLed_GPS;   //GPS工作状态  0-未定位,1-定位
	uint8 ucLed_WiFi;  //WiFi工作状态
	uint8 ucLed_ETH;   //ETH工作状态
	uint8 ucreserve;   //保留
}STU_SYS_LEDState;

void TaskUart3Recv(void *pvData);
void TaskGsmService(void * pvData);
void A5_MCU_SendReqCmdData(uint8 ucCommand, uint8 ucSN, uint8 ucResult,uint8 ucSrc);
void A5_Deal_RecVA5_Data(uint8 *ptr, uint16 uslen);
void A5_MCUSendCANDataToA5(void);
void A5_AddCanFrameToBuff(uint32 id, uint8 ucFF, uint8 ucDataLen, uint8 *data);
void AccountSysDate(void);
void A5_CommErr_Judge(void);
void ModemClose(void);
#endif





























