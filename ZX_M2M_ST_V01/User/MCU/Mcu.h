//文件名：Mcu_Can.h
//功能：  敝人自己函数生成的GPIO别名地址的宏定义
#ifndef _MCU_H_
#define _MCU_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "McuHW.h"


//#define CAN1								0	//
//#define CAN2								1


#define	TASK_MCU_CAN_ID						6
#define TASK_MCU_CAN_PRIO       			6
#define TASK_MCU_CAN_STK_SIZE				500

#define MCU_DATA_LENGTH						1460 //普通MCU数据的长度, MCU增加了控制器、显示器的版本等信息2013-05-03
#define TIME_COMM_UNNORMAL          		120	 //通信异常判定时间

#define MCU_STATE_NORMAL					0	//与MCU通信正常
#define MCU_STATE_UNNORMAL					1	//与MCU通信异常

//#define MCU_CAN_BRT		                    BPS_500K//CAN通信的波特率
#define MAX_CAN_FRAME_NUM					80	//容许接收的最大CAN帧数量
#define MAX_ACTIVE_FAULTCODE_NUM		    10	//可以接收的最多激活故障码数量
#define TIME_FAULTCODE_ACTIVE			    30	//故障码激活时间，超过此时间故障码没有出现则认为该故障消失
#define CAN_ECM_ADDR						0x00//发动机地址
#define CAN_CONTROLER_ADDR					0x31//控制器地址
/************************MCU模块消息类型定义说明**********************************************/
#define MCU_MSG_KIND_CAN1COMM_ERR           1	//can通信异常
#define MCU_MSG_KIND_CAN1COMM_OK            2	//can通信正常
#define MCU_MSG_KIND_CAN2COMM_ERR           3	//can通信异常
#define MCU_MSG_KIND_CAN2COMM_OK            4	//can通信正常
#define MCU_MSG_KIND_MCUDATA_CHANGE			5	//MCU数据变化
#define MCU_MSG_KIND_CAN1_RCV_STOP			6	//CAN1接收停止
#define MCU_MSG_KIND_CAN1_RCV_START			7	//CAN1接收开始
#define MCU_MSG_KIND_CAN2_RCV_STOP			8	//CAN2接收停止
#define MCU_MSG_KIND_CAN2_RCV_START			9	//CAN2接收开始


#define MCU_ALARM_CNT 						12
#define MAX_CAN1_RCV_ERR_TIME				3000//can1接收无数据超时时间,单位:10ms
#define MAX_CAN2_RCV_ERR_TIME				1000 //can1接收无数据超时时间,单位:10ms

#define MCU_GPSCMD_SEND_TIME                10
#define MCU_GPSSTATE_SEND_TIME              50
#define MAX_CAN_FaultCode_NUM				15 //容许接收的故障码最大CAN帧数量


typedef struct _STU_CommState{
	uint8  ucCan1RcvErr;					//Can1接收数据错误,0:没有收到数据,1:收到数据
	uint16 usCan1RcvErrTime;				//Can1没有收到数据计时	
	uint8  ucCan1CommState;					//Can1接收数据状态通信状态，1:通信异常,0:通信正常
	uint8  ucRcvCan1DataFlag;				//Can1接收数据状态 1:收到了CAN1数据，2:没有收到CAN1数据

	uint8  ucCan2RcvErr;					//Can2接收数据错误,0:没有收到数据,1:收到数据
	uint16 usCan2RcvErrTime;				//Can2没有收到数据计时	
	uint8  ucCan2CommState;					//Can2通信状态，1:通信异常,0:通信正常
	uint8  ucRcvCan2DataFlag;				//Can2接收数据状态 1:收到了CAN1数据，2:没有收到CAN1数据

	uint8  ucSleepState;					//休眠状态,0=未休眠, 1=休眠

}STU_CommState,*PSTU_CommState;	

typedef struct _STU_McuCmd{
	uint8 ucLock;							//收到锁车命令, 0xaa:收到锁车/解锁命令, 其他:没有收到
	uint8 ucLockStage;						//锁车级别:  0:解锁, 1~3:锁车级别
    uint16 usSpeedLimit;					//锁车/解锁:0=禁止启动, 28000:解绑,1~27999速度限制
	uint8 ucBindCmd;						//绑定命令,0:无任何命令, 0xaa:绑定, 0x55:解绑
	uint8 ucEcuFeedBackBindState;			//ECU反馈的绑定状态:1=绑定, 0=解除绑定
	uint8 ucEcuFeedBackLockState;			//ECU反馈的锁车状态:1=锁车, 0=解锁

    uint8 LockControlFlag;              //锁车与监控模式控制 20100826 
                                        //B0 : 0=不需要发送监控模式命令;1=需要发送监控模式
                                        //B1 : 0=监控模式关闭; 1=监控模式打开
                                        //B2 : 0=不需要发送一级锁车命令;1=需要发送一级锁车
                                        //B3 : 0=取消一级锁车命令;1=执行一级锁车
                                        //B4 : 0=不需要发送二级锁车命令;1=需要发送二级锁车
                                        //B5 : 0=取消二级锁车命令;1=执行二级锁车
    uint8 ucRespSerFlag;	//响应平台标志,
                                        //B0=1:需要上报平台一级锁车成功,
                                        //B1=1:需要上报平台二级锁车成功,  
                                        //B2=1:需要上报平台一级解锁成功, 
                                        //B3=1:需要上报平台二级解锁成功,
                                        //B4=1:需要上报平台开启监控模式成功,
                                        //B5=1:需要上报平台关闭监控模式成功,
                            
   	uint16 usLockOneSq;		   //平台下发的一级锁车流水号
	uint16 usLockSecSq;		   //平台下发的二级锁车流水号
	uint16 usUnLockSq;		   //平台下发的解锁流水号
	uint16 usMonitorSq;		   //平台下发的设置监控模式流水号

	uint8  ucSimQueryFlag;	   //MCU查询SIM卡  1=收到查询命令
	uint16 usSendMCULockTimer; //发送MCU锁车命令计时器
	uint16 usSendGPSDataTimer; //定时发送GPS定位数据计时器
	uint32 uiFs;               //校验得到的结果,需要备份
	uint32 uiKey;              //key值  默认为 0x18FE0AF4
	uint8  ucCmdCheckFlag;     //MCU回复命令中校验结果标志,51-成功;52-失败

}STU_McuCmd,*PSTU_McuCmd;

typedef struct _STU_CanFrame{
	uint32 id;
	uint8 aucData[8];
}STU_CanFrame;

typedef struct _faultCode{
	uint32 faultCode;						// 4字节故障码(暂时忽略OC和CM)
	uint8 activeTimes;						// 故障码激活时间计时
}faultCode,*pFaultCode;


typedef struct _XWFaultCode
{
    uint8 ucFaultCodeNum;      //故障码帧总个数
    STU_CanFrame CanData[MAX_CAN_FaultCode_NUM];//CAN数据缓存 
  //  uint8 ucRecvOverFlag;      //0-未接收或者接收未完成,1-接收完成
    uint8 ucVehicleType;         //0-默认值,不接收故障码;1-小于450的车辆类型(采用0x18FE27F3);2-大于等于450的车辆类型(采用0x18FE25F3)
                                 //
}stuXWFaultCode,*pstuXWFaultCode;

typedef struct _STU_MCUSend_GPSData_
{
    uint8 aGpsstate[8];
	uint8 aGpsData[8];
	uint8 aControlData[2];
	uint8 ucCount;
	
}STU_MCUSend_GPSData;

/************************end of md5****************************************************************/


void   McuInit(void);
BOOL CanFrameFilter(uint32 id);
void   DealCan1Message(MessageDetail msg);
void   SendCan1Message(void);
//void   McuReadFromMemory(void);
//void   McuSaveToMemory(void);
void   MCUCommStateJudge(void);
uint16 ConfirmMcuMsg(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr);
void  MCU_TimerCount_NoDelay(void);
void  MCU_TimerCount_Delay(void);
uint8 GetCanCommState(uint8 channel);
uint8 GetCanRcvState(uint8 channel);
uint8 GetMcuFaultCode(uint8 * faultCodeData);
uint8 IsNewMcuFaultCode(void);
void ClearNewMcuFaultCodeFlag(void);	
void Mcu_CanOpen(void);
void Mcu_CanClose(void);
uint8 Mcu_GetCanSleepState(void);
void AddCanFrameToBuff(uint32 id, uint8 *data);
BOOL MCU_GetVoltageState(void);
uint8 MCU_SendNewLockCmd(uint8 *ptr);
void TaskMCU(void *pdata);
#ifdef  __cplusplus
}
#endif

#endif
