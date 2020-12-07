/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: SystemWorkMode.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为System功能模块工作模式处理文件的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, 创建本文件
 *
 */

#ifndef _SYSTEMWORKMODE_H
#define _SYSTEMWORKMODE_H


//-----常量定义----------------------------------------------------------------

//设别工作模式 目前共有4中模式

#define WORK_MODEL_NORMAL           0     //正常工作
#define WORK_MODEL_SLEEP            1     //此时主CPU进入掉电模式,GPS模块和GSM模块均休眠
//#define WORK_MODEL_POWEROFF         2     //检测到挖机蓄电瓶馈电时，PIC切断设备外部电源
//#define WORK_MODEL_MOVE             3     //ACC及保留开关关闭后车辆速度>30KM/h

#define WEAKLAST_TIME              300    //休眠唤醒后持续运行时间5分钟
#define WORK_HEART_TIME            20     //20秒
#define TRANSITMODESPEED           300    //判定为运输模式的速度下限单位0.1Km/h

//-----结构定义----------------------------------------------------------------

typedef struct _STU_Sleep_
{
   uint32 uiSleepWakenSlot;            //休眠唤醒间隔,单位为秒，每隔m分钟后醒来发送数据，然后等待n分钟后继续休眠
   uint16 usSleepBeforSlot;            //休眠前时间间隔,单位为秒，switch1低电平n分钟后休眠
   uint16 usWakeLastTime;              //休眠唤醒后持续在线,单位为秒，持续时间5分钟
//  uint8  ucBtwnGpsGsmTime;            //唤醒GPS到再唤醒GSM之间的间隔时间
   uint8  ucFlag;                      //B0 保留;
                                      //B1=1  唤醒后持续运行5分钟计时到,需要重新进入休眠
                                      //B2=1  休眠前时间变为0开始休眠,
                                      
   uint8  ucSpecialFlag;               //特殊情况下判定是否该休眠或者唤醒的标志
                                      //B0=1 车机报警时候不允许进入到休眠
                                      //B1=1 车机报警时候处在休眠期间立即唤醒
   uint32 usSleepTimes;				  //休眠后，终端需要睡眠的时间。 
    uint8 ucWorkingMode;            //0=正常、1=休眠、2=掉电  3 = 运输 
    uint8 ucGsmSleepState;          //GSM模块休眠状态:0=不休眠;1=休眠
    uint8 ucGpsSleepState;          //GPS模块休眠状态:0=不休眠;1=休眠
    uint8 ucWork_Heart_Count;       //初始值为20  为0 对变量时Work_heart_Flag=1,表示可以进入休眠或者下一模式 
    uint8 ucWork_heart_Flag;        //1=表示需要向服务器发送模式切换的定位数据;0=不要发送 
    uint8 ucReadRTCClockFlag;       //终端唤醒后读取RTC时钟标志,0-不读取,1-读取

}STUSleep,*PSTUSleep;






//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
void SYS_GPSSleep(void);
void SYS_GPSWake(void);
void SYS_GSMSleep(void);
void SYS_GSMWake(void);
void SYS_WorkMode_Exec(void);
void SYS_WorkModeInit(void);
void SYS_WorkModeSleep_Count(void);
void SYS_VoltageLowMode_Exec(void);
void MainPowerCheck(void);
#endif















