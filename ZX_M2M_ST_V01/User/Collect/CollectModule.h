/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: CollectModule.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为Collect功能模块所有对外接口层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-13, by , 创建本文件
 *
 */
#ifndef _CollectModule_H
#define _CollectModule_H

#include "CollectHW.h"
//-----常量定义----------------------------------------------------------------
#define TASK_Collect_STK_SIZE                   300
#define TASK_Collect_PRIO                       11
#define TASK_Collect_ID                         11


//P1
#define BAT_POW           (1UL << 18)
#define RING_PIN           (1UL << 6)
#define ACC_PIN           (1UL << 9)

#define  AlarmOFF               0x01                //报警开关关闭  
#define  AlarmON                0x02                //报警开关打开
#define  KeyOFF                 0x03                //钥匙开关ACC关闭  
#define  KeyON                  0x04                //钥匙开关ACC打开
#define  EngworkingOFF          0x05                //小时计开关关闭  
#define  EngworkingON           0x06                //小时计开关打开
#define  PositiveOFF            0x07                //外接电正极关闭  
#define  PositiveON             0x08                //外接电正极打开
#define  NegativeOFF            0x09                //外接电负极关闭  
#define  NegativeON             0x0a                //外接电负极打开
#define  HulledOFF              0x0b                //搭铁极关闭  
#define  HulledON               0x0c                //搭铁极打开

#define  BOXAlarm               0x10                //开盒异常  
#define  BOXNormal              0x11                //开盒正常  

/*上报原因类型定义*/

#define GET5V                    0x20     	//外电供电
#define NO5V                     0x21     	//电池供电
#define PowerLow                 0x22     	//电源电压低
#define PowerNormal              0x23     	//电源电压正常
#define Battery_Charge           0x24     	//电池充电
#define Battery_NOCharge         0x25    	//电池不充电


//采集状态
typedef struct STU_Sysstruct
{
    uint32 uiWorkingTime;			//小时计开关为高电平时开始统计，为低电平时停止统计,单位:s
    uint8 ucBAT_Voltage;			//内部电池电压
    uint16 usInput_Voltage;			//外电源电压
    uint16 usStandby_Voltage; 		//备用模拟量采集
    uint8 ucGPS_Temperature;		//GPS内部温度
    uint8 ucSwitch1;				//开关量采集1
								    //B0：报警开关    0=低电平,1=高电平
								    //B1：钥匙开关ACC     0=关闭,1=打开
								    //B2：小时计开关  0=关闭,1=打开；
								    //B3：外接电正极状态  0=断开,1=联通
								    //B4：外接电负极状态 0=联通,1=断开
								    //B5：搭铁极状态  0=联通,1=断开
								    //B6：GSM天线报警   0=无报警, 1=报警
								    //B7：保留位
    
    uint8 ucSwitch2;				//开关量采集2
								    //B0：继电器1控制状态 0=未锁车;1=锁车
								    //B1：继电器2控制状态 0=未锁车;1=锁车
								    //B2：保留
								    //B3：开盒报警 0=正常;1=报警
								    //B4：保留
								    //B5：RS232通讯状态 0= 未通讯；1=通讯；
								    //B6：RS232通讯仪表提示状态 0= 未通讯；1=通讯
								    //B7：CAN通讯状态 0= 未通讯；1=通讯；
    uint8 ucSwitch3;				//开关量采集3
								    //B0：供电状态 0=外电供电；1=电池供电；
								    //B1：外电馈电状态   0=不馈电；1=馈电
								    //B2：内部电池充电状态    0=未充电,1=充电；
								    //B3：车辆行驶速度报警 0=速度正常；1=超速报警；
								    //B4：车辆位置越界报警 0=不越界；1= 越界报警。
								    //B5：内置电池电压低 0=不馈电；1=馈电
								    //B6：保留
								    //B7：保留

    uint8  DisassmbleSwitch;		//防拆类开关量
								    //B0：EGND(外壳螺丝)       0=EGND OFF,   1=EGND ON
								    //B1：开盒报警1            0=正常,       1=报警
								    //B2：开盒报警2            0=正常,       1=报警
								    //B3：GSM天线报警,Reserved
								    //B4: GSM卡盖检测          0=正常,       1=拆除
								    //B5:GPS天线短路		   0=短路,       1=正常
								    //B6:GPS天线开路		   0=正常,       1=开路
								    //B7~B7:Reserved
}STU_Sysstruct,*PSTU_Sysstruct;


struct STU_ADstruct
{
    uint16 usInput_Voltage[10];
    uint16 usVBAT[10];
    uint16 usAD1[10];
    uint16 usAD2[10];
};	

//-----外部函数----------------------------------------------------------------


uint16 GetInput_Voltage(void);
uint8  GetBAT_Voltage(void);
void   SetInputVol(void);
void   SetBatVol(void);
uint8  GetGpsSwitchState(void);
uint8  GetGpsAntOpenState(void);
uint8  GetGpsAntShortState(void);
uint8  GetSwitch1State(void);
uint8  GetSwitch2State(void);
uint8  GetSwitch3State(void);
uint8  GetAccState(void);
uint8  GetPwrSupplyState(void);
uint8  GetPwrLowState(void);
uint8  GetBoxOpenState(void);
uint8  GetBatChargeState(void);
uint8  GetBatVolLowState(void);
void   SaveWorktime(uint32 uiworktime);
uint32 GetWorkingTime(void);
void   Collect_TimerCount_Delay(void);
void SetWorkingTime(uint32 uiWorkTime);
uint8 BuildAD_Switch(uint8 *buf);
void   TaskCollect (void *pData);
#endif

