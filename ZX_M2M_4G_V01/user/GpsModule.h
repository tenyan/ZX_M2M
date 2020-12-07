/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved 
 * 
 * 文件名称: GpsModule.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为GPS功能模块所有对外接口文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , 创建本文件
 *
 */

#ifndef GPSMODULE_H_20110321_FF9FED25_F92A_4FB0_9B79_61E85B793F14
#define GPSMODULE_H_20110321_FF9FED25_F92A_4FB0_9B79_61E85B793F14
#include "GpsHardWareLayer.h"

//-----常量定义----------------------------------------------------------------
#define TASK_GPS_STK_SIZE                   250
#define TASK_GPS_PRIO                       7
#define TASK_GPS_ID                         7

#define GPS_MSG_WARN_SPEED_VOER             1    //超速报警
#define GPS_MSG_WARN_SPEED_OK               2    //解除超速报警
#define GPS_MSG_WARN_ANTENNA_ERROR          3    //天线异常报警
#define GPS_MSG_WARN_ANTENNA_OK             4
#define GPS_MSG_WARN_ROUND                  5    //拐弯报警
#define Antenna_ok_time						10
#define Antenna_err_time					10
#define GPS_RECEIVE_BUFF_LEN        		650
#define GPS_RECEIVE_DATAOUT          		5                         //5秒没有收到GPS模块数据认为模块异常
//-----结构定义----------------------------------------------------------------

//日期时间结构体
typedef struct _STU_Date_
{
  uint8  ucYear;             //年
  uint8  ucMon;              //月
  uint8  ucDay;              //日
  uint8  ucHour;             //时
  uint8  ucMin;              //分
  uint8  ucSec;              //秒
}STU_Date, *PSTU_Date;

//GPS模块数据结构体
typedef struct _STU_Orient_
{


    int32   lLongitude;      //经度,1:度,2:分,3\4:分小数
    int32   lLatitude;       //纬度,1:度,2:分,3\4:分小数
    uint16   usSpeed;         //速度,单位:公里/小时
    int16    sHeight;         //高度,单位米,海平面以下为负数
    uint16   usForDirect;     //0-360
    STU_Date stuDate;         //时间
    uint8    ucGpsState;      //GPS状态字节
                              // B0=0 天线正常,1天线故障
                              // B1=1 GPS定位,0 GPS不定位
                              // B5-B2=GPS 收到的星数
                              // B6 =0 GPS模块正常(有数据输出)1=模块异常
                              // B7=0 速度正常, 1=超速
    uint8    ucPostion;       //方位值   b0:0=西,1=东, b1:0=南,1=北
    int32 lLongitude_PPMDegree;		//经度,单位:百万分之一度

	int32 lLatitude_PPMDegree;		    //纬度,单位:百万分之一度
    uint8 satellitenums;			//收到的星数
}STU_Orient, *PSTU_Orient;


//GPS模块参数结构体
typedef struct _STU_GPS_PARA_
{
	//OS_EVENT *pMQ;         //消息队列事件指针
	    
}STU_GpsPara, *PSTU_GpsPara;


struct _NMEAdata  //52
{
    uint8 Latitude[10];                                      //纬度
    uint8 NS;                                                   //N南北
    uint8 Longitude[11];                                   //经度
    uint8 EW;                                                   //E东西
    uint8 Height[7];                                         //高度
    uint8 HeightSing;                                      //表示高度有"+""-"
    uint8 UtcTime[6];                                      //
    uint8 UtcDate[6];                                       //
    uint8 Speed[8];                                          //
    uint8 ForDirect[3];                                     //
    uint8 DataStatus;                                       //GPS是否有效:为"A"表示有效,为"V"表示无效
    uint8 SatelliteNum[2];                                //当前收到的星数
};


typedef struct _GPS
{ 
    struct _NMEAdata NMEAdata;                      //GPS收到的数据
    uint8 RecvBuff[GPS_RECEIVE_BUFF_LEN];
    uint16 RecvBuffLen;
    uint8 RecvTimeOut;                                    // GPS没有数据输出计时
    uint8 SleepState;                                        //=0 不休眠；=1休眠状态
    uint8 SleepLedTime;                                   //GPS休眠期间灯灭的时间定为3秒钟,灭3秒之后闪烁一次
    uint8 modelFlag;                                        //判定GPS模块输出的语句是有没有包含天线检测语句
                                                                    //B0 = 0:GPS输出正常3条语句;=1:GPS恢复出厂设置
}STU_Gps;


//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
PSTU_Orient GPS_GetOrient(void);
uint32   GPS_GetLongitude(void);
uint32   GPS_GetLatitude(void);
uint8 	GPS_GetLongitudeHemisphere(void);
uint8 	GPS_GetLatitudeHemisphere(void);	
uint16 	GPS_GetForDirect(void);
int16 	GPS_GetHeight(void);
uint16   GPS_GetSpeed(void);
uint8    GPS_GetSpeedoverState(void);
uint32   GPS_GetDistance(void);
uint8    GPS_GetSatellitenums(void);
uint8    GPS_GetState(void);
void     GPS_SleepGps(void);
void     GPS_WakeGps(void);
uint8    GPS_GetGpsState(void);
uint8    GPS_GetAnteState(void);
uint8    GPS_GetOrientState(void);
void     GPS_SetDistance(uint32 uiDistance);
STU_Date GPS_GetUtcTime(void);
STU_Date GPS_GetBeiJinTime(void);
void     GPS_TimerCount_Delay(void);
void 	 GPS_TimerCount_NoDelay(void);
void     TaskGps(void *pvData);
uint8 *GPS_GetLatitude_Original(void);
uint8 *GPS_GetLongitude_Original(void);
void GPS_Function(uint8 *buff,uint16 uslen);
void* pthread_Gps_Function(void* data);
#endif 


