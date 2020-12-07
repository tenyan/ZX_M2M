/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GpsDataCalc.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GPS模块数据二次加工接口申明文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-13, by  创建本文件
 *
 */

#ifndef  GPSDATACALC_H_20110326_A943E736_48DF_4288_B804_2CC910969EA2
#define  GPSDATACALC_H_20110326_A943E736_48DF_4288_B804_2CC910969EA2

//-----常量定义----------------------------------------------------------------
#define  SPEEDQUEUEDEPTH        5
#define  SPEEDOVER              110              

//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
uint16   CalcSpeed(uint8 *pucStr);
uint32   CalcLatitude(uint8 *pucStr,  uint8 ucFlag);
uint32   CalcLongitude(uint8 *pucStr, uint8 ucFlag);
int16    CalcHeight(uint8 *pucStr, uint8 ucLength);
uint16   CalcDirect(uint8 *pucStr);
uint8    CalcSatellite(uint8 *pucStr);
void     CalcDateTime(uint8 *pucDate, uint8 *pucTime, PSTU_Date pstuDT);
uint8    CalcPosition(uint8 ucLon, uint8 ucLat);


#endif

























