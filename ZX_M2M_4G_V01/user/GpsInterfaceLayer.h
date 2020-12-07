/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GpsInterfaceLayer.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为GPS功能模块接口层实现的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , 创建本文件
 *
 */

#ifndef GPSINTERFACELAYER_H_20110321_641F6404_7CD7_4C49_892_C89E52ED969F
#define GPSINTERFACELAYER_H_20110321_641F6404_7CD7_4C49_892_C89E52ED969F

//-----常量定义----------------------------------------------------------------


//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------
extern uint16  m_usGpsDly;  
extern uint32  m_uiDistance;

//-----外部函数----------------------------------------------------------------
void InitGps(uint32 uiBps);
void ReInitGps(void);
void GPSRMCGGA(void);
//void SaveDistanceToEEPROM_AccOff(void);
void SaveDistanceToRTCRAM(uint32 uiDistance);
void SaveDistanceToEEPROM(uint32 uiDistance);
#endif




























