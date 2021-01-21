/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsModule.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-10-11
 * @brief:    本文件为GPS功能模块所有对外接口文件
******************************************************************************/
#ifndef _GPS_MODULE_H_
#define _GPS_MODULE_H_
#include "types.h"
#include "GpsNMEA.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
 #define GPS_RECV_TIMEOUT_SP  30   // 15秒没有收到GPS模块数据认为模块异常

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef enum
{
  GPS_MODULE_STATE_POWERON = 0x00,
  GPS_MODULE_STATE_INIT,
  GPS_MODULE_STATE_WORKING,
  GPS_MODULE_STATE_RESTART,
  GPS_MODULE_STATE_SLEPT
} gps_module_state_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void GPS_ServiceInit(void);
void GPS_ServiceStart(void);
void GPS_Do1sTasks(void);
void GSP_ModuleStateManage(void);
void GPS_SetModuleState(uint8_t state);

// GPS定位信息函数接口
uint8_t GPS_GetSatelliteNum(void);
uint8_t GPS_GetPositioningStatus(void);
uint8_t GPS_GetEastWest(void);
uint8_t GPS_GetNorthSouth(void);
uint32_t GPS_GetLongitude(void);
uint32_t GPS_GetLatitude(void);
uint16_t GPS_GetSpeed(void);
uint16_t GPS_GetHeading(void);
int16_t GPS_GetHeight(void);
utc_time_t GPS_GetUtcTime(void);
uint8_t GPS_GetModuleStatus(void);
uint8_t GPS_GetAntennaStatus(void);
uint8_t GPS_GetAntShortStatus(void);
uint8_t GPS_GetAntOpenStatus(void);
uint8_t GPS_GetSpeedOverrunStatus(void);

#endif

