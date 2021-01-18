/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: CollectModule.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-10-23
* @brief     本文件为Collect功能模块所有对外接口层的头文件
******************************************************************************/
#ifndef _COLLECT_MODULE_H_
#define _COLLECT_MODULE_H_

#include "Collect.h"

/******************************************************************************
 * Pre-Processor Defines
 ******************************************************************************/
#define CAN_CHANNEL1        0x00
#define CAN_CHANNEL2        0x01


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void COLT_SetTotalWorkTime(uint32_t time);
uint32_t COLT_GetTotalWorkTime(void);

// 采集模块信息函数接口
uint16_t COLT_GetMainPowerVoltage(void);
uint16_t COLT_GetBatVoltage(void);
uint8_t COLT_GetBatLowStatus(void);
uint8_t COLT_GetBatChargeStatus(void);
uint8_t COLT_GetAccStatus(void);
uint8_t COLT_GetMainPowerStatus(void);
uint8_t COLT_GetMainPowerLowStatus(void);
uint8_t COLT_GetSwitch1Status(void);
uint8_t COLT_GetSwitch2Status(void);
uint8_t COLT_GetSwitch3Status(void);
uint8_t COLT_GetBoxOpenStatus(void);
uint8_t COLT_GetVehicleTowingStatus(void);
void COLT_ReadInternalWdtState(void);
uint16_t COLT_GetStVersion(void);

// 用户API函数
uint8_t CAN_GetCommState(uint8_t channel);
uint8_t CAN_GetRecvState(uint8_t channel);
uint8_t CAN_GetVinState(void);
uint8_t CAN_GetEpDataState(void);
uint8_t CAN_GetEpType(void);
uint16_t CAN_GetEngineSpeed(void);
uint8_t CAN_GetEngineType(void);
uint16_t CAN_GetMilLampState(void);

// 周期性函数
void CTL_Do1sTasks(void);
void CTL_SetPwroffDelayTime(uint8_t time);
void CTL_SetRestartDelayTime(uint8_t time);

// 任务函数接口
void Collect_ServiceInit(void);
void Collect_ServiceStart(void);

#endif

