/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: CollectModule.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version:  V1.0
* @Date:     2020-10-23
* @brief:    ���ļ�ΪCollect����ģ�����ж���ӿڲ��ͷ�ļ�
******************************************************************************/
#ifndef _COLLECT_MODULE_H_
#define _COLLECT_MODULE_H_

#include "CollectHW.h"
#include "Collect.h"

/******************************************************************************
 * Pre-Processor Defines
 ******************************************************************************/


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void COLT_SetTotalWorkTime(uint32_t time);
uint32_t COLT_GetTotalWorkTime(void);

// �ɼ�ģ����Ϣ�����ӿ�
uint8_t COLT_GetAccStatus(void);
uint16_t COLT_GetBatVoltage(void);
uint8_t COLT_GetBatLowStatus(void);
uint8_t COLT_GetBatChargeStatus(void);
uint16_t COLT_GetMainPowerVoltage(void);
uint8_t COLT_GetMainPowerStatus(void);
uint8_t COLT_GetMainPowerLowStatus(void);
uint8_t COLT_GetMainPowerFullStatus(void);
uint8_t COLT_GetSwitch1Status(void);
uint8_t COLT_GetSwitch2Status(void);
uint8_t COLT_GetSwitch3Status(void);
uint8_t COLT_GetBoxOpenStatus(void);
uint8_t COLT_GetVehicleTowingStatus(void);
void COLT_ReadInternalWdtState(void);

uint8_t COLT_GetGpsAntennaStatus(void);
uint8_t COLT_GetGpsAntShortStatus(void);
uint8_t COLT_GetGpsAntOpenStatus(void);
uint8_t COLT_GetGpsPositioningStatus(void);
uint8_t COLT_Get4gGpsStatus(void);
uint8_t COLT_GetSimCardStatus(void);

// �ն˲�����ʱ��ͳ��
void COLT_CumulateOfflineTimeInWork(void);     // ��������ʱ������ʱ��
void COLT_CumulateOfflineTimeInSleep(void); // ���߲�����ʱ��

void CTL_Do1sTasks(void);
void CTL_SetPwroffDelayTime(uint8_t time);
void CTL_SetRestartDelayTime(uint8_t time);

// �������ӿ�
void Collect_ServiceInit(void);
void Collect_ServiceStart(void);

#endif

