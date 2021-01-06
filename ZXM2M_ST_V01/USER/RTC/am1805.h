/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: am1805.h
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-12-12
 * @brief:    C Header file containing AM1815 driver function prototypes
 *******************************************************************************/
#ifndef _AM1805_H_
#define _AM1805_H_

#include "types.h"

/******************************************************************************
 * Macros
 ******************************************************************************/ 
#define AM1805_I2C_ADDR     0xD2

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef struct
{
	uint8_t hundredth;
	uint8_t second;
	uint8_t minute;
	uint8_t hour;
	uint8_t date;
	uint8_t weekday;
	uint8_t month;
	uint8_t year;
	uint8_t century;
	uint8_t mode;
} am18x5_time_regs_t;
extern am18x5_time_regs_t am18x5_time_regs;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void am18x5_Initialize(void);
void am18x5_GetDateTime(rtc_date_t *time);
void am18x5_SetDateTime(rtc_date_t *time);
void am18x5_PswControl(uint8_t state);
void am18x5_SetSleepTime(uint32_t uiSlot, uint8_t ucPin);
void am18x5_ClearIntFlag(void);
void am18x5_DisableAlarm(void);

#endif /* _AM1805_H_ */

