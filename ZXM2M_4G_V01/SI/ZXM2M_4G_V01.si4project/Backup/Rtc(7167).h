/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: rtc.h
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-9-17
 * @brief:
 *******************************************************************************/
#ifndef _RTC_H_
#define _RTC_H_

#include "types.h"
#include "pcf8563.h"

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void RTC_Onchip_Init(void);

rtc_date_t RTC_GetBjTime(void);
rtc_date_t RTC_GetUtcTime(void);
void RTC_CorrectTime(rtc_date_t date);
uint8_t RTC_CovertUtcToBjt(rtc_date_t *utc, rtc_date_t *bj);
uint8_t RTC_ConvertBjToUtc(rtc_date_t *bj, rtc_date_t *utc);

void RTC_ConvertDataTimeToSeconds(rtc_date_t* time, uint32_t* seconds);
void RTC_ConvertSecondsToDateTime(uint32_t seconds, date_time_t* p_date_time);

void rtc_Initialize(void);
#endif /* _RTC_H_ */

