/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: Rtc.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-9-17
 * @brief:
 *******************************************************************************/
#include "rtc.h"
#include "PcDebug.h"
#include "main.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define ONE_DAY_SECONDS      (86400U)
#define ONE_HOUR_SECONDS     (3600U)
#define ONE_MINUTE_SECONDS   (60U)

extern rtc_date_t st_rtc_data;

/******************************************************************************
 * Return the number of seconds since January, 1 1970
 * Seconds is stored as an unsigned 32 bit number so it won't roll over until
 * 6:28:15 AM 7-Feb-2106
 * 输入：n时区的具体日期时间
 * 输出：0时区的标准秒数
******************************************************************************/
void RTC_ConvertDataTimeToSeconds(rtc_date_t* time, uint32_t* seconds)
{
  uint32_t temp;

  // Days per month: 31,28,31,30,31,30,31,31,30,31,30,31
  uint16_t days[] = { 0, 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };

  temp = time->year + 2000; // Get the current year

  // Start with the number of days since 1970 assuming 365 days per year */
  (*seconds) = (temp - 1970) * 365;

  // Add leap year days
  (*seconds) += ((temp / 4) - (1970 / 4));

  // Add number of days till given month
  (*seconds) += days[time->month];

  /* Add days in given month*/
  (*seconds) += time->day-1;

  /* For leap year if month less than or equal to Febraury, decrement day counter*/
  if ((!(temp & 0x03)) && (time->month <= 0x02))
  {
    (*seconds)--;
  }

  (*seconds) = (*seconds) * 86400;   // 计算天的秒
  (*seconds) += (uint32_t)time->hour * 3600; // 计算小时的秒
  (*seconds) += (uint32_t)time->minute * 60;    // 计算分钟的秒
  (*seconds) += time->second;         // 计算秒
  (*seconds)++;
}

/************************************************************************
 * 秒转换为年，月，日，时，分，秒(Start from 1970-01-01)
 * 输入：n时区的标准秒数
 * 输出：n时区的具体日期时间
 ************************************************************************/
void RTC_ConvertSecondsToDateTime(uint32_t seconds, date_time_t* p_date_time)
{
  uint32_t days_left;
  uint16_t days_of_year;
  uint8_t days_of_month;

  // 秒转换为时分秒
  days_left = seconds / ONE_DAY_SECONDS; // 获取总天数
  seconds -= days_left*ONE_DAY_SECONDS;  // 计算天秒数

  p_date_time->hour = seconds / ONE_HOUR_SECONDS; // 小时(注意时区)
  seconds -= p_date_time->hour * ONE_HOUR_SECONDS;

  p_date_time->minute = seconds / ONE_MINUTE_SECONDS; // 分钟
  p_date_time->second = seconds - p_date_time->minute*ONE_MINUTE_SECONDS; // 秒

  // 转换天为年月日
  p_date_time->year = 1970; // 从1970-01-01开始
  p_date_time->month = 1;
  p_date_time->day = 1;

  while (days_left > 0)
  {
    if (p_date_time->year & 0x03)
      days_of_year = 365;  // 非闰年
    else
      days_of_year = 366; // 闰年(leap year)

    if (days_left < days_of_year)
    {
      while (days_left > 0) // 开始计算月
      {
        switch (p_date_time->month)
        {
        case 1:
        case 3:
        case 5:
        case 7:
        case 8:
        case 10:
        case 12:
          days_of_month = 31;
          break;

        case 4:
        case 6:
        case 9:
        case 11:
          days_of_month = 30;
          break;

        case 2:
          days_of_month = (days_of_year == 365) ? 28 : 29;
          break;

        default:
          break;
        }

        if (days_left < days_of_month)
        {
          p_date_time->day = days_left + 1; // 计算日期
          days_left = 0;
        }
        else
        {
          days_left -= days_of_month;
          p_date_time->month++; // 计算月
        }
      }
    }
    else
    {
      days_left -= days_of_year;
      p_date_time->year++;  // 计算年
    }
  }
}

/******************************************************************************
 * 将UTC时间转换为北京时间
 * 1.年份是可以被 400整除
 * 2.年份可以被4整除，并且是并且不能被100整除。
 ******************************************************************************/
uint8_t RTC_CovertUtcToBjt(rtc_date_t *utc, rtc_date_t *bj)
{
  const uint8_t day_of_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

  if ((0==utc->year) && (0==utc->month) && (0==utc->day))
  {
    return FALSE;
  }

  *bj = *utc;

  if (utc->hour < 16)
  {
    bj->hour = utc->hour + 8;
    bj->day = utc->day;
  }
  else
  {
    bj->hour = utc->hour + 8 - 24;
    bj->day = utc->day + 1;
    if (((bj->year%400==0)||((bj->year%4==0)&&(bj->year%100!=0)))&&(bj->month==2)) // 闰年
    {
      if (bj->day > 29)
      {
        bj->day = 1;
        bj->month = 3;
      }
    }
    else if (bj->day > day_of_month[bj->month - 1])
    {
      bj->day = 1;
      if ((++bj->month) > 12)
      {
        bj->month = 1;
        bj->year++;
      }
    }
  }

  return TRUE;
}

/******************************************************************************
 * 将北京时间转换为UTC时间
 ******************************************************************************/
uint8_t RTC_ConvertBjToUtc(rtc_date_t *bj, rtc_date_t *utc)
{
  if ((bj->year==0) && (bj->month==0) && (bj->day==0))
  {
    return FALSE;
  }

  *utc = *bj;
  if (bj->hour < 8)
  {
    utc->hour = bj->hour + 16;
    utc->day  = bj->day - 1;
    if (((utc->year%400==0)||((utc->year%4==0)&&(utc->year%100!=0)))&&(utc->month==3))   // 闰年
    {
      if (utc->day==0)
      {
        utc->day = 29;
        utc->month = 2;
      }
    }
    else if (utc->day==0)
    {
      utc->day = 1;
      if ((--utc->month)==0)
      {
        utc->month = 12;
        utc->year--;
      }
    }
  }
  else
  {
    utc->hour = bj->hour - 8;
    utc->day  = bj->day;
  }

  return TRUE;
}

/******************************************************************************
 * 获取RTC北京时间
 ******************************************************************************/
rtc_date_t RTC_GetBjTime(void)
{
  return st_rtc_data;
}

/******************************************************************************
 * 获取RTC的UTC时间
 ******************************************************************************/
rtc_date_t RTC_GetUtcTime(void)
{
  rtc_date_t utc;
  
  utc.year = 0;
  utc.month = 0;
  utc.weekday = 0;
  utc.day = 0;
  utc.hour = 0;
  utc.minute = 0;
  utc.second = 0;
  if(GPS_GetPositioningStatus()==1) // GPS已定位
  {
    utc = GPS_GetUtcTime();  // 使用GPS时间
  }
  else // GPS未定位
  {
    RTC_ConvertBjToUtc(&st_rtc_data, &utc); // 使用外部RTC时间
  }

  return utc;
}

//-----文件rtc.c结束---------------------------------------------
