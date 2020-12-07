/*****************************************************************************
 *   rtc.c:  Realtime clock C file for NXP LPC177x_8x Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.26  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/
#include<stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include<string.h>
#include "config.h"
#include "rtc.h"
#include "amx8xx.h"

#define SECONDS_IN_A_DAY     (86400U)
#define SECONDS_IN_A_HOUR    (3600U)
#define SECONDS_IN_A_MIN     (60U)
#define DAYS_IN_A_YEAR       (365U)
#define DAYS_IN_A_LEAP_YEAR  (366U)

// Table of month length (in days) for the Un-leap-year
static const uint8_t ULY[] = {0U, 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
// Table of month length (in days) for the Leap-year
static const uint8_t  LY[] = {0U, 31U, 29U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};
// Number of days from begin of the non Leap-year
static const uint16_t MONTH_DAYS[] = {0U, 0U, 31U, 59U, 90U, 120U, 151U, 181U, 212U, 243U, 273U, 304U, 334U};
//STU_Date  Date;
static const uint8 DayOfMonth[12]={31,28,31,30,31,30,31,31,30,31,30,31};
uint32 virtual_secs = 0;
extern STU_STM32_MCU stu_STM32_MCU;
/*uint8 correct_time_flag = 0;
void SetCorrectTimeOk()
{
	correct_time_flag = 1;
}

uint8 IsCorrectTimeOk()
{
	return correct_time_flag;
}
*/
#if 0
//校准RTC时间, 每次系统复位后只校准一次,只在定位或收到平台的校时命令时执行
void AccountRTCDate(STU_Date date)
{
	STU_Date time;
		
//	if(IsCorrectTimeOk())
//		return;
    am_get_time_beijin(&time.ucYear,&time.ucMon,&time.ucDay,&time.ucHour,&time.ucMin,&time.ucSec);
	if(date.ucYear!=time.ucYear ||date.ucMon!=time.ucMon||date.ucDay!=time.ucDay||
	   date.ucHour!=time.ucHour ||date.ucMin!=time.ucMin||date.ucSec!=time.ucSec)
	{
		am_set_time_beijin(time.ucYear,time.ucMon,time.ucDay,time.ucHour,time.ucMin,time.ucSec);
		printf("Correct RTC");
	}
	
	SetCorrectTimeOk();
	
}
#endif
//获取RTC时间,该时间为北京时间
STU_Date GetRTCTime()
{
	STU_Date time;
	
	am_get_time_beijin(&time.ucYear,&time.ucMon,&time.ucDay,&time.ucHour,&time.ucMin,&time.ucSec);

	return time;
}

//将UTC时间转换为北京时间
BOOL UtcToBjTime(STU_Date *utc, STU_Date *bj)
{
	if((0==utc->ucYear) ||(0==utc->ucMon) || (0==utc->ucDay))
		return FALSE;
	
	
	*bj = *utc;
	
    if(utc->ucHour<16)                                          //20080506
    {
        bj->ucHour = utc->ucHour+8;

        bj->ucDay = utc->ucDay;
    }
    else 
    {
        bj->ucHour = utc->ucHour+8-24;
        bj->ucDay = utc->ucDay+1;
		if((!(bj->ucYear%400)||!(bj->ucYear%4)&&(bj->ucYear%100))&&(bj->ucMon==2))  //闰年
		{
            if(bj->ucDay > 29)
            {
                bj->ucDay = 1;
                bj->ucMon = 3;
            }
		}
        else if(bj->ucDay > DayOfMonth[bj->ucMon-1])
        {
            bj->ucDay = 1;
            if((++bj->ucMon)>12)
            {
                bj->ucMon = 1;
                bj->ucYear++;
            }
		}
    }
	return TRUE;
}

//将北京时间转换为UTC时间
BOOL BjToUtcTime(STU_Date *bj,STU_Date *utc)
{
	if((bj->ucYear==0) && (bj->ucMon==0) && (bj->ucDay==0))
		return FALSE;
	
	*utc = *bj;
    if(bj->ucHour < 8)          //20080506
    {
    	utc->ucHour = bj->ucHour + 16;
        utc->ucDay  = bj->ucDay - 1;
		if((!(utc->ucYear%400)||!(utc->ucYear%4)&&(utc->ucYear%100))&&(utc->ucMon==3))  //闰年
		{
            if(utc->ucDay==0)
            {
                utc->ucDay = 29;
                utc->ucMon = 2;
            }
		}
        else if(utc->ucDay==0)
        {
            utc->ucDay = 1;
            if((--utc->ucMon)==0)
            {
                utc->ucMon = 12;
                utc->ucYear--;
            }
		}
    }
    else 
    {
        utc->ucHour = bj->ucHour-8;
        utc->ucDay  = bj->ucDay;
    }
	return TRUE;
}

STU_Date GetRTCTime_UTC()
{
	STU_Date time, utc;
	
//	time = GetRTCTime();
    time = stu_STM32_MCU.Date;
	if(BjToUtcTime(&time, &utc))
	{
		return utc;
	}
	else
	{
		memset(&utc, 0, 6);
		return utc;
	}
	#if 0
    if(time.ucHour<16)                                          //20080506
    {
        time.ucHour = time.ucHour+8;
        time.ucDay = time.ucDay;
    }
    else 
    {
        time.ucHour = time.ucHour+8-24;
        time.ucDay = time.ucDay+1;
		if((!(time.ucYear%400)||!(time.ucYear%4)&&(time.ucYear%100))&&time.ucMon==2)  //闰年
		{
            if(time.ucDay>29)
            {
                time.ucDay = 1;
                time.ucMon = 3;
            }
		}
        else if(time.ucDay>DayOfMonth[time.ucMon-1])
        {
            time.ucDay = 1;
            if((++time.ucMon)>12)
            {
                time.ucMon = 1;
                time.ucYear++;
            }
        }
	}
	return time;
	#endif
}


#if 0
//获取RTC的UTC时间
STU_Date GetRTCTime_UTC()
{
	STU_Date time, utc;
	
	time = GetRTCTime();
	if(BjToUtcTime(&time, &utc))
	{
		return utc;
	}
	else
	{
		memset(&utc, 0, 6);
		return utc;
	}
	#if 0
    if(time.ucHour<16)                                          //20080506
    {
        time.ucHour = time.ucHour+8;
        time.ucDay = time.ucDay;
    }
    else 
    {
        time.ucHour = time.ucHour+8-24;
        time.ucDay = time.ucDay+1;
		if((!(time.ucYear%400)||!(time.ucYear%4)&&(time.ucYear%100))&&time.ucMon==2)  //闰年
		{
            if(time.ucDay>29)
            {
                time.ucDay = 1;
                time.ucMon = 3;
            }
		}
        else if(time.ucDay>DayOfMonth[time.ucMon-1])
        {
            time.ucDay = 1;
            if((++time.ucMon)>12)
            {
                time.ucMon = 1;
                time.ucYear++;
            }
        }
	}
	return time;
	#endif
}
#endif

//获取RTC的北京时间
STU_Date GetRTCTime_Beijin()
{
	return GetRTCTime();
}

/************************************************************************
 * @brief  由秒计算出日期
 * @param  seconds           :输入的秒
 * @param  datetime  :计算出来的年月日等信息结构体
 * @retval None
 ************************************************************************/
 void RTC_SecondToDateTime(const uint32_t * seconds, STU_Date_LongYear* datetime)
{
    uint32_t x;
    uint32_t Seconds, Days, Days_in_year;
    const uint8_t *Days_in_month;
    /* Start from 1970-01-01*/
    Seconds = *seconds;
    /* days*/
    Days = Seconds / SECONDS_IN_A_DAY;
    /* seconds left*/
    Seconds = Seconds % SECONDS_IN_A_DAY;
    /* hours*/
    datetime->ucHour= Seconds / SECONDS_IN_A_HOUR;
    /* seconds left*/
    Seconds = Seconds % SECONDS_IN_A_HOUR;
    /* minutes*/
    datetime->ucMin= Seconds / SECONDS_IN_A_MIN;
    /* seconds*/
    datetime->ucSec= Seconds % SECONDS_IN_A_MIN;
    /* year*/
    datetime->usYear= 1970;
    Days_in_year = DAYS_IN_A_YEAR;

    while (Days > Days_in_year)
    {
        Days -= Days_in_year;
        datetime->usYear++;
        if  (datetime->usYear& 3U)
        {
            Days_in_year = DAYS_IN_A_YEAR;
        }
        else
        {
            Days_in_year = DAYS_IN_A_LEAP_YEAR;    
        }
    }

    if  (datetime->usYear& 3U)
    {
        Days_in_month = ULY;
    }
    else
    {
        Days_in_month = LY;    
    }

    for (x=1U; x <= 12U; x++)
    {
        if (Days <= (*(Days_in_month + x)))
        {
            datetime->ucMon= x;
            break;
        }
        else
        {
            Days -= (*(Days_in_month + x));
        }
    }
    datetime->ucDay= Days;
}

 void RTC_DateTimeToSecond(const STU_Date_LongYear* datetime, uint32_t * seconds)
{
    /* Compute number of days from 1970 till given year*/
    *seconds = (datetime->usYear- 1970U) * DAYS_IN_A_YEAR;
    /* Add leap year days */
    *seconds += ((datetime->usYear / 4) - (1970U / 4));
    /* Add number of days till given month*/
    *seconds += MONTH_DAYS[datetime->ucMon];
    /* Add days in given month*/
    *seconds += datetime->ucDay;
    /* For leap year if month less than or equal to Febraury, decrement day counter*/
    if ((!(datetime->usYear& 3U)) && (datetime->ucMon<= 2U))
    {
        (*seconds)--;
    }

    *seconds = ((*seconds) * SECONDS_IN_A_DAY) + (datetime->ucHour* SECONDS_IN_A_HOUR) + 
               (datetime->ucMin* SECONDS_IN_A_MIN) + datetime->ucSec;
    (*seconds)++;
}

void VirtualTimeInit()
{
	STU_Date_LongYear virtual_date = {2015,6,8,23,57,0};
	
	RTC_DateTimeToSecond(&virtual_date, &virtual_secs);
}
//虚拟RTC计时器,次函数1s运行一次 
void VirtualRTC_Timer()
{
	virtual_secs++;
}

STU_Date RTC_GetVirtualTime()
{
	STU_Date date;
	STU_Date_LongYear date_longyear;

	RTC_SecondToDateTime(&virtual_secs, &date_longyear);
	date.ucYear = date_longyear.usYear-2000;
	date.ucMon= date_longyear.ucMon;
	date.ucDay= date_longyear.ucDay;
	date.ucHour= date_longyear.ucHour;
	date.ucMin= date_longyear.ucMin;
	date.ucSec= date_longyear.ucSec;
	return date;
}

#if 0
void main()
{
	STU_Date time;
	
	while(1)
	{
		time = GetRTCTime_Beijin();
		printf("20d%-d%-d%-d%-d%-d%",time.ucYear,time.ucMon,time.ucDay,time.ucHour,time.ucMin,time.ucSec);
		sleep(1);
	}
}
#endif

/*****************************************************************************
**                            End Of File
******************************************************************************/

