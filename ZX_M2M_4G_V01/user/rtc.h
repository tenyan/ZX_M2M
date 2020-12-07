/*****************************************************************************
 *   rtc.h:  Header file for NXP LPC177x_8x Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __RTC_H 
#define __RTC_H

/*
typedef unsigned char  uint8;                   // �޷���8λ���ͱ���  
typedef signed   char  int8;                    // �з���8λ���ͱ���  
typedef unsigned short uint16;                  // �޷���16λ���ͱ��� 
typedef signed   short int16;                   // �з���16λ���ͱ��� 
typedef unsigned int   uint32;                  // �޷���32λ���ͱ��� 
typedef signed   int   int32;                   // �з���32λ���ͱ��� 
typedef float          fp32;                    // �����ȸ�������32λ���ȣ� 
typedef double         fp64;                    // ˫���ȸ�������64λ���ȣ�
typedef bool           BOOL; 

typedef unsigned char  uint8_t;                   // �޷���8λ���ͱ���  
typedef signed   char  int8_t;                    // �з���8λ���ͱ���  
typedef unsigned short uint16_t;                  // �޷���16λ���ͱ��� 
typedef signed   short int16_t;                   // �з���16λ���ͱ��� 
typedef unsigned int   uint32_t;                  // �޷���32λ���ͱ��� 
typedef signed   int   int32_t;                   // �з���32λ���ͱ��� 
*/
typedef struct {
    uint32_t RTC_Sec;                        /* Second value - [0,59]           */
    uint32_t RTC_Min;                        /* Minute value - [0,59]           */
    uint32_t RTC_Hour;                       /* Hour value - [0,23]             */
    uint32_t RTC_Mday;                       /* Day of the month value - [1,31] */
    uint32_t RTC_Mon;                        /* Month value - [1,12]            */
    uint32_t RTC_Year;                       /* Year value - [0,4095]           */
    uint32_t RTC_Wday;                       /* Day of week value - [0,6]       */
    uint32_t RTC_Yday;                       /* Day of year value - [1,365]     */
} RTCTime;

//����ʱ��ṹ��,�������������
typedef struct _STU_Date_LongYear
{
  uint16  usYear;           //��
  uint8  ucMon;            //��
  uint8  ucDay;            //��
  uint8  ucHour;           //ʱ
  uint8  ucMin;            //��
  uint8  ucSec;            //��
}STU_Date_LongYear, *PSTU_Date_LongYear;

/*����ʱ��ṹ��
typedef struct _STU_Date_
{
  uint8  ucYear;             //��
  uint8  ucMon;              //��
  uint8  ucDay;              //��
  uint8  ucHour;             //ʱ
  uint8  ucMin;              //��
  uint8  ucSec;              //��
}STU_Date, *PSTU_Date;
*/
#define IMSEC        0x00000001
#define IMMIN        0x00000002
#define IMHOUR       0x00000004
#define IMDOM        0x00000008
#define IMDOW        0x00000010
#define IMDOY        0x00000020
#define IMMON        0x00000040
#define IMYEAR       0x00000080

#define AMRSEC        0x00000001               /* Alarm mask for Seconds          */
#define AMRMIN        0x00000002               /* Alarm mask for Minutes          */
#define AMRHOUR       0x00000004               /* Alarm mask for Hours            */
#define AMRDOM        0x00000008               /* Alarm mask for Day of Month     */
#define AMRDOW        0x00000010               /* Alarm mask for Day of Week      */
#define AMRDOY        0x00000020               /* Alarm mask for Day of Year      */
#define AMRMON        0x00000040               /* Alarm mask for Month            */
#define AMRYEAR       0x00000080               /* Alarm mask for Year             */

#define PREINT_RTC    0x000001C8               /* Prescaler value, integer portion, 
                                                                     PCLK = 15Mhz */
#define PREFRAC_RTC    0x000061C0              /* Prescaler value, fraction portion, 
                                                                     PCLK = 15Mhz */
#define ILR_RTCCIF    0x01
#define ILR_RTCALF    0x02

#define CCR_CLKEN    0x01
#define CCR_CTCRST    0x02
#define CCR_CCALEN    0x10

//void AccountRTCDate(STU_Date date);
STU_Date GetRTCTime(void);
STU_Date GetRTCTime_Beijin(void);
STU_Date RTC_GetVirtualTime(void);
void VirtualRTC_Timer(void);
BOOL UtcToBjTime(STU_Date *utc, STU_Date *bj);
BOOL BjToUtcTime(STU_Date *bj,STU_Date *utc);
STU_Date GetRTCTime_UTC(void);

#endif /* end __RTC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
