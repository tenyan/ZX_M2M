/**********************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: types.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2020-10-9
************************************************************************/
#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>

/******************************************************************************
*   Macros
******************************************************************************/
#define BIT(x)    ((uint32_t)1 << (x))
#define PART(x)     1

#define OS_TICKS_PER_SEC  1000UL
//#define osWaitForever     0xFFFF

typedef char  int8;
typedef unsigned char  uint8;
typedef signed short   int16;
typedef unsigned short uint16;
typedef signed int     int32;
typedef unsigned int   uint32;

#define UNUSED_VARIABLE(X)  ((void)(X))

/// Little Endian Order
//#define NUM_WORD(pbuf) (pbuf[0] + (pbuf[1]<<8))
//#define NUM_DWORD(pbuf) (pbuf[0] + (pbuf[1]<<8) + (pbuf[2]<<16) + (pbuf[3]<<24))
/// Big Endian Order
#define NUM_WORD(pbuf) (pbuf[1] + (pbuf[0]<<8))
#define NUM_DWORD(pbuf) (pbuf[3] + (pbuf[2]<<8) + (pbuf[1]<<16) + (pbuf[0]<<24))

/******************************************************************************
 *   Data Types
 ******************************************************************************/
typedef enum
{
  OFF = 0,
  ON  = 1,
} on_off_bool_t;

typedef enum
{
  FALSE = 0,
  TRUE  = 1,
} bool_t;

typedef union
{
	signed short word;
	struct
	{
		unsigned char low_byte;
		unsigned char high_byte;
	}byte;
}bytetype;

typedef union
{
	unsigned short word;
	struct{
		unsigned bit0 : 1;
		unsigned bit1 : 1;
		unsigned bit2 : 1;
		unsigned bit3 : 1;
		unsigned bit4 : 1;
		unsigned bit5 : 1;
		unsigned bit6 : 1;
		unsigned bit7 : 1;
		unsigned bit8 : 1;
		unsigned bit9 : 1;
		unsigned bit10 : 1;
		unsigned bit11 : 1;
		unsigned bit12 : 1;
		unsigned bit13 : 1;
		unsigned bit14 : 1;
		unsigned bit15 : 1;
	} w;
}bittype2;
 
typedef union
{
	unsigned char byte;
	struct
	{
		unsigned char bit0 : 1;
	  unsigned char bit1 : 1;
	  unsigned char bit2 : 1;
	  unsigned char bit3 : 1;
	  unsigned char bit4 : 1;
	  unsigned char bit5 : 1;
	  unsigned char bit6 : 1;
	  unsigned char bit7 : 1;
	} b;
} bittype;

// 日期+时间
typedef struct
{
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
}date_time_t;

// 日期+时间
typedef struct 
{
	uint8_t year;	
	uint8_t month;
	uint8_t weekday;
	uint8_t day;
	uint8_t hour;	
	uint8_t minute;
	uint8_t second;
} rtc_date_t;
typedef rtc_date_t  utc_time_t;

// 软件RTC
typedef struct
{
	uint8_t year;	
	uint8_t month;
	uint8_t day;
	uint8_t hour;	
	uint8_t minute;
	uint8_t second;
  uint16_t ms;  // 毫秒
  uint16_t ms_temp;
}rtc_soft_t;


#endif /* TYPES_H_ */

