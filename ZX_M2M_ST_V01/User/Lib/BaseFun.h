/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: BaseFun.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 此文件为基础函数库申明文件
 * 
 *--------------------------------------------------------------
 * 修改记录
 *--------------------------------------------------------------
 *
 * 2019-03-24, by , 创建本文件
 *
 */

#ifndef BASEFUN_H
#define BASEFUN_H

#define M_PI        3.14159265358979323846
#define NUMBER      6370693.4856530580439461631130889
#define MEM_MOD_LITLE   0//小端模式
#define MEM_MOD_BIG     1//大端模式

//日期时间结构体
typedef struct _STU_Date_
{
  uint8  ucYear;             //年
  uint8  ucMon;              //月
  uint8  ucDay;              //日
  uint8  ucHour;             //时
  uint8  ucMin;              //分
  uint8  ucSec;              //秒
}STU_Date, *PSTU_Date;



void   ClearMem(uint8_t* p,uint16_t len);
uint16_t CRC16(uint8_t *message, uint16_t len);
uint8_t  DecToUint8(uint8_t * str,uint8_t len);
uint16_t  hex2uint(uint8_t * str);
uint32_t  dec2uint(uint8_t * str);
uint16_t DecToUint16(uint8_t * str,uint8_t len);
uint32_t DecToUint32(uint8_t * str,uint8_t len);
void   Uint8ToDec2(uint8_t value,uint8_t* destbuff);
uint8_t  Uint8ToDec(uint8_t value,uint8_t* destbuff);
uint8_t  Uint16ToDec(uint16_t value,uint8_t* dest);
uint8_t  *SearchString(uint8_t *str,uint16_t strlen,char *substr,uint8_t substrlen);
uint8_t  Uint32IPDatatoDecWithDot(uint32_t IPData,uint8_t* destbuff);
uint16_t HexToASCII(uint8_t* data, uint8_t* buffer, uint16_t len);
uint8_t  ASCIITOHex(uint8_t* OData,uint8_t* NData,uint8_t len);
uint8_t  SumCalc(uint8* p,uint16 len);
uint8_t  *GetStringMiddle(uint8_t *str,uint16_t strlen, char str1,uint16_t Nostr1, char str2,uint16_t Nostr2,uint16_t *substrlen);
uint8_t  SingleASCIITOHex(uint8_t NData);
uint8_t  *GetStringBack(uint8_t *str,uint16_t strlen,char str1,uint16_t Nostr1);
uint8_t  Hex2ToUint8(uint8_t *ahex);
uint8_t  Dec3ToUint8(uint8_t * ahex,uint8_t len);
uint8_t  Dec2ToUint8(uint8_t * ahex);
void   HexToDatabuff(uint8_t *buff,uint8_t *hex,uint16_t hexlen);
void   DataToHexbuff(uint8_t* dest,uint8_t* src,uint8_t srclen);
void   Uint8ToHex2(uint8_t value,uint8_t* destbuff);
uint8_t  Dec3ToUint8(uint8_t * ahex,uint8_t len);
uint8_t  Dec2ToUint8(uint8_t * ahex);
void   HexToDatabuff(uint8_t *buff,uint8_t *hex,uint16_t hexlen);
void   ConvertBCDB(uint8_t *Abuff,uint8_t len);
uint8_t  Hex2ToUint8(uint8_t *ahex);
double deg2rad(double deg);
double distance(double lat1,double lon1,double lat2,double lon2);
uint8_t  Lrc(uint8_t *p, uint16_t len);
uint16 DigitFilter1(uint16*p, uint16 len, uint16 offsize);
uint16 StrToUint16(uint8 *ptr, uint8 mem_mod);
uint32 StrToUint32(uint8 *ptr, uint8 mem_mod);	
void Delay(uint32 uiTime);

#endif









