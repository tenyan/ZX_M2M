/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: BaseFun.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�Ϊ���������������ļ�
 * 
 *--------------------------------------------------------------
 * �޸ļ�¼
 *--------------------------------------------------------------
 *
 * 2019-03-24, by , �������ļ�
 *
 */

#ifndef BASEFUN_H
#define BASEFUN_H

#define M_PI        3.14159265358979323846
#define NUMBER      6370693.4856530580439461631130889
#define MEM_MOD_LITLE   0//С��ģʽ
#define MEM_MOD_BIG     1//���ģʽ

//����ʱ��ṹ��
typedef struct _STU_Date_
{
  uint8  ucYear;             //��
  uint8  ucMon;              //��
  uint8  ucDay;              //��
  uint8  ucHour;             //ʱ
  uint8  ucMin;              //��
  uint8  ucSec;              //��
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









