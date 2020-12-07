/*
 * Copyright(c)2019,硬件研发部
 * All right reseved
 *
 * 文件名称: GpsProtocolLayer.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GPS功能模块协议层实现的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by  创建本文件
 *
 */
 
#ifndef GPSPROTOCOLLAYER_H_20110321_3CBC0E8C_C3FC_4EA8_941E_69B41383AC1D
#define GPSPROTOCOLLAYER_H_20110321_3CBC0E8C_C3FC_4EA8_941E_69B41383AC1D

//-----常量定义----------------------------------------------------------------

//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
BOOL   CheckGpsPara(const uint8 *pucData, uint16 usLength);
BOOL   CheckGpsData(const uint8 *pucData, uint16 usLength);
void   ParseGpsData(const uint8 *pucData, uint16 usLen, PSTU_Orient pstuDes);

void   SetAntennaOk(void);
void   SetAntennaErr(void);
BOOL   IsAntennaOk(void);
void   ReadFromGpsSerialBuff(uint8 *buff,uint16 len);
void   ParseNMEA(void);
#endif



























