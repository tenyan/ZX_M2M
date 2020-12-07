/*
 * Copyright(c)2019,Ӳ���з���
 * All right reseved
 *
 * �ļ�����: GpsProtocolLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGPS����ģ��Э���ʵ�ֵ�ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by  �������ļ�
 *
 */
 
#ifndef GPSPROTOCOLLAYER_H_20110321_3CBC0E8C_C3FC_4EA8_941E_69B41383AC1D
#define GPSPROTOCOLLAYER_H_20110321_3CBC0E8C_C3FC_4EA8_941E_69B41383AC1D

//-----��������----------------------------------------------------------------

//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
BOOL   CheckGpsPara(const uint8 *pucData, uint16 usLength);
BOOL   CheckGpsData(const uint8 *pucData, uint16 usLength);
void   ParseGpsData(const uint8 *pucData, uint16 usLen, PSTU_Orient pstuDes);

void   SetAntennaOk(void);
void   SetAntennaErr(void);
BOOL   IsAntennaOk(void);
void   ReadFromGpsSerialBuff(uint8 *buff,uint16 len);
void   ParseNMEA(void);
#endif



























