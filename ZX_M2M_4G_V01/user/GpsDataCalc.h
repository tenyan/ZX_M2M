/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GpsDataCalc.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGPSģ�����ݶ��μӹ��ӿ������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-13, by  �������ļ�
 *
 */

#ifndef  GPSDATACALC_H_20110326_A943E736_48DF_4288_B804_2CC910969EA2
#define  GPSDATACALC_H_20110326_A943E736_48DF_4288_B804_2CC910969EA2

//-----��������----------------------------------------------------------------
#define  SPEEDQUEUEDEPTH        5
#define  SPEEDOVER              110              

//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
uint16   CalcSpeed(uint8 *pucStr);
uint32   CalcLatitude(uint8 *pucStr,  uint8 ucFlag);
uint32   CalcLongitude(uint8 *pucStr, uint8 ucFlag);
int16    CalcHeight(uint8 *pucStr, uint8 ucLength);
uint16   CalcDirect(uint8 *pucStr);
uint8    CalcSatellite(uint8 *pucStr);
void     CalcDateTime(uint8 *pucDate, uint8 *pucTime, PSTU_Date pstuDT);
uint8    CalcPosition(uint8 ucLon, uint8 ucLat);


#endif

























