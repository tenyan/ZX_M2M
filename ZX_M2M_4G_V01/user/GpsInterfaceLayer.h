/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GpsInterfaceLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪGPS����ģ��ӿڲ�ʵ�ֵ�ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , �������ļ�
 *
 */

#ifndef GPSINTERFACELAYER_H_20110321_641F6404_7CD7_4C49_892_C89E52ED969F
#define GPSINTERFACELAYER_H_20110321_641F6404_7CD7_4C49_892_C89E52ED969F

//-----��������----------------------------------------------------------------


//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
extern uint16  m_usGpsDly;  
extern uint32  m_uiDistance;

//-----�ⲿ����----------------------------------------------------------------
void InitGps(uint32 uiBps);
void ReInitGps(void);
void GPSRMCGGA(void);
//void SaveDistanceToEEPROM_AccOff(void);
void SaveDistanceToRTCRAM(uint32 uiDistance);
void SaveDistanceToEEPROM(uint32 uiDistance);
#endif




























