/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: CollectInterfaceLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-08, , �������ļ�
 *
 */

#ifndef _CollectInterfaceLayer_H
#define _CollectInterfaceLayer_H

//----�ڲ���������----------------------------------------------


#define Input_VoltageAlarm          90    //�ⲿ��ѹ�����趨��ѹ����ֵ�Ҵ���9V
#define ADC_CHANNEL_BAT				10		
#define ADC_CHANNEL_PWR				11

//-----�ṹ����----------------------------------------------------------------





//----�ⲿ��������--------------------------------------------------------------------
void ADInit(void);
void ADConvert(void);
void CountWorkTime(void);
void InitWorktime(void);
void CheckSwitchState(void);
void CheckBoxState(void);
void ReportSwitchState (void);
void WatchDogHard(void);
void InputVoltageCheck(void);
void BatVoltageCheck(void);
void BatContronl(void);
void DealVoltage(void);


#endif



/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/
