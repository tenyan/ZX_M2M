/*
 * Copyright(c)2011, Ӳ���з���
 * All right reserved 
 * 
 * �ļ�����: CollectHW.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�Ϊ�ɼ�����ģ���ۺϲ�ʵ�ֵ�ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2011-03-21, by zwp, �������ļ�
 *
 */

#ifndef CollectHW_H
#define CollectHW_H

//----�����궨��---------------------------------------------------------------
//#define BatPowOn()			GPIO_SetBits()
//#define BatPowOff()			GPIO_ResetBits()
#define MainPowOn()			1//GPIO_ResetBits(GPIO_PD28)
#define MainPowOff()		1//GPIO_SetBits(GPIO_PD28)
#define ChargeOn() 			1//GPIO_ResetBits(GPIO_PD29)
#define ChargeOff()			1//GPIO_SetBits(GPIO_PD29)
    
#define ReadSwitchACC()		1//GPIO_ReadInputDataBit(GPIO_PB2)
#define ReadSwitchIn1()		1//GPIO_ReadInputDataBit(GPIO_PC5)
#define ReadGpsAntShot()	1//GPIO_ReadInputDataBit(GPIO_PD0)
#define ReadGpsAntOpen()	1//GPIO_ReadInputDataBit(GPIO_PD1)

#define ReadSIMCardState()  1//GPIO_ReadInputDataBit(GPIO_PC20)

//-----�ⲿ��������------------------------------------------------------------
void Collect_GPIO_Pin_Init(void);


#endif


























