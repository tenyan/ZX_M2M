/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: CollectInterfaceLayer.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 基础函数文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-08, , 创建本文件
 *
 */

#ifndef _CollectInterfaceLayer_H
#define _CollectInterfaceLayer_H

//----内部变量定义----------------------------------------------


#define Input_VoltageAlarm          90    //外部电压低于设定低压报警值且大于9V
#define ADC_CHANNEL_BAT				10		
#define ADC_CHANNEL_PWR				11

//-----结构定义----------------------------------------------------------------





//----外部函数定义--------------------------------------------------------------------
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
