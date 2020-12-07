/*
 * Copyright(c)2011, 硬件研发部
 * All right reserved 
 * 
 * 文件名称: CollectHW.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为采集功能模块综合层实现的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2011-03-21, by zwp, 创建本文件
 *
 */

#ifndef CollectHW_H
#define CollectHW_H

//----常量宏定义---------------------------------------------------------------
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

//-----外部函数定义------------------------------------------------------------
void Collect_GPIO_Pin_Init(void);


#endif


























