/*
 * Copyright(c)2019, 硬件研发部
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
 * 2019-03-21, by , 创建本文件
 *
 */

#ifndef CollectHW_H
#define CollectHW_H

//----常量宏定义---------------------------------------------------------------

//#define BatPowOn()			GPIO_SetBits(GPIOD, GPIO_Pin_3)
//#define BatPowOff()			GPIO_ResetBits(GPIOD, GPIO_Pin_3)
#define MainPowOn()			GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define MainPowOff()		GPIO_SetBits(GPIOB, GPIO_Pin_8)
#define ChargeOn() 			GPIO_ResetBits(GPIOB, GPIO_Pin_11)
#define ChargeOff()			GPIO_SetBits(GPIOB, GPIO_Pin_11)
    
#define ReadSwitchACC()		GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5)
#define ReadSwitchIn1()		GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)
#define ReadGpsAntShot()	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define ReadGpsAntOpen()	GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)

#define ReadSIMCardState()  GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)


#define ETHER_LED_ON()			(GPIO_ResetBits(GPIOA, GPIO_Pin_3))	//PA3
#define ETHER_LED_OFF()     	(GPIO_SetBits(GPIOA, GPIO_Pin_3))	//PA3

#define WiFi_LEN_ON()			(GPIO_ResetBits(GPIOA, GPIO_Pin_1))	//PA1
#define WiFi_LEN_OFF()     		(GPIO_SetBits(GPIOA, GPIO_Pin_1))	//PA1

#define GpsLedOn()				(GPIO_ResetBits(GPIOA, GPIO_Pin_5))	//PA5
#define GpsLedOff()     		(GPIO_SetBits(GPIOA, GPIO_Pin_5))	//PA5

#define USB_VBUS_OFF()			(GPIO_ResetBits(GPIOA, GPIO_Pin_4))	//PA4
#define USB_VBUS_ON()     		(GPIO_SetBits(GPIOA, GPIO_Pin_4))	//PA4

#define SD_PWR_ON()			    (GPIO_SetBits(GPIOC, GPIO_Pin_4))	//PC4
#define SD_PWR_OFF()     		(GPIO_ResetBits(GPIOC, GPIO_Pin_4))//PC4

#define POWER_LED_ON()			(GPIO_ResetBits(GPIOB, GPIO_Pin_0))	//PB0
#define POWER_LED_OFF()     	(GPIO_SetBits(GPIOB, GPIO_Pin_0))	//PB0
//-----外部函数定义------------------------------------------------------------
void Collect_GPIO_Pin_Init(void);
void IWdtInit(uint16 usTime);
void IWdtFeed(void);

#endif


























