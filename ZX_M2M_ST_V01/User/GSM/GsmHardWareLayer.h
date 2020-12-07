/*
 * Copyright(c)2019,硬件研发部
 * All right reserved
 *
 * 文件名称: GsmHardWareLayer.h
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GSM模块硬件驱动层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-05-09 by , 创建本文件
 *
 */
 
#ifndef GSMHARDWARELAYER_H_20110609_A228C809_39C9_43EE_87BF_92CE3ACDEB20
#define GSMHARDWARELAYER_H_20110609_A228C809_39C9_43EE_87BF_92CE3ACDEB20

//-----常量定义----------------------------------------------------------------
#define ModemRSTHigh() 			(GPIO_SetBits(GPIOC, GPIO_Pin_8))		//GSM模块复位
#define ModemRSTLow() 			(GPIO_ResetBits(GPIOC, GPIO_Pin_8)	   //GSM模块复位

#define ModemDTRHigh() 			(GPIO_SetBits(GPIOC, GPIO_Pin_9))		//GSM模块休眠
#define ModemDTRLow() 			(GPIO_ResetBits(GPIOC, GPIO_Pin_9))     //GSM模块唤醒
#define ModemPwrHigh() 			(GPIO_SetBits(GPIOA, GPIO_Pin_15)) 		//拉高供电(PA15)
#define ModemPwrLow() 			(GPIO_ResetBits(GPIOA, GPIO_Pin_15))   	//拉低断电(PA15)
#define ModemOpenHigh()			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  	//拉高modem的IGN,开机(PB14)
#define ModemOpenLow() 			(GPIO_SetBits(GPIOB, GPIO_Pin_14))  	//拉低modem的IGN,关机(PB14)
#define Modem_Status()			(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))		//开关机状态
#define GsmLedOn()          	(GPIO_ResetBits(GPIOE, GPIO_Pin_2))   	//拉低 GSN led 亮 (PE2)
#define GsmLedOff()				(GPIO_SetBits(GPIOE, GPIO_Pin_2))   	//拉高 GSN led 灭 (PE2)

//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------
void GSM_Uart_Init(void);
void GSM_UART_Write(uint8 *Data, uint16 len);
void GSM_GPIO_Pin_Init(void);
uint8 WriteGsmUartData(uint8 *Data, uint16 Len);
BOOL ReadGsmUartData(uint8 **data, uint16* len);

#endif

































