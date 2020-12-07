/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: can.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为can.h文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-10-21, 创建本文件
 *
 */

#ifndef _CANHW_H
#define _CANHW_H

#define	STD_FRAME                   0               // 标准帧
#define	EXT_FRAME                   1               // 扩展帧
#define CAN1_DATA_OK  				0x01	//成功收到CAN1数据
#define CAN1_DATA_TIMEOUT  			0x02	//接收CAN1数据超时gg
#define CAN_CHANNEL1                0x00
#define CAN_CHANNEL2                0x01

#define POWERON_CAN()         (GPIO_SetBits(GPIOA, GPIO_Pin_2))  	//打开CAN模块供电  PA2
#define POWEROFF_CAN()        (GPIO_ResetBits(GPIOA, GPIO_Pin_2))   //关闭CAN模块供电  PA2
//#define POWERON_485()         (GPIO_SetBits(GPIOD, GPIO_Pin_12))  	//打开CAN模块供电  PD12
//#define POWEROFF_485()        (GPIO_ResetBits(GPIOD, GPIO_Pin_12))  //关闭CAN模块供电  PD12
#define CAN1_LED_OFF()        (GPIO_SetBits(GPIOA, GPIO_Pin_6))  	//can1 LED 关闭    PE4
#define CAN1_LED_ON()         (GPIO_ResetBits(GPIOA, GPIO_Pin_6))  	//can1 LED 点亮    PE4
#define CAN2_LED_OFF()        (GPIO_SetBits(GPIOA, GPIO_Pin_7))  	//can1 LED 关闭    PE5
#define CAN2_LED_ON()         (GPIO_ResetBits(GPIOA, GPIO_Pin_7))  	//can1 LED 点亮    PE5
//蓝牙指示灯
//#define BLU_LED_OFF()         (GPIO_SetBits(GPIOE, GPIO_Pin_6))  	//蓝牙 LED 关闭    PE6
//#define BLU_LED_ON()          (GPIO_ResetBits(GPIOE, GPIO_Pin_6))  	//蓝牙 LED 点亮    PE6





//can1消息ID
#define ID_FAULTCODE_ONE			0x18FECA31		//只有一个故障码时的ID
#define ID_FAULTCODE_ONE_ENGINE		0x18FECA00		
#define ID_BROADCAST				0x18ECFF31		//广播帧ID
#define ID_BROADCAST_ENGINE			0x18ECFF00
#define ID_FAULTCODE_MULTI			0x18EBFF31		//多个故障码时的ID
#define ID_FAULTCODE_MULTI_ENGINE	0x18EBFF00	
#define ID_HANDLE_TEST              0XFFFFFF       //手持测试仪ID 去掉首字节


typedef struct _MessageDetail
{
	unsigned char LEN;
	unsigned char FF;           	// 是否标准帧
	unsigned int  CANID;			// 长度与FF有关
	unsigned char  CANRE[8];
}MessageDetail;
//-----外部函数----------------------------------------------------------------
void McuHwInit(void);
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data);
uint8 WaitForCAN1Message(uint16 timeout);
MessageDetail CAN1Read(void);
void Sleep_Can_Io(void);



#endif

