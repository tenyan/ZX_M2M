/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: McuHW.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为McuHW.h文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-3-21, 创建本文件
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


//can1消息ID
//#define ID_FAULTCODE_ONE			0x18FECA31		//只有一个故障码时的ID
#define ID_FAULTCODE_ONE_ENGINE		0x18FECA00		
//#define ID_BROADCAST				0x18ECFF31		//广播帧ID
#define ID_BROADCAST_ENGINE			0x18ECFF00
//#define ID_FAULTCODE_MULTI			0x18EBFF31		//多个故障码时的ID
#define ID_FAULTCODE_MULTI_ENGINE	0x18EBFF00	


typedef struct _MessageDetail
{
	unsigned char LEN;
	unsigned char FF;           	// 是否标准帧
	unsigned int  CANID;			// 长度与FF有关
	unsigned char  CANRE[8];
}MessageDetail;

typedef struct _McuCan_Tx
{
    uint8 ucCommand;
	uint8 SN;
	
}stu_McuCan_Tx;

typedef struct _McuCan_Rx
{
	uint16 usLen;
    uint8 ucCommand;	
	uint8 SN;
	uint16 CANNum;
	uint8 CanData[1200];	
	
}stu_McuCan_Rx;

//-----外部函数----------------------------------------------------------------
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data);
void MCU_CAN_Uart_Init(void); 
BOOL ReadMCU_CAN_UartData(uint8 **data, uint16* len);
uint16 WriteMCU_CAN_UartData(uint8 *data, uint16 Len);
#endif

