/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: PcDebug.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为PcDebug功能模块协议层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-11, by lxf, 创建本文件
 *
 */

#ifndef _PCDEBUGHW_H
#define _PCDEBUGHW_H

#define DEV_TTYGS0_CDC_DEBUG "/dev/ttyGS0"
#define DEBUG_DMA_UART_Rx_BSIZE        500


//-----结构定义----------------------------------------------------------------

//-----外部变量----------------------------------------------------------------

//-----外部函数----------------------------------------------------------------

uint16 DEBUG_UART_Write(uint8 *data, uint16 Len);
//extern void DEBUG_UART_Write_Query(uint8 * Data, uint16 len);
//uint16 WriteDebugUartData(uint8 *data, uint16 Len);
BOOL ReadDebugUartData(uint8 **data, uint16* len);
void DEBUG_Uart_Init(void);  
#endif
