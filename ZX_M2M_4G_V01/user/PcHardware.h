/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: PcDebug.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪPcDebug����ģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-11, by lxf, �������ļ�
 *
 */

#ifndef _PCDEBUGHW_H
#define _PCDEBUGHW_H

#define DEV_TTYGS0_CDC_DEBUG "/dev/ttyGS0"
#define DEBUG_DMA_UART_Rx_BSIZE        500


//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

uint16 DEBUG_UART_Write(uint8 *data, uint16 Len);
//extern void DEBUG_UART_Write_Query(uint8 * Data, uint16 len);
//uint16 WriteDebugUartData(uint8 *data, uint16 Len);
BOOL ReadDebugUartData(uint8 **data, uint16* len);
void DEBUG_Uart_Init(void);  
#endif
