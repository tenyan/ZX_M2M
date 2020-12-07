/*
 * Copyright(c)2019,Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmHardWareLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ��Ӳ���������ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by , �������ļ�
 *
 */
 
#ifndef _GPSHARDWARELAYER_H_
#define _GPSHARDWARELAYER_H_

//-----��������----------------------------------------------------------------
#define GPS_UART_BAUD_RATE              9600

#define GPS_UART_SEND_BUFF_LENGTH       256
#define GPS_UART_RECV_BUFF_LENGTH       1024

#define GpsPower_SET()          //(GPIO_SetBits(GPIO_PC30)) //��GPS��ԴPB4
#define GpsPower_CLR()          //(GPIO_ResetBits(GPIO_PC30)) //�ر�GPS��ԴPB4

//#define GpsRest()			    ()  //GPSģ������
//#define GpsWork()				()  //GPSģ��ָ�����

//#define GpsLedOn()				//(GPIO_ResetBits(GPIO_PC15))	//PE3
//#define GpsLedOff()     		//(GPIO_SetBits(GPIO_PC15))	//PE3 


//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
void GPS_Uart_Init(void);
BOOL ReadGpsUartData(uint8 **data, uint16* len);
uint16 WriteGpsUartData(uint8 *data, uint16 Len);
#endif


































