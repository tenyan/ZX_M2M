/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: McuHW.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪMcuHW.h�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-3-21, �������ļ�
 *
 */

#ifndef _CANHW_H
#define _CANHW_H

#define	STD_FRAME                   0               // ��׼֡
#define	EXT_FRAME                   1               // ��չ֡
#define CAN1_DATA_OK  				0x01	//�ɹ��յ�CAN1����
#define CAN1_DATA_TIMEOUT  			0x02	//����CAN1���ݳ�ʱgg
#define CAN_CHANNEL1                0x00
#define CAN_CHANNEL2                0x01


//can1��ϢID
//#define ID_FAULTCODE_ONE			0x18FECA31		//ֻ��һ��������ʱ��ID
#define ID_FAULTCODE_ONE_ENGINE		0x18FECA00		
//#define ID_BROADCAST				0x18ECFF31		//�㲥֡ID
#define ID_BROADCAST_ENGINE			0x18ECFF00
//#define ID_FAULTCODE_MULTI			0x18EBFF31		//���������ʱ��ID
#define ID_FAULTCODE_MULTI_ENGINE	0x18EBFF00	


typedef struct _MessageDetail
{
	unsigned char LEN;
	unsigned char FF;           	// �Ƿ��׼֡
	unsigned int  CANID;			// ������FF�й�
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

//-----�ⲿ����----------------------------------------------------------------
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data);
void MCU_CAN_Uart_Init(void); 
BOOL ReadMCU_CAN_UartData(uint8 **data, uint16* len);
uint16 WriteMCU_CAN_UartData(uint8 *data, uint16 Len);
#endif

