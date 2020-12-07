/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: can.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�Ϊcan.h�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-10-21, �������ļ�
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

#define POWERON_CAN()         (GPIO_SetBits(GPIOA, GPIO_Pin_2))  	//��CANģ�鹩��  PA2
#define POWEROFF_CAN()        (GPIO_ResetBits(GPIOA, GPIO_Pin_2))   //�ر�CANģ�鹩��  PA2
//#define POWERON_485()         (GPIO_SetBits(GPIOD, GPIO_Pin_12))  	//��CANģ�鹩��  PD12
//#define POWEROFF_485()        (GPIO_ResetBits(GPIOD, GPIO_Pin_12))  //�ر�CANģ�鹩��  PD12
#define CAN1_LED_OFF()        (GPIO_SetBits(GPIOA, GPIO_Pin_6))  	//can1 LED �ر�    PE4
#define CAN1_LED_ON()         (GPIO_ResetBits(GPIOA, GPIO_Pin_6))  	//can1 LED ����    PE4
#define CAN2_LED_OFF()        (GPIO_SetBits(GPIOA, GPIO_Pin_7))  	//can1 LED �ر�    PE5
#define CAN2_LED_ON()         (GPIO_ResetBits(GPIOA, GPIO_Pin_7))  	//can1 LED ����    PE5
//����ָʾ��
//#define BLU_LED_OFF()         (GPIO_SetBits(GPIOE, GPIO_Pin_6))  	//���� LED �ر�    PE6
//#define BLU_LED_ON()          (GPIO_ResetBits(GPIOE, GPIO_Pin_6))  	//���� LED ����    PE6





//can1��ϢID
#define ID_FAULTCODE_ONE			0x18FECA31		//ֻ��һ��������ʱ��ID
#define ID_FAULTCODE_ONE_ENGINE		0x18FECA00		
#define ID_BROADCAST				0x18ECFF31		//�㲥֡ID
#define ID_BROADCAST_ENGINE			0x18ECFF00
#define ID_FAULTCODE_MULTI			0x18EBFF31		//���������ʱ��ID
#define ID_FAULTCODE_MULTI_ENGINE	0x18EBFF00	
#define ID_HANDLE_TEST              0XFFFFFF       //�ֲֳ�����ID ȥ�����ֽ�


typedef struct _MessageDetail
{
	unsigned char LEN;
	unsigned char FF;           	// �Ƿ��׼֡
	unsigned int  CANID;			// ������FF�й�
	unsigned char  CANRE[8];
}MessageDetail;
//-----�ⲿ����----------------------------------------------------------------
void McuHwInit(void);
void CanWrite(uint8 ch, unsigned char FF, unsigned int ID, unsigned char len, unsigned char *data);
uint8 WaitForCAN1Message(uint16 timeout);
MessageDetail CAN1Read(void);
void Sleep_Can_Io(void);



#endif

