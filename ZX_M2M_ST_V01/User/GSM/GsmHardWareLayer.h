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
 * 2019-05-09 by , �������ļ�
 *
 */
 
#ifndef GSMHARDWARELAYER_H_20110609_A228C809_39C9_43EE_87BF_92CE3ACDEB20
#define GSMHARDWARELAYER_H_20110609_A228C809_39C9_43EE_87BF_92CE3ACDEB20

//-----��������----------------------------------------------------------------
#define ModemRSTHigh() 			(GPIO_SetBits(GPIOC, GPIO_Pin_8))		//GSMģ�鸴λ
#define ModemRSTLow() 			(GPIO_ResetBits(GPIOC, GPIO_Pin_8)	   //GSMģ�鸴λ

#define ModemDTRHigh() 			(GPIO_SetBits(GPIOC, GPIO_Pin_9))		//GSMģ������
#define ModemDTRLow() 			(GPIO_ResetBits(GPIOC, GPIO_Pin_9))     //GSMģ�黽��
#define ModemPwrHigh() 			(GPIO_SetBits(GPIOA, GPIO_Pin_15)) 		//���߹���(PA15)
#define ModemPwrLow() 			(GPIO_ResetBits(GPIOA, GPIO_Pin_15))   	//���Ͷϵ�(PA15)
#define ModemOpenHigh()			(GPIO_ResetBits(GPIOB, GPIO_Pin_14))  	//����modem��IGN,����(PB14)
#define ModemOpenLow() 			(GPIO_SetBits(GPIOB, GPIO_Pin_14))  	//����modem��IGN,�ػ�(PB14)
#define Modem_Status()			(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))		//���ػ�״̬
#define GsmLedOn()          	(GPIO_ResetBits(GPIOE, GPIO_Pin_2))   	//���� GSN led �� (PE2)
#define GsmLedOff()				(GPIO_SetBits(GPIOE, GPIO_Pin_2))   	//���� GSN led �� (PE2)

//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
void GSM_Uart_Init(void);
void GSM_UART_Write(uint8 *Data, uint16 len);
void GSM_GPIO_Pin_Init(void);
uint8 WriteGsmUartData(uint8 *Data, uint16 Len);
BOOL ReadGsmUartData(uint8 **data, uint16* len);

#endif

































