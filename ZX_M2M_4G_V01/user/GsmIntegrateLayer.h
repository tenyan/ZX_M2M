/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmIntegrateLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ���ۺϲ��ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by  �������ļ�
 *
 */

#ifndef GSMINTERGRATELAYER_H_20110609_5B055A0D_2784_442E_A1B3_4E9B7914E18B
#define GSMINTERGRATELAYER_H_20110609_5B055A0D_2784_442E_A1B3_4E9B7914E18B

#define  GSM_SERVICE_NUMBER   	10    //gsm �����б�ĳ���
//-----��������----------------------------------------------------------------
#define SMS_SER_NONE 	0//��sms ����
#define SMS_SER_IDLE 	1//sms�������,���Է���sms
#define SMS_SER_BUSY 	2//sms������æ,���ܷ���sms 




//-----�ṹ����----------------------------------------------------------------
//GSM ģ������ṹ��
typedef struct _STU_GSM_OPERATE_
{
	uint8 GsmEnable;						//GSMģ��ʹ��,0=��ֹ,1=ʹ��
	uint8 GsmOperate;						//GSM������0=����,1=����,2=�ػ�
	uint8 GsmSleepOperate;					//���߲���,0=����,1=����,2=����

	uint8 LinkOperate[3];					//3�����ӵĲ���,0=����,1=���ӻ򱣳�,2=�Ͽ�,3=����
	uint8 FtpOperate;						//ftp����, 0=����,1=���ӻ򱣳�,2=��������,3=�Ͽ�
	uint16 usFtpGetLen;						//����ftp���ݵĳ���
}STU_GsmOperate;

#if 0
//����绰�ṹ��
typedef struct _STU_DIAL_NUMBER_
{
	uint8 Request;						//0:�޲�������,1:�в�������
	uint8 Number[15];					//�绰����
	uint8 Len;							//���볤��
	uint8 State;						//����״̬  0:����,1=���ڲ���,2=�ѽ�ͨ
}STU_DialNumber;

//�����绰�ṹ��
typedef struct _STU_ANSER_NUMBER_
{
	uint8 IncomeFlag;					//�����־,0:������,1:������
	uint8 Number[15];					//�绰����
	uint8 Len;							//���볤��
	uint8 State;						//����״̬  0:����,1:���ڽ���,2=�ѹҶ�,
}STU_AnserNumber;
#endif
//SMS �ṹ��
typedef struct _STU_SMS_
{
	uint8 SmsSerState;	//SMS ����״̬, SMS_SER_IDLE, SMS_SER_BUSY, SMS_SER_NONE 
	uint8 Index;		//��ǰ���ڷ��͵�sms�ڷ��ͻ����е�����.
	uint8 SendState;	//���ŷ���״̬, 1=�ѷ���, ��δ�ɹ�;2=�ѷ��ͳɹ�
}STU_Sms;

//���������ṹ��
typedef struct _STU_TTS_
{
	uint8 BusyFlag;		//æ��־, 1=���ڲ���, 0=����
	uint32 PlayFlag;	//������־, bit0~bit31�ֱ��Ӧ��1������32��, 1=������Ӧ���, 0=������
	uint8 PlayIndex;	//��ǰ���ڲ������������,ȡֵ0~31
	uint16 PlayOverTimer;//�ȴ����Ž�����ʱ��
} STU_TTS;

#define GSMSERVICE_TYPE_OPEN				1
#define GSMSERVICE_TYPE_CLOSE				2
#define GSMSERVICE_TYPE_RESET				3
#define GSMSERVICE_TYPE_WAKE				4
#define GSMSERVICE_TYPE_SLEEP				5
#define GSMSERVICE_TYPE_OPENLINK			6
#define GSMSERVICE_TYPE_CLOSELINK			7
#define GSMSERVICE_TYPE_SENDGPRS			8
#define GSMSERVICE_TYPE_SENDSMS				9
#define GSMSERVICE_TYPE_RCVSMS				10
#define GSMSERVICE_TYPE_DIALPHONENUMBER		11
#define GSMSERVICE_TYPE_ANSERPHONENUMBER	12
#define GSMSERVICE_TYPE_QUERYSTATE			13
//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

void QueryGsmState(void);

#endif

































