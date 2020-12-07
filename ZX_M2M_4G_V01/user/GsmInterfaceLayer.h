/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmInterfaceLayer.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ��ӿڲ��ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-03-09 by, �������ļ�
 *
 */

#ifndef GSMINTERFACE_H_20110609_7ABB65AE_41C0_48D1_912F_B2DD60D08089
#define GSMINTERFACE_H_20110609_7ABB65AE_41C0_48D1_912F_B2DD60D08089

#include "GsmProtocolLayer.h"  //������벻ͨ������,�Ժ�����취�õ�

//-----��������----------------------------------------------------------------


#define SMS_NUMBER_TYPE_MAS					1	//���ź�������--MAS��
#define SMS_NUMBER_TYPE_MOBILE				0	//���ź�������--��ͨ�ֻ���
#define SMS_NUMBER_LEN_MAS					15	//���ź��볤��--MAS��
#define SMS_NUMBER_LEN_MOBILE				11	//���ź��볤��--��ͨ�ֻ���

//-----�ṹ����----------------------------------------------------------------
typedef struct _GPRS_SEND_DATA_QUEUE_
{
	uint8 ucLinkNum;
	uint16 usLen;
	uint8 buff[GPRS_SEND_BUFF_LEN];
	uint8 priority;//���ȼ�:1=������ȼ�,2=�θ�
	uint8 flag;  //�Ƿ������, 0=�����, 1=������
}STU_GprsSendDataBuff;

typedef struct _STU_SMS_SEND
{
	uint8 aucDesNumber[15];
	uint8 ucNumberLen;
	uint8 aucSmsSendBuff[SMS_SEND_BUFF_LEN];//���Ͷ������ݻ���
	uint16 usDataLen;
	uint8 flag;  //�Ƿ������, 0=�����, 1=������, 2=�ѷ���,��δ���
}STU_SmsSend;

typedef struct _STU_SMS_RCV
{
	uint8 ucNumberLen;
	uint8 aucSrcNumber[15];
	uint16 usDataLen;
	uint8 aucSmsRcvBuff[SMS_SEND_BUFF_LEN];//���Ͷ������ݻ���
	uint8 flag;			//�ö��Ŵ������,���������ٴζ����Ų�����������, 0=�������, 1=������
}STU_SmsRcv;
//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

void   InitGsmModule(void);
void   InitSimSeq(void);


void   SendSms(void);
void   RecvSms(void);

void   SendGprs(void);
void   RecvGprs(void);
void   Connect(void);




#endif


