/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GsmModule.h
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGSMģ�����ӿ�ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-05-09 by , �������ļ�
 *
 */

#ifndef GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138
#define GSMMODULE_H_20110609_BA0E33A5_0319_46D9_8BA4_432119669138

#include "GsmHardWareLayer.h"
//-----��������----------------------------------------------------------------
#define TASK_GSM_RECV_STK_SIZE        300  //GSMģ���������ջ��С
#define TASK_GSM_RECV_ID              4    //GSMģ���������ID
#define TASK_GSM_RECV_PRIO            4    //GSMģ������������ȼ�

#define TASK_GSM_SERVICE_STK_SIZE     100  //GSMģ��������ջ��С
#define TASK_GSM_SERVICE_ID           7    //GSMģ��������ID
#define TASK_GSM_SERVICE_PRIO         7    //GSMģ�����������ȼ�

#define SIMCARD_Satate_Timer                 180    //�ж�SIM����ʱʱ��
#define Recv_4GMODULE_TIMEOUT                180    //������������������Ϣ���ʱʱ��

#define SEND_TO_A5_TIMER                     10
#define SEND_TO_A5_BUFF_LEN                  1024
#define SEND_TO_A5_BUFF_NUM                  5      //CAN�洢������
#define CAN_FRAME_MAX_NUM                    72     //û��CAN������Դ�ŵ����֡��
#define CAN_SEND_MIN_TIMER                   125    //��4Gģ�鷢����С���
#define CAN_SEND_MAX_COUNTER                 8      //1�뷢��������


typedef struct _STU_A5Comm_
{
    uint8 ucSendTimer;
	uint8 aSenDataBuffFlag[SEND_TO_A5_BUFF_NUM];  //0-δ�д�������,1-�������,2-���ڽ���CAN����,0xFF-���ճ�ʱ����������
    uint8 aCANDatabuff[SEND_TO_A5_BUFF_NUM][SEND_TO_A5_BUFF_LEN];  //B0-Tag,B1-B2=Len,V=CAN���� 1byteCan֡���� һ�����������Դ洢72��CAN֡
    uint8 ucSN;                //��ˮ��
	uint16 usmSec;             //����˵����B15-֡��ʽ0-��չ֡,1-��׼֡B14-B11��֡���ݳ���B10-B0��ƫ��ʱ��(����)
    uint8 ucCommflag;          //���İ���MCUͨѶ״̬��־ 0-δͨѶ ��1-ͨѶ
    uint8 ucCommErrTime;       //ͨѶ�жϼ�ʱ�� ��ʼ0��
}STU_A5Comm;

//����ʱ��ṹ��
typedef struct _STU_A5_Date_
{
  uint8  ucYear;             //��
  uint8  ucMon;              //��
  uint8  ucDay;              //��
  uint8  ucHour;             //ʱ
  uint8  ucMin;              //��
  uint8  ucSec;              //��
  uint16 usmSec;             //����
  uint16 usmSectemp;         //������ʱ����
}STU_A5_Date, *PSTU_A5_Date;

typedef struct _STU_GSM_State_
{
    uint8 ucRunState;          //4G ģ�鿪��״̬ 0-δ����,1-������������,2-�����ɹ�,3-ģ�鴮��ͨ���쳣,
                                                //4-ģ������,5-�ѽ�������
    uint8 ucRecvTimeErr;       //���ڽ���4Gģ�������жϼ�ʱ��
    
}STU_GSM_State;

typedef struct _STU_SYS_LEDState_
{
    uint8 ucLed_GPRS;  //GPRS����״̬ 0-δ����,1-����
	uint8 ucLed_GPS;   //GPS����״̬  0-δ��λ,1-��λ
	uint8 ucLed_WiFi;  //WiFi����״̬
	uint8 ucLed_ETH;   //ETH����״̬
	uint8 ucreserve;   //����
}STU_SYS_LEDState;

void TaskUart3Recv(void *pvData);
void TaskGsmService(void * pvData);
void A5_MCU_SendReqCmdData(uint8 ucCommand, uint8 ucSN, uint8 ucResult,uint8 ucSrc);
void A5_Deal_RecVA5_Data(uint8 *ptr, uint16 uslen);
void A5_MCUSendCANDataToA5(void);
void A5_AddCanFrameToBuff(uint32 id, uint8 ucFF, uint8 ucDataLen, uint8 *data);
void AccountSysDate(void);
void A5_CommErr_Judge(void);
void ModemClose(void);
#endif





























