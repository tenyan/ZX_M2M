/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: SystemModule.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪSystem����ģ�����ж���ӿڲ��ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, �������ļ�
 *
 */

#ifndef _SYSTEMModule_H
#define _SYSTEMModule_H

#include "SystemProtocol.h"


#define HEARTBEAT_UpNUM         			6             //���������������ݵò�����������Ӧ��������
#define HEARTBEAT_WAITTIME      			10            //��������֮��ʱ����Ϊ10��


//-----��������----------------------------------------------------------------
//��Ϣ������Դ�豸����
#define SRCDEVICE_ID_GSM                   	1            //������GPRS
#define SRCDEVICE_ID_MCU                   	3            //MCUģ��
#define SRCDEVICE_ID_GPS                   	4            //GPSģ��
#define SRCDEVICE_ID_COLLECT               	5            //�ɼ�ģ��
#define SRCDEVICE_ID_SETTOOL               	6            //���ó���
#define SRCDEVICE_ID_A5_COM1                7
#define SRCDEVICE_ID_5CGPS                	0x0B
#define SRCDEVICE_ID_5FGPS               	0x0E
#define SRCDEVICE_ID_5FCGPS                	0x12
#define SRCDEVICE_ID_3GGPS                	0x02


/*
//�豸������ȡ������INDEX

//#define  INDEX_SPARE_SEVER_IP_PORT        0x0F         //���÷�����IP��Port 6�ֽ�
#define  INDEX_USER_NUMBER                1            //�ͻ����̴���
#define  INDEX_GPS_DEVICEID               2            //GPS�ն�ID�� 4�ֽ�
#define  INDEX_SEVER_IP_PORT              3            //������IP��Port 6�ֽ�
#define  INDEX_ANP                        4            //APN     APN����+APN����   �䳤
#define  INDEX_SMS_CENTER                 5            //����è����   ����è���볤��+����è��������
#define  INDEX_SEVER_DNAME                6            //����������   ��������������+����������
#define  INDEX_SEVER_DISTANCE             7            //�����
//#define  INDEX_PING_TIME                   3            //PING���ͼ��ʱ�� 2�ֽ�
//#define  INDEX_HEARTBEAT_TIME              4            //�������ʱ�� 4�ֽ�
//#define  INDEX_LOW_VOLTAGE                 5            //�͵�ѹ����ֵ 2�ֽ�
//#define  INDEX_SLEEPWAKEN_TIME             6            //���߻��Ѽ�� 2�ֽ�

//#define  INDEX_SLEEPBEFOR_TIME             7            //����ǰʱ���� 1�ֽ�
//#define  INDEX_OVERSPEED                   8            //���ٱ��� �ٶ�:Km/h  ʱ��:�� 2�ֽ�
//#define  INDEX_RS_BPS                      9            //���ڲ�����  1�ֽ�
//#define  INDEX_SMS_COUNT                  0x0A          //�Զ��ϱ��������� 1�ֽ� 0��ʾ���ϱ�  
//#define  INDEX_ALARM_SWITCH               0x0B          //������������   1�ֽ�
*/


#define  TASK_SYSTEM_STK_SIZE             400        //Systemģ�������ջ��С
#define  TASK_SYSTEM_ID                    5         //
#define  TASK_SYSTEM_PRIO                  5         //systemģ���������ȼ�

#define	TASK_SYSTIMER_ID					8
#define TASK_SYSTIMER_PRIO       			8
#define TASK_SYSTIMER_STK_SIZE				300  


//-----�ṹ����----------------------------------------------------------------
typedef struct _STU_tcp_
{
    uint8_t net[32];
    uint8_t port[8];
    uint8_t filename[22];
	uint8_t flag;
	uint8_t sum;
}STUSYStcp;

typedef struct _STU_SYSParamSet_
{
	uint8 aucDeviceID[7];				//�ն˵�ID
	uint8 ucAPNLen;						//APN����
	uint8 aucAPN[32];					//APN
	uint8 aucUser[32];					//M2Mƽ̨��¼�û���
	uint8 ucUserLen;					//M2Mƽ̨��¼�û�������
	uint8 aucPassword[32];				//M2Mƽ̨��¼����
	uint8 ucPasswordLen;				//M2Mƽ̨��¼���볤��
	uint8 aucSmsCenterNum[20];			//�������ĺ���
	uint8 ucSmsCenterNumLen;			//�������ĺ��볤��
	uint8 aucHostIP[4];                 //������IP��ַ
	uint8 aucSpareHostIP[4];           	//������IP��ַ
	uint16 usHostPort;              	//�����Ķ˿�
	uint16 usSpareHostPort;             //�����Ķ˿�
	uint8  ucHostProtocolType;			//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��
	uint8  ucSpareHostProtocolType;		//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��
	uint32 uiHeartbeatInterval;			//�����������λ����0x0000-����������,Ĭ���������Ϊ30��
	uint8  ucMaxLandonRepeats;			//����¼�ظ�����
	uint32 uiLandonFailedMinRepeatsInterval;//��¼ʧ����С���Լ��
	uint32 uiLandonFailedMaxRepeatsInterval;//��¼ʧ��������Լ��
	uint16 usSmsRcvTimeout;				//���Ž��ճ�ʱʱ�䣬��λ����
	uint16 usCanBrt;					//����CAN���߲�����
	uint16 usCanFmt;					//����CAN���ĸ�ʽ
	uint32 auiCanId[90];				//CAN ID �������ã�4�ֽ�һ��
	uint8  ucCanIdNum;					//can id ����
	uint16 usSleepBeforSlot;           	//��������ʱ��,��λ:s
	uint16 usSleepWakeSlot;				//�����ڼ䶨ʱ���Ѽ������λ����
	uint8  ucSSRepSlot;					//�ն˻���״̬ͬ�������Զ����ͼ����λ����
	uint8  aucSim[6];					//SIM���ţ������λ��0,
	uint8  ucHostDnLen;					//��������������
	uint8  aucHostDn[32];				//����������
	uint8  ucSpareDnLen;				//��������������
	uint8  aucSpareDn[32];				//����������
	uint8  aucDns[4];					//dns
	uint16 usHwVersion;					//Ӳ���汾�ţ���V1.5��ʾΪ0x0105
	uint16 usMainPwrRateVol;			//���Դ���ѹ��λ��0.1V
	uint16 usBatRateVol;				//�ն˵�ض��ѹ��λ��0.1V	
	
	uint8  ucCanErrTime;				//CAN�����ж�ʱ��
	uint8  ucCanOKTime;					//CAN�ָ������ж�ʱ��
	uint8  ucPwrOffTime;				//�ն˶ϵ�ʱ������
	uint8  ucPwrOnTime;					//�ն��ϵ�ʱ������
	uint8  ucPwrLowVol;					//�ⲿ��Դ�͵�ѹ������ֵ����λ��1%
	uint8  ucPwrLowTime;				//�ⲿ��Դ�͵�ѹ������ʱ���������λ��1s
	uint8  ucBatLowVol;					//�ڲ���Դ�͵�ѹ������ֵ����λ��1%
	uint8  ucBatLowTime;				//�ⲿ��Դ�͵�ѹ������ʱ�����
	uint8  ucGpsAntErrTime;				//�ն����߹��ϱ�����ʱ���������λ��1s
	uint8  ucGpsAntOKTime;				//�ն����߹��ϱ����Ľ��ʱ���������λ��1s
	uint8  ucGpsModuleErrTime;			//�ն�GPSģ����ϱ�����ʱ�����
	uint8  ucGpsModuleOKTime;			//�ն�GPSģ����ϱ��������ʱ���������λ��1s	
	uint8  ucSpeedOver;					//��ʾ���ٱ�����ֵ����λ��1KM/H
    uint8  ucSpeedOverLastTime;			//���ٱ�����ʱ���������λ��1s
    uint8  ucTransportCar;				//�������ƶ����ϳ�������������ֵ����λ��1KM

	uint8  ucDeviceWorkTimeRepCfg;		//�豸����ʱ���ͳ�����ò���
	uint8  ucWorkDataRepModel;			//�������������������ݵ����ϴ�ģʽ	0x00��Ĭ�ϣ���ʱ�����ϴ�
										//								    0x01������������ĳ���������ı�Ƶ����ΪƵ�ʷ��ͣ�
										//                                  0xFF�����Ե����ϴ�ģʽ����

	uint8  ucWorkDataRepInterval;		//�������������������������ʱ�䣬��λ��1��
	uint8  ucPosiInforRepModel;			//λ����Ϣ�����ϴ�ģʽ
	uint8  ucPosiInforRepInterval;		//λ����Ϣ�ϴ����
}STUSYSParamSet,*PSTUSYSParamSet;
//-----�ṹ����----------------------------------------------------------------


typedef struct _STU_SYSMsgBus_
{
    uint8 ucSrcDevice;	//��Ϣ��Դ�豸  1=������GPRS��2=����; 3=MCUģ��; 
                        //4=GPSģ�飻5=�ɼ�ģ��;  6=���ó���
    uint8 ucKind;       //��Ϣ����
    uint8 ucPrior;      //��Ϣ���ȼ�  (��ʱ���� ��0)
    uint8 ucResv;       //��ʱ���� ��0
    uint16 usSize;      //��Ϣ����,�ó��Ȳ���������4��������У�����ݵĳ��ȣ�   ���ȿ���Ϊ0��
    uint8* pMsgPacket;  //��Ϣ��ָ��
    uint8 ucCheck;      //��ϢУ��ֵ  �����ۼӺͺ�ȡ���ֽڷ�ʽУ�飬��ucSrcDevice��ʼ�ۼ���У���֮ǰ

}STUSYSMsgBus, *PSTUSYSMsgBus;



typedef struct _STU_System_
{
    uint8 ucOnline;                 //���߱�־,��¼�ɹ���ʾ���ߣ�0=δ����,1=����
    uint8 ucDebugPrint;             //����״״̬  
									//Bit3	1�����SYS��ص�����Ϣ��0�������
									//Bit2	1�����MCU CAN��ص�����Ϣ��0�������
									//Bit1	1�����GPS��ص�����Ϣ��0�������
									//Bit0	1�����Modem��ص�����Ϣ��0�������
    uint8 ucShutDownTime;           //�豸�յ��ػ������ȴ��ػ���ʱ�䣬��λ5��
    uint32 uiPingID;                //����ping����ʱ������
    uint8  ucRstTime;               //�����������߶����·���λ����ʱ����Ҫ��ʱ20���λϵͳ��ȷ�����ſ���ɾ����
    uint8 ucSerAddrChangeFlag;		//��������ַ(IP���������˿ں�)�ı��־, 1=�ı�,0=δ�ı�
    uint8 ucResetModem;				//��λGSMģ��,1=��λ, 0=����λ
	uint8 ucCorrectTimeOk;			//1:ʱ���Ѿ�У׼, 0:ʱ��δУ׼

	uint32 uiAccOffTimer;           //ACC�رպ��ʱ�� ��ʱ��>=�������߼��ʱ, ֪ͨRTC�����ϵ�10����
	uint32 uiResetTimer;            //��GPS��ʼ����ʼ��ʱ ʱ��>=24Сʱʱ, ֪ͨRTC�����ϵ�
    uint8 ucAccFlag;                //ACC��ON��ΪOFF��־λ
	uint8 ucWDTState;				//���Ź�״̬,0xaa=����,����=δ����
	
}STUSystem,*PSTUSystem;


//-----�ⲿ����----------------------------------------------------------------
extern STUSYSParamSet g_stuSYSParamSet;
//-----�ⲿ����----------------------------------------------------------------
uint8_t Check_DataSum(uint8_t *padd,uint16_t len);  //�ۼӺ�У���ж�����

//-----�ⲿ����----------------------------------------------------------------
uint16 SYS_GPS_CommandAll_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
//uint16 SYS_GetGPS_CollectData(uint8_t* PtrTxt);
void   SYS_ParamRead(void);
void   SYS_ParamWrite(void);
void   GoToUpdate(void);
void   SYS_PutDataQ(void* ptr);
void   SYS_TimerCount_NoDelay(void);
void   SYS_TimerCount_Delay(void);
void   TaskSYS(void *pdata);
void   TaskSysTimer (void *pData);
PSTUSYSParamSet SYS_GetParam(void);     //��ȡ�豸�����Ľӿں���
void ClearNoUploadGprsTimer(void);
uint8 IsCorrectTimeOk(void);
void SetCorrectTimeOk(void);
uint16 RestoreTranslateData(uint8 *src,uint16 srcLen);
uint16 TranslateData( uint8* src,uint16 srcLen);	
void SYS_ResetSleeptime(void);
uint32 GetCrc32(void);

#endif
