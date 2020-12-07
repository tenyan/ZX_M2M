/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
 * All right reserved
 *
 * �ļ�����: SystemProtocol.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪSystem����ģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-16, by lxf, �������ļ�
 *
 */

#ifndef _SYSTEMPROTOCOL_H
#define _SYSTEMPROTOCOL_H

//-----��������----------------------------------------------------------------
#define	PROTOCOL_VERSION		0x01	//ʹ�õ�Э��汾��		
#define	SUPPLY_CODE				0x00	//��Ӧ�̱��
#define	TERMINAL_TYPE			0x00	//�ն��ͺ�
#define CUSTOMER_CODE			0X00	//ʹ�÷����


#define CONNECT_MAX_REPEATS			2	//���������ϴ�����ظ�����
#define CONNECT_RESPOND_TIMEOUT		8	//���������ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
//��ƽ̨���͵�¼ָ�����Ľṹ��
typedef struct _Connect{
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint8  ucSucceedFlag;				//��¼�ɹ���־,1=�ɹ�,0=ʧ��
}stuConnect,*pstuConnect;

//��ƽ̨����״̬ͬ������
#define SSDATA_MAX_REPEATS		3	//�������������ϴ�����ظ�����
#define SSDATA_RESPOND_TIMEOUT	10	//�������������ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
#define SSDATA_REP_INTERVAL		30	//�������������ϱ����
typedef struct _STU_SSData_
{
    uint16 usTimer;						//�ϱ������ʱ��
    uint8 ucTimeToRepFlag;				//�����ϱ���ʱ���־,1=��Ч
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint16 usRepSn;						//�ϱ�����ˮ��
}STU_SSData;


#define MAX_NO_UPLOAD_GPRS_TIME		30	//û���ϴ��κ�GPRS���ݵ��ʱ��	
#define HEARTBEAT_MAX_REPEATS		2	//�����ϴ�����ظ�����
#define HEARTBEAT_RESPOND_TIMEOUT	8	//���������ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
//��ƽ̨���������Ľṹ��
typedef struct _STU_HeartBeat_
{
    uint16 usNoUploadGprsTimer;     	//û����ƽ̨�����κ����ݼ�ʱ
    uint16 usTimer;						//�ϱ������ʱ��
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
}stuHeartBeat;

#define WORKDATAREP_MAX_REPEATS		2	//���������ϴ�����ظ�����
#define WORKDATAREP_RESPOND_TIMEOUT	8	//���������ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
//��ƽ̨�ϴ��豸������������Ľṹ��
typedef struct _WorkDataRep{
	uint16 usTimer;						//�ϱ������ʱ��
	uint8 ucTrackModel;					//0x00����ʱ����׷��
										//0xFF�����β�ѯ

	uint8 ucTrackInterval;				//�ϴ������
										//׷��ģʽΪ0x00ʱ����ʾʱ�䣬��λ��1��
										//׷��ģʽΪ��0xFFʱ����ֵ��Ч��

	uint16 usTrackScope;				//�ϴ���Чʱ�䣺
										//׷��ģʽΪ0x00ʱ����ʾʱ�䣬��λ��1s
										//����ֵΪ0x00ʱ����ʾֹͣ��ע��
										//׷��ģʽΪ0xFF����ֵ��Ч��
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��									
	uint16 usRepSn;						//�ϱ�����ˮ��									
}stuWorkDataRep,*pstuWorkDataRep;

//��ƽ̨�ϴ�λ����Ϣ����Ľṹ��
typedef struct _PositionRep{
	uint16 usTimer;						//�ϱ������ʱ��
	uint8  ucTrackModel;				//׷��ģʽ,0x00:��ʱ����׷��,0x01:�Ⱦ�����׷��,0xFF:����׷��
	uint8  ucTrackInterval;				//׷���������õ�׷�ټ��,
										//׷��ģʽΪ0x00ʱ,��ʾʱ�䣬��λ��1��
										//׷��ģʽΪ0x01ʱ,��ʾ���룬��λ��0.1ǧ��
										//׷��ģʽΪ0xFFʱ,��ֵ��Ч��
	uint16 usTrackScope;					//׷����Ч���䣺
										//׷��ģʽΪ0x00ʱ����ʾʱ�䣬��λ��1s
										//׷��ģʽΪ0x01ʱ����ʾ���룬��λ��1ǧ��
										//׷��ģʽΪ0xFF����ֵ��Ч
										//����ֵΪ0x00ʱ����ʾ�ر�׷�ٹ��ܡ�
}stuPositionRep,*pstuPositionRep;

#define ALARM_MAX_REPEATS			2	//�����ϴ�����ظ�����
#define ALARM_RESPOND_TIMEOUT		8	//�����ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
#define ALARM_MAX_NUM				20	//��������
//��ƽ̨�ϴ�������Ϣ
typedef struct _Alarm{
	uint8 ucType;						//��������
	uint8 ucState;						//�ϱ�״̬,0=δ�ϱ���1=���ϱ�
	//uint8 aucData[22];					//����Я��������
}stuAlarm,*pstuAlarm;

typedef struct _AlarmRep{
	stuAlarm Alarm[ALARM_MAX_NUM];
	uint8 ucRepIndex;					//��ǰ�����ϱ��ı�������,0xff��ʾû����Ҫ�ϱ��ı���
	uint8 ucRepeats;					//�ظ��ϱ�����
	uint8 ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8 ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint8 ucNewAlarmFlag;				//�µı������鷢��,1=��,0=��
	uint16 usRepSn;						//�ϱ�����ˮ��
	
}stuAlarmRep,*pstuAlarmRep;

#define DTC_MAX_REPEATS			2	//DTC�ϴ�����ظ�����
#define DTC_RESPOND_TIMEOUT		8	//DTC�ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s
typedef struct _DTCRep{
	uint8 ucRepeats;					//�ظ��ϱ�����
	uint8 ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8 ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint16 usRepSn;						//�ϱ�����ˮ��
	
}stuDTCRep,*pstuDTCRep;

#define ACCSTATE_MAX_REPEATS		2	//���������ϴ�����ظ�����
#define ACCSTATE_RESPOND_TIMEOUT	8	//���������ϴ���ȴ���Ӧ��ʱ���ʱ��,��λ:s

//���ػ���Ϣ�ϱ��ṹ��
typedef struct _AccRep{
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=�ȴ��ϱ�,2=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint8  ucSucceedFlag;				//����ָ��ͳɹ���־,1=�ɹ�,0=ʧ��
	uint8 aucData[23];					//�ϱ�������
}stuAccRep,pstuAccRep;


//���ػ���Ϣ�ϱ��ṹ��
typedef struct _McuResp{
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��

}stuMcuResp,pstuMcuResp;

//����ָ���ϱ��ṹ��
typedef struct _SleepRep{
	uint8  ucRepeats;					//�ظ��ϱ�����
	uint8  ucRepFlag;					//���ϱ���־,1=���ϱ�
	uint8  ucRespondTimer;				//�ȴ�ƽ̨�ظ���ʱ��
	uint8  ucSucceedFlag;				//����ָ��ͳɹ���־,1=�ɹ�,0=ʧ��
}stuSleepRep,pstuSleepRep;

//�̼������ṹ��
#define FIRMWARE_PACKET_LEN			1024//�̼��������ĳ���
typedef struct _FirmwareUpdate{
	uint8 uctype;            //������ʽ 0-ѯ������,1-ǿ������
	uint8 ucdev;			//������Ŀ���豸0x00:�ն�,0x01:������,0x02:��ʾ��,0x03:Э������
	uint8 aucSerIp[4];		//������IP��ַ
	uint16 usSerPort;		//�������˿ں�
	uint8 ucSerProtocolType;	//Э������
	uint8 ucSWNameLen;		//�����̼����Ƴ���
	uint8 aucSWName[50];	//�����̼�����
	uint8 ucSWVersionLen;	//�����̼��汾�ų���
	uint8 aucSWVersion[10];	//�����̼��汾��
	uint32 uiSWSize;		//�����ļ��Ĵ�С
	uint32 uiCrc;			//���������ļ�CRC32У����
	uint16 usPackets;		//�����ļ��ܰ���
	uint16 usRequestPacketSn;//����İ����
	uint16 usLastPacketLen;		//���һ�����ݵĳ���
	uint16 usTimeoutCnt;	//������ʱ������,���10������û��������������
	uint8 ucRepeats;		//���������ظ�����
	uint8 ucStep;			/*�������裬0=����,
	                                    1=�յ�����֪ͨ�����ӵ�����������,
	                                    2=���������������ɹ�,��������,
	                                    3=���������ѷ���,�ȴ���������Ӧ
	                                    4=���������ɹ�,��������������,
	                                    5=�������������������ѷ���,�ȴ���������Ӧ
	                                    6=�������������,����ƽ̨�ϱ��°����,
	                                    7=��ƽ̨�ϱ��°���������ѷ���,�ȴ�ƽ̨��Ӧ
	                        */
	uint8 ucRet;			/*�°����0=�ɹ�,1=ʧ��*/                        
	uint8 ucUpgradeRequestTimer;		//��������ʱ��ʱ��
	uint8 ucUpgradeRequesRepeats;		//���������ظ�����
	uint8 ucPacketRequestTimer;			//��������������ʱ��ʱ��
	uint8 ucPacketRequesRepeats;		//���������������ظ�����
	uint8 ucSendRetTimer;				//�������������ʱ��ʱ��
	uint8 ucSendRetRepeats;				//������������ظ�����
}stuFirmwareUpdate;



typedef struct _SerAddr{
	uint8 aucIp[4];
	uint16 usPort;
	uint8  ucDnLen;
	uint8  aucDn[32];
	uint8  ucProtocolType;
}stuSerAddr;


uint8 BuildMsgHead(uint8 *buf,uint8 ucMsgType,uint16 usMsgBodyLen, uint8 ucFlag, uint16 usSn);
uint16 BuildSS(uint8 *buf);
void SYS_Reset(uint8 delay);
uint32 BuildState(void);
void SYS_POWERLED_Display(void);
void SYS_PowerOff_Reset(void);
void Sys_GPSState_Change(void);
uint16 Build_ZX_TCD(uint8 *buf);

#endif








