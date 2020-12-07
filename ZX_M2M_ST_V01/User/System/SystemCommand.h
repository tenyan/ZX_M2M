/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: SystemCommand.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪSystem����ģ��Э������ݽ�����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, �������ļ�
 *
 */

#ifndef _SYSTEMCOMMAND_H
#define _SYSTEMCOMMAND_H
//-----��������----------------------------------------------------------------
#define  MSG_HEADER_ALLDATA_LEN             16         	//Э����Ϣͷ����Ϊ16
#define  MSG_HEADER_DATA_LEN                12         	//Э����Ϣͷ�ܳ���-GPS����ID�������(4�ֽ�)
/********************************��������ID********************************************/
#define UP_CMD_ID_GETSERVERPARA				0x80		//��ȡ���ķ�����ͨ�Ų���ָ��
#define UP_CMD_ID_UPLOADHEARTBEAT			0x81		//�ն�����ͨѶָ��
#define UP_CMD_ID_RESPSERDEFCMD				0x82		//�ն�����Ӧ��ָ��
#define UP_CMD_ID_UPLOADPOSITIONINFOR		0x83		//�ϴ�λ����Ϣָ��
#define UP_CMD_ID_UPLOADPARA				0x84		//�ϴ��ն˲���ָ��
#define UP_CMD_ID_LANDON					0x85		//��¼ָ��
#define UP_CMD_ID_UPLOADWORKDATA			0x87		//�ϴ���������ָ��
#define UP_CMD_ID_UPLOADSLEEPNOTE			0x89		//����������ʾָ��
#define UP_CMD_ID_UPLOADALARMDATA			0x8A		//�ϴ�����ָ��
#define UP_CMD_ID_UPLOADACCSTATE			0x8D		//�ϴ����ػ���Ϣ
/**************************************************************************************/

/********************************��Ϣ���Ͷ���********************************************/
#define MSG_TYPE_MESSAGEACK					0x00		//ͨ�ñ�����Ӧ
#define MSG_TYPE_CONN_REQ					0x01		//��������
#define MSG_TYPE_CONN_RESP					0x02		//������Ӧ
#define MSG_TYPE_PUSH_DATA					0x03		//�ն������˷������ݻ��������ն˷�������
#define MSG_TYPE_ALERT						0x04		//�ն������˷������ѡ��澯��������Ϣ
#define MSG_TYPE_CMD_REQ					0x05		//�ն������˷����������󣬻��������ն˷�����������
#define MSG_TYPE_CMD_RESP					0x06		//���ն˶��������Ӧ
#define MSG_TYPE_PING_REQ					0x07		//�ն˶Է���˷��͵���������
#define MSG_TYPE_PING_RESP					0x08		//����˶��ն���������Ӧ
#define MSG_TYPE_DISCONNECT					0x09		//�ն˶Ͽ�����
#define MSG_TYPE_UPDATE						0x0A		//����֪ͨ
#define MSG_TYPE_UPDATE_ACK					0x0B		//������Ӧ
#define MSG_TYPE_REGIST_REQ					0x0C		//ע������
#define MSG_TYPE_REGIST_RESP				0x0D		//ע����Ӧ
#define MSG_TYPE_DEREG_REQ					0x0E		//ע������
#define MSG_TYPE_DEREG_RESP					0x0F		//ע����Ӧ
/********************************��������ID********************************************/
#define DOWN_CMD_ID_SERVERPARA				0x01		//�������ķ�����ͨ�Ų���ָ��
#define DOWN_CMD_ID_RESPGPSDEFCMD			0x02		//��������Ӧ��ָ��
#define DOWN_CMD_ID_GETPOSITIONINFOR		0x03		//��λָ��
#define DOWN_CMD_ID_SETPARA					0x04		//�趨����
#define DOWN_CMD_ID_FIRMWAREUPDATE			0x06		//�����̼�ָ��
#define DOWN_CMD_ID_QUERYPARA				0x07		//��ѯ����
#define DOWN_CMD_ID_QUERYWORKDATA			0x08		//��ѯ��������
#define DOWN_CMD_ID_SETWORKDATAREP			0x09		//���������ϴ�����
#define DOWN_CMD_ID_ATTENTION				0x0c		//��ע�ն�λ��/����ָ��
#define DOWN_CMD_ID_TRACK					0x0d		//׷��ָ��
#define DOWN_CMD_ID_CTRL					0x0f		//�ն˿���
/**************************************************************************************/
#define  SYS_SHUTDOWN_TIME                  10         	//�յ��ػ������10����Զ��ػ�
#define  SYS_GPSLOCATION_DATA_LEN           48

#define MSG_HEAD_LEN						13			//��Ϣͷ����
typedef struct _STU_SYSPactHeader_
{
   uint16 usMsgLen;			//��Ϣʣ�೤��,���������У���ֵ��ܳ��ȡ�ʣ�೤�Ȳ��������ڱ����ʣ�೤���ֶα�����ֽ�����
   uint8 aucId[7];			//�ն�ID
   uint8 ucMsgType;			//��Ϣ����
   uint8 ucFlag;			//���ұ��
   uint16 usSq;				//�������
   uint16 usMsgBodyLen;		//��Ϣ�峤��,Ϊ��Ϣ����
   uint8 ucSrc;				//��Ϣ��Դ
}STUSYSPactHeader,*PSTUSYSPactHeader;



//����һЩ��Ҫ��ʱ�ı���ͳһ����
typedef struct _SYS_Counter_
{
	uint16 usDailyReportCounter;	//�ϴ��ձ���Ϣ�����ʱ
	uint16 usAlarmReportCounter;	//�ϴ�������Ϣ�����ʱ
}STU_SYSCounter;

typedef struct _UnNomalGSMData
{
	uint8 aucUnpackData1[250];
	uint8 ucLen1;
	uint8 aucUnpackData2[250];//[0]:���ݳ���
	uint8 ucLen2;
	uint8 ucDealFlag;	//b0=1��һ��������ɣ�b1=1:�ڶ����������
}STUUnNomalData;

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
uint16 SYS_GPS_CommandAll_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr, uint8 ucKind);
//uint16 SYS_CollectModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
uint16 SYS_GPSModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
//uint16 SYS_MCUModule_ReceiveData_Execution(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucKind);
//uint16 SYS_SMS_CommandAll_Execution_Universal(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
void SYS_ParamRead(void);
uint16 EscapeMassage( uint8* src,uint16 srcLen);
void DealUnNormData(void);
void SaveEcuPara(void);
void ReadEcuPara(void);
void ReadEcuFaultAddrFromFlash(void);
void SerAddrChangeGsmRst(void);
void UpgradeManage(void);
void UpgradeTimer(void);
uint16 DealSerCmd(uint8* PtrTxt,uint16 usPtrTxtLen,uint8 ucSrcAddr,uint8 ucKind);
#endif




























