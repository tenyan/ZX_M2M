//�ļ�����Canconfig.h
//���ܣ� 
#ifndef _CANCONFIG_H_
#define _CANCONFIG_H_

#define CAN_FRAME_MAX_MUN       100
#define CAN_DATA_MAX_LEN        1024     //
#define CAN_PARAMGROUP_MAX_NUM   5      //������������
#define CAN_CFG_FILE_MAX_LEN    2048
typedef struct _STU_CanParamFrame
{
	uint32 uiID;            //֡ID    
	uint8 ucOptionByte;     //ѡ���ֽ�
	//uint32 uiTime;          //�ϴ�CAN���ݱ���ʱ�� ��λms  �ɷ�-��-���뻻��õ�
}STU_CanParamFrame;

typedef struct _STU_CanParam_Group{
    uint16 usTlvName;
	uint16 usTlvLen;
	uint16 usSamplingFreq;  //�ɼ�Ƶ��  ����ֵ<=200ms ��200msΪ׼
	uint16 usUpFreq;        //�������ϴ�Ƶ��
	uint8  ucCanNum;        //�������CAN����
	STU_CanParamFrame stuCanFrame[CAN_FRAME_MAX_MUN];
	
}STU_CanParam_Group;

typedef struct _STU_CanProjects
{
    uint8 ucParamGroupNum;           //���õ����еĲ������ܸ���
    STU_CanParam_Group stuParamGroup[CAN_PARAMGROUP_MAX_NUM];
	uint8 agroupSendFlag[CAN_PARAMGROUP_MAX_NUM];   //�����鷢�ͱ�ʶ  1-�ɼ���ɴ������� ,2-��Ҫ�ٲɼ�,0-�������\������
	uint8 agrouptype[CAN_PARAMGROUP_MAX_NUM];       //�ж� 1-�ɼ���5���ϱ�,2-���ղɼ�Ƶ���ϱ�
    uint8 aucCanRcvFlag[CAN_PARAMGROUP_MAX_NUM];    //��ʱ����ʱ�䵽����Ҫ�жϸò������Ƿ��յ���CAN֡ 1-�յ�,0-δ�յ�
	//
}STU_CanProjects;

typedef struct _STU_CanData
{
    uint16 usDataLen;
	uint8 aData[CAN_DATA_MAX_LEN];
	//uint16 usDataptr;
}STU_CanData;

typedef struct _STU_CanDataUp{
    uint16 usTlvName;        //CAN�������ʼ����ʱ����Ҫ����ֵ
	uint16 usTlvLen;         //���������������(�����ж��)
	uint16 usSamplingFreq;   //�ɼ�Ƶ��  ��Ӧ���õĲɼ�TLV�вɼ�Ƶ�� by lxf 20200316
	STU_Date Date;
	uint8 ucTlvNum;         //���������
	uint8 aCount[CAN_PARAMGROUP_MAX_NUM];  //��¼����  һ���ϱ�������5��
	uint16 usDataLen;        // һ�����ݵĳ���  ����������2byte
	
	uint8 aData[CAN_DATA_MAX_LEN];
	//STU_CanData stuCanData[3];	
	uint32 ustimer;         //ͨ�����յ���CAN��ʱ���ʱ
	//uint8 ucUpTimer;
}STU_CanDataUp;

typedef struct _STU_DateTime
{
  uint8  ucYear;             //��
  uint8  ucMon;              //��
  uint8  ucDay;              //��
  uint8  ucHour;             //ʱ
  uint8  ucMin;              //��
  uint8  ucSec;              //��
  uint16 usmSec;             //����
}STU_DataTime;

typedef struct _STU_Can_Message
{
    uint16 usmSec;
	uint32 CanID;
	uint8 ucLen;
	uint8 ucFF;           	// �Ƿ��׼֡
	uint8 data[8];
}STU_Can_Message;

void CanConfig_Data_Collect(STU_DataTime stuDate, uint8 *ptr);
uint16 CAN_GetCacheLocation_Function(uint8 ucGroupIndex,uint8 ucCanNumIndex);
uint8 CanConfig_Data_Send(void);
void CanConfig_Param_Init(void);
uint8 CANConfig_extract_Param(void);
void CAN_ClearCanDataUpbuff(void);
void CANConfig_ReadCANCFGFile_Init(void);
uint16 CANConfig_Read_CanParam(uint8* ptr);

#endif
