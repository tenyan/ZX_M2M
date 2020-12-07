//�ļ�����Mcu.h
//���ܣ� 
#ifndef _MCU_H_
#define _MCU_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "McuHW.h"
#include "GpsModule.h"

#define	TASK_MCU_CAN_ID						9
#define TASK_MCU_CAN_PRIO       			9
#define TASK_MCU_CAN_STK_SIZE				500

#define MCU_DATA_LENGTH						1460 //��ͨMCU���ݵĳ���, MCU�����˿���������ʾ���İ汾����Ϣ2013-05-03
#define TIME_COMM_UNNORMAL          		120	 //ͨ���쳣�ж�ʱ��

#define MCU_STATE_NORMAL					0	//��MCUͨ������
#define MCU_STATE_UNNORMAL					1	//��MCUͨ���쳣

//#define MCU_CAN_BRT		                    BPS_500K//CANͨ�ŵĲ�����
#define MAX_CAN_FRAME_NUM					90	//������յ����CAN֡����
#define MAX_ACTIVE_FAULTCODE_NUM		    40	//���Խ��յ���༤�����������
#define MAX_ACTIVE_FAULTCODE_NUM_ENG 	    30
 
#define TIME_FAULTCODE_ACTIVE			    40	//�����뼤��ʱ�䣬������ʱ�������û�г�������Ϊ�ù�����ʧ
#define CAN_ECM_ADDR						0x00//��������ַ
#define CAN_CONTROLER_ADDR					0x31//��������ַ


/************************MCUģ����Ϣ���Ͷ���˵��**********************************************/
#define MCU_MSG_KIND_CAN1COMM_ERR           1	//canͨ���쳣
#define MCU_MSG_KIND_CAN1COMM_OK            2	//canͨ������
#define MCU_MSG_KIND_CAN2COMM_ERR           3	//canͨ���쳣
#define MCU_MSG_KIND_CAN2COMM_OK            4	//canͨ������
#define MCU_MSG_KIND_MCUDATA_CHANGE			5	//MCU���ݱ仯
#define MCU_MSG_KIND_CAN1_RCV_STOP			6	//CAN1����ֹͣ
#define MCU_MSG_KIND_CAN1_RCV_START			7	//CAN1���տ�ʼ
#define MCU_MSG_KIND_CAN2_RCV_STOP			8	//CAN2����ֹͣ
#define MCU_MSG_KIND_CAN2_RCV_START			9	//CAN2���տ�ʼ


#define MCU_ALARM_CNT 						12
#define MAX_CAN1_RCV_ERR_TIME				3000//can1���������ݳ�ʱʱ��,��λ:10ms
#define MAX_CAN2_RCV_ERR_TIME				1000 //can2���������ݳ�ʱʱ��,��λ:10ms

#define MCU_GPSCMD_SEND_TIME                15
#define MCU_GPSSTATE_SEND_TIME              50
#define MAX_CAN_FaultCode_NUM				15 //������յĹ��������CAN֡����


#define MultiFrame_TempLEN                  140
#define Control_Faultcode_MAX_NUM            30
#define Control_CANFrame_ID                 0x1ADC08C1
#define VIN_BUFFER_MAX_SIZE               30

typedef struct _STU_CommState{
	uint8  ucCan1RcvErr;					//Can1�������ݴ���,0:û���յ�����,1:�յ�����
	uint16 usCan1RcvErrTime;				//Can1û���յ����ݼ�ʱ	
	uint8  ucCan1CommState;					//Can1��������״̬ͨ��״̬��1:ͨ���쳣,0:ͨ������
	uint8  ucRcvCan1DataFlag;				//Can1��������״̬ 1:�յ���CAN1���ݣ�2:û���յ�CAN1����

	uint8  ucCan2RcvErr;					//Can2�������ݴ���,0:û���յ�����,1:�յ�����
	uint16 usCan2RcvErrTime;				//Can2û���յ����ݼ�ʱ	
	uint8  ucCan2CommState;					//Can2ͨ��״̬��1:ͨ���쳣,0:ͨ������
	uint8  ucRcvCan2DataFlag;				//Can2��������״̬ 1:�յ���CAN1���ݣ�2:û���յ�CAN1����

	uint8  ucSleepState;					//����״̬,0=δ����, 1=����

}STU_CommState,*PSTU_CommState;	


typedef struct _STU_McuCmd{

	uint8 ucEcuFeedBackBindState;		//ECU�����İ�״̬:1=��, 0=�����
	uint8 ucEcuFeedBackLockState;		//ECU����������״̬:1=����, 0=����
    uint8 ucECURemoveBindingFlag;       //�������־ 0-����Ҫ��� 1-��Ҫ���
	uint8 aucECURemoveBinding[4];       //�������
    uint8 LockControlFlag;              //��������ģʽ���� 20100826 
                                        //B0 : 0=����Ҫ���ͼ��ģʽ����;1=��Ҫ���ͼ��ģʽ
                                        //B1 : 0=���ģʽ�ر�; 1=���ģʽ��
                                        //B2 : 0=����Ҫ����һ����������;1=��Ҫ����һ������
                                        //B3 : 0=ȡ��һ����������;1=ִ��һ������
                                        //B4 : 0=����Ҫ���Ͷ�����������;1=��Ҫ���Ͷ�������
                                        //B5 : 0=ȡ��������������;1=ִ�ж�������
    uint8 ucRespSerFlag;	   //��Ӧƽ̨��־,
                                        //B0=1:��Ҫ�ϱ�ƽ̨һ�������ɹ�,
                                        //B1=1:��Ҫ�ϱ�ƽ̨���������ɹ�,  
                                        //B2=1:��Ҫ�ϱ�ƽ̨һ�������ɹ�, 
                                        //B3=1:��Ҫ�ϱ�ƽ̨���������ɹ�,
                                        //B4=1:��Ҫ�ϱ�ƽ̨�������ģʽ�ɹ�,
                                        //B5=1:��Ҫ�ϱ�ƽ̨�رռ��ģʽ�ɹ�,
                            
   	uint16 usLockOneSq;		   //ƽ̨�·���һ��������ˮ��
	uint16 usLockSecSq;		   //ƽ̨�·��Ķ���������ˮ��
	uint16 usUnLockSq;		   //ƽ̨�·��Ľ�����ˮ��
	uint16 usMonitorSq;		   //ƽ̨�·������ü��ģʽ��ˮ��

	uint16 usSendMCULockTimer; //����MCU���������ʱ��
	uint16 usSendGPSDataTimer; //��ʱ����GPS��λ���ݼ�ʱ��
	uint32 uiFs;               //У��õ��Ľ��,��Ҫ����
	uint32 uiKey;              //keyֵ  Ĭ��Ϊ 0x18FE0AF4
	uint8  ucCmdCheckFlag;     //MCU�ظ�������У������־,51-�ɹ�;52-ʧ��

	uint8 ucSendCANheartbeatFlag;   //��ECU����CAN������־ 0-����, 1-������
    uint8 ucSendLockCount;          //Ϋ����ECU������������ Ĭ��Ϊ3��
	uint8 ucEngineUpEnable;         //B0:����DB���� 0-�ر�,1-���� Ĭ�Ϲر�;B1:����DB���� 0-�ر�,1-���� Ĭ�Ϲر�
	uint8 ucVinValidFlag;           //������VIN�������:0-vinƽ̨���ùر�,1-vinƽ̨���ü��� 
	uint8 ucVinLen;                 //������VIN����
	uint8 aucVin[VIN_BUFFER_MAX_SIZE];	

}STU_McuCmd,*PSTU_McuCmd;

typedef struct _STU_CanFrame{
	uint32 id;
	uint8 aucData[8];
}STU_CanFrame;

typedef struct _faultCode{
	uint32 faultCode;						// 4�ֽڹ�����(��ʱ����OC��CM)
	uint8 activeTimes;						// �����뼤��ʱ���ʱ
	uint8 ucSrc;							//0=ECU,1=VCU,2=ABS,3=Gear

}faultCode,*pFaultCode;

#if 0
typedef struct _XWFaultCode
{
    uint8 ucFaultCodeNum;      //������֡�ܸ���
    STU_CanFrame CanData[MAX_CAN_FaultCode_NUM];//CAN���ݻ��� 

    uint8 ucVehicleType;         //0-Ĭ��ֵ,�����չ�����;1-С��450�ĳ�������(����0x18FE27F3);2-���ڵ���450�ĳ�������(����0x18FE25F3)
                                 //
}stuXWFaultCode,*pstuXWFaultCode;
#endif

typedef struct _STU_MCUSend_GPSData_
{
    uint8 aGpsstate[8];
	uint8 aGpsData[8];
	uint8 aControlData[2];
	uint8 ucCount;
	
}STU_MCUSend_GPSData;

typedef struct _STU_STM32_MCU_
{
    STU_Date Date;

  	uint8 aControlData[15];
    uint8 ucControlFlag;       //1=��͸������ 0=��͸������
}STU_STM32_MCU;

typedef struct _STU_MultiFrame_
{
    uint8 aControllerHours[39];    //������Сʱ�ƣ�T=0xA001��
    uint8 ucControllerHoursLen;
	uint8 aControllerVer[12];      //�������汾��Ϣ��T=0xA002��
	uint8 ucControllerVerLen;
    uint8 aCumminsEngineVer[10];   //Cummins����������汾�ţ�T=0xA003��
    uint8 ucCumminsEngineVerLen;
    uint8 aISUZUEngineInfo[50];    //ISUZU��������Ϣ��T=0xA004��
    uint8 ucISUZUEngineInfoLen;
    uint8 aISUZUEngineVer[30];     //ISUZU����������汾�ţ�T=0xA005��
    uint8 ucISUZUEngineVerLen;

    uint8 aTempData[MultiFrame_TempLEN];          //��ʱ�������ݻ���
    uint8 ucDataIndex;             //���ݱ����ַ������0��ʼ 
	uint8 ucFrameTotalNum;         //���֡�ܸ���
	uint8 ucDataLen;               //��Ч���ݳ���
	uint8 ucFrameSN;               //֡���
	uint8 ucPGNRecvFlag;           //��ӦPGN�������״̬ 1-�յ���ʮ�巢��������汾�㲥֡,
	                                 //2-�յ���ʮ�巢������Ϣ�㲥֡,3-�յ����������ϰ�,0-�������
    uint8 ucEngineType;            //���������� 0-Cummins,1-ISUZU

	//������������������
  //  uint32 uiCanID;
	uint8 ucControlCanDataNum;
	uint8 aControlData[Control_Faultcode_MAX_NUM*8];
    uint8 aControlTempData[Control_Faultcode_MAX_NUM*8];
}STU_MultiFrame,*PSTU_MultiFrame;

void  McuInit(void);
BOOL  CanFrameFilter(uint32 id);
void   DealCan1Message(MessageDetail msg);
void   SendCan1Message(void);
void   McuReadFromMemory(void);
void   McuSaveToMemory(void);
void   MCUCommStateJudge(void);
uint16 ConfirmMcuMsg(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr);
void  MCU_TimerCount_NoDelay(void);
void  MCU_TimerCount_Delay(void);
uint8 GetCanCommState(uint8 channel);
uint8 GetCanRcvState(uint8 channel);
uint8 GetMcuFaultCode(uint8 * faultCodeData);
uint8 IsNewMcuFaultCode(void);
void  ClearNewMcuFaultCodeFlag(void);	
void  Mcu_CanOpen(void);
void  Mcu_CanClose(void);
uint8 Mcu_GetCanSleepState(void);
void  AddCanFrameToBuff(uint32 id, uint8 *data);
BOOL  MCU_GetVoltageState(void);

void MCU_SendCorePlateData(uint8 ucCommand);
void MCU_GetControllerFaultcodedata(uint32 uiID,uint8 *arr);
void MCU_CanRcvDataTimer(void);
void MCU_GetEngineFaultcodedata(uint32 uiID,uint8 *pdata);
void* pthread_MCU_Function(void* data);
#ifdef  __cplusplus
}
#endif

#endif
