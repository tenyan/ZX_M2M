//�ļ�����Mcu_Can.h
//���ܣ�  �����Լ��������ɵ�GPIO������ַ�ĺ궨��
#ifndef _MCU_H_
#define _MCU_H_

#ifdef  __cplusplus
extern "C" {
#endif

#include "McuHW.h"


//#define CAN1								0	//
//#define CAN2								1


#define	TASK_MCU_CAN_ID						6
#define TASK_MCU_CAN_PRIO       			6
#define TASK_MCU_CAN_STK_SIZE				500

#define MCU_DATA_LENGTH						1460 //��ͨMCU���ݵĳ���, MCU�����˿���������ʾ���İ汾����Ϣ2013-05-03
#define TIME_COMM_UNNORMAL          		120	 //ͨ���쳣�ж�ʱ��

#define MCU_STATE_NORMAL					0	//��MCUͨ������
#define MCU_STATE_UNNORMAL					1	//��MCUͨ���쳣

//#define MCU_CAN_BRT		                    BPS_500K//CANͨ�ŵĲ�����
#define MAX_CAN_FRAME_NUM					80	//������յ����CAN֡����
#define MAX_ACTIVE_FAULTCODE_NUM		    10	//���Խ��յ���༤�����������
#define TIME_FAULTCODE_ACTIVE			    30	//�����뼤��ʱ�䣬������ʱ�������û�г�������Ϊ�ù�����ʧ
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
#define MAX_CAN2_RCV_ERR_TIME				1000 //can1���������ݳ�ʱʱ��,��λ:10ms

#define MCU_GPSCMD_SEND_TIME                10
#define MCU_GPSSTATE_SEND_TIME              50
#define MAX_CAN_FaultCode_NUM				15 //������յĹ��������CAN֡����


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
	uint8 ucLock;							//�յ���������, 0xaa:�յ�����/��������, ����:û���յ�
	uint8 ucLockStage;						//��������:  0:����, 1~3:��������
    uint16 usSpeedLimit;					//����/����:0=��ֹ����, 28000:���,1~27999�ٶ�����
	uint8 ucBindCmd;						//������,0:���κ�����, 0xaa:��, 0x55:���
	uint8 ucEcuFeedBackBindState;			//ECU�����İ�״̬:1=��, 0=�����
	uint8 ucEcuFeedBackLockState;			//ECU����������״̬:1=����, 0=����

    uint8 LockControlFlag;              //��������ģʽ���� 20100826 
                                        //B0 : 0=����Ҫ���ͼ��ģʽ����;1=��Ҫ���ͼ��ģʽ
                                        //B1 : 0=���ģʽ�ر�; 1=���ģʽ��
                                        //B2 : 0=����Ҫ����һ����������;1=��Ҫ����һ������
                                        //B3 : 0=ȡ��һ����������;1=ִ��һ������
                                        //B4 : 0=����Ҫ���Ͷ�����������;1=��Ҫ���Ͷ�������
                                        //B5 : 0=ȡ��������������;1=ִ�ж�������
    uint8 ucRespSerFlag;	//��Ӧƽ̨��־,
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

	uint8  ucSimQueryFlag;	   //MCU��ѯSIM��  1=�յ���ѯ����
	uint16 usSendMCULockTimer; //����MCU���������ʱ��
	uint16 usSendGPSDataTimer; //��ʱ����GPS��λ���ݼ�ʱ��
	uint32 uiFs;               //У��õ��Ľ��,��Ҫ����
	uint32 uiKey;              //keyֵ  Ĭ��Ϊ 0x18FE0AF4
	uint8  ucCmdCheckFlag;     //MCU�ظ�������У������־,51-�ɹ�;52-ʧ��

}STU_McuCmd,*PSTU_McuCmd;

typedef struct _STU_CanFrame{
	uint32 id;
	uint8 aucData[8];
}STU_CanFrame;

typedef struct _faultCode{
	uint32 faultCode;						// 4�ֽڹ�����(��ʱ����OC��CM)
	uint8 activeTimes;						// �����뼤��ʱ���ʱ
}faultCode,*pFaultCode;


typedef struct _XWFaultCode
{
    uint8 ucFaultCodeNum;      //������֡�ܸ���
    STU_CanFrame CanData[MAX_CAN_FaultCode_NUM];//CAN���ݻ��� 
  //  uint8 ucRecvOverFlag;      //0-δ���ջ��߽���δ���,1-�������
    uint8 ucVehicleType;         //0-Ĭ��ֵ,�����չ�����;1-С��450�ĳ�������(����0x18FE27F3);2-���ڵ���450�ĳ�������(����0x18FE25F3)
                                 //
}stuXWFaultCode,*pstuXWFaultCode;

typedef struct _STU_MCUSend_GPSData_
{
    uint8 aGpsstate[8];
	uint8 aGpsData[8];
	uint8 aControlData[2];
	uint8 ucCount;
	
}STU_MCUSend_GPSData;

/************************end of md5****************************************************************/


void   McuInit(void);
BOOL CanFrameFilter(uint32 id);
void   DealCan1Message(MessageDetail msg);
void   SendCan1Message(void);
//void   McuReadFromMemory(void);
//void   McuSaveToMemory(void);
void   MCUCommStateJudge(void);
uint16 ConfirmMcuMsg(uint8* PtrTxt, uint16 usPtrTxtLen, uint8 ucSrcAddr);
void  MCU_TimerCount_NoDelay(void);
void  MCU_TimerCount_Delay(void);
uint8 GetCanCommState(uint8 channel);
uint8 GetCanRcvState(uint8 channel);
uint8 GetMcuFaultCode(uint8 * faultCodeData);
uint8 IsNewMcuFaultCode(void);
void ClearNewMcuFaultCodeFlag(void);	
void Mcu_CanOpen(void);
void Mcu_CanClose(void);
uint8 Mcu_GetCanSleepState(void);
void AddCanFrameToBuff(uint32 id, uint8 *data);
BOOL MCU_GetVoltageState(void);
uint8 MCU_SendNewLockCmd(uint8 *ptr);
void TaskMCU(void *pdata);
#ifdef  __cplusplus
}
#endif

#endif
