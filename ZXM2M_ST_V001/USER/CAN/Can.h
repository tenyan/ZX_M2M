/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Can.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-6-19
* @brief:
******************************************************************************/
#ifndef _CAN_H_
#define _CAN_H_

#include "CanHW.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MCU_DATA_LENGTH        1460// ��ͨMCU���ݵĳ���, MCU�����˿���������ʾ���İ汾����Ϣ2013-05-03
#define TIME_COMM_UNNORMAL     30   // ͨ���쳣�ж�ʱ��

#define MAX_CAN_FRAME_NUM      90   // ������յ����CAN֡����
#define MAX_CAN_RCV_TIMEOUT_SP 100  //can1���������ݳ�ʱʱ��,��λ:10ms

#define HZEP_FAULTCODE_TIMEOUT_SP  30 //�����볬ʱ���ʱ��30��

/******************************************************************************
 * Data Types
 ******************************************************************************/
#define CAN1_RECV_TIMEOUT_SP    10 // can1���ճ�ʱʱ��,��λ:100ms
#define CAN2_RECV_TIMEOUT_SP    10 // can2���ճ�ʱʱ��,��λ:100ms
typedef struct
{
  uint8_t ep_valid_flag; // ����������Ч��־: 0=��Ч, 1=��Ч
  uint8_t mil_lamp;      // ���ϵ�״̬:0:δ����, 1=����
  uint8_t got_ci_code_flag; // ��ȡCI���־: 0=δ��ȡ, 1=�ѻ�ȡ
  uint16_t up_engine_speed;  // �ϳ�������ת��
  uint16_t dw_engine_speed;  // �³�������ת��

  uint8_t comm_state1;  // ͨ��״̬:0=�쳣, 1=����
  uint8_t recv_state1;  // ����״̬:0=δ�յ�����, 1=���յ�����
  uint16_t recv_timer1; // ���ճ�ʱ������

  uint8_t comm_state2;  // ͨ��״̬:0=�쳣, 1=����
  uint8_t recv_state2;  // ����״̬:0=δ�յ�����, 1=���յ�����
  uint16_t recv_timer2; // ���ճ�ʱ������

  uint8_t  sleep_state; // ����״̬,0=δ����, 1=����

  uint32_t twt_up;  // �ϳ��ܹ���ʱ��
  uint32_t twt_down;  // �³��ܹ���ʱ��
  uint32_t tfc_up;  // �ϳ����ͺ�
  uint32_t tfc_down;  // �³����ͺ�
  uint32_t odo_down;  // �³������

  uint8_t pid_up_type;      // �ϳ�����״̬��
  uint8_t pid_up_config1;   // �ϳ�����״̬��1
  uint8_t pid_up_config2;   // �ϳ�����״̬��2
  uint8_t pid_up_can;       // �ϳ�Э������״̬��
  uint8_t pid_down_type;    // ��������״̬��
  uint8_t pid_down_config1; // ��������״̬��1
  uint8_t pid_down_config2; // ��������״̬��2
  uint8_t pid_down_can;     // ����CANЭ��
  uint8_t pid_save_flag;    // ����Э����Ϣ��־λ

  uint8_t hzep_fault_data[100];  // ���ݻ����������������ݻ���
  uint16_t hzep_fault_data_len;  // ���ݻ����������������ݳ���
  uint8_t hzep_fault_data_flag;  // 0-δ�յ�����������յ��Ĺ������Ѿ���ʱ,��ʱʱ��Ϊ30��
  uint8_t hzep_fault_data_timer;  // ���ݻ������ϳ�ʱ��ʱ��  �ݶ�30��
  uint8_t hzep_new_fdata_flag;  // ���յ��µ�����֡

  uint8_t eng_ci_buffer[100];  // ��֡CI�뻺��
  uint8_t eng_ci_index;        // ��֡CI������
  uint8_t eng_ci_new_flag;     // ���յ��µ�����֡

  uint8_t ecu_type;  // ����������
  uint8_t ep_type;   // ��������
  uint8_t vin[17];   // ����ʶ�����(Vehicle Identification Number)�򳵼ܺ���
  uint8_t vin_valid_flag;  // VIN����Ч��ʶ
}can_context_t;
extern can_context_t can_context;

// BOOL����
enum
{
  CAN_NOK = 0x00,
  CAN_OK = 0x01
};

#define MAX_DTC_TABLE_SIZE    10  // �������������
#define DTC_BUFFER_SIZE       (MAX_DTC_TABLE_SIZE*4)
#define DTC_DEBOUNCE_TIME_SP  30  // ������ȥ��ʱ��
// ������
typedef struct
{
  uint8_t new_flag;    // ��DTC��־,0=��,1=��
  uint8_t total_num; // ����������������ֽ���

  uint8_t index; // ��֡����������
  uint8_t buffer[DTC_BUFFER_SIZE]; // ��֡�����뻺��

  uint32_t code[MAX_DTC_TABLE_SIZE]; // 4�ֽڹ�����
  uint8_t debounce_tmr[MAX_DTC_TABLE_SIZE]; // ȥ���ʱ��
}dtc_context_t;
extern dtc_context_t dtc_1939;
extern dtc_context_t dtc_27145;

// CAN֡�ṹ��
typedef struct
{
  uint32_t id;
  uint8_t data[8];
}can_frame_t;

// ������
typedef struct
{
  uint32_t dtcode; // 4�ֽڹ�����
  uint8_t debounce_tmr; // ȥ���ʱ��
}dtc_t;
// extern dtc_t dtc_table[MAX_DTC_TABLE_SIZE];

#define MAX_ACTIVE_DTC_NUM		    20 	// ���Խ��յ���༤�����������
// OBD��Ϣ�ṹ��
typedef struct
{
  uint8_t protocol_type;  // OBD���Э������ 0-IOS15765 1-IOS27145 2-SAEJ1939   0xfe:��Ч
  uint8_t mil_status;			// 0~1 0:δ����  1:���� 0xfe:��Ч

  uint8_t diag_valid_flag;
  uint16_t diag_supported_status; // ���֧��״̬:0-��֧��, 1-֧��
  uint16_t diag_readiness_status;	  // ��Ͼ���״̬:0-��֧��, 1-֧��

  uint8_t vin_valid_flag;
  uint8_t vin[17];		// Vehicle Identification Number����ʶ���� ֻ��һ��

  uint8_t calid_valid_flag;
  uint8_t calid[18];	// Calibration Identifications����궨ʶ�������Զ��� ��ĸ��������� ����Ĳ��ַ�"0"  ֻ��һ��

  uint8_t cvn_valid_flag;
  uint8_t cvn[18];		// Calibration Verification Numbers�궨��֤�� �Զ��� ��ĸ��������� ����Ĳ��ַ�"0" ֻ��һ��

  uint8_t iupr_valid_flag;
  uint8_t iupr[36];	// �ο�SAEJ1979-DA G11

  uint8_t dtc_cnt;  // ���������� 0~253 0xfe ��Ч
  uint32_t dtc[MAX_ACTIVE_DTC_NUM];	// ������ ÿ��������Ϊ4�ֽ�
}obd_info_t;
extern obd_info_t obd_info;

// ���������Ͷ���
enum
{
  ENGINE_TYPE_NONE = 0x00,
  ENGINE_TYPE_WEICHAI,
  ENGINE_TYPE_HANGFA,
  ENGINE_TYPE_SHANGCHAI,
  ENGINE_TYPE_YUCHAI,
  ENGINE_TYPE_BENZ,
  NUMBER_OF_ENGINE_TYPE
};

// �������Ͷ���
enum
{
  EP_TYPE_NONE = 0x00,
  EP_TYPE_HJ = 0x01,
  EP_TYPE_GB = 0x02,
};

/******************************************************************************
 * Data Types
 ******************************************************************************/
// CAN��Ϣ����
#define CAN_MSG_QUEUE_MAX_SIZE  16
typedef struct
{
  uint8_t head;  // ����ͷ
  uint8_t tail;  // ����β
  CanRxMsg msg_buff[CAN_MSG_QUEUE_MAX_SIZE];  // ���ݻ�����
}can_msg_queue_t;
extern can_msg_queue_t can1_msg_queue;
extern can_msg_queue_t can2_msg_queue;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
// DTC�ӿں���
uint8_t DTC_GetNewFlag(dtc_context_t* pThis);
void DTC_ClearNewFlag(dtc_context_t* pThis);
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf);
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis);
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode);

// �û�API����
uint8_t CAN1_GetCommState(void);
uint8_t CAN2_GetCommState(void);
uint8_t CAN1_GetRecvState(void);
uint8_t CAN2_GetRecvState(void);

uint8_t CAN_GetVinState(void);
uint8_t CAN_GetObdVinState(void);
uint8_t CAN_GetUserVinState(void);
uint8_t CAN_GetCiCodeState(void);

uint8_t CAN_GetEngineType(void);
uint16_t CAN_GetEngineSpeed(void);

uint8_t CAN_GetDwEngineState(void);
uint8_t CAN_GetUpEngineState(void);

uint32_t CAN_GetUpEngineTwt(void);    // �ϳ��ܹ���ʱ��
uint32_t CAN_GetDownEngineTwt(void);    // �³��ܹ���ʱ��

uint32_t CAN_GetUpEngineTfc(void);// �ϳ����ͺ�
uint32_t CAN_GetDownEngineTfc(void);// �³����ͺ�

// ��Ϣ����
void can_msg_queue_reset(can_msg_queue_t* pThis);
void can_msg_queue_push(can_msg_queue_t* pThis, CanRxMsg* pMsg);
static void can_msg_queue_pop(can_msg_queue_t* pThis, can_msg_t* pMsg);
static uint8_t can_msg_queue_size(can_msg_queue_t* pThis);

// �ӿں���
void Can_ServiceInit(void);
void Can_ServiceStart(void);
void Can_Do10msTasks(void);
void Can_Do100msTasks(void);
void Can_Do1sTasks(void);

#endif   /* _Can_H_ */

