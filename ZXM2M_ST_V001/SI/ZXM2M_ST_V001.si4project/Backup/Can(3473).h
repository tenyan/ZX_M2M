/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Can.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-19
* @brief
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

/******************************************************************************
 * Data Types
 ******************************************************************************/
#define CAN1_RECV_TIMEOUT_SP    10 // can1���ճ�ʱʱ��,��λ:100ms
#define CAN2_RECV_TIMEOUT_SP    10 // can2���ճ�ʱʱ��,��λ:100ms
typedef struct
{
  uint8_t ep_valid_flag; // ����������Ч��־: 0=��Ч, 1=��Ч
  uint8_t mil_lamp;      // ���ϵ�״̬:0:δ����, 1=����
  uint16_t engine_speed; // ������ת��

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

// �������� LVC = Lock Vehicle Control
__packed typedef struct
{

  uint32_t header; // У���ֽ�  0x55AA5AA5

  // B0=���ͼ��ģʽ����,B1=���ģʽ�����͹ر�
  // B2=����һ����������,B3=һ�����������͹ر�
  // B4=���Ͷ�����������,B5=�������������͹ر�
  uint8_t lock_cmd_flag;      // ƽ̨�·��������Ͱ������ʶ

  // B0=һ�������ɹ�, B1=���������ɹ�
  // B2=һ�������ɹ�,B3=���������ɹ�
  // B4=�󶨳ɹ�, B5=���ɹ�
  uint8_t report_srv_flag;    // �ϱ�ƽ̨��־
  
  uint8_t lock_command;       // ��������:  0:����, 1~3:��������
  uint8_t bind_command;       // ������,0:���κ�����, 0xaa:��, 0x55:���
  
  uint16_t lock_level1_sn;    // ƽ̨�·���һ��������ˮ��
  uint16_t lock_level2_sn;    // ƽ̨�·��Ķ���������ˮ��
  uint16_t unlock_sn;         // ƽ̨�·��Ľ�����ˮ��
  uint16_t bind_sn;           // ƽ̨�·������ü��ģʽ��ˮ��
  
  uint8_t ecu_rsp_lock_state; // ECU����������״̬:1=����, 0=����
  uint8_t ecu_rsp_bind_state; // ECU�����İ�״̬:1=��, 0=�����

  uint8_t xor_value;    // У��ֵ
}lvc_context_t;
extern lvc_context_t lvc_context;
#define SIZEOF_LVC_CONTEXT  (sizeof(lvc_context_t))

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
  ENGINE_TYPE_WEICHAI = 0x00,
  ENGINE_TYPE_HANGFA = 0x01,
  ENGINE_TYPE_SHANGHAI = 0x02,
  ENGINE_TYPE_YUCHAI = 0x03,
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
uint8_t CAN_GetCommState(uint8_t channel);
uint8_t CAN_GetRecvState(uint8_t channel);
uint8_t CAN_GetVinState(void);

uint8_t CAN_GetEngineType(void);
uint16_t CAN_GetEngineSpeed(void);

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
void Can_Do100msTasks(void);
void Can_Do1sTasks(void);

#endif   /* _Can_H_ */

