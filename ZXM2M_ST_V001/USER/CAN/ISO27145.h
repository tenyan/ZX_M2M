/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: ISO27145.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.1
* @Date:     2020-10-25
* @brief:    ISO-27145Э���ȡOBD��Ϣ��ص�.H�ļ�
******************************************************************************/
#ifndef _ISO27145_H_
#define _ISO27145_H_

//-----ͷ�ļ�����------------------------------------------------------------
#include "types.h"

/******************************************************************************
* Includes
******************************************************************************/

/******************************************************************************
* Macros
******************************************************************************/

/******************************************************************************
 * Data Types
 ******************************************************************************/
// ISO27145���������
typedef enum
{
  ISO_REQUEST_STATE_INIT=0x00,
  ISO_REQUEST_STATE_OFF,
  ISO_REQUEST_STATE_IDLE,
  ISO_REQUEST_STATE_RDBI_PID_F810, //Protocol Identification
  ISO_REQUEST_STATE_RDBI_VIN_F802, // Vehicle Identification Number
  ISO_REQUEST_STATE_RDBI_SCI_F804, // Software Calibration Identification
  ISO_REQUEST_STATE_RDBI_CVN_F806, // Calibration Verification Number
  ISO_REQUEST_STATE_RDBI_IUPR_F80B, // In Use (Monitor) Performance Ratio
  ISO_REQUEST_STATE_RDBI_READINESS_F401, // the readiness of the OBD system
  ISO_REQUEST_STATE_RDTCI_S19_42, // = reportWWHOBDDTCByMaskRecord
} iso_request_state_t;
extern iso_request_state_t iso_request_state;

// ISO-27145������
typedef struct
{
  uint8_t engine_type; // ����������:Ϋ��/����/����
  // request
  iso_request_state_t request_state;
  uint8_t state_started_delay;
  uint8_t state_transition;
  // response
  uint8_t first_frame_flag; // �յ���֡��־λ
  uint8_t frame_index;      // ֡���
  // request success flag
  uint8_t requested_pid_flag;
  uint8_t requested_vin_flag;
  uint8_t requested_calid_flag;
  uint8_t requested_cvn_flag;
  uint8_t requested_iupr_flag;
  uint8_t requested_readiness_flag;

  uint16_t timer_100ms;  // ����100ms��ʱ��
}iso27145_context_t;
extern iso27145_context_t iso27145_ctx;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Can_ISO27145Init(iso27145_context_t* pThis);
void Can_SendISO27145Request(iso27145_context_t* pThis);
void Can_ProcessISO27145Response(iso27145_context_t* pThis,uint8_t *pdata,uint8_t size);

#endif /* _ISO27145_H_ */

