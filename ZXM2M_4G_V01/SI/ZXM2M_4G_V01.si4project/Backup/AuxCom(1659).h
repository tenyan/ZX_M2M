/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief     ���ļ�Ϊ����ͨ��ģ���ͷ�ļ�
 ********************************************************************************/
#ifndef _AUX_COM_H_
#define _AUX_COM_H_

//-----ͷ�ļ�����------------------------------------------------------------
#include "types.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define DEV_TTYS4_UART0      "/dev/ttyHS3"

#define AuxCom_Transmit          AUX_UartTransmitData
#define AuxCom_Receive           AUX_UartReceiveData

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// ����״̬
typedef enum
{
  FSRV_STATE_IDLE = 0x00,  /// ����
  FSRV_STATE_SEND_UN,      /// ����֪ͨ����
  FSRV_STATE_WAIT_REQ,     /// �ȴ�ST����
  FSRV_STATE_SEND_UD,      /// ������������(��ӦST����)
  FSRV_STATE_REPORT_REQ,   /// ��������ϱ��������
}fsrv_state_t;

#define FSRV_FIRMWARE_PACKET_LEN    1024//�̼��������ĳ���
// �̼������ṹ��
typedef struct 
{
  uint8_t state; // ����״̬
  uint8_t sent_flag;         // �����ѷ���
  uint8_t rsp_timeout_timer; // ��ʱ�趨ֵ
  uint8_t retry_cnt;         // ���Լ�����
  uint16_t timeout_cnt;      // ������ʱ������,���10������û��������������
  uint8_t result;            // �°����0=�ɹ�,1=ʧ��
}fsrv_context_t;


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif
