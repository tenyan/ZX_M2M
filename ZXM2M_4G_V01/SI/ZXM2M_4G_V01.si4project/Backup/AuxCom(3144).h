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
#define DEV_TTYS4_UART0    "/dev/ttyHS3"

#define AuxCom_Transmit    AUX_UartTransmitData
#define AuxCom_Receive     AUX_UartReceiveData

#define FSRV_DEBUG    1

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// ����״̬
typedef enum
{
  FSRV_STATE_IDLE = 0x00,  /// ����
  FSRV_STATE_START,        /// ��������
  FSRV_STATE_SEND_UN,      /// ����֪ͨ����
  FSRV_STATE_WAIT_REQ,     /// �ȴ�ST�������Ӧ
  FSRV_STATE_SEND_UD,      /// ������������(��ӦST����)
  FSRV_STATE_REPORT,       /// ��������ϱ��������
}fsrv_state_t;

#define FSRV_FIRMWARE_PACKET_LEN    1024//�̼��������ĳ���
// �̼������ṹ��
typedef struct 
{
  uint8_t state;  // ����״̬
  uint8_t dev_id;  // �������豸��: 0x00=��Ч, 0x01=������, 0x03=Э������
  uint8_t msg_sn;  // ��ˮ��
  uint16_t fw_packet_index;  // ���������

  uint32_t start_address;  // �ѽ��ճ���
  uint32_t ending_address;  // �ܳ���

  uint16_t fw_block_size; // �ְ���С:�̶�Ϊ512��1024
  uint16_t total_block_count;  // �����ܿ���
  uint8_t percent;             // ��������

  uint8_t retry_sp;  // ���Դ���
  uint8_t retry_cnt;  // ���Լ�����
  uint8_t timeout_100ms_sp;  // ��ʱ�趨ֵ
  uint8_t timeout_100ms;  // ��ʱ��ʱ��

  uint8_t result;  // �°����0=�ɹ�,1=ʧ��,2=�����������ɹ�,3=�������ܾ�����,4=����������ʧ��

  // ����
  uint16_t tx_size;  // �������ݳ���
  uint8_t* tx_data;  // �������ݵ�ַ

  // ����
  uint16_t rx_size;  // �������ݳ���
  uint8_t* rx_data;  // �������ݵ�ַ
}fsrv_context_t;
extern fsrv_context_t fsrv_context;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif

