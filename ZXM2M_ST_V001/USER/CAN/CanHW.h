/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CanHW.h
 * @Engineer: TenYan
 * @Company:  �칤��Ϣ����Ӳ����
 * @version   V1.0
 * @Date:     2020-10-15
 * @brief     ���ļ�ΪCANӲ���������ͷ�ļ�
******************************************************************************/
#ifndef _CAN_HW_H_
#define _CAN_HW_H_

//-----ͷ�ļ�����------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define CAN_CHANNEL1    0x00
#define CAN_CHANNEL2    0x01

/******************************************************************************
 * Data Types
 ******************************************************************************/
// CAN֡����
typedef enum
{
  CAN_FRAME_TYPE_STD = 0,  // ��׼֡
  CAN_FRAME_TYPE_EXT = 1   // ��չ֡
} can_frame_type_t;

// CAN��Ϣ�ṹ��
typedef struct
{
  uint32_t id;
  uint8_t len;
  uint8_t type;  // Ext or Std
  uint8_t data[8];
}can_msg_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void CAN_GpioInitialize(void);
void CAN_Initialize(void);
uint8_t CAN1_ReceiveMessage(uint16_t timeout);
void CAN1_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata);
uint8_t CAN2_ReceiveMessage(uint16_t timeout);
void CAN2_TransmitData(uint8_t frame_type, uint32_t can_id, uint8_t len, uint8_t* pdata);

#endif  /* _CAN_HW_H_ */


