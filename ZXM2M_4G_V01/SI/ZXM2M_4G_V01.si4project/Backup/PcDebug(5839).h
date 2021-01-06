/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebug.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-12-26
 * @brief     ���ļ�ΪPcDebug����ģ��Э����ͷ�ļ�
 ********************************************************************************/
#ifndef _PCDEBUG_H_
#define _PCDEBUG_H_

//-----ͷ�ļ�����------------------------------------------------------------
#include "types.h"

//-----��������----------------------------------------------------------------
#define UART0_RECV_BUFF_LENGTH           256           /*uart0 ����������󳤶�*/

#define DEV_TTYGS0_CDC_DEBUG         "/dev/ttyGS0"

/******************************************************************************
 * Macros
 ******************************************************************************/
// �����������ģ��ѡ����
#define DBG_MSG_TYPE_AT       1    // ��ӡATָ��
#define DBG_MSG_TYPE_MODEM    1    // MODEMģ��
#define DBG_MSG_TYPE_GPS      2    // GPSģ��
#define DBG_MSG_TYPE_CAN      3    // CANģ��
#define DBG_MSG_TYPE_RS232    4    // MCU-RS232�ӿ�
#define DBG_MSG_TYPE_SYS      5    // systemģ��
#define DBG_MSG_TYPE_GPRS     5    // ��ӡGPRS����
#define DBG_MSG_TYPE_ANYDATA  10   // �����������

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void PcDebug_SetStatus(uint8_t enable_type);
void PcDebug_SendData(uint8_t* pdata,uint16_t size,uint8_t msg_types);
void PcDebug_SendM2mRsp(uint8_t *pdata, uint16_t size);
void PcDebug_SendString(const char *pstr);
void PcDebug_Printf(const char *format,...);
void PcDebug_ServiceInit(void);
void PcDebug_ServiceStart(void);

#endif
