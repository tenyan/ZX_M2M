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


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn);

void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif
