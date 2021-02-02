/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: AuxComHW.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2021-1-12
* @brief     本文件为4G模块辅助串口硬件驱动层的头文件
******************************************************************************/
#ifndef _AUX_COM_HW_H_
#define _AUX_COM_HW_H_

//-----头文件调用-------------------------------------------------------------
#include <stdint.h>

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define USART6_RX_BUFFER_SIZE   1500
#define USART6_TX_BUFFER_SIZE   1500

#define USART6_DMA_RX_INT_PRIO  0x0D
#define USART6_DMA_TX_INT_PRIO  0x0E

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void USART6_Initialize(uint32_t baudrate);
void USART6_TransmitData(uint8_t *data,uint16_t size);
void USART6_TransmitString(const char *pstr);
uint8_t USART6_ReceiveData(uint8_t **data, uint16_t* size);

#endif  /* _AUX_COM_HW_H_ */

