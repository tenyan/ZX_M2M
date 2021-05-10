/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: PcDebugHW.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version:  V1.0
* @Date:     2020-9-21
* @brief:
******************************************************************************/
#ifndef _PC_DEBUG_HW_H_
#define _PC_DEBUG_HW_H_

#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define USART1_RX_BUFFER_SIZE   1500
#define USART1_TX_BUFFER_SIZE   1500

#define USART1_DMA_RX_INT_PRIO  11
#define USART1_DMA_TX_INT_PRIO  12


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void USART1_Initialize(uint32_t baudrate);
void USART1_TransmitData(uint8_t *data,uint16_t size);
void USART1_TransmitString(const char *pstr);
uint8_t USART1_ReceiveData(uint8_t **data, uint16_t* size);

#endif /* _PC_DEBUG_HW_H_ */

