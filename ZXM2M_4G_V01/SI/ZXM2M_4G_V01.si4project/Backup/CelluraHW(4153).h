/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraHW.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-6-5
 * @brief     本文件为4G模块硬件驱动层的头文件
******************************************************************************/
#ifndef _CELLURA_HW_H_
#define _CELLURA_HW_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define USART3_RX_BUFFER_SIZE   1500
#define USART3_TX_BUFFER_SIZE   1500

#define USART3_DMA_RX_INT_PRIO  7
#define USART3_DMA_TX_INT_PRIO  8

#define ENABLE_MODEM_PWR()   MODEM_PWR_PORT->BSRRL = MODEM_PWR_PIN
#define DISABLE_MODEM_PWR()  MODEM_PWR_PORT->BSRRH = MODEM_PWR_PIN

#define MODEM_DTR_H()        MODEM_DTR_PORT->BSRRL = MODEM_DTR_PIN
#define MODEM_DTR_L()        MODEM_DTR_PORT->BSRRH = MODEM_DTR_PIN

#define MODEM_PWRKEY_H()     MODEM_PWRKEY_PORT->BSRRH = MODEM_PWRKEY_PIN // 极性相反
#define MODEM_PWRKEY_L()     MODEM_PWRKEY_PORT->BSRRL = MODEM_PWRKEY_PIN

#define MODEM_RESET_H()      MODEM_RESET_PORT->BSRRH = MODEM_RESET_PIN // 极性相反
#define MODEM_RESET_L()      MODEM_RESET_PORT->BSRRL = MODEM_RESET_PIN

#define MODEM_STATUS_IN()    (MODEM_STATUS_IN_PORT->IDR & MODEM_STATUS_IN_PIN)

//#define LTE_LED_OFF()        LTE_LED_PORT->BSRRH = LTE_LED_PIN
//#define LTE_LED_ON()         LTE_LED_PORT->BSRRL = LTE_LED_PIN

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Modem_GpioInitialize(void);
void USART3_Initialize(uint32_t baudrate);
void USART3_TransmitData(uint8_t *data,uint16_t size);
void USART3_TransmitString(const char *pstr);
uint8_t USART3_ReceiveData(uint8_t **data, uint16_t* size);

#endif  /* _CELLURA_HW_H_ */

