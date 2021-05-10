/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsHW.h
 * @Engineer: TenYan
 * @Company:  徐工信息智能硬件部
 * @version:  V1.0
 * @Date:     2020-10-11
 * @brief:    本文件为GPS模块硬件驱动层的头文件
******************************************************************************/
#ifndef _GPS_HW_H_
#define _GPS_HW_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define GPS_UART_BAUD_RATE      9600

#define USART2_RX_BUFFER_SIZE   384
#define USART2_TX_BUFFER_SIZE   256

#define USART2_DMA_RX_INT_PRIO  9
#define USART2_DMA_TX_INT_PRIO  10

#define ENABLE_GPS_PWR()   GPS_POWER_PORT->BSRRL = GPS_POWER_PIN  // 打开GPS电源
#define DISABLE_GPS_PWR()  GPS_POWER_PORT->BSRRH = GPS_POWER_PIN  // 关闭GPS电源

#define GPS_ON()           GPS_ON_OFF_PORT->BSRRL = GPS_ON_OFF_PIN // 使能GPS
#define GPS_OFF()          GPS_ON_OFF_PORT->BSRRH = GPS_ON_OFF_PIN // 关断GPS

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void GPS_GpioInitialize(void);
void USART2_Initialize(uint32_t baudrate);
void USART2_TransmitData(uint8_t *data,uint16_t size);
void USART2_TransmitString(const char *pstr);
uint8_t USART2_ReceiveData(uint8_t **data, uint16_t* size);

#endif  /* _GPS_HW_H_ */

