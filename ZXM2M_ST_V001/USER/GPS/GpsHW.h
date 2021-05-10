/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsHW.h
 * @Engineer: TenYan
 * @Company:  �칤��Ϣ����Ӳ����
 * @version:  V1.0
 * @Date:     2020-10-11
 * @brief:    ���ļ�ΪGPSģ��Ӳ���������ͷ�ļ�
******************************************************************************/
#ifndef _GPS_HW_H_
#define _GPS_HW_H_

//-----ͷ�ļ�����-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define GPS_UART_BAUD_RATE      9600

#define USART2_RX_BUFFER_SIZE   384
#define USART2_TX_BUFFER_SIZE   256

#define USART2_DMA_RX_INT_PRIO  9
#define USART2_DMA_TX_INT_PRIO  10

#define ENABLE_GPS_PWR()   GPS_POWER_PORT->BSRRL = GPS_POWER_PIN  // ��GPS��Դ
#define DISABLE_GPS_PWR()  GPS_POWER_PORT->BSRRH = GPS_POWER_PIN  // �ر�GPS��Դ

#define GPS_ON()           GPS_ON_OFF_PORT->BSRRL = GPS_ON_OFF_PIN // ʹ��GPS
#define GPS_OFF()          GPS_ON_OFF_PORT->BSRRH = GPS_ON_OFF_PIN // �ض�GPS

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void GPS_GpioInitialize(void);
void USART2_Initialize(uint32_t baudrate);
void USART2_TransmitData(uint8_t *data,uint16_t size);
void USART2_TransmitString(const char *pstr);
uint8_t USART2_ReceiveData(uint8_t **data, uint16_t* size);

#endif  /* _GPS_HW_H_ */

