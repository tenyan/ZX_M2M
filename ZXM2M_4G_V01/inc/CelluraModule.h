/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraModule.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-12-31
 * @brief     
******************************************************************************/
#ifndef _CELLURA_MODULE_H_
#define _CELLURA_MODULE_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"
#include "Cellura.h"
#include "NetSocket.h"

#define CELLURA_MAX_CONNS          6
#define SOCKET_CONN_MAX_DATA_LEN   1460


/******************************************************************************
 *   Macros
 ******************************************************************************/
// MODEM parameters
#define MODEM_UART_BAUDRATE (B115200)
#define MODEM_MAX_SOCKET_TX_DATA_SIZE ((uint32_t)1460U)  // cf AT+QISEND
#define MODEM_MAX_SOCKET_RX_DATA_SIZE ((uint32_t)1500U)  // cf AT+QIRD

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Cellura_ServiceInit(void);
void Cellura_ServiceStart(void);

// Cellura信息函数接口
uint8_t *Cellura_GetIccid(void);
uint8_t Cellura_GetCsq(void);
uint32_t Cellura_GetLacCellID(void);
uint8_t Cellura_GetModemStatus(void);
uint8_t Cellura_GetSimCardState(void);
uint8_t Cellura_GetCsRegistState(void);
uint8_t Cellura_GetPsRegistState(void);

#endif  /* _CELLURA_MODULE_H_ */

