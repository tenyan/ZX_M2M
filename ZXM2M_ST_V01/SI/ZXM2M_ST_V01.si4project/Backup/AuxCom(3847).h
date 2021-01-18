/*****************************************************************************
* @FileName: AuxCom.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modem模块和Micro辅助通信协议头文件
******************************************************************************/
#ifndef _AUX_COM_H_
#define _AUX_COM_H_

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define AUXCOM_DEBUG    0
#define AUXCOM_UART_BAUDRATE (115200U)

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
#define MOMI_FRAME_STX_FIELD        0x00
#define MOMI_FRAME_SIZE_HIGH_FIELD  0x01
#define MOMI_FRAME_SIZE_LOW_FIELD   0x02
#define MOMI_FUNCTION_CODE_FIELD    0x03
#define MOMI_SN_HIGH_FIELD          0x04
#define MOMI_SN_LOW_FIELD           0x05
#define MOMI_DATA_START_FIELD       0x06


/******************************************************************************
 *   Data Types
 ******************************************************************************/

// BOOL定义
enum
{
  AUXCOM_NOK = 0x00,
  AUXCOM_OK = 0x01
};

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif  /* _AUX_COM_H_ */

