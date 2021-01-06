/*****************************************************************************
* @FileName: MomiProtocol.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modem模块和Micro通信协议头文件
******************************************************************************/
#ifndef _MOMI_PROTOCOL_H_
#define _MOMI_PROTOCOL_H_

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MODEM_UART_BAUDRATE (115200U)

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// BOOL定义
enum
{
  MOMI_NOK = 0x00,
  MOMI_OK = 0x01
};

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void Modem_SetState(modem_state_t state);
modem_state_t Modem_GetState(void);

void Momi_ServiceInit(void);
void Momi_ServiceStart(void);

#if 0
static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len);  // 终端自身采集信息
static uint8_t iMomi_AnalyzeTlvMsg_F002(uint8_t* pValue, uint16_t len);  // GPS采集信息
static uint8_t iMomi_AnalyzeTlvMsg_F010(uint8_t* pValue, uint16_t len); // 起重机上车信息
static uint8_t iMomi_AnalyzeTlvMsg_F011(uint8_t* pValue, uint16_t len);  // 起重机底盘信息
static uint8_t iMomi_AnalyzeTlvMsg_F012(uint8_t* pValue, uint16_t len); // 起重机下车发动机信息
static uint8_t iMomi_AnalyzeTlvMsg_F020(uint8_t* pValue, uint16_t len); //故障类消息信息
static uint8_t iMomi_AnalyzeTlvMsg_F030(uint8_t* pValue, uint16_t len); // 统计类消息信息
static uint8_t iMomi_AnalyzeTlvMsg_F040(uint8_t* pValue, uint16_t len); // 车身ECU版本信息
static uint8_t iMomi_AnalyzeTlvMsg_F050(uint8_t* pValue, uint16_t len); // CAN帧信息
#endif

#endif  /* _MOMI_PROTOCOL_H_ */
