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
#define MOMI_DEBUG    0
#define MODEM_UART_BAUDRATE (115200U)

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
#define MOMI_FRAME_STX_FIELD        0x00
#define MOMI_FRAME_SIZE_HIGH_FIELD  0x01
#define MOMI_FRAME_SIZE_LOW_FIELD   0x02
#define MOMI_FUNCTION_CODE_FIELD    0x03
#define MOMI_SN_HIGH_FIELD          0x04
#define MOMI_SN_LOW_FIELD           0x05
#define MOMI_DATA_START_FIELD       0x06


//==MOMI TLV E001定义==================================================
#define MOMI_E001_POS1    0                   // 网络工作状态
#define MOMI_E001_POS2    (MOMI_E001_POS1+1)  // GPS定位状态
#define MOMI_E001_POS3    (MOMI_E001_POS2+1)  // WIFI工作状态
#define MOMI_E001_POS4    (MOMI_E001_POS3+1)  // ETH工作状态
#define MOMI_E001_POS5    (MOMI_E001_POS4+1)  // 远程升级标志
#define MOMI_E001_POS6    (MOMI_E001_POS5+1)  // GPS时间
#define SIZE_OF_MOMI_E001 (MOMI_E001_POS6+6)  // 总字节数(6B)


/******************************************************************************
 *   Data Types
 ******************************************************************************/
// 模块状态定义
typedef enum
{
  MODEM_STATE_OFF = 0x00,
  MODEM_STATE_POWER_OFF,
  MODEM_STATE_POWER_ON,
  MODEM_STATE_REPOWER_OFF,
  MODEM_STATE_DATA_READY,
  MODEM_STATE_SILENCE
}modem_state_t;
extern modem_state_t modem_state;

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
static uint16_t iMomi_BuildTlvMsg_F001(uint8_t *pbuf);  // 终端自身采集信息
static uint16_t iMomi_BuildTlvMsg_F002(uint8_t *pbuf);  // GPS采集信息
static uint16_t iMomi_BuildTlvMsg_F010(uint8_t *pbuf); // 起重机上车信息
static uint16_t iMomi_BuildTlvMsg_F011(uint8_t *pbuf);  // 起重机底盘信息
static uint16_t iMomi_BuildTlvMsg_F012(uint8_t *pbuf); // 起重机下车发动机信息
static uint16_t iMomi_BuildTlvMsg_F020(uint8_t *pbuf); // 故障类消息信息
static uint16_t iMomi_BuildTlvMsg_F030(uint8_t *pbuf); // 统计类消息信息
static uint16_t iMomi_BuildTlvMsg_F040(uint8_t *pbuf); // 车身ECU版本信息
static uint16_t iMomi_BuildTlvMsg_F050(uint8_t *pbuf); // CAN帧信息

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
