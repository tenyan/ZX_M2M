/*****************************************************************************
* @FileName: MomiProtocol.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modemģ���Microͨ��Э��ͷ�ļ�
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
// ģ��״̬����
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

// BOOL����
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
static uint16_t iMomi_BuildTlvMsg_F001(uint8_t *pbuf);  // �ն�����ɼ���Ϣ
static uint16_t iMomi_BuildTlvMsg_F002(uint8_t *pbuf);  // GPS�ɼ���Ϣ
static uint16_t iMomi_BuildTlvMsg_F010(uint8_t *pbuf); // ���ػ��ϳ���Ϣ
static uint16_t iMomi_BuildTlvMsg_F011(uint8_t *pbuf);  // ���ػ�������Ϣ
static uint16_t iMomi_BuildTlvMsg_F012(uint8_t *pbuf); // ���ػ��³���������Ϣ
static uint16_t iMomi_BuildTlvMsg_F020(uint8_t *pbuf); // ��������Ϣ��Ϣ
static uint16_t iMomi_BuildTlvMsg_F030(uint8_t *pbuf); // ͳ������Ϣ��Ϣ
static uint16_t iMomi_BuildTlvMsg_F040(uint8_t *pbuf); // ����ECU�汾��Ϣ
static uint16_t iMomi_BuildTlvMsg_F050(uint8_t *pbuf); // CAN֡��Ϣ

static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len);  // �ն�����ɼ���Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F002(uint8_t* pValue, uint16_t len);  // GPS�ɼ���Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F010(uint8_t* pValue, uint16_t len); // ���ػ��ϳ���Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F011(uint8_t* pValue, uint16_t len);  // ���ػ�������Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F012(uint8_t* pValue, uint16_t len); // ���ػ��³���������Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F020(uint8_t* pValue, uint16_t len); //��������Ϣ��Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F030(uint8_t* pValue, uint16_t len); // ͳ������Ϣ��Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F040(uint8_t* pValue, uint16_t len); // ����ECU�汾��Ϣ
static uint8_t iMomi_AnalyzeTlvMsg_F050(uint8_t* pValue, uint16_t len); // CAN֡��Ϣ
#endif

#endif  /* _MOMI_PROTOCOL_H_ */
