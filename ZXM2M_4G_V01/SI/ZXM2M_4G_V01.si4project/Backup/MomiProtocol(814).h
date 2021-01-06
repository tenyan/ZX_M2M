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
