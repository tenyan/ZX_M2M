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
#define MOMI_DEBUG    1
#define MAX_CAN_FRAME_NUM      90   // ������յ����CAN֡����

// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
#define MOMI_FRAME_STX_FIELD        0x00
#define MOMI_FRAME_SIZE_HIGH_FIELD  0x01
#define MOMI_FRAME_SIZE_LOW_FIELD   0x02
#define MOMI_FUNCTION_CODE_FIELD    0x03
#define MOMI_SN_HIGH_FIELD          0x04
#define MOMI_SN_LOW_FIELD           0x05
#define MOMI_DATA_START_FIELD       0x06

//==MOMI TLV E001����==================================================
#define MOMI_E001_POS1     0                   // ���繤��״̬
#define MOMI_E001_POS2     (MOMI_E001_POS1+1)  // GPS��λ״̬
#define MOMI_E001_POS3     (MOMI_E001_POS2+1)  // WIFI����״̬
#define MOMI_E001_POS4     (MOMI_E001_POS3+1)  // ETH����״̬
#define MOMI_E001_POS5     (MOMI_E001_POS4+1)  // Զ��������־
#define MOMI_E001_POS6     (MOMI_E001_POS5+1)  // GPSʱ��
#define SIZE_OF_MOMI_E001  (MOMI_E001_POS6+6)  // ���ֽ���(6B)

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// BOOL����
enum
{
  MOMI_NOK = 0x00,
  MOMI_OK = 0x01
};

// CAN֡�ṹ��
typedef struct
{
  uint32_t id;
  uint8_t data[8];
}can_frame_t;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void Momi_ServiceInit(void);
void Momi_ServiceStart(void);
void Momi_AnalyzeAdSwitch(void);

#endif  /* _MOMI_PROTOCOL_H_ */

