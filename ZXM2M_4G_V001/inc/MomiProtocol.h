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
#define MOMI_DEBUG    0
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
#define MOMI_E001_POS2     (MOMI_E001_POS1+1)  // SIM��ʶ��״̬
#define MOMI_E001_POS3     (MOMI_E001_POS2+1)  // GPS��λ״̬
#define MOMI_E001_POS4     (MOMI_E001_POS3+1)  // WIFI����״̬
#define MOMI_E001_POS5     (MOMI_E001_POS4+1)  // ETH����״̬
#define MOMI_E001_POS6     (MOMI_E001_POS5+1)  // Զ��������־
#define MOMI_E001_POS7     (MOMI_E001_POS6+1)  // GPSʱ��
#define SIZE_OF_MOMI_E001  (MOMI_E001_POS7+6)  // ���ֽ���(6B)


/******************************************************************************
* Macros(�������ݶ���)
******************************************************************************/
//��������������Ϣ����
#define EP_POS1_ADDRESS    (0)  // ����
#define EP_POS2_ADDRESS    (EP_POS1_ADDRESS+2) // ����ѹ��
#define EP_POS3_ADDRESS    (EP_POS2_ADDRESS+1) // ʵ��Ť�ذٷֱ�
#define EP_POS4_ADDRESS    (EP_POS3_ADDRESS+1) // Ħ��Ť�ذٷֱ�
#define EP_POS5_ADDRESS    (EP_POS4_ADDRESS+1) // ������ת��
#define EP_POS6_ADDRESS    (EP_POS5_ADDRESS+2) // ������ȼ������
#define EP_POS7_ADDRESS    (EP_POS6_ADDRESS+2) // �������ε���Ũ��
#define EP_POS8_ADDRESS    (EP_POS7_ADDRESS+2) // �������ε���Ũ��
#define EP_POS9_ADDRESS    (EP_POS8_ADDRESS+2) // ������Һλ
#define EP_POS10_ADDRESS   (EP_POS9_ADDRESS+1) // ��������
#define EP_POS11_ADDRESS   (EP_POS10_ADDRESS+2) // �������������¶�
#define EP_POS12_ADDRESS   (EP_POS11_ADDRESS+2) // �������������¶�
#define EP_POS13_ADDRESS   (EP_POS12_ADDRESS+2) // (DPF/POC)����ѹ��
#define EP_POS14_ADDRESS   (EP_POS13_ADDRESS+2) // ��ȴҺ�¶�(B1)
#define EP_POS15_ADDRESS   (EP_POS14_ADDRESS+1) // ����Һλ(B2)

// ��������������
#define EP_POS16_ADDRESS   (EP_POS15_ADDRESS+1) // ������Ť��ģʽ
#define EP_POS17_ADDRESS   (EP_POS16_ADDRESS+1) // ����̤��
#define EP_POS18_ADDRESS   (EP_POS17_ADDRESS+1) // ���ͺ�
#define EP_POS19_ADDRESS   (EP_POS18_ADDRESS+4) // �������¶�
#define EP_POS20_ADDRESS   (EP_POS19_ADDRESS+1) // ʵ������������
#define EP_POS21_ADDRESS   (EP_POS20_ADDRESS+4) // ����������
#define EP_POS22_ADDRESS   (EP_POS21_ADDRESS+4) // DPF�����¶�

// ������Ϣ
#define EP_POS23_ADDRESS   (EP_POS22_ADDRESS+2) // ����ʻ���
#define EP_POS24_ADDRESS   (EP_POS23_ADDRESS+4) // MIL��״̬
#define EP_POS25_ADDRESS   (EP_POS24_ADDRESS+1) // ������������ʱ��

#define NUMBER_OF_EP_POS   (EP_POS25_ADDRESS+4)

// ����������ź͵�ַ����
//#define ep_data_buffer      g_stuZXMcuData.aucHzepData
extern uint8_t ep_data_buffer[NUMBER_OF_EP_POS];

#define CAN_CHANNEL1        0x00
#define CAN_CHANNEL2        0x01

// OBDЭ������
#define OBD_PROTOCOL_ISO15765    0
#define OBD_PROTOCOL_ISO27145    1
#define OBD_PROTOCOL_SAEJ1939    2

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

// ������
typedef struct
{
  uint32_t dtcode; // 4�ֽڹ�����
  uint8_t debounce_tmr; // ȥ���ʱ��
}dtc_t;
// extern dtc_t dtc_table[MAX_DTC_TABLE_SIZE];

#define MAX_ACTIVE_DTC_NUM		    20 	// ���Խ��յ���༤�����������
// OBD��Ϣ�ṹ��
typedef struct
{
  uint8_t protocol_type;  // OBD���Э������ 0-IOS15765 1-IOS27145 2-SAEJ1939   0xfe:��Ч
  uint8_t mil_status;  // 0~1 0:δ����  1:���� 0xfe:��Ч

  uint8_t diag_valid_flag;
  uint16_t diag_supported_status;  // ���֧��״̬:0-��֧��, 1-֧��
  uint16_t diag_readiness_status;   // ��Ͼ���״̬:0-��֧��, 1-֧��

  uint8_t vin_valid_flag;
  uint8_t vin[17];  // Vehicle Identification Number����ʶ���� ֻ��һ��

  uint8_t calid_valid_flag;
  uint8_t calid[18];  // Calibration Identifications����궨ʶ�������Զ��� ��ĸ��������� ����Ĳ��ַ�"0"  ֻ��һ��

  uint8_t cvn_valid_flag;
  uint8_t cvn[18];  // Calibration Verification Numbers�궨��֤�� �Զ��� ��ĸ��������� ����Ĳ��ַ�"0" ֻ��һ��

  uint8_t iupr_valid_flag;
  uint8_t iupr[36];  // �ο�SAEJ1979-DA G11

  uint8_t dtc_num;  // ���������� 0~253 0xfe ��Ч
  uint32_t dtc[MAX_ACTIVE_DTC_NUM];  // ������ ÿ��������Ϊ4�ֽ�
}obd_info_t;
extern obd_info_t obd_info;

// ������ṹ��
#define MAX_DTC_TABLE_SIZE    10  // �������������
#define DTC_BUFFER_SIZE       (MAX_DTC_TABLE_SIZE*4)
#define DTC_DEBOUNCE_TIME_SP  30  // ������ȥ��ʱ��
// ������
typedef struct
{
  uint8_t new_flag;    // ��DTC��־,0=��,1=��
  uint8_t total_num; // ����������������ֽ���

  uint8_t index; // ��֡����������
  uint8_t buffer[DTC_BUFFER_SIZE]; // ��֡�����뻺��

  uint32_t code[MAX_DTC_TABLE_SIZE]; // 4�ֽڹ�����
  uint8_t debounce_tmr[MAX_DTC_TABLE_SIZE]; // ȥ���ʱ��
}dtc_context_t;
extern dtc_context_t dtc_1939;
extern dtc_context_t dtc_27145;

typedef enum
{
  TBOX_STATE_POWERON = 0x00,
  TBOX_STATE_WORKING,
  TBOX_STATE_SLEEP,
  TBOX_STATE_IAP
} tbox_state_t;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void Momi_ServiceInit(void);
void Momi_ServiceStart(void);
void Momi_AnalyzeAdSwitch(void);

// DTC�ӿں���
uint8_t DTC_GetNewFlag(dtc_context_t* pThis);
void DTC_ClearNewFlag(dtc_context_t* pThis);
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf);
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis);
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode);

// ��ȡOBD�е�DTC����
void OBD_GetDtcCode(obd_info_t* pThis, uint8_t *pBuf);

#endif  /* _MOMI_PROTOCOL_H_ */

