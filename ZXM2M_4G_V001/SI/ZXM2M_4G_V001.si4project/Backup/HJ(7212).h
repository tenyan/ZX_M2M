/*****************************************************************************
* @FileName: HJ.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.2
* @Date:     2020-1-6
* @brief:    �й����һ���������HJ��׼(���ͳ�Զ���ŷż�ؼ����淶)ʵ��H�ļ�
******************************************************************************/
#ifndef HJ_H_
#define HJ_H_

/******************************************************************************
* Includes
******************************************************************************/
//#define HJEP_CLOUD_SERVER_IP    "120.195.166.245"
//#define HJEP_CLOUD_SERVER_PORT  10012

#define HJEP_DEBUG       1  // 1-ʹ��, 0-��ֹ
#define HJEP_BZ_DEBUG    0  // 1-ʹ��, 0-��ֹ

/******************************************************************************
* Macros(HJ�������ݶ���)
* ��ʼ��(2B)+���Ԫ(1B)+����ʶ���(17B)+�ն�����汾(1B)+���ݼ��ܷ�ʽ(1B)+���ݵ�Ԫ����(2B)+���ݵ�Ԫ(NB)+У����(1B)
******************************************************************************/
/******************����֡ͷ��ַ***********************/
#define HJEP_POS1_ADDRESS    0                       // ��ʼ��
#define HJEP_POS2_ADDRESS    (HJEP_POS1_ADDRESS+2)   // ���Ԫ
#define HJEP_POS3_ADDRESS    (HJEP_POS2_ADDRESS+1)   // ����ʶ���
#define HJEP_POS4_ADDRESS    (HJEP_POS3_ADDRESS+17)  // �ն�����汾��
#define HJEP_POS5_ADDRESS    (HJEP_POS4_ADDRESS+1)   // ���ݼ��ܷ�ʽ
#define HJEP_POS6_ADDRESS    (HJEP_POS5_ADDRESS+1)   // ���ݵ�Ԫ����
#define SIZE_OF_HJEP_HEADER  (HJEP_POS6_ADDRESS+2)   // ���ݵ�Ԫ

//== ������������Ϣ��ַ ========================================================
#define HJEP_ENG_POS1_ADDRESS    (0) // ����
#define HJEP_ENG_POS2_ADDRESS    (HJEP_ENG_POS1_ADDRESS+2) // ����ѹ��
#define HJEP_ENG_POS3_ADDRESS    (HJEP_ENG_POS2_ADDRESS+1) // ʵ��Ť�ذٷֱ�
#define HJEP_ENG_POS4_ADDRESS    (HJEP_ENG_POS3_ADDRESS+1) // Ħ��Ť�ذٷֱ�
#define HJEP_ENG_POS5_ADDRESS    (HJEP_ENG_POS4_ADDRESS+1) // ������ת��
#define HJEP_ENG_POS6_ADDRESS    (HJEP_ENG_POS5_ADDRESS+2) // ������ȼ������
#define HJEP_ENG_POS7_ADDRESS    (HJEP_ENG_POS6_ADDRESS+2) // �������ε���Ũ��
#define HJEP_ENG_POS8_ADDRESS    (HJEP_ENG_POS7_ADDRESS+2) // �������ε���Ũ��
#define HJEP_ENG_POS9_ADDRESS    (HJEP_ENG_POS8_ADDRESS+2) // ������Һλ
#define HJEP_ENG_POS10_ADDRESS   (HJEP_ENG_POS9_ADDRESS+1) // ��������
#define HJEP_ENG_POS11_ADDRESS   (HJEP_ENG_POS10_ADDRESS+2) // �������������¶�
#define HJEP_ENG_POS12_ADDRESS   (HJEP_ENG_POS11_ADDRESS+2) // �������������¶�
#define HJEP_ENG_POS13_ADDRESS   (HJEP_ENG_POS12_ADDRESS+2) // (DPF/POC)����ѹ��
#define HJEP_ENG_POS14_ADDRESS   (HJEP_ENG_POS13_ADDRESS+2) // ��ȴҺ�¶�
#define HJEP_ENG_POS15_ADDRESS   (HJEP_ENG_POS14_ADDRESS+1) // ����Һλ
#define HJEP_ENG_POS16_ADDRESS   (HJEP_ENG_POS15_ADDRESS+1) // ��λ״̬
#define HJEP_ENG_POS17_ADDRESS   (HJEP_ENG_POS16_ADDRESS+1) // ����
#define HJEP_ENG_POS18_ADDRESS   (HJEP_ENG_POS17_ADDRESS+4) // γ��
#define HJEP_ENG_POS19_ADDRESS   (HJEP_ENG_POS18_ADDRESS+4) // ����ʻ���
#define SIZE_OF_HJEP_ENG_MAND    (HJEP_ENG_POS19_ADDRESS+4 - HJEP_ENG_POS1_ADDRESS)

//== ����������������Ϣ��ַ ======================================================
#define HJEP_ADD_POS1_ADDRESS    (0)  // ������Ť��ģʽ
#define HJEP_ADD_POS2_ADDRESS    (HJEP_ADD_POS1_ADDRESS+1)  // ����̤��
#define HJEP_ADD_POS3_ADDRESS    (HJEP_ADD_POS2_ADDRESS+1)  // �ۼ��ͺ�(���ͺ�)
#define HJEP_ADD_POS4_ADDRESS    (HJEP_ADD_POS3_ADDRESS+4)  // �������¶�
#define HJEP_ADD_POS5_ADDRESS    (HJEP_ADD_POS4_ADDRESS+1)  // ʵ������������
#define HJEP_ADD_POS6_ADDRESS    (HJEP_ADD_POS5_ADDRESS+4)  // �ۼ���������(����������)
#define HJEP_ADD_POS7_ADDRESS    (HJEP_ADD_POS6_ADDRESS+4)  // DPF�����¶�
#define SIZE_OF_HJEP_ENG_ADD     (HJEP_ADD_POS7_ADDRESS+2 - HJEP_ADD_POS1_ADDRESS)

//== OBD������Ϣ��ַ ==============================================================
#define HJEP_OBD_POS1_ADDRESS   (0)     // OBD���Э��
#define HJEP_OBD_POS2_ADDRESS   (HJEP_OBD_POS1_ADDRESS+1)   // MIL״̬
#define HJEP_OBD_POS3_ADDRESS   (HJEP_OBD_POS2_ADDRESS+1)   // ���֧��״̬
#define HJEP_OBD_POS4_ADDRESS   (HJEP_OBD_POS3_ADDRESS+2)   // ��Ͼ���״̬
#define HJEP_OBD_POS5_ADDRESS   (HJEP_OBD_POS4_ADDRESS+2)   // ����ʶ����(VIN)
#define HJEP_OBD_POS6_ADDRESS   (HJEP_OBD_POS5_ADDRESS+17) // ����궨ʶ���
#define HJEP_OBD_POS7_ADDRESS   (HJEP_OBD_POS6_ADDRESS+18) // �궨��֤�루CVN��
#define HJEP_OBD_POS8_ADDRESS   (HJEP_OBD_POS7_ADDRESS+18) // IUPRֵ
#define HJEP_OBD_POS9_ADDRESS   (HJEP_OBD_POS8_ADDRESS+36) // ����������
#define HJEP_OBD_POS10_ADDRESS  (HJEP_OBD_POS9_ADDRESS+1)   // ��������Ϣ�б�(N*4B)
#define SIZE_OF_HJEP_OBD        (HJEP_OBD_POS10_ADDRESS-HJEP_OBD_POS1_ADDRESS)

//== ������Ϣ���� ==============================================================
#define HJEP_PACKET_HEADER1       0x23  // #
#define HJEP_PACKET_HEADER2       0x23  // #

#define HJEP_PACKET_CMD_LOGIN     0x01  // ��������
#define HJEP_PACKET_CMD_LOGOUT    0x04  // �����ǳ�
#define HJEP_PACKET_CMD_NTP       0x05 // �ն�Уʱ
#define HJEP_PACKET_CMD_NEW_DATA  0x02 // ʵʱ��Ϣ�ϱ�
#define HJEP_PACKET_CMD_BZ_DATA   0x03 // ������Ϣ�ϱ�

#define HJEP_PACKET_SW_VERSION    EP_FIRMWARE_VERSION // �ն�����汾��

#define HJEP_PACKET_ENCRYPT_NONE     0x01  // ���ݲ�����
#define HJEP_PACKET_ENCRYPT_SM2      0x02  // SM2����
#define HJEP_PACKET_ENCRYPT_SM4      0x03  // SM4����
#define HJEP_PACKET_ENCRYPT_RAS      0x04  // RSA����
#define HJEP_PACKET_ENCRYPT_AES128   0x05  //  AES128����
#define HJEP_PACKET_ENCRYPT_ERROR    0xFE  // �쳣
#define HJEP_PACKET_ENCRYPT_INVALID  0xFF  // ��Ч

#define HJEP_DATA_TYPE_OBD           0x01  // OBD��Ϣ
#define HJEP_DATA_TYPE_ENG_SCR       0x02  // ��������Ϣ
#define HJEP_DATA_TYPE_ENG_TWC       0x03  // ��������Ϣ(��Ԫ�߻�)
#define HJEP_DATA_TYPE_ADD           0x80  // ����������(��ǿ��Ҫ��)

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef enum
{
  HJEP_STATE_SEND_INIT=0x00,
  HJEP_STATE_SEND_IDLE,
  HJEP_STATE_SEND_LOGIN,
  HJEP_STATE_SEND_NTP,
  HJEP_STATE_SEND_OBD,
  HJEP_STATE_SEND_ENG,
  HJEP_STATE_SEND_BZ,
  HJEP_STATE_SEND_LOGOUT
} hjep_state_t;
extern hjep_state_t hjep_state;

extern volatile bittype  hjep_flags1;
#define HJEP_SEND_LOGIN_FLAG  hjep_flags1.b.bit0
#define HJEP_SEND_NTP_FLAG    hjep_flags1.b.bit1
#define HJEP_SEND_OBD_FLAG    hjep_flags1.b.bit2
#define HJEP_SEND_ENG_FLAG    hjep_flags1.b.bit3
#define HJEP_SEND_LOGOUT_FLAG hjep_flags1.b.bit4
#define HJEP_SEND_BZ_FLAG     hjep_flags1.b.bit5
//#define x  hjep_flags1.b.bit6
//#define x  hjep_flags1.b.bit7

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void HJEP_Initialize(void);
void HJEP_StateMachine(void);
void HJEP_ProduceSendData(void);
void HJEP_ProcessRecvData(uint8_t* pdata, uint16_t len);

#endif /* HJ_H_ */

