/*****************************************************************************
* @FileName: GB17691.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.1
* @Date:     2021-1-25
* @brief:    �й����ұ�׼GB17691-2018(���Ͳ��ͳ�Զ���ŷż�ؼ����淶)ʵ��.H�ļ�
******************************************************************************/
#ifndef _GB_H_
#define _GB_H_

/******************************************************************************
* Includes
******************************************************************************/
#define GBEP_CLOUD_SERVER_IP    "120.195.166.245"
#define GBEP_CLOUD_SERVER_PORT  10012

#define BJEP_CLOUD_SERVER_DNS   "jc.bjmemc.com.cn"
#define BJEP_CLOUD_SERVER_PORT  7740

/******************************************************************************
* Macros(GB�������ݶ���)
* ��ʼ��(2B)+���Ԫ(1B)+����ʶ���(17B)+�ն�����汾(1B)+���ݼ��ܷ�ʽ(1B)+���ݵ�Ԫ����(2B)+���ݵ�Ԫ(NB)+У����(1B)
******************************************************************************/
/******************����֡ͷ��ַ***********************/
#define GBEP_POS1_ADDRESS    0                       // ��ʼ��
#define GBEP_POS2_ADDRESS    (GBEP_POS1_ADDRESS+2)   // ���Ԫ
#define GBEP_POS3_ADDRESS    (GBEP_POS2_ADDRESS+1)   // ����ʶ���
#define GBEP_POS4_ADDRESS    (GBEP_POS3_ADDRESS+17)  // �ն�����汾��
#define GBEP_POS5_ADDRESS    (GBEP_POS4_ADDRESS+1)   // ���ݼ��ܷ�ʽ
#define GBEP_POS6_ADDRESS    (GBEP_POS5_ADDRESS+1)   // ���ݵ�Ԫ����
#define SIZE_OF_GBEP_HEADER  (GBEP_POS6_ADDRESS+2)   // ���ݵ�Ԫ

//== ������������Ϣ��ַ ========================================================
#define GBEP_ENG_POS1_ADDRESS   (0) // ����
#define GBEP_ENG_POS2_ADDRESS   (GBEP_ENG_POS1_ADDRESS+2) // ����ѹ��
#define GBEP_ENG_POS3_ADDRESS   (GBEP_ENG_POS2_ADDRESS+1) // ʵ��Ť�ذٷֱ�
#define GBEP_ENG_POS4_ADDRESS   (GBEP_ENG_POS3_ADDRESS+1) // Ħ��Ť�ذٷֱ�
#define GBEP_ENG_POS5_ADDRESS   (GBEP_ENG_POS4_ADDRESS+1) // ������ת��
#define GBEP_ENG_POS6_ADDRESS   (GBEP_ENG_POS5_ADDRESS+2) // ������ȼ������
#define GBEP_ENG_POS7_ADDRESS   (GBEP_ENG_POS6_ADDRESS+2) // �������ε���Ũ��
#define GBEP_ENG_POS8_ADDRESS   (GBEP_ENG_POS7_ADDRESS+2) // �������ε���Ũ��
#define GBEP_ENG_POS9_ADDRESS   (GBEP_ENG_POS8_ADDRESS+2) // ������Һλ
#define GBEP_ENG_POS10_ADDRESS  (GBEP_ENG_POS9_ADDRESS+1) // ��������
#define GBEP_ENG_POS11_ADDRESS  (GBEP_ENG_POS10_ADDRESS+2) // �������������¶�
#define GBEP_ENG_POS12_ADDRESS  (GBEP_ENG_POS11_ADDRESS+2) // �������������¶�
#define GBEP_ENG_POS13_ADDRESS  (GBEP_ENG_POS12_ADDRESS+2) // (DPF/POC)����ѹ��
#define GBEP_ENG_POS14_ADDRESS  (GBEP_ENG_POS13_ADDRESS+2) // ��ȴҺ�¶�
#define GBEP_ENG_POS15_ADDRESS  (GBEP_ENG_POS14_ADDRESS+1) // ����Һλ
#define GBEP_ENG_POS16_ADDRESS  (GBEP_ENG_POS15_ADDRESS+1) // ��λ״̬
#define GBEP_ENG_POS17_ADDRESS  (GBEP_ENG_POS16_ADDRESS+1) // ����
#define GBEP_ENG_POS18_ADDRESS  (GBEP_ENG_POS17_ADDRESS+4) // γ��
#define GBEP_ENG_POS19_ADDRESS  (GBEP_ENG_POS18_ADDRESS+4) // ����ʻ���
#define SIZE_OF_GBEP_ENG        (GBEP_ENG_POS19_ADDRESS+4 - GBEP_ENG_POS1_ADDRESS)

//== OBD������Ϣ��ַ ==============================================================
#define GBEP_OBD_POS1_ADDRESS   (0)     // OBD���Э��
#define GBEP_OBD_POS2_ADDRESS   (GBEP_OBD_POS1_ADDRESS+1)   // MIL״̬
#define GBEP_OBD_POS3_ADDRESS   (GBEP_OBD_POS2_ADDRESS+1)   // ���֧��״̬
#define GBEP_OBD_POS4_ADDRESS   (GBEP_OBD_POS3_ADDRESS+2)   // ��Ͼ���״̬
#define GBEP_OBD_POS5_ADDRESS   (GBEP_OBD_POS4_ADDRESS+2)   // ����ʶ����(VIN)
#define GBEP_OBD_POS6_ADDRESS   (GBEP_OBD_POS5_ADDRESS+17) // ����궨ʶ���
#define GBEP_OBD_POS7_ADDRESS   (GBEP_OBD_POS6_ADDRESS+18) // �궨��֤�루CVN��
#define GBEP_OBD_POS8_ADDRESS   (GBEP_OBD_POS7_ADDRESS+18) // IUPRֵ
#define GBEP_OBD_POS9_ADDRESS   (GBEP_OBD_POS8_ADDRESS+36) // ����������
#define GBEP_OBD_POS10_ADDRESS  (GBEP_OBD_POS9_ADDRESS+1)   // ��������Ϣ�б�(N*4B)
#define SIZE_OF_GBEP_OBD        (GBEP_OBD_POS10_ADDRESS-GBEP_OBD_POS1_ADDRESS)

//#define GBEP_OBD_CHK_ADDRESS    (GBEP_ADD_POS7_ADDRESS+2) // У����
//#define NUMBER_OF_GBEP_OBD_POS  (GBEP_OBD_POS10_ADDRESS+1)

//== ������Ϣ���� ==============================================================
#define GBEP_PACKET_HEADER1       0x23  // #
#define GBEP_PACKET_HEADER2       0x23  // #

#define GBEP_PACKET_CMD_LOGIN     0x01  // ��������
#define GBEP_PACKET_CMD_NEW_DATA  0x02  // ʵʱ��Ϣ�ϱ�
#define GBEP_PACKET_CMD_BZ_DATA  0x03 // ������Ϣ�ϱ�
#define GBEP_PACKET_CMD_LOGOUT    0x04  // �����ǳ�
#define GBEP_PACKET_CMD_NTP       0x05  // �ն�Уʱ

#define GBEP_PACKET_SW_VERSION    EP_FIRMWARE_VERSION // �ն�����汾��

#define GBEP_PACKET_ENCRYPT_NONE     0x01  // ���ݲ�����
#define GBEP_PACKET_ENCRYPT_RAS      0x02  // RSA����
#define GBEP_PACKET_ENCRYPT_SM2      0x03  // SM2����
#define GBEP_PACKET_ENCRYPT_ERROR    0xFE  // �쳣
#define GBEP_PACKET_ENCRYPT_INVALID  0xFF  // ��Ч

#define GBEP_DATA_TYPE_OBD           0x01  // OBD��Ϣ
#define GBEP_DATA_TYPE_ENG_SCR       0x02  // ��������Ϣ

#define SIZE_OF_GBEP_ENG_TIME        6
#define GBEP_ENG_PACKET_MAX_NUM      10

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef enum
{
  GBEP_STATE_SEND_INIT = 0x00,
  GBEP_STATE_SEND_IDLE,
  GBEP_STATE_SEND_LOGIN,
  GBEP_STATE_SEND_NTP,
  GBEP_STATE_SEND_OBD,
  GBEP_STATE_SEND_ENG,
  GBEP_STATE_SEND_BZ,
  GBEP_STATE_SEND_LOGOUT
} gbep_state_t;
extern gbep_state_t gbep_state;

extern volatile bittype  gbep_flags1;
#define GBEP_SEND_LOGIN_FLAG  gbep_flags1.b.bit0
#define GBEP_SEND_NTP_FLAG    gbep_flags1.b.bit1
#define GBEP_SEND_OBD_FLAG    gbep_flags1.b.bit2
#define GBEP_SEND_ENG_FLAG    gbep_flags1.b.bit3
#define GBEP_SEND_LOGOUT_FLAG gbep_flags1.b.bit4
#define GBEP_SEND_BZ_FLAG     gbep_flags1.b.bit5
//#define x  gbep_flags1.b.bit6
//#define x  gbep_flags1.b.bit7

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void GBEP_Initialize(void);
void GBEP_StateMachine(void);
void GBEP_ProduceSendData(void);
void GBEP_ProcessRecvData(uint8_t* pdata, uint16_t len);

#endif /* _GB_H_ */
