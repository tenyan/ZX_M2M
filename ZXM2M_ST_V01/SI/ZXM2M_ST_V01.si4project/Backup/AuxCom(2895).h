/*****************************************************************************
* @FileName: AuxCom.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modemģ���Micro����ͨ��Э��ͷ�ļ�
******************************************************************************/
#ifndef _AUX_COM_H_
#define _AUX_COM_H_

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define AUXCOM_DEBUG    0
#define AUXCOM_UART_BAUDRATE (115200U)

// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
#define MOMI_FRAME_STX_FIELD        0x00
#define MOMI_FRAME_SIZE_HIGH_FIELD  0x01
#define MOMI_FRAME_SIZE_LOW_FIELD   0x02
#define MOMI_FUNCTION_CODE_FIELD    0x03
#define MOMI_SN_HIGH_FIELD          0x04
#define MOMI_SN_LOW_FIELD           0x05
#define MOMI_DATA_START_FIELD       0x06

#define FCLIT_DEBUG    1

/******************************************************************************
 *   Data Types
 ******************************************************************************/
#define DEFAULT_RFU_FILE_NAME_SIZE     50
#define DEFAULT_RFU_FILE_VERSION_SIZE  16
#define DEFAULT_MD5_VAL_SIZE           32
// Զ�̹̼�����rfu = remote firmware update
__packed typedef struct
{
  uint8_t type;     // ��ʽ: 0=ѯ������,1=ǿ������
  uint8_t dev;      // Ŀ���豸: 0x00=�ն�,0x01=������,0x02=��ʾ��,0x03:����

  uint8_t srv_ip[4];  // ������IP��ַ
  uint16_t srv_port;  // �������˿ں�
  uint8_t srv_protocol;   // ͨ��Э������:0=UDP, 1=TCP

  uint8_t file_name[DEFAULT_RFU_FILE_NAME_SIZE]; // �ļ���
  uint8_t file_name_length; // �ļ�������

  uint8_t file_version[DEFAULT_RFU_FILE_VERSION_SIZE]; // �汾��
  uint8_t file_version_length; // �汾�ų���

  //uint8_t file_md5val[DEFAULT_MD5_VAL_SIZE];
  //uint8_t md5val[DEFAULT_MD5_VAL_SIZE];

  uint32_t file_length;        // �����ļ���С
  uint32_t plain_crc32val;    // ����CRC32У��
  //uint32_t cipher_file_length; // �����ļ���С
  //uint32_t cipher_crc32val; // ����CRC32У��
  uint32_t crc32val;          // �����CRC32У��

  uint32_t cumulated_address;  // �ѽ��ճ���
  uint32_t ending_address;     // �ܳ���

  uint16_t block;              // ��ǰ���������
  uint16_t total_block_count;  // �����ܿ���
  uint8_t percent;             // ��������

  uint8_t status;
}rfu_context_t;
extern rfu_context_t rfu_context;

// ��������
typedef enum
{
  FCLIT_COMMAND_TYPE_NOTIFIE = 0x01,  // ����֪ͨ
  FCLIT_COMMAND_TYPE_DOWNLOAD = 0x02,  // ��������
  FCLIT_COMMAND_TYPE_REPORT = 0x03,  // �ϱ����
}fclit_command_t;

// ����״̬
typedef enum
{
  FCLIT_STATE_IDLE = 0x00,  /// ����
  FCLIT_STATE_NOTIFIED,     /// ��֪ͨ����
  FCLIT_STATE_START,        /// ��������
  FCLIT_STATE_SEND_UD_REQ, /// ������������
  FCLIT_STATE_WAIT_UD_RSP, /// �ȴ�������Ӧ
  FCLIT_STATE_SEND_UR_REQ,   /// ���ͽ������
  FCLIT_STATE_WAIT_UR_RSP,   /// �ȴ������Ӧ
}fclit_state_t;

#define FCLIT_FIRMWARE_PACKET_LEN    1024//�̼��������ĳ���
// �̼������ṹ��
typedef struct 
{
  uint8_t state;  // ����״̬
  uint8_t dev_id;  // �������豸��: 0x00=��Ч, 0x01=������, 0x03=Э������
  uint8_t msg_sn;  // ��ˮ��
  uint16_t fw_packet_index;  // ���������

  uint32_t start_address;  // �ѽ��ճ���
  uint32_t ending_address;  // �ܳ���

  uint16_t fw_block_size; // �ְ���С:�̶�Ϊ512��1024
  uint16_t total_block_count;  // �����ܿ���
  uint8_t percent;             // ��������

  uint8_t retry_sp;  // ���Դ���
  uint8_t retry_cnt;  // ���Լ�����
  uint8_t timeout_100ms_sp;  // ��ʱ�趨ֵ
  uint8_t timeout_100ms;  // ��ʱ��ʱ��

  uint8_t result;  // �°����0=�ɹ�,1=ʧ��,2=�����������ɹ�,3=�������ܾ�����,4=����������ʧ��

  uint8_t un_result;
  uint8_t ud_result;

  // ����
  uint16_t tx_size;  // �������ݳ���
  uint8_t* tx_data;  // �������ݵ�ַ

  // ����
  uint8_t rx_msg_sn; // ������Ϣ��ˮ��
  uint16_t rx_size;  // �������ݳ���
  uint8_t* rx_data;  // �������ݵ�ַ
}fclit_context_t;
extern fclit_context_t fclit_context;

// BOOL����
enum
{
  FCLIT_NOK = 0x00,
  FCLIT_OK = 0x01
};

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void AuxCom_ServiceInit(void);
void AuxCom_ServiceStart(void);

#endif  /* _AUX_COM_H_ */

