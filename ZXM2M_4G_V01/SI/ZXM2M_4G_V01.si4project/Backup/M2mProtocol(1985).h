/*****************************************************************************
* @FileName: M2mProtocol.h
* @Engineer: TenYan & ZPY
* @version   V1.0
* @Date:     2020-10-20
* @brief     M2M Э��ͷ�ļ�
******************************************************************************/
#ifndef _M2M_PROTOCOL_H_
#define _M2M_PROTOCOL_H_

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/
#define RFU_BUFFER_SIZE  1024
extern uint8_t rfu_data_buffer[RFU_BUFFER_SIZE];

/******************************************************************************
 *   Macros
 ******************************************************************************/
// Frame Field Defines
#define M2M_MSG_HEAD_LEN    13  //��Ϣͷ����
#define M2M_MSG_TYPE_FIELD  0
#define M2M_DEV_ID_FIELD    1
#define M2M_FLAG_FIELD      8
#define M2M_SN_FIELD        9
#define M2M_BODY_LEN_FIELD  11

// �������ں���Ӧ��ʱ���ط���������
#define DEFAULT_CONN_DATA_SEND_PERIOD_SP  30
#define DEFAULT_CONN_DATA_RSP_TIMEOUT_SP  10
#define DEFAULT_CONN_DATA_RETRY_SP        5

#define DEFAULT_SS_DATA_SEND_PERIOD_SP  30
#define DEFAULT_SS_DATA_RSP_TIMEOUT_SP  10
#define DEFAULT_SS_DATA_RETRY_SP        5

#define DEFAULT_PING_DATA_SEND_PERIOD_SP  120
#define DEFAULT_PING_DATA_RSP_TIMEOUT_SP  8
#define DEFAULT_PING_DATA_RETRY_SP        2

#define DEFAULT_ALARM_DATA_SEND_PERIOD_SP  120
#define DEFAULT_ALARM_DATA_RSP_TIMEOUT_SP  10
#define DEFAULT_ALARM_DATA_RETRY_SP        3

#define DEFAULT_DTC_DATA_SEND_PERIOD_SP  120
#define DEFAULT_DTC_DATA_RSP_TIMEOUT_SP  10
#define DEFAULT_DTC_DATA_RETRY_SP        3

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// BOOL����
enum
{
  M2M_FALSE = 0x00,
  M2M_TRUE = 0x01
};

// BOOL����
enum
{
  M2M_NOK = 0x00,
  M2M_OK = 0x01
};

// �澯���Ͷ���
typedef enum _m2m_alarm_type__list
{
  M2M_ALARM_TYPE_TOWING = 0x01,        // �ϳ��澯(��������)
  M2M_ALARM_TYPE_OUT_OF_RANGE = 0x02,  // ��Χ��(��������)
  M2M_ALARM_TYPE_OVER_SPEED = 0x03,    // ���ٸ澯(��������)
  M2M_ALARM_TYPE_BUS_COMM_ERR = 0x04,  // �ն����豸������ͨ�Ź���(�ն�Ӳ������)
  M2M_ALARM_TYPE_GPS_MODULE_ERR = 0x05,// GPSģ�����(�ն�Ӳ������)
  M2M_ALARM_TYPE_GPS_ANT_ERR = 0x06,   // GPS���߹���(�ն�Ӳ������)
  M2M_ALARM_TYPE_POWER_LOW = 0x07,     // �ն��ⲿ��Դ�͵�ѹ (�ն�Ӳ������)
  M2M_ALARM_TYPE_POWER_OFF = 0x08,     // �ն��ⲿ��Դ�ϵ�(�ն�Ӳ������)
  M2M_ALARM_TYPE_BAT_LOW = 0x09,       // �ն��ڲ���ص͵�ѹ(�ն�Ӳ������)
  M2M_ALARM_TYPE_CHANGE_SIM = 0x0A,    // SIM����������(�ն�Ӳ������)
  M2M_ALARM_TYPE_GPS_SQ_LOW = 0X0B,    // GPS�ź�ǿ����(�ն�Ӳ������)
  M2M_ALARM_TYPE_MODEM_SQ_LOW = 0x0C,  // MODEM�ź�ǿ����(�ն�Ӳ������)
  M2M_ALARM_TYPE_MODEM_ERR = 0x0D,     // MODEMSģ�����
  M2M_ALARM_TYPE_BOX_OPEN = 0x0E,      // ���Ǹ澯
  NUM_OF_M2M_ALARM_TYPE
}m2m_alarm_type_t;

/// m2mЭ������
typedef enum
{
  M2M_MSG_TYPE_MESSAGEACK = 0, /// ͨ�ñ�����Ӧ
  M2M_MSG_TYPE_CONN_REQ,       /// �ն��������ӷ�����
  M2M_MSG_TYPE_CONN_RESP,      /// ����������Ӧ
  M2M_MSG_TYPE_PUSH_DATA,      /// �ն������˷������ݻ��������ն˷�������
  M2M_MSG_TYPE_ALERT,          /// �ն������˷������ѡ��澯��������Ϣ
  M2M_MSG_TYPE_CMD_REQ,        /// �ն������˷����������󣬻��������ն˷�����������
  M2M_MSG_TYPE_CMD_RESP,       /// ���ն˶��������Ӧ
  M2M_MSG_TYPE_PING_REQ,       /// �ն˶Է���˷��͵���������
  M2M_MSG_TYPE_PING_RESP,      /// ����˶��ն���������Ӧ
  M2M_MSG_TYPE_DISCONNECT,     /// �ն˶Ͽ�����
  M2M_MSG_TYPE_UPDATE,         /// ��������ն�����������Ϣ
  M2M_MSG_TYPE_UPDATE_ACK,     /// �ն���Ӧ����֪ͨ
  M2M_MSG_TYPE_REGIST_REQ,     /// �ն�������ע��
  M2M_MSG_TYPE_REGIST_RESP,    /// �������Ӧע����
  M2M_MSG_TYPE_DEREG_REQ,      /// �ն�������ע��
  M2M_MSG_TYPE_DEREG_RESP,     /// �������Ӧע�����
  M2M_MSG_TYPE_MAX,
}im2m_msg_type_t;

// ������Ϣ����
typedef enum
{
  M2M_UPDATE_TYPE_NONE = 0x00, /// δ֪��������
  M2M_UPDATE_TYPE_UN = 0x01,   /// ��UN��Զ�̹̼�����֪ͨ(ƽ̨->�ն�)
  M2M_UPDATE_TYPE_UQ = 0x02,   /// ��UQ����Ӧ�ն˹̼���������(ƽ̨->�ն�)
  M2M_UPDATE_TYPE_UL = 0x04,   /// ��UL����Ӧ�ն���������������(ƽ̨->�ն�)
  M2M_UPDATE_TYPE_UR = 0x10,   /// ��UR����Ӧ�ն��ϱ��������(ƽ̨->�ն�)
}im2m_update_msg_type_t;

// ������������
typedef enum
{
  M2M_CMD_REQ_TYPE_NONE = 0x00, /// δ֪��������
  M2M_CMD_REQ_TYPE_PW = 0x01,   /// ��PW���趨��������
  M2M_CMD_REQ_TYPE_PR = 0x02,   /// ��PR����ȡ��������
  M2M_CMD_REQ_TYPE_LT = 0x04,   /// ��LT��λ��׷������
  M2M_CMD_REQ_TYPE_RC = 0x10,   /// ��RC��Զ�̿�������
  M2M_CMD_REQ_TYPE_AT = 0x20,   /// ��AT�� ָ��͸������(ͨ���������ն˱��ص���)
}im2m_cmd_req_type_t;

// ����ͷ����
typedef struct
{
  uint8_t msgType;   // ��������
  uint8_t devId[7];  // �豸���
  uint8_t flag;      // ��ʶflag
  uint16_t sn;       // ������ˮ��
  uint16_t bodyLen;  // �����峤��
}im2m_HeaderInfo_t;

// ����ṹ��
typedef struct
{
  uint16_t sn;            // �ϱ�����ˮ��
  uint16_t send_timer_sp; // ���������趨ֵ
  uint16_t send_timer;    // ���Ͷ�ʱ��
  uint8_t sent_flag;      // �����ѷ���
  uint8_t rsp_timeout_sp;    // ��ʱ�趨ֵ
  uint8_t rsp_timeout_timer; // ��ʱ��ʱ��
  uint8_t retry_sp;      // �����趨ֵ
  uint8_t retry_cnt;     // ���Լ�����
}im2m_request_t;

// λ��׷���ϱ���Ϣ�ṹ��(lt=Location Tracking)
typedef struct
{
  uint8_t  mode;     // ģʽ: 0x00=��ʱ��,0x01=�Ⱦ���,0xFF=����
  uint16_t timer;    // ��ʱ��
  uint8_t  timer_sp; // �ϱ����:ʱ��(��),����(0.1KM),����(��Ч)
  uint16_t total_time_sp; // ׷����ʱ�����ܾ���:ʱ��(��),����(KM),����(��Ч),0x00=�ر�׷�ٹ���
}im2m_lt_report_t;

// �����������Ͷ���
typedef union
{
  uint8_t state; // ״̬:0=δ�ϱ�,1=���ϱ�
  uint8_t code;  // ��������
}im2m_alarm_t;

// ����״̬
typedef enum
{
  M2M_UPDATE_STATE_IDLE = 0x00,  /// ����
  M2M_UPDATE_STATE_NOTIFIED,     /// ֪ͨ����
  M2M_UPDATE_STATE_CONN,         /// ��������������
  M2M_UPDATE_STATE_SEND_REQ,     /// ������������
  M2M_UPDATE_STATE_WAIT_RSP,     /// �ȴ�������Ӧ
  M2M_UPDATE_STATE_DOWNLOAD_REQ, /// ������������
  M2M_UPDATE_STATE_DOWNLOAD_RSP, /// �ȴ�������Ӧ
  M2M_UPDATE_STATE_REPORT_REQ,   /// ���ͽ������
  M2M_UPDATE_STATE_REPORT_RSP,   /// �ȴ������Ӧ
  M2M_UPDATE_STATE_ERROR,        /// ����ʧ��
  M2M_UPDATE_STATE_SUCESS,       /// �����ɹ�
}im2m_update_state_t;

#define M2M_FIRMWARE_PACKET_LEN    1024//�̼��������ĳ���
// �̼������ṹ��
typedef struct 
{
  uint8_t state; // ����״̬
  uint8_t sent_flag;         // �����ѷ���
  uint8_t rsp_timeout_timer; // ��ʱ�趨ֵ
  uint8_t retry_cnt;         // ���Լ�����
  uint16_t timeout_cnt;      // ������ʱ������,���10������û��������������
  uint8_t result;            // �°����0=�ɹ�,1=ʧ��
}im2m_update_t;

// m2m�ṹ��
typedef struct
{
  uint8_t srv_ip[4];  // ������IP��ַ
  uint16_t srv_port;  // �������˿ں�
  uint8_t srv_protocol; // ͨ��Э������:0=UDP, 1=TCP
  uint8_t conn_success_flag;  // ���ӳɹ���־
  uint8_t new_srv_address_flag;  //  ��������ַ���±�־

  // ����
  uint16_t upload_sn;  // ��ˮ��
  uint8_t new_alarm_flag; // �±����¼�����(1=��,0=��)
  im2m_alarm_t alarm_table[NUM_OF_M2M_ALARM_TYPE];
  im2m_request_t conn_req;  // ��������
  im2m_request_t ss_req;    // ״̬ͬ������
  im2m_lt_report_t lt_report; // λ��׷���ϱ�
  im2m_request_t ping_req;  // ��������
  im2m_request_t alarm_req; // �澯����
  im2m_request_t dtc_req;   // ����������
  im2m_update_t update;     // Զ�̹̼�����
  uint16_t tx_size;  // �������ݳ���
  uint8_t* tx_data;  // �������ݵ�ַ

  // ����
  uint16_t rx_size;  // �������ݳ���
  uint8_t* rx_data;  // �������ݵ�ַ
  uint8_t rx_from;   // ������Դ(���Թ��ߡ�ƽ̨)
  uint8_t frame_status; // ֡У��״̬
  uint8_t rx_msg_type; // ������Ϣ����
  uint8_t rx_devId[7]; // �����ն�ID
  uint8_t rx_flag;     // ���ճ��ұ��
  uint16_t rx_sn;      // �������
  uint16_t rx_bodyLen; // ��Ϣ�峤��
  uint8_t* rx_body_data;  // �������ݵ�ַ

  // ���ñ���
  uint16_t cmdTypeLen;
  uint8_t *pCmdType;
  uint16_t cmdBodyLen;
  uint8_t *pCmdBody;
}m2m_context_t;
extern m2m_context_t m2m_context;

// M2M�趨����(sp=set point)(dt=debounce time)
#define MAX_SIZE_OF_APN   32
#define MAX_SIZE_OF_USER_NAME   32
#define MAX_SIZE_OF_PASSWORD   32
#define MAX_SIZE_OF_SRV_DN   32
#define MAX_NUM_OF_CAN_ID    90
#define MAX_SIZE_OF_SMSC_NUMBER 20
__packed typedef struct
{
  uint32_t header; // У���ֽ�  0x55AA5AA5

  uint8_t devId[7];    // �ն˵�ID
  uint8_t apn[MAX_SIZE_OF_APN];    // APN
  uint8_t apn_len;    // APN����
  uint8_t user_name[MAX_SIZE_OF_USER_NAME];    // M2Mƽ̨��¼�û���
  uint8_t user_name_len;    // M2Mƽ̨��¼�û�������
  uint8_t password[MAX_SIZE_OF_PASSWORD];    // M2Mƽ̨��¼����
  uint8_t password_len;    // M2Mƽ̨��¼���볤��
  uint8_t smsc_number[MAX_SIZE_OF_SMSC_NUMBER]; // �������ĺ���
  uint8_t smsc_number_len; // �������ĺ��볤��
  uint8_t main_srv_ip[4];  // ������IP��ַ
  uint16_t main_srv_port;  // �����Ķ˿�
  uint8_t main_srv_protocol; // ������ͨ��Э������: 0=UDPЭ��, 1=TCPЭ��
  uint8_t sub_srv_ip[4];   // ������IP��ַ
  uint16_t sub_srv_port;   // �����Ķ˿�
  uint8_t sub_srv_protocol; // ������ͨ��Э������: 0=UDPЭ��, 1=TCPЭ��

  uint32_t hb_timer_sp;  // �������(��),0=����������,Ĭ���������Ϊ30��
  uint8_t login_retry_sp;   // ����¼�ظ�����
  
  uint32_t login_min_time_sp; // ��¼ʧ����С���Լ��
  uint32_t login_max_time_sp; // ��¼ʧ��������Լ��
  uint16_t sms_recv_timeout_sp;  // ���Ž��ճ�ʱʱ��(��)
  
  uint16_t can_baudrate;  // ����CAN���߲�����: 0=Ĭ��250K, 1=125K, 2=250K, 3=500K
  uint16_t can_format;    // ����CAN���ĸ�ʽ: 0=��׼, 1=��չ, 3=���ֶ���
  
  uint32_t canId_tbl[MAX_NUM_OF_CAN_ID];  // CAN ID ��������,4�ֽ�һ��
  uint8_t canId_num;       // can id ����
  
  uint16_t wakeup_work_time_sp;  // ���Ѻ���ʱ��(s)
  uint16_t sleep_time_sp;        // ����ʱ��(��)
  uint8_t ss_report_time_sp;     // �ն˻���״̬ͬ�������Զ����ͼ��(��)
  
  uint8_t msin[6];          // SIM���ţ������λ��0
  
  uint8_t main_srv_dn_len;  // ��������������
  uint8_t main_srv_dn[MAX_SIZE_OF_SRV_DN];  // ����������
  
  uint8_t sub_srv_dn_len;   // ��������������
  uint8_t sub_srv_dn[MAX_SIZE_OF_SRV_DN];   // ����������
  
  uint8_t dns[4];         // dns
  uint16_t hw_version;    // Ӳ���汾��,��V1.5��ʾΪ0x0105
  
  uint16_t vraw_rated;    // ���Դ���ѹ��λ��0.1V
  uint16_t vbat_rated;    // �ն˵�ض��ѹ��λ��0.1V

  uint8_t can_err_dt_sp;  // CAN�����ж�ʱ��
  uint8_t can_ok_dt_sp;   // CAN�ָ������ж�ʱ��
  
  uint8_t power_off_dt_sp;  // �ն˶ϵ�ʱ������
  uint8_t power_on_dt_sp;   // �ն��ϵ�ʱ������
  
  uint8_t low_power_voltage_sp;  // �ⲿ��Դ�͵�ѹ������ֵ(1%)
  uint8_t low_power_dt_sp;       // �ⲿ��Դ�͵�ѹ������ʱ�����(1s)
  
  uint8_t low_bat_voltage_sp;  // �ڲ���Դ�͵�ѹ������ֵ(1%)
  uint8_t low_bat_dt_sp;  // �ⲿ��Դ�͵�ѹ������ʱ�����
  
  uint8_t gps_ant_err_dt_sp; // �ն����߹��ϱ�����ʱ�����(1s)
  uint8_t gps_ant_ok_dt_sp;  // �ն����߹��ϱ����Ľ��ʱ�����(1s)
  
  uint8_t gps_module_err_dt_sp;  // �ն�GPSģ����ϱ�����ʱ�����
  uint8_t gps_module_ok_dt_sp;   // �ն�GPSģ����ϱ��������ʱ�����(1s)
  
  uint8_t over_speed_sp;  // ��ʾ���ٱ�����ֵ(1KM/H)
  uint8_t over_speed_dt_sp;  // ���ٱ�����ʱ�����(1s)
  
  uint8_t towing_alarm_range_sp; //�������ƶ�(�ϳ�)����������ֵ����λ��1KM

  uint8_t work_time_report_cfg;  // �豸����ʱ���ͳ�����ò���
  uint8_t work_data_report_mode_sp; // �������������������ݵ����ϴ�ģʽ
                                    // 0x00:Ĭ��,��ʱ�����ϴ�
                                    // 0x01:����(����ĳ���������ı�Ƶ����ΪƵ�ʷ���)
                                    // 0xFF:���Ե����ϴ�ģʽ����
  uint8_t work_data_report_time_sp; // ��������(����)���������ʱ��(��)
  
  uint8_t position_report_mode_sp;  // λ����Ϣ�����ϴ�ģʽ
  uint8_t position_report_time_sp;  // λ����Ϣ�ϴ����
  
  uint8_t ecu_type;                 // ����������
  uint8_t ep_type;                  // ��������
  uint8_t vin[17];  // ����ʶ�����(Vehicle Identification Number)�򳵼ܺ���
  uint8_t vin_valid_flag;  // VIN����Ч��ʶ
  
  uint8_t xor_value;    // У��ֵ
}m2m_asset_data_t;
extern m2m_asset_data_t m2m_asset_data;
#define SIZEOF_M2M_ASSET_DATA  (sizeof(m2m_asset_data_t))

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

#if 0
typedef enum 
{
  RFU_DOWNLOAD_STATUS_INIT = 0,
  RFU_DOWNLOAD_STATUS_HEADER,
  RFU_DOWNLOAD_STATUS_BODY,
  RFU_DOWNLOAD_STATUS_SUCCESS,
  RFU_DOWNLOAD_STATUS_FAIL
}rfu_download_status_t;

/* ================================================================== */
void rfu_SetDownloadStatus(rfu_context_t* pThis,uint8_t status)
{
  pThis->status = status;
}

/* ================================================================== */
uint8_t rfu_GetDownloadStatus(rfu_context_t* pThis)
{
  return pThis->status;
}
#endif

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void M2M_Initialize(void);
void M2M_ProduceSendMsg(m2m_context_t* pThis);
uint16_t M2M_ProcessRecvMsg(m2m_context_t* pThis);
void M2M_Do1sTasks(void);
void M2M_AddNewAlarmToList(uint8_t type, uint8_t flag);
uint8_t M2M_GetConnStatus(void);

void M2M_NetSocketInit(void);

#endif  /* _M2M_PROTOCOL_H_ */

