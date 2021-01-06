/*****************************************************************************
* @FileName: M2mProtocol.h
* @Engineer: TenYan & ZPY
* @version   V1.0
* @Date:     2020-10-20
* @brief     M2M 协议头文件
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
#define M2M_MSG_HEAD_LEN    13  //消息头长度
#define M2M_MSG_TYPE_FIELD  0
#define M2M_DEV_ID_FIELD    1
#define M2M_FLAG_FIELD      8
#define M2M_SN_FIELD        9
#define M2M_BODY_LEN_FIELD  11

// 发送周期和响应超时、重发次数定义
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
// BOOL定义
enum
{
  M2M_FALSE = 0x00,
  M2M_TRUE = 0x01
};

// BOOL定义
enum
{
  M2M_NOK = 0x00,
  M2M_OK = 0x01
};

// 告警类型定义
typedef enum _m2m_alarm_type__list
{
  M2M_ALARM_TYPE_TOWING = 0x01,        // 拖车告警(功能提醒)
  M2M_ALARM_TYPE_OUT_OF_RANGE = 0x02,  // 出围栏(功能提醒)
  M2M_ALARM_TYPE_OVER_SPEED = 0x03,    // 超速告警(功能提醒)
  M2M_ALARM_TYPE_BUS_COMM_ERR = 0x04,  // 终端与设备的总线通信故障(终端硬件报警)
  M2M_ALARM_TYPE_GPS_MODULE_ERR = 0x05,// GPS模块故障(终端硬件报警)
  M2M_ALARM_TYPE_GPS_ANT_ERR = 0x06,   // GPS天线故障(终端硬件报警)
  M2M_ALARM_TYPE_POWER_LOW = 0x07,     // 终端外部电源低电压 (终端硬件报警)
  M2M_ALARM_TYPE_POWER_OFF = 0x08,     // 终端外部电源断电(终端硬件报警)
  M2M_ALARM_TYPE_BAT_LOW = 0x09,       // 终端内部电池低电压(终端硬件报警)
  M2M_ALARM_TYPE_CHANGE_SIM = 0x0A,    // SIM卡换卡报警(终端硬件报警)
  M2M_ALARM_TYPE_GPS_SQ_LOW = 0X0B,    // GPS信号强度弱(终端硬件报警)
  M2M_ALARM_TYPE_MODEM_SQ_LOW = 0x0C,  // MODEM信号强度弱(终端硬件报警)
  M2M_ALARM_TYPE_MODEM_ERR = 0x0D,     // MODEMS模块故障
  M2M_ALARM_TYPE_BOX_OPEN = 0x0E,      // 开盖告警
  NUM_OF_M2M_ALARM_TYPE
}m2m_alarm_type_t;

/// m2m协议类型
typedef enum
{
  M2M_MSG_TYPE_MESSAGEACK = 0, /// 通用报文响应
  M2M_MSG_TYPE_CONN_REQ,       /// 终端请求连接服务器
  M2M_MSG_TYPE_CONN_RESP,      /// 连接请求响应
  M2M_MSG_TYPE_PUSH_DATA,      /// 终端向服务端发送数据或服务端向终端发送数据
  M2M_MSG_TYPE_ALERT,          /// 终端向服务端发送提醒、告警等特殊消息
  M2M_MSG_TYPE_CMD_REQ,        /// 终端向服务端发送命令请求，或服务端向终端发送命令请求
  M2M_MSG_TYPE_CMD_RESP,       /// 接收端对命令的响应
  M2M_MSG_TYPE_PING_REQ,       /// 终端对服务端发送的心跳请求
  M2M_MSG_TYPE_PING_RESP,      /// 服务端对终端心跳的响应
  M2M_MSG_TYPE_DISCONNECT,     /// 终端断开连接
  M2M_MSG_TYPE_UPDATE,         /// 服务端向终端推送升级信息
  M2M_MSG_TYPE_UPDATE_ACK,     /// 终端响应升级通知
  M2M_MSG_TYPE_REGIST_REQ,     /// 终端向服务端注册
  M2M_MSG_TYPE_REGIST_RESP,    /// 服务端响应注册结果
  M2M_MSG_TYPE_DEREG_REQ,      /// 终端向服务端注销
  M2M_MSG_TYPE_DEREG_RESP,     /// 服务端响应注销结果
  M2M_MSG_TYPE_MAX,
}im2m_msg_type_t;

// 升级消息类型
typedef enum
{
  M2M_UPDATE_TYPE_NONE = 0x00, /// 未知命令类型
  M2M_UPDATE_TYPE_UN = 0x01,   /// ‘UN’远程固件升级通知(平台->终端)
  M2M_UPDATE_TYPE_UQ = 0x02,   /// ‘UQ’响应终端固件升级请求(平台->终端)
  M2M_UPDATE_TYPE_UL = 0x04,   /// ‘UL’响应终端请求下载升级包(平台->终端)
  M2M_UPDATE_TYPE_UR = 0x10,   /// ‘UR’响应终端上报升级结果(平台->终端)
}im2m_update_msg_type_t;

// 命令请求类型
typedef enum
{
  M2M_CMD_REQ_TYPE_NONE = 0x00, /// 未知命令类型
  M2M_CMD_REQ_TYPE_PW = 0x01,   /// ‘PW’设定参数命令
  M2M_CMD_REQ_TYPE_PR = 0x02,   /// ‘PR’读取参数命令
  M2M_CMD_REQ_TYPE_LT = 0x04,   /// ‘LT’位置追踪命令
  M2M_CMD_REQ_TYPE_RC = 0x10,   /// ‘RC’远程控制命令
  M2M_CMD_REQ_TYPE_AT = 0x20,   /// ‘AT’ 指令透传命令(通常仅用于终端本地调试)
}im2m_cmd_req_type_t;

// 报文头定义
typedef struct
{
  uint8_t msgType;   // 报文类型
  uint8_t devId[7];  // 设备编号
  uint8_t flag;      // 标识flag
  uint16_t sn;       // 报文流水号
  uint16_t bodyLen;  // 报文体长度
}im2m_HeaderInfo_t;

// 请求结构体
typedef struct
{
  uint16_t sn;            // 上报的流水号
  uint16_t send_timer_sp; // 发送周期设定值
  uint16_t send_timer;    // 发送定时器
  uint8_t sent_flag;      // 请求已发出
  uint8_t rsp_timeout_sp;    // 超时设定值
  uint8_t rsp_timeout_timer; // 超时计时器
  uint8_t retry_sp;      // 重试设定值
  uint8_t retry_cnt;     // 重试计数器
}im2m_request_t;

// 位置追踪上报信息结构体(lt=Location Tracking)
typedef struct
{
  uint8_t  mode;     // 模式: 0x00=等时间,0x01=等距离,0xFF=单次
  uint16_t timer;    // 计时器
  uint8_t  timer_sp; // 上报间隔:时间(秒),距离(0.1KM),单次(无效)
  uint16_t total_time_sp; // 追踪总时长或总距离:时间(分),距离(KM),单次(无效),0x00=关闭追踪功能
}im2m_lt_report_t;

// 报警代码类型定义
typedef union
{
  uint8_t state; // 状态:0=未上报,1=已上报
  uint8_t code;  // 报警代码
}im2m_alarm_t;

// 升级状态
typedef enum
{
  M2M_UPDATE_STATE_IDLE = 0x00,  /// 空闲
  M2M_UPDATE_STATE_NOTIFIED,     /// 通知升级
  M2M_UPDATE_STATE_CONN,         /// 连接升级服务器
  M2M_UPDATE_STATE_SEND_REQ,     /// 发送升级请求
  M2M_UPDATE_STATE_WAIT_RSP,     /// 等待升级响应
  M2M_UPDATE_STATE_DOWNLOAD_REQ, /// 发送下载请求
  M2M_UPDATE_STATE_DOWNLOAD_RSP, /// 等待下载响应
  M2M_UPDATE_STATE_REPORT_REQ,   /// 发送结果请求
  M2M_UPDATE_STATE_REPORT_RSP,   /// 等待结果响应
  M2M_UPDATE_STATE_ERROR,        /// 升级失败
  M2M_UPDATE_STATE_SUCESS,       /// 升级成功
}im2m_update_state_t;

#define M2M_FIRMWARE_PACKET_LEN    1024//固件升级包的长度
// 固件升级结构体
typedef struct 
{
  uint8_t state; // 升级状态
  uint8_t sent_flag;         // 请求已发出
  uint8_t rsp_timeout_timer; // 超时设定值
  uint8_t retry_cnt;         // 重试计数器
  uint16_t timeout_cnt;      // 升级超时计数器,如果10分钟内没有升级完成则放弃
  uint8_t result;            // 下包结果0=成功,1=失败
}im2m_update_t;

// m2m结构体
typedef struct
{
  uint8_t srv_ip[4];  // 服务器IP地址
  uint16_t srv_port;  // 服务器端口号
  uint8_t srv_protocol; // 通信协议类型:0=UDP, 1=TCP
  uint8_t conn_success_flag;  // 连接成功标志
  uint8_t new_srv_address_flag;  //  服务器地址更新标志

  // 上行
  uint16_t upload_sn;  // 流水号
  uint8_t new_alarm_flag; // 新报警事件发生(1=有,0=无)
  im2m_alarm_t alarm_table[NUM_OF_M2M_ALARM_TYPE];
  im2m_request_t conn_req;  // 连接请求
  im2m_request_t ss_req;    // 状态同步请求
  im2m_lt_report_t lt_report; // 位置追踪上报
  im2m_request_t ping_req;  // 心跳请求
  im2m_request_t alarm_req; // 告警请求
  im2m_request_t dtc_req;   // 故障码请求
  im2m_update_t update;     // 远程固件升级
  uint16_t tx_size;  // 发送数据长度
  uint8_t* tx_data;  // 发送数据地址

  // 下行
  uint16_t rx_size;  // 接收数据长度
  uint8_t* rx_data;  // 接收数据地址
  uint8_t rx_from;   // 数据来源(调试工具、平台)
  uint8_t frame_status; // 帧校验状态
  uint8_t rx_msg_type; // 接收消息类型
  uint8_t rx_devId[7]; // 接收终端ID
  uint8_t rx_flag;     // 接收厂家编号
  uint16_t rx_sn;      // 命令序号
  uint16_t rx_bodyLen; // 消息体长度
  uint8_t* rx_body_data;  // 接收数据地址

  // 公用变量
  uint16_t cmdTypeLen;
  uint8_t *pCmdType;
  uint16_t cmdBodyLen;
  uint8_t *pCmdBody;
}m2m_context_t;
extern m2m_context_t m2m_context;

// M2M设定参数(sp=set point)(dt=debounce time)
#define MAX_SIZE_OF_APN   32
#define MAX_SIZE_OF_USER_NAME   32
#define MAX_SIZE_OF_PASSWORD   32
#define MAX_SIZE_OF_SRV_DN   32
#define MAX_NUM_OF_CAN_ID    90
#define MAX_SIZE_OF_SMSC_NUMBER 20
__packed typedef struct
{
  uint32_t header; // 校验字节  0x55AA5AA5

  uint8_t devId[7];    // 终端的ID
  uint8_t apn[MAX_SIZE_OF_APN];    // APN
  uint8_t apn_len;    // APN长度
  uint8_t user_name[MAX_SIZE_OF_USER_NAME];    // M2M平台登录用户名
  uint8_t user_name_len;    // M2M平台登录用户名长度
  uint8_t password[MAX_SIZE_OF_PASSWORD];    // M2M平台登录密码
  uint8_t password_len;    // M2M平台登录密码长度
  uint8_t smsc_number[MAX_SIZE_OF_SMSC_NUMBER]; // 短信中心号码
  uint8_t smsc_number_len; // 短信中心号码长度
  uint8_t main_srv_ip[4];  // 主中心IP地址
  uint16_t main_srv_port;  // 主中心端口
  uint8_t main_srv_protocol; // 主中心通信协议类型: 0=UDP协议, 1=TCP协议
  uint8_t sub_srv_ip[4];   // 副中心IP地址
  uint16_t sub_srv_port;   // 副中心端口
  uint8_t sub_srv_protocol; // 副中心通信协议类型: 0=UDP协议, 1=TCP协议

  uint32_t hb_timer_sp;  // 心跳间隔(秒),0=不发送心跳,默认心跳间隔为30秒
  uint8_t login_retry_sp;   // 最大登录重复次数
  
  uint32_t login_min_time_sp; // 登录失败最小重试间隔
  uint32_t login_max_time_sp; // 登录失败最大重试间隔
  uint16_t sms_recv_timeout_sp;  // 短信接收超时时间(秒)
  
  uint16_t can_baudrate;  // 本地CAN总线波特率: 0=默认250K, 1=125K, 2=250K, 3=500K
  uint16_t can_format;    // 本地CAN报文格式: 0=标准, 1=扩展, 3=两种都有
  
  uint32_t canId_tbl[MAX_NUM_OF_CAN_ID];  // CAN ID 过滤配置,4字节一组
  uint8_t canId_num;       // can id 个数
  
  uint16_t wakeup_work_time_sp;  // 唤醒后工作时间(s)
  uint16_t sleep_time_sp;        // 休眠时间(分)
  uint8_t ss_report_time_sp;     // 终端基本状态同步数据自动发送间隔(秒)
  
  uint8_t msin[6];          // SIM卡号，不足高位补0
  
  uint8_t main_srv_dn_len;  // 主中心域名长度
  uint8_t main_srv_dn[MAX_SIZE_OF_SRV_DN];  // 主中心域名
  
  uint8_t sub_srv_dn_len;   // 副中心域名长度
  uint8_t sub_srv_dn[MAX_SIZE_OF_SRV_DN];   // 副中心域名
  
  uint8_t dns[4];         // dns
  uint16_t hw_version;    // 硬件版本号,如V1.5表示为0x0105
  
  uint16_t vraw_rated;    // 外电源额定电压单位：0.1V
  uint16_t vbat_rated;    // 终端电池额定电压单位：0.1V

  uint8_t can_err_dt_sp;  // CAN故障判断时间
  uint8_t can_ok_dt_sp;   // CAN恢复正常判断时间
  
  uint8_t power_off_dt_sp;  // 终端断电时间条件
  uint8_t power_on_dt_sp;   // 终端上电时间条件
  
  uint8_t low_power_voltage_sp;  // 外部电源低电压报警阈值(1%)
  uint8_t low_power_dt_sp;       // 外部电源低电压报警的时间参数(1s)
  
  uint8_t low_bat_voltage_sp;  // 内部电源低电压报警阈值(1%)
  uint8_t low_bat_dt_sp;  // 外部电源低电压报警的时间参数
  
  uint8_t gps_ant_err_dt_sp; // 终端天线故障报警的时间参数(1s)
  uint8_t gps_ant_ok_dt_sp;  // 终端天线故障报警的解除时间参数(1s)
  
  uint8_t gps_module_err_dt_sp;  // 终端GPS模块故障报警的时间参数
  uint8_t gps_module_ok_dt_sp;   // 终端GPS模块故障报警解除的时间参数(1s)
  
  uint8_t over_speed_sp;  // 表示超速报警阈值(1KM/H)
  uint8_t over_speed_dt_sp;  // 超速报警的时间参数(1s)
  
  uint8_t towing_alarm_range_sp; //非自主移动(拖车)报警距离阈值，单位：1KM

  uint8_t work_time_report_cfg;  // 设备工作时间段统计配置参数
  uint8_t work_data_report_mode_sp; // 工作参数（工况）数据单条上传模式
                                    // 0x00:默认,等时间间隔上传
                                    // 0x01:其他(如以某工况参数的变频函数为频率发送)
                                    // 0xFF:不以单条上传模式传输
  uint8_t work_data_report_time_sp; // 工作参数(工况)传输参数。时间(秒)
  
  uint8_t position_report_mode_sp;  // 位置信息单条上传模式
  uint8_t position_report_time_sp;  // 位置信息上传间隔
  
  uint8_t ecu_type;                 // 发动机类型
  uint8_t ep_type;                  // 环保类型
  uint8_t vin[17];  // 车辆识别号码(Vehicle Identification Number)或车架号码
  uint8_t vin_valid_flag;  // VIN码有效标识
  
  uint8_t xor_value;    // 校验值
}m2m_asset_data_t;
extern m2m_asset_data_t m2m_asset_data;
#define SIZEOF_M2M_ASSET_DATA  (sizeof(m2m_asset_data_t))

#define DEFAULT_RFU_FILE_NAME_SIZE     50
#define DEFAULT_RFU_FILE_VERSION_SIZE  16
#define DEFAULT_MD5_VAL_SIZE           32
// 远程固件升级rfu = remote firmware update
__packed typedef struct
{
  uint8_t type;     // 方式: 0=询问升级,1=强制升级
  uint8_t dev;      // 目标设备: 0x00=终端,0x01=控制器,0x02=显示器,0x03:其他

  uint8_t srv_ip[4];  // 服务器IP地址
  uint16_t srv_port;  // 服务器端口号
  uint8_t srv_protocol;   // 通信协议类型:0=UDP, 1=TCP

  uint8_t file_name[DEFAULT_RFU_FILE_NAME_SIZE]; // 文件名
  uint8_t file_name_length; // 文件名长度

  uint8_t file_version[DEFAULT_RFU_FILE_VERSION_SIZE]; // 版本号
  uint8_t file_version_length; // 版本号长度

  //uint8_t file_md5val[DEFAULT_MD5_VAL_SIZE];
  //uint8_t md5val[DEFAULT_MD5_VAL_SIZE];

  uint32_t file_length;        // 明文文件大小
  uint32_t plain_crc32val;    // 明文CRC32校验
  //uint32_t cipher_file_length; // 密文文件大小
  //uint32_t cipher_crc32val; // 密文CRC32校验
  uint32_t crc32val;          // 计算出CRC32校验

  uint32_t cumulated_address;  // 已接收长度
  uint32_t ending_address;     // 总长度

  uint16_t block;              // 当前下来块序号
  uint16_t total_block_count;  // 程序总块数
  uint8_t percent;             // 升级进度

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

