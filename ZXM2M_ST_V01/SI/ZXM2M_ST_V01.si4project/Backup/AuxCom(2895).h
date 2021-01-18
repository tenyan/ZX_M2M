/*****************************************************************************
* @FileName: AuxCom.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modem模块和Micro辅助通信协议头文件
******************************************************************************/
#ifndef _AUX_COM_H_
#define _AUX_COM_H_

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define AUXCOM_DEBUG    0
#define AUXCOM_UART_BAUDRATE (115200U)

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
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

// 命令类型
typedef enum
{
  FCLIT_COMMAND_TYPE_NOTIFIE = 0x01,  // 升级通知
  FCLIT_COMMAND_TYPE_DOWNLOAD = 0x02,  // 下载数据
  FCLIT_COMMAND_TYPE_REPORT = 0x03,  // 上报结果
}fclit_command_t;

// 升级状态
typedef enum
{
  FCLIT_STATE_IDLE = 0x00,  /// 空闲
  FCLIT_STATE_NOTIFIED,     /// 被通知升级
  FCLIT_STATE_START,        /// 启动升级
  FCLIT_STATE_SEND_UD_REQ, /// 发送下载请求
  FCLIT_STATE_WAIT_UD_RSP, /// 等待下载响应
  FCLIT_STATE_SEND_UR_REQ,   /// 发送结果请求
  FCLIT_STATE_WAIT_UR_RSP,   /// 等待结果响应
}fclit_state_t;

#define FCLIT_FIRMWARE_PACKET_LEN    1024//固件升级包的长度
// 固件升级结构体
typedef struct 
{
  uint8_t state;  // 升级状态
  uint8_t dev_id;  // 被升级设备号: 0x00=无效, 0x01=控制器, 0x03=协处理器
  uint8_t msg_sn;  // 流水号
  uint16_t fw_packet_index;  // 升级包序号

  uint32_t start_address;  // 已接收长度
  uint32_t ending_address;  // 总长度

  uint16_t fw_block_size; // 分包大小:固定为512或1024
  uint16_t total_block_count;  // 程序总块数
  uint8_t percent;             // 升级进度

  uint8_t retry_sp;  // 重试次数
  uint8_t retry_cnt;  // 重试计数器
  uint8_t timeout_100ms_sp;  // 超时设定值
  uint8_t timeout_100ms;  // 超时计时器

  uint8_t result;  // 下包结果0=成功,1=失败,2=控制器升级成功,3=控制器拒绝升级,4=控制器升级失败

  uint8_t un_result;
  uint8_t ud_result;

  // 发送
  uint16_t tx_size;  // 发送数据长度
  uint8_t* tx_data;  // 发送数据地址

  // 接收
  uint8_t rx_msg_sn; // 接收信息流水号
  uint16_t rx_size;  // 接收数据长度
  uint8_t* rx_data;  // 接收数据地址
}fclit_context_t;
extern fclit_context_t fclit_context;

// BOOL定义
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

