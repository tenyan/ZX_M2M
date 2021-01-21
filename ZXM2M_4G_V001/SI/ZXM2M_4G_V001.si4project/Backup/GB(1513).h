/*****************************************************************************
* @FileName: GB17691.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.1
* @Date:     2021-1-25
* @brief:    中国国家标准GB17691-2018(重型柴油车远程排放监控技术规范)实现.H文件
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
* Macros(GB环保数据定义)
* 起始符(2B)+命令单元(1B)+车辆识别号(17B)+终端软件版本(1B)+数据加密方式(1B)+数据单元长度(2B)+数据单元(NB)+校验码(1B)
******************************************************************************/
/******************公用帧头地址***********************/
#define GBEP_POS1_ADDRESS    0                       // 起始符
#define GBEP_POS2_ADDRESS    (GBEP_POS1_ADDRESS+2)   // 命令单元
#define GBEP_POS3_ADDRESS    (GBEP_POS2_ADDRESS+1)   // 车辆识别号
#define GBEP_POS4_ADDRESS    (GBEP_POS3_ADDRESS+17)  // 终端软件版本号
#define GBEP_POS5_ADDRESS    (GBEP_POS4_ADDRESS+1)   // 数据加密方式
#define GBEP_POS6_ADDRESS    (GBEP_POS5_ADDRESS+1)   // 数据单元长度
#define SIZE_OF_GBEP_HEADER  (GBEP_POS6_ADDRESS+2)   // 数据单元

//== 发动机数据信息地址 ========================================================
#define GBEP_ENG_POS1_ADDRESS   (0) // 车速
#define GBEP_ENG_POS2_ADDRESS   (GBEP_ENG_POS1_ADDRESS+2) // 大气压力
#define GBEP_ENG_POS3_ADDRESS   (GBEP_ENG_POS2_ADDRESS+1) // 实际扭矩百分比
#define GBEP_ENG_POS4_ADDRESS   (GBEP_ENG_POS3_ADDRESS+1) // 摩擦扭矩百分比
#define GBEP_ENG_POS5_ADDRESS   (GBEP_ENG_POS4_ADDRESS+1) // 发动机转速
#define GBEP_ENG_POS6_ADDRESS   (GBEP_ENG_POS5_ADDRESS+2) // 发动机燃料流量
#define GBEP_ENG_POS7_ADDRESS   (GBEP_ENG_POS6_ADDRESS+2) // 后处理上游氮氧浓度
#define GBEP_ENG_POS8_ADDRESS   (GBEP_ENG_POS7_ADDRESS+2) // 后处理下游氮氧浓度
#define GBEP_ENG_POS9_ADDRESS   (GBEP_ENG_POS8_ADDRESS+2) // 尿素箱液位
#define GBEP_ENG_POS10_ADDRESS  (GBEP_ENG_POS9_ADDRESS+1) // 进气流量
#define GBEP_ENG_POS11_ADDRESS  (GBEP_ENG_POS10_ADDRESS+2) // 后处理上游排气温度
#define GBEP_ENG_POS12_ADDRESS  (GBEP_ENG_POS11_ADDRESS+2) // 后处理下游排气温度
#define GBEP_ENG_POS13_ADDRESS  (GBEP_ENG_POS12_ADDRESS+2) // (DPF/POC)载体压差
#define GBEP_ENG_POS14_ADDRESS  (GBEP_ENG_POS13_ADDRESS+2) // 冷却液温度
#define GBEP_ENG_POS15_ADDRESS  (GBEP_ENG_POS14_ADDRESS+1) // 油箱液位
#define GBEP_ENG_POS16_ADDRESS  (GBEP_ENG_POS15_ADDRESS+1) // 定位状态
#define GBEP_ENG_POS17_ADDRESS  (GBEP_ENG_POS16_ADDRESS+1) // 经度
#define GBEP_ENG_POS18_ADDRESS  (GBEP_ENG_POS17_ADDRESS+4) // 纬度
#define GBEP_ENG_POS19_ADDRESS  (GBEP_ENG_POS18_ADDRESS+4) // 总行驶里程
#define SIZE_OF_GBEP_ENG        (GBEP_ENG_POS19_ADDRESS+4 - GBEP_ENG_POS1_ADDRESS)

//== OBD数据信息地址 ==============================================================
#define GBEP_OBD_POS1_ADDRESS   (0)     // OBD诊断协议
#define GBEP_OBD_POS2_ADDRESS   (GBEP_OBD_POS1_ADDRESS+1)   // MIL状态
#define GBEP_OBD_POS3_ADDRESS   (GBEP_OBD_POS2_ADDRESS+1)   // 诊断支持状态
#define GBEP_OBD_POS4_ADDRESS   (GBEP_OBD_POS3_ADDRESS+2)   // 诊断就绪状态
#define GBEP_OBD_POS5_ADDRESS   (GBEP_OBD_POS4_ADDRESS+2)   // 车辆识别码(VIN)
#define GBEP_OBD_POS6_ADDRESS   (GBEP_OBD_POS5_ADDRESS+17) // 软件标定识别号
#define GBEP_OBD_POS7_ADDRESS   (GBEP_OBD_POS6_ADDRESS+18) // 标定验证码（CVN）
#define GBEP_OBD_POS8_ADDRESS   (GBEP_OBD_POS7_ADDRESS+18) // IUPR值
#define GBEP_OBD_POS9_ADDRESS   (GBEP_OBD_POS8_ADDRESS+36) // 故障码总数
#define GBEP_OBD_POS10_ADDRESS  (GBEP_OBD_POS9_ADDRESS+1)   // 故障码信息列表(N*4B)
#define SIZE_OF_GBEP_OBD        (GBEP_OBD_POS10_ADDRESS-GBEP_OBD_POS1_ADDRESS)

//#define GBEP_OBD_CHK_ADDRESS    (GBEP_ADD_POS7_ADDRESS+2) // 校验码
//#define NUMBER_OF_GBEP_OBD_POS  (GBEP_OBD_POS10_ADDRESS+1)

//== 数据信息定义 ==============================================================
#define GBEP_PACKET_HEADER1       0x23  // #
#define GBEP_PACKET_HEADER2       0x23  // #

#define GBEP_PACKET_CMD_LOGIN     0x01  // 车辆登入
#define GBEP_PACKET_CMD_NEW_DATA  0x02  // 实时信息上报
#define GBEP_PACKET_CMD_BZ_DATA  0x03 // 补发信息上报
#define GBEP_PACKET_CMD_LOGOUT    0x04  // 车辆登出
#define GBEP_PACKET_CMD_NTP       0x05  // 终端校时

#define GBEP_PACKET_SW_VERSION    EP_FIRMWARE_VERSION // 终端软件版本号

#define GBEP_PACKET_ENCRYPT_NONE     0x01  // 数据不加密
#define GBEP_PACKET_ENCRYPT_RAS      0x02  // RSA加密
#define GBEP_PACKET_ENCRYPT_SM2      0x03  // SM2加密
#define GBEP_PACKET_ENCRYPT_ERROR    0xFE  // 异常
#define GBEP_PACKET_ENCRYPT_INVALID  0xFF  // 无效

#define GBEP_DATA_TYPE_OBD           0x01  // OBD信息
#define GBEP_DATA_TYPE_ENG_SCR       0x02  // 数据流信息

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
