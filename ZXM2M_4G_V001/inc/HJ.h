/*****************************************************************************
* @FileName: HJ.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.2
* @Date:     2020-1-6
* @brief:    中国国家环境保护部HJ标准(重型车远程排放监控技术规范)实现H文件
******************************************************************************/
#ifndef HJ_H_
#define HJ_H_

/******************************************************************************
* Includes
******************************************************************************/
//#define HJEP_CLOUD_SERVER_IP    "120.195.166.245"
//#define HJEP_CLOUD_SERVER_PORT  10012

#define HJEP_DEBUG       1  // 1-使能, 0-禁止
#define HJEP_BZ_DEBUG    0  // 1-使能, 0-禁止

/******************************************************************************
* Macros(HJ环保数据定义)
* 起始符(2B)+命令单元(1B)+车辆识别号(17B)+终端软件版本(1B)+数据加密方式(1B)+数据单元长度(2B)+数据单元(NB)+校验码(1B)
******************************************************************************/
/******************公用帧头地址***********************/
#define HJEP_POS1_ADDRESS    0                       // 起始符
#define HJEP_POS2_ADDRESS    (HJEP_POS1_ADDRESS+2)   // 命令单元
#define HJEP_POS3_ADDRESS    (HJEP_POS2_ADDRESS+1)   // 车辆识别号
#define HJEP_POS4_ADDRESS    (HJEP_POS3_ADDRESS+17)  // 终端软件版本号
#define HJEP_POS5_ADDRESS    (HJEP_POS4_ADDRESS+1)   // 数据加密方式
#define HJEP_POS6_ADDRESS    (HJEP_POS5_ADDRESS+1)   // 数据单元长度
#define SIZE_OF_HJEP_HEADER  (HJEP_POS6_ADDRESS+2)   // 数据单元

//== 发动机数据信息地址 ========================================================
#define HJEP_ENG_POS1_ADDRESS    (0) // 车速
#define HJEP_ENG_POS2_ADDRESS    (HJEP_ENG_POS1_ADDRESS+2) // 大气压力
#define HJEP_ENG_POS3_ADDRESS    (HJEP_ENG_POS2_ADDRESS+1) // 实际扭矩百分比
#define HJEP_ENG_POS4_ADDRESS    (HJEP_ENG_POS3_ADDRESS+1) // 摩擦扭矩百分比
#define HJEP_ENG_POS5_ADDRESS    (HJEP_ENG_POS4_ADDRESS+1) // 发动机转速
#define HJEP_ENG_POS6_ADDRESS    (HJEP_ENG_POS5_ADDRESS+2) // 发动机燃料流量
#define HJEP_ENG_POS7_ADDRESS    (HJEP_ENG_POS6_ADDRESS+2) // 后处理上游氮氧浓度
#define HJEP_ENG_POS8_ADDRESS    (HJEP_ENG_POS7_ADDRESS+2) // 后处理下游氮氧浓度
#define HJEP_ENG_POS9_ADDRESS    (HJEP_ENG_POS8_ADDRESS+2) // 尿素箱液位
#define HJEP_ENG_POS10_ADDRESS   (HJEP_ENG_POS9_ADDRESS+1) // 进气流量
#define HJEP_ENG_POS11_ADDRESS   (HJEP_ENG_POS10_ADDRESS+2) // 后处理上游排气温度
#define HJEP_ENG_POS12_ADDRESS   (HJEP_ENG_POS11_ADDRESS+2) // 后处理下游排气温度
#define HJEP_ENG_POS13_ADDRESS   (HJEP_ENG_POS12_ADDRESS+2) // (DPF/POC)载体压差
#define HJEP_ENG_POS14_ADDRESS   (HJEP_ENG_POS13_ADDRESS+2) // 冷却液温度
#define HJEP_ENG_POS15_ADDRESS   (HJEP_ENG_POS14_ADDRESS+1) // 油箱液位
#define HJEP_ENG_POS16_ADDRESS   (HJEP_ENG_POS15_ADDRESS+1) // 定位状态
#define HJEP_ENG_POS17_ADDRESS   (HJEP_ENG_POS16_ADDRESS+1) // 经度
#define HJEP_ENG_POS18_ADDRESS   (HJEP_ENG_POS17_ADDRESS+4) // 纬度
#define HJEP_ENG_POS19_ADDRESS   (HJEP_ENG_POS18_ADDRESS+4) // 总行驶里程
#define SIZE_OF_HJEP_ENG_MAND    (HJEP_ENG_POS19_ADDRESS+4 - HJEP_ENG_POS1_ADDRESS)

//== 发动机补充数据信息地址 ======================================================
#define HJEP_ADD_POS1_ADDRESS    (0)  // 发动机扭矩模式
#define HJEP_ADD_POS2_ADDRESS    (HJEP_ADD_POS1_ADDRESS+1)  // 油门踏板
#define HJEP_ADD_POS3_ADDRESS    (HJEP_ADD_POS2_ADDRESS+1)  // 累计油耗(总油耗)
#define HJEP_ADD_POS4_ADDRESS    (HJEP_ADD_POS3_ADDRESS+4)  // 尿素箱温度
#define HJEP_ADD_POS5_ADDRESS    (HJEP_ADD_POS4_ADDRESS+1)  // 实际尿素喷射量
#define HJEP_ADD_POS6_ADDRESS    (HJEP_ADD_POS5_ADDRESS+4)  // 累计尿素消耗(总尿素消耗)
#define HJEP_ADD_POS7_ADDRESS    (HJEP_ADD_POS6_ADDRESS+4)  // DPF排气温度
#define SIZE_OF_HJEP_ENG_ADD     (HJEP_ADD_POS7_ADDRESS+2 - HJEP_ADD_POS1_ADDRESS)

//== OBD数据信息地址 ==============================================================
#define HJEP_OBD_POS1_ADDRESS   (0)     // OBD诊断协议
#define HJEP_OBD_POS2_ADDRESS   (HJEP_OBD_POS1_ADDRESS+1)   // MIL状态
#define HJEP_OBD_POS3_ADDRESS   (HJEP_OBD_POS2_ADDRESS+1)   // 诊断支持状态
#define HJEP_OBD_POS4_ADDRESS   (HJEP_OBD_POS3_ADDRESS+2)   // 诊断就绪状态
#define HJEP_OBD_POS5_ADDRESS   (HJEP_OBD_POS4_ADDRESS+2)   // 车辆识别码(VIN)
#define HJEP_OBD_POS6_ADDRESS   (HJEP_OBD_POS5_ADDRESS+17) // 软件标定识别号
#define HJEP_OBD_POS7_ADDRESS   (HJEP_OBD_POS6_ADDRESS+18) // 标定验证码（CVN）
#define HJEP_OBD_POS8_ADDRESS   (HJEP_OBD_POS7_ADDRESS+18) // IUPR值
#define HJEP_OBD_POS9_ADDRESS   (HJEP_OBD_POS8_ADDRESS+36) // 故障码总数
#define HJEP_OBD_POS10_ADDRESS  (HJEP_OBD_POS9_ADDRESS+1)   // 故障码信息列表(N*4B)
#define SIZE_OF_HJEP_OBD        (HJEP_OBD_POS10_ADDRESS-HJEP_OBD_POS1_ADDRESS)

//== 数据信息定义 ==============================================================
#define HJEP_PACKET_HEADER1       0x23  // #
#define HJEP_PACKET_HEADER2       0x23  // #

#define HJEP_PACKET_CMD_LOGIN     0x01  // 车辆登入
#define HJEP_PACKET_CMD_LOGOUT    0x04  // 车辆登出
#define HJEP_PACKET_CMD_NTP       0x05 // 终端校时
#define HJEP_PACKET_CMD_NEW_DATA  0x02 // 实时信息上报
#define HJEP_PACKET_CMD_BZ_DATA   0x03 // 补发信息上报

#define HJEP_PACKET_SW_VERSION    EP_FIRMWARE_VERSION // 终端软件版本号

#define HJEP_PACKET_ENCRYPT_NONE     0x01  // 数据不加密
#define HJEP_PACKET_ENCRYPT_SM2      0x02  // SM2加密
#define HJEP_PACKET_ENCRYPT_SM4      0x03  // SM4加密
#define HJEP_PACKET_ENCRYPT_RAS      0x04  // RSA加密
#define HJEP_PACKET_ENCRYPT_AES128   0x05  //  AES128加密
#define HJEP_PACKET_ENCRYPT_ERROR    0xFE  // 异常
#define HJEP_PACKET_ENCRYPT_INVALID  0xFF  // 无效

#define HJEP_DATA_TYPE_OBD           0x01  // OBD信息
#define HJEP_DATA_TYPE_ENG_SCR       0x02  // 数据流信息
#define HJEP_DATA_TYPE_ENG_TWC       0x03  // 数据流信息(三元催化)
#define HJEP_DATA_TYPE_ADD           0x80  // 补充数据流(非强制要求)

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

