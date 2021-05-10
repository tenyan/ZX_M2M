/*****************************************************************************
* @FileName: MomiProtocol.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modem模块和Micro通信协议头文件
******************************************************************************/
#ifndef _MOMI_PROTOCOL_H_
#define _MOMI_PROTOCOL_H_

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MOMI_DEBUG    0
#define MAX_CAN_FRAME_NUM      90   // 容许接收的最大CAN帧数量

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
#define MOMI_FRAME_STX_FIELD        0x00
#define MOMI_FRAME_SIZE_HIGH_FIELD  0x01
#define MOMI_FRAME_SIZE_LOW_FIELD   0x02
#define MOMI_FUNCTION_CODE_FIELD    0x03
#define MOMI_SN_HIGH_FIELD          0x04
#define MOMI_SN_LOW_FIELD           0x05
#define MOMI_DATA_START_FIELD       0x06

//==MOMI TLV E001定义==================================================
#define MOMI_E001_POS1     0                   // 网络工作状态
#define MOMI_E001_POS2     (MOMI_E001_POS1+1)  // SIM卡识别状态
#define MOMI_E001_POS3     (MOMI_E001_POS2+1)  // GPS定位状态
#define MOMI_E001_POS4     (MOMI_E001_POS3+1)  // WIFI工作状态
#define MOMI_E001_POS5     (MOMI_E001_POS4+1)  // ETH工作状态
#define MOMI_E001_POS6     (MOMI_E001_POS5+1)  // 远程升级标志
#define MOMI_E001_POS7     (MOMI_E001_POS6+1)  // GPS时间
#define SIZE_OF_MOMI_E001  (MOMI_E001_POS7+6)  // 总字节数(6B)


/******************************************************************************
* Macros(环保数据定义)
******************************************************************************/
//发动机数据流信息数据
#define EP_POS1_ADDRESS    (0)  // 车速
#define EP_POS2_ADDRESS    (EP_POS1_ADDRESS+2) // 大气压力
#define EP_POS3_ADDRESS    (EP_POS2_ADDRESS+1) // 实际扭矩百分比
#define EP_POS4_ADDRESS    (EP_POS3_ADDRESS+1) // 摩擦扭矩百分比
#define EP_POS5_ADDRESS    (EP_POS4_ADDRESS+1) // 发动机转速
#define EP_POS6_ADDRESS    (EP_POS5_ADDRESS+2) // 发动机燃料流量
#define EP_POS7_ADDRESS    (EP_POS6_ADDRESS+2) // 后处理上游氮氧浓度
#define EP_POS8_ADDRESS    (EP_POS7_ADDRESS+2) // 后处理下游氮氧浓度
#define EP_POS9_ADDRESS    (EP_POS8_ADDRESS+2) // 尿素箱液位
#define EP_POS10_ADDRESS   (EP_POS9_ADDRESS+1) // 进气流量
#define EP_POS11_ADDRESS   (EP_POS10_ADDRESS+2) // 后处理上游排气温度
#define EP_POS12_ADDRESS   (EP_POS11_ADDRESS+2) // 后处理下游排气温度
#define EP_POS13_ADDRESS   (EP_POS12_ADDRESS+2) // (DPF/POC)载体压差
#define EP_POS14_ADDRESS   (EP_POS13_ADDRESS+2) // 冷却液温度(B1)
#define EP_POS15_ADDRESS   (EP_POS14_ADDRESS+1) // 油箱液位(B2)

// 补充数据流数据
#define EP_POS16_ADDRESS   (EP_POS15_ADDRESS+1) // 发动机扭矩模式
#define EP_POS17_ADDRESS   (EP_POS16_ADDRESS+1) // 油门踏板
#define EP_POS18_ADDRESS   (EP_POS17_ADDRESS+1) // 总油耗
#define EP_POS19_ADDRESS   (EP_POS18_ADDRESS+4) // 尿素箱温度
#define EP_POS20_ADDRESS   (EP_POS19_ADDRESS+1) // 实际尿素喷射量
#define EP_POS21_ADDRESS   (EP_POS20_ADDRESS+4) // 总尿素消耗
#define EP_POS22_ADDRESS   (EP_POS21_ADDRESS+4) // DPF排气温度

// 其他信息
#define EP_POS23_ADDRESS   (EP_POS22_ADDRESS+2) // 总行驶里程
#define EP_POS24_ADDRESS   (EP_POS23_ADDRESS+4) // MIL灯状态
#define EP_POS25_ADDRESS   (EP_POS24_ADDRESS+1) // 发动机总运行时间

#define NUMBER_OF_EP_POS   (EP_POS25_ADDRESS+4)

// 环保数据序号和地址定义
//#define ep_data_buffer      g_stuZXMcuData.aucHzepData
extern uint8_t ep_data_buffer[NUMBER_OF_EP_POS];

#define CAN_CHANNEL1        0x00
#define CAN_CHANNEL2        0x01

// OBD协议类型
#define OBD_PROTOCOL_ISO15765    0
#define OBD_PROTOCOL_ISO27145    1
#define OBD_PROTOCOL_SAEJ1939    2

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// BOOL定义
enum
{
  MOMI_NOK = 0x00,
  MOMI_OK = 0x01
};

// CAN帧结构体
typedef struct
{
  uint32_t id;
  uint8_t data[8];
}can_frame_t;

// 故障码
typedef struct
{
  uint32_t dtcode; // 4字节故障码
  uint8_t debounce_tmr; // 去耦计时器
}dtc_t;
// extern dtc_t dtc_table[MAX_DTC_TABLE_SIZE];

#define MAX_ACTIVE_DTC_NUM		    20 	// 可以接收的最多激活故障码数量
// OBD信息结构体
typedef struct
{
  uint8_t protocol_type;  // OBD诊断协议类型 0-IOS15765 1-IOS27145 2-SAEJ1939   0xfe:无效
  uint8_t mil_status;  // 0~1 0:未点亮  1:点亮 0xfe:无效

  uint8_t diag_valid_flag;
  uint16_t diag_supported_status;  // 诊断支持状态:0-不支持, 1-支持
  uint16_t diag_readiness_status;   // 诊断就绪状态:0-不支持, 1-支持

  uint8_t vin_valid_flag;
  uint8_t vin[17];  // Vehicle Identification Number车辆识别码 只读一次

  uint8_t calid_valid_flag;
  uint8_t calid[18];  // Calibration Identifications软件标定识别码由自定义 字母或数字组成 不足的补字符"0"  只读一次

  uint8_t cvn_valid_flag;
  uint8_t cvn[18];  // Calibration Verification Numbers标定验证码 自定义 字母或数字组成 不足的补字符"0" 只读一次

  uint8_t iupr_valid_flag;
  uint8_t iupr[36];  // 参考SAEJ1979-DA G11

  uint8_t dtc_num;  // 故障码总数 0~253 0xfe 无效
  uint32_t dtc[MAX_ACTIVE_DTC_NUM];  // 故障码 每个故障码为4字节
}obd_info_t;
extern obd_info_t obd_info;

// 故障码结构体
#define MAX_DTC_TABLE_SIZE    10  // 故障码最大数量
#define DTC_BUFFER_SIZE       (MAX_DTC_TABLE_SIZE*4)
#define DTC_DEBOUNCE_TIME_SP  30  // 故障码去耦时间
// 故障码
typedef struct
{
  uint8_t new_flag;    // 新DTC标志,0=无,1=有
  uint8_t total_num; // 发动机多故障码总字节数

  uint8_t index; // 多帧故障码索引
  uint8_t buffer[DTC_BUFFER_SIZE]; // 多帧故障码缓存

  uint32_t code[MAX_DTC_TABLE_SIZE]; // 4字节故障码
  uint8_t debounce_tmr[MAX_DTC_TABLE_SIZE]; // 去耦计时器
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

// DTC接口函数
uint8_t DTC_GetNewFlag(dtc_context_t* pThis);
void DTC_ClearNewFlag(dtc_context_t* pThis);
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf);
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis);
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode);

// 获取OBD中的DTC数据
void OBD_GetDtcCode(obd_info_t* pThis, uint8_t *pBuf);

#endif  /* _MOMI_PROTOCOL_H_ */

