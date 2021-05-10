/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Can.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-6-19
* @brief:
******************************************************************************/
#ifndef _CAN_H_
#define _CAN_H_

#include "CanHW.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MCU_DATA_LENGTH        1460// 普通MCU数据的长度, MCU增加了控制器、显示器的版本等信息2013-05-03
#define TIME_COMM_UNNORMAL     30   // 通信异常判定时间

#define MAX_CAN_FRAME_NUM      90   // 容许接收的最大CAN帧数量
#define MAX_CAN_RCV_TIMEOUT_SP 100  //can1接收无数据超时时间,单位:10ms

#define HZEP_FAULTCODE_TIMEOUT_SP  30 //故障码超时最大时间30秒

/******************************************************************************
 * Data Types
 ******************************************************************************/
#define CAN1_RECV_TIMEOUT_SP    10 // can1接收超时时间,单位:100ms
#define CAN2_RECV_TIMEOUT_SP    10 // can2接收超时时间,单位:100ms
typedef struct
{
  uint8_t ep_valid_flag; // 环保数据有效标志: 0=无效, 1=有效
  uint8_t mil_lamp;      // 故障灯状态:0:未点亮, 1=点亮
  uint8_t got_ci_code_flag; // 获取CI码标志: 0=未获取, 1=已获取
  uint16_t up_engine_speed;  // 上车发动机转速
  uint16_t dw_engine_speed;  // 下车发动机转速

  uint8_t comm_state1;  // 通信状态:0=异常, 1=正常
  uint8_t recv_state1;  // 接收状态:0=未收到数据, 1=已收到数据
  uint16_t recv_timer1; // 接收超时计数器

  uint8_t comm_state2;  // 通信状态:0=异常, 1=正常
  uint8_t recv_state2;  // 接收状态:0=未收到数据, 1=已收到数据
  uint16_t recv_timer2; // 接收超时计数器

  uint8_t  sleep_state; // 休眠状态,0=未休眠, 1=休眠

  uint32_t twt_up;  // 上车总工作时间
  uint32_t twt_down;  // 下车总工作时间
  uint32_t tfc_up;  // 上车总油耗
  uint32_t tfc_down;  // 下车总油耗
  uint32_t odo_down;  // 下车总里程

  uint8_t pid_up_type;      // 上车类型状态字
  uint8_t pid_up_config1;   // 上车配置状态字1
  uint8_t pid_up_config2;   // 上车配置状态字2
  uint8_t pid_up_can;       // 上车协议类型状态字
  uint8_t pid_down_type;    // 底盘类型状态字
  uint8_t pid_down_config1; // 底盘配置状态字1
  uint8_t pid_down_config2; // 底盘配置状态字2
  uint8_t pid_down_can;     // 底盘CAN协议
  uint8_t pid_save_flag;    // 保存协议信息标志位

  uint8_t hzep_fault_data[100];  // 杭州环保发动机故障数据缓冲
  uint16_t hzep_fault_data_len;  // 杭州环保发动机故障数据长度
  uint8_t hzep_fault_data_flag;  // 0-未收到故障码或者收到的故障码已经超时,超时时间为30秒
  uint8_t hzep_fault_data_timer;  // 杭州环保故障超时计时器  暂定30秒
  uint8_t hzep_new_fdata_flag;  // 接收到新的数据帧

  uint8_t eng_ci_buffer[100];  // 多帧CI码缓存
  uint8_t eng_ci_index;        // 多帧CI码索引
  uint8_t eng_ci_new_flag;     // 接收到新的数据帧

  uint8_t ecu_type;  // 发动机类型
  uint8_t ep_type;   // 环保类型
  uint8_t vin[17];   // 车辆识别号码(Vehicle Identification Number)或车架号码
  uint8_t vin_valid_flag;  // VIN码有效标识
}can_context_t;
extern can_context_t can_context;

// BOOL定义
enum
{
  CAN_NOK = 0x00,
  CAN_OK = 0x01
};

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
  uint8_t mil_status;			// 0~1 0:未点亮  1:点亮 0xfe:无效

  uint8_t diag_valid_flag;
  uint16_t diag_supported_status; // 诊断支持状态:0-不支持, 1-支持
  uint16_t diag_readiness_status;	  // 诊断就绪状态:0-不支持, 1-支持

  uint8_t vin_valid_flag;
  uint8_t vin[17];		// Vehicle Identification Number车辆识别码 只读一次

  uint8_t calid_valid_flag;
  uint8_t calid[18];	// Calibration Identifications软件标定识别码由自定义 字母或数字组成 不足的补字符"0"  只读一次

  uint8_t cvn_valid_flag;
  uint8_t cvn[18];		// Calibration Verification Numbers标定验证码 自定义 字母或数字组成 不足的补字符"0" 只读一次

  uint8_t iupr_valid_flag;
  uint8_t iupr[36];	// 参考SAEJ1979-DA G11

  uint8_t dtc_cnt;  // 故障码总数 0~253 0xfe 无效
  uint32_t dtc[MAX_ACTIVE_DTC_NUM];	// 故障码 每个故障码为4字节
}obd_info_t;
extern obd_info_t obd_info;

// 发动机类型定义
enum
{
  ENGINE_TYPE_NONE = 0x00,
  ENGINE_TYPE_WEICHAI,
  ENGINE_TYPE_HANGFA,
  ENGINE_TYPE_SHANGCHAI,
  ENGINE_TYPE_YUCHAI,
  ENGINE_TYPE_BENZ,
  NUMBER_OF_ENGINE_TYPE
};

// 环保类型定义
enum
{
  EP_TYPE_NONE = 0x00,
  EP_TYPE_HJ = 0x01,
  EP_TYPE_GB = 0x02,
};

/******************************************************************************
 * Data Types
 ******************************************************************************/
// CAN消息队列
#define CAN_MSG_QUEUE_MAX_SIZE  16
typedef struct
{
  uint8_t head;  // 队列头
  uint8_t tail;  // 队列尾
  CanRxMsg msg_buff[CAN_MSG_QUEUE_MAX_SIZE];  // 数据缓冲区
}can_msg_queue_t;
extern can_msg_queue_t can1_msg_queue;
extern can_msg_queue_t can2_msg_queue;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
// DTC接口函数
uint8_t DTC_GetNewFlag(dtc_context_t* pThis);
void DTC_ClearNewFlag(dtc_context_t* pThis);
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf);
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis);
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode);

// 用户API函数
uint8_t CAN1_GetCommState(void);
uint8_t CAN2_GetCommState(void);
uint8_t CAN1_GetRecvState(void);
uint8_t CAN2_GetRecvState(void);

uint8_t CAN_GetVinState(void);
uint8_t CAN_GetObdVinState(void);
uint8_t CAN_GetUserVinState(void);
uint8_t CAN_GetCiCodeState(void);

uint8_t CAN_GetEngineType(void);
uint16_t CAN_GetEngineSpeed(void);

uint8_t CAN_GetDwEngineState(void);
uint8_t CAN_GetUpEngineState(void);

uint32_t CAN_GetUpEngineTwt(void);    // 上车总工作时间
uint32_t CAN_GetDownEngineTwt(void);    // 下车总工作时间

uint32_t CAN_GetUpEngineTfc(void);// 上车总油耗
uint32_t CAN_GetDownEngineTfc(void);// 下车总油耗

// 消息队列
void can_msg_queue_reset(can_msg_queue_t* pThis);
void can_msg_queue_push(can_msg_queue_t* pThis, CanRxMsg* pMsg);
static void can_msg_queue_pop(can_msg_queue_t* pThis, can_msg_t* pMsg);
static uint8_t can_msg_queue_size(can_msg_queue_t* pThis);

// 接口函数
void Can_ServiceInit(void);
void Can_ServiceStart(void);
void Can_Do10msTasks(void);
void Can_Do100msTasks(void);
void Can_Do1sTasks(void);

#endif   /* _Can_H_ */

