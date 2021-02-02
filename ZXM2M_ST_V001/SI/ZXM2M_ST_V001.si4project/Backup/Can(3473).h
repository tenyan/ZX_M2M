/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Can.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-6-19
* @brief
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

/******************************************************************************
 * Data Types
 ******************************************************************************/
#define CAN1_RECV_TIMEOUT_SP    10 // can1接收超时时间,单位:100ms
#define CAN2_RECV_TIMEOUT_SP    10 // can2接收超时时间,单位:100ms
typedef struct
{
  uint8_t ep_valid_flag; // 环保数据有效标志: 0=无效, 1=有效
  uint8_t mil_lamp;      // 故障灯状态:0:未点亮, 1=点亮
  uint16_t engine_speed; // 发动机转速

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

// 锁车命令 LVC = Lock Vehicle Control
__packed typedef struct
{

  uint32_t header; // 校验字节  0x55AA5AA5

  // B0=发送监控模式命令,B1=监控模式开启和关闭
  // B2=发送一级锁车命令,B3=一级锁车开启和关闭
  // B4=发送二级锁车命令,B5=二级锁车开启和关闭
  uint8_t lock_cmd_flag;      // 平台下发的锁车和绑定命令标识

  // B0=一级锁车成功, B1=二级锁车成功
  // B2=一级解锁成功,B3=二级解锁成功
  // B4=绑定成功, B5=解绑成功
  uint8_t report_srv_flag;    // 上报平台标志
  
  uint8_t lock_command;       // 锁车级别:  0:解锁, 1~3:锁车级别
  uint8_t bind_command;       // 绑定命令,0:无任何命令, 0xaa:绑定, 0x55:解绑
  
  uint16_t lock_level1_sn;    // 平台下发的一级锁车流水号
  uint16_t lock_level2_sn;    // 平台下发的二级锁车流水号
  uint16_t unlock_sn;         // 平台下发的解锁流水号
  uint16_t bind_sn;           // 平台下发的设置监控模式流水号
  
  uint8_t ecu_rsp_lock_state; // ECU反馈的锁车状态:1=锁车, 0=解锁
  uint8_t ecu_rsp_bind_state; // ECU反馈的绑定状态:1=绑定, 0=解除绑定

  uint8_t xor_value;    // 校验值
}lvc_context_t;
extern lvc_context_t lvc_context;
#define SIZEOF_LVC_CONTEXT  (sizeof(lvc_context_t))

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
  ENGINE_TYPE_WEICHAI = 0x00,
  ENGINE_TYPE_HANGFA = 0x01,
  ENGINE_TYPE_SHANGHAI = 0x02,
  ENGINE_TYPE_YUCHAI = 0x03,
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
uint8_t CAN_GetCommState(uint8_t channel);
uint8_t CAN_GetRecvState(uint8_t channel);
uint8_t CAN_GetVinState(void);

uint8_t CAN_GetEngineType(void);
uint16_t CAN_GetEngineSpeed(void);

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
void Can_Do100msTasks(void);
void Can_Do1sTasks(void);

#endif   /* _Can_H_ */

