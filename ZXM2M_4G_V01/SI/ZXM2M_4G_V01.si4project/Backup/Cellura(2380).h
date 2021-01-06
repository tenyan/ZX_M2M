/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: Cellura.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-9-25
 * @brief     本文件为4G模块硬件驱动层的头文件
******************************************************************************/
#ifndef _CELLURA_CORE_H_
#define _CELLURA_CORE_H_

//-----头文件调用-------------------------------------------------------------
#include "types.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
typedef uint16_t at_msg_t;
typedef int16_t at_handle_t;
typedef uint8_t  at_buf_t;


#define CELLULAR_MAX_SOCKETS       (6U)

#define TCPIP_ERROR_CODE_SUCCEEDED                0
#define TCPIP_ERROR_CODE_FAILURE                  1
#define TCPIP_ERROR_CODE_NOTOPENED                2
#define TCPIP_ERROR_CODE_WRONG_PARAMETER          3
#define TCPIP_ERROR_CODE_NOT_SUPPORTED            4
#define TCPIP_ERROR_CODE_FAILED_CREATE_SOCKET     5
#define TCPIP_ERROR_CODE_FAILED_BIND_SOCKET       6
#define TCPIP_ERROR_CODE_BUSY                     8
#define TCPIP_ERROR_CODE_SOCKETS_OPENED           9
#define TCPIP_ERROR_CODE_TIMEOUT                  10
#define TCPIP_ERROR_CODE_DNS_FAILED               11
#define TCPIP_ERROR_CODE_UNKNOW_ERROR             255


/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// AT指令编号定义
typedef enum
{
  AT_CMD_IDLE = 0,    //!< 空闲模式
  AT_CMD_CRESET,      //!< 模块软件复位
  AT_CMD_CFUN0_SET,   //!< 模块进入MiniFun
  AT_CMD_CFUN1_SET,   //!< 模块为全功能(online mode)

  AT_CMD_AT,          //!< AT测试
  AT_CMD_ATE0,        //!< 关闭回显
  AT_CMD_CGPS_SET,    //!< 开启GPS会话
  AT_CMD_CGPSXE_SET,  //!< 使能GPS XTRA
  AT_CMD_CGPSINFOCFG_SET,//!< 配置NMEA-0183输出周期5S,输出GPGGA和GPRMC信息
  AT_CMD_CGMR,        //!< 查询模块软件版本
  AT_CMD_CMEE_SET,    //!< 用数字表示错误消息
  AT_CMD_CPIN_GET,    //!< 查询SIM卡是否READY
  AT_CMD_CIMI_GET,    //!< 获取SIM卡的IMSI
  AT_CMD_CICCID_GET,  //!< 获取SIM卡的ICCID
  AT_CMD_CMGF_SET,    //!< 设置短信格式
  AT_CMD_CPMS_SET,    //!< 设置短信存储位置
  AT_CMD_CNMI_SET,    //!< SMS事件上报配置:不上报
  AT_CMD_CNMP_SET,    //!< 设置网络搜索模式为Automatic(LTE/WCDMA/GSM)
  AT_CMD_CSQ_GET,     //!< 查询无线信号质量
  AT_CMD_CREG0_SET,   //!< 关闭+CREG: URC上报
  AT_CMD_CREG2_SET,   //!< 使能+CREG: URC上报
  AT_CMD_CREG_GET,    //!< CS业务:网络注册状态
  AT_CMD_CGREG_GET,   //!< PS业务:GPRS网络注册状态
  AT_CMD_CEREG_GET,   //!< PS业务:LTE网络注册状态
  AT_CMD_CGATT_SET,   //!< 使能PS附着
  AT_CMD_CGATT_GET,   //!< 查询PS附着
  AT_CMD_CGDCONT_SET, //!< 定义PDP上下文
  AT_CMD_CGACT_SET,   //!< 激活PDP上下文
  AT_CMD_CGPADDR_GET, //!< 查询PDP地址

  AT_CMD_CSCLK,       //!< 设置模块进入休眠模式
  AT_CMD_CWMAP0_SET,  //!< 关闭WIFI

  AT_CMD_CMGL,        //!< 列出所有短息
  AT_CMD_CMGD,        //!< 删除所有短信
  AT_CMD_CMGS,
  AT_CMD_CMGS_DATA,

  AT_CMD_END,         //!< Last CMD entry
} at_cmd_t;

// AT指令响应状态
typedef enum
{
  ATCMD_RSP_NONE = 0X00,
  ATCMD_RSP_NOK = 0x01,
  ATCMD_RSP_OK = 0x02,
}atStatus_t;

// enum
typedef enum
{
  CELLULAR_FALSE = 0,
  CELLULAR_TRUE  = 1,
} CS_Bool_t;

// BOOL定义
enum 
{
  AT_FALSE = 0x00,
  AT_TRUE = 0x01
};

// 协议定义
typedef enum
{
  CS_UDP_PROTOCOL = 0,
  CS_TCP_PROTOCOL = 1,
} CS_TransportProtocol_t;

typedef enum
{
  CS_PS_DETACHED = 0,
  CS_PS_ATTACHED = 1,
} CS_PSattach_t;

typedef enum
{
  CS_CS_DETACHED = 0,
  CS_CS_ATTACHED = 1,
} CS_CSattach_t;

// 模块状态定义
typedef enum
{
  MODEM_STATE_INIT = 0x00,
  MODEM_STATE_MINI_FUN,
  MODEM_STATE_RESET,
  MODEM_STATE_DATA_READY,
  MODEM_STATE_SILENCE
}modem_state_t;
extern modem_state_t modem_state;

#define MAX_SIZE_ICCID           ((uint8_t) 20U)  // MAX = 32 characters
#define MAX_SIZE_IMSI            ((uint8_t) 15U)  // MAX = 32 characters
typedef struct
{
  modem_state_t modem_state;      // 模块状态
  uint8_t imsi[MAX_SIZE_IMSI];    // SIM卡IMSI
  uint8_t iccid[MAX_SIZE_ICCID];  // SIM卡ICCID

  uint8_t modem_init_fail_cnt;    // 模块开机次数
  uint8_t at_fail_cnt;            // AT指令错误计数
  uint8_t sim_fail_cnt;           // CPIN指令错误计数
  uint8_t cgatt_fail_cnt;         // 网络附着错误计数
  uint8_t pdpdeact_cnt;           // PDP去激活通知
  
  int sim_err_flag : 1;
  int sms_ready_flag : 1;
  int modem_err_flag : 1;  // 模块故障标志位: 0=正常,1=故障
  
  int sim_card_ok_flag : 1; // SIM卡正常标志位: 1=正常,0=故障
  int sim_card_suspend_flag : 1; // SIM卡欠费标志位: 1=欠费,0=未欠费
  
  int pdp_actived_flag : 1;
  int network_ready_flag : 1;
  
  int cme_error_flag : 1;

  uint8_t csq_rssi;       // 信号强度
  uint8_t csq_ber;

  uint8_t cs_network_regist_status; // 网络注册状态
  uint8_t ps_network_regist_status; // 网络注册状态
  uint8_t network_attach_status; // 网络附着状态

  uint16_t location_area_code;  // 位置码: lac or tac
  uint32_t cell_id;  // 小区信息
}modem_info_t;
extern modem_info_t modem_info;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Modem_SetState(modem_state_t state);
modem_state_t Modem_GetState(void);

#endif  /* _CELLURA_CORE_H_ */

