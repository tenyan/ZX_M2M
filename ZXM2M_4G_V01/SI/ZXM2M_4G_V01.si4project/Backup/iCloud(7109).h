/*****************************************************************************
* @FileName: iCloud.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-6
* @brief
******************************************************************************/
#ifndef _ICLOUD_H_
#define _ICLOUD_H_

/******************************************************************************
 *   Pre-Processor Defines
 ******************************************************************************/


/******************************************************************************
 *   Macros
 ******************************************************************************/
extern osMessageQueueId_t mqid_SysBusMbox;
#define SYSBUS_PutMbox(msg)          do{osMessageQueuePut(mqid_SysBusMbox, &msg, 0, NULL);}while(0)
#define SYSBUS_GetMbox(msg,timeout)  osMessageQueueGet(mqid_SysBusMbox, &msg, NULL, timeout)

/******************************************************************************
 *   Data Types
 ******************************************************************************/
typedef enum
{
  ICLOUD_FALSE = 0,
  ICLOUD_TRUE  = 1
} icloud_bool_t;
  
typedef enum
{
  ALARM_FALSE = 0, // 消失(NORMAL)
  ALARM_TRUE = 1   // 产生(FAIl)
} alarm_bool_t;

// 总线设备类型定义
enum
{
  SYSBUS_DEVICE_TYPE_CELLURA = 0x01, // 蜂窝网模块
  SYSBUS_DEVICE_TYPE_CAN,     // CAN模块
  SYSBUS_DEVICE_TYPE_GPS,     // GPS模块
  SYSBUS_DEVICE_TYPE_COLLECT, // 采集模块
  SYSBUS_DEVICE_TYPE_PC_DEBUG,  // 配置工具
};

// 蜂窝模块总线消息类型定义
enum
{
  CELLURA_MSG_TYPE_DATA=0x01, // 收到GPRS包
  CELLURA_MSG_TYPE_SMS,       // 收到SMS
  CELLURA_MSG_TYPE_FTP,       // 收到ftp数据包
  CELLURA_MSG_TYPE_RING,      // 收到RING
  CELLURA_MSG_TYPE_ONLINE,    // GSM上线标识,通知外部可发GPRS包
};

// CAN模块总线消息类型定义
// CAN模块总线消息类型定义
enum
{
  CAN_MSG_TYPE_CAN1_COMM_ERR=0x01, // CAN1通信异常
  CAN_MSG_TYPE_CAN1_COMM_OK,  // CAN1通信正常
  CAN_MSG_TYPE_CAN2_COMM_ERR, // CAN2通信异常
  CAN_MSG_TYPE_CAN2_COMM_OK,  // CAN2通信正常
  CAN_MSG_TYPE_CAN1_RECV_STOP,  // CAN1接收停止
  CAN_MSG_TYPE_CAN1_RECV_START, // CAN1接收开始
  CAN_MSG_TYPE_CAN2_RECV_STOP,  // CAN2接收停止
  CAN_MSG_TYPE_CAN2_RECV_START, // CAN2接收开始
};

// GPS模块总线消息类型定义
enum
{
  GPS_MSG_TYPE_OVER_SPEED=0x01,
  GPS_MSG_TYPE_NORMAL_SPEED,
  GPS_MSG_TYPE_ANTENNA_ERR,
  GPS_MSG_TYPE_ANTENNA_OK,
  GPS_MSG_TYPE_MODULE_ERR,
  GPS_MSG_TYPE_MODULE_OK,
  GPS_MSG_TYPE_3D_NOK,
  GPS_MSG_TYPE_3D_OK,
};

// COLLECT模块总线消息类型定义
enum
{
  COLLECT_MSG_TYPE_ACC_ON=0x01,
  COLLECT_MSG_TYPE_ACC_OFF,
  COLLECT_MSG_TYPE_BOX_OPENED,
  COLLECT_MSG_TYPE_BOX_CLOSED,
  COLLECT_MSG_TYPE_MAIN_POWER_LOW,
  COLLECT_MSG_TYPE_MAIN_POWER_NORMAL,
  COLLECT_MSG_TYPE_MAIN_POWER_ON,
  COLLECT_MSG_TYPE_MAIN_POWER_OFF,
  COLLECT_MSG_TYPE_MAIN_BAT_LOW,
  COLLECT_MSG_TYPE_MAIN_BAT_NORMAL,
};

// 总线消息定义
typedef struct
{
  uint8_t device;     // 消息来源设备  1=服务器GPRS；2=备用; 3=MCU模块;
  // 4=GPS模块；5=采集模块;  6=设置程序；
  uint8_t type;       // 消息类型
  uint8_t prior;      // 消息优先级  (暂时保留)
  uint8_t resv;       // 暂时保留
  uint16_t data_size; // 数据长度
  uint8_t* data;      // 数据地址
}sysbus_msg_t;
extern sysbus_msg_t SbusMsg_Collect;
extern sysbus_msg_t SbusMsg_GpsWarning;
extern sysbus_msg_t SbusMsg_Gprs;
extern sysbus_msg_t SbusMsg_Sms;
extern sysbus_msg_t SbusMsg_Ftp;
extern sysbus_msg_t SbusMsg_PcDebug;
extern sysbus_msg_t SbusMsg_Can;

extern uint8_t public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void iCloud_ServiceInit(void);
void iCloud_ServiceStart(void);

#endif /* _ICLOUD_MACHINE_H_ */
