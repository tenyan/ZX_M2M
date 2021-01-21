/*****************************************************************************
* @FileName: icloud_machine.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-11-16
* @brief
******************************************************************************/
#ifndef _ICLOUD_MACHINE_H_
#define _ICLOUD_MACHINE_H_

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
  SYSBUS_DEVICE_TYPE_AUXCOM,  // 辅助通信接口
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
extern sysbus_msg_t SbusMsg_PcDebug;
extern sysbus_msg_t SbusMsg_AuxCom;

extern uint8_t public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void iCloud_ServiceInit(void);
void iCloud_ServiceStart(void);

#endif /* _ICLOUD_MACHINE_H_ */
