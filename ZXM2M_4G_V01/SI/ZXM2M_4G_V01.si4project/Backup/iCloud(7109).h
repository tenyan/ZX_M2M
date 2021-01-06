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
  ALARM_FALSE = 0, // ��ʧ(NORMAL)
  ALARM_TRUE = 1   // ����(FAIl)
} alarm_bool_t;

// �����豸���Ͷ���
enum
{
  SYSBUS_DEVICE_TYPE_CELLURA = 0x01, // ������ģ��
  SYSBUS_DEVICE_TYPE_CAN,     // CANģ��
  SYSBUS_DEVICE_TYPE_GPS,     // GPSģ��
  SYSBUS_DEVICE_TYPE_COLLECT, // �ɼ�ģ��
  SYSBUS_DEVICE_TYPE_PC_DEBUG,  // ���ù���
};

// ����ģ��������Ϣ���Ͷ���
enum
{
  CELLURA_MSG_TYPE_DATA=0x01, // �յ�GPRS��
  CELLURA_MSG_TYPE_SMS,       // �յ�SMS
  CELLURA_MSG_TYPE_FTP,       // �յ�ftp���ݰ�
  CELLURA_MSG_TYPE_RING,      // �յ�RING
  CELLURA_MSG_TYPE_ONLINE,    // GSM���߱�ʶ,֪ͨ�ⲿ�ɷ�GPRS��
};

// CANģ��������Ϣ���Ͷ���
// CANģ��������Ϣ���Ͷ���
enum
{
  CAN_MSG_TYPE_CAN1_COMM_ERR=0x01, // CAN1ͨ���쳣
  CAN_MSG_TYPE_CAN1_COMM_OK,  // CAN1ͨ������
  CAN_MSG_TYPE_CAN2_COMM_ERR, // CAN2ͨ���쳣
  CAN_MSG_TYPE_CAN2_COMM_OK,  // CAN2ͨ������
  CAN_MSG_TYPE_CAN1_RECV_STOP,  // CAN1����ֹͣ
  CAN_MSG_TYPE_CAN1_RECV_START, // CAN1���տ�ʼ
  CAN_MSG_TYPE_CAN2_RECV_STOP,  // CAN2����ֹͣ
  CAN_MSG_TYPE_CAN2_RECV_START, // CAN2���տ�ʼ
};

// GPSģ��������Ϣ���Ͷ���
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

// COLLECTģ��������Ϣ���Ͷ���
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

// ������Ϣ����
typedef struct
{
  uint8_t device;     // ��Ϣ��Դ�豸  1=������GPRS��2=����; 3=MCUģ��;
  // 4=GPSģ�飻5=�ɼ�ģ��;  6=���ó���
  uint8_t type;       // ��Ϣ����
  uint8_t prior;      // ��Ϣ���ȼ�  (��ʱ����)
  uint8_t resv;       // ��ʱ����
  uint16_t data_size; // ���ݳ���
  uint8_t* data;      // ���ݵ�ַ
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
