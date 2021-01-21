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
  SYSBUS_DEVICE_TYPE_AUXCOM,  // ����ͨ�Žӿ�
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
extern sysbus_msg_t SbusMsg_PcDebug;
extern sysbus_msg_t SbusMsg_AuxCom;

extern uint8_t public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void iCloud_ServiceInit(void);
void iCloud_ServiceStart(void);

#endif /* _ICLOUD_MACHINE_H_ */
