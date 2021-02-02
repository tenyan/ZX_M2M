/*****************************************************************************
* @FileName: iCloud.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-6
* @brief
******************************************************************************/
#ifndef _ICLOUD_H_
#define _ICLOUD_H_

#include "NetSocket.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
// ���Ӷ���
enum
{
  ZXM2M_SOCKET_ID = 0, // ZXM2M����
  MAINEP_SOCKET_ID,    // ����������
  SUBEP_SOCKET_ID,     // ����������
  NUMBER_OF_SOCKET_ID
};

extern skt_context_t skt_context[NUMBER_OF_SOCKET_ID];
#define zxm2m_socket  skt_context[ZXM2M_SOCKET_ID] // ZXM2M����
#define ZxM2mSocketFd  zxm2m_socket.socket_fd

#define mainep_socket   skt_context[MAINEP_SOCKET_ID]  // ��������(��)
#define MainEpSocketFd   mainep_socket.socket_fd

#define subep_socket   skt_context[SUBEP_SOCKET_ID]  // ��������(��)
#define SubEpSocketFd   subep_socket.socket_fd

//==HJ����Э��(����HJƽ̨,ת������ƽ̨)============
#define hjep_socket   skt_context[MAINEP_SOCKET_ID]  // HJ��������
#define HJEP_SOCKET_ID  MAINEP_SOCKET_ID // HJ����
#define HjepSocketFd   mainep_socket.socket_fd

//==GB����Э��(��������ƽ̨(ֱ��)������GBƽ̨)====
#define bjep_socket   skt_context[MAINEP_SOCKET_ID]  // ������������(��������)
#define BJEP_SOCKET_ID  MAINEP_SOCKET_ID // HJ����
#define BjepSocketFd   mainep_socket.socket_fd

#define gbep_socket   skt_context[SUBEP_SOCKET_ID]  // GB��������(�ø�����)
#define GBEP_SOCKET_ID  SUBEP_SOCKET_ID // GB����
#define GbepSocketFd   subep_socket.socket_fd

//==�������ڶ���====================================
#define HJEP_DATA_SEND_TIME     10   //��������10��
#define HJEP_HEART_BEAT_TIME    120  //��������2����

#define ZXM2M_DATA_SEND_TIME    30   //��������30��
#define ZXM2M_HEART_BEAT_TIME   120  //��������2����

#define GBEP_DATA_SEND_TIME     10   //��������10��
#define GBEP_HEART_BEAT_TIME    120  //��������2����

#define ZXM2M_HEART_BEAT_TIMEOUT_SP  (3*ZXM2M_HEART_BEAT_TIME*100)   // for 10ms time base
#define HJEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base
#define GBEP_HEART_BEAT_TIMEOUT_SP   (3*HJEP_HEART_BEAT_TIME*100)  // for 10ms time base

#define M2M_CLOUD_SERVER_IP         "58.218.196.200" // ZXM2Mƽ̨
#define M2M_CLOUD_SERVER_PORT       10004
#define M2M_CLOUD_SERVER_PROTOCOL   CS_TCP_PROTOCOL

#define HJEP_CLOUD_SERVER_IP        "120.195.166.245"  // HJ����ƽ̨
#define HJEP_CLOUD_SERVER_PORT      10012
#define HJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define GBEP_CLOUD_SERVER_IP    "120.195.166.245"  // GB�ڲ�ƽ̨
#define GBEP_CLOUD_SERVER_PORT  10012
#define GBEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

#define BJEP_CLOUD_SERVER_DNS   "jc.bjmemc.com.cn"  // ��������ƽ̨
#define BJEP_CLOUD_SERVER_PORT  7740
#define BJEP_CLOUD_SERVER_PROTOCOL  CS_TCP_PROTOCOL

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
  SYSBUS_DEVICE_TYPE_PCDEBUG,  // ���ù���
};

//extern uint8_t public_data_buffer[1460];

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
// ZxM2mƽ̨����
void ZxM2m_ServiceInit(void);
void ZxM2m_ServiceStart(void);

// ����ƽ̨����
void EP_ServiceInit(void);
void EP_ServiceStart(void);

void Net_CheckIsModemError(void);

#endif /* _ICLOUD_MACHINE_H_ */

