/*****************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: Cellura.h
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-9-25
 * @brief     ���ļ�Ϊ4Gģ��Ӳ���������ͷ�ļ�
******************************************************************************/
#ifndef _CELLURA_CORE_H_
#define _CELLURA_CORE_H_

//-----ͷ�ļ�����-------------------------------------------------------------
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
// ATָ���Ŷ���
typedef enum
{
  AT_CMD_IDLE = 0,    //!< ����ģʽ
  AT_CMD_CRESET,      //!< ģ�������λ
  AT_CMD_CFUN0_SET,   //!< ģ�����MiniFun
  AT_CMD_CFUN1_SET,   //!< ģ��Ϊȫ����(online mode)

  AT_CMD_AT,          //!< AT����
  AT_CMD_ATE0,        //!< �رջ���
  AT_CMD_CGPS_SET,    //!< ����GPS�Ự
  AT_CMD_CGPSXE_SET,  //!< ʹ��GPS XTRA
  AT_CMD_CGPSINFOCFG_SET,//!< ����NMEA-0183�������5S,���GPGGA��GPRMC��Ϣ
  AT_CMD_CGMR,        //!< ��ѯģ������汾
  AT_CMD_CMEE_SET,    //!< �����ֱ�ʾ������Ϣ
  AT_CMD_CPIN_GET,    //!< ��ѯSIM���Ƿ�READY
  AT_CMD_CIMI_GET,    //!< ��ȡSIM����IMSI
  AT_CMD_CICCID_GET,  //!< ��ȡSIM����ICCID
  AT_CMD_CMGF_SET,    //!< ���ö��Ÿ�ʽ
  AT_CMD_CPMS_SET,    //!< ���ö��Ŵ洢λ��
  AT_CMD_CNMI_SET,    //!< SMS�¼��ϱ�����:���ϱ�
  AT_CMD_CNMP_SET,    //!< ������������ģʽΪAutomatic(LTE/WCDMA/GSM)
  AT_CMD_CSQ_GET,     //!< ��ѯ�����ź�����
  AT_CMD_CREG0_SET,   //!< �ر�+CREG: URC�ϱ�
  AT_CMD_CREG2_SET,   //!< ʹ��+CREG: URC�ϱ�
  AT_CMD_CREG_GET,    //!< CSҵ��:����ע��״̬
  AT_CMD_CGREG_GET,   //!< PSҵ��:GPRS����ע��״̬
  AT_CMD_CEREG_GET,   //!< PSҵ��:LTE����ע��״̬
  AT_CMD_CGATT_SET,   //!< ʹ��PS����
  AT_CMD_CGATT_GET,   //!< ��ѯPS����
  AT_CMD_CGDCONT_SET, //!< ����PDP������
  AT_CMD_CGACT_SET,   //!< ����PDP������
  AT_CMD_CGPADDR_GET, //!< ��ѯPDP��ַ

  AT_CMD_CSCLK,       //!< ����ģ���������ģʽ
  AT_CMD_CWMAP0_SET,  //!< �ر�WIFI

  AT_CMD_CMGL,        //!< �г����ж�Ϣ
  AT_CMD_CMGD,        //!< ɾ�����ж���
  AT_CMD_CMGS,
  AT_CMD_CMGS_DATA,

  AT_CMD_END,         //!< Last CMD entry
} at_cmd_t;

// ATָ����Ӧ״̬
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

// BOOL����
enum 
{
  AT_FALSE = 0x00,
  AT_TRUE = 0x01
};

// Э�鶨��
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

// ģ��״̬����
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
  modem_state_t modem_state;      // ģ��״̬
  uint8_t imsi[MAX_SIZE_IMSI];    // SIM��IMSI
  uint8_t iccid[MAX_SIZE_ICCID];  // SIM��ICCID

  uint8_t modem_init_fail_cnt;    // ģ�鿪������
  uint8_t at_fail_cnt;            // ATָ��������
  uint8_t sim_fail_cnt;           // CPINָ��������
  uint8_t cgatt_fail_cnt;         // ���總�Ŵ������
  uint8_t pdpdeact_cnt;           // PDPȥ����֪ͨ
  
  int sim_err_flag : 1;
  int sms_ready_flag : 1;
  int modem_err_flag : 1;  // ģ����ϱ�־λ: 0=����,1=����
  
  int sim_card_ok_flag : 1; // SIM��������־λ: 1=����,0=����
  int sim_card_suspend_flag : 1; // SIM��Ƿ�ѱ�־λ: 1=Ƿ��,0=δǷ��
  
  int pdp_actived_flag : 1;
  int network_ready_flag : 1;
  
  int cme_error_flag : 1;

  uint8_t csq_rssi;       // �ź�ǿ��
  uint8_t csq_ber;

  uint8_t cs_network_regist_status; // ����ע��״̬
  uint8_t ps_network_regist_status; // ����ע��״̬
  uint8_t network_attach_status; // ���總��״̬

  uint16_t location_area_code;  // λ����: lac or tac
  uint32_t cell_id;  // С����Ϣ
}modem_info_t;
extern modem_info_t modem_info;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void Modem_SetState(modem_state_t state);
modem_state_t Modem_GetState(void);

#endif  /* _CELLURA_CORE_H_ */

