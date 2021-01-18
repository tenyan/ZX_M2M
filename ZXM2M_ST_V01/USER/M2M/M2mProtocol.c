/*****************************************************************************
* @FileName: M2mProtocol.c
* @Engineer: TenYan & ZPY
* @version   V1.0
* @Date:     2020-10-20
* @brief     M2M Э��ʵ��
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Typedef
******************************************************************************/
extern uint8_t net_public_data_buffer[1460];
extern can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];

typedef uint16_t (*im2m_BuildTlvMsgFun)(uint8_t *pbuf);
typedef uint16_t (*im2m_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  im2m_BuildTlvMsgFun pfun_build;
  im2m_AnalyzeTlvMsgFun pfun_analyze;
}im2m_CmdTlv_t;

/******************************************************************************
* Define
******************************************************************************/
#define M2M_UNUSED_ARG(x)    (void)x

///ʵ�ʷ��ͺ���
#define m2m_data_buffer        net_public_data_buffer
#define im2m_GetLinkState()    1
#define im2m_ReconnectLink()   ()
//#define im2m_SendNetData(pData, len)  ()

/******************************************************************************
* Macros
******************************************************************************/
#define M2M_DELAY(ms)    do { osDelay(ms); } while(0)
#define PART(x)     1


/******************************************************************************
* Data Types and Globals
******************************************************************************/
m2m_context_t m2m_context;

m2m_asset_data_t m2m_asset_data = {
  .header = 0x55AA5AA5, // У��ͷ
    
  .devId = {0xA0,0xC1,0x10,0x09,0,0,7},  // �ն˵�ID

  .apn = "CMNET",  // APN
  .apn_len = 5,   // APN����
  
  .user_name = "TenYan",  // M2Mƽ̨��¼�û���
  .user_name_len = 6,       // M2Mƽ̨��¼�û�������
  
  .password = {'1','2','3'},    // M2Mƽ̨��¼����
  .password_len = 3,      // M2Mƽ̨��¼���볤��
  
  .smsc_number = "15150578385", // �������ĺ���
  .smsc_number_len = 11,        // �������ĺ��볤��
  
  .main_srv_ip = {58,218,196,200},  // ������IP��ַ  
  .main_srv_port = 10004,  // �����Ķ˿�
  .main_srv_protocol = 1,  // ������ͨ��Э������: 0=UDPЭ��, 1=TCPЭ��

  .sub_srv_ip = {218,80,94,178},   // ������IP��ַ
  .sub_srv_port = 6601,   // �����Ķ˿�
  .sub_srv_protocol = 1,  // ������ͨ��Э������: 0=UDPЭ��, 1=TCPЭ��

  .hb_timer_sp = 120,    // �������(��),0=����������,Ĭ���������Ϊ30��
  .login_retry_sp = 3,   // ����¼�ظ�����
  
  .login_min_time_sp = 5,      // ��¼ʧ����С���Լ��
  .login_max_time_sp = 30,     // ��¼ʧ��������Լ��
  .sms_recv_timeout_sp = 300,  // ���Ž��ճ�ʱʱ��(��)
  
  .can_baudrate = 0,  // ����CAN���߲�����: 0=Ĭ��250K, 1=125K, 2=250K, 3=500K
  .can_format = 1,    // ����CAN���ĸ�ʽ: 0=��׼, 1=��չ, 3=���ֶ���

  // CAN ID ��������,4�ֽ�һ��
  .canId_tbl= {0x00000215,0x00000225,0x00000235,0x00000245,0x00000256,0x00000266,0x00000267,0x00000268,0x18FE01F4,
               0x18FE02F4,0x18FE03F4,0x18FE04F4,0x18FE05F4,0x18FE06F4,0x18FE07F4},
  .canId_num = 15,       // can id ����
  
  .wakeup_work_time_sp = 300,  // ���Ѻ���ʱ��(s)
  .sleep_time_sp = 120,        // ����ʱ��(��)
  .ss_report_time_sp = 60,     // �ն˻���״̬ͬ�������Զ����ͼ��(��)
  //.msin = {04,84,33,60,89,92}, // SIM����
  .can_err_dt_sp = 120,  // CAN�����ж�ʱ��
  .can_ok_dt_sp = 10,   // CAN�ָ������ж�ʱ��
  
  .power_off_dt_sp = 10,  // �ն˶ϵ�ʱ������
  .power_on_dt_sp = 1,   // �ն��ϵ�ʱ������
  
  .low_power_voltage_sp = 100,  // �ⲿ��Դ�͵�ѹ������ֵ����λ��1%
  .low_power_dt_sp = 10,       // �ⲿ��Դ�͵�ѹ������ʱ���������λ��1s
  
  .low_bat_voltage_sp = 35,  // �ڲ���Դ�͵�ѹ������ֵ����λ��1%
  .low_bat_dt_sp = 10,  // �ⲿ��Դ�͵�ѹ������ʱ�����
  
  .gps_ant_err_dt_sp = 30, // �ն����߹��ϱ�����ʱ���������λ��1s
  .gps_ant_ok_dt_sp = 30,  // �ն����߹��ϱ����Ľ��ʱ���������λ��1s
  
  .gps_module_err_dt_sp = 30,  // �ն�GPSģ����ϱ�����ʱ�����
  .gps_module_ok_dt_sp = 5,   // �ն�GPSģ����ϱ��������ʱ�����(1s)
  
  .over_speed_sp = 60,  // ��ʾ���ٱ�����ֵ����λ��1KM/H
  .over_speed_dt_sp = 20,  // ���ٱ�����ʱ���������λ��1s
  
  .towing_alarm_range_sp = 2, //�������ƶ����ϳ�������������ֵ����λ��1KM

  .work_time_report_cfg = 0x00,  // �豸����ʱ���ͳ�����ò���
  .work_data_report_mode_sp = 0, // �������������������ݵ����ϴ�ģʽ
                                 // 0x00:Ĭ��,��ʱ�����ϴ�
                                 // 0x01:����(����ĳ���������ı�Ƶ����ΪƵ�ʷ���)
                                 // 0xFF:���Ե����ϴ�ģʽ����
  .work_data_report_time_sp = 30, // ��������(����)���������ʱ�䣬��λ��1��
  .position_report_mode_sp = 0,   // λ����Ϣ�����ϴ�ģʽ
  .position_report_time_sp = 60,  // λ����Ϣ�ϴ����

  .vin_valid_flag = 0,            // VIN����Ч
  .vin = "LXGBPA123test0088",     // VIN��
  //.vin_valid_flag = 0,            // VIN����Ч
  .ecu_type = ENGINE_TYPE_WEICHAI,// ����������
  .ep_type = EP_TYPE_HJ,          // ��������
};

/******************************************************************************
*
******************************************************************************/
#if (PART("M2M�����ͽ���TLV"))
//==TLV-0x0111 ICCID=======================================================
static uint16_t im2m_BuildTlvMsg_0111(uint8_t *pbuf)
{
  uint16_t len =0;
  //uint8_t* p_iccid;

  pbuf[len++] = 0x01;  // TAG
  pbuf[len++] = 0x11;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 20;
  //p_iccid = Cellura_GetIccid();
  //memcpy(&pbuf[len], p_iccid, 20);  // VALUE
  memset(&pbuf[len], 0x31, 20);  // VALUE
  len += 20;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0111(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;

  return retVal;
}

//==�����ն�״̬λ=========================================================
uint32_t im2m_BuildDeviceStatus(void)
{
  uint32_t uiTemp = 0;

//  if (1==GPS_GetPositioningStatus()) // ��λ״̬
//  {
//    uiTemp |= BIT(31);
//  }

  // BIT29Ԥ��

  if (0 != colt_info.tobx_state) // �ն˹���״̬(0:������(��������), 1:δ����(ʡ������))
  {
    uiTemp |= BIT(28);
  }

  if (0xAA==colt_info.wdt_status) // �ն˽���״̬(0:����, 1:���ڸ澯�쳣)
  {
    uiTemp |= BIT(27);
  }

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // �����豸����״̬(0:������, 1:δ����)
  {
    uiTemp |= BIT(26);
  }
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // �����豸����״̬(0:����, 1:���ڹ����쳣)
  {
    uiTemp |= BIT(25);
  }

  if (1==COLT_GetAccStatus())  // ACC״̬(0:OFF, 1:ON)
  {
    uiTemp |= BIT(24);
  }

  if (COLT_GetMainPowerStatus()) // ����ϵ��־(0:δ�ϵ�, 1:�ϵ�)
  {
    uiTemp |= BIT(23);
  }
  if (COLT_GetMainPowerLowStatus()) // �����ѹ������ֵ��־(0:δ����, 1:����)
  {
    uiTemp |= BIT(22);
  }

  if (COLT_GetBatChargeStatus()) // ���õ�س���־(0:δ���, 1:�����)
  {
    uiTemp |= BIT(21);
  }
  if (COLT_GetBatLowStatus()) // ���õ�ص�ѹ������ֵ��־(0:δ����, 1:����)
  {
    uiTemp |= BIT(20);
  }

//  if (GPS_GetModuleStatus()) // GPSģ����ϱ�־(0:�޹���, 1:����)
//  {
//    uiTemp |= BIT(19);
//  }
//  if (GPS_GetAntOpenStatus()) // GPS���߶Ͽ���־(0:δ�Ͽ�, 1:�Ͽ�)
//  {
//    uiTemp |= BIT(18);
//  }
//  if (GPS_GetAntShortStatus()) // GPS���߶�·��־(0:δ��·, 1:��·)
//  {
//    uiTemp |= BIT(17);
//  }
  
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // CAN����ͨ���жϱ�־(0:δ�ж� 1:�ж�)
  {
    uiTemp |= BIT(16);
  }

  // BIT15-���������̵�����־
  // 14~8Ԥ��

//  if (GPS_GetSpeedOverrunStatus())  // ���ٱ�־(0:δ����, 1:�ѳ���)
//  {
//    uiTemp |= BIT(7);
//  }

  if (COLT_GetVehicleTowingStatus()) // �ϳ�����(0:δ����, 1:�ѷ���)
  {
    uiTemp |= BIT(6);
  }

  //BIT5:4-����״̬��־(00:����, 01:����, 10:����, 11:δ֪)
  // 3~1Ԥ��

  if (COLT_GetBoxOpenStatus()) // �ն˿��Ǳ�ʶ(0:δ����, 1:�ѿ���)
  {
    uiTemp |= BIT(0);
  }

  return uiTemp;
}

//==�ն˲�ƷΨһ���=======================================================
static uint16_t im2m_BuildTlvMsg_0000(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x07;
  memcpy(&pbuf[len],m2m_asset_data.devId, 7); // VALUE
  len += 7;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0000(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (len==7) // У���������
  {
    retVal = 1;
    memcpy(m2m_asset_data.devId, pValue, 7);
  }

  return retVal;
}

//==�ն��豸����汾��=====================================================
static uint16_t im2m_BuildTlvMsg_0001(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x01;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = SW_VERSION_LEN;
  memcpy(&pbuf[len], SW_VERSION, SW_VERSION_LEN); // VALUE
  len += SW_VERSION_LEN;

  return len;
}

//==M2Mƽ̨������������(APN)=============================================
static uint16_t im2m_BuildTlvMsg_0002(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x02;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.apn_len;
  memcpy(&pbuf[len],m2m_asset_data.apn, m2m_asset_data.apn_len); // VALUE
  len += m2m_asset_data.apn_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0002(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (len<=MAX_SIZE_OF_APN) // У���������
  {
    retVal = 1;
    m2m_asset_data.apn_len = len;
    memcpy(m2m_asset_data.apn, pValue, len);
  }

  return retVal;
}

//==M2Mƽ̨��¼�û���=======================================================
static uint16_t im2m_BuildTlvMsg_0003(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x03;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.user_name_len;
  memcpy(&pbuf[len], m2m_asset_data.user_name, m2m_asset_data.user_name_len); // VALUE
  len += m2m_asset_data.user_name_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0003(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if(len<=MAX_SIZE_OF_USER_NAME)
  {
    retVal = 1;
    m2m_asset_data.user_name_len = len;
    memcpy(m2m_asset_data.user_name, pValue, len);
    //memcpy(GBPara.aucVIN, m2m_asset_data.user_name, 17); // VIN��
  }
  return retVal;
}

//==M2Mƽ̨��¼����========================================================
static uint16_t im2m_BuildTlvMsg_0004(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x04;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.password_len;
  memcpy(&pbuf[len], m2m_asset_data.password, m2m_asset_data.password_len); // VALUE
  len += m2m_asset_data.password_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0004(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if(len<=MAX_SIZE_OF_PASSWORD)
  {
    retVal = 1;
    m2m_asset_data.password_len = len;
    memcpy(m2m_asset_data.password, pValue, len);
  }
  
  return retVal;
}

//==�������ĺ���===========================================================
static uint16_t im2m_BuildTlvMsg_0005(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.smsc_number_len;
  memcpy(&pbuf[len],m2m_asset_data.smsc_number, m2m_asset_data.smsc_number_len); // VALUE
  len += m2m_asset_data.smsc_number_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0005(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  
  if(len <= MAX_SIZE_OF_SMSC_NUMBER)
  {
    retVal = 1;
    m2m_asset_data.smsc_number_len = len;
    memcpy(m2m_asset_data.smsc_number, pValue, len);
  }
  
  return retVal;
}

//==������IP��ַ===========================================================
static uint16_t im2m_BuildTlvMsg_0006(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x06;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  memcpy(&pbuf[len], m2m_asset_data.main_srv_ip, 4); // VALUE
  len += 4;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0006(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (4==len) // У���������
  {
    retVal = 1;
    memcpy(m2m_asset_data.main_srv_ip, pValue, 4);
    memcpy(m2m_context.srv_ip, m2m_asset_data.main_srv_ip, 4); // ��Ҫ����
    m2m_context.new_srv_address_flag = M2M_TRUE;
  }

  return retVal;
}

//==������IP��ַ===========================================================
static uint16_t im2m_BuildTlvMsg_0007(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x07;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  memcpy(&pbuf[len], m2m_asset_data.sub_srv_ip, 4); // VALUE
  len += 4;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0007(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (4==len) // У���������
  {
    retVal = 1;
    memcpy(m2m_asset_data.sub_srv_ip, pValue, 4);
  }

  return retVal;
}

//==�����Ķ˿�=============================================================
static uint16_t im2m_BuildTlvMsg_0008(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x08;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.main_srv_port>>8) & 0xFF; // VALUE
  pbuf[len++] =  m2m_asset_data.main_srv_port & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0008(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.main_srv_port = (*pValue<<8) + *(pValue+1);
    m2m_context.srv_port = m2m_asset_data.main_srv_port;
    m2m_context.new_srv_address_flag = M2M_TRUE;
  }

  return retVal;
}

//==�����Ķ˿�=============================================================
static uint16_t im2m_BuildTlvMsg_0009(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x09;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.sub_srv_port>>8) & 0xFF; // VALUE
  pbuf[len++] =  m2m_asset_data.sub_srv_port & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0009(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.sub_srv_port = (*pValue<<8) + *(pValue+1);
  }

  return retVal;
}

//==�������(��)===========================================================
static uint16_t im2m_BuildTlvMsg_000A(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x0A;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  pbuf[len++] = (m2m_asset_data.hb_timer_sp>>24) & 0xFF; // VALUE
  pbuf[len++] = (m2m_asset_data.hb_timer_sp>>16) & 0xFF;
  pbuf[len++] = (m2m_asset_data.hb_timer_sp>>8) & 0xFF;
  pbuf[len++] =  m2m_asset_data.hb_timer_sp & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_000A(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (4==len)
  {
    retVal = 1;
    m2m_asset_data.hb_timer_sp = (*(pValue)<<24) + (*(pValue+1)<<16) + (*(pValue+2)<<8) + *(pValue+3);
  }

  return retVal;
}

//==����¼�ظ���������===================================================
static uint16_t im2m_BuildTlvMsg_000B(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x0B;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.login_retry_sp; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_000B(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.login_retry_sp = *pValue;
  }

  return retVal;
}

//==��¼ʧ�����Լ������===================================================
static uint16_t im2m_BuildTlvMsg_000C(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x00; // TAG
  pbuf[len++] = 0x0C;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x08;
  pbuf[len++] = (m2m_asset_data.login_min_time_sp>>24) & 0xFF; // VALUE
  pbuf[len++] = (m2m_asset_data.login_min_time_sp>>16) & 0xFF;
  pbuf[len++] = (m2m_asset_data.login_min_time_sp>>8) & 0xFF;
  pbuf[len++] =  m2m_asset_data.login_min_time_sp & 0xFF;
  pbuf[len++] = (m2m_asset_data.login_max_time_sp>>24) & 0xFF;
  pbuf[len++] = (m2m_asset_data.login_max_time_sp>>16) & 0xFF;
  pbuf[len++] = (m2m_asset_data.login_max_time_sp>>8) & 0xFF;
  pbuf[len++] =  m2m_asset_data.login_max_time_sp & 0xFF;
  
  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_000C(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (8==len)
  {
    retVal = 1;
    m2m_asset_data.login_min_time_sp = (*(pValue)<<24) + (*(pValue+1)<<16) + (*(pValue+2)<<8) + *(pValue+3);
    m2m_asset_data.login_max_time_sp = (*(pValue+4)<<24) + (*(pValue+5)<<16) + (*(pValue+6)<<8) + *(pValue+7);
  }

  return retVal;
}

//==����CAN���߲�����======================================================
static uint16_t im2m_BuildTlvMsg_0106(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x06;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.can_baudrate>>8) & 0xFF; // VALUE
  pbuf[len++] = m2m_asset_data.can_baudrate & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0106(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.can_baudrate= (*pValue<<8) + *(pValue+1);
  }

  return retVal;
}

//==����CAN���ĸ�ʽ========================================================
static uint16_t im2m_BuildTlvMsg_0107(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x07;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.can_format>>8) & 0xFF; // VALUE
  pbuf[len++] = m2m_asset_data.can_format & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0107(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.can_format= (*pValue<<8) + *(pValue+1);
  }

  return retVal;
}

//==CAN ID ��������========================================================
static uint16_t im2m_BuildTlvMsg_0108(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t usTemp = 0;
  uint16_t i = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x08;

  usTemp = m2m_asset_data.canId_num * 4;
  pbuf[len++] = (usTemp>>8) & 0xFF; // LENGTH
  pbuf[len++] = usTemp & 0xFF;

  for (i=0; i<m2m_asset_data.canId_num; i++) // VALUE
  {
    pbuf[len++] = (m2m_asset_data.canId_tbl[i]>>24) & 0xFF;
    pbuf[len++] = (m2m_asset_data.canId_tbl[i]>>16) & 0xFF;
    pbuf[len++] = (m2m_asset_data.canId_tbl[i]>>8)  & 0xFF;
    pbuf[len++] = m2m_asset_data.canId_tbl[i] & 0xFF;
  }

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0108(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;
  uint8_t it;
  uint16_t pos=0;
  uint32_t canId;

  if ((len<4) && (len%4 != 0))
  {
    retVal = 0;
  }

  m2m_asset_data.canId_num = len/4;
  for (it=0; it<m2m_asset_data.canId_num; it++)
  {
    canId = pValue[pos++];
    canId <<= 8;
    canId += pValue[pos++];
    canId <<= 8;
    canId += pValue[pos++];
    canId <<= 8;
    canId += pValue[pos++];
    
    m2m_asset_data.canId_tbl[it] = canId;
  }
  return retVal;
}

//==//��������ʱ��(��)=====================================================
static uint16_t im2m_BuildTlvMsg_010B(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x0B;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.wakeup_work_time_sp>>8) & 0xFF; // VALUE
  pbuf[len++] = m2m_asset_data.wakeup_work_time_sp & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_010B(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.wakeup_work_time_sp = (*pValue<<8) + *(pValue+1);
  }

  return retVal;
}

//==�����ڼ䶨ʱ���Ѽ��(����)============================================
static uint16_t im2m_BuildTlvMsg_010C(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x0C;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.sleep_time_sp>>8) & 0xFF; // VALUE
  pbuf[len++] =  m2m_asset_data.sleep_time_sp & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_010C(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.sleep_time_sp = (*pValue<<8) + *(pValue+1);
  }

  return retVal;
}

//==�ն˻���״̬ͬ�������Զ����ͼ��(��)==================================
static uint16_t im2m_BuildTlvMsg_010D(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x0D;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.ss_report_time_sp; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_010D(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.ss_report_time_sp = *pValue;
  }
  
  return retVal;
}

//==SIM����,�����λ��0===================================================
static uint16_t im2m_BuildTlvMsg_0110(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x10;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x06;
  memcpy(&pbuf[len], m2m_asset_data.msin, 6); // VALUE
  len += 6;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0110(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (6==len)
  {
    retVal = 1;
    memcpy(m2m_asset_data.msin, pValue, 6);
  }
  
  return retVal;
}

//==����������============================================================
static uint16_t im2m_BuildTlvMsg_0113(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x13;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.main_srv_dn_len;
  memcpy(&pbuf[len], m2m_asset_data.main_srv_dn, m2m_asset_data.main_srv_dn_len); // VALUE
  len += m2m_asset_data.main_srv_dn_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0113(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (len<=MAX_SIZE_OF_SRV_DN)
  {
    retVal = 1;
    m2m_asset_data.main_srv_dn_len = len;
    memcpy(m2m_asset_data.main_srv_dn, pValue, len);
  }

  return retVal;
}

//==����������============================================================
static uint16_t im2m_BuildTlvMsg_0114(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x14;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = m2m_asset_data.sub_srv_dn_len;
  memcpy(&pbuf[len], m2m_asset_data.sub_srv_dn, m2m_asset_data.sub_srv_dn_len); // VALUE
  len += m2m_asset_data.sub_srv_dn_len;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0114(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (len<=MAX_SIZE_OF_SRV_DN)
  {
    retVal = 1;
    m2m_asset_data.sub_srv_dn_len= len;
    memcpy(m2m_asset_data.sub_srv_dn, pValue, len);
  }

  return retVal;
}

//==DNS====================================================================
static uint16_t im2m_BuildTlvMsg_0115(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x15;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  memcpy(&pbuf[len], m2m_asset_data.dns, 4); // VALUE
  len += 4;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0115(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (4==len)
  {
    retVal = 1;
    memcpy(m2m_asset_data.dns, pValue, 4);
  }

  return retVal;
}

//==Ӳ���汾��============================================================
static uint16_t im2m_BuildTlvMsg_0116(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x16;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (HW_VERSION>>8)& 0xFF; // VALUE
  pbuf[len++] = HW_VERSION & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0116(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;

  return retVal;
}

//==���Դ���ѹ(0.1V)==================================================
static uint16_t im2m_BuildTlvMsg_0117(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x17;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.vraw_rated>>8)& 0xFF; // VALUE
  pbuf[len++] = m2m_asset_data.vraw_rated & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0117(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.vraw_rated = (*pValue<<8) + *(pValue+1);
  }
  return retVal;
}

//==�ն˵�ض��ѹ(0.1V)================================================
static uint16_t im2m_BuildTlvMsg_0118(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x18;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (m2m_asset_data.vbat_rated>>8) & 0xFF; // VALUE
  pbuf[len++] = m2m_asset_data.vbat_rated & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0118(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.vbat_rated = (*pValue<<8) + *(pValue+1);
  }
  
  return retVal;
}

//�����ĳ���Э������(0:UDPЭ��,1:TCPЭ��)==================================
static uint16_t im2m_BuildTlvMsg_0119(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x19;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.main_srv_protocol; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0119(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.main_srv_protocol = *pValue;
    m2m_context.srv_protocol = m2m_asset_data.main_srv_protocol;
    m2m_context.new_srv_address_flag = M2M_TRUE;
  }

  return retVal;
}

//==�����ĳ���Э������(0:UDPЭ��,1:TCPЭ��)================================
static uint16_t im2m_BuildTlvMsg_011A(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x01; // TAG
  pbuf[len++] = 0x1A;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.sub_srv_protocol; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_011A(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.sub_srv_protocol = *pValue;
  }

  return retVal;
}

//==���������ն�����ͨ���жϱ�������======================================
static uint16_t im2m_BuildTlvMsg_0201(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x01;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.can_err_dt_sp; // VALUE
  pbuf[len++] = m2m_asset_data.can_ok_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0201(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.can_err_dt_sp = *pValue;
    m2m_asset_data.can_ok_dt_sp = *(pValue+1);
  }

  return retVal;
}

//==�ն��ⲿ��Դ�ϵ籨������==============================================
static uint16_t im2m_BuildTlvMsg_0202(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x02;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.power_off_dt_sp; // VALUE
  pbuf[len++] = m2m_asset_data.power_on_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0202(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.power_off_dt_sp = *pValue;
    m2m_asset_data.power_on_dt_sp = *(pValue+1);
  }

  return retVal;
}

//==�ն��ⲿ��Դ�͵�ѹ��������============================================
static uint16_t im2m_BuildTlvMsg_0203(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x03;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.low_power_voltage_sp; // VALUE
  pbuf[len++] = m2m_asset_data.low_power_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0203(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.low_power_voltage_sp = *pValue;
    m2m_asset_data.low_power_dt_sp= *(pValue+1);
  }

  return retVal;
}

//�ն��ڲ���Դ(���)�͵�ѹ����(TLV-0x300D-0x09����)����===================
static uint16_t im2m_BuildTlvMsg_0204(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x04;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.low_bat_voltage_sp; // VALUE
  pbuf[len++] = m2m_asset_data.low_bat_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0204(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.low_bat_voltage_sp= *pValue;
    m2m_asset_data.low_bat_dt_sp= *(pValue+1);
  }

  return retVal;
}

//==�ն�GPS���߹��ϱ���(TLV-0x300D-0x06����)����==========================
static uint16_t im2m_BuildTlvMsg_0205(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.gps_ant_err_dt_sp; // VALUE
  pbuf[len++] = m2m_asset_data.gps_ant_ok_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0205(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.gps_ant_err_dt_sp = *pValue;
    m2m_asset_data.gps_ant_ok_dt_sp  = *(pValue+1);
  }

  return retVal;
}

//�ն�GPS��λģ����ϱ���(TLV-0x300D-0x05����)����========================
static uint16_t im2m_BuildTlvMsg_0206(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x06;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.gps_module_err_dt_sp; // VALUE
  pbuf[len++] = m2m_asset_data.gps_module_ok_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_0206(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.gps_module_err_dt_sp= *pValue;
    m2m_asset_data.gps_module_ok_dt_sp= *(pValue+1);
  }

  return retVal;
}

//==���ٱ���(TLV-0x300D-0x03)��������=====================================
static uint16_t im2m_BuildTlvMsg_020A(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x0A;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.over_speed_sp; // VALUE
  pbuf[len++] = m2m_asset_data.over_speed_dt_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_020A(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.over_speed_sp = *pValue;
    m2m_asset_data.over_speed_dt_sp = *(pValue+1);
  }

  return retVal;
}

//==�ϳ�����(TLV-0x300D-0x01)��������=====================================
static uint16_t im2m_BuildTlvMsg_020B(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x02; // TAG
  pbuf[len++] = 0x0B;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.towing_alarm_range_sp; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_020B(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.towing_alarm_range_sp = *pValue;
  }

  return retVal;
}

//==//����������IP========================================================
static uint16_t im2m_AnalyzeTlvMsg_1002(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (4==len)
  {
    retVal = 1;
    memcpy(rfu_context.srv_ip, pValue, 4);
  }

  return retVal;
}

//==�����������˿ں�======================================================
static uint16_t im2m_AnalyzeTlvMsg_1003(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    rfu_context.srv_port = pValue[0];
    rfu_context.srv_port <<= 8;
    rfu_context.srv_port += pValue[1];
  }

  return retVal;
}

//==TLV-1005�����̼��汾��================================================
static uint16_t im2m_BuildTlvMsg_1005(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x10; // TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = rfu_context.file_version_length;
  memcpy(&pbuf[len], rfu_context.file_version, rfu_context.file_version_length); // VALUE
  len += rfu_context.file_version_length;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_1005(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if ((len>0) && (len<DEFAULT_RFU_FILE_VERSION_SIZE))
  {
    retVal = 1;
    rfu_context.file_version_length = len;
    memcpy(rfu_context.file_version, pValue, len);
  }

  return retVal;
}

//==TLV-100C�����̼��ļ�������,������׺====================================
static uint16_t im2m_BuildTlvMsg_100C(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x10; // TAG
  pbuf[len++] = 0x0C;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = rfu_context.file_name_length;
  memcpy(&pbuf[len], rfu_context.file_name, rfu_context.file_name_length); // VALUE
  len += rfu_context.file_name_length;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_100C(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if ((len>0) && (len<DEFAULT_RFU_FILE_NAME_SIZE))
  {
    retVal = 1;
    rfu_context.file_name_length = len;
    memcpy(rfu_context.file_name, pValue, len);
  }

  return retVal;
}

//==TLV-100D-��ǰ����汾��================================================
static uint16_t im2m_BuildTlvMsg_100D(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x10; // TAG
  pbuf[len++] = 0x0D;
  pbuf[len++] = (SW_VERSION_LEN>>8) & 0xFF; // LENGTH
  pbuf[len++] = SW_VERSION_LEN & 0xFF;
  memcpy(&pbuf[len], SW_VERSION, SW_VERSION_LEN); // VALUE
  len += SW_VERSION_LEN;

  return len;
}

//����������Э������
static uint16_t im2m_AnalyzeTlvMsg_100E(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    rfu_context.srv_protocol = pValue[0];
  }

  return retVal;
}

//==�豸����ʱ���ͳ�����ò���============================================
static uint16_t im2m_BuildTlvMsg_2000(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x20; // TAG
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.work_time_report_cfg; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_2000(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    m2m_asset_data.work_time_report_cfg = *pValue;
  }

  return retVal;
}

/*************************************************************************
 * ��1�ֽڱ�ʾ��������(����)���ݵ����ϴ�ģʽ
 * ��2�ֽڵ���1�ֽ�Ϊ0x00ʱ����ʾ��������(����)�������
*************************************************************************/
static uint16_t im2m_BuildTlvMsg_2001(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x20; // TAG
  pbuf[len++] = 0x01;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.work_data_report_mode_sp; // VALUE
  pbuf[len++] = m2m_asset_data.work_data_report_time_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_2001(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.work_data_report_mode_sp = *pValue;
    m2m_asset_data.work_data_report_time_sp= *(pValue+1);
  }

  return retVal;
}

/*************************************************************************
 * ��1�ֽڱ�ʾλ����Ϣ�����ϴ�ģʽ
 * ��2�ֽڱ�ʾλ����Ϣ������
*************************************************************************/
static uint16_t im2m_BuildTlvMsg_2002(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x20; // TAG
  pbuf[len++] = 0x02;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = m2m_asset_data.position_report_mode_sp; // VALUE
  pbuf[len++] = m2m_asset_data.position_report_time_sp;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_2002(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (2==len)
  {
    retVal = 1;
    m2m_asset_data.position_report_mode_sp = *pValue;
    m2m_asset_data.position_report_time_sp = *(pValue+1);
  }

  return retVal;
}

//==�ն�ACC ON �ۼ�ʱ��===================================================
static uint16_t im2m_BuildTlvMsg_3016(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint32_t uiTemp = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x16;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;

  uiTemp = COLT_GetTotalWorkTime();
  pbuf[len++] = (uiTemp>>24) & 0xFF; // VALUE
  pbuf[len++] = (uiTemp>>16) & 0xFF;
  pbuf[len++] = (uiTemp>>8)  & 0xFF;
  pbuf[len++] =  uiTemp & 0xFF;

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_3016(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  uint32_t uiTemp = 0;

  if (4==len)
  {
    retVal = 1;
    uiTemp = (*pValue<<24)+(*(pValue+1)<<16)+(*(pValue+2)<<8)+*(pValue+3);
    COLT_SetTotalWorkTime(uiTemp);
  }

  return retVal;
}

//==�ն���������(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4000(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;
    tempVal = FLASH_OB_GetUser();
    if (0 == (0x01 & tempVal)) // �ж��ڲ����Ź��Ƿ���Ӳ��ʹ��
    {
      FLASH_OB_Unlock();
      // OB_IWDG_HW: ѡ��Ӳ���������Ź� 
      // OB_IWDG_SW: ѡ������������Ź� 
      // ���� STOP ģʽ��������λ 
      // OB_STDBY_NoRST: ���� Standby ģʽ��������λ 
      FLASH_OB_UserConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST); // ԭ����OB_IWDG_HW
      FLASH_OB_Launch();
      FLASH_OB_Lock();
    }
    CTL_SetRestartDelayTime(5); // ��ʱ��λ����
  }

  return retVal;
}

//==�ն��豸������ʼ��(RC)================================================
static uint16_t im2m_AnalyzeTlvMsg_4001(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (0==len)
  {
    retVal = 1;
    Parm_ResetM2mAssetDataToFactory(); // ���ò�������
    CTL_SetRestartDelayTime(5); // ������ʱ��λ����
  }

  return retVal;
}

//==��׼����ָ��(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4004(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;
  uint8_t cmd;
  bittype lock_cmd_flag;

  lock_cmd_flag.byte = lvc_context.lock_cmd_flag;
  cmd = *pValue;
  if (0==cmd)  // ����
  {
    lock_cmd_flag.b.bit2 = 1;
    lock_cmd_flag.b.bit3 = 0;
    lock_cmd_flag.b.bit4 = 1;
    lock_cmd_flag.b.bit5 = 0;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.unlock_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else if (1==cmd) // һ������:��������:1200rpm
  {
    lock_cmd_flag.b.bit2 = 1;
    lock_cmd_flag.b.bit3 = 1;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.lock_level1_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else if (2==cmd) // ��������:��ֹ����
  {
    lock_cmd_flag.b.bit4 = 1;
    lock_cmd_flag.b.bit5 = 1;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.lock_level2_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else
  {
    retVal = 0;
  }

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // ��������ִ�н��
  {
    retVal = 0;
    m2m_context.ss_req.send_timer = 8;
  }

  return retVal;
}

//==�ն������ػ�(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4008(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (0==len)
  {
    retVal = 1;
    CTL_SetPwroffDelayTime(5); // ������ʱ�ػ�ʱ��
  }

  return retVal;
}

//==����ģʽ����(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4FFF(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (1==len)
  {
    retVal = 1;
    PcDebug_SetStatus(*pValue);
  }

  return retVal;
}

//==ECU����================================================================
static uint16_t im2m_AnalyzeTlvMsg_A1FE(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t ucTemp = 0;

  if (1==len)
  {
    retVal = 1;

#if 0
    ucTemp = *pValue;
    PcDebug_SendData("����\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    if (ucTemp==1)
    {
      m2m_asset_data.ecu_type=1;
      PcDebug_SendData("Ϋ��\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==2)
    {
      m2m_asset_data.ecu_type=2;
      PcDebug_SendData("���\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==3)
    {
      m2m_asset_data.ecu_type=3;
      PcDebug_SendData("�ϲ�\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==4)
    {
      m2m_asset_data.ecu_type=4;
      PcDebug_SendData("����\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==5)
    {
      m2m_asset_data.ecu_type=5;
      PcDebug_SendData("�ɺ�\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else
    {
      m2m_asset_data.ecu_type=1;
      PcDebug_SendData("Ĭ��Ϋ��\r\n", 10, DBG_MSG_TYPE_ANYDATA);
    }
#endif
  }
  return retVal;
}

//==�����ն�==============================================================
static uint16_t im2m_BuildTlvMsg_A1FE(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0xA1; // TAG
  pbuf[len++] = 0xFE;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = m2m_asset_data.ecu_type; // VALUE

  return len;
}

static uint16_t im2m_AnalyzeTlvMsg_A1FF(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;

#if 0
  /*����ECU���ʹ���*/
  if (g_stuSYSParamSet.EcuType==2) //���
  {
    memcpy(pDataBuff,p+1,4);
    m_stuMcuCmd.P_uiDeviceID=pDataBuff[0]+(pDataBuff[1]<<8)+(pDataBuff[2]<<16)+(pDataBuff[3]<<24);

    m_stuMcuCmd.ucBindCmd=0xbb;
    m_stuMcuCmd.F_Step=3;
    Parm_SaveLvcInfo();
  }
  else if (g_stuSYSParamSet.EcuType==4)   //����
  {
    if (9!=usCmdBodyLen)
    {
      retVal = 0;
    }
    else if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1))
    {
      retVal = 0;
    }
    else
    {
      ucTemp =*p++;
      if (0==ucTemp)
      {
        m_stuMcuCmd.ucTempUnbindFlag = 0x55;
        memcpy(m_stuMcuCmd.aucTempKey, p, 4);
        if (0!=m_stuMcuCmd.ucBindCmd)
        {
          m_stuMcuCmd.ucBindCmd=0;
          Parm_SaveLvcInfo();
        }
      }
    }
  }
#endif
  return retVal;
}

//==type���յ���˳����д=================================================
im2m_CmdTlv_t m2m_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/
  {0x0000, im2m_BuildTlvMsg_0000, im2m_AnalyzeTlvMsg_0000},/*�豸�ն˱��*/
  {0x0001, im2m_BuildTlvMsg_0001, NULL},                   /*�ն��豸����汾��ţ�ֻ��*/
  {0x0002, im2m_BuildTlvMsg_0002, im2m_AnalyzeTlvMsg_0002},/*M2Mƽ̨�����������ƣ�APN��*/
  {0x0003, im2m_BuildTlvMsg_0003, im2m_AnalyzeTlvMsg_0003},/*M2Mƽ̨��¼�û���*/
  {0x0004, im2m_BuildTlvMsg_0004, im2m_AnalyzeTlvMsg_0004},//M2Mƽ̨��¼����
  {0x0005, im2m_BuildTlvMsg_0005, im2m_AnalyzeTlvMsg_0005},//�������ĺ���
  {0x0006, im2m_BuildTlvMsg_0006, im2m_AnalyzeTlvMsg_0006},//������IP��ַ
  {0x0007, im2m_BuildTlvMsg_0007, im2m_AnalyzeTlvMsg_0007},//������IP��ַ
  {0x0008, im2m_BuildTlvMsg_0008, im2m_AnalyzeTlvMsg_0008},//�����Ķ˿�
  {0x0009, im2m_BuildTlvMsg_0009, im2m_AnalyzeTlvMsg_0009},//�����Ķ˿�
  {0x000A, im2m_BuildTlvMsg_000A, im2m_AnalyzeTlvMsg_000A},//�����������λ����
  {0x000B, im2m_BuildTlvMsg_000B, im2m_AnalyzeTlvMsg_000B},//����¼�ظ���������
  {0x000C, im2m_BuildTlvMsg_000C, im2m_AnalyzeTlvMsg_000C},//��¼ʧ�����Լ������

  {0x0106, im2m_BuildTlvMsg_0106, im2m_AnalyzeTlvMsg_0106},//����CAN���߲�����
  {0x0107, im2m_BuildTlvMsg_0107, im2m_AnalyzeTlvMsg_0107},//����CAN���ĸ�ʽ
  {0x0108, im2m_BuildTlvMsg_0108, im2m_AnalyzeTlvMsg_0108},//CAN ID ��������
  {0x010B, im2m_BuildTlvMsg_010B, im2m_AnalyzeTlvMsg_010B},//��������ʱ�䣬��λ����
  {0x010C, im2m_BuildTlvMsg_010C, im2m_AnalyzeTlvMsg_010C},//�����ڼ䶨ʱ���Ѽ������λ����
  {0x010D, im2m_BuildTlvMsg_010D, im2m_AnalyzeTlvMsg_010D},//�ն˻���״̬ͬ�������Զ����ͼ��,��λ����
  {0x0110, im2m_BuildTlvMsg_0110, im2m_AnalyzeTlvMsg_0110},//SIM���ţ������λ��0
  {0x0111, im2m_BuildTlvMsg_0111, im2m_AnalyzeTlvMsg_0111},//ICCID�룬ֻ��
  {0x0113, im2m_BuildTlvMsg_0113, im2m_AnalyzeTlvMsg_0113},//����������
  {0x0114, im2m_BuildTlvMsg_0114, im2m_AnalyzeTlvMsg_0114},//����������
  {0x0115, im2m_BuildTlvMsg_0115, im2m_AnalyzeTlvMsg_0115},//DNS
  {0x0116, im2m_BuildTlvMsg_0116, im2m_AnalyzeTlvMsg_0116},//Ӳ���汾�ţ���V1.5��ʾΪ0x0105,ֻ��
  {0x0117, im2m_BuildTlvMsg_0117, im2m_AnalyzeTlvMsg_0117},//���Դ���ѹ,��λ��0.1V
  {0x0118, im2m_BuildTlvMsg_0118, im2m_AnalyzeTlvMsg_0118},//�ն˵�ض��ѹ,��λ��0.1V
  {0x0119, im2m_BuildTlvMsg_0119, im2m_AnalyzeTlvMsg_0119},//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��
  {0x011A, im2m_BuildTlvMsg_011A, im2m_AnalyzeTlvMsg_011A},//�����ĳ���Э�����ͣ�0��UDPЭ��,1: TCPЭ��

  {0x0201, im2m_BuildTlvMsg_0201, im2m_AnalyzeTlvMsg_0201},//���������ն�����ͨ���жϱ�������
  {0x0202, im2m_BuildTlvMsg_0202, im2m_AnalyzeTlvMsg_0202},//�ն��ⲿ��Դ�ϵ籨������
  {0x0203, im2m_BuildTlvMsg_0203, im2m_AnalyzeTlvMsg_0203},//�ն��ⲿ��Դ�͵�ѹ��������
  {0x0204, im2m_BuildTlvMsg_0204, im2m_AnalyzeTlvMsg_0204},//�ն��ڲ���Դ����أ��͵�ѹ������TLV-0x300D-0x09����������
  {0x0205, im2m_BuildTlvMsg_0205, im2m_AnalyzeTlvMsg_0205},//�ն�GPS���߹��ϱ�����TLV-0x300D-0x06����������
  {0x0206, im2m_BuildTlvMsg_0206, im2m_AnalyzeTlvMsg_0206},//�ն�GPS��λģ����ϱ�����TLV-0x300D-0x05����������
  {0x020A, im2m_BuildTlvMsg_020A, im2m_AnalyzeTlvMsg_020A},//���ٱ�����������
  {0x020B, im2m_BuildTlvMsg_020B, im2m_AnalyzeTlvMsg_020B},//�ϳ�������������

  {0x1002,                  NULL, im2m_AnalyzeTlvMsg_1002},//����������IP
  {0x1003,                  NULL, im2m_AnalyzeTlvMsg_1003},//�����������˿ں�
  {0x1005, im2m_BuildTlvMsg_1005, im2m_AnalyzeTlvMsg_1005},//�����̼��汾��
  {0x100C, im2m_BuildTlvMsg_100C, im2m_AnalyzeTlvMsg_100C},//�����̼�����
  {0x100E,                  NULL, im2m_AnalyzeTlvMsg_100E},//����������Э������

  {0x2000, im2m_BuildTlvMsg_2000, im2m_AnalyzeTlvMsg_2000},//�豸����ʱ���ͳ�����ò���
  {0x2001, im2m_BuildTlvMsg_2001, im2m_AnalyzeTlvMsg_2001},//��1�ֽڱ�ʾ�������������������ݵ����ϴ�ģʽ,��2�ֽڵ���1�ֽ�Ϊ0x00ʱ����ʾ�����������������������
  {0x2002, im2m_BuildTlvMsg_2002, im2m_AnalyzeTlvMsg_2002},//��1�ֽڱ�ʾλ����Ϣ�����ϴ�ģʽ,��2�ֽڱ�ʾλ����Ϣ������

  {0x3016, im2m_BuildTlvMsg_3016, im2m_AnalyzeTlvMsg_3016},//����Сʱ

  {0x4000,                  NULL, im2m_AnalyzeTlvMsg_4000},//�ն���������
  {0x4001,                  NULL, im2m_AnalyzeTlvMsg_4001},//�ն��豸������ʼ��
  {0x4004,                  NULL, im2m_AnalyzeTlvMsg_4004},//��׼����ָ��
  {0x4008,                  NULL, im2m_AnalyzeTlvMsg_4008},//�ն������ػ�
  {0x4FFF,                  NULL, im2m_AnalyzeTlvMsg_4FFF},//����ģʽ����

  {0xA1FE, im2m_BuildTlvMsg_A1FE, im2m_AnalyzeTlvMsg_A1FE},//���ͻ�����
  {0xA1FF,                   NULL, im2m_AnalyzeTlvMsg_A1FF},//�����ն�
};
#define NUM_OF_M2M_CMD_DEAL   (sizeof(m2m_CmdDealTbl)/sizeof(im2m_CmdTlv_t))

#endif

#if (PART("M2M��Ϣ�齨����"))
/*************************************************************************
 * ����У���
*************************************************************************/
uint8_t im2m_CalcSumCheck(uint8_t* pbuf,uint16 len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  return sum;
}

/*************************************************************************
 * ������ͷ
*************************************************************************/
uint8_t im2m_BuildMsgHead(uint8_t *pbuf, im2m_msg_type_t msgType, uint16_t msgBodyLen, uint8_t flag, uint16_t SerialNumber)
{
  uint8_t len = 0;
  uint16_t temp_val;

  pbuf[len++] = msgType; // ��������

  memcpy(&pbuf[len], m2m_asset_data.devId, 7); // ��ƷΨһ���
  len += 7;

  pbuf[len++] = flag; // ��ʶ�����ض���Ϣ:bit3-ä��������־

  pbuf[len++] = (SerialNumber>>8) & 0xFF; // ������ˮ��
  pbuf[len++] =  SerialNumber & 0xFF;

  temp_val = msgBodyLen + 1; // +1ΪУ����
  pbuf[len++] = (temp_val>>8) & 0xFF; // �����峤��+У���ֳ���
  pbuf[len++] =  temp_val & 0xFF;

  return len;
}
#endif

#if (PART("ƽ̨�·�����������(CMD_REQ)"))
/******************************************************************************
 *
 *****************************************************************************/
uint8_t im2m_GetCmdReqType(const char *pCmdType,uint16_t cmdTypeLen)
{
  if (2==cmdTypeLen)
  {
    if (0 == memcmp(pCmdType, "PW", 2))
    {
      return M2M_CMD_REQ_TYPE_PW;
    }
    else if (0 == memcmp(pCmdType, "PR", 2))
    {
      return M2M_CMD_REQ_TYPE_PR;
    }
    else if (0 == memcmp(pCmdType, "RC", 2))
    {
      return M2M_CMD_REQ_TYPE_RC;
    }
    else if (0 == memcmp(pCmdType, "AT", 2))
    {
      return M2M_CMD_REQ_TYPE_AT;
    }
  }

  return M2M_CMD_REQ_TYPE_NONE;
}

//=============================================================================
uint16_t im2m_AnalyzeParaData(uint16_t tag, uint16_t len, uint8_t *pValue)
{
  uint16_t retVal = 0;
  uint16_t it = 0;
  im2m_CmdTlv_t* p_CmdTlvDeal = NULL;

  for (it = 0; it<NUM_OF_M2M_CMD_DEAL; it++)
  {
    p_CmdTlvDeal = &m2m_CmdDealTbl[it];
    if (p_CmdTlvDeal->type != tag)
    {
      continue;
    }

    if (p_CmdTlvDeal->pfun_analyze != NULL)
    {
      retVal = p_CmdTlvDeal->pfun_analyze(pValue, len);
    }

    break;
  }

  return retVal;
}

//=============================================================================
uint16_t im2m_BuildParaData(uint8_t *pbuf, uint16_t tag)
{
  uint16_t len = 0;
  uint16_t it = 0;
  im2m_CmdTlv_t* p_CmdTlvDeal = NULL;

  for (it = 0; it<NUM_OF_M2M_CMD_DEAL; it++)
  {
    p_CmdTlvDeal = &m2m_CmdDealTbl[it];
    if (p_CmdTlvDeal->type != tag)
    {
      continue;
    }

    if (p_CmdTlvDeal->pfun_build != NULL)
    {
      len = p_CmdTlvDeal->pfun_build(pbuf);
    }
    break;
  }

  return len;
}

//==����������Ӧ��Ϣ(�趨����)=================================================
uint16_t im2m_BuildCmdRspPwMsg(m2m_context_t* pThis, uint16_t* fail_tag_tbl, uint8_t fail_tag_num)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;
  uint8_t it;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��Ӧ��ԭ������ˮ��==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==��������========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'P';
  pbuf[len++] = 'W';

  //==������Ӧ����====
  if (fail_tag_num == 0)
  {
    pbuf[len++] = 0x00; // ����
    pbuf[len++] = 0x01;
    pbuf[len++] = 0x00; // ִ�н��:0x00=�ɹ�
  }
  else
  {
    msg_len = 1+1+2*fail_tag_num; // ������Ӧ���ݳ���
    pbuf[len++] = (msg_len>>8) & 0xFF; // ����
    pbuf[len++] = (msg_len & 0xFF);
    pbuf[len++] = 0x01;         // ִ�н��:0x01=ʧ��
    pbuf[len++] = fail_tag_num; // ִ��ʧ�ܵ�TLV����
    for (it=0; it<fail_tag_num; it++) // ִ��ʧ�ܵ�TLV��TAG
    {
      pbuf[len++] = (fail_tag_tbl[it]>>8) & 0xFF;
      pbuf[len++] = (fail_tag_tbl[it] & 0xFF);
    }
  }
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==������������(�趨����)====================================================
uint16_t im2m_AnalyzeCmdReqPwMsg(m2m_context_t* pThis)
{
  uint8_t tlvNum = 0;
  uint16_t tag;
  uint16_t length;
  //uint8_t* pValue;
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t pos = 0;
  uint8_t retVal;
  uint16_t fail_tag_tbl[50];
  uint8_t fail_tag_num = 0;

  if (pThis->cmdBodyLen < 5) // ���ȴ���
  {
    return 0;
  }

  tlvNum = pbuf[pos++];
  while (tlvNum) // TLV����
  {
    if (pos >= pThis->cmdBodyLen) // �����ж�
    {
      break; // ���ȴ���,�˳�ѭ��
    }

    //==��ȡTAG==================================
    tag = pbuf[pos++];
    tag <<= 8;
    tag += pbuf[pos++];

    //==��ȡLENGTH===============================
    length = pbuf[pos++];
    length <<= 8;
    length += pbuf[pos++];

    //==��ȡVALUE================================
    retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // ����
    if (M2M_FALSE == retVal) // ����ʧ��
    {
      fail_tag_tbl[fail_tag_num] = tag;
      fail_tag_num++;
    }
    pos += length; // ָ����һ��TLV
    tlvNum--;
  }

  Parm_SaveM2mAssetData();  // ���������FLASH
  //if (1==save_gb_flag)
  //  SaveAllGBPara();

  pThis->tx_size = im2m_BuildCmdRspPwMsg(pThis, fail_tag_tbl, fail_tag_num); // ����������Ӧ��Ϣ
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PC_DEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    //im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  }

  return pThis->tx_size;
}

//===========================================================================
uint16_t im2m_BuildCmdRspPrMsg(m2m_context_t* pThis, uint8_t tlvNum, uint16_t tlvTotalLen)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf= pThis->tx_data;
  uint16_t len=0;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��Ӧ��ԭ������ˮ��==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==��������========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'P';
  pbuf[len++] = 'R';

  //==������Ӧ����====
  msg_len = tlvTotalLen+1;
  pbuf[len++] = (msg_len>>8) & 0xFF; // ������Ӧ���ݳ���
  pbuf[len++] = (msg_len & 0xFF);
  pbuf[len++] = tlvNum; // TLV����
  len += tlvTotalLen;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==������������(��ȡ����)====================================================
uint16 im2m_AnalyzeCmdReqPrMsg(m2m_context_t* pThis)
{
  uint8_t* pTlvBuf = pThis->tx_data + M2M_MSG_HEAD_LEN + 9; // TLV��Ϣ����
  uint16_t tlvTotalLen = 0; // TLV��Ϣ�ۼƳ���
  uint8_t txTlvNum = 0;
  uint8_t rxTlvNum = 0;
  uint16_t tag = 0;
  uint16_t tlvLen = 0;
  uint16_t pos = 0;

  rxTlvNum = pThis->pCmdBody[pos++]; // tlv����
  while (rxTlvNum)
  {
    tag = pThis->pCmdBody[pos++];  // ��ȡTLV��TAG
    tag <<= 8;
    tag += pThis->pCmdBody[pos++];
    tlvLen = im2m_BuildParaData(pTlvBuf, tag);
    if (tlvLen>0)
    {
      txTlvNum++;
      pTlvBuf += tlvLen;
      tlvTotalLen += tlvLen;
    }
    rxTlvNum--;
  }

  pThis->tx_size = im2m_BuildCmdRspPrMsg(pThis, txTlvNum, tlvTotalLen); // ����������Ӧ��Ϣ

  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PC_DEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    //im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  }

  return pThis->tx_size;
}

//================================================================================
uint16_t im2m_BuildCmdRspRcMsg(m2m_context_t* pThis, uint8_t result)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf= pThis->tx_data;
  uint16_t len=0;
  uint32_t temp_val = 0;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��Ӧ��ԭ������ˮ��==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==��������========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'R';
  pbuf[len++] = 'C';

  //==������Ӧ����====
  pbuf[len++] = result; // ִ�н��
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x05;
  temp_val = im2m_BuildDeviceStatus();
  pbuf[len++] = (temp_val>>24) & 0xFF; // �ն�״̬��
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==������������(ATָ��͸��)===================================================
uint16_t im2m_AnalyzeCmdReqAtMsg(m2m_context_t* pThis)
{
#if 0
  uint8 *p = pCmdBody;
  uint8 *pBuf = &aMsgSendData[MSG_HEAD_LEN];

  if (pThis->cmdBodyLen > 0)
  {
    if (0==memcmp("t6", p, 2))	//ä��
    {
      EnableBlindSimulator();
    }
    else if (0==memcmp("obd", p, 3))	//ģ��OBD����
    {
      EnableOBDSimulator();
    }
    else if (0==memcmp("sign", p, 4))	//��Ҫ����ǩ��
    {
      EncryptInfor.signable=1;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("unsign", p, 6))	//����Ҫ����ǩ��
    {
      EncryptInfor.signable=0;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("sec", p, 3))	//��Ҫ����
    {
      EncryptInfor.secretable=1;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("unsec", p, 5))	//����Ҫ����
    {
      EncryptInfor.secretable=0;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("AT+", p, 3))	//����Ҫ����
    {
      memcpy(pBuf,p,usCmdBodyLen);
      pBuf[usCmdBodyLen]='\r';
      pBuf[usCmdBodyLen+1]='\n';
      WriteGsmUartData(pBuf, usCmdBodyLen+2);
    }
  }
#endif
  return 0;
}

/*******************************************************************************
 * ����ƽ̨����������������Ϣ
 *******************************************************************************/
uint16_t im2m_AnalyzeCmdReqMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0;
  uint8_t cmd_req_type;
  uint8_t* pbuf;

  pThis->cmdTypeLen = (uint16_t)(pThis->rx_body_data[0]<<8)+pThis->rx_body_data[1]; // �������ͳ���
  pThis->pCmdType = &pThis->rx_body_data[2]; // ��������
  if ((pThis->cmdTypeLen>0) && (pThis->cmdTypeLen<=pThis->rx_bodyLen))
  {
    pbuf = pThis->pCmdType + pThis->cmdTypeLen; // �������ݳ��ȵ�ַ
    pThis->cmdBodyLen = (uint16_t)(pbuf[0]<<8)+pbuf[1]; // �������ݳ���
    pThis->pCmdBody = pbuf+2; // ��������

    cmd_req_type = im2m_GetCmdReqType((const char *)pThis->pCmdType,pThis->cmdTypeLen);
    switch (cmd_req_type)
    {
    case M2M_CMD_REQ_TYPE_PW:
      frame_len = im2m_AnalyzeCmdReqPwMsg(pThis);
      break;

    case M2M_CMD_REQ_TYPE_PR:
      frame_len = im2m_AnalyzeCmdReqPrMsg(pThis);
      break;

    case M2M_CMD_REQ_TYPE_AT:
      frame_len = im2m_AnalyzeCmdReqAtMsg(pThis);
      break;

    default:
      break;
    }
  }

  return frame_len;
}
#endif

/******************************************************************************
 * im2m_CheckFrame() verifies that the received Modbus frame is addressed to us &
 * that the CRC is correct. This function returns MB_OK if the frame verifies
 * OK, else MB_NOK is returned.
 ******************************************************************************/
uint8_t im2m_CheckRecvMsg(m2m_context_t* pThis)
{
  uint8_t received_lrc;
  uint8_t calculated_lrc;
  uint16_t msg_body_len;

  //==��С�����ж�===================================
  if (pThis->rx_size < 14)
  {
    PcDebug_SendData((uint8_t *)("ERR:011"), 7, DBG_MSG_TYPE_ANYDATA);
    PcDebug_SendData(pThis->rx_data, pThis->rx_size, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==�����ж�=======================================
  msg_body_len = pThis->rx_data[M2M_BODY_LEN_FIELD];
  msg_body_len <<= 8;
  msg_body_len += pThis->rx_data[M2M_BODY_LEN_FIELD+1];
  if (pThis->rx_size != (msg_body_len+M2M_MSG_HEAD_LEN))
  {
    PcDebug_SendData((uint8_t *)("ERR:014"), 7, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==У����ж�=====================================
  calculated_lrc = im2m_CalcSumCheck(pThis->rx_data, (pThis->rx_size-1));
  received_lrc = pThis->rx_data[pThis->rx_size - 1];
  if (calculated_lrc == received_lrc) // check if lrc's match
    return M2M_OK;
  else
  {
    PcDebug_SendData((uint8_t *)("ERR:013"), 7, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }
}

/*******************************************************************************
 *
 *******************************************************************************/
uint16_t im2m_ProcessRecvMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0; //this is the number of bytes that are to be sent

  //==��������ͷ========================================
  pThis->rx_msg_type = pThis->rx_data[M2M_MSG_TYPE_FIELD]; // ������Ϣ����
  memcpy(pThis->rx_devId, &pThis->rx_data[M2M_DEV_ID_FIELD] ,7); // �����ն�ID
  pThis->rx_flag = pThis->rx_data[M2M_FLAG_FIELD]; // ���ճ��ұ��
  pThis->rx_sn = pThis->rx_data[M2M_SN_FIELD]; // �������
  pThis->rx_sn <<= 8;
  pThis->rx_sn += pThis->rx_data[M2M_SN_FIELD+1];
  pThis->rx_bodyLen = pThis->rx_data[M2M_BODY_LEN_FIELD]; // ��Ϣ�峤��
  pThis->rx_bodyLen <<= 8;
  pThis->rx_bodyLen += pThis->rx_data[M2M_BODY_LEN_FIELD+1];
  pThis->rx_bodyLen -= 1; // ����У��͵ĳ���
  pThis->rx_body_data = pThis->rx_data + M2M_MSG_HEAD_LEN;

  //==����������========================================
  switch (pThis->rx_msg_type)
  {
  case M2M_MSG_TYPE_CMD_REQ: // ƽ̨��������������
    frame_len = im2m_AnalyzeCmdReqMsg(pThis);
    break;

  default:
    break;
  }

  return frame_len;
}

/******************************************************************************
 * call this function to process the data that be received from the usart
*******************************************************************************/
uint16_t M2M_ProcessRecvMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0;

  pThis->frame_status = im2m_CheckRecvMsg(pThis); // У��
  if (pThis->frame_status == M2M_OK)
  {
    //_m2m_disable_interrutps();
    frame_len = im2m_ProcessRecvMsg(pThis); // ����
    //_m2m_enable_interrutps();
  }

  return frame_len;
}

/******************************************************************************
 * 
*******************************************************************************/
void M2M_ProduceSendMsg(m2m_context_t* pThis)
{

}

/******************************************************************************
 * M2M�����Ե��ú���,1S����һ��,������
*******************************************************************************/
void M2M_Do1sTasks(void)
{

}

/******************************************************************************
 * ����: ��������������ʧʱ����ӵ������б�����
 * ����: type:��������
 *       flag:��������ʧ��ʶ,1=����(FAIl), 0=��ʧ(NORMAL)
*******************************************************************************/
void M2M_AddNewAlarmToList(uint8_t type, uint8_t flag)
{
  uint8_t tempVal;

  if (1==flag)
    tempVal = type | BIT(8); // ���ϲ���
  else
    tempVal = type; // ������ʧ

  if ((tempVal==m2m_context.alarm_table[type].code) && (1==m2m_context.alarm_table[type].state))
  {
    return;
  }

  m2m_context.alarm_table[type].code = tempVal;
  m2m_context.alarm_table[type].state = 0;
  m2m_context.new_alarm_flag = 1;
}

/******************************************************************************
 * 
*******************************************************************************/
uint8_t M2M_GetConnStatus(void)
{
  return m2m_context.conn_success_flag;
}

/******************************************************************************
 * 
*******************************************************************************/
void M2M_Initialize(void)
{
  m2m_context.tx_data = m2m_data_buffer;
  
  m2m_context.conn_success_flag = M2M_FALSE;
  m2m_context.new_srv_address_flag = M2M_FALSE;
  m2m_context.new_alarm_flag = M2M_FALSE;
  m2m_context.upload_sn = 0x00;
}




