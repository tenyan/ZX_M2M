/*****************************************************************************
* @FileName: M2mProtocol.c
* @Engineer: TenYan
* @version   V1.1
* @Date:     2020-12-31
* @brief     M2M Э��ʵ��
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* �ⲿ����
******************************************************************************/
extern void iZxM2m_SendConnenctMsg(m2m_context_t* pThis);  // ������������ָ��
extern void iZxM2m_SendTcMsg(m2m_context_t* pThis, uint8_t msg_type);  // ������������
extern void iZxM2m_SendBzData(m2m_context_t* pThis);  // ����äĿ��������

extern void ZxM2mBlindZone_Init(void);
extern void ZxM2mBlindZone_Service(void);

/******************************************************************************
* Typedef
******************************************************************************/
uint8_t m2m_data_buffer[1460];
extern can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];

typedef uint16_t (*im2m_BuildTlvMsgFun)(uint8_t *pbuf);
typedef uint16_t (*im2m_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  im2m_BuildTlvMsgFun pfun_build;
  im2m_AnalyzeTlvMsgFun pfun_analyze;
}im2m_CmdTlv_t;

uint8_t rfu_data_buffer[RFU_BUFFER_SIZE];

extern blind_zone_t zxm2m_blind_zone;

/******************************************************************************
* Define
******************************************************************************/
#define M2M_UNUSED_ARG(x)    (void)x

/******************************************************************************
* Macros
******************************************************************************/
#define M2M_DELAY(ms)    msleep(ms)

/******************************************************************************
* Data Types and Globals
******************************************************************************/
m2m_context_t m2m_context;
rfu_context_t rfu_context;

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
};

/******************************************************************************
*
******************************************************************************/
#if (PART("M2M�����ͽ���TLV"))
//==TLV-0x0111 ICCID=======================================================
uint16_t im2m_BuildTlvMsg_0111(uint8_t *pbuf)
{
  uint16_t len =0;
  uint8_t* p_iccid;

  pbuf[len++] = 0x01;  // TAG
  pbuf[len++] = 0x11;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 20;
  p_iccid = Cellura_GetIccid();
  memcpy(&pbuf[len], p_iccid, 20);  // VALUE
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

  if (1==GPS_GetPositioningStatus()) // ��λ״̬
  {
    uiTemp |= BIT(31);
  }

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

  if (GPS_GetModuleStatus()) // GPSģ����ϱ�־(0:�޹���, 1:����)
  {
    uiTemp |= BIT(19);
  }
  if (GPS_GetAntOpenStatus()) // GPS���߶Ͽ���־(0:δ�Ͽ�, 1:�Ͽ�)
  {
    uiTemp |= BIT(18);
  }
  if (GPS_GetAntShortStatus()) // GPS���߶�·��־(0:δ��·, 1:��·)
  {
    uiTemp |= BIT(17);
  }
  
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // CAN����ͨ���жϱ�־(0:δ�ж� 1:�ж�)
  {
    uiTemp |= BIT(16);
  }

  // BIT15-���������̵�����־
  // 14~8Ԥ��

  if (GPS_GetSpeedOverrunStatus())  // ���ٱ�־(0:δ����, 1:�ѳ���)
  {
    uiTemp |= BIT(7);
  }

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

//==TLV1-״̬λ(0x3000)====================================================
uint16_t im2m_BuildTlvMsg_3000(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint32_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  temp_val = im2m_BuildDeviceStatus();
  pbuf[len++] = (temp_val>>24) & 0xFF;  // VALUE
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;

  return len;
}

//==���λ����Ϣ===========================================================
uint8_t im2m_BuildPositionInfo(uint8_t *pbuf)
{
  uint8_t temp_val = 0;
  uint8_t len = 0;
  uint32_t lat;
  uint32_t lon;
  utc_time_t utc_time;

  //==��γ����======================================
  if (0==GPS_GetEastWest()) // ���Ȱ���
  {
    temp_val |= 0x01;
  }

  if (0==GPS_GetNorthSouth()) // γ�Ȱ���
  {
    temp_val |= 0x10;
  }
  pbuf[len++] = temp_val;

  //==����=========================================
  lat = GPS_GetLatitude();
  pbuf[len++] = (lat>>24) & 0xFF;
  pbuf[len++] = (lat>>16) & 0xFF;
  pbuf[len++] = (lat>>8) & 0xFF;
  pbuf[len++] = lat & 0xFF;

  //==����=========================================
  lon = GPS_GetLongitude();
  pbuf[len++] = (lon>>24) & 0xFF;
  pbuf[len++] = (lon>>16) & 0xFF;
  pbuf[len++] = (lon>>8) & 0xFF;
  pbuf[len++] = lon & 0xFF;

  pbuf[len++] = (GPS_GetSpeed()/10) & 0xFF; // �ٶ�
  pbuf[len++] = (GPS_GetHeading()/2) & 0xFF; // ����

  //==���θ߶�====================================
  pbuf[len++] = (GPS_GetHeight()>>8) & 0xFF; // �߶�
  pbuf[len++] =  GPS_GetHeight() & 0xFF;			 // �̶�����20+1�ֽ�

  //==���ں�ʱ��=================================
  if (GPS_GetPositioningStatus())
  {
    utc_time = GPS_GetUtcTime();  // GPS�Ѷ�λ,ʹ��GPS�ṩ��ʱ��
  }
  else
  {
    utc_time = RTC_GetUtcTime(); // GPSδ��λ,ʹ���ڲ�RTCʱ��
  }
  pbuf[len++] = utc_time.year;
  pbuf[len++] = utc_time.month;
  pbuf[len++] = utc_time.day;
  pbuf[len++] = utc_time.hour;
  pbuf[len++] = utc_time.minute;
  pbuf[len++] = utc_time.second;

  return len;
}

//==TLV2-λ����Ϣ����(0x2101)==============================================
uint16_t im2m_BuildTlvMsg_2101(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t info_len = 0;

  info_len = im2m_BuildPositionInfo(&pbuf[4]); // VALUE
  pbuf[len++] = 0x21; // TAG
  pbuf[len++] = 0x01;
  pbuf[len++] = (info_len>>8) & 0xFF; // LENGTH
  pbuf[len++] = (info_len) & 0xFF;
  len += info_len;

  return len;
}

//==TLV3-�ⲿ��Դ��ѹ(0x3004)==============================================
uint16_t im2m_BuildTlvMsg_3004(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30;  // TAG
  pbuf[len++] = 0x04;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 0x02;
  temp_val = COLT_GetMainPowerVoltage();
  pbuf[len++] = (temp_val>>8) & 0xff;  // VALUE
  pbuf[len++] = temp_val & 0xff;

  return len;
}

//==TLV4-�ն����õ�ص�ѹ(0x3005)==========================================
uint16_t im2m_BuildTlvMsg_3005(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t temp_val = 0;

  pbuf[len++] = 0x30;	// TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  temp_val = COLT_GetBatVoltage(); // ��λ10mv
  temp_val = temp_val/10;
  pbuf[len++] = 0;    // VALUE
  pbuf[len++] = temp_val & 0xff;

  return len;
}

//==TLV5-�����źų�ǿ(0x3007)==============================================
uint16_t im2m_BuildTlvMsg_3007(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x30;  // TAG
  pbuf[len++] = 0x07;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = Cellura_GetCsq(); // VALUE

  return len;
}

//==TLV6-��ǰGPS���ǿ���(0x3008)===========================================
uint16_t im2m_BuildTlvMsg_3008(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x08;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = GPS_GetSatelliteNum();  // VALUE

  return len;
}

//==TLV8-��վID(0x3006)LAC(2byet)+CELL(2Byte)==============================
static uint16_t im2m_BuildTlvMsg_3006(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint32_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x06;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x04;
  temp_val = Cellura_GetLacCellID();
  pbuf[len++] = (temp_val>>24) & 0xFF; // VALUE
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;

  return len;
}

//==TLV8- PPP(�˶Զ�Э��)״̬(0x3017)======================================
uint16_t im2m_BuildTlvMsg_3017(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x17;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;

  if (1==Cellura_GetModemStatus()) // ģ�����
  {
    temp_val = 4;
  }
  else if (1==Cellura_GetSimCardState()) // SIM������
  {
    temp_val = 6;
  }
  else
  {
    temp_val = 0;  // AT״̬
  }

  pbuf[len++] = temp_val;  // VALUE

  return len;
}

//==TLV9-GSMע��״̬(0x3018)===============================================
uint16_t im2m_BuildTlvMsg_3018(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30;  // TAG
  pbuf[len++] = 0x18;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 0x01;
  temp_val = Cellura_GetCsRegistState();
  pbuf[len++] = temp_val; // VALUE

  return len;
}

//==TLV10-GPRSע��״̬(0x3019)=============================================
uint16_t im2m_BuildTlvMsg_3019(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x19;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  temp_val = Cellura_GetPsRegistState();
  pbuf[len++] = temp_val;  // VALUE

  return len;
}

//==TLV11-��ƽ̨����״̬(0x301A)===========================================
uint16_t im2m_BuildTlvMsg_301A(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x1A;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  if (M2M_TRUE==colt_info.m2m_online_flag)
  {
    temp_val =1;
  }
  else
  {
    temp_val = 0;
  }
  pbuf[len++] = temp_val; // VALUE

  return len;
}

//==can��������(0x2103)====================================================
static uint16_t im2m_BuildTlvMsg_2103(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t it;
  uint8_t can_num = 0;
  uint16_t tlv_length = 0;

  if (((1==CAN_GetRecvState(CAN_CHANNEL1)) || (1==CAN_GetRecvState(CAN_CHANNEL2))) && (1==COLT_GetAccStatus()))
  {
    pbuf[len++] = 0x21; // TAG
    pbuf[len++] = 0x03;
    len += 3;           // ����(2b)+CAN���ݰ���(1B)
    for (it=0; it<MAX_CAN_FRAME_NUM; it++)
    {
      if (can_frame_table[it].id != 0)
      {
        pbuf[len++] = (can_frame_table[it].id>>24) & 0xFF;
        pbuf[len++] = (can_frame_table[it].id>>16) & 0xFF;
        pbuf[len++] = (can_frame_table[it].id>>8) & 0xFF;
        pbuf[len++] =  can_frame_table[it].id & 0xFF;
        can_frame_table[it].id = 0;
        memcpy(&pbuf[len], can_frame_table[it].data, 8);
        len += 8;
        can_num++;
      }
    }
    tlv_length = can_num*12 + 1;
    pbuf[2] = (tlv_length>>8) & 0xFF; // LENGTH
    pbuf[3] = tlv_length & 0xFF;
    pbuf[4] = can_num;  // CAN֡����
  }
  
  if (0 == can_num) // ��CAN����
  {
    len = 0;
  }

  return len;
}

//==����===================================================================
static uint16_t im2m_BuildTlvMsg_300D(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t alarm_num = 0;
  uint8_t it;

  pbuf[len++] = 0x30;  // TAG
  pbuf[len++] = 0x0D;
  len += 2;  // ָ��VALUE
  for (it=0; it<NUM_OF_M2M_ALARM_TYPE; it++)
  {
    if ((0==m2m_context.alarm_table[it].state) && (0!=m2m_context.alarm_table[it].code))
    {
      pbuf[len++] = m2m_context.alarm_table[it].code;
      alarm_num++;
    }
  }
  pbuf[2] = 0x00; // LENGTH
  pbuf[3] = alarm_num;

  //if(0==alarm_num) // �޿��ϱ��澯
  //{
  //  len = 0x00;
  //}

  return len;
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
uint16_t im2m_BuildTlvMsg_100D(uint8_t *pbuf)
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
uint16_t im2m_BuildTlvMsg_3016(uint8_t *pbuf)
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
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;
#if 0
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
#endif
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
  
#if 0
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
#endif

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

//==�������ܼ���/�ر�(RC)=================================================
static uint16_t im2m_AnalyzeTlvMsg_4009(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;
#if 0
  uint8_t cmd;
  bittype lock_cmd_flag;

  lock_cmd_flag.byte = lvc_context.lock_cmd_flag;
  cmd = *pValue;
  if (0==cmd) // �ر���������
  {
    lock_cmd_flag.b.bit0 = 1;
    lock_cmd_flag.b.bit1 = 0;
    lock_cmd_flag.b.bit2 = 0;
    lock_cmd_flag.b.bit3 = 0;
    lock_cmd_flag.b.bit4 = 0;
    lock_cmd_flag.b.bit5 = 0;
    lock_cmd_flag.b.bit6 = 0;
    lock_cmd_flag.b.bit7 = 0;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.bind_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else if (1==cmd) // ������������
  {
    lock_cmd_flag.b.bit0 = 1;
    lock_cmd_flag.b.bit1 = 1;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.bind_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else
  {
    retVal = 0;
  }
#endif

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // �����󶨺ͽ��ָ��
  {  retVal = 0;}

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

//==����������============================================================
static uint16_t im2m_BuildTlvMsg_A1FE(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0xA1; // TAG
  pbuf[len++] = 0xFE;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;
  pbuf[len++] = zxtcw_context.ecu_type; // VALUE

  return len;
}

//==�����ն�==============================================================
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

  {0xA1FE, im2m_BuildTlvMsg_A1FE, im2m_AnalyzeTlvMsg_A1FE},//����������
  {0xA1FF,                   NULL, im2m_AnalyzeTlvMsg_A1FF},//�����ն�

  {0xA510,                   NULL, iZxM2m_AnalyzeTlvMsg_A510}, //����ר��:������
  {0xA511,                   NULL, iZxM2m_AnalyzeTlvMsg_A511}, //����ר��:���������
  {0xA512,                   NULL, iZxM2m_AnalyzeTlvMsg_A512}, //����ר��:����Э������
  {0xA513,                   NULL, iZxM2m_AnalyzeTlvMsg_A513}, //����ר��:VIN������
  {0xA514,                   NULL, iZxM2m_AnalyzeTlvMsg_A514}, //����ר��:ǿ�ơ����Ͼ�Ԯָ��
};
#define NUM_OF_M2M_CMD_DEAL   (sizeof(m2m_CmdDealTbl)/sizeof(im2m_CmdTlv_t))

#endif

#if (PART("M2M��Ϣ�齨����"))
/*************************************************************************
 * ����У���
*************************************************************************/
uint8_t im2m_CalcSumCheck(uint8_t* pbuf,uint16_t len)
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

/*************************************************************************
 * ����SatusSync��Ϣ
*************************************************************************/
uint16_t im2m_BuildStatusSyncBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==��������=========================================
  pbuf[len++] = 0x00; // �������ͳ���
  pbuf[len++] = 0x02;
  pbuf[len++] = 'S';  // ��������
  pbuf[len++] = 'S';

  //==��������=========================================
  len++;  // �������ݳ���
  len++;
  len++;  // ״̬ͬ��TLV����

  //==��ҪTLV==========================================
  /// TLV1-״̬λ(0x3000)
  temp_val = im2m_BuildTlvMsg_3000(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// λ����Ϣ����(0x2101)
  temp_val = im2m_BuildTlvMsg_2101(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// ����Դ��ѹֵ(0x3004)
  temp_val = im2m_BuildTlvMsg_3004(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// �ն����õ�ص�ѹ(0x3005)
  temp_val = im2m_BuildTlvMsg_3005(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GSM�ź�ǿ��
  temp_val = im2m_BuildTlvMsg_3007(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GPS���ǿ���
  temp_val = im2m_BuildTlvMsg_3008(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// ACC ON�ۼ�ʱ��(0x3016)
  temp_val = im2m_BuildTlvMsg_3016(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// PPP(�˶Զ�Э��)״̬(0x3017)
  temp_val = im2m_BuildTlvMsg_3017(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GCS��ע��״̬(0x3018)
  temp_val = im2m_BuildTlvMsg_3018(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// PS��ע��״̬(0x3019)
  temp_val = im2m_BuildTlvMsg_3019(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// ��ƽ̨����״̬(0x301A)
  temp_val = im2m_BuildTlvMsg_301A(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// Cellular ID,�ն��豸���ڵ�С����ʶ
  temp_val = im2m_BuildTlvMsg_3006(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// CAN����
  temp_val = im2m_BuildTlvMsg_2103(&pbuf[len]);
  len += temp_val;
  tlv_num += ((temp_val == 0)? 0: 1);

  /// ���SS�������ݳ��ȼ�tlv����
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // �������ݳ���
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV����

  return len;
}

/*************************************************************************
 * 0x04����: �����ն��쳣�澯/����澯(DeviceAlert)��Ϣ
*************************************************************************/
uint16_t im2m_BuildDeviceAlertBody(uint8_t* pbuf)
{
  uint16_t len=0;
  uint16_t alarm_len;

  //==��������===========================================
  pbuf[len++] = 00; // �������ͳ���
  pbuf[len++] = 02;
  pbuf[len++] = 'D'; // ��������
  pbuf[len++] = 'A';

  //==��������==========================================
  len++;  // �������ݳ���
  len++;
  alarm_len = im2m_BuildTlvMsg_300D(&pbuf[len]);
  len += alarm_len;
  pbuf[4] = ((alarm_len>>8) & 0xFF);
  pbuf[5] = (alarm_len & 0xFF);

  return len;
}

/*************************************************************************
 * 0x04����: ������е�豸������(DTC)��Ϣ
*************************************************************************/
uint16_t im2m_BuildDtcMsg(uint8_t* pbuf)
{
  uint16_t len =0;
  
#if 0
  void SYS_SendDTCData()
  {
    uint8 ucTemp = 0;
    uint16 usSendLen;
  //  uint8 i;
    uint8 *p, *p2;
    uint8 *buf;
  
    f_usUpLoadCmdSn++;
    AlarmRep.usRepSn = f_usUpLoadCmdSn;
    aMsgSendData[MSG_HEAD_LEN] = 0;
    aMsgSendData[MSG_HEAD_LEN+1] = 3;
    aMsgSendData[MSG_HEAD_LEN+2] = 'D';
    aMsgSendData[MSG_HEAD_LEN+3] = 'T';
    aMsgSendData[MSG_HEAD_LEN+4] = 'C';
    //�������ݳ���MSBs
    //�������ݳ���LSB
  
    buf = &aMsgSendData[MSG_HEAD_LEN+7];
    *buf++ = 0x30;
    *buf++ = 0xEF;
    p = buf;    //���ݳ���
    buf += 2;
    p2= buf;    //DTC����
    buf += 1;
    ucTemp = GetMcuFaultCode(buf);
    if (0==ucTemp)
      return;
    *p2 = ucTemp;
    *p++ = 0;
    *p++ = ucTemp*4+1;
  
    aMsgSendData[MSG_HEAD_LEN+5] = 0;
    aMsgSendData[MSG_HEAD_LEN+6] = ucTemp*4+12;
    usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_ALERT, ucTemp*4+12, 0, f_usUpLoadCmdSn);
    usSendLen += ucTemp*4+12;
    aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
    GSM_SendGprs(aMsgSendData, usSendLen+1, 0);
  }
#endif

  return len;
}

/*************************************************************************
 * 0x04����: ������ʻ��Ϊ������(DrivingBehavior)��Ϣ
*************************************************************************/
uint16_t im2m_BuildDrivingBehaviorMsg(uint8_t* buf)
{
  uint16_t len =0;

  return len;
}

#endif

#if (PART("����M2M�̼���������"))
/*************************************************************************
 * �̼����������Ϣ
*************************************************************************/
//==��ƽ̨����Զ�̹̼�����������(�ն�->ƽ̨)================================
uint16_t im2m_SendUpdataReqUqMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��������========
  pbuf[len++] = 0x00;  // �������ͳ���
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // ����
  pbuf[len++] = 'Q';

  //==��������========
  len += 2; // ����
  pbuf[len++] = (M2M_FIRMWARE_PACKET_LEN>>8) & 0xFF;  // ÿ�����ݵĴ�С
  pbuf[len++] = M2M_FIRMWARE_PACKET_LEN & 0xFF;
  len +=im2m_BuildTlvMsg_100C(&pbuf[len]);
  len +=im2m_BuildTlvMsg_1005(&pbuf[len]);
  len +=im2m_BuildTlvMsg_100D(&pbuf[len]);
  msg_len = len - 6;
  pbuf[M2M_MSG_HEAD_LEN+4] = (msg_len >> 8) & 0xFF; // �������ݳ���
  pbuf[M2M_MSG_HEAD_LEN+5] = msg_len & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  
  return msg_len;
}

//==�ն˹̼���������(�ն�->ƽ̨)==============================================
uint16_t im2m_SendUpdateReqUlMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;
  
  //==��������========
  pbuf[len++] = 0x00;  // �������ͳ���
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // ����
  pbuf[len++] = 'L';

  //==��������========
  len += 2; // ����
  pbuf[len++] = (rfu_context.block>>8) & 0xFF; // ��������к�
  pbuf[len++] =  rfu_context.block & 0xFF;
  pbuf[len++] = (rfu_context.total_block_count>>8) & 0xFF; // �ļ��ܰ���
  pbuf[len++] =  rfu_context.total_block_count & 0xFF;
  len +=im2m_BuildTlvMsg_100C(&pbuf[len]); // TLV-100C�����̼�����
  len +=im2m_BuildTlvMsg_1005(&pbuf[len]); // TLV-1005�����̼��汾��
  msg_len = len - 6;
  pbuf[M2M_MSG_HEAD_LEN+4] = (msg_len >> 8) & 0xFF; // �������ݳ���
  pbuf[M2M_MSG_HEAD_LEN+5] = msg_len & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨

  return msg_len;
}

//==��ƽ̨�ϱ��������(�ն�->ƽ̨)==============================================
uint16_t im2m_SendUpdateReqUrMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��������========
  pbuf[len++] = 0x00;  // �������ͳ���
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // ����
  pbuf[len++] = 'R';

  //==��������========
  pbuf[len++] = pThis->update.result; // �������
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨

  return msg_len;
}

/******************************************************************************
 *
******************************************************************************/
//==����M2M������=============================================================
void im2m_ReConnectDefaultServer(void)
{
  uint8_t srv_ip_flag = 0;
  uint8_t srv_port_flag = 0;
  uint8_t srv_protocol = 0;

  if(memcmp(rfu_context.srv_ip, m2m_context.srv_ip, 4)==0) // ipһ��
  {
    srv_ip_flag = 1;
  }

  if(rfu_context.srv_port==m2m_context.srv_port) // portһ��
  {
    srv_port_flag = 1;
  }

  if(rfu_context.srv_protocol==m2m_context.srv_protocol) // protocolһ��
  {
    srv_protocol = 1;
  }
  
  if(srv_ip_flag==0 || srv_port_flag==0 || srv_protocol==0) // ��һ����ͬ����Ҫ��������
  {
    memcpy(m2m_context.srv_ip, m2m_asset_data.main_srv_ip, 4);
    m2m_context.srv_port = m2m_asset_data.main_srv_port;
    m2m_context.srv_protocol = m2m_asset_data.main_srv_protocol;
    im2m_ReconnectLink();
  }
}

//==��������������============================================================
void im2m_ConnectUpdateServer(void)
{
  uint8_t srv_ip_flag = 0;
  uint8_t srv_port_flag = 0;
  uint8_t srv_protocol = 0;

  if(memcmp(rfu_context.srv_ip, m2m_context.srv_ip, 4)==0) // ipһ��
  {
    srv_ip_flag = 1;
  }

  if(rfu_context.srv_port==m2m_context.srv_port) // portһ��
  {
    srv_port_flag = 1;
  }

  if(rfu_context.srv_protocol==m2m_context.srv_protocol) // protocolһ��
  {
    srv_protocol = 1;
  }

  if(srv_ip_flag==0 || srv_port_flag==0 || srv_protocol==0) // ��һ����ͬ����Ҫ��������
  {
    memcpy(m2m_context.srv_ip, rfu_context.srv_ip, 4);
    m2m_context.srv_port= rfu_context.srv_port;
    m2m_context.srv_protocol = rfu_context.srv_protocol;
    im2m_ReconnectLink();
  }
}

#endif

#if (PART("����M2M��������"))
/******************************************************************************
 * ����connect request��Ϣ
*******************************************************************************/
void im2m_SendConnenctMsg(m2m_context_t* pThis)
{
  uint16_t len = 0;
  uint16_t msg_body_len = 0;
  uint16_t msg_header_len = 0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN; // ����Ϣ�忪ʼ���

  /// Э����
  pbuf[len++] = 0x00; // Э��������
  pbuf[len++] = 0x04;
  memcpy(&pbuf[len], "XM2M", 4);
  len += 4;

  /// Э��汾
  pbuf[len++] = 1;

  /// TLV100D-��ǰ����汾��
  len += im2m_BuildTlvMsg_100D(&pbuf[len]);

  /// TLV-0x0111 ICCID
  len += im2m_BuildTlvMsg_0111(&pbuf[len]);

  /// ���ӱ�ʶ
  pbuf[len++] = 0; // �������޼�Ȩ��Ϣ
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  pThis->conn_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CONN_REQ, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  len = msg_header_len + msg_body_len;
  check_sum = im2m_CalcSumCheck(pbuf, len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  //return msg_len;
}

/*************************************************************************
 * �����Ͽ�����(0x09-DISCONNECT)��Ϣ
*************************************************************************/
void im2m_SendDisconnectMsg(m2m_context_t* pThis)
{
  uint16_t len =0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_DISCONNECT, 1, 0, pThis->upload_sn); // M2mUploadSN

  //==����������==========================================
  pbuf[len++] = 0x00; // �ж�����ԭ��: 0x00 �������ߣ���������

  //==����У����==========================================
  check_sum = im2m_CalcSumCheck(pbuf,len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
}

/*************************************************************************
 * ������������(PING REQUEST)��Ϣ
*************************************************************************/
void im2m_SendPingMsg(m2m_context_t* pThis)
{
  uint16_t len = 0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==��������ͷ==========================================
  pThis->upload_sn++;
  pThis->ping_req.sn = pThis->upload_sn;
  len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PING_REQ, 0, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  check_sum = im2m_CalcSumCheck(pbuf,len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  //return msg_len;
}

/*************************************************************************
 * ����SatusSync��Ϣ
*************************************************************************/
void im2m_SendStatusSyncMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==����������==========================================
  msg_body_len = im2m_BuildStatusSyncBody(&pbuf[M2M_MSG_HEAD_LEN]);

  //==��������ͷ==========================================
  pThis->upload_sn++;
  pThis->ss_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PUSH_DATA, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  //return msg_len;
}

/*************************************************************************
 * �����ն��쳣�澯��Ϣ
*************************************************************************/
void im2m_SendDeviceAlertMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==����������==========================================
  msg_body_len = im2m_BuildDeviceAlertBody(&pbuf[M2M_MSG_HEAD_LEN]);

  //==��������ͷ==========================================
  pThis->upload_sn++;
  pThis->alarm_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_ALERT, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  //return msg_len;
}

/*************************************************************************
 * ����Զ�̹̼����������Ϣ
*************************************************************************/
void im2m_SendUpdateMsg(m2m_context_t* pThis)
{
  if (M2M_UPDATE_STATE_IDLE == pThis->update.state)
  {  return;}

  switch (pThis->update.state)
  {
  case M2M_UPDATE_STATE_NOTIFIED:
  case M2M_UPDATE_STATE_CONN:
    if (SOCKET_LINK_STATE_READY==im2m_GetLinkState())
    {
      pThis->update.state = M2M_UPDATE_STATE_SEND_REQ;
    }
    break;

  case M2M_UPDATE_STATE_SEND_REQ:
    im2m_SendUpdataReqUqMsg(pThis);
    pThis->update.rsp_timeout_timer = 5;
    pThis->update.retry_cnt++;
    pThis->update.state = M2M_UPDATE_STATE_WAIT_RSP;
    break;

  case M2M_UPDATE_STATE_DOWNLOAD_REQ:
    im2m_SendUpdateReqUlMsg(pThis);
    pThis->update.rsp_timeout_timer = 10;
    pThis->update.retry_cnt++;
    pThis->update.state = M2M_UPDATE_STATE_DOWNLOAD_RSP;
    break;

  case M2M_UPDATE_STATE_REPORT_REQ:
    im2m_SendUpdateReqUrMsg(pThis);
    pThis->update.rsp_timeout_timer = 5;
    pThis->update.retry_cnt++;
    pThis->update.state = M2M_UPDATE_STATE_REPORT_RSP;
    break;

  case M2M_UPDATE_STATE_ERROR: // ����ʧ��
    break;

  case M2M_UPDATE_STATE_SUCESS: // �����ɹ�
    break;

  default:
    break;
  }
}
#endif

//==�����ͳ�ʱ����=========================================================
uint8_t im2m_ReqTimeOutManage(im2m_request_t* pThis)
{
  uint8_t timeout_flag = M2M_FALSE; // ��ʱ��־

  if (pThis->send_timer) // �������ڵ���
    pThis->send_timer--;

  if (pThis->sent_flag==M2M_TRUE) // ��Ӧ��ʱ�ж�
  {
    if (pThis->rsp_timeout_timer)
      pThis->rsp_timeout_timer--;
    else
    {
      if (pThis->retry_cnt >= pThis->retry_sp)
      {
        pThis->retry_cnt = 0;
        timeout_flag = M2M_TRUE;
      }
      pThis->sent_flag = M2M_FALSE;
    }
  }

  return timeout_flag;
}

//==�̼����������ͳ�ʱ����=================================================
void im2m_UpdateTimeOutManage(void)
{
  if (M2M_UPDATE_STATE_IDLE==m2m_context.update.state)
  {
    return;
  }

  if (m2m_context.update.rsp_timeout_timer > 0)
    m2m_context.update.rsp_timeout_timer--;
  else
  {
    switch (m2m_context.update.state)
    {
    case M2M_UPDATE_STATE_NOTIFIED:
    case M2M_UPDATE_STATE_CONN:
      m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
      im2m_ReConnectDefaultServer(); // ʧ��,����Ĭ��M2Mƽ̨
      break;

    case M2M_UPDATE_STATE_WAIT_RSP:
      if(m2m_context.update.retry_cnt > 5)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // ʧ��,����Ĭ��M2Mƽ̨
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_SEND_REQ; // ����
      }
      break;

    case M2M_UPDATE_STATE_DOWNLOAD_RSP:
      if(m2m_context.update.retry_cnt > 5)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // ʧ��,����Ĭ��M2Mƽ̨
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_DOWNLOAD_REQ; // ����
      }
      break;

     case M2M_UPDATE_STATE_REPORT_RSP:
      if(m2m_context.update.retry_cnt > 3)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // ʧ��,����Ĭ��M2Mƽ̨
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_REPORT_REQ; // ����
      }
      break;
    
    case M2M_UPDATE_STATE_ERROR: // ����ʧ��
      break;
    
    case M2M_UPDATE_STATE_SUCESS: // �����ɹ�
      break;
    
    default:
      break;
    }
  }
}

//==��������ַ�ı����������,�ú���1sִ��һ��=================================
void im2m_ConnectNewSrvManage(void)
{
  static uint8_t cnt = 10;

  if (M2M_TRUE==m2m_context.new_srv_address_flag)
  {
    if (cnt)
    {
      cnt--;
    }
    else
    {
      m2m_context.new_srv_address_flag = M2M_FALSE;
      im2m_ReconnectLink();
    }
  }
  else
  {
    cnt = 10;
  }
}

#if (PART("����M2MЭ���յ�������"))
/*******************************************************************************
 * ͨ�ñ�����Ӧ��Ϣ����
 *******************************************************************************/
uint16_t im2m_AnalyzeAckMsg(m2m_context_t* pThis)
{
  uint16_t rsp_sn;
  uint8_t rsp_msg_type;
  uint8_t result;

  //==���������===============================
  rsp_sn = pThis->rx_body_data[0];  // ����Ӧ���ĵ���ˮ��
  rsp_sn <<= 8;
  rsp_sn += pThis->rx_body_data[1];
  rsp_msg_type = pThis->rx_body_data[2];  // ����Ӧ�ı�������
  result = pThis->rx_body_data[3];  // ������

  //==��Ӧ����������ж�=======================
  if (result==0x00) // 0x00��ʾ����ɹ�
  {
    switch (rsp_msg_type)
    {
    case M2M_MSG_TYPE_PUSH_DATA:
      if (rsp_sn==pThis->ss_req.sn)
      {
        pThis->ss_req.retry_cnt = 0;
        pThis->ss_req.sent_flag = 0;
      }
      else if (rsp_sn==pThis->tc_req.sn)
      {
        pThis->tc_req.retry_cnt = 0;
        pThis->tc_req.sent_flag = 0;
      }
      else if (rsp_sn==pThis->bz_req.sn)
      {
        pThis->bz_req.retry_cnt = 0;
        pThis->bz_req.sent_flag = 0;
      }
      break;

    case M2M_MSG_TYPE_ALERT:
      if (rsp_sn==pThis->alarm_req.sn)
      {
        pThis->alarm_req.retry_cnt = 0;
        pThis->alarm_req.sent_flag = 0;
        pThis->new_alarm_flag = 0;
      }
      else if (rsp_sn==pThis->dtc_req.sn)
      {
        pThis->dtc_req.retry_cnt = 0;
        pThis->dtc_req.sent_flag = 0;
        // DTC_ClearNewFlag(&dtc_1939);
      }
      break;

    default:
      break;
    }
  }
  return 0;
}

/*******************************************************************************
 * ������Ӧ��Ϣ����
 *******************************************************************************/
uint16_t im2m_AnalyzeConnRespMsg(m2m_context_t* pThis)
{
  uint8_t result;

  result = pThis->rx_body_data[0]; // ���ӷ�����
  if (result==0x00) // 0x00=���ӳɹ�
  {
    pThis->conn_success_flag = M2M_TRUE;
    pThis->conn_req.rsp_timeout_timer = pThis->conn_req.rsp_timeout_sp;
    pThis->conn_req.retry_cnt = 0x00;
    pThis->conn_req.sent_flag = 0x00;

    //pThis->ss_req.send_timer = pThis->ss_req.send_timer_sp; // ����ss���Ͷ�ʱ��
    pThis->ss_req.send_timer = 5; // ����ss���Ͷ�ʱ��
    pThis->ss_req.retry_cnt = 0;
    pThis->ss_req.sent_flag = M2M_FALSE;
    //pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp;
    pThis->ping_req.send_timer = 10; // ����ping���Ͷ�ʱ��
    pThis->ping_req.retry_cnt = 0;
    pThis->ping_req.sent_flag = M2M_FALSE;
    colt_info.m2m_online_flag = M2M_TRUE;
  }

  return 0;
}

/*******************************************************************************
 * ������Ӧ��Ϣ����
 *******************************************************************************/
uint16_t im2m_AnalyzePingRspMsg(m2m_context_t* pThis)
{
  pThis->ping_req.retry_cnt = 0;
  pThis->ping_req.sent_flag = 0;
  pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp;

  return 0;
}

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
    else if (0 == memcmp(pCmdType, "LT", 2))
    {
      return M2M_CMD_REQ_TYPE_LT;
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
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
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
uint16_t im2m_AnalyzeCmdReqPrMsg(m2m_context_t* pThis)
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

  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
  }

  return pThis->tx_size;
}

//==����CMD��Ӧ����LT======================================================
uint16_t im2m_BuildCmdRspLtMsg(m2m_context_t* pThis)
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
  pbuf[len++] = 'L';
  pbuf[len++] = 'T';

  //==��������Ӧ����====
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==������������(λ��׷��)=====================================================
uint16_t im2m_AnalyzeCmdReqLtMsg(m2m_context_t* pThis)
{
  uint8_t pos = 0;
  uint8_t lt_mode;
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  if (pThis->rx_from != SYSBUS_DEVICE_TYPE_PCDEBUG) // ����ƽ̨�·�����׷��ģʽ
  {
    lt_mode = pThis->pCmdBody[pos++]; // ׷��ģʽ
    if (lt_mode==0xFF) // ����׷��
    {
      pThis->lt_report.mode = 0x00;
      pThis->lt_report.timer_sp = 1;
      pThis->lt_report.total_time_sp = 1;
    }
    else if (lt_mode==0x00) // ��ʱ����׷��
    {
      pThis->lt_report.mode = 0x00;
      pThis->lt_report.timer_sp = pThis->pCmdBody[pos++];
      pThis->lt_report.total_time_sp = (uint16_t)(pThis->pCmdBody[pos++]*60); // 60
    }
    else if (lt_mode==0x01) // �Ⱦ�����׷��
    {
      pThis->lt_report.mode = 0x01;
      pThis->lt_report.timer_sp = pThis->pCmdBody[pos++];
      pThis->lt_report.total_time_sp = (uint16_t)(pThis->pCmdBody[pos++]*10);
    }

    if (0x00==pThis->lt_report.mode)
    {
      pThis->ss_req.send_timer = 1;
    }
  }

  pThis->tx_size = im2m_BuildCmdRspLtMsg(pThis); // ����LT��Ӧ��Ϣ
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
    M2M_DELAY(OS_TICKS_PER_SEC/50); // ��ʱ20ms
    
    //pThis->tx_size = im2m_BuildStatusSyncMsg(pThis->tx_data);
    //==����������==========================================
    msg_body_len = im2m_BuildStatusSyncBody(&pbuf[M2M_MSG_HEAD_LEN]);
    
    //==��������ͷ==========================================
    msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PUSH_DATA, msg_body_len, 0, pThis->rx_sn);
    
    //==����У����==========================================
    msg_len = msg_body_len + msg_header_len;
    check_sum = im2m_CalcSumCheck(pbuf, msg_len);
    pbuf[msg_len++] = check_sum;
    pThis->tx_size = msg_len;

    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
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
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x05;
  pbuf[len++] = result; // ִ�н��
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

//==������������(Զ�̿���)=====================================================
uint16_t im2m_AnalyzeCmdReqRcMsg(m2m_context_t* pThis)
{
  // ÿһ��Զ�̿�������ֻЯ��һ��TLV
  uint16_t tag;
  uint16_t length;
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t pos = 0;
  uint8_t retVal;
  uint8_t result;

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
  result = retVal? 0x00 : 0x01; // 0x00=�ɹ�, 0x01=ʧ��

  pThis->tx_size = im2m_BuildCmdRspRcMsg(pThis, result); // ����RC��Ӧ��Ϣ
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // ���ù���
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // ƽ̨
    if(pThis->ss_req.sent_flag==M2M_FALSE)
    {
      pThis->ss_req.send_timer = 0x05;
    }
  }

  return pThis->tx_size;
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

    case M2M_CMD_REQ_TYPE_LT:
      frame_len = im2m_AnalyzeCmdReqLtMsg(pThis);
      break;

    case M2M_CMD_REQ_TYPE_RC:
      frame_len = im2m_AnalyzeCmdReqRcMsg(pThis);
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

#if (PART("ƽ̨�·�������Ϣ"))
/******************************************************************************
 * ��ȡ������Ϣ����
 *****************************************************************************/
uint8_t im2m_GetUpdateType(const char *pCmdType,uint16_t cmdTypeLen)
{
  if (cmdTypeLen==2)
  {
    if (0 == memcmp(pCmdType, "UN", 2))
    {
      return M2M_UPDATE_TYPE_UN;
    }
    else if (0 == memcmp(pCmdType, "UQ", 2))
    {
      return M2M_UPDATE_TYPE_UQ;
    }
    else if (0 == memcmp(pCmdType, "UL", 2))
    {
      return M2M_UPDATE_TYPE_UL;
    }
    else if (0 == memcmp(pCmdType, "UR", 2))
    {
      return M2M_UPDATE_TYPE_UR;
    }
  }

  return M2M_UPDATE_TYPE_NONE;
}

//==����UN��ӦRFU֪ͨ��Ϣ======================================================
uint16_t im2m_BuildUpdateRspUnMsg(m2m_context_t* pThis, uint8_t result)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len = 0;

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��������========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';
  pbuf[len++] = 'N';

  //==������Ӧ����====
  pbuf[len++] = result; // ִ�н��:0x00=�ɹ�
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE_ACK, msg_body_len, 0, pThis->rx_sn);

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}


//==ƽ̨�����·�Զ�̹̼�����֪ͨ(ƽ̨->�ն�)===================================
uint16_t im2m_AnalyzeUpdateReqUnMsg(m2m_context_t* pThis)
{
  uint16_t tag;
  uint16_t length;
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t pos = 0;
  uint8_t retVal;
  uint16_t tlvTypeList[] = {0x1002,0x1003,0x100E,0x100C,0x1005}; // �̶�TLV����˳���
  uint8_t tlvNum = 0;
  uint8_t it=0;
  uint8_t err_flag = 0;
  
  if ((pThis->update.state>M2M_UPDATE_STATE_IDLE) && (FSRV_GetState()>FSRV_STATE_IDLE)) // ԭ������δ���,�ܾ�����������
  {
    return 0;
  }

  pThis->cmdBodyLen = pbuf[pos++]; // �������ݳ���
  pThis->cmdBodyLen <<= 8;
  pThis->cmdBodyLen += pbuf[pos++];
  
  rfu_context.type = pbuf[pos++]; // ��������
  rfu_context.dev= pbuf[pos++];   // �����豸

  // ����TLV
  tlvNum = sizeof(tlvTypeList)/sizeof(uint16_t);
  for(it=0; it<tlvNum; it++)
  {
    //==��ȡTAG==================================
    tag = pbuf[pos++];
    tag <<= 8;
    tag += pbuf[pos++];
    
    //==��ȡLENGTH===============================
    length = pbuf[pos++];
    length <<= 8;
    length += pbuf[pos++];

    if(tlvTypeList[it] != tag)
    {
      err_flag = 1;
      break;
    }
    
    retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // ����
    if(M2M_FALSE == retVal)
    {
      err_flag = 1;
      break;
    }
    
    pos += length;
  }

  //==�жϹ̼�����================================
  if(memcmp(rfu_context.file_name, "ZXM2M_4G", 8)==0) // 4G����(���ļ���Ϊ׼)
  {
    rfu_context.dev = 0x00;
  }
  else if(memcmp(rfu_context.file_name, "ZXM2M_ST", 8)==0)  // Э����������(���ļ���Ϊ׼)
  {
    rfu_context.dev = 0x03;
  }
  else
  {
    if(rfu_context.dev==0x00)  // �̼����ʹ���
    {
      err_flag = 1;
    }
  }
  //==============================================

  pThis->tx_size = im2m_BuildUpdateRspUnMsg(pThis, err_flag); // ������Ӧ��Ϣ
  im2m_SendNetData(pThis->tx_data, pThis->tx_size);

  M2M_DELAY(OS_TICKS_PER_SEC/10); // ��ʱ100ms

  if(err_flag==0) // �޴���
  {
    im2m_ConnectUpdateServer();
    pThis->update.rsp_timeout_timer = 60;  //1����û�������Ͼ���ʧ��
    pThis->update.state = M2M_UPDATE_STATE_NOTIFIED;
  }
  else
  {
    pThis->update.state = M2M_UPDATE_STATE_IDLE;
  }
  
  return 0;
}

//==ƽ̨��Ӧ�ն˹̼���������(ƽ̨->�ն�)=======================================
uint16_t im2m_AnalyzeUpdateRspUqMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t len = 0;

  if (0 == pbuf[len++]) // 0x00=����ɹ�
  {
    // �����ļ��Ĵ�С(BYTE)
    rfu_context.file_length = pbuf[len++]; // ����ֽ�
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // �м���ֽ�
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // �м���ֽ�
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // ����ֽ�

    // CRC32У����
    rfu_context.plain_crc32val = pbuf[len++]; // ����ֽ�
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // �м���ֽ�
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // �м���ֽ�
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // ����ֽ�

    // �ļ��ܰ���
    rfu_context.total_block_count = pbuf[len++]; // ����ֽ�
    rfu_context.total_block_count <<= 8;
    rfu_context.total_block_count += pbuf[len++]; // ����ֽ�
    rfu_context.block = 0;

    rfu_EraseFlashHexFile(&rfu_context);

    pThis->update.retry_cnt = 0;
    pThis->update.rsp_timeout_timer = 5;
    pThis->update.state = M2M_UPDATE_STATE_DOWNLOAD_REQ;
  }

  return 0;
}

//==ƽ̨��Ӧ�ն���������������(ƽ̨->�ն�)������=============================
uint16_t im2m_AnalyzeUpdateRspUlMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t len=0;
  uint16_t current_block_index;  // ��ǰ����
  uint16_t total_block_num;  // �ܰ�����
  uint16_t packet_len;
  uint8_t rfu_status;

  pThis->cmdBodyLen = pbuf[len++]; // �������ݳ���
  pThis->cmdBodyLen <<= 8;
  pThis->cmdBodyLen += pbuf[len++];
  if (pThis->cmdBodyLen < 4) // �������ݳ����ж�
  {
    return 0;
  }

  //==��ǰ�����к�========
  current_block_index = pbuf[len++];
  current_block_index <<= 8;
  current_block_index += pbuf[len++];
  if(current_block_index != rfu_context.block)
  {
    return 0;
  }

  //==�ļ��ܰ���==========
  total_block_num = pbuf[len++];
  total_block_num <<= 8;
  total_block_num += pbuf[len++];
  if(total_block_num != rfu_context.total_block_count)
  {
    return 0;
  }

  //==�����̼����ݳ���====
  packet_len = pbuf[len++];
  packet_len <<= 8;
  packet_len += pbuf[len++];
  if (packet_len == 0)
  {
    return 0;
  }

  rfu_SaveFlashHexFile(&rfu_context, &pbuf[len], packet_len);  // ��BIN�ļ�д��SPI FLASH
  rfu_context.block++;
  rfu_context.percent = rfu_context.block * 100L / rfu_context.total_block_count;
  if (current_block_index==(total_block_num-1)) // �������(���һ��)
  {
    rfu_status = rfu_CheckNewFirmware(&rfu_context, rfu_data_buffer, RFU_BUFFER_SIZE);
    if (RFU_OK==rfu_status)
    {
      pThis->update.result = 0;
      // �̼������ж�
      if(rfu_context.dev==0x00)  //==�ն�
      {
        PcDebug_SendString("RfuCrc:4G-Ok!\n");
      }
      else
      {
        if(rfu_context.dev==0x01)  //==������
        {
          PcDebug_SendString("RfuCrc:CTL-Ok!\n");
        }
        else if(rfu_context.dev==0x02)  //==��ʾ��
        {
          PcDebug_SendString("RfuCrc:LCD-Ok!\n");
        }
        else if(rfu_context.dev==0x03)  //==Э������
        {
          PcDebug_SendString("RfuCrc:ST-Ok!\n");
        }
        
        FSRV_SetState(FSRV_STATE_START);  // �����ⲿ��������
      }
    }
    else
    {
      pThis->update.result = 1;
      PcDebug_SendString("RfuCrc:Err!\n");
    }

    pThis->update.state = M2M_UPDATE_STATE_REPORT_REQ;
    rfu_context.cumulated_address = 0;
    rfu_context.block = 0;
    return 0;
  }

  pThis->update.retry_cnt = 0;
  pThis->update.rsp_timeout_timer = 5;
  pThis->update.state = M2M_UPDATE_STATE_DOWNLOAD_REQ;

  return 0;
}

//==ƽ̨��Ӧ�ն��ϱ��������(ƽ̨->�ն�)==============================================
uint16_t im2m_AnalyzeUpdateRspUrMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;

  if ((0==pbuf[0]))  // ƽ̨�յ��ն��ϱ����������
  {
    pThis->update.state = M2M_UPDATE_STATE_IDLE;
    
    if(0==pThis->update.result)  // ����BIN�ļ��ɹ�
    {
      if(rfu_context.dev==0x00)  // �ն˳���
      {
        // Tbox_SetMachineState(TBOX_STATE_IAP); // T-BOX��������ģʽ
      }
    }
  }

  return 0;
}

/*******************************************************************************
 * ����֪ͨ��������Ӧ��Ϣ����
 *******************************************************************************/
uint16_t im2m_AnalyzeUpdateMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0;
  uint8_t update_msg_type;

  pThis->cmdTypeLen = (uint16_t)(pThis->rx_body_data[0]<<8)+pThis->rx_body_data[1]; // �������ͳ���
  pThis->pCmdType = &pThis->rx_body_data[2]; // ��������
  if ((pThis->cmdTypeLen>0) && (pThis->cmdTypeLen<=pThis->rx_bodyLen))
  {
    pThis->pCmdBody = pThis->pCmdType + pThis->cmdTypeLen; // �������ݵ�ַ
    update_msg_type = im2m_GetUpdateType((const char *)pThis->pCmdType, pThis->cmdTypeLen);
    switch (update_msg_type)
    {
    case M2M_UPDATE_TYPE_UN:
      frame_len = im2m_AnalyzeUpdateReqUnMsg(pThis); // ƽ̨�����·�
      break;

    case M2M_UPDATE_TYPE_UQ:
      frame_len = im2m_AnalyzeUpdateRspUqMsg(pThis); // ƽ̨������Ӧ
      break;

    case M2M_UPDATE_TYPE_UL:
      frame_len = im2m_AnalyzeUpdateRspUlMsg(pThis); // ƽ̨������Ӧ
      break;

    case M2M_UPDATE_TYPE_UR:
      frame_len = im2m_AnalyzeUpdateRspUrMsg(pThis); // ƽ̨������Ӧ
      break;

    default:
      break;
    }
  }

  return frame_len;
}
#endif

#endif

// 05 A0C11009000007 00 0163 000C 000252430005A5130001004B
// 05 A0C11009000007 00 0164 0010 000252430009A5100005023132333421
// 05 A0C11009000007 00 0165 0010 000252430009A510000502313131311C
// 05 A0C11009000007 00 0166 0010 000252430009A5100005023132333423
// 05 A0C11009000007 00 0167 0010 000252430009A5100005023132333424
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
    PcDebug_SendData((uint8_t *)("ERR:011\n"), 7, DBG_MSG_TYPE_ANYDATA);
    PcDebug_SendData(pThis->rx_data, pThis->rx_size, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==�����ж�=======================================
  msg_body_len = pThis->rx_data[M2M_BODY_LEN_FIELD];
  msg_body_len <<= 8;
  msg_body_len += pThis->rx_data[M2M_BODY_LEN_FIELD+1];
  if (pThis->rx_size != (msg_body_len+M2M_MSG_HEAD_LEN))
  {
    PcDebug_SendData((uint8_t *)("ERR:014\n"), 7, DBG_MSG_TYPE_ANYDATA);
    PcDebug_Printf("rx_size=%d,body_len=%d\n", pThis->rx_size, msg_body_len);
    PcDebug_SendData(pThis->rx_data, pThis->rx_size, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==У����ж�=====================================
  calculated_lrc = im2m_CalcSumCheck(pThis->rx_data, (pThis->rx_size-1));
  received_lrc = pThis->rx_data[pThis->rx_size - 1];
  if (calculated_lrc == received_lrc) // check if lrc's match
    return M2M_OK;
  else
  {
    PcDebug_SendData((uint8_t *)("ERR:013\n"), 7, DBG_MSG_TYPE_ANYDATA);
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
  case M2M_MSG_TYPE_MESSAGEACK: // ͨ�ñ���Ӧ��
    frame_len = im2m_AnalyzeAckMsg(pThis);
    break;

  case M2M_MSG_TYPE_CONN_RESP: // ƽ̨��Ӧ����
    frame_len = im2m_AnalyzeConnRespMsg(pThis);
    break;

  case M2M_MSG_TYPE_CMD_REQ: // ƽ̨��������������
    frame_len = im2m_AnalyzeCmdReqMsg(pThis);
    break;

  case M2M_MSG_TYPE_PING_RESP: // ƽ̨��Ӧ����
    frame_len = im2m_AnalyzePingRspMsg(pThis);
    break;

  case M2M_MSG_TYPE_UPDATE: // ƽ̨����������֪ͨ
  case M2M_MSG_TYPE_UPDATE_ACK: // ƽ̨������������Ӧ
    frame_len = im2m_AnalyzeUpdateMsg(pThis);
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
    //Usart3DmaSendPacket(pThis->data,pThis->tx_size);
  }

  return frame_len;
}

/******************************************************************************
 * 10ms���ڵ���
*******************************************************************************/
void M2M_ProduceSendMsg(m2m_context_t* pThis)
{
  //int32_t retVal;
  //static uint8_t report_blind_zone_timer = 50;
  uint8_t acc_state;
  static uint32_t acc_on_timer_10ms = 0x00;
  static uint32_t acc_off_timer_10ms = 0x00;
  static uint8_t send_tcb_flag = 1; // ���Ͱ汾��Ϣ��־
  static uint8_t send_tct_flag = 0; // ����ͳ����Ϣ��־

  acc_state = COLT_GetAccStatus();
  //==ACC״̬��ʱ=============================================================
  if (acc_state) // ACC��
  {
    acc_on_timer_10ms++;
    acc_off_timer_10ms = 0x00;
    send_tct_flag = 1;  // ��Ҫ����ͳ����Ϣ
  }
  else
  {
    acc_off_timer_10ms++;
    acc_on_timer_10ms = 0x00;
    send_tcb_flag = 1;  // ��Ҫ���Ͱ汾��Ϣ
  }

  //==ƽ̨δ����======
  if (im2m_GetLinkState() == SOCKET_LINK_STATE_CLOSED)
  {
    pThis->conn_success_flag = M2M_FALSE;
    pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp; // ������������ʱ��
    if (M2M_TRUE==colt_info.m2m_online_flag)
    {  colt_info.m2m_online_flag = M2M_FALSE;}
  }

  // ACC��
  if (acc_state==1)
  {
    //ZxM2mBlindZone_Service();  // ��¼ä������
  }

  if (im2m_GetLinkState() != SOCKET_LINK_STATE_READY)
  {
    //report_blind_zone_timer = 50;
    return;
  }

  //==Զ������=================================================================
  if (pThis->update.state > M2M_UPDATE_STATE_IDLE)
  {
    im2m_SendUpdateMsg(pThis);
    return;
  }

  //==����ƽ̨=================================================================
  if (pThis->conn_success_flag==M2M_FALSE)
  {
    if (pThis->conn_req.sent_flag==M2M_FALSE)
    {
      if (pThis->conn_req.send_timer==0x00)
      {
        pThis->conn_req.send_timer = pThis->conn_req.send_timer_sp;
        pThis->conn_req.sent_flag = M2M_TRUE;
        pThis->conn_req.retry_cnt++;
        //im2m_SendConnenctMsg(pThis);
        iZxM2m_SendConnenctMsg(pThis);  // ������������ָ��
        
        pThis->ss_req.retry_cnt = 0;
        pThis->ss_req.sent_flag = M2M_FALSE;
      }
    }
    return; // δ������������ǰ,���ɷ���������������
  }

  //==״̬ͬ��================================================================
  if ((pThis->ss_req.send_timer==0x00) && (pThis->ss_req.sent_flag==M2M_FALSE))
  {
    pThis->ss_req.retry_cnt++;
    pThis->ss_req.sent_flag = M2M_TRUE;
    pThis->ss_req.rsp_timeout_timer = pThis->ss_req.rsp_timeout_sp;
    pThis->ss_req.send_timer = m2m_asset_data.ss_report_time_sp;
    //im2m_SendStatusSyncMsg(pThis);
    iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCS);  // ���������ն�����
    return;
  }

  //==��������================================================================
  if ((pThis->tc_req.send_timer==0x00) && (pThis->tc_req.sent_flag==M2M_FALSE))
  {
    pThis->tc_req.retry_cnt++;
    pThis->tc_req.sent_flag = M2M_TRUE;
    pThis->tc_req.rsp_timeout_timer = pThis->tc_req.rsp_timeout_sp;
    pThis->tc_req.send_timer = m2m_asset_data.hb_timer_sp; // ʹ����������ֵ��Ϊ����ʱ��
    pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp; // ������������ʱ��

    if((send_tcb_flag==1) && (acc_on_timer_10ms>18000))  // �������Ͱ汾����(AccOn,40s��һ��)
    {
      send_tcb_flag = 0;
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCB);
    }
    else if((send_tct_flag==1) && (acc_off_timer_10ms>4000))  // ��������ͳ������(AccOff,30s��һ��)
    {
      send_tct_flag = 0;
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCT);
    }
    else  // �������͹�������(AccOn ÿ10s��һ��)
    {
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCW);
    }
    return;
  }

  //iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCD);  // �������͹�������(�й�����������)

  //==����ָ��================================================================
  if (pThis->ping_req.send_timer_sp > 0)
  {
    if ((pThis->ping_req.send_timer==0x00) && (pThis->ping_req.sent_flag==M2M_FALSE))
    {
      pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp;
      pThis->ping_req.sent_flag = M2M_TRUE;
      pThis->ping_req.retry_cnt++;
      pThis->ping_req.rsp_timeout_timer = pThis->ping_req.rsp_timeout_sp;
      im2m_SendPingMsg(pThis);
      return;
    }
  }

#if 0
  //==ä������==============================================================
  if ((pThis->bz_req.send_timer==0x00) && (pThis->bz_req.sent_flag==M2M_FALSE))
  {
    if(BlindZone_GetStackSize(&zxm2m_blind_zone)>0)
    {
      pThis->bz_req.retry_cnt++;
      pThis->bz_req.sent_flag = M2M_TRUE;
      pThis->bz_req.rsp_timeout_timer = pThis->bz_req.rsp_timeout_sp;
      pThis->bz_req.send_timer = pThis->bz_req.send_timer_sp;
      iZxM2m_SendBzData(pThis);
      return;
    }
  }
#endif

#if 0
  //��ƽ̨���ͱ�������
  if ((AlarmRep.ucNewAlarmFlag) && (0==AlarmRep.ucRepFlag))
  {
    SYS_SendAlarmData();
    AlarmRep.ucRepFlag = 1;
    AlarmRep.ucRepeats++;
    AlarmRep.ucRespondTimer = ALARM_RESPOND_TIMEOUT;
    return;
  }
  
  if ((1==IsNewMcuFaultCode()) && (0==DTCRep.ucRepFlag))
  {
    SYS_SendDTCData();
    DTCRep.ucRepFlag = 1;
    DTCRep.ucRepeats++;
    DTCRep.ucRespondTimer = DTC_RESPOND_TIMEOUT;
    return;
  }
#endif

}

/******************************************************************************
 * M2M�����Ե��ú���,1S����һ��,������
*******************************************************************************/
void M2M_Do1sTasks(void)
{
  uint8_t timeout_flag; // ��ʱ��־

  //==���ӹ���====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.conn_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M CONN FAILD!\n");
    im2m_ReconnectLink();
  }

  //==״̬ͬ������================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.ss_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M SS REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==�������ݹ���================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.tc_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M TC REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==ä����������================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.bz_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M BZ REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==��������====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.ping_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M PING REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==�澯����====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.alarm_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M ALARM REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==���������==================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.dtc_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M DCT REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==Զ�̹̼���������===========================
  im2m_UpdateTimeOutManage();

  //==�����·���������==========================
  im2m_ConnectNewSrvManage();
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
void M2M_Initialize(void)
{
  ZxM2mBlindZone_Init();

  m2m_context.tx_data = m2m_data_buffer;
  
  m2m_context.conn_success_flag = M2M_FALSE;
  m2m_context.new_srv_address_flag = M2M_FALSE;
  m2m_context.new_alarm_flag = M2M_FALSE;
  m2m_context.upload_sn = 0x00;

  // ��������
  m2m_context.conn_req.sent_flag = M2M_FALSE;
  m2m_context.conn_req.send_timer_sp = DEFAULT_CONN_DATA_SEND_PERIOD_SP;
  m2m_context.conn_req.rsp_timeout_sp = DEFAULT_CONN_DATA_RSP_TIMEOUT_SP;
  m2m_context.conn_req.retry_sp = DEFAULT_CONN_DATA_RETRY_SP;

  // ״̬ͬ������
  m2m_context.ss_req.sent_flag = M2M_FALSE;
  m2m_context.ss_req.send_timer_sp = DEFAULT_SS_DATA_SEND_PERIOD_SP;
  m2m_context.ss_req.rsp_timeout_sp = DEFAULT_SS_DATA_RSP_TIMEOUT_SP;
  m2m_context.ss_req.retry_sp = DEFAULT_SS_DATA_RETRY_SP;

  // ������������
  m2m_context.tc_req.sent_flag = M2M_FALSE;
  m2m_context.tc_req.send_timer_sp = DEFAULT_TC_DATA_SEND_PERIOD_SP;
  m2m_context.tc_req.rsp_timeout_sp = DEFAULT_TC_DATA_RSP_TIMEOUT_SP;
  m2m_context.tc_req.retry_sp = DEFAULT_TC_DATA_RETRY_SP;

  // ä����������
  m2m_context.bz_req.sent_flag = M2M_FALSE;
  m2m_context.bz_req.send_timer_sp = DEFAULT_BZ_DATA_SEND_PERIOD_SP;
  m2m_context.bz_req.rsp_timeout_sp = DEFAULT_BZ_DATA_RSP_TIMEOUT_SP;
  m2m_context.bz_req.retry_sp = DEFAULT_BZ_DATA_RETRY_SP;

  // ��������
  m2m_context.ping_req.sent_flag = M2M_FALSE;
  //m2m_context.ping_req.send_timer_sp = DEFAULT_PING_DATA_SEND_PERIOD_SP;
  m2m_context.ping_req.send_timer_sp = m2m_asset_data.hb_timer_sp;
  m2m_context.ping_req.rsp_timeout_sp = DEFAULT_PING_DATA_RSP_TIMEOUT_SP;
  m2m_context.ping_req.retry_sp = DEFAULT_PING_DATA_RETRY_SP;

  // �澯����
  m2m_context.alarm_req.sent_flag = M2M_FALSE;
  m2m_context.alarm_req.send_timer_sp = DEFAULT_ALARM_DATA_SEND_PERIOD_SP;
  m2m_context.alarm_req.rsp_timeout_sp = DEFAULT_ALARM_DATA_RSP_TIMEOUT_SP;
  m2m_context.alarm_req.retry_sp = DEFAULT_ALARM_DATA_RETRY_SP;

  // ����������
  m2m_context.dtc_req.sent_flag = M2M_FALSE;
  m2m_context.dtc_req.send_timer_sp = DEFAULT_DTC_DATA_SEND_PERIOD_SP;
  m2m_context.dtc_req.rsp_timeout_sp = DEFAULT_DTC_DATA_RSP_TIMEOUT_SP;
  m2m_context.dtc_req.retry_sp = DEFAULT_DTC_DATA_RETRY_SP;

  // Զ������
  m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
}

/******************************************************************************
 * 
*******************************************************************************/
uint8_t M2M_GetConnStatus(void)
{
  return m2m_context.conn_success_flag;
}

uint8_t M2M_GetRfuStatus(void)
{
  return m2m_context.update.state;
}


