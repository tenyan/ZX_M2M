/*****************************************************************************
* @FileName: GB17691.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2021-1-25
* @brief:     �й����ұ�׼GB17691-2018(���Ͳ��ͳ�Զ���ŷż�ؼ����淶)ʵ��.C�ļ�
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
#define GBEP_DEBUG     1  // 1-ʹ��, 0-��ֹ
#define GBEP_BZ_DEBUG    0  // 1-ʹ��, 0-��ֹ

// ����ӿڶ���
#define gbep_send_buffer              gbep_data_buffer
#define GBEP_SendNetData(pData, len)  NetSocket_Send(&gbep_socket, pData, len)
#define GBEP_GetLinkState()           NetSocket_GetLinkState(&gbep_socket)
#define GBEP_DisableLink()            NetSocket_Disable(&gbep_socket)
#define GBEP_EnableLink()             NetSocket_Enable(&gbep_socket)

// GB���ݷ������ڶ���
#define GBEP_SEND_ENG_DATA_TIME_SP        90   //��������10��
#define GBEP_SEND_OBD_DATA_TIME_SP        190   //��������20��

#define GBEP_SEND_HB_TIME_SP          1200  //��������3����
#define GBEP_SEND_BZ_DATA_TIME_SP     14    //��������1.5��

// ä������
#define GBEP_BZ_SAVE_PERIOD_SP  299 // 30s
#define GBEP_BLIND_ZONE_PACKET_SIZE  512
#define GBEP_BDZE_WRITE_ERROR_SP  6
#define GBEP_BDZE_READ_ERROR_SP   6

/******************************************************************************
* Data Types and Globals
******************************************************************************/
static uint8 gbep_data_buffer[1024];

// ˫����(ʱ��ͷ���������)
static uint8_t gbep_eng_packet_buffer[GBEP_ENG_PACKET_MAX_NUM][SIZE_OF_GBEP_ENG]; // ���������ݻ���
static uint8_t gbep_eng_data[GBEP_ENG_PACKET_MAX_NUM][SIZE_OF_GBEP_ENG];
static uint8_t gbep_eng_time_buffer[SIZE_OF_GBEP_ENG_TIME]; // ���ݲɼ�ʱ��
static uint8_t gbep_eng_time[SIZE_OF_GBEP_ENG_TIME];
static uint8_t gbep_eng_data_valid_flag = 0;

// GB״̬
volatile bittype gbep_flags1;
gbep_state_t gbep_state = GBEP_STATE_SEND_INIT;

// ��ˮ��
uint16_t gbep_login_sn;
uint16_t gbep_send_data_sn;

// ä��������������
blind_zone_t gbep_blind_zone;
blind_zone_para_t gbep_blind_zone_para;

// ����
static uint8_t gbep_blind_zone_buffer[GBEP_BLIND_ZONE_PACKET_SIZE];
static uint16_t gbep_blind_zone_length;

void GbepBlindZone_Save(void);

/******************************************************************************
 * ��ȡʱ��,���ڻ����ϴ�
 ******************************************************************************/
rtc_date_t GBEP_GetDataTime(void)
{
  rtc_date_t bj_time,utc_time;

  if (GPS_GetPositioningStatus()==0) // �ն˲���λ �����ⲿRTCʱ��
  {
    bj_time = RTC_GetBjTime(); // ��ȡRTCʱ��
  }
  else
  {
    utc_time = GPS_GetUtcTime();
    RTC_CovertUtcToBjt(&utc_time, &bj_time);
  }

  if (bj_time.year<20 || bj_time.year>100 || bj_time.month>12 || bj_time.day>31 || bj_time.hour>23 || bj_time.minute>59 || bj_time.second>59)
  {
    bj_time.year = 20;
    bj_time.month = 11;
    bj_time.day = 18;
    bj_time.hour = 17;
    bj_time.minute  = 20;
    bj_time.second = 0;
  }

  return bj_time;
}

/*************************************************************************
 * �������У��ֵ
*************************************************************************/
uint8_t GBEP_CalcXorCheck(uint8_t *p, uint16_t len)
{
  uint8_t sum = 0;
  uint16_t i;

  for (i = 0; i < len; i++)
  {
    sum ^= p[i];
  }
  return sum;
}

/*************************************************************************
 * ����GB����֡--��������
*************************************************************************/
int32_t GBEP_SendLogin(void)
{
  uint16_t len;     // ��������֡�ĳ���
  uint8_t* pdata = gbep_send_buffer;
  uint8_t* p_iccid;
  int32_t retVal;

  //== GB�������ݹ������� =========================================
  pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // ��ʼ��
  pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
  pdata[GBEP_POS2_ADDRESS] = GBEP_PACKET_CMD_LOGIN; // ���Ԫ
  if (obd_info_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info_data.vin,17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 28;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_GBEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = GBEP_GetDataTime(); //��¼ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  gbep_login_sn++; // ��ˮ�ż�1
  pdata[len++] = (uint8_t)(gbep_login_sn>>8);// ������ˮ��(2B)
  pdata[len++] = (uint8_t)gbep_login_sn;

  p_iccid = Cellura_GetIccid();
  memcpy(&pdata[len],p_iccid,20); // SIM����(ICCID��)(20B)
  len += 20;

  //== У���� =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  //retVal = gsm_socket_send(&gbep_socket, pdata, len);
  //gsm_socket_send(&zxep_socket, pdata, len);
  retVal = GBEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * ����GB����֡--�����ǳ�
*************************************************************************/
int32_t GBEP_SendLogout(void)
{
  uint16_t len;     // ��������֡�ĳ���
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB�������ݹ������� =========================================
  pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // ��ʼ��
  pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
  pdata[GBEP_POS2_ADDRESS] = GBEP_PACKET_CMD_LOGOUT; // ���Ԫ
  if (obd_info_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info_data.vin,17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 0x08;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_GBEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = GBEP_GetDataTime(); //�ǳ�ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  // �ǳ���ˮ���뵱�ε�����ˮ��һ��
  pdata[len++] = (uint8_t)(gbep_login_sn>>8);// �ǳ���ˮ��(2B)
  pdata[len++] = (uint8_t)gbep_login_sn;

  //== У���� =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  //retVal = gsm_socket_send(&gbep_socket, pdata, len);
  //gsm_socket_send(&zxep_socket, pdata, len);
  retVal = GBEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * ����GB����֡--�ն�Уʱ
*************************************************************************/
int32_t GBEP_SendNtpData(void)
{
  uint16_t len;     // ��������֡�ĳ���
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB�������ݹ������� =========================================
  pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // ��ʼ��
  pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
  pdata[GBEP_POS2_ADDRESS] = GBEP_PACKET_CMD_NTP; // ���Ԫ
  if (obd_info_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info_data.vin,17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 0x00;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_GBEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  // �����ն�Уʱ�����ݵ�ԪΪ��

  //== У���� =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  //retVal = gsm_socket_send(&gbep_socket, pdata, len);
  //gsm_socket_send(&zxep_socket, pdata, len);
  retVal = GBEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * GB����: ����������������Ϣ��(������Ϣ)
 * ���ݻ�����С�ˣ�GB�����Ǵ��
*************************************************************************/
void GBEP_BuildEngMessage(uint8_t* pdata)
{
  int32_t lLongitude;
  int32_t lLatitude;
  uint8_t ew_flag;
  uint8_t ns_flag;

  //=============================================================
  memcpy(pdata, ep_data_buffer, SIZE_OF_HJEP_ENG_MAND);

  // �������ε���Ũ��
  //if (pdata[HJEP_ENG_POS7_ADDRESS]==0x00 && pdata[HJEP_ENG_POS7_ADDRESS+1]==0x00)
  //{
  //  pdata[HJEP_ENG_POS7_ADDRESS] = 0xFF;
  //  pdata[HJEP_ENG_POS7_ADDRESS+1] = 0xFF;
  //}

  // ��֧���ֶ����Ϊ0xFF
  //if (CAN_GetEngineType() == ENGINE_TYPE_OTHER) // ����
 // {
 //   pdata[HJEP_ENG_POS7_ADDRESS] = 0xFF; // �������ε���Ũ��(B1:B2)
  //  pdata[HJEP_ENG_POS7_ADDRESS+1] = 0xFF;

 //   pdata[HJEP_ENG_POS13_ADDRESS] = 0xFF; // (DPF/POC)����ѹ��(B4:B5)
  //  pdata[HJEP_ENG_POS13_ADDRESS+1] = 0xFF;
  //}

  //=============================================================
  ew_flag = GPS_GetEastWest();  //���Ȱ���:1=����, 0=����
  ns_flag = GPS_GetNorthSouth();  // γ�Ȱ���:1=��γ, 0=��γ
  if (GPS_GetPositioningStatus()==0) // �ն˲���λ
  {
    //��λ��Ч,�������һ����Ч��λ��Ϣ
    //lLongitude = 0xFFFFFFFF;
    //lLatitude = 0xFFFFFFFF;
    pdata[HJEP_ENG_POS16_ADDRESS] |= 0x01;  // ��λ״̬(1B):1-��λ��Ч
  }
  else
  {
    pdata[HJEP_ENG_POS16_ADDRESS] = 0;  // ��λ״̬(1B):0-��λ��Ч
    if (ns_flag == 0) // ��γ
    {
      pdata[HJEP_ENG_POS16_ADDRESS] |= 0x02;
    }
    if (ew_flag == 0) // ����
    {
      pdata[HJEP_ENG_POS16_ADDRESS] |= 0x04;
    }
  }
  // ����(4B)
  lLongitude = GPS_GetLongitude();  // ����,��λ:�����֮һ��
  pdata[HJEP_ENG_POS17_ADDRESS] = (uint8_t)(lLongitude >> 24);
  pdata[HJEP_ENG_POS17_ADDRESS+1] = (uint8_t)(lLongitude >> 16);
  pdata[HJEP_ENG_POS17_ADDRESS+2] = (uint8_t)(lLongitude >> 8);
  pdata[HJEP_ENG_POS17_ADDRESS+3] = (uint8_t)(lLongitude & 0xFF);
  // γ��(4B)
  lLatitude = GPS_GetLatitude();    // γ��,��λ:�����֮һ��
  pdata[HJEP_ENG_POS18_ADDRESS] = (uint8_t)(lLatitude >> 24);
  pdata[HJEP_ENG_POS18_ADDRESS+1] = (uint8_t)(lLatitude >> 16);
  pdata[HJEP_ENG_POS18_ADDRESS+2] = (uint8_t)(lLatitude >> 8);
  pdata[HJEP_ENG_POS18_ADDRESS+3] = (uint8_t)(lLatitude & 0xFF);
  // ����ʻ���(4B)
  pdata[HJEP_ENG_POS19_ADDRESS] = ep_data_buffer[EP_POS23_ADDRESS];
  pdata[HJEP_ENG_POS19_ADDRESS+1] = ep_data_buffer[EP_POS23_ADDRESS+1];
  pdata[HJEP_ENG_POS19_ADDRESS+2] = ep_data_buffer[EP_POS23_ADDRESS+2];
  pdata[HJEP_ENG_POS19_ADDRESS+3] = ep_data_buffer[EP_POS23_ADDRESS+3];
}

/*************************************************************************
* ����: 1-�ѻ�����������֡, 0-����֡δ����
*************************************************************************/
void GBEP_CacheEngMessage(void)
{
  static uint8_t gbep_eng_packet_index = 0; // ���������ݻ������
  
  if (gbep_eng_packet_index==0) // ��һ������,��¼ʱ��
  {
    rtc_date_t bj_time = GBEP_GetDataTime(); // ���ݲɼ�ʱ��(6B)
    gbep_eng_time_buffer[0] = bj_time.year; // ��
    gbep_eng_time_buffer[1] = bj_time.month;  // ��
    gbep_eng_time_buffer[2] = bj_time.day;  // ��
    gbep_eng_time_buffer[3] = bj_time.hour; // ʱ
    gbep_eng_time_buffer[4] = bj_time.minute;  // ��
    gbep_eng_time_buffer[5] = bj_time.second;  // ��
  }
  
  if(gbep_eng_packet_index < GBEP_ENG_PACKET_MAX_NUM)
  {
    GBEP_BuildEngMessage(&gbep_eng_packet_buffer[gbep_eng_packet_index][0]); // ���淢�������� 
    gbep_eng_packet_index++; // ����ۼ�
    if (gbep_eng_packet_index==GBEP_ENG_PACKET_MAX_NUM)) // ����һ֡��Ҫ10������������
    {
      gbep_eng_packet_index = 0; // �������
      memcpy(gbep_eng_time, gbep_eng_time_buffer, SIZE_OF_GBEP_ENG_TIME); // ����ʱ��
      memcpy(gbep_eng_data, gbep_eng_packet_buffer, (GBEP_ENG_PACKET_MAX_NUM*SIZE_OF_GBEP_ENG)); // ��������
      gbep_eng_data_valid_flag = 1; // ������Ч
    }
  }
}

//==��Ϣ�ɼ�ʱ��+{(��Ϣ���ͱ�־+��ˮ��+��Ϣ��)*10}=======================================
uint16_t GBEP_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;
  
  //== ��Ϣ�ɼ�ʱ��======================================================
  pdata[len++] = gbep_eng_time[0]; // ��
  pdata[len++] = gbep_eng_time[1]; // ��
  pdata[len++] = gbep_eng_time[2]; // ��
  pdata[len++] = gbep_eng_time[3]; // ʱ
  pdata[len++] = gbep_eng_time[4]; // ��
  pdata[len++] = gbep_eng_time[5]; // ��

  for (i=0; i<GBEP_ENG_PACKET_MAX_NUM; i++) // ����������
  {
    gbep_send_data_sn++;  // ��ˮ�ż�1
    
    pdata[len++] = GBEP_DATA_TYPE_ENG_SCR;// ��Ϣ���ͱ�־--����������
    pdata[len++] = (uint8_t)(gbep_send_data_sn>>8);// ��Ϣ��ˮ��(2B)
    pdata[len++] = (uint8_t)gbep_send_data_sn;

    memcpy(&pdata[len], &gbep_eng_data[i][0], SIZE_OF_GBEP_ENG); // ������������Ϣ��
    len += SIZE_OF_GBEP_ENG;
  }
  
  return len;
}

/*************************************************************************
 * ����GB����֡--ʵʱ��Ϣ�ϱ�(���ͻ�����)
*************************************************************************/
int32_t GBEP_SendEngData(void)
{
  uint8_t i;
  uint16_t len = 0;     // ��������֡�ĳ���
  uint16_t data_size = 0; // ���ݵ�Ԫ����
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal = -1;

  if (gbep_eng_data_valid_flag==1) // ������Ч
  {
    gbep_eng_data_valid_flag = 0;

    //== GB�������ݹ������� =========================================
    pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // ��ʼ��
    pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
    pdata[GBEP_POS2_ADDRESS] = GBEP_PACKET_CMD_NEW_DATA; // ���Ԫ
    if (obd_info_data.vin_valid_flag)
    {
      memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info_data.vin,17);// ����ʶ���VIN
    }
    else if (m2m_asset_data.vin_valid_flag)
    {
      memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// ����ʶ���VIN
    }
    else
    {
      return -1;
    }
    pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // �ն�����汾��
    pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

    //== ���ݵ�Ԫ ======================================================
    len = SIZE_OF_GBEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
    msg_len = GBEP_BuildEngData(pdata[len]);
    len += msg_len;
    data_size = msg_len;

    //== ���ݵ�Ԫ���� ==================================================
    pdata[GBEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
    pdata[GBEP_POS6_ADDRESS+1] = (uint8_t)data_size;

    //== У���� =========================================================
    pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //У����(1B)
    len++;

    //retVal = gsm_socket_send(&gbep_socket, pdata, len);
    //gsm_socket_send(&zxep_socket, pdata, len);
    retVal = GBEP_SendNetData(pdata, len);
  }

  return retVal;
}

//==��Ϣ�ɼ�ʱ��+��Ϣ���ͱ�־+��ˮ��+��Ϣ��========================================
uint16_t GBEP_BuildObdData(uint8_t* pdata)
{
  uint16_t len = 0;
  
  rtc_date_t bj_time = GBEP_GetDataTime(); //��Ϣ�ɼ�ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  pdata[len++] = GBEP_DATA_TYPE_OBD;// ��Ϣ���ͱ�־
  
  gbep_send_data_sn++; // ��ˮ�ż�1
  pdata[len++] = (uint8_t)(gbep_send_data_sn>>8);// ��Ϣ��ˮ��(2B)
  pdata[len++] = (uint8_t)gbep_send_data_sn;

  //OBD��Ϣ��==================================
  pdata[len++] = obd_info.protocol_type; // OBD���Э��(1B)  // OBD_PROTOCOL_ISO27145
  pdata[len++] = obd_info.mil_status;  // MIL״̬(1B)
  pdata[len++] = (uint8_t)(obd_info.diag_supported_status>>8);// ���֧��״̬(2B)
  pdata[len++] = (uint8_t)obd_info.diag_supported_status;
  pdata[len++] = (uint8_t)(obd_info.diag_readiness_status>>8);// ��Ͼ���״̬(2B)
  pdata[len++] = (uint8_t)obd_info.diag_readiness_status;
  if (obd_info.vin_valid_flag)
  {
    memcpy(&pdata[len], obd_info.vin, 17);// ����ʶ���VIN(17B)
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[len], m2m_asset_data.vin, 17);// ����ʶ���VIN(17B)
  }
  len += 17;

  memcpy(&pdata[len],obd_info.calid,18);// ����궨ʶ���(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.cvn,18);// �궨��֤��(CVN)(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.iupr,36);// IUPRֵ(36B)
  len += 36;

  pdata[len++] = obd_info.dtc_num;// ����������(1B)
  OBD_GetDtcCode(&obd_info, &pdata[len]);  // ��ȡ�����뵽���ͻ���
  len += (obd_info.dtc_num * 4);    // ��������Ϣ�б�(N*4B)

  return len;
}

/*************************************************************************
 * ����GB����֡--ʵʱ��Ϣ�ϱ�(OBD����)
*************************************************************************/
int32_t GBEP_SendObdData(void)
{
  uint16_t len = 0;     // ��������֡�ĳ���
  uint16_t msg_len;
  uint16_t data_size = 0; // ���ݵ�Ԫ����
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB�������ݹ������� =========================================
  pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // ��ʼ��
  pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
  pdata[GBEP_POS2_ADDRESS] = GBEP_PACKET_CMD_NEW_DATA; // ���Ԫ
  if (obd_info_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info_data.vin,17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_GBEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  msg_len = GBEP_BuildObdData(&pdata[len]); // �������ݵ�Ԫ
  len += msg_len;
  data_size = msg_len;

  //== ���ݵ�Ԫ���� ==================================================
  pdata[GBEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[GBEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== У���� ========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  //retVal = gsm_socket_send(&gbep_socket, pdata, len);
  //gsm_socket_send(&zxep_socket, pdata, len);
  retVal = GBEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * ����GB����֡(10ms����һ��)
*************************************************************************/
void GBEP_ProduceSendData(void)
{
  int32_t retVal;

  switch (gbep_state)
  {
  case GBEP_STATE_SEND_INIT:
    g_stuZXMcuData.gbepDataSN = 0;
    g_stuZXMcuData.gbepLoginSN = 0;
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_IDLE:
    break;

  case GBEP_STATE_SEND_LOGIN: // ������¼
    retVal = GBEP_SendLogin();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:Login!\r\n");
#endif
      gbep_flags1.byte = 0x00;
      GBEP_SEND_LOGIN_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_NTP: // �ն�Уʱ
    retVal = GBEP_SendNtpData();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:Ntp!\r\n");
#endif
      GBEP_SEND_NTP_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_OBD: // ʵʱ��Ϣ�ϱ�(OBD)
    retVal = GBEP_SendObdData();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:OBD!\r\n");
#endif
      GBEP_SEND_OBD_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_ENG: // ʵʱ��Ϣ�ϱ�(������)
    retVal = GBEP_SendEngData();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:ENG!\r\n");
#endif
      GBEP_SEND_ENG_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;
    
  case GBEP_STATE_SEND_BZ: // ������Ϣ�ϱ�
    retVal = GBEP_SendBzData();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:BZ!\r\n");
#endif
      GBEP_SEND_BZ_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break

  case GBEP_STATE_SEND_LOGOUT: // �����ǳ�
    retVal = GBEP_SendLogout();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:Logout!\r\n");
#endif
      gbep_flags1.byte = 0x00;
      GBEP_SEND_LOGOUT_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;
  }
}

/*************************************************************************
* 1�����һ��
*************************************************************************/
void service_gbep_tasks(void)
{
  //uint8_t temp_val = 0x00;
  //static uint8_t obd_data_report_timer = 29;

  if (stu_SYSZXParamSet.ucEngineUpEnable & BIT(1)) // ʹ��GBDBʱ
  {
    // ACC�������³����ݺ�VIN����Ч
    if (GetAccState()==1 && (g_stuZXMcuData.aucHzepDataLen) && ((g_stuZXMcuData.ucVinValidFlag == 0x01) || (obd_info_data.vin_valid_flag==0x01)))
    {
      gbep_socket.enable_flag = SOCKET_TRUE; // ʹ��gbep�׽�������
    }
    else // ACC��
    {
      gbep_socket.enable_flag = SOCKET_FALSE; // ��ֹgbep�׽�������
    }

    // ACC�������³�����
    if (GetAccState()==1 && (g_stuZXMcuData.aucHzepDataLen))
    {
      gbep_CacheEngMessage(); // ��������֡
    }
    else
    {
      gbep_eng_data_valid_flag = 0;
      gbep_eng_packet_index = GBEP_ENG_PACKET_MAX_NUM; // �������
    }

    // ACC=ON,���ͳ�����¼
    if ((GBEP_SEND_LOGIN_FLAG==0) && (GetAccState()==1) && ((g_stuZXMcuData.ucVinValidFlag == 0x01) || (obd_info_data.vin_valid_flag==0x01))) // ACC��
    {
      gbep_state = GBEP_STATE_SEND_LOGIN;
      g_stuZXMcuData.usBjepHeartbeatTimer = MCU_GBEP_HEART_BEAT_TIME;
      //g_stuZXMcuData.usBjepDataSendTimer = 0;
      return;
    }

    // ACC=OFF,���ͳ����ǳ�
    if ((GBEP_SEND_LOGIN_FLAG==1)&&(GetAccState()==0)) // ACC��
    {
      gbep_state = GBEP_STATE_SEND_LOGOUT;
      //g_stuZXMcuData.usBjepDataSendTimer = 0;
      return;
    }

    // ������¼30s��,����NTP����
    if ((GBEP_SEND_LOGIN_FLAG==1) && (g_stuZXMcuData.usBjepHeartbeatTimer==0x00))
    {
      g_stuZXMcuData.usBjepHeartbeatTimer = MCU_GBEP_HEART_BEAT_TIME;
      g_stuZXMcuData.usBjepDataSendTimer = 0;
      gbep_state = GBEP_STATE_SEND_NTP;
      return;
    }

    // ����������30s��,����OBD����
    if ((GBEP_SEND_LOGIN_FLAG==1) && (GBEP_SEND_OBD_FLAG==0))
    {
      if (g_stuZXMcuData.usEngineSpeed > 300) // �³�����������
      {
        gbep_state = GBEP_STATE_SEND_OBD;
        return;
      }
    }

    // ����������,���ͷ���������
    if (GBEP_SEND_LOGIN_FLAG==1)
    {
      gbep_state = GBEP_STATE_SEND_ENG;
      return;
    }
  }
  else
  {
    gbep_state = GBEP_STATE_SEND_INIT;
    gbep_flags1.byte = 0x00;
    gbep_socket.enable_flag = SOCKET_FALSE; // ��ֹgbep�׽�������
  }
}

/******************************************************************************
* ����GBEP����������(10012)��������
*******************************************************************************/
void GBEP_ProcessRecvData(uint8_t* pdata, uint16_t len)
{
  if ((pdata[0]==0x23) && (pdata[1]==0x23)) // ֡ͷ
  {
    gbep_socket.error_cnt = 0x00; // ����������
    gbep_socket.error_flag = FALSE;
    gbep_socket.hb_timer_10ms = gbep_socket.hb_timer_10ms_sp; // ��������������
  }
}


