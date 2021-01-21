/*****************************************************************************
* @FileName: HJ.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.2
* @Date:     2020-1-6
* @brief:    �й����һ���������HJ��׼(���ͳ�Զ���ŷż�ؼ����淶)ʵ��C�ļ�
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
#define HJEP_DEBUG                    1  // 1-ʹ��, 0-��ֹ
#define hjep_send_buffer              hjep_data_buffer
#define HJEP_SendNetData(pData, len)  NetSocket_Send(&hjep_socket, pData, len)
#define HJEP_GetLinkState()           NetSocket_GetLinkState(&hjep_socket)
#define HJEP_DisableLink()            NetSocket_Disable(&hjep_socket)
#define HJEP_EnableLink()             NetSocket_Enable(&hjep_socket)

#define HJEP_SEND_DATA_TIME_SP        99   //��������10��
#define HJEP_SEND_HB_TIME_SP          1800  //��������3����
#define HJEP_SEND_BZ_DATA_TIME_SP     19    //��������2��

/******************************************************************************
* Data Types and Globals
******************************************************************************/
uint8_t hjep_data_buffer[1460];

volatile bittype hjep_flags1;
hjep_state_t hjep_state = HJEP_STATE_SEND_INIT;

uint16_t hjep_login_sn;
uint16_t hjep_send_data_sn;

/******************************************************************************
 * ��ȡʱ��,���ڻ����ϴ�
 ******************************************************************************/
rtc_date_t HJEP_GetDataTime(void)
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
uint8_t HJEP_CalcXorCheck(uint8_t *p, uint16_t len)
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
 *  ����HJ����֡--��������
*************************************************************************/
int32_t HJEP_SendLogin(void)
{
  uint16_t len; // ��������֡�ĳ���
  uint8_t* pdata = hjep_send_buffer;
  uint8_t* p_iccid;
  int32_t retVal;

  //== ��ɽ�������ݹ������� =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_LOGIN; // ���Ԫ
  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN(CAN���߻�ȡ)
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN(ƽ̨�·�)
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 28;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = HJEP_GetDataTime(); //��¼ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  hjep_login_sn++; // ��ˮ�ż�1
  pdata[len++] = (uint8_t)(hjep_login_sn>>8);// ������ˮ��(2B)
  pdata[len++] = (uint8_t)hjep_login_sn;

  p_iccid = Cellura_GetIccid();
  memcpy(&pdata[len],p_iccid,20); // SIM����(ICCID��)(20B)
  len += 20;

  //== У���� =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  ����HJ����֡--�����ǳ�
*************************************************************************/
int32_t HJEP_SendLogout(void)
{
  uint16_t len;     // ��������֡�ĳ���
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== ��ɽ�������ݹ������� =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_LOGOUT; // ���Ԫ
  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 0x08;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = HJEP_GetDataTime(); //�ǳ�ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  // �ǳ���ˮ���뵱�ε�����ˮ��һ��
  pdata[len++] = (uint8_t)(hjep_login_sn>>8);// �ǳ���ˮ��(2B)
  pdata[len++] = (uint8_t)hjep_login_sn;

  //== У���� =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  ����HJ����֡--�ն�Уʱ
*************************************************************************/
int32_t HJEP_SendNtpData(void)
{
  uint16_t len;     // ��������֡�ĳ���
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== ��ɽ�������ݹ������� =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_NTP; // ���Ԫ
  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ���� ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 0x00;

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  // �����ն�Уʱ�����ݵ�ԪΪ��

  //== У���� =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * HJ����: ����������������Ϣ��(������Ϣ)
 * ���ݻ�����С�ˣ���ɽ�����Ǵ��
*************************************************************************/
void HJEP_BuildEngMandMessage(uint8_t* pdata)
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
 * ��ɽ����: ����������������Ϣ��(������Ϣ)
 * ���ݻ�����С�ˣ���ɽ�����Ǵ��
*************************************************************************/
void HJEP_BuildEngAddMessage(uint8_t* pdata)
{
  memcpy(pdata, &ep_data_buffer[EP_POS16_ADDRESS], SIZE_OF_HJEP_ENG_ADD);

  // ��֧���ֶ����Ϊ0xFF
  //if (CAN_GetEngineType() == ENGINE_TYPE_OTHER) // ����
  //{
  //  pdata[HJEP_ADD_POS7_ADDRESS] = 0xFF; // (DPF/POC)�����¶�(B5:B6)
  //  pdata[HJEP_ADD_POS7_ADDRESS+1] = 0xFF;
  //}
}

/*************************************************************************
 * ������ɽ����֡--ʵʱ��Ϣ�ϱ�(���ͻ�����)
*************************************************************************/
int32_t HJEP_SendEngData(void)
{
  uint16_t len = 0;     // ��������֡�ĳ���
  uint16_t data_size = 0; // ���ݵ�Ԫ����
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== ��ɽ�������ݹ������� =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_NEW_DATA; // ���Ԫ
  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = HJEP_GetDataTime(); //���ݷ���ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  hjep_send_data_sn++; // ��ˮ�ż�1
  pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// ��Ϣ��ˮ��(2B)
  pdata[len++] = (uint8_t)hjep_send_data_sn;

  data_size = (6 + 2);

  pdata[len++] = HJEP_DATA_TYPE_ENG_SCR;// ��Ϣ���ͱ�־--����������
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  if (bj_time.second > 5)  // ��
  {
    pdata[len++] = bj_time.second - 2;
  }
  else
  {
    pdata[len++] = 0x00;
  }
  HJEP_BuildEngMandMessage(&pdata[len]); // ������������Ϣ��
  len += SIZE_OF_HJEP_ENG_MAND;
  data_size += (SIZE_OF_HJEP_ENG_MAND + 1 + 6);

  /*************************************************************/
  pdata[len++] = HJEP_DATA_TYPE_ADD;// ��Ϣ���ͱ�־--��������������
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  if (bj_time.second > 5)  // ��
  {
    pdata[len++] = bj_time.second - 2;
  }
  else
  {
    pdata[len++] = 0x00;
  }

  HJEP_BuildEngAddMessage(&pdata[len]);  // ����������������Ϣ��
  len += SIZE_OF_HJEP_ENG_ADD;
  data_size += (SIZE_OF_HJEP_ENG_ADD + 1 + 6);

  //== ���ݵ�Ԫ���� ==================================================
  pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== У���� =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  ������ɽ����֡--ʵʱ��Ϣ�ϱ�(OBD����)
*************************************************************************/
int32_t HJEP_SendObdData(void)
{
  uint16_t len = 0;     // ��������֡�ĳ���
  uint16_t data_size = 0; // ���ݵ�Ԫ����
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  // OBD��Ϣ������Ч
  if (obd_info.diag_valid_flag==0 || obd_info.calid_valid_flag==0 ||\
      obd_info.cvn_valid_flag==0 || obd_info.iupr_valid_flag==0)
  {
    return -1;
  }

  //== ��ɽ�������ݹ������� =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_NEW_DATA; // ���Ԫ
  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

  //== ���ݵ�Ԫ ======================================================
  len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
  rtc_date_t bj_time = HJEP_GetDataTime(); //���ݷ���ʱ��(6B)
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  pdata[len++] = bj_time.second;  // ��

  hjep_send_data_sn++; // ��ˮ�ż�1
  pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// ��Ϣ��ˮ��(2B)
  pdata[len++] = (uint8_t)hjep_send_data_sn;
  data_size = (6 + 2);

  pdata[len++] = HJEP_DATA_TYPE_OBD;// ��Ϣ���ͱ�־
  pdata[len++] = bj_time.year; // ��
  pdata[len++] = bj_time.month;  // ��
  pdata[len++] = bj_time.day;  // ��
  pdata[len++] = bj_time.hour; // ʱ
  pdata[len++] = bj_time.minute;  // ��
  if (bj_time.second > 5)  // ��
  {
    pdata[len++] = bj_time.second - 2;
  }
  else
  {
    pdata[len++] = 0x00;
  }
  data_size += (1 + 6 + SIZE_OF_HJEP_OBD);

  //OBD��Ϣ��==================================
  pdata[len++] = OBD_PROTOCOL_ISO27145;// OBD���Э��(1B)
  if (CAN_GetMilLampState() == 0x00) // MIL״̬(1B)
  {
    pdata[len++] = 0x00;  // δ����
  }
  else
  {
    pdata[len++] = 0x01;  // ����
  }

  pdata[len++] = (uint8_t)(obd_info.diag_supported_status>>8);// ���֧��״̬(2B)
  pdata[len++] = (uint8_t)obd_info.diag_supported_status;
  pdata[len++] = (uint8_t)(obd_info.diag_readiness_status>>8);// ��Ͼ���״̬(2B)
  pdata[len++] = (uint8_t)obd_info.diag_readiness_status;

  if (obd_info.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN(17B)
  }
  else if (m2m_asset_data.vin_valid_flag == 0x01)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN(17B)
  }
  len += 17;

  memcpy(&pdata[len],obd_info.calid,18);// ����궨ʶ���(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.cvn,18);// �궨��֤��(CVN)(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.iupr,36);// IUPRֵ(36B)
  len += 36;

  uint8_t dtc_num = DTC_GetCode(&dtc_27145,&pdata[len+1]); // ��ȡ�����뵽���ͻ���
  pdata[len++] = dtc_num;// ����������(1B)
  len += (dtc_num*4);    // ��������Ϣ�б�(N*4B)
  data_size += (dtc_num*4);

  //== ���ݵ�Ԫ���� ==================================================
  pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== У���� ========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

#if (PART("HJ����ä������"))
/******************************************************************************
* �������ͻ���ä������
******************************************************************************/
void HjepBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_HJEP_BZ_PARA_ADDR, &hjep_blind_zone_para); // ��ȡFLASH�еĲ���
  hjep_blind_zone.wr_error_cnt = hjep_blind_zone_para.wr_error_cnt;
  hjep_blind_zone.rd_error_cnt = hjep_blind_zone_para.rd_error_cnt;
  hjep_blind_zone.top = hjep_blind_zone_para.top;
  hjep_blind_zone.bottom = hjep_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    hjep_blind_zone.data[i] = hjep_blind_zone_para.data[i];
  }

  hjep_blind_zone.timer_1s = HJEP_BZ_SAVE_PERIOD_SP;
  hjep_blind_zone.file_name = FILE_HJEP_BZ_DATA_ADDR; // �ļ���
  hjep_blind_zone.frame_size = HJEP_BLIND_ZONE_PACKET_SIZE; // ����֡�̶�����
  pthread_mutex_init(&hjep_blind_zone.file_mutex, NULL);

#if HJEP_BZ_DEBUG
  PcDebug_Printf("HjepBzInit:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt,hjep_blind_zone.top,hjep_blind_zone.bottom);
#endif
}

//=====================================================================================
void HjepBlindZone_Save(void)
{
  uint16_t i;

  pthread_mutex_lock(&hjep_blind_zone.file_mutex);

  hjep_blind_zone_para.wr_error_cnt = hjep_blind_zone.wr_error_cnt;
  hjep_blind_zone_para.rd_error_cnt = hjep_blind_zone.rd_error_cnt;
  hjep_blind_zone_para.top = hjep_blind_zone.top;
  hjep_blind_zone_para.bottom = hjep_blind_zone.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    hjep_blind_zone_para.data[i] = hjep_blind_zone.data[i];
  }

  BlindZone_SaveParameter(FILE_HJEP_BZ_PARA_ADDR, &hjep_blind_zone_para);

  pthread_mutex_unlock(&hjep_blind_zone.file_mutex);

}

//==��Ϣ���ͱ�־+��Ϣ�ɼ�ʱ��+��Ϣ��===================================================
uint16_t HjepBlindZone_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;

  pdata[len++] = HJEP_DATA_TYPE_ENG_SCR;// ��Ϣ���ͱ�־--����������

  STU_Date stu_date = GetRTCTime_XGZX_TSBJTime(); // ���ݲɼ�ʱ��(6B)
  pdata[len++] = stu_date.ucYear; // ��
  pdata[len++] = stu_date.ucMon;  // ��
  pdata[len++] = stu_date.ucDay;  // ��
  pdata[len++] = stu_date.ucHour; // ʱ
  pdata[len++] = stu_date.ucMin;  // ��
  pdata[len++] = stu_date.ucSec;  // ��

  HJEP_BuildEngMandMessage(&pdata[len]); // ������������Ϣ��
  len += SIZE_OF_HJEP_ENG_MAND;

  /*************************************************************/
  pdata[len++] = TSEP_DATA_TYPE_ADD;// ��Ϣ���ͱ�־--��������������
  pdata[len++] = stu_date.ucYear; // ��
  pdata[len++] = stu_date.ucMon;  // ��
  pdata[len++] = stu_date.ucDay;  // ��
  pdata[len++] = stu_date.ucHour; // ʱ
  pdata[len++] = stu_date.ucMin;  // ��
  pdata[len++] = stu_date.ucSec;  // ��
  TSEP_BuildEngAddMessage(&pdata[len]);  // ����������������Ϣ��
  len += SIZE_OF_TSEP_ENG_ADD;

  return len;
}

//==��Ϣ���ͱ�־+��Ϣ�ɼ�ʱ��+��Ϣ��===================================================
uint16_t HjepBlindZone_BuildObdData(uint8_t* pdata)
{
  uint16_t len = 0;

  pdata[len++] = TSEP_DATA_TYPE_OBD;// ��Ϣ���ͱ�־

  STU_Date stu_date = GetRTCTime_XGZX_TSBJTime(); //��Ϣ�ɼ�ʱ��(6B)
  pdata[len++] = stu_date.ucYear; // ��
  pdata[len++] = stu_date.ucMon;  // ��
  pdata[len++] = stu_date.ucDay;  // ��
  pdata[len++] = stu_date.ucHour; // ʱ
  pdata[len++] = stu_date.ucMin;  // ��
  pdata[len++] = stu_date.ucSec;  // ��

  //OBD��Ϣ��==================================
  pdata[len++] = OBD_PROTOCOL_SAEJ1939; // OBD���Э��(1B)
  if (HZEP_data_buffer[HZEP_POS28_ADDRESS] == 0x00) // MIL״̬(1B)
  {
    pdata[len++] = 0x00;  // δ����
  }
  else
  {
    pdata[len++] = 0x01;  // ����
  }
  pdata[len++] = 0x00;// ���֧��״̬(2B)
  pdata[len++] = 0x00;
  pdata[len++] = 0x00;// ��Ͼ���״̬(2B)
  pdata[len++] = 0x00;

  memcpy(&pdata[len],g_stuZXMcuData.aucVin,17);// ����ʶ����(VIN)(17B)
  len += 17;

  memset(&pdata[len],0xFF,18);// ����궨ʶ���(18B)
  len += 18;

  memset(&pdata[len],0xFF,18);// �궨��֤��(CVN)(18B)
  len += 18;

  memset(&pdata[len],0xFF,36);// IUPRֵ(36B)
  len += 36;

  uint8_t dtc_num = EP_OBD_GetDTC(&pdata[len+1]); // ��ȡ�����뵽���ͻ���
  pdata[len++] = dtc_num;// ����������(1B)
  len += (dtc_num*4);    // ��������Ϣ�б�(N*4B)

  return len;
}

//==1s����һ��(����������ACC�������³����ݺ�VIN����Ч�µ���)============================
void HjepBlindZone_Service(void)
{
  static uint8_t obd_data_report_timer = 10;
  uint8_t acc_state;

  acc_state = COLT_GetAccStatus();
  if (acc_state==1)  // ACC�������³����ݺ�VIN����Ч
  {
    if (hjep_blind_zone.timer_1s)
      hjep_blind_zone.timer_1s--;
    else
    {
      hjep_blind_zone.timer_1s = HJEP_BZ_SAVE_PERIOD_SP; // ������һ��
      if (HJEP_GetLinkState() != SOCKET_LINK_STATE_READY) // δ����
      {
        if (GPS_GetPositioningStatus()==1 && (g_stuZXMcuData.aucHzepDataLen != 0)) // �ն��Ѷ�λ��������
        {
          memset(hjep_blind_zone_buffer,0xFF,HJEP_BLIND_ZONE_PACKET_SIZE); // ��ջ���

          if (obd_data_report_timer)
          {
            obd_data_report_timer--;
            hjep_blind_zone_length = HjepBlindZone_BuildEngData(hjep_blind_zone_buffer); // ��������Ϣ
          }
          else
          {
            obd_data_report_timer = 10;
            hjep_blind_zone_length = HjepBlindZone_BuildObdData(hjep_blind_zone_buffer); // OBD��Ϣ
          }

          if (hjep_blind_zone_length <= HJEP_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&hjep_blind_zone, hjep_blind_zone_buffer, hjep_blind_zone_length); // ��������֡
            HjepBlindZone_Save(); // ����ջ����

#if HJEP_BZ_DEBUG
            PcDebug_Printf("HjepBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt,hjep_blind_zone.top,hjep_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC�ر�
  {
    hjep_blind_zone.timer_1s = HJEP_BZ_SAVE_PERIOD_SP; // ���洢
  }

  // ���ֶ�д����,��λջΪ0
  if ((hjep_blind_zone.rd_error_cnt > HJEP_BDZE_READ_ERROR_SP) || (hjep_blind_zone.wr_error_cnt > HJEP_BDZE_WRITE_ERROR_SP))
  {
#if HJEP_BZ_DEBUG
    PcDebug_Printf("HjepBzErr:Ewr=%d,Erd=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt);
#endif
    hjep_blind_zone.wr_error_cnt = 0;
    hjep_blind_zone.rd_error_cnt = 0;
    hjep_blind_zone.top = 0;
    hjep_blind_zone.bottom = 0;
    memset(hjep_blind_zone.data, 0x00, BLIND_ZONE_STACK_MAX_SIZE);
    HjepBlindZone_Save(); // ����ջ����
  }
}

//==������Ϣ�ϱ�==============================================================================
int32_t HjepBlindZone_SendData(void)
{
  uint16_t len = 0;     // ��������֡�ĳ���
  uint16_t data_size = 0; // ���ݵ�Ԫ����
  uint16_t stack_size = 0x00;
  uint16_t bind_zone_data_size = 0x00;
  uint8_t* pdata = tsep_send_buffer;
  int32_t retVal = -1;

  stack_size = BlindZone_GetStackSize(&hjep_blind_zone);
  if (stack_size > 0)
  {
    //== ��ɽ�������ݹ������� =========================================
    pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // ��ʼ��
    pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
    pdata[HJEP_POS2_ADDRESS] = HJEP_PACKET_CMD_BZ_DATA; // ���Ԫ
    if (obd_info.vin_valid_flag == 0x01)
    {
      memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// ����ʶ���VIN
    }
    else if (m2m_asset_data.vin_valid_flag == 0x01)
    {
      memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// ����ʶ���VIN
    }
    else
    {
      return -1;
    }
    pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // �ն�����汾��
    pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // ���ݼ��ܷ�ʽ

    //== ���ݵ�Ԫ ======================================================
    len = SIZE_OF_HJEP_HEADER; // ���ݵ�Ԫ��ʼ��ַ
    rtc_date_t bj_time = HJEP_GetDataTime(); //���ݷ���ʱ��(6B)
    pdata[len++] = bj_time.year; // ��
    pdata[len++] = bj_time.month;  // ��
    pdata[len++] = bj_time.day;  // ��
    pdata[len++] = bj_time.hour; // ʱ
    pdata[len++] = bj_time.minute;  // ��
    pdata[len++] = bj_time.second;  // ��

    hjep_send_data_sn++; // ��ˮ�ż�1
    pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// ��Ϣ��ˮ��(2B)
    pdata[len++] = (uint8_t)hjep_send_data_sn;
    
    data_size = (6 + 2);

    /*******************************��Ϣ��(��ʼ)*******************************/
    BlindZone_PopData(&hjep_blind_zone, &pdata[len], &bind_zone_data_size);
    HjepBlindZone_Save(); // ����ջ����
    if (bind_zone_data_size == 0x00)
    {
#if HJEP_BZ_DEBUG
      PcDebug_Printf("HjepBzPop:Err\r\n");
#endif
      return retVal;
    }

    len += bind_zone_data_size;
    data_size += bind_zone_data_size;
    /*******************************��Ϣ��(����)*******************************/

    //== ���ݵ�Ԫ���� ==================================================
    pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
    pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

    //== У���� =========================================================
    pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //У����(1B)
    len++;

    retVal = HJEP_SendNetData(pdata, len);

#if HJEP_BZ_DEBUG
    PcDebug_Printf("HjepBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt,hjep_blind_zone.top,hjep_blind_zone.bottom);
#endif
  }

  return retVal;
}
#endif

/*************************************************************************
* HJ����״̬��,100ms����һ��(������)
*************************************************************************/
void HJEP_StateMachine(void)
{
  static uint8_t hjep_send_bz_data_time = 50;
  static uint16_t hjep_send_data_time;
  static uint16_t hjep_send_hb_time;
  uint8_t vin_valid_flag;
  uint8_t ep_data_valid_flag;
  uint8_t acc_state;
  uint16_t engine_speed;
  static uint8_t hjep_socket_enable_flag = FALSE;

  if (CAN_GetEpType()==EP_TYPE_HJ) // �������ܿ���
  {
    vin_valid_flag = CAN_GetVinState();
    acc_state = COLT_GetAccStatus();
    ep_data_valid_flag = CAN_GetEpDataState();
    engine_speed = CAN_GetEngineSpeed();
    
    // ACC�������³����ݺ�VIN����Ч
    if ((acc_state==1) && ((ep_data_valid_flag==1) || (vin_valid_flag==1)))
    {
      HJEP_EnableLink(); // ʹ��hjep����
      hjep_socket_enable_flag = TRUE;
    }
    else // ACC��
    {
      HJEP_DisableLink(); // ��ֹhjep����
      hjep_socket_enable_flag = FALSE;
    }

    if (HJEP_GetLinkState() != SOCKET_LINK_STATE_READY) // �ȴ��������
    {
      return;
    }

    // ACC=ON,���ͳ�����¼
    if (HJEP_SEND_LOGIN_FLAG==0) // ����δ��¼
    {
      if((acc_state==1) && (vin_valid_flag==1)) // ACC����VIN����Ч
      {
        hjep_state = HJEP_STATE_SEND_LOGIN;
        hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
        hjep_send_data_time = 30;
        hjep_send_bz_data_time = 50;
        return;
      }
    }
    else  // �����ѵ�¼
    {
      // ACC=OFF,���ͳ����ǳ�
      if (acc_state==0) // ACC��
      {
        hjep_state = HJEP_STATE_SEND_LOGOUT;
        return;
      }

      // ������¼30s��,����NTP����
      if (hjep_send_hb_time==0x00)
      {
        hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
        hjep_state = HJEP_STATE_SEND_NTP;
        return;
      }

      // ����������30s��,����OBD����
      if (HJEP_SEND_OBD_FLAG==0 && (hjep_send_data_time==0))
      {
        if (engine_speed > 300) // �³�����������
        {
          // OBD��Ϣ������Ч
          if (obd_info.diag_valid_flag==1 && obd_info.calid_valid_flag==1 &&\
              obd_info.cvn_valid_flag==1 && obd_info.iupr_valid_flag==1)
          {
            hjep_send_data_time = HJEP_SEND_DATA_TIME_SP;
            hjep_state = HJEP_STATE_SEND_OBD;
            return;
          }
        }
      }

      // ����������,���ͷ���������
      if (hjep_send_data_time==0)
      {
        hjep_send_data_time = HJEP_SEND_DATA_TIME_SP;  // �������ݷ���ʱ��
        hjep_send_bz_data_time = HJEP_SEND_BZ_DATA_TIME_SP;  // ����ä������ʱ��
        hjep_state = HJEP_STATE_SEND_ENG;
        return;
      }

      // ������Ϣ�ϱ�
      if(hjep_send_bz_data_time==0)
      {
        hjep_send_bz_data_time = HJEP_SEND_BZ_DATA_TIME_SP;  // ����ä������ʱ��
        hjep_state = HJEP_STATE_SEND_BZ;
        return;
      }
    }// end if (HJEP_SEND_LOGIN_FLAG==1)
  }
  else // ��������δ����
  {
    hjep_state = HJEP_STATE_SEND_INIT;
    hjep_flags1.byte = 0x00;
    HJEP_DisableLink(); // ��ֹbjep�׽�������
    hjep_socket_enable_flag = FALSE;
  }

  // ���ݷ���ʱ�������ʱ�䵹��
  if (hjep_socket_enable_flag == TRUE)
  {
    if (hjep_send_data_time) // ���ݷ��Ͷ�ʱ��
      hjep_send_data_time--;

    if (hjep_send_hb_time) // �������Ͷ�ʱ��
      hjep_send_hb_time--;

    if(hjep_send_bz_data_time) // �������ݶ�ʱ��
      hjep_send_bz_data_time--;
  }
  else
  {
    hjep_send_data_time = HJEP_SEND_DATA_TIME_SP;
    hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
  }
}


/*************************************************************************
 * ������ɽ����֡(10ms����һ��)
*************************************************************************/
void HJEP_ProduceSendData(void)
{
  int32_t retVal;
  static uint8_t divide_for_100ms = 9;

  // �ն���Ϣ(������300ms)
  if (divide_for_100ms)
  {
    divide_for_100ms--;
  }
  else
  {
    divide_for_100ms = 9;  // 100ms����
    HJEP_StateMachine();
  }

  switch (hjep_state)
  {
  case HJEP_STATE_SEND_INIT: // ��ʼ����¼��ˮ�ź�������ˮ��
    // hjep_login_sn = 0;
    // hjep_send_data_sn = 0;
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_IDLE: // ����״̬
    break;

  case HJEP_STATE_SEND_LOGIN: // ������¼
    retVal = HJEP_SendLogin();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:Login!\r\n");
#endif
      hjep_flags1.byte = 0x00;
      HJEP_SEND_LOGIN_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_NTP: // �ն�Уʱ
    retVal = HJEP_SendNtpData();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:Ntp!\r\n");
#endif
      HJEP_SEND_NTP_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_OBD: // ʵʱ��Ϣ�ϱ�(OBD)
    retVal = HJEP_SendObdData();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:OBD!\r\n");
#endif
      HJEP_SEND_OBD_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_ENG: // ʵʱ��Ϣ�ϱ�(������)
    retVal = HJEP_SendEngData();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:ENG!\r\n");
#endif
      HJEP_SEND_ENG_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_BZ: // ������Ϣ�ϱ�
//    retVal = HJEP_SendEngData();
//    if (retVal==0)
//    {
//#if HJEP_DEBUG
//      PcDebug_SendString("HJEP:BZ!\r\n");
//#endif
//      HJEP_SEND_BZ_FLAG = 1;
//    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_LOGOUT: // �����ǳ�
    retVal = HJEP_SendLogout();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:Logout!\r\n");
#endif
      hjep_flags1.byte = 0x00;
      HJEP_SEND_LOGOUT_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;
  }
}

/******************************************************************************
* ����HJEP����������(10012)��������
*******************************************************************************/
void HJEP_ProcessRecvData(uint8_t* pdata, uint16_t len)
{
#if 0
  if((pdata[0]==0x23) && (pdata[1]==0x23)) // ֡ͷ
  {
    hjep_socket.error_cnt = 0x00; // ����������
    hjep_socket.error_flag = FALSE;
  }
#endif
}

/******************************************************************************
* ��ʼ������
*******************************************************************************/
void HJEP_Initialize(void)
{
  HjepBlindZone_Init();  // ä������(��������)

  hjep_state = HJEP_STATE_SEND_INIT;
  hjep_flags1.byte = 0x00;
  //HJEP_DisableLink();  // ��ֹhjep�׽�������

  // ������
  m2m_asset_data.vin_valid_flag = 0x01;
  m2m_asset_data.ep_type = EP_TYPE_HJ;
}

