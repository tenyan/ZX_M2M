/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsNMEA.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-12-20
 * @brief:    Basic parser for the NMEA0813 protocol.
 * @Status:   Parsing GGA, RMC, GSA ,VTG and GSV
 *******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define NMEA_CONVSTR_BUF    (32)

/******************************************************************************
 * Data Types
 ******************************************************************************/
static nmea_msg_queue_t nmea_msg_queue = {0,0};

static uint16_t nmea_msg_tail = 0;
static uint16_t nmea_msg_pos = 0;

#define ONE_NMEA_MSG_MAX_SIZE   256
uint8_t nmea_msg_buffer[ONE_NMEA_MSG_MAX_SIZE]; // ָ���

nav_info_t nav_data;
gps_context_t gps_context;

pthread_mutex_t nmeaMsgQueueMutex;

/******************************************************************************
 * NMEA�ִ�������
 ******************************************************************************/
typedef char*  token_t;  // ���Ͷ���
#define MAX_NMEA_TOKEN_COUNT 32

typedef struct
{
  uint16_t count;
  token_t tokens[MAX_NMEA_TOKEN_COUNT];
}nmea_tokenizer_t;
nmea_tokenizer_t tokenizer;

/******************************************************************************
* ���ָ������
******************************************************************************/
void nmea_msg_queue_reset(void)
{
  nmea_msg_queue.head = 0;
  nmea_msg_queue.tail = 0;
  //nmea_msg_state = 0;
  nmea_msg_pos = 0;
}

/******************************************************************************
* ���ڽ��յ����ݣ�ͨ���˺�������ָ�����
******************************************************************************/
void nmea_msg_queue_push(uint8_t dat)
{
  uint16_t pos;

  pos = (nmea_msg_queue.head + 1) % NMEA_MSG_QUEUE_MAX_SIZE;

  if (pos != nmea_msg_queue.tail) // ����״̬
  {
    nmea_msg_queue.data[nmea_msg_queue.head] = dat;
    nmea_msg_queue.head = pos;
  }
}

/******************************************************************************
* �Ӷ�����ȡһ������
******************************************************************************/
static void nmea_msg_queue_pop(uint8_t* p_data)
{
  if (nmea_msg_queue.tail != nmea_msg_queue.head) //�ǿ�״̬
  {
    *p_data = nmea_msg_queue.data[nmea_msg_queue.tail];
    nmea_msg_queue.tail = (nmea_msg_queue.tail + 1) % NMEA_MSG_QUEUE_MAX_SIZE;
  }
}

/******************************************************************************
* ��ȡ��������Ч���ݸ���
******************************************************************************/
static uint16_t nmea_msg_queue_size(void)
{
  return ((nmea_msg_queue.head + NMEA_MSG_QUEUE_MAX_SIZE - nmea_msg_queue.tail) % NMEA_MSG_QUEUE_MAX_SIZE);
}

/******************************************************************************
 * \brief : Parse number from string
 * \param[in,out]   str: string to parse
 * \return  Parsed number
 *****************************************************************************/
int32_t nmea_parse_number(const char* str)
{
  int32_t val = 0;
  uint8_t minus = 0;

  if (*str == '-') // Check negative number
  {
    minus = 1;
    str++;
  }

  // Parse until character is valid number
  while (((*str) >= '0' && (*str) <= '9'))
  {
    val = val * 10 + ((*str)-'0');
    str++;
  }

  return minus ? -val : val;
}

/*******************************************************************************
 * \brief : Parse number from string as hex
 * \note Input string pointer is changed and number is skipped
 * \param[in,out] str: Pointer to pointer to string to parse
 * \return  Parsed number
 *******************************************************************************/
uint32_t nmea_parse_hex_number(const char* str)
{
  uint32_t val = 0;

  while (1) // Parse until character is valid number
  {
    if ((*str >= '0') && (*str <= '9'))
    {
      val = val * 16 + ((*str)-'0');
      str++;
    }
    else if ((*str >= 'A') && (*str <= 'F'))
    {
      val = val * 16 + ((*str)-'A'+10);
      str++;
    }
    else if ((*str >= 'a') && (*str <= 'f'))
    {
      val = val * 16 + ((*str)-'a'+10);
      str++;
    }
    else
    {
      return val;
    }
  }
}

/******************************************************************************
 * ����һ��������NMEA���
 * NMEA-0183Э���У��ͼ��㷽�� ��
 * �� �� ��$�� �� ��*�� ֮����ַ�ASCIIֵ�����������õ�����ֵ��16���Ƹ�ʽ���ֵ��ַ���
 *****************************************************************************/
uint8_t nmea_check_crc(const char *buff, uint16_t buff_sz)
{
  uint16_t it;
  uint8_t calculated_crc;
  uint8_t received_crc;

  if (buff[buff_sz-5]=='*')
  {
    calculated_crc = buff[1]; // �ӡ�$������ַ���ʼ
    for (it = 2; buff[it]!='*'; it++)
    {
      calculated_crc ^= buff[it];
    }
    received_crc = nmea_parse_hex_number((buff+it+1));

    if (calculated_crc == received_crc) //check if crc's match
      return NMEA_OK;
    else
      return NMEA_NOK;
  }
  else
  {
    return NMEA_NOK; // У��ʧ��
  }
}

/******************************************************************************
*\brief  ��ָ�������ȡ��һ��������ָ��
*\param  pbuffer ָ����ջ�����
*\param  size ָ����ջ�������С
*\return  ָ��ȣ�0��ʾ������������ָ��
******************************************************************************/
uint16_t nmea_msg_queue_find(uint8_t *buffer, uint16_t size)
{
  uint16_t msg_size = 0;
  uint8_t _data = 0;

  while (nmea_msg_queue_size() > 0)
  {
    nmeaMsgQueueENTER_CRITICAL(); // �رմ��ڽ����ж�
    nmea_msg_queue_pop(&_data);   // ȡһ������
    nmeaMsgQueueEXIT_CRITICAL();  // �������ڽ����ж�

    if ((nmea_msg_pos == 0)&&(_data != NMEA_MSG_HEAD)) // ָ���һ���ֽڱ�����֡ͷ����������
    {
      continue;
    }

    if (nmea_msg_pos < size) // ��ֹ���������
      buffer[nmea_msg_pos++] = _data;

    nmea_msg_tail = ((nmea_msg_tail << 8) | _data); // ƴ�����2���ֽڣ����һ��16λ����
    if (nmea_msg_tail == NMEA_MSG_TAIL) // ���2���ֽ���֡βƥ�䣬�õ�����֡
    {
      msg_size = nmea_msg_pos;  // ָ���ֽڳ���
      nmea_msg_tail = 0;        // ���¼��֡β��
      nmea_msg_pos = 0;         // ��λָ��ָ��

      if (nmea_check_crc((const char *)buffer, msg_size)==NMEA_NOK) // CRCУ��(����һ��������NMEA���)
      {
#if GPS_DEBUG
        PcDebug_Printf("NMEA-CRC!\n");
#endif
        return 0;
      }
      msg_size -= 5;  // tail_sz = 3 /* *[CRC] */ + 2 /* \r\n */
      return msg_size;
    }
  }
  return 0; // û����������֡
}

/******************************************************************************
 * ��һ��NEMA�����ȡ����Ϣ���׵�ַ����ִ���,�������ж����滻��NULL
 * Ѱ�����ж��ŵ�λ�ã���������
 ******************************************************************************/
void nmea_convert_to_tokenizer(nmea_tokenizer_t *tokenizer,uint8_t *p_msg, uint16_t msg_size)
{
  uint16_t i, counter = 0;
  uint8_t flag = NMEA_TRUE;

  for (i= 0; i<msg_size; i++) // ��ȡһ���ֽ�
  {
    if (flag==NMEA_TRUE)
    {
      flag = NMEA_FALSE;
      token_t token = (token_t)(p_msg+i); // ��ȡ�ַ����׵�ַ
      tokenizer->tokens[counter] = token; // �����׵�ַ
      counter++;
    }

    if (p_msg[i] == ',')
    {
      p_msg[i] = 0x00; // �����С�,���滻��NULL
      flag = TRUE;
    }
  }
  tokenizer->count = counter; // �����ַ�������
}

//=======================================================================
void nmea_utc_to_date_time(utc_time_t* data_time, token_t utcDate, token_t utcTime)
{
  uint32_t n;

  // HHMMSS.MS: 101943.235-> 10:19:43.235 ->
  // hour = 10, minute = 19, second = 43, ms = 235
  n = nmea_parse_number(utcTime);
  data_time->hour = (n / 10000);
  data_time->minute = (n / 100) % 100;
  data_time->second = n % 100;
  // DDMMYY: 100816 -> 2016-08-10
  n = nmea_parse_number(utcDate);
  data_time->year = n % 100;
  data_time->month = (n / 100) % 100;
  data_time->day = n / 10000;
}

//=======================================================================
int32_t nmae_parse_lat_lon(token_t value, token_t sign)
{
  // latγ��:ddmm.mmmm
  // lon����:dddmm.mmmm
  int ddmm;
  int dd;
  int mm;
  int mmmmm;
  uint8_t str_size;
  uint8_t i=0;

  ddmm = nmea_parse_number(value);
  dd = (int)(ddmm / 100);
  mm = ddmm % 100;

  str_size = strlen(value); // ��ȡ�ַ�����С
  while (value[i] != '.')
  {
    i++;
    if (i >= str_size)
    {
      return 0xFFFFFFFF;
    }
  }

  mmmmm = nmea_parse_number((value+i+1));
  ddmm = dd*1000000;
  ddmm += (mm*100000+mmmmm)/6;

  if (strcmp(sign, "W") == 0 || strcmp(sign, "S") == 0)
  {
    ddmm = -ddmm;
  }

  return ddmm;
}

//=======================================================================
uint32_t nmae_parse_speed(token_t speed)
{
  // 1��(kn)=1����/Сʱ=1852/3600(m/s)
  // �ٶ�:ii.fff
  uint8_t bDec = 0;
  char chByte = '0';
  char* pSor = speed;
  uint32_t val = 0;

  if (!speed)
  {
    return 0;
  }

  while (*pSor != '\0')
  {
    chByte = *pSor;

    if (bDec) // С��
    {
      if (chByte >= '0' && chByte <= '9')
      {
        val = (val*10) + (chByte-'0');
      }
      else
      {
        return (val*463/25); // ��λ��0.1KM/H
      }
    }
    else // ����
    {
      if (chByte >= '0' && chByte <= '9')
      {
        val = (val*10) + (chByte-'0');
      }
      else if (chByte == '.')
      {
        bDec = 1;
      }
      else
      {
        return (val*463/25); // ��λ��0.1KM/H
      }
    }

    pSor++;
  }

  return val;
}


//=======================================================================
uint8_t checkFixValid(token_t fixValidToken)
{
  if (strcmp(fixValidToken, "1") == 0 || strcmp(fixValidToken, "2") == 0 || strcmp(fixValidToken, "6") == 0)
  {
    return NMEA_TRUE;
  }

  return NMEA_FALSE;
}

/******************************************************************************
 * \brief: GPS��λ��Ϣ
 * $GNGGA,072647.00,3000.22101,N,11957.86974,E,1,12,0.60,22.8,M,6.9,M,,*4C
 ******************************************************************************/
uint8_t nmea_parse_gga(nmea_tokenizer_t *tokenizer, nav_info_t *navdata)
{
  uint8_t fixValid;

  if (tokenizer->count < 10)
  {
    return NMEA_FALSE;
  }

#if GPS_DEBUG
  //PcDebug_Printf("NMEA-GGA!\n");
#endif

  // get pix from tokens
  //token_t utcToken = nmea_tokenizer_get(tokenizer, 1); // ��ǰ��λ�� UTC ʱ��
  //token_t latToken = nmea_tokenizer_get(tokenizer, 2); // γ��
  //token_t nsToken = nmea_tokenizer_get(tokenizer, 3);  // γ�ȷ���
  //token_t lonToken = nmea_tokenizer_get(tokenizer, 4); // ����
  //token_t ewToken = nmea_tokenizer_get(tokenizer, 5);  // ���ȷ���
  token_t fixValidToken = tokenizer->tokens[6]; // ָʾ��ǰ��λ����
  token_t numSvFixedToken = tokenizer->tokens[7]; // ���ڶ�λ��������Ŀ
  //token_t hdopToken = tokenizer->tokens[8]; // ˮƽ��������
  token_t altToken = tokenizer->tokens[9];  // ���θ߶�

  fixValid = checkFixValid(fixValidToken);
  if (fixValid)
  {
    navdata->numSvFix = nmea_parse_number(numSvFixedToken);
    navdata->alt = nmea_parse_number(altToken);
    //navdata->hdop = atof(hdopToken);
  }
  return NMEA_TRUE;
}

/******************************************************************************
 * \brief: �Ƽ���С��λ��Ϣ
 * $GNRMC,072648.00,A,3000.22252,N,11957.88619,E,51.487,84.34,290716,,,A,V*09
 ******************************************************************************/
uint8_t nmea_parse_rmc(nmea_tokenizer_t *tokenizer, nav_info_t *navdata)
{
  if (tokenizer->count < 10)
  {
    return NMEA_FALSE;
  }

#if GPS_DEBUG
  PcDebug_Printf("NMEA-RMC!\n");
#endif

  token_t utcTimeToken = tokenizer->tokens[1]; // ��ǰ��λ�� UTC ʱ��
  token_t pixToken = tokenizer->tokens[2]; // λ����Ч��־
  token_t latToken = tokenizer->tokens[3]; // γ��
  token_t nsToken = tokenizer->tokens[4];  // γ�ȷ���
  token_t lonToken = tokenizer->tokens[5]; // ����
  token_t ewToken = tokenizer->tokens[6];  // ���ȷ���
  token_t speedToken = tokenizer->tokens[7]; // �Ե��ٶ�(��λΪ��)
  token_t headingToken = tokenizer->tokens[8]; // �Ե��溽��
  token_t utcDateToken = tokenizer->tokens[9]; // ����

  navdata->fixValid = strcmp(pixToken, "A") == 0;
  if (navdata->fixValid)
  {
    nmea_utc_to_date_time(&navdata->utc,utcDateToken,utcTimeToken);

    if (strcmp(nsToken, "S") == 0)
    {
      navdata->ns = 'S';
    }
    else if (strcmp(nsToken, "N") == 0)
    {
      navdata->ns = 'N';
    }

    if (strcmp(ewToken, "W")==0)
    {
      navdata->ew = 'W';
    }
    else if (strcmp(ewToken, "E")==0)
    {
      navdata->ew = 'E';
    }

    navdata->lon = nmae_parse_lat_lon(lonToken, ewToken);
    navdata->lat = nmae_parse_lat_lon(latToken ,nsToken);
    navdata->speed = nmae_parse_speed(speedToken);
    navdata->heading = nmea_parse_number(headingToken);
  }
  return NMEA_TRUE;
}

/******************************************************************************
 * \brief: ��ǰ������Ϣ
 ******************************************************************************/
uint8_t nmea_parse_gsa(nmea_tokenizer_t *tokenizer, nav_info_t *navdata)
{
#if 0
  if (tokenizer->count < 18)
  {
    return NMEA_FALSE;
  }

#if GPS_DEBUG
  PcDebug_Printf("NMEA-GSA!\n");
#endif

  token_t headToken = tokenizer->tokens[0];
  int svSys = nmea_parser_get_sv_sys(headToken);
  int i, prn;
  for (i = 3; i <= 14;i++)
  {
    prn = atoi(tokenizer->tokens[i]);
    if (prn > 0)
    {
      prn = encode_prn(prn, svSys);
      navdata->svFixMask[prn] = 1;
    }
  }
  token_t pdopToken = tokenizer->tokens[15];
  token_t hdopToken = tokenizer->tokens[16];
  token_t vdopToken = tokenizer->tokens[17];

  navdata->pdop = atof(pdopToken);
  navdata->hdop = atof(hdopToken);
  navdata->vdop = atof(vdopToken);
#endif

  return NMEA_TRUE;
}

/******************************************************************************
 * \brief: �ɼ�������Ϣ
 ******************************************************************************/
uint8_t nmea_parse_gsv(nmea_tokenizer_t *tokenizer, nav_info_t *navdata)
{
#if 0
  if (tokenizer->count < 3)
  {
    return NMEA_FALSE;
  }

#if GPS_DEBUG
  PcDebug_Printf("NMEA-GSV!\n");
#endif

  token_t headToken = tokenizer->tokens[0];
  int svSys = nmea_parser_get_sv_sys(headToken);
  int i, prn;

  for (i = 4; i + 4 <= nmea_tokenizer_get_count(tokenizer); i += 4)
  {
    token_t prnToken = tokenizer->tokens[i];
    token_t elevToken = tokenizer->tokens[i+1];
    token_t azimToken = tokenizer->tokens[i+2];
    token_t cn0Token = tokenizer->tokens[i+3];

    prn = atoi(prnToken);
    if (prn > 0)
    {
      prn = encode_prn(prn, svSys);
      int n = navdata->numSvView;

      SATE *sate = &navdata->svArray[n];
      sate->prn = decode_prn(prn);
      sate->sys = get_sv_sys_from_prn(prn);
      sate->inUse = navdata->svFixMask[prn] != 0;
      sate->cn0 = atoi(cn0Token);
      sate->azim = atoi(azimToken);
      sate->elev = atoi(elevToken);

      navdata->numSvView = n + 1;
    }
  }
#endif

  return NMEA_TRUE;
}

/******************************************************************************
 * \brief: �����ٶ���Ϣ
 ******************************************************************************/
uint8_t nmea_parse_vtg(nmea_tokenizer_t *tokenizer, nav_info_t *navdata)
{
#if 0
  if (tokenizer->count < 3)
  {
    return NMEA_FALSE;
  }

#if GPS_DEBUG
  PcDebug_Printf("NMEA-VTG!\n");
#endif

#endif

  return NMEA_TRUE;
}

/******************************************************************************
 * \brief Define packet type by header (nmeaPACKTYPE).
 * @param buff a constant character pointer of packet buffer.
 * @param buff_sz buffer size.
 * @return The defined packet type
 * @see nmeaPACKTYPE
 *****************************************************************************/
int nmea_pack_type(const char *buff)
{
  static const char *pheads[] = {
    "GGA", "RMC","GSA", "GSV",  "VTG",  // GPS
    //"GPGGA", "GPGSA", "GPGSV", "GPRMC", "GPVTG",  // GPS
    //"BDGGA", "BDGSA", "BDGSV", "BDRMC", "BDVTG",  // ����
    //"GNGGA", "GNGSA", "GNGSV", "GNRMC", "GNVTG",  // GLONASS
  };

  if (0 == memcmp(buff, pheads[0], 3))
  {
    return NMEA_PACK_TYPE_GGA;
  }
  else if (0 == memcmp(buff, pheads[1], 3))
  {
    return NMEA_PACK_TYPE_RMC;
  }
  else if (0 == memcmp(buff, pheads[2], 3))
  {
    return NMEA_PACK_TYPE_GSA;
  }
  else if (0 == memcmp(buff, pheads[3], 3))
  {
    return NMEA_PACK_TYPE_GSV;
  }
  else if (0 == memcmp(buff, pheads[4], 3))
  {
    return NMEA_PACK_TYPE_VTG;
  }

  return NMEA_PACK_TYPE_NONE;
}

/*********************************************************************************
 * \brief Analysis of buffer and put results to information structure
 * @return Number of packets was parsed
 ********************************************************************************/
void gps_nmea_parse(nav_info_t *p_info, const char *p_msg, uint16_t msg_size)
{
  static nmea_tokenizer_t tokenizer;
  token_t headToken;
  uint8_t pack_type;

  if (p_msg[6] == ',' && p_msg[7] == ',') // ����NMEA���Ϊ��
  {
#if GPS_DEBUG
    PcDebug_Printf("NMEA-NONE!\n");
#endif
    return;
  }

  memset(&tokenizer, 0x00, sizeof(nmea_tokenizer_t)); // 0x00 == NULL
  nmea_convert_to_tokenizer(&tokenizer,(uint8_t*)(p_msg+3), (msg_size-3));
  //nmea_convert_to_tokenizer(&tokenizer,(uint8_t*)(p_msg), msg_size);
  headToken = tokenizer.tokens[0];      // ��ȡNMEA���ͷ
  pack_type = nmea_pack_type(headToken); // ��ȡNMEA�������
  switch (pack_type) // ����������ͽ�����Ӧ�����
  {
  case NMEA_PACK_TYPE_GGA:
    nmea_parse_gga(&tokenizer, p_info);
    break;

  case NMEA_PACK_TYPE_RMC:
    nmea_parse_rmc(&tokenizer, p_info);
    break;

  case NMEA_PACK_TYPE_GSA:
    nmea_parse_gsa(&tokenizer, p_info);
    break;

  case NMEA_PACK_TYPE_GSV:
    nmea_parse_gsv(&tokenizer, p_info);
    break;

  case NMEA_PACK_TYPE_VTG:
    nmea_parse_vtg(&tokenizer, p_info);
    break;

  default:
    break;
  }
}

/******************************************************************************
 * \brief
 ******************************************************************************/
void nmea_zero_info(nav_info_t *info)
{
  memset(info, 0, sizeof(nav_info_t));
  info->fixValid = 0;
  info->ns = 'N';
  info->ew = 'E';
}

/*********************************************************************************
 * \brief
 ********************************************************************************/
void gps_nmea_parse_init(void)
{
  nmea_msg_queue_reset();
  nmea_zero_info(&nav_data);
}

/*************************************************************************
 *
*************************************************************************/
void GPS_Service(void)
{
  uint16_t nmea_msg_size = 0x00;

  while (nmea_msg_queue_size() > 0) // ���������������
  {
    nmea_msg_size = nmea_msg_queue_find(nmea_msg_buffer, ONE_NMEA_MSG_MAX_SIZE); // �ӻ������л�ȡһ��NMEA���(����*CRC\r\n)
    if (nmea_msg_size > 0) // �õ�һ����ɵ�NMEA���
    {
      if (gps_context.moduleStatus == GPS_NOK)
      {
        gps_context.moduleStatus = GPS_OK;
        //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_MODULE_OK;
        //SYSBUS_PutMbox(SbusMsg_GpsWarning);
        PcDebug_SendString("GpsModule:OK!\n");
      }
      gps_context.no_msg_recv_timer = GPS_RECV_TIMEOUT_SP;
      //PcDebug_Printf("NmeaQ:head=%d,tail=%d\n",nmea_msg_queue.head, nmea_msg_queue.tail);
      //PcDebug_SendData(nmea_msg_buffer, nmea_msg_size, DBG_MSG_TYPE_GPS);
      gps_nmea_parse(&nav_data, (const char*)nmea_msg_buffer, nmea_msg_size);
    }
  }
}

#define SPEED_OVERRUN_YES_DT_SP  10  // 10��
#define SPEED_OVERRUN_NO_DT_SP   5   // 5��
//==���GPS�ٶ��Ƿ���====================================================
void GPS_CheckSpeedIsOverrun(void)
{
  static uint8_t speed_overrun_yes_debounce_cnt = 0;
  static uint8_t speed_overrun_no_debounce_cnt = 0;
  uint16_t tempVal;

  if (nav_data.fixValid==NMEA_TRUE) // ģ���Ѷ�λ
  {
    if (m2m_asset_data.over_speed_sp > 5) // ���ó��ٱ���
    {
      tempVal = nav_data.speed / 10; // ת��KM/H
      if (tempVal >= m2m_asset_data.over_speed_sp)
      {
        speed_overrun_no_debounce_cnt = 0;
        if (speed_overrun_yes_debounce_cnt > SPEED_OVERRUN_YES_DT_SP)
        {
          if (gps_context.speedStatus == GPS_OK)
          {
            colt_info.switch3 |= BIT(3);
            gps_context.speedStatus = GPS_NOK; // ����
            //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_OVER_SPEED;
            //SYSBUS_PutMbox(SbusMsg_GpsWarning);
          }
        }
        else
          speed_overrun_yes_debounce_cnt++;
      }
      else
      {
        speed_overrun_yes_debounce_cnt = 0;
        if (speed_overrun_no_debounce_cnt > SPEED_OVERRUN_NO_DT_SP)
        {
          if (gps_context.speedStatus == GPS_NOK)
          {
            colt_info.switch3 &= ~BIT(3);
            gps_context.speedStatus = GPS_OK; // ����
            //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_NORMAL_SPEED;
            //SYSBUS_PutMbox(SbusMsg_GpsWarning);
          }
        }
        else
          speed_overrun_no_debounce_cnt++;
      }
    }
  }
}

#define GPS_3D_YES_DT_SP  5   // 5��
#define GPS_3D_NO_DT_SP   5   // 5��
/******************************************************************************
 * ���GPS��λ״̬�仯
*******************************************************************************/
void GPS_CheckPositioningState(void)
{
  static uint8_t gps_3d_state = 0; // ��ʼ״̬
  static uint8_t gps_3d_yes_debounce_cnt = 0;
  static uint8_t gps_3d_no_debounce_cnt = 0;

  //if (gps_module_state==GPS_MODULE_STATE_WORKING)
  //{
    if(GPS_GetPositioningStatus()) // ģ�鶨λ
    {
      gps_3d_no_debounce_cnt = 0;
      if(gps_3d_yes_debounce_cnt > GPS_3D_YES_DT_SP)
      {
        if(gps_3d_state==0)
        {
          PcDebug_SendString("GPS: 3D!\n");  // ��λ��Ч
          //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_3D_OK;
          //SYSBUS_PutMbox(SbusMsg_GpsWarning);
        }
        gps_3d_state =1;
      }
      else
        gps_3d_yes_debounce_cnt++;
    }
    else  // ģ��δ��λ
    {
      gps_3d_yes_debounce_cnt = 0;
      if(gps_3d_no_debounce_cnt > GPS_3D_NO_DT_SP)
      {
        if(gps_3d_state==1)
        {
          PcDebug_SendString("GPS: NOD!\n"); // ��λ��Ч
          //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_3D_NOK;
          //SYSBUS_PutMbox(SbusMsg_GpsWarning);
        }
        gps_3d_state = 0;
      }
      else
        gps_3d_no_debounce_cnt++;
    }
  //}
}

/*************************************************************************
 * ���GPSģ���ǲ�����������
*************************************************************************/
void GPS_CheckModuleIsOk(void)
{
  //if (gps_module_state==GPS_MODULE_STATE_WORKING)
  //{
    if (gps_context.no_msg_recv_timer)
      gps_context.no_msg_recv_timer--;
    else
    {
      if (gps_context.moduleStatus == GPS_OK)
      {
        //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_MODULE_ERR;
        //SYSBUS_PutMbox(SbusMsg_GpsWarning);
        PcDebug_SendString("GpsModule:Err!\n");
      }
      gps_context.no_msg_recv_flag = GPS_NOK;
      gps_context.moduleStatus = GPS_NOK;  // ģ���쳣
      nav_data.fixValid = NMEA_FALSE;      // GPSǿ�Ʋ���λ
    }
  //}
}

/******************************************************************************
 * GPS�����Ե��ú���,1S����һ��,������
*******************************************************************************/
void GPS_Do1sTasks(void)
{
  GPS_CheckModuleIsOk();
  GPS_CheckSpeedIsOverrun();
  GPS_CheckPositioningState();
}

/******************************************************************************
 * GPSģ�����
******************************************************************************/
void* phread_GpsProcess(void *argument)
{
  while (1)
  {
    msleep(50);  // 50ms����
    GPS_Service();
  }
}

//==========================================================================
void GPS_ServiceInit(void)
{
  gps_context.no_msg_recv_timer = GPS_RECV_TIMEOUT_SP;
  gps_context.no_msg_recv_flag = GPS_OK;
  //gps_context.antStatus = GPS_OK;
  //gps_context.antOpenStatus = GPS_OK;
  //gps_context.antShortStatus = GPS_OK;
  //gps_context.moduleStatus = GPS_OK;
  gps_context.speedStatus = GPS_OK;
  gps_nmea_parse_init();
}

//==========================================================================
void GPS_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_GPS_PROCESS_ID], NULL, phread_GpsProcess, NULL);
  usleep(10);
}

/******************************************************************************
 * �ⲿ�û��ӿ�
 *******************************************************************************/
//==��ȡGPS����ʹ�ÿ���========================================================
uint8_t GPS_GetSatelliteNum(void)
{
  return nav_data.numSvFix;
}

//==��ȡGPSģ�鶨λ״̬(1=��λ,0=δ��λ)=======================================
uint8_t GPS_GetPositioningStatus(void)
{
  return nav_data.fixValid;
}

//==��ȡ���Ȱ���(1=����,0=����)==============================================
uint8_t GPS_GetEastWest(void)
{
  if (nav_data.ew=='W')
  {
    return 0x00;
  }
  else
  {
    return 0x01;
  }
}

//==��ȡά�Ȱ���(1=��γ,0=��γ)==============================================
uint8_t GPS_GetNorthSouth(void)
{
  if (nav_data.ns=='S')
  {
    return 0x00;
  }
  else
  {
    return 0x01;
  }
}

//==��ȡ��ǰ�ľ���,��λ�����֮һ��============================================
uint32_t GPS_GetLongitude(void)
{
  return nav_data.lon;
}

//==��ȡ��ǰ��γ��,��λ�����֮һ��============================================
uint32_t GPS_GetLatitude(void)
{
  return nav_data.lat;
}

//==��ȡ�����Ե��ٶ�(0.1km/h) 0-2200===========================================
uint16_t GPS_GetSpeed(void)
{
  return nav_data.speed;
}

//==��ȡGPS�Ե��溽��(��λΪ��) 0-360==========================================
uint16_t GPS_GetHeading(void)
{
  return nav_data.heading;
}

//=��ȡGPS���θ߶�(��),��ƽ������Ϊ����========================================
int16_t GPS_GetHeight(void)
{
  return nav_data.alt;
}

//==��ȡGPSģ���UTCʱ��=======================================================
utc_time_t GPS_GetUtcTime(void)
{
  return nav_data.utc;
}

//==��ȡGPSģ��״̬(1�쳣;0����)==============================================
uint8_t GPS_GetModuleStatus(void)
{
  return gps_context.moduleStatus;
}

//==��ȡGPS����״̬(0=����,1=�쳣)============================================
uint8_t GPS_GetAntennaStatus(void)
{
  //if ( (colt_info.alarm&BIT(5)) || (colt_info.alarm&BIT(6)) )
  //  return 1;
  //else
  //  return 0;
  return gps_context.antStatus;
}

//==��ȡGPS���߶�·״̬=======================================================
uint8_t GPS_GetAntShortStatus(void)
{
  //return (!(colt_info.alarm & BIT(5)))? 1:0; // 1=��·,0=����
  return gps_context.antShortStatus;
}

//==��ȡGPS���߿�·״̬=======================================================
uint8_t GPS_GetAntOpenStatus(void)
{
  //return (colt_info.alarm & BIT(6))? 1:0; // 1=��·,0=����
  return gps_context.antOpenStatus;
}

//==��ȡGPS����״̬(1=����;0=����)============================================
uint8_t GPS_GetSpeedOverrunStatus(void)
{
  return gps_context.speedStatus;
}

