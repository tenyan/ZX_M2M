/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: GpsNMEA.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-12-20
 * @brief:    Basic parser for the NMEA0813 protocol.
 * @Status:   Parsing GGA, RMC, GSA ,VTG and GSV
 *******************************************************************************/
//-----头文件调用------------------------------------------------------------
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
uint8_t nmea_msg_buffer[ONE_NMEA_MSG_MAX_SIZE]; // 指令缓存

nav_info_t nav_data;
gps_context_t gps_context;

pthread_mutex_t nmeaMsgQueueMutex;

/******************************************************************************
 * NMEA分词器定义
 ******************************************************************************/
typedef char*  token_t;  // 类型定义
#define MAX_NMEA_TOKEN_COUNT 32

typedef struct
{
  uint16_t count;
  token_t tokens[MAX_NMEA_TOKEN_COUNT];
}nmea_tokenizer_t;
nmea_tokenizer_t tokenizer;

/******************************************************************************
* 清空指令数据
******************************************************************************/
void nmea_msg_queue_reset(void)
{
  nmea_msg_queue.head = 0;
  nmea_msg_queue.tail = 0;
  //nmea_msg_state = 0;
  nmea_msg_pos = 0;
}

/******************************************************************************
* 串口接收的数据，通过此函数放入指令队列
******************************************************************************/
void nmea_msg_queue_push(uint8_t dat)
{
  uint16_t pos;

  pos = (nmea_msg_queue.head + 1) % NMEA_MSG_QUEUE_MAX_SIZE;

  if (pos != nmea_msg_queue.tail) // 非满状态
  {
    nmea_msg_queue.data[nmea_msg_queue.head] = dat;
    nmea_msg_queue.head = pos;
  }
}

/******************************************************************************
* 从队列中取一个数据
******************************************************************************/
static void nmea_msg_queue_pop(uint8_t* p_data)
{
  if (nmea_msg_queue.tail != nmea_msg_queue.head) //非空状态
  {
    *p_data = nmea_msg_queue.data[nmea_msg_queue.tail];
    nmea_msg_queue.tail = (nmea_msg_queue.tail + 1) % NMEA_MSG_QUEUE_MAX_SIZE;
  }
}

/******************************************************************************
* 获取队列中有效数据个数
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
 * 输入一条完整的NMEA语句
 * NMEA-0183协议的校验和计算方法 ：
 * 是 将 “$” 和 “*” 之间的字符ASCII值进行异或运算得到的数值以16进制格式体现的字符串
 *****************************************************************************/
uint8_t nmea_check_crc(const char *buff, uint16_t buff_sz)
{
  uint16_t it;
  uint8_t calculated_crc;
  uint8_t received_crc;

  if (buff[buff_sz-5]=='*')
  {
    calculated_crc = buff[1]; // 从‘$’后的字符开始
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
    return NMEA_NOK; // 校验失败
  }
}

/******************************************************************************
*\brief  从指令队列中取出一条完整的指令
*\param  pbuffer 指令接收缓存区
*\param  size 指令接收缓存区大小
*\return  指令长度，0表示队列中无完整指令
******************************************************************************/
uint16_t nmea_msg_queue_find(uint8_t *buffer, uint16_t size)
{
  uint16_t msg_size = 0;
  uint8_t _data = 0;

  while (nmea_msg_queue_size() > 0)
  {
    nmeaMsgQueueENTER_CRITICAL(); // 关闭串口接收中断
    nmea_msg_queue_pop(&_data);   // 取一个数据
    nmeaMsgQueueEXIT_CRITICAL();  // 开启串口接收中断

    if ((nmea_msg_pos == 0)&&(_data != NMEA_MSG_HEAD)) // 指令第一个字节必须是帧头，否则跳过
    {
      continue;
    }

    if (nmea_msg_pos < size) // 防止缓冲区溢出
      buffer[nmea_msg_pos++] = _data;

    nmea_msg_tail = ((nmea_msg_tail << 8) | _data); // 拼接最后2个字节，组成一个16位整数
    if (nmea_msg_tail == NMEA_MSG_TAIL) // 最后2个字节与帧尾匹配，得到完整帧
    {
      msg_size = nmea_msg_pos;  // 指令字节长度
      nmea_msg_tail = 0;        // 重新检测帧尾巴
      nmea_msg_pos = 0;         // 复位指令指针

      if (nmea_check_crc((const char *)buffer, msg_size)==NMEA_NOK) // CRC校验(输入一条完整的NMEA语句)
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
  return 0; // 没有完整数据帧
}

/******************************************************************************
 * 从一条NEMA语句提取各信息的首地址存入分词器,并将所有逗号替换成NULL
 * 寻找所有逗号的位置，创建索引
 ******************************************************************************/
void nmea_convert_to_tokenizer(nmea_tokenizer_t *tokenizer,uint8_t *p_msg, uint16_t msg_size)
{
  uint16_t i, counter = 0;
  uint8_t flag = NMEA_TRUE;

  for (i= 0; i<msg_size; i++) // 读取一个字节
  {
    if (flag==NMEA_TRUE)
    {
      flag = NMEA_FALSE;
      token_t token = (token_t)(p_msg+i); // 获取字符串首地址
      tokenizer->tokens[counter] = token; // 存入首地址
      counter++;
    }

    if (p_msg[i] == ',')
    {
      p_msg[i] = 0x00; // 将所有‘,’替换成NULL
      flag = TRUE;
    }
  }
  tokenizer->count = counter; // 存入字符串条数
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
  // lat纬度:ddmm.mmmm
  // lon经度:dddmm.mmmm
  int ddmm;
  int dd;
  int mm;
  int mmmmm;
  uint8_t str_size;
  uint8_t i=0;

  ddmm = nmea_parse_number(value);
  dd = (int)(ddmm / 100);
  mm = ddmm % 100;

  str_size = strlen(value); // 获取字符串大小
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
  // 1节(kn)=1海里/小时=1852/3600(m/s)
  // 速度:ii.fff
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

    if (bDec) // 小数
    {
      if (chByte >= '0' && chByte <= '9')
      {
        val = (val*10) + (chByte-'0');
      }
      else
      {
        return (val*463/25); // 单位是0.1KM/H
      }
    }
    else // 整数
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
        return (val*463/25); // 单位是0.1KM/H
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
 * \brief: GPS定位信息
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
  //token_t utcToken = nmea_tokenizer_get(tokenizer, 1); // 当前定位的 UTC 时间
  //token_t latToken = nmea_tokenizer_get(tokenizer, 2); // 纬度
  //token_t nsToken = nmea_tokenizer_get(tokenizer, 3);  // 纬度方向
  //token_t lonToken = nmea_tokenizer_get(tokenizer, 4); // 经度
  //token_t ewToken = nmea_tokenizer_get(tokenizer, 5);  // 经度方向
  token_t fixValidToken = tokenizer->tokens[6]; // 指示当前定位质量
  token_t numSvFixedToken = tokenizer->tokens[7]; // 用于定位的卫星数目
  //token_t hdopToken = tokenizer->tokens[8]; // 水平精度因子
  token_t altToken = tokenizer->tokens[9];  // 海拔高度

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
 * \brief: 推荐最小定位信息
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

  token_t utcTimeToken = tokenizer->tokens[1]; // 当前定位的 UTC 时间
  token_t pixToken = tokenizer->tokens[2]; // 位置有效标志
  token_t latToken = tokenizer->tokens[3]; // 纬度
  token_t nsToken = tokenizer->tokens[4];  // 纬度方向
  token_t lonToken = tokenizer->tokens[5]; // 经度
  token_t ewToken = tokenizer->tokens[6];  // 经度方向
  token_t speedToken = tokenizer->tokens[7]; // 对地速度(单位为节)
  token_t headingToken = tokenizer->tokens[8]; // 对地真航向
  token_t utcDateToken = tokenizer->tokens[9]; // 日期

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
 * \brief: 当前卫星信息
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
 * \brief: 可见卫星信息
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
 * \brief: 地面速度信息
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
    //"BDGGA", "BDGSA", "BDGSV", "BDRMC", "BDVTG",  // 北斗
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

  if (p_msg[6] == ',' && p_msg[7] == ',') // 此条NMEA语句为空
  {
#if GPS_DEBUG
    PcDebug_Printf("NMEA-NONE!\n");
#endif
    return;
  }

  memset(&tokenizer, 0x00, sizeof(nmea_tokenizer_t)); // 0x00 == NULL
  nmea_convert_to_tokenizer(&tokenizer,(uint8_t*)(p_msg+3), (msg_size-3));
  //nmea_convert_to_tokenizer(&tokenizer,(uint8_t*)(p_msg), msg_size);
  headToken = tokenizer.tokens[0];      // 获取NMEA语句头
  pack_type = nmea_pack_type(headToken); // 获取NMEA语句类型
  switch (pack_type) // 根据语句类型解析响应的语句
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

  while (nmea_msg_queue_size() > 0) // 缓存队列中有数据
  {
    nmea_msg_size = nmea_msg_queue_find(nmea_msg_buffer, ONE_NMEA_MSG_MAX_SIZE); // 从缓冲区中获取一条NMEA语句(不含*CRC\r\n)
    if (nmea_msg_size > 0) // 得到一条完成的NMEA语句
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

#define SPEED_OVERRUN_YES_DT_SP  10  // 10秒
#define SPEED_OVERRUN_NO_DT_SP   5   // 5秒
//==检查GPS速度是否超限====================================================
void GPS_CheckSpeedIsOverrun(void)
{
  static uint8_t speed_overrun_yes_debounce_cnt = 0;
  static uint8_t speed_overrun_no_debounce_cnt = 0;
  uint16_t tempVal;

  if (nav_data.fixValid==NMEA_TRUE) // 模块已定位
  {
    if (m2m_asset_data.over_speed_sp > 5) // 启用超速报警
    {
      tempVal = nav_data.speed / 10; // 转成KM/H
      if (tempVal >= m2m_asset_data.over_speed_sp)
      {
        speed_overrun_no_debounce_cnt = 0;
        if (speed_overrun_yes_debounce_cnt > SPEED_OVERRUN_YES_DT_SP)
        {
          if (gps_context.speedStatus == GPS_OK)
          {
            colt_info.switch3 |= BIT(3);
            gps_context.speedStatus = GPS_NOK; // 超速
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
            gps_context.speedStatus = GPS_OK; // 正常
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

#define GPS_3D_YES_DT_SP  5   // 5秒
#define GPS_3D_NO_DT_SP   5   // 5秒
/******************************************************************************
 * 检测GPS定位状态变化
*******************************************************************************/
void GPS_CheckPositioningState(void)
{
  static uint8_t gps_3d_state = 0; // 初始状态
  static uint8_t gps_3d_yes_debounce_cnt = 0;
  static uint8_t gps_3d_no_debounce_cnt = 0;

  //if (gps_module_state==GPS_MODULE_STATE_WORKING)
  //{
    if(GPS_GetPositioningStatus()) // 模块定位
    {
      gps_3d_no_debounce_cnt = 0;
      if(gps_3d_yes_debounce_cnt > GPS_3D_YES_DT_SP)
      {
        if(gps_3d_state==0)
        {
          PcDebug_SendString("GPS: 3D!\n");  // 定位有效
          //SbusMsg_GpsWarning.type = GPS_MSG_TYPE_3D_OK;
          //SYSBUS_PutMbox(SbusMsg_GpsWarning);
        }
        gps_3d_state =1;
      }
      else
        gps_3d_yes_debounce_cnt++;
    }
    else  // 模块未定位
    {
      gps_3d_yes_debounce_cnt = 0;
      if(gps_3d_no_debounce_cnt > GPS_3D_NO_DT_SP)
      {
        if(gps_3d_state==1)
        {
          PcDebug_SendString("GPS: NOD!\n"); // 定位无效
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
 * 检查GPS模块是不是正常工作
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
      gps_context.moduleStatus = GPS_NOK;  // 模块异常
      nav_data.fixValid = NMEA_FALSE;      // GPS强制不定位
    }
  //}
}

/******************************************************************************
 * GPS周期性调用函数,1S调用一次,无阻塞
*******************************************************************************/
void GPS_Do1sTasks(void)
{
  GPS_CheckModuleIsOk();
  GPS_CheckSpeedIsOverrun();
  GPS_CheckPositioningState();
}

/******************************************************************************
 * GPS模块进程
******************************************************************************/
void* phread_GpsProcess(void *argument)
{
  while (1)
  {
    msleep(50);  // 50ms周期
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
 * 外部用户接口
 *******************************************************************************/
//==获取GPS卫星使用颗数========================================================
uint8_t GPS_GetSatelliteNum(void)
{
  return nav_data.numSvFix;
}

//==获取GPS模块定位状态(1=定位,0=未定位)=======================================
uint8_t GPS_GetPositioningStatus(void)
{
  return nav_data.fixValid;
}

//==获取经度半球(1=东经,0=西经)==============================================
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

//==获取维度半球(1=北纬,0=南纬)==============================================
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

//==获取当前的经度,单位百万分之一度============================================
uint32_t GPS_GetLongitude(void)
{
  return nav_data.lon;
}

//==获取当前的纬度,单位百万分之一度============================================
uint32_t GPS_GetLatitude(void)
{
  return nav_data.lat;
}

//==获取车辆对地速度(0.1km/h) 0-2200===========================================
uint16_t GPS_GetSpeed(void)
{
  return nav_data.speed;
}

//==获取GPS对地真航向(单位为度) 0-360==========================================
uint16_t GPS_GetHeading(void)
{
  return nav_data.heading;
}

//=获取GPS海拔高度(米),海平面以下为负数========================================
int16_t GPS_GetHeight(void)
{
  return nav_data.alt;
}

//==获取GPS模块的UTC时间=======================================================
utc_time_t GPS_GetUtcTime(void)
{
  return nav_data.utc;
}

//==获取GPS模块状态(1异常;0正常)==============================================
uint8_t GPS_GetModuleStatus(void)
{
  return gps_context.moduleStatus;
}

//==获取GPS天线状态(0=正常,1=异常)============================================
uint8_t GPS_GetAntennaStatus(void)
{
  //if ( (colt_info.alarm&BIT(5)) || (colt_info.alarm&BIT(6)) )
  //  return 1;
  //else
  //  return 0;
  return gps_context.antStatus;
}

//==获取GPS天线短路状态=======================================================
uint8_t GPS_GetAntShortStatus(void)
{
  //return (!(colt_info.alarm & BIT(5)))? 1:0; // 1=短路,0=正常
  return gps_context.antShortStatus;
}

//==获取GPS天线开路状态=======================================================
uint8_t GPS_GetAntOpenStatus(void)
{
  //return (colt_info.alarm & BIT(6))? 1:0; // 1=开路,0=正常
  return gps_context.antOpenStatus;
}

//==获取GPS超速状态(1=超速;0=正常)============================================
uint8_t GPS_GetSpeedOverrunStatus(void)
{
  return gps_context.speedStatus;
}

