/*****************************************************************************
* @FileName: HJ.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.2
* @Date:     2020-1-6
* @brief:    中国国家环境保护部HJ标准(重型车远程排放监控技术规范)实现C文件
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
// 网络接口定义
#define hjep_send_buffer              hjep_data_buffer
#define HJEP_SendNetData(pData, len)  NetSocket_Send(&hjep_socket, pData, len)
#define HJEP_GetLinkState()           NetSocket_GetLinkState(&hjep_socket)
#define HJEP_DisableLink()            NetSocket_Disable(&hjep_socket)
#define HJEP_EnableLink()             NetSocket_Enable(&hjep_socket)

// HJ数据发送周期定义
#define HJEP_SEND_ENG_DATA_TIME_SP        90   //发送周期10秒
#define HJEP_SEND_OBD_DATA_TIME_SP        190   //发送周期20秒

#define HJEP_SEND_HB_TIME_SP          1200  //发送周期3分钟
#define HJEP_SEND_BZ_DATA_TIME_SP     14    //发送周期1.5秒

// 盲区补发
#define HJEP_BZ_SAVE_PERIOD_SP  299 // 30s
#define HJEP_BLIND_ZONE_PACKET_SIZE  512
#define HJEP_BDZE_WRITE_ERROR_SP  6
#define HJEP_BDZE_READ_ERROR_SP   6

/******************************************************************************
* Data Types and Globals
******************************************************************************/
uint8_t hjep_data_buffer[1460];

// 盲区补偿对象生命
blind_zone_t hjep_blind_zone;
blind_zone_para_t hjep_blind_zone_para;

// 缓存
static uint8_t hjep_blind_zone_buffer[HJEP_BLIND_ZONE_PACKET_SIZE];
static uint16_t hjep_blind_zone_length;

// HJ状态
volatile bittype hjep_flags1;
hjep_state_t hjep_state = HJEP_STATE_SEND_INIT;

// 流水号
uint16_t hjep_login_sn;
uint16_t hjep_send_data_sn;

void HjepBlindZone_Save(void);

/******************************************************************************
 * 获取时间,用于环保上传
 ******************************************************************************/
rtc_date_t HJEP_GetDataTime(void)
{
  rtc_date_t bj_time,utc_time;

  if (GPS_GetPositioningStatus()==0) // 终端不定位 采用外部RTC时间
  {
    bj_time = RTC_GetBjTime(); // 获取RTC时间
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
 * 计算异或校验值
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
 * 构建包头
*************************************************************************/
int32_t HJEP_BuildMsgHead(uint8_t cmdType, uint8_t *pdata)
{
  //== HJ环保数据公共部分 =========================================
  pdata[HJEP_POS1_ADDRESS] = HJEP_PACKET_HEADER1; // 起始符
  pdata[HJEP_POS1_ADDRESS+1] = HJEP_PACKET_HEADER2;
  pdata[HJEP_POS2_ADDRESS] = cmdType; // 命令单元
  if (obd_info.vin_valid_flag)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], obd_info.vin, 17);// 车辆识别号VIN(CAN总线获取)
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[HJEP_POS3_ADDRESS], m2m_asset_data.vin, 17);// 车辆识别号VIN(平台下发)
  }
  else
  {
    return -1;
  }
  pdata[HJEP_POS4_ADDRESS] = HJEP_PACKET_SW_VERSION; // 终端软件版本号
  pdata[HJEP_POS5_ADDRESS] = HJEP_PACKET_ENCRYPT_NONE; // 数据加密方式
  
  return 0;
}


/*************************************************************************
 *  发送HJ环保帧--车辆登入
*************************************************************************/
int32_t HJEP_SendLogin(void)
{
  uint16_t len; // 整个数据帧的长度
  uint8_t* pdata = hjep_send_buffer;
  uint8_t* p_iccid;
  int32_t retVal;

  //== HJ环保数据公共部分 =========================================
  retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_LOGIN, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元长度 ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 28;

  //== 数据单元 ======================================================
  len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = HJEP_GetDataTime(); //登录时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  hjep_login_sn++; // 流水号加1
  pdata[len++] = (uint8_t)(hjep_login_sn>>8);// 登入流水号(2B)
  pdata[len++] = (uint8_t)hjep_login_sn;

  p_iccid = Cellura_GetIccid();
  memcpy(&pdata[len],p_iccid,20); // SIM卡号(ICCID号)(20B)
  len += 20;

  //== 校验码 =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  发送HJ环保帧--车辆登出
*************************************************************************/
int32_t HJEP_SendLogout(void)
{
  uint16_t len;     // 整个数据帧的长度
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== HJ环保数据公共部分 =========================================
  retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_LOGOUT, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元长度 ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 0x08;

  //== 数据单元 ======================================================
  len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = HJEP_GetDataTime(); //登出时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  // 登出流水号与当次登入流水号一致
  pdata[len++] = (uint8_t)(hjep_login_sn>>8);// 登出流水号(2B)
  pdata[len++] = (uint8_t)hjep_login_sn;

  //== 校验码 =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  发送HJ环保帧--终端校时
*************************************************************************/
int32_t HJEP_SendNtpData(void)
{
  uint16_t len;     // 整个数据帧的长度
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== HJ环保数据公共部分 =========================================
  retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_NTP, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元长度 ==================================================
  pdata[HJEP_POS6_ADDRESS] = 0x00;
  pdata[HJEP_POS6_ADDRESS+1] = 0x00;

  //== 数据单元 ======================================================
  len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
  // 车载终端校时的数据单元为空

  //== 校验码 =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * HJ环保: 创建发动机数据信息体(必须信息)
 * 杭州环保是小端，HJ环保是大端
*************************************************************************/
void HJEP_BuildEngMandMessage(uint8_t* pdata)
{
  int32_t lLongitude;
  int32_t lLatitude;
  uint8_t ew_flag;
  uint8_t ns_flag;

  //=============================================================
  memcpy(pdata, ep_data_buffer, SIZE_OF_HJEP_ENG_MAND);

  // 后处理上游氮氧浓度
  //if (pdata[HJEP_ENG_POS7_ADDRESS]==0x00 && pdata[HJEP_ENG_POS7_ADDRESS+1]==0x00)
  //{
  //  pdata[HJEP_ENG_POS7_ADDRESS] = 0xFF;
  //  pdata[HJEP_ENG_POS7_ADDRESS+1] = 0xFF;
  //}

  // 不支持字段填充为0xFF
  //if (CAN_GetEngineType() == ENGINE_TYPE_OTHER) // 杭发
 // {
 //   pdata[HJEP_ENG_POS7_ADDRESS] = 0xFF; // 后处理上游氮氧浓度(B1:B2)
  //  pdata[HJEP_ENG_POS7_ADDRESS+1] = 0xFF;

 //   pdata[HJEP_ENG_POS13_ADDRESS] = 0xFF; // (DPF/POC)载体压差(B4:B5)
  //  pdata[HJEP_ENG_POS13_ADDRESS+1] = 0xFF;
  //}

  //=============================================================
  ew_flag = GPS_GetEastWest();  //经度半球:1=东经, 0=西经
  ns_flag = GPS_GetNorthSouth();  // 纬度半球:1=北纬, 0=南纬
  if (GPS_GetPositioningStatus()==0) // 终端不定位
  {
    //地位无效,发送最后一次有效定位信息
    //lLongitude = 0xFFFFFFFF;
    //lLatitude = 0xFFFFFFFF;
    pdata[HJEP_ENG_POS16_ADDRESS] |= 0x01;  // 定位状态(1B):1-定位无效
  }
  else
  {
    pdata[HJEP_ENG_POS16_ADDRESS] = 0;  // 定位状态(1B):0-定位有效
    if (ns_flag == 0) // 南纬
    {
      pdata[HJEP_ENG_POS16_ADDRESS] |= 0x02;
    }
    if (ew_flag == 0) // 西经
    {
      pdata[HJEP_ENG_POS16_ADDRESS] |= 0x04;
    }
  }
  // 经度(4B)
  lLongitude = GPS_GetLongitude();  // 经度,单位:百万分之一度
  pdata[HJEP_ENG_POS17_ADDRESS] = (uint8_t)(lLongitude >> 24);
  pdata[HJEP_ENG_POS17_ADDRESS+1] = (uint8_t)(lLongitude >> 16);
  pdata[HJEP_ENG_POS17_ADDRESS+2] = (uint8_t)(lLongitude >> 8);
  pdata[HJEP_ENG_POS17_ADDRESS+3] = (uint8_t)(lLongitude & 0xFF);
  // 纬度(4B)
  lLatitude = GPS_GetLatitude();    // 纬度,单位:百万分之一度
  pdata[HJEP_ENG_POS18_ADDRESS] = (uint8_t)(lLatitude >> 24);
  pdata[HJEP_ENG_POS18_ADDRESS+1] = (uint8_t)(lLatitude >> 16);
  pdata[HJEP_ENG_POS18_ADDRESS+2] = (uint8_t)(lLatitude >> 8);
  pdata[HJEP_ENG_POS18_ADDRESS+3] = (uint8_t)(lLatitude & 0xFF);
  // 总行驶里程(4B)
  pdata[HJEP_ENG_POS19_ADDRESS] = ep_data_buffer[EP_POS23_ADDRESS];
  pdata[HJEP_ENG_POS19_ADDRESS+1] = ep_data_buffer[EP_POS23_ADDRESS+1];
  pdata[HJEP_ENG_POS19_ADDRESS+2] = ep_data_buffer[EP_POS23_ADDRESS+2];
  pdata[HJEP_ENG_POS19_ADDRESS+3] = ep_data_buffer[EP_POS23_ADDRESS+3];
}

/*************************************************************************
 * HJ环保: 创建发动机数据信息体(补充信息)
 * 杭州环保是小端，HJ环保是大端
*************************************************************************/
void HJEP_BuildEngAddMessage(uint8_t* pdata)
{
  memcpy(pdata, &ep_data_buffer[EP_POS16_ADDRESS], SIZE_OF_HJEP_ENG_ADD);

  // 不支持字段填充为0xFF
  //if (CAN_GetEngineType() == ENGINE_TYPE_OTHER) // 杭发
  //{
  //  pdata[HJEP_ADD_POS7_ADDRESS] = 0xFF; // (DPF/POC)排气温度(B5:B6)
  //  pdata[HJEP_ADD_POS7_ADDRESS+1] = 0xFF;
  //}
}

//==信息类型标志+信息采集时间+信息体===================================================
uint16_t HJEP_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;

  pdata[len++] = HJEP_DATA_TYPE_ENG_SCR;// 信息类型标志--发动机数据

  rtc_date_t bj_time = HJEP_GetDataTime(); // 数据采集时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  HJEP_BuildEngMandMessage(&pdata[len]); // 发动机数据信息体
  len += SIZE_OF_HJEP_ENG_MAND;

  /*************************************************************/
  pdata[len++] = HJEP_DATA_TYPE_ADD;// 信息类型标志--发动机补充数据
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second; // 秒

  HJEP_BuildEngAddMessage(&pdata[len]);  // 发动机补充数据信息体
  len += SIZE_OF_HJEP_ENG_ADD;

  return len;
}

/*************************************************************************
 * 发送HJ环保帧--实时信息上报(发送机数据)
*************************************************************************/
int32_t HJEP_SendEngData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t msg_len;
  uint16_t data_size = 0; // 数据单元长度
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== HJ环保数据公共部分 =========================================
  retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_NEW_DATA, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元 ======================================================
  len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = HJEP_GetDataTime(); //数据发送时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  hjep_send_data_sn++; // 流水号加1
  pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// 信息流水号(2B)
  pdata[len++] = (uint8_t)hjep_send_data_sn;

  data_size = (6 + 2);
  
  msg_len = HJEP_BuildEngData(&pdata[len]); // 信息单元
  len += msg_len;
  data_size += msg_len;

  //== 数据单元长度 ==================================================
  pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== 校验码 =========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

//==信息类型标志+信息采集时间+信息体===================================================
uint16_t HJEP_BuildObdData(uint8_t* pdata)
{
  uint16_t len = 0;

  pdata[len++] = HJEP_DATA_TYPE_OBD;// 信息类型标志

  rtc_date_t bj_time = HJEP_GetDataTime(); //信息采集时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  //OBD信息体==================================
  pdata[len++] = obd_info.protocol_type; // OBD诊断协议(1B)  // OBD_PROTOCOL_ISO27145
  pdata[len++] = obd_info.mil_status;  // MIL状态(1B)
  pdata[len++] = (uint8_t)(obd_info.diag_supported_status>>8);// 诊断支持状态(2B)
  pdata[len++] = (uint8_t)obd_info.diag_supported_status;
  pdata[len++] = (uint8_t)(obd_info.diag_readiness_status>>8);// 诊断就绪状态(2B)
  pdata[len++] = (uint8_t)obd_info.diag_readiness_status;

  if (obd_info.vin_valid_flag)
  {
    memcpy(&pdata[len], obd_info.vin, 17);// 车辆识别号VIN(17B)
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[len], m2m_asset_data.vin, 17);// 车辆识别号VIN(17B)
  }
  len += 17;

  memcpy(&pdata[len],obd_info.calid,18);// 软件标定识别号(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.cvn,18);// 标定验证码(CVN)(18B)
  len += 18;

  memcpy(&pdata[len],obd_info.iupr,36);// IUPR值(36B)
  len += 36;

  pdata[len++] = obd_info.dtc_num;// 故障码总数(1B)
  OBD_GetDtcCode(&obd_info, &pdata[len]);  // 读取故障码到发送缓存
  len += (obd_info.dtc_num * 4);    // 故障码信息列表(N*4B)

  return len;
}

/*************************************************************************
 *  发送HJ环保帧--实时信息上报(OBD数据)
*************************************************************************/
int32_t HJEP_SendObdData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t msg_len;
  uint16_t data_size = 0; // 数据单元长度
  uint8_t* pdata = hjep_send_buffer;
  int32_t retVal;

  //== HJ环保数据公共部分 =========================================
  retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_NEW_DATA, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元 ======================================================
  len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = HJEP_GetDataTime(); //数据发送时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  hjep_send_data_sn++; // 流水号加1
  pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// 信息流水号(2B)
  pdata[len++] = (uint8_t)hjep_send_data_sn;
  data_size = (6 + 2);

  msg_len = HJEP_BuildObdData(&pdata[len]); // 信息单元
  len += msg_len;
  data_size += msg_len;

  //== 数据单元长度 ==================================================
  pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== 校验码 ========================================================
  pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = HJEP_SendNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  发送HJ环保帧--补发信息上报(发动机和OBD数据)
*************************************************************************/
int32_t HJEP_SendBzData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t data_size = 0; // 数据单元长度
  uint16_t stack_size = 0x00;
  uint16_t bind_zone_data_size = 0x00;
  uint8_t* pdata = hjep_send_buffer;;


  int32_t retVal = -1  stack_size = BlindZone_GetStackSize(&hjep_blind_zone);
  if (stack_size > 0)
  {
    //== HJ环保数据公共部分 =========================================
    retVal = HJEP_BuildMsgHead(HJEP_PACKET_CMD_BZ_DATA, pdata);
    if(retVal==-1)
    {
      return -1;
    }

    //== 数据单元 ======================================================
    len = SIZE_OF_HJEP_HEADER; // 数据单元起始地址
    rtc_date_t bj_time = HJEP_GetDataTime(); //数据发送时间(6B)
    pdata[len++] = bj_time.year; // 年
    pdata[len++] = bj_time.month;  // 月
    pdata[len++] = bj_time.day;  // 日
    pdata[len++] = bj_time.hour; // 时
    pdata[len++] = bj_time.minute;  // 分
    pdata[len++] = bj_time.second;  // 秒

    hjep_send_data_sn++; // 流水号加1
    pdata[len++] = (uint8_t)(hjep_send_data_sn>>8);// 信息流水号(2B)
    pdata[len++] = (uint8_t)hjep_send_data_sn;
    
    data_size = (6 + 2);

    /*******************************信息体(开始)*******************************/
    BlindZone_PopData(&hjep_blind_zone, &pdata[len], &bind_zone_data_size);
    HjepBlindZone_Save(); // 存入栈参数
    if (bind_zone_data_size == 0x00)
    {
#if HJEP_BZ_DEBUG
      PcDebug_Printf("HjepBzPop:Err\r\n");
#endif
      return retVal;
    }

    len += bind_zone_data_size;
    data_size += bind_zone_data_size;
    /*******************************信息体(结束)*******************************/

    //== 数据单元长度 ==================================================
    pdata[HJEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
    pdata[HJEP_POS6_ADDRESS+1] = (uint8_t)data_size;

    //== 校验码 =========================================================
    pdata[len] = HJEP_CalcXorCheck(&pdata[HJEP_POS2_ADDRESS],(len-2));  //校验码(1B)
    len++;

    retVal = HJEP_SendNetData(pdata, len);

#if HJEP_BZ_DEBUG
    PcDebug_Printf("HjepBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt,hjep_blind_zone.top,hjep_blind_zone.bottom);
#endif
  }

  return retVal;
}

#if (PART("HJ环保盲区补偿"))
/******************************************************************************
* 补发重型环保盲区数据
******************************************************************************/
void HjepBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_HJEP_BZ_PARA_ADDR, &hjep_blind_zone_para); // 读取FLASH中的参数
  hjep_blind_zone.wr_error_cnt = hjep_blind_zone_para.wr_error_cnt;
  hjep_blind_zone.rd_error_cnt = hjep_blind_zone_para.rd_error_cnt;
  hjep_blind_zone.top = hjep_blind_zone_para.top;
  hjep_blind_zone.bottom = hjep_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    hjep_blind_zone.data[i] = hjep_blind_zone_para.data[i];
  }

  hjep_blind_zone.timer_100ms = HJEP_BZ_SAVE_PERIOD_SP;
  hjep_blind_zone.file_name = FILE_HJEP_BZ_DATA_ADDR; // 文件名
  hjep_blind_zone.frame_size = HJEP_BLIND_ZONE_PACKET_SIZE; // 数据帧固定长度
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

//==100ms调用一次(环保开启且ACC开、有下车数据和VIN码有效下调用)============================
void HjepBlindZone_Service(void)
{
  static uint8_t obd_data_report_timer = 20;
  uint8_t acc_state;
  uint16_t engine_speed = 0x00;

  acc_state = COLT_GetAccStatus();
  engine_speed = CAN_GetEngineSpeed();
  if (acc_state==1)  // ACC开、有下车数据和VIN码有效
  {
    if (hjep_blind_zone.timer_100ms)
      hjep_blind_zone.timer_100ms--;
    else
    {
      hjep_blind_zone.timer_100ms = HJEP_BZ_SAVE_PERIOD_SP; // 1分钟一条
      if (HJEP_GetLinkState() != SOCKET_LINK_STATE_READY) // 未连接
      {
        if ((GPS_GetPositioningStatus()==1) && (engine_speed>300)) // 终端已定位且发动机启动
        {
          memset(hjep_blind_zone_buffer, 0xFF, HJEP_BLIND_ZONE_PACKET_SIZE); // 清空缓存
          if (obd_data_report_timer)
          {
            obd_data_report_timer--;
            hjep_blind_zone_length = HJEP_BuildEngData(hjep_blind_zone_buffer); // 发动机信息
          }
          else
          {
            obd_data_report_timer = 20;
            hjep_blind_zone_length = HJEP_BuildObdData(hjep_blind_zone_buffer); // OBD信息
          }

          if (hjep_blind_zone_length <= HJEP_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&hjep_blind_zone, hjep_blind_zone_buffer, hjep_blind_zone_length); // 存入数据帧
            HjepBlindZone_Save(); // 存入栈参数
#if HJEP_BZ_DEBUG
            PcDebug_Printf("HjepBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",hjep_blind_zone.wr_error_cnt,hjep_blind_zone.rd_error_cnt,hjep_blind_zone.top,hjep_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC关闭
  {
    obd_data_report_timer = 20;
    hjep_blind_zone.timer_100ms = HJEP_BZ_SAVE_PERIOD_SP; // 不存储
  }

  // 出现读写错误,复位栈为0
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
    HjepBlindZone_Save(); // 存入栈参数
  }
}
#endif

#define HJEP_SEND_LOGIN_DATA_TIME_SP  3000
/*************************************************************************
* HJ环保状态机,100ms调用一次(无阻塞)
*************************************************************************/
void HJEP_StateMachine(void)
{
  static uint16_t hjep_send_login_data_time = 50;
  static uint8_t hjep_send_bz_data_time = 50;
  static uint16_t hjep_send_eng_data_time = HJEP_SEND_ENG_DATA_TIME_SP;
  static uint16_t hjep_send_obd_data_time = HJEP_SEND_OBD_DATA_TIME_SP;
  static uint16_t hjep_send_hb_time;
  uint8_t vin_valid_flag;
  uint8_t ep_data_valid_flag;
  uint8_t acc_state;
  uint16_t engine_speed = 0x00;
  static uint8_t hjep_socket_enable_flag = FALSE;

  if (CAN_GetEpType()==EP_TYPE_HJ) // 环保功能开启
  {
    vin_valid_flag = CAN_GetVinState();
    ep_data_valid_flag = CAN_GetEpDataState();
    acc_state = COLT_GetAccStatus();
    engine_speed = CAN_GetEngineSpeed();
    
    // ACC开、有下车数据和VIN码有效
    if ((acc_state==1) && (ep_data_valid_flag==1) && (vin_valid_flag==1))
    {
      HjepBlindZone_Service();  // 记录盲区数据
    }

    // VIN码有效就可联网,用于发送登录和登出指令
    if (vin_valid_flag==1)
    {
      HJEP_EnableLink(); // 使能hjep连接
      hjep_socket_enable_flag = TRUE;
    }
    else
    {
      HJEP_DisableLink(); // 禁止hjep连接
      hjep_socket_enable_flag = FALSE;
    }

    if (HJEP_GetLinkState() != SOCKET_LINK_STATE_READY) // 等待网络空闲
    {
      return;
    }

    // VIN码有效,发送车辆登录
    if (HJEP_SEND_LOGIN_FLAG==0) // 车辆未登录
    {
      if((hjep_send_login_data_time==0) || (acc_state==1))
      {
        hjep_send_login_data_time = HJEP_SEND_LOGIN_DATA_TIME_SP; // 重发登录指令时间
        hjep_state = HJEP_STATE_SEND_LOGIN;
        
        hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
        hjep_send_eng_data_time = 50; // 登录后,5秒发送数据
        hjep_send_obd_data_time = HJEP_SEND_OBD_DATA_TIME_SP;
        hjep_send_bz_data_time = 80;
        return;
      }
    }
    else  // 车辆已登录
    {
      // ACC=OFF,发送车辆登出
      if (acc_state==0) // ACC关
      {
        hjep_send_login_data_time = HJEP_SEND_LOGIN_DATA_TIME_SP; // 重发登录指令时间
        hjep_state = HJEP_STATE_SEND_LOGOUT;
        return;
      }

      // 车辆登录30s后,发送NTP数据
      if (hjep_send_hb_time==0x00)
      {
        hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
        hjep_state = HJEP_STATE_SEND_NTP;
        return;
      }

      // 发动机启动30s后,发送OBD数据
      if (HJEP_SEND_OBD_FLAG==0 && (hjep_send_obd_data_time==0))
      {
        if (engine_speed > 300) // 下车发动机发动
        {
          hjep_send_obd_data_time = HJEP_SEND_ENG_DATA_TIME_SP;
          hjep_state = HJEP_STATE_SEND_OBD;
          return;
        }
      }

      // 发动机启动,发送发动机数据
      if (hjep_send_eng_data_time==0)
      {
        hjep_send_eng_data_time = HJEP_SEND_ENG_DATA_TIME_SP;  // 重置数据发送时间
        hjep_send_bz_data_time = HJEP_SEND_BZ_DATA_TIME_SP;  // 重置盲区发送时间
        hjep_state = HJEP_STATE_SEND_ENG;
        return;
      }

      // 补发信息上报
      if(hjep_send_bz_data_time==0)
      {
        hjep_send_bz_data_time = HJEP_SEND_BZ_DATA_TIME_SP;  // 重置盲区发送时间
        hjep_state = HJEP_STATE_SEND_BZ;
        return;
      }
    }// end if (HJEP_SEND_LOGIN_FLAG==1)
  }
  else // 环保功能未开启
  {
    hjep_state = HJEP_STATE_SEND_INIT;
    hjep_flags1.byte = 0x00;
    HJEP_DisableLink(); // 禁止bjep套接字连接
    hjep_socket_enable_flag = FALSE;
  }

  // 数据发送时间和心跳时间倒计
  if (hjep_socket_enable_flag == TRUE)
  {
    if (engine_speed > 300) // 下车发动机发动(OBD获取需要时间)
    {
      if(hjep_send_obd_data_time)
        hjep_send_obd_data_time--;
    }

    if (hjep_send_eng_data_time) // 数据发送定时器
      hjep_send_eng_data_time--;

    if (hjep_send_hb_time) // 心跳发送定时器
      hjep_send_hb_time--;

    if(hjep_send_bz_data_time) // 补发数据定时器
      hjep_send_bz_data_time--;

    if(hjep_send_login_data_time) // 登录指令
      hjep_send_login_data_time--;
  }
  else
  {
    hjep_send_eng_data_time = HJEP_SEND_ENG_DATA_TIME_SP;
    hjep_send_obd_data_time = HJEP_SEND_OBD_DATA_TIME_SP;
    hjep_send_hb_time = HJEP_SEND_HB_TIME_SP;
    hjep_send_login_data_time = HJEP_SEND_LOGIN_DATA_TIME_SP;
  }
}

/*************************************************************************
 * 发送HJ环保帧(10ms调用一次)
*************************************************************************/
void HJEP_ProduceSendData(void)
{
  int32_t retVal;
  static uint8_t divide_for_100ms = 9;

  if (divide_for_100ms)
  {
    divide_for_100ms--;
  }
  else
  {
    divide_for_100ms = 9;  // 100ms任务
    HJEP_StateMachine();
  }

  switch (hjep_state)
  {
  case HJEP_STATE_SEND_INIT: // 初始化登录流水号和数据流水号
    // hjep_login_sn = 0;
    // hjep_send_data_sn = 0;
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_IDLE: // 空闲状态
    break;

  case HJEP_STATE_SEND_LOGIN: // 车辆登录
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

  case HJEP_STATE_SEND_NTP: // 终端校时
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

  case HJEP_STATE_SEND_OBD: // 实时信息上报(OBD)
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

  case HJEP_STATE_SEND_ENG: // 实时信息上报(发动机)
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

  case HJEP_STATE_SEND_BZ: // 补发信息上报
    retVal = HJEP_SendBzData();
    if (retVal==0)
    {
#if HJEP_DEBUG
      PcDebug_SendString("HJEP:BZ!\r\n");
#endif
      HJEP_SEND_BZ_FLAG = 1;
    }
    hjep_state = HJEP_STATE_SEND_IDLE;
    break;

  case HJEP_STATE_SEND_LOGOUT: // 车辆登出
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
* 处理HJEP环保服务器(10012)下行数据
*******************************************************************************/
void HJEP_ProcessRecvData(uint8_t* pdata, uint16_t len)
{
#if 0
  if((pdata[0]==0x23) && (pdata[1]==0x23)) // 帧头
  {
    hjep_socket.error_cnt = 0x00; // 清除错误计数
    hjep_socket.error_flag = FALSE;
  }
#endif
}

/******************************************************************************
* 初始化参数
*******************************************************************************/
void HJEP_Initialize(void)
{
  HjepBlindZone_Init();  // 盲区补偿(补发数据)

  hjep_state = HJEP_STATE_SEND_INIT;
  hjep_flags1.byte = 0x00;
  //HJEP_DisableLink();  // 禁止hjep套接字连接
}

