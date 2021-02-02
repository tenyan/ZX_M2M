/*****************************************************************************
* @FileName: GB.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2021-1-25
* @brief:    中国国家标准GB17691-2018(重型柴油车远程排放监控技术规范)实现.C文件
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
// 网络接口定义
#define gbep_send_buffer              gbep_data_buffer

#define GBEP_GetMainLinkState()           NetSocket_GetLinkState(&gbep_socket)
#define GBEP_DisableMainLink()            NetSocket_Disable(&gbep_socket)
#define GBEP_EnableMainLink()             NetSocket_Enable(&gbep_socket)
#define GBEP_SendMainNetData(pData, len)  NetSocket_Send(&gbep_socket, pData, len)  // 发到北京市环保平台

#define GBEP_GetSubLinkState()           NetSocket_GetLinkState(&bjep_socket)
#define GBEP_DisableSubLink()            NetSocket_Disable(&bjep_socket)
#define GBEP_EnableSubLink()             NetSocket_Enable(&bjep_socket)
#define GBEP_SendSubNetData(pData, len)  NetSocket_Send(&bjep_socket, pData, len)  // 发到重型环保平台

#define GBEP_DisableLink()            do{GBEP_DisableMainLink();GBEP_DisableSubLink();}while(0)
#define GBEP_EnableLink()             do{GBEP_EnableMainLink();GBEP_EnableSubLink();}while(0)

// GB数据发送周期定义
#define GBEP_SEND_ENG_DATA_TIME_SP    90   //发送周期10秒
#define GBEP_SEND_OBD_DATA_TIME_SP    190   //发送周期20秒

#define GBEP_SEND_HB_TIME_SP          1200  //发送周期3分钟
#define GBEP_SEND_BZ_DATA_TIME_SP     14    //发送周期1.5秒

// 盲区补发
#define GBEP_BZ_SAVE_PERIOD_SP  299 // 30s
#define GBEP_BLIND_ZONE_PACKET_SIZE  512
#define GBEP_BDZE_WRITE_ERROR_SP  6
#define GBEP_BDZE_READ_ERROR_SP   6

/******************************************************************************
* Data Types and Globals
******************************************************************************/
static uint8_t gbep_data_buffer[1024];

// 双缓存(时间和发动机数据)
static uint8_t gbep_eng_packet_buffer[GBEP_ENG_PACKET_MAX_NUM][SIZE_OF_GBEP_ENG]; // 发动机数据缓存
static uint8_t gbep_eng_data[GBEP_ENG_PACKET_MAX_NUM][SIZE_OF_GBEP_ENG];
static uint8_t gbep_eng_time_buffer[SIZE_OF_GBEP_ENG_TIME]; // 数据采集时间
static uint8_t gbep_eng_time[SIZE_OF_GBEP_ENG_TIME];
static uint8_t gbep_eng_data_valid_flag = 0;

// GB状态
volatile bittype gbep_flags1;
gbep_state_t gbep_state = GBEP_STATE_SEND_INIT;

// 流水号
uint16_t gbep_login_sn;
uint16_t gbep_send_data_sn;

// 盲区补偿对象生命
blind_zone_t gbep_blind_zone;
blind_zone_para_t gbep_blind_zone_para;

// 缓存
static uint8_t gbep_blind_zone_buffer[GBEP_BLIND_ZONE_PACKET_SIZE];
static uint16_t gbep_blind_zone_length;

void GbepBlindZone_Save(void);

/******************************************************************************
 * 获取时间,用于环保上传
 ******************************************************************************/
rtc_date_t GBEP_GetDataTime(void)
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
 * 构建包头
*************************************************************************/
int32_t GBEP_BuildMsgHead(uint8_t cmdType, uint8_t *pdata)
{
  //== GB环保数据公共部分 =========================================
  pdata[GBEP_POS1_ADDRESS] = GBEP_PACKET_HEADER1; // 起始符
  pdata[GBEP_POS1_ADDRESS+1] = GBEP_PACKET_HEADER2;
  pdata[GBEP_POS2_ADDRESS] = cmdType; // 命令单元
  if (obd_info.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],obd_info.vin,17);// 车辆识别号VIN
  }
  else if (m2m_asset_data.vin_valid_flag)
  {
    memcpy(&pdata[GBEP_POS3_ADDRESS],m2m_asset_data.vin,17);// 车辆识别号VIN
  }
  else
  {
    return -1;
  }
  pdata[GBEP_POS4_ADDRESS] = GBEP_PACKET_SW_VERSION; // 终端软件版本号
  pdata[GBEP_POS5_ADDRESS] = GBEP_PACKET_ENCRYPT_NONE; // 数据加密方式
  
  return 0;
}

/*************************************************************************
 * 发送GB环保帧--车辆登入
*************************************************************************/
int32_t GBEP_SendLogin(void)
{
  uint16_t len;     // 整个数据帧的长度
  uint8_t* pdata = gbep_send_buffer;
  uint8_t* p_iccid;
  int32_t retVal;

  //== GB环保数据公共部分 =========================================
  retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_LOGIN, pdata);
  if(retVal==-1)
  {
    return -1;
  }
  
  //== 数据单元长度 ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 28;

  //== 数据单元 ======================================================
  len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = GBEP_GetDataTime(); //登录时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  gbep_login_sn++; // 流水号加1
  pdata[len++] = (uint8_t)(gbep_login_sn>>8);// 登入流水号(2B)
  pdata[len++] = (uint8_t)gbep_login_sn;

  p_iccid = Cellura_GetIccid();
  memcpy(&pdata[len],p_iccid,20); // SIM卡号(ICCID号)(20B)
  len += 20;

  //== 校验码 =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = GBEP_SendMainNetData(pdata, len);
  GBEP_SendSubNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * 发送GB环保帧--车辆登出
*************************************************************************/
int32_t GBEP_SendLogout(void)
{
  uint16_t len;     // 整个数据帧的长度
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB环保数据公共部分 =========================================
  retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_LOGOUT, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元长度 ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 0x08;

  //== 数据单元 ======================================================
  len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
  rtc_date_t bj_time = GBEP_GetDataTime(); //登出时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  // 登出流水号与当次登入流水号一致
  pdata[len++] = (uint8_t)(gbep_login_sn>>8);// 登出流水号(2B)
  pdata[len++] = (uint8_t)gbep_login_sn;

  //== 校验码 =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = GBEP_SendMainNetData(pdata, len);
  GBEP_SendSubNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * 发送GB环保帧--终端校时
*************************************************************************/
int32_t GBEP_SendNtpData(void)
{
  uint16_t len;     // 整个数据帧的长度
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB环保数据公共部分 =========================================
  retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_NTP, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元长度 ==================================================
  pdata[GBEP_POS6_ADDRESS] = 0x00;
  pdata[GBEP_POS6_ADDRESS+1] = 0x00;

  //== 数据单元 ======================================================
  len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
  // 车载终端校时的数据单元为空

  //== 校验码 =========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = GBEP_SendMainNetData(pdata, len);
  GBEP_SendSubNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 * GB环保: 创建发动机数据信息体(必须信息)
 * 杭州环保是小端，GB环保是大端
*************************************************************************/
void GBEP_BuildEngMessage(uint8_t* pdata)
{
  int32_t lLongitude;
  int32_t lLatitude;
  uint8_t ew_flag;
  uint8_t ns_flag;

  //=============================================================
  memcpy(pdata, ep_data_buffer, SIZE_OF_GBEP_ENG);

  // 后处理上游氮氧浓度
  //if (pdata[GBEP_ENG_POS7_ADDRESS]==0x00 && pdata[GBEP_ENG_POS7_ADDRESS+1]==0x00)
  //{
  //  pdata[GBEP_ENG_POS7_ADDRESS] = 0xFF;
  //  pdata[GBEP_ENG_POS7_ADDRESS+1] = 0xFF;
  //}

  // 不支持字段填充为0xFF
  //if (CAN_GetEngineType() == ENGINE_TYPE_OTHER) // 杭发
 // {
 //   pdata[GBEP_ENG_POS7_ADDRESS] = 0xFF; // 后处理上游氮氧浓度(B1:B2)
  //  pdata[GBEP_ENG_POS7_ADDRESS+1] = 0xFF;

 //   pdata[GBEP_ENG_POS13_ADDRESS] = 0xFF; // (DPF/POC)载体压差(B4:B5)
  //  pdata[GBEP_ENG_POS13_ADDRESS+1] = 0xFF;
  //}

  //=============================================================
  ew_flag = GPS_GetEastWest();  //经度半球:1=东经, 0=西经
  ns_flag = GPS_GetNorthSouth();  // 纬度半球:1=北纬, 0=南纬
  if (GPS_GetPositioningStatus()==0) // 终端不定位
  {
    //地位无效,发送最后一次有效定位信息
    //lLongitude = 0xFFFFFFFF;
    //lLatitude = 0xFFFFFFFF;
    pdata[GBEP_ENG_POS16_ADDRESS] |= 0x01;  // 定位状态(1B):1-定位无效
  }
  else
  {
    pdata[GBEP_ENG_POS16_ADDRESS] = 0;  // 定位状态(1B):0-定位有效
    if (ns_flag == 0) // 南纬
    {
      pdata[GBEP_ENG_POS16_ADDRESS] |= 0x02;
    }
    if (ew_flag == 0) // 西经
    {
      pdata[GBEP_ENG_POS16_ADDRESS] |= 0x04;
    }
  }
  // 经度(4B)
  lLongitude = GPS_GetLongitude();  // 经度,单位:百万分之一度
  pdata[GBEP_ENG_POS17_ADDRESS] = (uint8_t)(lLongitude >> 24);
  pdata[GBEP_ENG_POS17_ADDRESS+1] = (uint8_t)(lLongitude >> 16);
  pdata[GBEP_ENG_POS17_ADDRESS+2] = (uint8_t)(lLongitude >> 8);
  pdata[GBEP_ENG_POS17_ADDRESS+3] = (uint8_t)(lLongitude & 0xFF);
  // 纬度(4B)
  lLatitude = GPS_GetLatitude();    // 纬度,单位:百万分之一度
  pdata[GBEP_ENG_POS18_ADDRESS] = (uint8_t)(lLatitude >> 24);
  pdata[GBEP_ENG_POS18_ADDRESS+1] = (uint8_t)(lLatitude >> 16);
  pdata[GBEP_ENG_POS18_ADDRESS+2] = (uint8_t)(lLatitude >> 8);
  pdata[GBEP_ENG_POS18_ADDRESS+3] = (uint8_t)(lLatitude & 0xFF);
  // 总行驶里程(4B)
  pdata[GBEP_ENG_POS19_ADDRESS] = ep_data_buffer[EP_POS23_ADDRESS];
  pdata[GBEP_ENG_POS19_ADDRESS+1] = ep_data_buffer[EP_POS23_ADDRESS+1];
  pdata[GBEP_ENG_POS19_ADDRESS+2] = ep_data_buffer[EP_POS23_ADDRESS+2];
  pdata[GBEP_ENG_POS19_ADDRESS+3] = ep_data_buffer[EP_POS23_ADDRESS+3];
}

/*************************************************************************
* 返回: 1-已缓存完整数据帧, 0-数据帧未完整
*************************************************************************/
void GBEP_CacheEngMessage(void)
{
  static uint8_t gbep_eng_packet_index = 0; // 发动机数据缓存序号
  uint8_t acc_state;
  
  acc_state = COLT_GetAccStatus();
  if(acc_state==0)
  {
    gbep_eng_packet_index = 0;
    return;
  }
  
  if (gbep_eng_packet_index==0) // 第一条数据,记录时间
  {
    rtc_date_t bj_time = GBEP_GetDataTime(); // 数据采集时间(6B)
    gbep_eng_time_buffer[0] = bj_time.year; // 年
    gbep_eng_time_buffer[1] = bj_time.month;  // 月
    gbep_eng_time_buffer[2] = bj_time.day;  // 日
    gbep_eng_time_buffer[3] = bj_time.hour; // 时
    gbep_eng_time_buffer[4] = bj_time.minute;  // 分
    gbep_eng_time_buffer[5] = bj_time.second;  // 秒
  }
  
  if(gbep_eng_packet_index < GBEP_ENG_PACKET_MAX_NUM)
  {
    GBEP_BuildEngMessage(&gbep_eng_packet_buffer[gbep_eng_packet_index][0]); // 缓存发动机数据 
    gbep_eng_packet_index++; // 序号累计
    if (gbep_eng_packet_index==GBEP_ENG_PACKET_MAX_NUM) // 完整一帧需要10条发动机数据
    {
      gbep_eng_packet_index = 0; // 清零序号
      memcpy(gbep_eng_time, gbep_eng_time_buffer, SIZE_OF_GBEP_ENG_TIME); // 拷贝时间
      memcpy(gbep_eng_data, gbep_eng_packet_buffer, (GBEP_ENG_PACKET_MAX_NUM*SIZE_OF_GBEP_ENG)); // 拷贝数据
      gbep_eng_data_valid_flag = 1; // 数据有效
    }
  }
}

//==信息采集时间+{(信息类型标志+流水号+信息体)*10}=======================================
uint16_t GBEP_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;
  uint8_t it;
  
  //== 信息采集时间======================================================
  pdata[len++] = gbep_eng_time[0]; // 年
  pdata[len++] = gbep_eng_time[1]; // 月
  pdata[len++] = gbep_eng_time[2]; // 日
  pdata[len++] = gbep_eng_time[3]; // 时
  pdata[len++] = gbep_eng_time[4]; // 分
  pdata[len++] = gbep_eng_time[5]; // 秒
  for (it=0; it<GBEP_ENG_PACKET_MAX_NUM; it++) // 发动机数据
  {
    gbep_send_data_sn++;  // 流水号加1
    
    pdata[len++] = GBEP_DATA_TYPE_ENG_SCR;// 信息类型标志--发动机数据
    pdata[len++] = (uint8_t)(gbep_send_data_sn>>8);// 信息流水号(2B)
    pdata[len++] = (uint8_t)gbep_send_data_sn;
    memcpy(&pdata[len], &gbep_eng_data[it][0], SIZE_OF_GBEP_ENG); // 发动机数据信息体
    
    len += SIZE_OF_GBEP_ENG;
  }
  
  return len;
}

/*************************************************************************
 * 发送GB环保帧--实时信息上报(发送机数据)
*************************************************************************/
int32_t GBEP_SendEngData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t msg_len;
  uint16_t data_size = 0; // 数据单元长度
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal = -1;

  if (gbep_eng_data_valid_flag==1) // 数据有效
  {
    gbep_eng_data_valid_flag = 0;

    //== GB环保数据公共部分 =========================================
    retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_NEW_DATA, pdata);
    if(retVal==-1)
    {
      return -1;
    }

    //== 数据单元 ======================================================
    len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
    msg_len = GBEP_BuildEngData(&pdata[len]);
    len += msg_len;
    data_size = msg_len;

    //== 数据单元长度 ==================================================
    pdata[GBEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
    pdata[GBEP_POS6_ADDRESS+1] = (uint8_t)data_size;

    //== 校验码 =========================================================
    pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
    len++;

    retVal = GBEP_SendMainNetData(pdata, len);
    GBEP_SendSubNetData(pdata, len);

  }

  return retVal;
}

//==信息采集时间+信息类型标志+流水号+信息体========================================
uint16_t GBEP_BuildObdData(uint8_t* pdata)
{
  uint16_t len = 0;
  
  rtc_date_t bj_time = GBEP_GetDataTime(); //信息采集时间(6B)
  pdata[len++] = bj_time.year; // 年
  pdata[len++] = bj_time.month;  // 月
  pdata[len++] = bj_time.day;  // 日
  pdata[len++] = bj_time.hour; // 时
  pdata[len++] = bj_time.minute;  // 分
  pdata[len++] = bj_time.second;  // 秒

  pdata[len++] = GBEP_DATA_TYPE_OBD;// 信息类型标志
  
  gbep_send_data_sn++; // 流水号加1
  pdata[len++] = (uint8_t)(gbep_send_data_sn>>8);// 信息流水号(2B)
  pdata[len++] = (uint8_t)gbep_send_data_sn;

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
 * 发送GB环保帧--实时信息上报(OBD数据)
*************************************************************************/
int32_t GBEP_SendObdData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t msg_len;
  uint16_t data_size = 0; // 数据单元长度
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal;

  //== GB环保数据公共部分 =========================================
  retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_NEW_DATA, pdata);
  if(retVal==-1)
  {
    return -1;
  }

  //== 数据单元 ======================================================
  len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
  msg_len = GBEP_BuildObdData(&pdata[len]); // 创建数据单元
  len += msg_len;
  data_size = msg_len;

  //== 数据单元长度 ==================================================
  pdata[GBEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
  pdata[GBEP_POS6_ADDRESS+1] = (uint8_t)data_size;

  //== 校验码 ========================================================
  pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
  len++;

  retVal = GBEP_SendMainNetData(pdata, len);
  GBEP_SendSubNetData(pdata, len);

  return retVal;
}

/*************************************************************************
 *  发送GB环保帧--补发信息上报(发动机和OBD数据)
*************************************************************************/
int32_t GBEP_SendBzData(void)
{
  uint16_t len = 0;     // 整个数据帧的长度
  uint16_t data_size = 0; // 数据单元长度
  uint16_t stack_size = 0x00;
  uint16_t bind_zone_data_size = 0x00;
  uint8_t* pdata = gbep_send_buffer;
  int32_t retVal = -1;

  stack_size = BlindZone_GetStackSize(&gbep_blind_zone);
  if (stack_size > 0)
  {
    //== HJ环保数据公共部分 =========================================
    retVal = GBEP_BuildMsgHead(GBEP_PACKET_CMD_BZ_DATA, pdata);
    if(retVal==-1)
    {
      return -1;
    }

    //== 数据单元 ======================================================
    len = SIZE_OF_GBEP_HEADER; // 数据单元起始地址
    rtc_date_t bj_time = GBEP_GetDataTime(); //数据发送时间(6B)
    pdata[len++] = bj_time.year; // 年
    pdata[len++] = bj_time.month;  // 月
    pdata[len++] = bj_time.day;  // 日
    pdata[len++] = bj_time.hour; // 时
    pdata[len++] = bj_time.minute;  // 分
    pdata[len++] = bj_time.second;  // 秒

    gbep_send_data_sn++; // 流水号加1
    pdata[len++] = (uint8_t)(gbep_send_data_sn>>8);// 信息流水号(2B)
    pdata[len++] = (uint8_t)gbep_send_data_sn;
    
    data_size = (6 + 2);

    /*******************************信息体(开始)*******************************/
    BlindZone_PopData(&gbep_blind_zone, &pdata[len], &bind_zone_data_size);
    GbepBlindZone_Save(); // 存入栈参数
    if (bind_zone_data_size == 0x00)
    {
#if GBEP_BZ_DEBUG
      PcDebug_Printf("GbepBzPop:Err\r\n");
#endif
      return retVal;
    }

    len += bind_zone_data_size;
    data_size += bind_zone_data_size;
    /*******************************信息体(结束)*******************************/

    //== 数据单元长度 ==================================================
    pdata[GBEP_POS6_ADDRESS] = (uint8_t)(data_size >> 8);
    pdata[GBEP_POS6_ADDRESS+1] = (uint8_t)data_size;

    //== 校验码 =========================================================
    pdata[len] = GBEP_CalcXorCheck(&pdata[GBEP_POS2_ADDRESS],(len-2));  //校验码(1B)
    len++;

    retVal = GBEP_SendMainNetData(pdata, len);
    GBEP_SendSubNetData(pdata, len);

#if GBEP_BZ_DEBUG
    PcDebug_Printf("GbepBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",gbep_blind_zone.wr_error_cnt,gbep_blind_zone.rd_error_cnt,gbep_blind_zone.top,gbep_blind_zone.bottom);
#endif
  }

  return retVal;
}

#if (PART("HJ环保盲区补偿"))
/******************************************************************************
* 补发重型环保盲区数据
******************************************************************************/
void GbepBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_GBEP_BZ_PARA_ADDR, &gbep_blind_zone_para); // 读取FLASH中的参数
  gbep_blind_zone.wr_error_cnt = gbep_blind_zone_para.wr_error_cnt;
  gbep_blind_zone.rd_error_cnt = gbep_blind_zone_para.rd_error_cnt;
  gbep_blind_zone.top = gbep_blind_zone_para.top;
  gbep_blind_zone.bottom = gbep_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    gbep_blind_zone.data[i] = gbep_blind_zone_para.data[i];
  }

  gbep_blind_zone.timer_100ms = GBEP_BZ_SAVE_PERIOD_SP;
  gbep_blind_zone.file_name = FILE_GBEP_BZ_DATA_ADDR; // 文件名
  gbep_blind_zone.frame_size = GBEP_BLIND_ZONE_PACKET_SIZE; // 数据帧固定长度
  pthread_mutex_init(&gbep_blind_zone.file_mutex, NULL);

#if GBEP_BZ_DEBUG
  PcDebug_Printf("GbepBzInit:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",gbep_blind_zone.wr_error_cnt,gbep_blind_zone.rd_error_cnt,gbep_blind_zone.top,gbep_blind_zone.bottom);
#endif
}

//=====================================================================================
void GbepBlindZone_Save(void)
{
  uint16_t i;

  pthread_mutex_lock(&gbep_blind_zone.file_mutex);

  gbep_blind_zone_para.wr_error_cnt = gbep_blind_zone.wr_error_cnt;
  gbep_blind_zone_para.rd_error_cnt = gbep_blind_zone.rd_error_cnt;
  gbep_blind_zone_para.top = gbep_blind_zone.top;
  gbep_blind_zone_para.bottom = gbep_blind_zone.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    gbep_blind_zone_para.data[i] = gbep_blind_zone.data[i];
  }
  BlindZone_SaveParameter(FILE_GBEP_BZ_PARA_ADDR, &gbep_blind_zone_para);

  pthread_mutex_unlock(&gbep_blind_zone.file_mutex);
}

//==100ms调用一次(环保开启且ACC开、有下车数据和VIN码有效下调用)============================
void GbepBlindZone_Service(void)
{
  static uint8_t obd_data_report_timer = 20;
  uint8_t acc_state;
  uint16_t engine_speed = 0x00;

  acc_state = COLT_GetAccStatus();
  engine_speed = CAN_GetEngineSpeed();
  if (acc_state==1)  // ACC开、有下车数据和VIN码有效
  {
    if (gbep_blind_zone.timer_100ms)
      gbep_blind_zone.timer_100ms--;
    else
    {
      gbep_blind_zone.timer_100ms = GBEP_BZ_SAVE_PERIOD_SP; // 1分钟一条
      if (GBEP_GetMainLinkState() != SOCKET_LINK_STATE_READY) // 未连接
      {
        if ((GPS_GetPositioningStatus()==1) && (engine_speed>300)) // 终端已定位且发动机启动
        {
          memset(gbep_blind_zone_buffer, 0xFF, GBEP_BLIND_ZONE_PACKET_SIZE); // 清空缓存
          if (obd_data_report_timer)
          {
            obd_data_report_timer--;
            gbep_blind_zone_length = GBEP_BuildEngData(gbep_blind_zone_buffer); // 发动机信息
          }
          else
          {
            obd_data_report_timer = 20;
            gbep_blind_zone_length = GBEP_BuildObdData(gbep_blind_zone_buffer); // OBD信息
          }

          if (gbep_blind_zone_length <= GBEP_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&gbep_blind_zone, gbep_blind_zone_buffer, gbep_blind_zone_length); // 存入数据帧
            GbepBlindZone_Save(); // 存入栈参数
#if GBEP_BZ_DEBUG
            PcDebug_Printf("GbepBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",gbep_blind_zone.wr_error_cnt,gbep_blind_zone.rd_error_cnt,gbep_blind_zone.top,gbep_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC关闭
  {
    obd_data_report_timer = 20;
    gbep_blind_zone.timer_100ms = GBEP_BZ_SAVE_PERIOD_SP; // 不存储
  }

  // 出现读写错误,复位栈为0
  if ((gbep_blind_zone.rd_error_cnt > GBEP_BDZE_READ_ERROR_SP) || (gbep_blind_zone.wr_error_cnt > GBEP_BDZE_WRITE_ERROR_SP))
  {
#if GBEP_BZ_DEBUG
    PcDebug_Printf("GbepBzErr:Ewr=%d,Erd=%d\r\n",gbep_blind_zone.wr_error_cnt,gbep_blind_zone.rd_error_cnt);
#endif
    gbep_blind_zone.wr_error_cnt = 0;
    gbep_blind_zone.rd_error_cnt = 0;
    gbep_blind_zone.top = 0;
    gbep_blind_zone.bottom = 0;
    memset(gbep_blind_zone.data, 0x00, BLIND_ZONE_STACK_MAX_SIZE);
    GbepBlindZone_Save(); // 存入栈参数
  }
}
#endif

#define GBEP_SEND_LOGIN_DATA_TIME_SP  3000
/*************************************************************************
* GB环保状态机,100ms调用一次(无阻塞)
*************************************************************************/
void GBEP_StateMachine(void)
{
  static uint8_t gbep_send_bz_data_time = 50;
  static uint16_t gbep_send_login_data_time = 50;
  static uint16_t gbep_send_eng_data_time = GBEP_SEND_ENG_DATA_TIME_SP;
  static uint16_t gbep_send_obd_data_time = GBEP_SEND_OBD_DATA_TIME_SP;
  static uint16_t gbep_send_hb_time;
  uint8_t vin_valid_flag;
  uint8_t ep_data_valid_flag;
  uint8_t acc_state;
  uint16_t engine_speed = 0x00;
  static uint8_t gbep_socket_enable_flag = FALSE;

  if (CAN_GetEpType()==EP_TYPE_GB) // 环保功能开启
  {
    vin_valid_flag = CAN_GetVinState();
    acc_state = COLT_GetAccStatus();
    ep_data_valid_flag = CAN_GetEpDataState();
    engine_speed = CAN_GetEngineSpeed();
    
    // ACC开、有下车数据和VIN码有效
    if ((acc_state==1) && (ep_data_valid_flag==1) && (vin_valid_flag==1))
    {
      GBEP_CacheEngMessage();  // 缓冲数据帧
      GbepBlindZone_Service();  // 记录盲区数据
    }

   if (vin_valid_flag==1)
    {
      GBEP_EnableLink(); // 使能gbep连接
      gbep_socket_enable_flag = TRUE;
    }
    else // ACC关
    {
      GBEP_DisableLink(); // 禁止gbep连接
      gbep_socket_enable_flag = FALSE;
    }

    if (GBEP_GetMainLinkState() != SOCKET_LINK_STATE_READY) // 等待网络空闲
    {
      return;
    }

    // ACC=ON,发送车辆登录
    if (GBEP_SEND_LOGIN_FLAG==0) // 车辆未登录
    {
      if((gbep_send_login_data_time==0) || (acc_state==1))
      {
        gbep_send_login_data_time = GBEP_SEND_LOGIN_DATA_TIME_SP; // 重发登录指令时间
        gbep_state = GBEP_STATE_SEND_LOGIN;
        
        gbep_send_hb_time = GBEP_SEND_HB_TIME_SP;
        gbep_send_eng_data_time = 50; // 登录后,5秒发送数据
        gbep_send_obd_data_time = GBEP_SEND_OBD_DATA_TIME_SP;
        gbep_send_bz_data_time = 80;
        return;
      }
    }
    else  // 车辆已登录
    {
      // ACC=OFF,发送车辆登出
      if (acc_state==0) // ACC关
      {
        gbep_send_login_data_time = GBEP_SEND_LOGIN_DATA_TIME_SP; // 重发登录指令时间
        gbep_state = GBEP_STATE_SEND_LOGOUT;
        return;
      }

      // 车辆登录30s后,发送NTP数据
      if (gbep_send_hb_time==0x00)
      {
        gbep_send_hb_time = GBEP_SEND_HB_TIME_SP;
        gbep_state = GBEP_STATE_SEND_NTP;
        return;
      }

      // 发动机启动30s后,发送OBD数据
      if (GBEP_SEND_OBD_FLAG==0 && (gbep_send_obd_data_time==0))
      {
        if (engine_speed > 300) // 下车发动机发动
        {
          gbep_send_obd_data_time = GBEP_SEND_ENG_DATA_TIME_SP;
          gbep_state = GBEP_STATE_SEND_OBD;
          return;
        }
      }

      // 发动机启动,发送发动机数据
      if (gbep_send_eng_data_time==0)
      {
        gbep_send_eng_data_time = GBEP_SEND_ENG_DATA_TIME_SP;  // 重置数据发送时间
        gbep_send_bz_data_time = GBEP_SEND_BZ_DATA_TIME_SP;  // 重置盲区发送时间
        gbep_state = GBEP_STATE_SEND_ENG;
        return;
      }

      // 补发信息上报
      if(gbep_send_bz_data_time==0)
      {
        gbep_send_bz_data_time = GBEP_SEND_BZ_DATA_TIME_SP;  // 重置盲区发送时间
        gbep_state = GBEP_STATE_SEND_BZ;
        return;
      }
    }// end if (GBEP_SEND_LOGIN_FLAG==1)
  }
  else // 环保功能未开启
  {
    gbep_state = GBEP_STATE_SEND_INIT;
    gbep_flags1.byte = 0x00;
    GBEP_DisableLink(); // 禁止bjep套接字连接
    gbep_socket_enable_flag = FALSE;
  }

  // 数据发送时间和心跳时间倒计
  if (gbep_socket_enable_flag == TRUE)
  {
    if (engine_speed > 300) // 下车发动机发动(OBD获取需要时间)
    {
      if(gbep_send_obd_data_time)
        gbep_send_obd_data_time--;
    }

    if (gbep_send_eng_data_time) // 数据发送定时器
      gbep_send_eng_data_time--;

    if (gbep_send_hb_time) // 心跳发送定时器
      gbep_send_hb_time--;

    if(gbep_send_bz_data_time) // 补发数据定时器
      gbep_send_bz_data_time--;

    if(gbep_send_login_data_time) // 登录指令
      gbep_send_login_data_time--;

  }
  else
  {
    gbep_send_eng_data_time = GBEP_SEND_ENG_DATA_TIME_SP;
    gbep_send_obd_data_time = GBEP_SEND_OBD_DATA_TIME_SP;
    gbep_send_hb_time = GBEP_SEND_HB_TIME_SP;
  }
}

/*************************************************************************
 * 发送GB环保帧(10ms调用一次)
*************************************************************************/
void GBEP_ProduceSendData(void)
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
    GBEP_StateMachine();
  }

  switch (gbep_state)
  {
  case GBEP_STATE_SEND_INIT:
    gbep_login_sn = 0;
    gbep_send_data_sn = 0;
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_IDLE:
    break;

  case GBEP_STATE_SEND_LOGIN: // 车辆登录
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

  case GBEP_STATE_SEND_NTP: // 终端校时
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

  case GBEP_STATE_SEND_OBD: // 实时信息上报(OBD)
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

  case GBEP_STATE_SEND_ENG: // 实时信息上报(发动机)
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
    
  case GBEP_STATE_SEND_BZ: // 补发信息上报
    retVal = GBEP_SendBzData();
    if (retVal==0)
    {
#if GBEP_DEBUG
      PcDebug_SendString("GBEP:BZ!\r\n");
#endif
      GBEP_SEND_BZ_FLAG = 1;
    }
    gbep_state = GBEP_STATE_SEND_IDLE;
    break;

  case GBEP_STATE_SEND_LOGOUT: // 车辆登出
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

/******************************************************************************
* 处理GBEP环保服务器(10012)下行数据
*******************************************************************************/
void GBEP_ProcessRecvData(uint8_t* pdata, uint16_t len)
{
#if 0
  if ((pdata[0]==0x23) && (pdata[1]==0x23)) // 帧头
  {
    gbep_socket.error_cnt = 0x00; // 清除错误计数
    gbep_socket.error_flag = FALSE;
    gbep_socket.hb_timer_10ms = gbep_socket.hb_timer_10ms_sp; // 重置心跳计数器
  }
#endif
}

/******************************************************************************
* 初始化参数
*******************************************************************************/
void GBEP_Initialize(void)
{
  GbepBlindZone_Init();  // 盲区补偿(补发数据)

  gbep_state = GBEP_STATE_SEND_INIT;
  gbep_flags1.byte = 0x00;
  //GBEP_DisableLink();  // 禁止gbep套接字连接
}


