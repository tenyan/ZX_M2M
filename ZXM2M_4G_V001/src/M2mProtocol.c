/*****************************************************************************
* @FileName: M2mProtocol.c
* @Engineer: TenYan
* @version   V1.1
* @Date:     2020-12-31
* @brief     M2M 协议实现
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* 外部函数
******************************************************************************/
extern void iZxM2m_SendConnenctMsg(m2m_context_t* pThis);  // 发送重型连接指令
extern void iZxM2m_SendTcMsg(m2m_context_t* pThis, uint8_t msg_type);  // 发送重型数据
extern void iZxM2m_SendBzData(m2m_context_t* pThis);  // 发送盲目补偿数据

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
  .header = 0x55AA5AA5, // 校验头
    
  .devId = {0xA0,0xC1,0x10,0x09,0,0,7},  // 终端的ID

  .apn = "CMNET",  // APN
  .apn_len = 5,   // APN长度
  
  .user_name = "TenYan",  // M2M平台登录用户名
  .user_name_len = 6,       // M2M平台登录用户名长度
  
  .password = {'1','2','3'},    // M2M平台登录密码
  .password_len = 3,      // M2M平台登录密码长度
  
  .smsc_number = "15150578385", // 短信中心号码
  .smsc_number_len = 11,        // 短信中心号码长度
  
  .main_srv_ip = {58,218,196,200},  // 主中心IP地址  
  .main_srv_port = 10004,  // 主中心端口
  .main_srv_protocol = 1,  // 主中心通信协议类型: 0=UDP协议, 1=TCP协议

  .sub_srv_ip = {218,80,94,178},   // 副中心IP地址
  .sub_srv_port = 6601,   // 副中心端口
  .sub_srv_protocol = 1,  // 副中心通信协议类型: 0=UDP协议, 1=TCP协议

  .hb_timer_sp = 120,    // 心跳间隔(秒),0=不发送心跳,默认心跳间隔为30秒
  .login_retry_sp = 3,   // 最大登录重复次数
  
  .login_min_time_sp = 5,      // 登录失败最小重试间隔
  .login_max_time_sp = 30,     // 登录失败最大重试间隔
  .sms_recv_timeout_sp = 300,  // 短信接收超时时间(秒)
  
  .can_baudrate = 0,  // 本地CAN总线波特率: 0=默认250K, 1=125K, 2=250K, 3=500K
  .can_format = 1,    // 本地CAN报文格式: 0=标准, 1=扩展, 3=两种都有

  // CAN ID 过滤配置,4字节一组
  .canId_tbl= {0x00000215,0x00000225,0x00000235,0x00000245,0x00000256,0x00000266,0x00000267,0x00000268,0x18FE01F4,
               0x18FE02F4,0x18FE03F4,0x18FE04F4,0x18FE05F4,0x18FE06F4,0x18FE07F4},
  .canId_num = 15,       // can id 个数
  
  .wakeup_work_time_sp = 300,  // 唤醒后工作时间(s)
  .sleep_time_sp = 120,        // 休眠时间(分)
  .ss_report_time_sp = 60,     // 终端基本状态同步数据自动发送间隔(秒)
  //.msin = {04,84,33,60,89,92}, // SIM卡号
  .can_err_dt_sp = 120,  // CAN故障判断时间
  .can_ok_dt_sp = 10,   // CAN恢复正常判断时间
  
  .power_off_dt_sp = 10,  // 终端断电时间条件
  .power_on_dt_sp = 1,   // 终端上电时间条件
  
  .low_power_voltage_sp = 100,  // 外部电源低电压报警阈值，单位：1%
  .low_power_dt_sp = 10,       // 外部电源低电压报警的时间参数，单位：1s
  
  .low_bat_voltage_sp = 35,  // 内部电源低电压报警阈值，单位：1%
  .low_bat_dt_sp = 10,  // 外部电源低电压报警的时间参数
  
  .gps_ant_err_dt_sp = 30, // 终端天线故障报警的时间参数，单位：1s
  .gps_ant_ok_dt_sp = 30,  // 终端天线故障报警的解除时间参数，单位：1s
  
  .gps_module_err_dt_sp = 30,  // 终端GPS模块故障报警的时间参数
  .gps_module_ok_dt_sp = 5,   // 终端GPS模块故障报警解除的时间参数(1s)
  
  .over_speed_sp = 60,  // 表示超速报警阈值，单位：1KM/H
  .over_speed_dt_sp = 20,  // 超速报警的时间参数，单位：1s
  
  .towing_alarm_range_sp = 2, //非自主移动（拖车）报警距离阈值，单位：1KM

  .work_time_report_cfg = 0x00,  // 设备工作时间段统计配置参数
  .work_data_report_mode_sp = 0, // 工作参数（工况）数据单条上传模式
                                 // 0x00:默认,等时间间隔上传
                                 // 0x01:其他(如以某工况参数的变频函数为频率发送)
                                 // 0xFF:不以单条上传模式传输
  .work_data_report_time_sp = 30, // 工作参数(工况)传输参数。时间，单位：1秒
  .position_report_mode_sp = 0,   // 位置信息单条上传模式
  .position_report_time_sp = 60,  // 位置信息上传间隔
};

/******************************************************************************
*
******************************************************************************/
#if (PART("M2M构建和解析TLV"))
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

//==构造终端状态位=========================================================
uint32_t im2m_BuildDeviceStatus(void)
{
  uint32_t uiTemp = 0;

  if (1==GPS_GetPositioningStatus()) // 定位状态
  {
    uiTemp |= BIT(31);
  }

  // BIT29预留

  if (0 != colt_info.tobx_state) // 终端工作状态(0:工作中(正常运行), 1:未工作(省电休眠))
  {
    uiTemp |= BIT(28);
  }

  if (0xAA==colt_info.wdt_status) // 终端健康状态(0:正常, 1:存在告警异常)
  {
    uiTemp |= BIT(27);
  }

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // 关联设备工作状态(0:工作中, 1:未工作)
  {
    uiTemp |= BIT(26);
  }
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // 关联设备健康状态(0:正常, 1:存在故障异常)
  {
    uiTemp |= BIT(25);
  }

  if (1==COLT_GetAccStatus())  // ACC状态(0:OFF, 1:ON)
  {
    uiTemp |= BIT(24);
  }

  if (COLT_GetMainPowerStatus()) // 主电断电标志(0:未断电, 1:断电)
  {
    uiTemp |= BIT(23);
  }
  if (COLT_GetMainPowerLowStatus()) // 主电电压低于阈值标志(0:未低于, 1:低于)
  {
    uiTemp |= BIT(22);
  }

  if (COLT_GetBatChargeStatus()) // 内置电池充电标志(0:未充电, 1:充电中)
  {
    uiTemp |= BIT(21);
  }
  if (COLT_GetBatLowStatus()) // 内置电池电压低于阈值标志(0:未低于, 1:低于)
  {
    uiTemp |= BIT(20);
  }

  if (GPS_GetModuleStatus()) // GPS模块故障标志(0:无故障, 1:故障)
  {
    uiTemp |= BIT(19);
  }
  if (GPS_GetAntOpenStatus()) // GPS天线断开标志(0:未断开, 1:断开)
  {
    uiTemp |= BIT(18);
  }
  if (GPS_GetAntShortStatus()) // GPS天线短路标志(0:未短路, 1:短路)
  {
    uiTemp |= BIT(17);
  }
  
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // CAN总线通信中断标志(0:未中断 1:中断)
  {
    uiTemp |= BIT(16);
  }

  // BIT15-内置锁车继电器标志
  // 14~8预留

  if (GPS_GetSpeedOverrunStatus())  // 超速标志(0:未超速, 1:已超速)
  {
    uiTemp |= BIT(7);
  }

  if (COLT_GetVehicleTowingStatus()) // 拖车报警(0:未发生, 1:已发生)
  {
    uiTemp |= BIT(6);
  }

  //BIT5:4-车辆状态标志(00:闲置, 01:怠速, 10:工作, 11:未知)
  // 3~1预留

  if (COLT_GetBoxOpenStatus()) // 终端开盖标识(0:未开盖, 1:已开盖)
  {
    uiTemp |= BIT(0);
  }

  return uiTemp;
}

//==TLV1-状态位(0x3000)====================================================
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

//==填充位置信息===========================================================
uint8_t im2m_BuildPositionInfo(uint8_t *pbuf)
{
  uint8_t temp_val = 0;
  uint8_t len = 0;
  uint32_t lat;
  uint32_t lon;
  utc_time_t utc_time;

  //==经纬半球======================================
  if (0==GPS_GetEastWest()) // 经度半球
  {
    temp_val |= 0x01;
  }

  if (0==GPS_GetNorthSouth()) // 纬度半球
  {
    temp_val |= 0x10;
  }
  pbuf[len++] = temp_val;

  //==经度=========================================
  lat = GPS_GetLatitude();
  pbuf[len++] = (lat>>24) & 0xFF;
  pbuf[len++] = (lat>>16) & 0xFF;
  pbuf[len++] = (lat>>8) & 0xFF;
  pbuf[len++] = lat & 0xFF;

  //==经度=========================================
  lon = GPS_GetLongitude();
  pbuf[len++] = (lon>>24) & 0xFF;
  pbuf[len++] = (lon>>16) & 0xFF;
  pbuf[len++] = (lon>>8) & 0xFF;
  pbuf[len++] = lon & 0xFF;

  pbuf[len++] = (GPS_GetSpeed()/10) & 0xFF; // 速度
  pbuf[len++] = (GPS_GetHeading()/2) & 0xFF; // 方向

  //==海拔高度====================================
  pbuf[len++] = (GPS_GetHeight()>>8) & 0xFF; // 高度
  pbuf[len++] =  GPS_GetHeight() & 0xFF;			 // 固定部分20+1字节

  //==日期和时间=================================
  if (GPS_GetPositioningStatus())
  {
    utc_time = GPS_GetUtcTime();  // GPS已定位,使用GPS提供的时间
  }
  else
  {
    utc_time = RTC_GetUtcTime(); // GPS未定位,使用内部RTC时间
  }
  pbuf[len++] = utc_time.year;
  pbuf[len++] = utc_time.month;
  pbuf[len++] = utc_time.day;
  pbuf[len++] = utc_time.hour;
  pbuf[len++] = utc_time.minute;
  pbuf[len++] = utc_time.second;

  return len;
}

//==TLV2-位置信息单包(0x2101)==============================================
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

//==TLV3-外部电源电压(0x3004)==============================================
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

//==TLV4-终端内置电池电压(0x3005)==========================================
uint16_t im2m_BuildTlvMsg_3005(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t temp_val = 0;

  pbuf[len++] = 0x30;	// TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  temp_val = COLT_GetBatVoltage(); // 单位10mv
  temp_val = temp_val/10;
  pbuf[len++] = 0;    // VALUE
  pbuf[len++] = temp_val & 0xff;

  return len;
}

//==TLV5-本地信号场强(0x3007)==============================================
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

//==TLV6-当前GPS卫星颗数(0x3008)===========================================
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

//==TLV8-基站ID(0x3006)LAC(2byet)+CELL(2Byte)==============================
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

//==TLV8- PPP(端对端协议)状态(0x3017)======================================
uint16_t im2m_BuildTlvMsg_3017(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x17;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x01;

  if (1==Cellura_GetModemStatus()) // 模块故障
  {
    temp_val = 4;
  }
  else if (1==Cellura_GetSimCardState()) // SIM卡故障
  {
    temp_val = 6;
  }
  else
  {
    temp_val = 0;  // AT状态
  }

  pbuf[len++] = temp_val;  // VALUE

  return len;
}

//==TLV9-GSM注册状态(0x3018)===============================================
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

//==TLV10-GPRS注册状态(0x3019)=============================================
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

//==TLV11-与平台连接状态(0x301A)===========================================
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

//==can总线数据(0x2103)====================================================
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
    len += 3;           // 长度(2b)+CAN数据包数(1B)
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
    pbuf[4] = can_num;  // CAN帧数量
  }
  
  if (0 == can_num) // 无CAN数据
  {
    len = 0;
  }

  return len;
}

//==报警===================================================================
static uint16_t im2m_BuildTlvMsg_300D(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t alarm_num = 0;
  uint8_t it;

  pbuf[len++] = 0x30;  // TAG
  pbuf[len++] = 0x0D;
  len += 2;  // 指向VALUE
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

  //if(0==alarm_num) // 无可上报告警
  //{
  //  len = 0x00;
  //}

  return len;
}

//==终端产品唯一编号=======================================================
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

  if (len==7) // 校验参数长度
  {
    retVal = 1;
    memcpy(m2m_asset_data.devId, pValue, 7);
  }

  return retVal;
}

//==终端设备软件版本号=====================================================
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

//==M2M平台网络接入点名称(APN)=============================================
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

  if (len<=MAX_SIZE_OF_APN) // 校验参数长度
  {
    retVal = 1;
    m2m_asset_data.apn_len = len;
    memcpy(m2m_asset_data.apn, pValue, len);
  }

  return retVal;
}

//==M2M平台登录用户名=======================================================
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
    //memcpy(GBPara.aucVIN, m2m_asset_data.user_name, 17); // VIN码
  }
  return retVal;
}

//==M2M平台登录密码========================================================
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

//==短信中心号码===========================================================
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

//==主中心IP地址===========================================================
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

  if (4==len) // 校验参数长度
  {
    retVal = 1;
    memcpy(m2m_asset_data.main_srv_ip, pValue, 4);
    memcpy(m2m_context.srv_ip, m2m_asset_data.main_srv_ip, 4); // 需要处理
    m2m_context.new_srv_address_flag = M2M_TRUE;
  }

  return retVal;
}

//==副中心IP地址===========================================================
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

  if (4==len) // 校验参数长度
  {
    retVal = 1;
    memcpy(m2m_asset_data.sub_srv_ip, pValue, 4);
  }

  return retVal;
}

//==主中心端口=============================================================
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

//==副中心端口=============================================================
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

//==心跳间隔(秒)===========================================================
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

//==最大登录重复次数参数===================================================
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

//==登录失败重试间隔参数===================================================
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

//==本地CAN总线波特率======================================================
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

//==本地CAN报文格式========================================================
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

//==CAN ID 过滤配置========================================================
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

//==//进入休眠时间(秒)=====================================================
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

//==休眠期间定时唤醒间隔(分钟)============================================
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

//==终端基本状态同步数据自动发送间隔(秒)==================================
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

//==SIM卡号,不足高位补0===================================================
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

//==主中心域名============================================================
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

//==副中心域名============================================================
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

//==硬件版本号============================================================
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

//==外电源额定电压(0.1V)==================================================
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

//==终端电池额定电压(0.1V)================================================
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

//主中心承载协议类型(0:UDP协议,1:TCP协议)==================================
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

//==副中心承载协议类型(0:UDP协议,1:TCP协议)================================
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

//==控制器与终端总线通信中断报警参数======================================
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

//==终端外部电源断电报警参数==============================================
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

//==终端外部电源低电压报警参数============================================
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

//终端内部电源(电池)低电压报警(TLV-0x300D-0x09报警)参数===================
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

//==终端GPS天线故障报警(TLV-0x300D-0x06报警)参数==========================
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

//终端GPS定位模块故障报警(TLV-0x300D-0x05报警)参数========================
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

//==超速报警(TLV-0x300D-0x03)报警参数=====================================
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

//==拖车报警(TLV-0x300D-0x01)报警参数=====================================
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

//==//升级服务器IP========================================================
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

//==升级服务器端口号======================================================
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

//==TLV-1005升级固件版本号================================================
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

//==TLV-100C升级固件文件的名称,包含后缀====================================
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

//==TLV-100D-当前软件版本号================================================
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

//升级服务器协议类型
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

//==设备工作时间段统计配置参数============================================
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
 * 第1字节表示工作参数(工况)数据单条上传模式
 * 第2字节当第1字节为0x00时，表示工作参数(工况)传输参数
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
 * 第1字节表示位置信息单条上传模式
 * 第2字节表示位置信息传输间隔
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

//==终端ACC ON 累计时间===================================================
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

//==终端重启控制(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4000(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;
#if 0
    tempVal = FLASH_OB_GetUser();
    if (0 == (0x01 & tempVal)) // 判断内部看门狗是否由硬件使能
    {
      FLASH_OB_Unlock();
      // OB_IWDG_HW: 选择硬件独立看门狗 
      // OB_IWDG_SW: 选择软件独立看门狗 
      // 进入 STOP 模式不产生复位 
      // OB_STDBY_NoRST: 进入 Standby 模式不产生复位 
      FLASH_OB_UserConfig(OB_IWDG_SW, OB_STOP_NoRST, OB_STDBY_NoRST); // 原来是OB_IWDG_HW
      FLASH_OB_Launch();
      FLASH_OB_Lock();
    }
    CTL_SetRestartDelayTime(5); // 延时复位函数
#endif
  }

  return retVal;
}

//==终端设备参数初始化(RC)================================================
static uint16_t im2m_AnalyzeTlvMsg_4001(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (0==len)
  {
    retVal = 1;
    Parm_ResetM2mAssetDataToFactory(); // 重置参数配置
    CTL_SetRestartDelayTime(5); // 设置延时复位函数
  }

  return retVal;
}

//==标准锁车指令(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4004(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;
  
#if 0
  uint8_t cmd;
  bittype lock_cmd_flag;

  lock_cmd_flag.byte = lvc_context.lock_cmd_flag;
  cmd = *pValue;
  if (0==cmd)  // 解锁
  {
    lock_cmd_flag.b.bit2 = 1;
    lock_cmd_flag.b.bit3 = 0;
    lock_cmd_flag.b.bit4 = 1;
    lock_cmd_flag.b.bit5 = 0;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.unlock_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else if (1==cmd) // 一级锁车:怠速运行:1200rpm
  {
    lock_cmd_flag.b.bit2 = 1;
    lock_cmd_flag.b.bit3 = 1;
    lvc_context.lock_cmd_flag = lock_cmd_flag.byte;
    lvc_context.lock_level1_sn = m2m_context.rx_sn;
    Parm_SaveLvcInfo();
  }
  else if (2==cmd) // 二级锁车:禁止启动
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

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // 解析锁车执行结果
  {
    retVal = 0;
    m2m_context.ss_req.send_timer = 8;
  }

  return retVal;
}

//==终端立即关机(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4008(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;

  if (0==len)
  {
    retVal = 1;
    CTL_SetPwroffDelayTime(5); // 设置延时关机时间
  }

  return retVal;
}

//==锁车功能激活/关闭(RC)=================================================
static uint16_t im2m_AnalyzeTlvMsg_4009(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;
#if 0
  uint8_t cmd;
  bittype lock_cmd_flag;

  lock_cmd_flag.byte = lvc_context.lock_cmd_flag;
  cmd = *pValue;
  if (0==cmd) // 关闭锁车功能
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
  else if (1==cmd) // 激活锁车功能
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

  if (CAN_NOK==CAN_GetRecvState(CAN_CHANNEL1)) // 解析绑定和解绑指令
  {  retVal = 0;}

  return retVal;
}

//==调试模式设置(RC)======================================================
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

//==ECU类型================================================================
static uint16_t im2m_AnalyzeTlvMsg_A1FE(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t ucTemp = 0;

  if (1==len)
  {
    retVal = 1;

#if 0
    ucTemp = *pValue;
    PcDebug_SendData("类型\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    if (ucTemp==1)
    {
      m2m_asset_data.ecu_type=1;
      PcDebug_SendData("潍柴\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==2)
    {
      m2m_asset_data.ecu_type=2;
      PcDebug_SendData("玉柴\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==3)
    {
      m2m_asset_data.ecu_type=3;
      PcDebug_SendData("上柴\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==4)
    {
      m2m_asset_data.ecu_type=4;
      PcDebug_SendData("杭发\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else if (ucTemp==5)
    {
      m2m_asset_data.ecu_type=5;
      PcDebug_SendData("飞鸿\r\n", 6, DBG_MSG_TYPE_ANYDATA);
    }
    else
    {
      m2m_asset_data.ecu_type=1;
      PcDebug_SendData("默认潍柴\r\n", 10, DBG_MSG_TYPE_ANYDATA);
    }
#endif
  }
  return retVal;
}

//==发动机类型============================================================
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

//==更换终端==============================================================
static uint16_t im2m_AnalyzeTlvMsg_A1FF(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 1;

#if 0
  /*根据ECU类型处理*/
  if (g_stuSYSParamSet.EcuType==2) //玉柴
  {
    memcpy(pDataBuff,p+1,4);
    m_stuMcuCmd.P_uiDeviceID=pDataBuff[0]+(pDataBuff[1]<<8)+(pDataBuff[2]<<16)+(pDataBuff[3]<<24);

    m_stuMcuCmd.ucBindCmd=0xbb;
    m_stuMcuCmd.F_Step=3;
    Parm_SaveLvcInfo();
  }
  else if (g_stuSYSParamSet.EcuType==4)   //杭发
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

//==type按照递增顺序填写=================================================
im2m_CmdTlv_t m2m_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/
  {0x0000, im2m_BuildTlvMsg_0000, im2m_AnalyzeTlvMsg_0000},/*设备终端编号*/
  {0x0001, im2m_BuildTlvMsg_0001, NULL},                   /*终端设备软件版本编号，只读*/
  {0x0002, im2m_BuildTlvMsg_0002, im2m_AnalyzeTlvMsg_0002},/*M2M平台网络接入点名称（APN）*/
  {0x0003, im2m_BuildTlvMsg_0003, im2m_AnalyzeTlvMsg_0003},/*M2M平台登录用户名*/
  {0x0004, im2m_BuildTlvMsg_0004, im2m_AnalyzeTlvMsg_0004},//M2M平台登录密码
  {0x0005, im2m_BuildTlvMsg_0005, im2m_AnalyzeTlvMsg_0005},//短信中心号码
  {0x0006, im2m_BuildTlvMsg_0006, im2m_AnalyzeTlvMsg_0006},//主中心IP地址
  {0x0007, im2m_BuildTlvMsg_0007, im2m_AnalyzeTlvMsg_0007},//副中心IP地址
  {0x0008, im2m_BuildTlvMsg_0008, im2m_AnalyzeTlvMsg_0008},//主中心端口
  {0x0009, im2m_BuildTlvMsg_0009, im2m_AnalyzeTlvMsg_0009},//副中心端口
  {0x000A, im2m_BuildTlvMsg_000A, im2m_AnalyzeTlvMsg_000A},//心跳间隔，单位：秒
  {0x000B, im2m_BuildTlvMsg_000B, im2m_AnalyzeTlvMsg_000B},//最大登录重复次数参数
  {0x000C, im2m_BuildTlvMsg_000C, im2m_AnalyzeTlvMsg_000C},//登录失败重试间隔参数

  {0x0106, im2m_BuildTlvMsg_0106, im2m_AnalyzeTlvMsg_0106},//本地CAN总线波特率
  {0x0107, im2m_BuildTlvMsg_0107, im2m_AnalyzeTlvMsg_0107},//本地CAN报文格式
  {0x0108, im2m_BuildTlvMsg_0108, im2m_AnalyzeTlvMsg_0108},//CAN ID 过滤配置
  {0x010B, im2m_BuildTlvMsg_010B, im2m_AnalyzeTlvMsg_010B},//进入休眠时间，单位：秒
  {0x010C, im2m_BuildTlvMsg_010C, im2m_AnalyzeTlvMsg_010C},//休眠期间定时唤醒间隔，单位：分
  {0x010D, im2m_BuildTlvMsg_010D, im2m_AnalyzeTlvMsg_010D},//终端基本状态同步数据自动发送间隔,单位：秒
  {0x0110, im2m_BuildTlvMsg_0110, im2m_AnalyzeTlvMsg_0110},//SIM卡号，不足高位补0
  {0x0111, im2m_BuildTlvMsg_0111, im2m_AnalyzeTlvMsg_0111},//ICCID码，只读
  {0x0113, im2m_BuildTlvMsg_0113, im2m_AnalyzeTlvMsg_0113},//主中心域名
  {0x0114, im2m_BuildTlvMsg_0114, im2m_AnalyzeTlvMsg_0114},//副中心域名
  {0x0115, im2m_BuildTlvMsg_0115, im2m_AnalyzeTlvMsg_0115},//DNS
  {0x0116, im2m_BuildTlvMsg_0116, im2m_AnalyzeTlvMsg_0116},//硬件版本号，如V1.5表示为0x0105,只读
  {0x0117, im2m_BuildTlvMsg_0117, im2m_AnalyzeTlvMsg_0117},//外电源额定电压,单位：0.1V
  {0x0118, im2m_BuildTlvMsg_0118, im2m_AnalyzeTlvMsg_0118},//终端电池额定电压,单位：0.1V
  {0x0119, im2m_BuildTlvMsg_0119, im2m_AnalyzeTlvMsg_0119},//主中心承载协议类型：0：UDP协议,1: TCP协议
  {0x011A, im2m_BuildTlvMsg_011A, im2m_AnalyzeTlvMsg_011A},//副中心承载协议类型：0：UDP协议,1: TCP协议

  {0x0201, im2m_BuildTlvMsg_0201, im2m_AnalyzeTlvMsg_0201},//控制器与终端总线通信中断报警参数
  {0x0202, im2m_BuildTlvMsg_0202, im2m_AnalyzeTlvMsg_0202},//终端外部电源断电报警参数
  {0x0203, im2m_BuildTlvMsg_0203, im2m_AnalyzeTlvMsg_0203},//终端外部电源低电压报警参数
  {0x0204, im2m_BuildTlvMsg_0204, im2m_AnalyzeTlvMsg_0204},//终端内部电源（电池）低电压报警（TLV-0x300D-0x09报警）参数
  {0x0205, im2m_BuildTlvMsg_0205, im2m_AnalyzeTlvMsg_0205},//终端GPS天线故障报警（TLV-0x300D-0x06报警）参数
  {0x0206, im2m_BuildTlvMsg_0206, im2m_AnalyzeTlvMsg_0206},//终端GPS定位模块故障报警（TLV-0x300D-0x05报警）参数
  {0x020A, im2m_BuildTlvMsg_020A, im2m_AnalyzeTlvMsg_020A},//超速报警报警参数
  {0x020B, im2m_BuildTlvMsg_020B, im2m_AnalyzeTlvMsg_020B},//拖车报警报警参数

  {0x1002,                  NULL, im2m_AnalyzeTlvMsg_1002},//升级服务器IP
  {0x1003,                  NULL, im2m_AnalyzeTlvMsg_1003},//升级服务器端口号
  {0x1005, im2m_BuildTlvMsg_1005, im2m_AnalyzeTlvMsg_1005},//升级固件版本号
  {0x100C, im2m_BuildTlvMsg_100C, im2m_AnalyzeTlvMsg_100C},//升级固件名称
  {0x100E,                  NULL, im2m_AnalyzeTlvMsg_100E},//升级服务器协议类型

  {0x2000, im2m_BuildTlvMsg_2000, im2m_AnalyzeTlvMsg_2000},//设备工作时间段统计配置参数
  {0x2001, im2m_BuildTlvMsg_2001, im2m_AnalyzeTlvMsg_2001},//第1字节表示工作参数（工况）数据单条上传模式,第2字节当第1字节为0x00时，表示工作参数（工况）传输参数
  {0x2002, im2m_BuildTlvMsg_2002, im2m_AnalyzeTlvMsg_2002},//第1字节表示位置信息单条上传模式,第2字节表示位置信息传输间隔

  {0x3016, im2m_BuildTlvMsg_3016, im2m_AnalyzeTlvMsg_3016},//工作小时

  {0x4000,                  NULL, im2m_AnalyzeTlvMsg_4000},//终端重启控制
  {0x4001,                  NULL, im2m_AnalyzeTlvMsg_4001},//终端设备参数初始化
  {0x4004,                  NULL, im2m_AnalyzeTlvMsg_4004},//标准锁车指令
  {0x4008,                  NULL, im2m_AnalyzeTlvMsg_4008},//终端立即关机
  {0x4FFF,                  NULL, im2m_AnalyzeTlvMsg_4FFF},//调试模式设置

  {0xA1FE, im2m_BuildTlvMsg_A1FE, im2m_AnalyzeTlvMsg_A1FE},//发动机类型
  {0xA1FF,                   NULL, im2m_AnalyzeTlvMsg_A1FF},//更换终端

  {0xA510,                   NULL, iZxM2m_AnalyzeTlvMsg_A510}, //重型专用:绑定与解绑
  {0xA511,                   NULL, iZxM2m_AnalyzeTlvMsg_A511}, //重型专用:锁车与解锁
  {0xA512,                   NULL, iZxM2m_AnalyzeTlvMsg_A512}, //重型专用:环保协议类型
  {0xA513,                   NULL, iZxM2m_AnalyzeTlvMsg_A513}, //重型专用:VIN码设置
  {0xA514,                   NULL, iZxM2m_AnalyzeTlvMsg_A514}, //重型专用:强制、故障救援指令
};
#define NUM_OF_M2M_CMD_DEAL   (sizeof(m2m_CmdDealTbl)/sizeof(im2m_CmdTlv_t))

#endif

#if (PART("M2M消息组建函数"))
/*************************************************************************
 * 计算校验和
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
 * 构建包头
*************************************************************************/
uint8_t im2m_BuildMsgHead(uint8_t *pbuf, im2m_msg_type_t msgType, uint16_t msgBodyLen, uint8_t flag, uint16_t SerialNumber)
{
  uint8_t len = 0;
  uint16_t temp_val;

  pbuf[len++] = msgType; // 报文类型

  memcpy(&pbuf[len], m2m_asset_data.devId, 7); // 产品唯一标号
  len += 7;

  pbuf[len++] = flag; // 标识报文特定信息:bit3-盲区补偿标志

  pbuf[len++] = (SerialNumber>>8) & 0xFF; // 报文流水号
  pbuf[len++] =  SerialNumber & 0xFF;

  temp_val = msgBodyLen + 1; // +1为校验字
  pbuf[len++] = (temp_val>>8) & 0xFF; // 报文体长度+校验字长度
  pbuf[len++] =  temp_val & 0xFF;

  return len;
}

/*************************************************************************
 * 构建SatusSync消息
*************************************************************************/
uint16_t im2m_BuildStatusSyncBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x02;
  pbuf[len++] = 'S';  // 数据类型
  pbuf[len++] = 'S';

  //==数据内容=========================================
  len++;  // 数据内容长度
  len++;
  len++;  // 状态同步TLV个数

  //==必要TLV==========================================
  /// TLV1-状态位(0x3000)
  temp_val = im2m_BuildTlvMsg_3000(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// 位置信息单包(0x2101)
  temp_val = im2m_BuildTlvMsg_2101(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// 主电源电压值(0x3004)
  temp_val = im2m_BuildTlvMsg_3004(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// 终端内置电池电压(0x3005)
  temp_val = im2m_BuildTlvMsg_3005(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GSM信号强度
  temp_val = im2m_BuildTlvMsg_3007(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GPS卫星颗数
  temp_val = im2m_BuildTlvMsg_3008(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// ACC ON累计时间(0x3016)
  temp_val = im2m_BuildTlvMsg_3016(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// PPP(端对端协议)状态(0x3017)
  temp_val = im2m_BuildTlvMsg_3017(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// GCS域注册状态(0x3018)
  temp_val = im2m_BuildTlvMsg_3018(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// PS域注册状态(0x3019)
  temp_val = im2m_BuildTlvMsg_3019(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// 与平台连接状态(0x301A)
  temp_val = im2m_BuildTlvMsg_301A(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// Cellular ID,终端设备所在的小区标识
  temp_val = im2m_BuildTlvMsg_3006(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// CAN数据
  temp_val = im2m_BuildTlvMsg_2103(&pbuf[len]);
  len += temp_val;
  tlv_num += ((temp_val == 0)? 0: 1);

  /// 填充SS数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}

/*************************************************************************
 * 0x04提醒: 构建终端异常告警/解除告警(DeviceAlert)消息
*************************************************************************/
uint16_t im2m_BuildDeviceAlertBody(uint8_t* pbuf)
{
  uint16_t len=0;
  uint16_t alarm_len;

  //==提醒类型===========================================
  pbuf[len++] = 00; // 提醒类型长度
  pbuf[len++] = 02;
  pbuf[len++] = 'D'; // 提醒类型
  pbuf[len++] = 'A';

  //==提醒内容==========================================
  len++;  // 提醒内容长度
  len++;
  alarm_len = im2m_BuildTlvMsg_300D(&pbuf[len]);
  len += alarm_len;
  pbuf[4] = ((alarm_len>>8) & 0xFF);
  pbuf[5] = (alarm_len & 0xFF);

  return len;
}

/*************************************************************************
 * 0x04提醒: 构建机械设备故障码(DTC)消息
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
    //数据内容长度MSBs
    //数据内容长度LSB
  
    buf = &aMsgSendData[MSG_HEAD_LEN+7];
    *buf++ = 0x30;
    *buf++ = 0xEF;
    p = buf;    //数据长度
    buf += 2;
    p2= buf;    //DTC个数
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
 * 0x04提醒: 构建驾驶行为类提醒(DrivingBehavior)消息
*************************************************************************/
uint16_t im2m_BuildDrivingBehaviorMsg(uint8_t* buf)
{
  uint16_t len =0;

  return len;
}

#endif

#if (PART("发送M2M固件升级数据"))
/*************************************************************************
 * 固件升级相关消息
*************************************************************************/
//==向平台发送远程固件升级的请求(终端->平台)================================
uint16_t im2m_SendUpdataReqUqMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==命令类型========
  pbuf[len++] = 0x00;  // 命令类型长度
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // 命令
  pbuf[len++] = 'Q';

  //==命令内容========
  len += 2; // 长度
  pbuf[len++] = (M2M_FIRMWARE_PACKET_LEN>>8) & 0xFF;  // 每包数据的大小
  pbuf[len++] = M2M_FIRMWARE_PACKET_LEN & 0xFF;
  len +=im2m_BuildTlvMsg_100C(&pbuf[len]);
  len +=im2m_BuildTlvMsg_1005(&pbuf[len]);
  len +=im2m_BuildTlvMsg_100D(&pbuf[len]);
  msg_len = len - 6;
  pbuf[M2M_MSG_HEAD_LEN+4] = (msg_len >> 8) & 0xFF; // 命令内容长度
  pbuf[M2M_MSG_HEAD_LEN+5] = msg_len & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  
  return msg_len;
}

//==终端固件升级请求(终端->平台)==============================================
uint16_t im2m_SendUpdateReqUlMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;
  
  //==命令类型========
  pbuf[len++] = 0x00;  // 命令类型长度
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // 命令
  pbuf[len++] = 'L';

  //==命令内容========
  len += 2; // 长度
  pbuf[len++] = (rfu_context.block>>8) & 0xFF; // 请求包序列号
  pbuf[len++] =  rfu_context.block & 0xFF;
  pbuf[len++] = (rfu_context.total_block_count>>8) & 0xFF; // 文件总包数
  pbuf[len++] =  rfu_context.total_block_count & 0xFF;
  len +=im2m_BuildTlvMsg_100C(&pbuf[len]); // TLV-100C升级固件名称
  len +=im2m_BuildTlvMsg_1005(&pbuf[len]); // TLV-1005升级固件版本号
  msg_len = len - 6;
  pbuf[M2M_MSG_HEAD_LEN+4] = (msg_len >> 8) & 0xFF; // 命令内容长度
  pbuf[M2M_MSG_HEAD_LEN+5] = msg_len & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台

  return msg_len;
}

//==向平台上报升级结果(终端->平台)==============================================
uint16_t im2m_SendUpdateReqUrMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==命令类型========
  pbuf[len++] = 0x00;  // 命令类型长度
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // 命令
  pbuf[len++] = 'R';

  //==命令内容========
  pbuf[len++] = pThis->update.result; // 升级结果
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  pThis->upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;
  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台

  return msg_len;
}

/******************************************************************************
 *
******************************************************************************/
//==重连M2M服务器=============================================================
void im2m_ReConnectDefaultServer(void)
{
  uint8_t srv_ip_flag = 0;
  uint8_t srv_port_flag = 0;
  uint8_t srv_protocol = 0;

  if(memcmp(rfu_context.srv_ip, m2m_context.srv_ip, 4)==0) // ip一样
  {
    srv_ip_flag = 1;
  }

  if(rfu_context.srv_port==m2m_context.srv_port) // port一样
  {
    srv_port_flag = 1;
  }

  if(rfu_context.srv_protocol==m2m_context.srv_protocol) // protocol一样
  {
    srv_protocol = 1;
  }
  
  if(srv_ip_flag==0 || srv_port_flag==0 || srv_protocol==0) // 有一个不同就需要重新连接
  {
    memcpy(m2m_context.srv_ip, m2m_asset_data.main_srv_ip, 4);
    m2m_context.srv_port = m2m_asset_data.main_srv_port;
    m2m_context.srv_protocol = m2m_asset_data.main_srv_protocol;
    im2m_ReconnectLink();
  }
}

//==连接升级服务器============================================================
void im2m_ConnectUpdateServer(void)
{
  uint8_t srv_ip_flag = 0;
  uint8_t srv_port_flag = 0;
  uint8_t srv_protocol = 0;

  if(memcmp(rfu_context.srv_ip, m2m_context.srv_ip, 4)==0) // ip一样
  {
    srv_ip_flag = 1;
  }

  if(rfu_context.srv_port==m2m_context.srv_port) // port一样
  {
    srv_port_flag = 1;
  }

  if(rfu_context.srv_protocol==m2m_context.srv_protocol) // protocol一样
  {
    srv_protocol = 1;
  }

  if(srv_ip_flag==0 || srv_port_flag==0 || srv_protocol==0) // 有一个不同就需要重新连接
  {
    memcpy(m2m_context.srv_ip, rfu_context.srv_ip, 4);
    m2m_context.srv_port= rfu_context.srv_port;
    m2m_context.srv_protocol = rfu_context.srv_protocol;
    im2m_ReconnectLink();
  }
}

#endif

#if (PART("发送M2M网络数据"))
/******************************************************************************
 * 发送connect request消息
*******************************************************************************/
void im2m_SendConnenctMsg(m2m_context_t* pThis)
{
  uint16_t len = 0;
  uint16_t msg_body_len = 0;
  uint16_t msg_header_len = 0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN; // 从消息体开始填充

  /// 协议名
  pbuf[len++] = 0x00; // 协议名长度
  pbuf[len++] = 0x04;
  memcpy(&pbuf[len], "XM2M", 4);
  len += 4;

  /// 协议版本
  pbuf[len++] = 1;

  /// TLV100D-当前软件版本号
  len += im2m_BuildTlvMsg_100D(&pbuf[len]);

  /// TLV-0x0111 ICCID
  len += im2m_BuildTlvMsg_0111(&pbuf[len]);

  /// 连接标识
  pbuf[len++] = 0; // 报文体无鉴权信息
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  pThis->upload_sn++;
  pThis->conn_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CONN_REQ, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  len = msg_header_len + msg_body_len;
  check_sum = im2m_CalcSumCheck(pbuf, len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  //return msg_len;
}

/*************************************************************************
 * 构建断开连接(0x09-DISCONNECT)消息
*************************************************************************/
void im2m_SendDisconnectMsg(m2m_context_t* pThis)
{
  uint16_t len =0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文头==========================================
  pThis->upload_sn++;
  len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_DISCONNECT, 1, 0, pThis->upload_sn); // M2mUploadSN

  //==创建报文体==========================================
  pbuf[len++] = 0x00; // 中断连接原因: 0x00 正常下线，进入休眠

  //==计算校验字==========================================
  check_sum = im2m_CalcSumCheck(pbuf,len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
}

/*************************************************************************
 * 发送心跳发送(PING REQUEST)消息
*************************************************************************/
void im2m_SendPingMsg(m2m_context_t* pThis)
{
  uint16_t len = 0;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文头==========================================
  pThis->upload_sn++;
  pThis->ping_req.sn = pThis->upload_sn;
  len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PING_REQ, 0, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  check_sum = im2m_CalcSumCheck(pbuf,len);
  pbuf[len++] = check_sum;
  pThis->tx_size = len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  //return msg_len;
}

/*************************************************************************
 * 发送SatusSync消息
*************************************************************************/
void im2m_SendStatusSyncMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文体==========================================
  msg_body_len = im2m_BuildStatusSyncBody(&pbuf[M2M_MSG_HEAD_LEN]);

  //==创建报文头==========================================
  pThis->upload_sn++;
  pThis->ss_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PUSH_DATA, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  //return msg_len;
}

/*************************************************************************
 * 发送终端异常告警消息
*************************************************************************/
void im2m_SendDeviceAlertMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文体==========================================
  msg_body_len = im2m_BuildDeviceAlertBody(&pbuf[M2M_MSG_HEAD_LEN]);

  //==创建报文头==========================================
  pThis->upload_sn++;
  pThis->alarm_req.sn = pThis->upload_sn;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_ALERT, msg_body_len, 0, pThis->upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  pThis->tx_size = msg_len;

  im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  //return msg_len;
}

/*************************************************************************
 * 发送远程固件升级相关消息
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

  case M2M_UPDATE_STATE_ERROR: // 升级失败
    break;

  case M2M_UPDATE_STATE_SUCESS: // 升级成功
    break;

  default:
    break;
  }
}
#endif

//==请求发送超时管理=========================================================
uint8_t im2m_ReqTimeOutManage(im2m_request_t* pThis)
{
  uint8_t timeout_flag = M2M_FALSE; // 超时标志

  if (pThis->send_timer) // 发送周期倒计
    pThis->send_timer--;

  if (pThis->sent_flag==M2M_TRUE) // 响应超时判断
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

//==固件升级请求发送超时管理=================================================
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
      im2m_ReConnectDefaultServer(); // 失败,连接默认M2M平台
      break;

    case M2M_UPDATE_STATE_WAIT_RSP:
      if(m2m_context.update.retry_cnt > 5)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // 失败,连接默认M2M平台
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_SEND_REQ; // 重试
      }
      break;

    case M2M_UPDATE_STATE_DOWNLOAD_RSP:
      if(m2m_context.update.retry_cnt > 5)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // 失败,连接默认M2M平台
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_DOWNLOAD_REQ; // 重试
      }
      break;

     case M2M_UPDATE_STATE_REPORT_RSP:
      if(m2m_context.update.retry_cnt > 3)
      {
        m2m_context.update.retry_cnt = 0;
        m2m_context.update.state = M2M_UPDATE_STATE_IDLE;
        im2m_ReConnectDefaultServer(); // 失败,连接默认M2M平台
      }
      else
      {
        m2m_context.update.state = M2M_UPDATE_STATE_REPORT_REQ; // 重试
      }
      break;
    
    case M2M_UPDATE_STATE_ERROR: // 升级失败
      break;
    
    case M2M_UPDATE_STATE_SUCESS: // 升级成功
      break;
    
    default:
      break;
    }
  }
}

//==服务器地址改变后重启连接,该函数1s执行一次=================================
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

#if (PART("处理M2M协议收到的数据"))
/*******************************************************************************
 * 通用报文响应消息处理
 *******************************************************************************/
uint16_t im2m_AnalyzeAckMsg(m2m_context_t* pThis)
{
  uint16_t rsp_sn;
  uint8_t rsp_msg_type;
  uint8_t result;

  //==报文体解析===============================
  rsp_sn = pThis->rx_body_data[0];  // 被响应报文的流水号
  rsp_sn <<= 8;
  rsp_sn += pThis->rx_body_data[1];
  rsp_msg_type = pThis->rx_body_data[2];  // 被响应的报文类型
  result = pThis->rx_body_data[3];  // 处理结果

  //==响应结果和类型判断=======================
  if (result==0x00) // 0x00表示处理成功
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
 * 连接响应消息处理
 *******************************************************************************/
uint16_t im2m_AnalyzeConnRespMsg(m2m_context_t* pThis)
{
  uint8_t result;

  result = pThis->rx_body_data[0]; // 连接返回码
  if (result==0x00) // 0x00=连接成功
  {
    pThis->conn_success_flag = M2M_TRUE;
    pThis->conn_req.rsp_timeout_timer = pThis->conn_req.rsp_timeout_sp;
    pThis->conn_req.retry_cnt = 0x00;
    pThis->conn_req.sent_flag = 0x00;

    //pThis->ss_req.send_timer = pThis->ss_req.send_timer_sp; // 重置ss发送定时器
    pThis->ss_req.send_timer = 5; // 重置ss发送定时器
    pThis->ss_req.retry_cnt = 0;
    pThis->ss_req.sent_flag = M2M_FALSE;
    //pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp;
    pThis->ping_req.send_timer = 10; // 重置ping发送定时器
    pThis->ping_req.retry_cnt = 0;
    pThis->ping_req.sent_flag = M2M_FALSE;
    colt_info.m2m_online_flag = M2M_TRUE;
  }

  return 0;
}

/*******************************************************************************
 * 心跳响应消息处理
 *******************************************************************************/
uint16_t im2m_AnalyzePingRspMsg(m2m_context_t* pThis)
{
  pThis->ping_req.retry_cnt = 0;
  pThis->ping_req.sent_flag = 0;
  pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp;

  return 0;
}

#if (PART("平台下发的命令请求(CMD_REQ)"))
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

//==构建命令响应消息(设定参数)=================================================
uint16_t im2m_BuildCmdRspPwMsg(m2m_context_t* pThis, uint16_t* fail_tag_tbl, uint8_t fail_tag_num)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len;
  uint8_t it;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==响应的原报文流水号==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==命令类型========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'P';
  pbuf[len++] = 'W';

  //==命令响应内容====
  if (fail_tag_num == 0)
  {
    pbuf[len++] = 0x00; // 长度
    pbuf[len++] = 0x01;
    pbuf[len++] = 0x00; // 执行结果:0x00=成功
  }
  else
  {
    msg_len = 1+1+2*fail_tag_num; // 命令响应内容长度
    pbuf[len++] = (msg_len>>8) & 0xFF; // 长度
    pbuf[len++] = (msg_len & 0xFF);
    pbuf[len++] = 0x01;         // 执行结果:0x01=失败
    pbuf[len++] = fail_tag_num; // 执行失败的TLV个数
    for (it=0; it<fail_tag_num; it++) // 执行失败的TLV的TAG
    {
      pbuf[len++] = (fail_tag_tbl[it]>>8) & 0xFF;
      pbuf[len++] = (fail_tag_tbl[it] & 0xFF);
    }
  }
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==处理命令请求(设定参数)====================================================
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

  if (pThis->cmdBodyLen < 5) // 长度错误
  {
    return 0;
  }

  tlvNum = pbuf[pos++];
  while (tlvNum) // TLV解析
  {
    if (pos >= pThis->cmdBodyLen) // 长度判断
    {
      break; // 长度错误,退出循环
    }

    //==获取TAG==================================
    tag = pbuf[pos++];
    tag <<= 8;
    tag += pbuf[pos++];

    //==获取LENGTH===============================
    length = pbuf[pos++];
    length <<= 8;
    length += pbuf[pos++];

    //==获取VALUE================================
    retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // 解析
    if (M2M_FALSE == retVal) // 解析失败
    {
      fail_tag_tbl[fail_tag_num] = tag;
      fail_tag_num++;
    }
    pos += length; // 指向下一个TLV
    tlvNum--;
  }

  Parm_SaveM2mAssetData();  // 保存参数到FLASH
  //if (1==save_gb_flag)
  //  SaveAllGBPara();

  pThis->tx_size = im2m_BuildCmdRspPwMsg(pThis, fail_tag_tbl, fail_tag_num); // 构建命令响应消息
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
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

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==响应的原报文流水号==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==命令类型========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'P';
  pbuf[len++] = 'R';

  //==命令响应内容====
  msg_len = tlvTotalLen+1;
  pbuf[len++] = (msg_len>>8) & 0xFF; // 命令响应内容长度
  pbuf[len++] = (msg_len & 0xFF);
  pbuf[len++] = tlvNum; // TLV个数
  len += tlvTotalLen;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==处理命令请求(读取参数)====================================================
uint16_t im2m_AnalyzeCmdReqPrMsg(m2m_context_t* pThis)
{
  uint8_t* pTlvBuf = pThis->tx_data + M2M_MSG_HEAD_LEN + 9; // TLV消息缓存
  uint16_t tlvTotalLen = 0; // TLV消息累计长度
  uint8_t txTlvNum = 0;
  uint8_t rxTlvNum = 0;
  uint16_t tag = 0;
  uint16_t tlvLen = 0;
  uint16_t pos = 0;

  rxTlvNum = pThis->pCmdBody[pos++]; // tlv个数
  while (rxTlvNum)
  {
    tag = pThis->pCmdBody[pos++];  // 获取TLV的TAG
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

  pThis->tx_size = im2m_BuildCmdRspPrMsg(pThis, txTlvNum, tlvTotalLen); // 构建命令响应消息

  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
  }

  return pThis->tx_size;
}

//==构建CMD响应命令LT======================================================
uint16_t im2m_BuildCmdRspLtMsg(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf= pThis->tx_data;
  uint16_t len=0;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==响应的原报文流水号==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==命令类型========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'L';
  pbuf[len++] = 'T';

  //==无命令响应内容====
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==处理命令请求(位置追踪)=====================================================
uint16_t im2m_AnalyzeCmdReqLtMsg(m2m_context_t* pThis)
{
  uint8_t pos = 0;
  uint8_t lt_mode;
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  if (pThis->rx_from != SYSBUS_DEVICE_TYPE_PCDEBUG) // 根据平台下发设置追踪模式
  {
    lt_mode = pThis->pCmdBody[pos++]; // 追踪模式
    if (lt_mode==0xFF) // 单次追踪
    {
      pThis->lt_report.mode = 0x00;
      pThis->lt_report.timer_sp = 1;
      pThis->lt_report.total_time_sp = 1;
    }
    else if (lt_mode==0x00) // 等时间间隔追踪
    {
      pThis->lt_report.mode = 0x00;
      pThis->lt_report.timer_sp = pThis->pCmdBody[pos++];
      pThis->lt_report.total_time_sp = (uint16_t)(pThis->pCmdBody[pos++]*60); // 60
    }
    else if (lt_mode==0x01) // 等距离间隔追踪
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

  pThis->tx_size = im2m_BuildCmdRspLtMsg(pThis); // 构建LT响应消息
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
    M2M_DELAY(OS_TICKS_PER_SEC/50); // 延时20ms
    
    //pThis->tx_size = im2m_BuildStatusSyncMsg(pThis->tx_data);
    //==创建报文体==========================================
    msg_body_len = im2m_BuildStatusSyncBody(&pbuf[M2M_MSG_HEAD_LEN]);
    
    //==创建报文头==========================================
    msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PUSH_DATA, msg_body_len, 0, pThis->rx_sn);
    
    //==计算校验字==========================================
    msg_len = msg_body_len + msg_header_len;
    check_sum = im2m_CalcSumCheck(pbuf, msg_len);
    pbuf[msg_len++] = check_sum;
    pThis->tx_size = msg_len;

    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
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

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==响应的原报文流水号==
  pbuf[len++] = (pThis->rx_sn>>8)&0xFF;
  pbuf[len++] = (pThis->rx_sn & 0xFF);

  //==命令类型========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'R';
  pbuf[len++] = 'C';

  //==命令响应内容====
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x05;
  pbuf[len++] = result; // 执行结果
  temp_val = im2m_BuildDeviceStatus();
  pbuf[len++] = (temp_val>>24) & 0xFF; // 终端状态字
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_CMD_RESP, msg_body_len, 0, pThis->rx_sn);

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}

//==处理命令请求(远程控制)=====================================================
uint16_t im2m_AnalyzeCmdReqRcMsg(m2m_context_t* pThis)
{
  // 每一个远程控制命令只携带一个TLV
  uint16_t tag;
  uint16_t length;
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t pos = 0;
  uint8_t retVal;
  uint8_t result;

  //==获取TAG==================================
  tag = pbuf[pos++];
  tag <<= 8;
  tag += pbuf[pos++];

  //==获取LENGTH===============================
  length = pbuf[pos++];
  length <<= 8;
  length += pbuf[pos++];

  //==获取VALUE================================
  retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // 解析
  result = retVal? 0x00 : 0x01; // 0x00=成功, 0x01=失败

  pThis->tx_size = im2m_BuildCmdRspRcMsg(pThis, result); // 构建RC响应消息
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PCDEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
    if(pThis->ss_req.sent_flag==M2M_FALSE)
    {
      pThis->ss_req.send_timer = 0x05;
    }
  }

  return pThis->tx_size;
}

//==处理命令请求(AT指令透传)===================================================
uint16_t im2m_AnalyzeCmdReqAtMsg(m2m_context_t* pThis)
{
#if 0
  uint8 *p = pCmdBody;
  uint8 *pBuf = &aMsgSendData[MSG_HEAD_LEN];

  if (pThis->cmdBodyLen > 0)
  {
    if (0==memcmp("t6", p, 2))	//盲区
    {
      EnableBlindSimulator();
    }
    else if (0==memcmp("obd", p, 3))	//模拟OBD数据
    {
      EnableOBDSimulator();
    }
    else if (0==memcmp("sign", p, 4))	//需要数字签名
    {
      EncryptInfor.signable=1;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("unsign", p, 6))	//不需要数字签名
    {
      EncryptInfor.signable=0;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("sec", p, 3))	//需要加密
    {
      EncryptInfor.secretable=1;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("unsec", p, 5))	//不需要加密
    {
      EncryptInfor.secretable=0;
      Save_EncryptToEEPROM();
    }
    else if (0==memcmp("AT+", p, 3))	//不需要加密
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
 * 处理平台发来的命令请求消息
 *******************************************************************************/
uint16_t im2m_AnalyzeCmdReqMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0;
  uint8_t cmd_req_type;
  uint8_t* pbuf;

  pThis->cmdTypeLen = (uint16_t)(pThis->rx_body_data[0]<<8)+pThis->rx_body_data[1]; // 命令类型长度
  pThis->pCmdType = &pThis->rx_body_data[2]; // 命令类型
  if ((pThis->cmdTypeLen>0) && (pThis->cmdTypeLen<=pThis->rx_bodyLen))
  {
    pbuf = pThis->pCmdType + pThis->cmdTypeLen; // 命令内容长度地址
    pThis->cmdBodyLen = (uint16_t)(pbuf[0]<<8)+pbuf[1]; // 命令内容长度
    pThis->pCmdBody = pbuf+2; // 命令内容

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

#if (PART("平台下发升级消息"))
/******************************************************************************
 * 获取升级消息类型
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

//==构建UN响应RFU通知消息======================================================
uint16_t im2m_BuildUpdateRspUnMsg(m2m_context_t* pThis, uint8_t result)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;
  uint16_t len = 0;

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==命令类型========
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';
  pbuf[len++] = 'N';

  //==命令响应内容====
  pbuf[len++] = result; // 执行结果:0x00=成功
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE_ACK, msg_body_len, 0, pThis->rx_sn);

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;

  return msg_len;
}


//==平台主动下发远程固件升级通知(平台->终端)===================================
uint16_t im2m_AnalyzeUpdateReqUnMsg(m2m_context_t* pThis)
{
  uint16_t tag;
  uint16_t length;
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t pos = 0;
  uint8_t retVal;
  uint16_t tlvTypeList[] = {0x1002,0x1003,0x100E,0x100C,0x1005}; // 固定TLV解析顺序表
  uint8_t tlvNum = 0;
  uint8_t it=0;
  uint8_t err_flag = 0;
  
  if ((pThis->update.state>M2M_UPDATE_STATE_IDLE) && (FSRV_GetState()>FSRV_STATE_IDLE)) // 原有升级未完成,拒绝新升级请求
  {
    return 0;
  }

  pThis->cmdBodyLen = pbuf[pos++]; // 命令内容长度
  pThis->cmdBodyLen <<= 8;
  pThis->cmdBodyLen += pbuf[pos++];
  
  rfu_context.type = pbuf[pos++]; // 升级类型
  rfu_context.dev= pbuf[pos++];   // 升级设备

  // 解析TLV
  tlvNum = sizeof(tlvTypeList)/sizeof(uint16_t);
  for(it=0; it<tlvNum; it++)
  {
    //==获取TAG==================================
    tag = pbuf[pos++];
    tag <<= 8;
    tag += pbuf[pos++];
    
    //==获取LENGTH===============================
    length = pbuf[pos++];
    length <<= 8;
    length += pbuf[pos++];

    if(tlvTypeList[it] != tag)
    {
      err_flag = 1;
      break;
    }
    
    retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // 解析
    if(M2M_FALSE == retVal)
    {
      err_flag = 1;
      break;
    }
    
    pos += length;
  }

  //==判断固件类型================================
  if(memcmp(rfu_context.file_name, "ZXM2M_4G", 8)==0) // 4G程序(以文件名为准)
  {
    rfu_context.dev = 0x00;
  }
  else if(memcmp(rfu_context.file_name, "ZXM2M_ST", 8)==0)  // 协处理器程序(以文件名为准)
  {
    rfu_context.dev = 0x03;
  }
  else
  {
    if(rfu_context.dev==0x00)  // 固件类型错误
    {
      err_flag = 1;
    }
  }
  //==============================================

  pThis->tx_size = im2m_BuildUpdateRspUnMsg(pThis, err_flag); // 构建响应消息
  im2m_SendNetData(pThis->tx_data, pThis->tx_size);

  M2M_DELAY(OS_TICKS_PER_SEC/10); // 延时100ms

  if(err_flag==0) // 无错误
  {
    im2m_ConnectUpdateServer();
    pThis->update.rsp_timeout_timer = 60;  //1分钟没有连接上就算失败
    pThis->update.state = M2M_UPDATE_STATE_NOTIFIED;
  }
  else
  {
    pThis->update.state = M2M_UPDATE_STATE_IDLE;
  }
  
  return 0;
}

//==平台响应终端固件升级请求(平台->终端)=======================================
uint16_t im2m_AnalyzeUpdateRspUqMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t len = 0;

  if (0 == pbuf[len++]) // 0x00=请求成功
  {
    // 升级文件的大小(BYTE)
    rfu_context.file_length = pbuf[len++]; // 最高字节
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // 中间高字节
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // 中间低字节
    rfu_context.file_length <<= 8;
    rfu_context.file_length += pbuf[len++]; // 最低字节

    // CRC32校验码
    rfu_context.plain_crc32val = pbuf[len++]; // 最高字节
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // 中间高字节
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // 中间低字节
    rfu_context.plain_crc32val <<= 8;
    rfu_context.plain_crc32val += pbuf[len++]; // 最低字节

    // 文件总包数
    rfu_context.total_block_count = pbuf[len++]; // 最高字节
    rfu_context.total_block_count <<= 8;
    rfu_context.total_block_count += pbuf[len++]; // 最低字节
    rfu_context.block = 0;

    rfu_EraseFlashHexFile(&rfu_context);

    pThis->update.retry_cnt = 0;
    pThis->update.rsp_timeout_timer = 5;
    pThis->update.state = M2M_UPDATE_STATE_DOWNLOAD_REQ;
  }

  return 0;
}

//==平台响应终端请求下载升级包(平台->终端)处理函数=============================
uint16_t im2m_AnalyzeUpdateRspUlMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;
  uint16_t len=0;
  uint16_t current_block_index;  // 当前包序
  uint16_t total_block_num;  // 总包数量
  uint16_t packet_len;
  uint8_t rfu_status;

  pThis->cmdBodyLen = pbuf[len++]; // 命令内容长度
  pThis->cmdBodyLen <<= 8;
  pThis->cmdBodyLen += pbuf[len++];
  if (pThis->cmdBodyLen < 4) // 命令内容长度判断
  {
    return 0;
  }

  //==当前包序列号========
  current_block_index = pbuf[len++];
  current_block_index <<= 8;
  current_block_index += pbuf[len++];
  if(current_block_index != rfu_context.block)
  {
    return 0;
  }

  //==文件总包数==========
  total_block_num = pbuf[len++];
  total_block_num <<= 8;
  total_block_num += pbuf[len++];
  if(total_block_num != rfu_context.total_block_count)
  {
    return 0;
  }

  //==本包固件数据长度====
  packet_len = pbuf[len++];
  packet_len <<= 8;
  packet_len += pbuf[len++];
  if (packet_len == 0)
  {
    return 0;
  }

  rfu_SaveFlashHexFile(&rfu_context, &pbuf[len], packet_len);  // 将BIN文件写入SPI FLASH
  rfu_context.block++;
  rfu_context.percent = rfu_context.block * 100L / rfu_context.total_block_count;
  if (current_block_index==(total_block_num-1)) // 传输结束(最后一包)
  {
    rfu_status = rfu_CheckNewFirmware(&rfu_context, rfu_data_buffer, RFU_BUFFER_SIZE);
    if (RFU_OK==rfu_status)
    {
      pThis->update.result = 0;
      // 固件类型判断
      if(rfu_context.dev==0x00)  //==终端
      {
        PcDebug_SendString("RfuCrc:4G-Ok!\n");
      }
      else
      {
        if(rfu_context.dev==0x01)  //==控制器
        {
          PcDebug_SendString("RfuCrc:CTL-Ok!\n");
        }
        else if(rfu_context.dev==0x02)  //==显示器
        {
          PcDebug_SendString("RfuCrc:LCD-Ok!\n");
        }
        else if(rfu_context.dev==0x03)  //==协处理器
        {
          PcDebug_SendString("RfuCrc:ST-Ok!\n");
        }
        
        FSRV_SetState(FSRV_STATE_START);  // 启动外部程序升级
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

//==平台响应终端上报升级结果(平台->终端)==============================================
uint16_t im2m_AnalyzeUpdateRspUrMsg(m2m_context_t* pThis)
{
  uint8_t *pbuf = pThis->pCmdBody;

  if ((0==pbuf[0]))  // 平台收到终端上报的升级结果
  {
    pThis->update.state = M2M_UPDATE_STATE_IDLE;
    
    if(0==pThis->update.result)  // 下载BIN文件成功
    {
      if(rfu_context.dev==0x00)  // 终端程序
      {
        // Tbox_SetMachineState(TBOX_STATE_IAP); // T-BOX进入升级模式
      }
    }
  }

  return 0;
}

/*******************************************************************************
 * 升级通知和升级响应消息处理
 *******************************************************************************/
uint16_t im2m_AnalyzeUpdateMsg(m2m_context_t* pThis)
{
  uint16_t frame_len = 0;
  uint8_t update_msg_type;

  pThis->cmdTypeLen = (uint16_t)(pThis->rx_body_data[0]<<8)+pThis->rx_body_data[1]; // 命令类型长度
  pThis->pCmdType = &pThis->rx_body_data[2]; // 命令类型
  if ((pThis->cmdTypeLen>0) && (pThis->cmdTypeLen<=pThis->rx_bodyLen))
  {
    pThis->pCmdBody = pThis->pCmdType + pThis->cmdTypeLen; // 命令内容地址
    update_msg_type = im2m_GetUpdateType((const char *)pThis->pCmdType, pThis->cmdTypeLen);
    switch (update_msg_type)
    {
    case M2M_UPDATE_TYPE_UN:
      frame_len = im2m_AnalyzeUpdateReqUnMsg(pThis); // 平台主动下发
      break;

    case M2M_UPDATE_TYPE_UQ:
      frame_len = im2m_AnalyzeUpdateRspUqMsg(pThis); // 平台被动响应
      break;

    case M2M_UPDATE_TYPE_UL:
      frame_len = im2m_AnalyzeUpdateRspUlMsg(pThis); // 平台被动响应
      break;

    case M2M_UPDATE_TYPE_UR:
      frame_len = im2m_AnalyzeUpdateRspUrMsg(pThis); // 平台被动响应
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

  //==最小长度判断===================================
  if (pThis->rx_size < 14)
  {
    PcDebug_SendData((uint8_t *)("ERR:011\n"), 7, DBG_MSG_TYPE_ANYDATA);
    PcDebug_SendData(pThis->rx_data, pThis->rx_size, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==长度判断=======================================
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

  //==校验和判断=====================================
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

  //==解析报文头========================================
  pThis->rx_msg_type = pThis->rx_data[M2M_MSG_TYPE_FIELD]; // 接收消息类型
  memcpy(pThis->rx_devId, &pThis->rx_data[M2M_DEV_ID_FIELD] ,7); // 接收终端ID
  pThis->rx_flag = pThis->rx_data[M2M_FLAG_FIELD]; // 接收厂家编号
  pThis->rx_sn = pThis->rx_data[M2M_SN_FIELD]; // 命令序号
  pThis->rx_sn <<= 8;
  pThis->rx_sn += pThis->rx_data[M2M_SN_FIELD+1];
  pThis->rx_bodyLen = pThis->rx_data[M2M_BODY_LEN_FIELD]; // 消息体长度
  pThis->rx_bodyLen <<= 8;
  pThis->rx_bodyLen += pThis->rx_data[M2M_BODY_LEN_FIELD+1];
  pThis->rx_bodyLen -= 1; // 不含校验和的长度
  pThis->rx_body_data = pThis->rx_data + M2M_MSG_HEAD_LEN;

  //==解析报文体========================================
  switch (pThis->rx_msg_type)
  {
  case M2M_MSG_TYPE_MESSAGEACK: // 通用报文应答
    frame_len = im2m_AnalyzeAckMsg(pThis);
    break;

  case M2M_MSG_TYPE_CONN_RESP: // 平台响应连接
    frame_len = im2m_AnalyzeConnRespMsg(pThis);
    break;

  case M2M_MSG_TYPE_CMD_REQ: // 平台发来的命令请求
    frame_len = im2m_AnalyzeCmdReqMsg(pThis);
    break;

  case M2M_MSG_TYPE_PING_RESP: // 平台响应心跳
    frame_len = im2m_AnalyzePingRspMsg(pThis);
    break;

  case M2M_MSG_TYPE_UPDATE: // 平台发来的升级通知
  case M2M_MSG_TYPE_UPDATE_ACK: // 平台发来的升级响应
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

  pThis->frame_status = im2m_CheckRecvMsg(pThis); // 校验
  if (pThis->frame_status == M2M_OK)
  {
    //_m2m_disable_interrutps();
    frame_len = im2m_ProcessRecvMsg(pThis); // 处理
    //_m2m_enable_interrutps();
    //Usart3DmaSendPacket(pThis->data,pThis->tx_size);
  }

  return frame_len;
}

/******************************************************************************
 * 10ms周期调用
*******************************************************************************/
void M2M_ProduceSendMsg(m2m_context_t* pThis)
{
  //int32_t retVal;
  //static uint8_t report_blind_zone_timer = 50;
  uint8_t acc_state;
  static uint32_t acc_on_timer_10ms = 0x00;
  static uint32_t acc_off_timer_10ms = 0x00;
  static uint8_t send_tcb_flag = 1; // 发送版本信息标志
  static uint8_t send_tct_flag = 0; // 发送统计信息标志

  acc_state = COLT_GetAccStatus();
  //==ACC状态计时=============================================================
  if (acc_state) // ACC开
  {
    acc_on_timer_10ms++;
    acc_off_timer_10ms = 0x00;
    send_tct_flag = 1;  // 需要发送统计信息
  }
  else
  {
    acc_off_timer_10ms++;
    acc_on_timer_10ms = 0x00;
    send_tcb_flag = 1;  // 需要发送版本信息
  }

  //==平台未连接======
  if (im2m_GetLinkState() == SOCKET_LINK_STATE_CLOSED)
  {
    pThis->conn_success_flag = M2M_FALSE;
    pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp; // 重置心跳发送时间
    if (M2M_TRUE==colt_info.m2m_online_flag)
    {  colt_info.m2m_online_flag = M2M_FALSE;}
  }

  // ACC开
  if (acc_state==1)
  {
    //ZxM2mBlindZone_Service();  // 记录盲区数据
  }

  if (im2m_GetLinkState() != SOCKET_LINK_STATE_READY)
  {
    //report_blind_zone_timer = 50;
    return;
  }

  //==远程升级=================================================================
  if (pThis->update.state > M2M_UPDATE_STATE_IDLE)
  {
    im2m_SendUpdateMsg(pThis);
    return;
  }

  //==连接平台=================================================================
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
        iZxM2m_SendConnenctMsg(pThis);  // 发送重型连接指令
        
        pThis->ss_req.retry_cnt = 0;
        pThis->ss_req.sent_flag = M2M_FALSE;
      }
    }
    return; // 未发送连接请求前,不可发送其他类型请求
  }

  //==状态同步================================================================
  if ((pThis->ss_req.send_timer==0x00) && (pThis->ss_req.sent_flag==M2M_FALSE))
  {
    pThis->ss_req.retry_cnt++;
    pThis->ss_req.sent_flag = M2M_TRUE;
    pThis->ss_req.rsp_timeout_timer = pThis->ss_req.rsp_timeout_sp;
    pThis->ss_req.send_timer = m2m_asset_data.ss_report_time_sp;
    //im2m_SendStatusSyncMsg(pThis);
    iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCS);  // 发送重型终端数据
    return;
  }

  //==工况数据================================================================
  if ((pThis->tc_req.send_timer==0x00) && (pThis->tc_req.sent_flag==M2M_FALSE))
  {
    pThis->tc_req.retry_cnt++;
    pThis->tc_req.sent_flag = M2M_TRUE;
    pThis->tc_req.rsp_timeout_timer = pThis->tc_req.rsp_timeout_sp;
    pThis->tc_req.send_timer = m2m_asset_data.hb_timer_sp; // 使用心跳设置值做为发送时间
    pThis->ping_req.send_timer = pThis->ping_req.send_timer_sp; // 重置心跳发送时间

    if((send_tcb_flag==1) && (acc_on_timer_10ms>18000))  // 发送重型版本数据(AccOn,40s后发一次)
    {
      send_tcb_flag = 0;
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCB);
    }
    else if((send_tct_flag==1) && (acc_off_timer_10ms>4000))  // 发送重型统计数据(AccOff,30s后发一次)
    {
      send_tct_flag = 0;
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCT);
    }
    else  // 发送重型工况数据(AccOn 每10s发一次)
    {
      iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCW);
    }
    return;
  }

  //iZxM2m_SendTcMsg(pThis, ZXTC_MSG_TYPE_TCD);  // 发送重型故障数据(有故障立即发送)

  //==心跳指令================================================================
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
  //==盲区补偿==============================================================
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
  //向平台发送报警数据
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
 * M2M周期性调用函数,1S调用一次,无阻塞
*******************************************************************************/
void M2M_Do1sTasks(void)
{
  uint8_t timeout_flag; // 超时标志

  //==连接管理====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.conn_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M CONN FAILD!\n");
    im2m_ReconnectLink();
  }

  //==状态同步管理================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.ss_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M SS REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==工况数据管理================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.tc_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M TC REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==盲区补偿管理================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.bz_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M BZ REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==心跳管理====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.ping_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M PING REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==告警管理====================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.alarm_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M ALARM REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==故障码管理==================================
  timeout_flag = im2m_ReqTimeOutManage(&m2m_context.dtc_req);
  if (timeout_flag==M2M_TRUE)
  {
    PcDebug_SendString("M2M DCT REQ:Fail!\n");
    im2m_ReconnectLink();
  }

  //==远程固件升级管理===========================
  im2m_UpdateTimeOutManage();

  //==重连新服务器管理==========================
  im2m_ConnectNewSrvManage();
}

/******************************************************************************
 * 功能: 当报警产生或消失时，添加到报警列表中来
 * 输入: type:报警类型
 *       flag:产生与消失标识,1=产生(FAIl), 0=消失(NORMAL)
*******************************************************************************/
void M2M_AddNewAlarmToList(uint8_t type, uint8_t flag)
{
  uint8_t tempVal;

  if (1==flag)
    tempVal = type | BIT(8); // 故障产生
  else
    tempVal = type; // 故障消失

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

  // 连接请求
  m2m_context.conn_req.sent_flag = M2M_FALSE;
  m2m_context.conn_req.send_timer_sp = DEFAULT_CONN_DATA_SEND_PERIOD_SP;
  m2m_context.conn_req.rsp_timeout_sp = DEFAULT_CONN_DATA_RSP_TIMEOUT_SP;
  m2m_context.conn_req.retry_sp = DEFAULT_CONN_DATA_RETRY_SP;

  // 状态同步请求
  m2m_context.ss_req.sent_flag = M2M_FALSE;
  m2m_context.ss_req.send_timer_sp = DEFAULT_SS_DATA_SEND_PERIOD_SP;
  m2m_context.ss_req.rsp_timeout_sp = DEFAULT_SS_DATA_RSP_TIMEOUT_SP;
  m2m_context.ss_req.retry_sp = DEFAULT_SS_DATA_RETRY_SP;

  // 工况数据请求
  m2m_context.tc_req.sent_flag = M2M_FALSE;
  m2m_context.tc_req.send_timer_sp = DEFAULT_TC_DATA_SEND_PERIOD_SP;
  m2m_context.tc_req.rsp_timeout_sp = DEFAULT_TC_DATA_RSP_TIMEOUT_SP;
  m2m_context.tc_req.retry_sp = DEFAULT_TC_DATA_RETRY_SP;

  // 盲区补偿请求
  m2m_context.bz_req.sent_flag = M2M_FALSE;
  m2m_context.bz_req.send_timer_sp = DEFAULT_BZ_DATA_SEND_PERIOD_SP;
  m2m_context.bz_req.rsp_timeout_sp = DEFAULT_BZ_DATA_RSP_TIMEOUT_SP;
  m2m_context.bz_req.retry_sp = DEFAULT_BZ_DATA_RETRY_SP;

  // 心跳请求
  m2m_context.ping_req.sent_flag = M2M_FALSE;
  //m2m_context.ping_req.send_timer_sp = DEFAULT_PING_DATA_SEND_PERIOD_SP;
  m2m_context.ping_req.send_timer_sp = m2m_asset_data.hb_timer_sp;
  m2m_context.ping_req.rsp_timeout_sp = DEFAULT_PING_DATA_RSP_TIMEOUT_SP;
  m2m_context.ping_req.retry_sp = DEFAULT_PING_DATA_RETRY_SP;

  // 告警请求
  m2m_context.alarm_req.sent_flag = M2M_FALSE;
  m2m_context.alarm_req.send_timer_sp = DEFAULT_ALARM_DATA_SEND_PERIOD_SP;
  m2m_context.alarm_req.rsp_timeout_sp = DEFAULT_ALARM_DATA_RSP_TIMEOUT_SP;
  m2m_context.alarm_req.retry_sp = DEFAULT_ALARM_DATA_RETRY_SP;

  // 故障码请求
  m2m_context.dtc_req.sent_flag = M2M_FALSE;
  m2m_context.dtc_req.send_timer_sp = DEFAULT_DTC_DATA_SEND_PERIOD_SP;
  m2m_context.dtc_req.rsp_timeout_sp = DEFAULT_DTC_DATA_RSP_TIMEOUT_SP;
  m2m_context.dtc_req.retry_sp = DEFAULT_DTC_DATA_RETRY_SP;

  // 远程升级
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


