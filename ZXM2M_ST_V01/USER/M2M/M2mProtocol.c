/*****************************************************************************
* @FileName: M2mProtocol.c
* @Engineer: TenYan & ZPY
* @version   V1.0
* @Date:     2020-10-20
* @brief     M2M 协议实现
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

///实际发送函数
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

  .vin_valid_flag = 0,            // VIN码无效
  .vin = "LXGBPA123test0088",     // VIN码
  //.vin_valid_flag = 0,            // VIN码无效
  .ecu_type = ENGINE_TYPE_WEICHAI,// 发动机类型
  .ep_type = EP_TYPE_HJ,          // 环保类型
};

/******************************************************************************
*
******************************************************************************/
#if (PART("M2M构建和解析TLV"))
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

//==构造终端状态位=========================================================
uint32_t im2m_BuildDeviceStatus(void)
{
  uint32_t uiTemp = 0;

//  if (1==GPS_GetPositioningStatus()) // 定位状态
//  {
//    uiTemp |= BIT(31);
//  }

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

//  if (GPS_GetModuleStatus()) // GPS模块故障标志(0:无故障, 1:故障)
//  {
//    uiTemp |= BIT(19);
//  }
//  if (GPS_GetAntOpenStatus()) // GPS天线断开标志(0:未断开, 1:断开)
//  {
//    uiTemp |= BIT(18);
//  }
//  if (GPS_GetAntShortStatus()) // GPS天线短路标志(0:未短路, 1:短路)
//  {
//    uiTemp |= BIT(17);
//  }
  
  if (CAN_NOK==CAN_GetCommState(CAN_CHANNEL1)) // CAN总线通信中断标志(0:未中断 1:中断)
  {
    uiTemp |= BIT(16);
  }

  // BIT15-内置锁车继电器标志
  // 14~8预留

//  if (GPS_GetSpeedOverrunStatus())  // 超速标志(0:未超速, 1:已超速)
//  {
//    uiTemp |= BIT(7);
//  }

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

//==终端重启控制(RC)======================================================
static uint16_t im2m_AnalyzeTlvMsg_4000(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;
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

//==更换终端==============================================================
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

  {0xA1FE, im2m_BuildTlvMsg_A1FE, im2m_AnalyzeTlvMsg_A1FE},//发送机类型
  {0xA1FF,                   NULL, im2m_AnalyzeTlvMsg_A1FF},//更换终端
};
#define NUM_OF_M2M_CMD_DEAL   (sizeof(m2m_CmdDealTbl)/sizeof(im2m_CmdTlv_t))

#endif

#if (PART("M2M消息组建函数"))
/*************************************************************************
 * 计算校验和
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
#endif

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
  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PC_DEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    //im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
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
uint16 im2m_AnalyzeCmdReqPrMsg(m2m_context_t* pThis)
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

  if (pThis->rx_from == SYSBUS_DEVICE_TYPE_PC_DEBUG)
  {
    PcDebug_SendM2mRsp(pThis->tx_data, pThis->tx_size); // 配置工具
  }
  else
  {
    //im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台
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
  pbuf[len++] = result; // 执行结果
  pbuf[len++] = 0x00;
  pbuf[len++] = 0x05;
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

  //==最小长度判断===================================
  if (pThis->rx_size < 14)
  {
    PcDebug_SendData((uint8_t *)("ERR:011"), 7, DBG_MSG_TYPE_ANYDATA);
    PcDebug_SendData(pThis->rx_data, pThis->rx_size, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==长度判断=======================================
  msg_body_len = pThis->rx_data[M2M_BODY_LEN_FIELD];
  msg_body_len <<= 8;
  msg_body_len += pThis->rx_data[M2M_BODY_LEN_FIELD+1];
  if (pThis->rx_size != (msg_body_len+M2M_MSG_HEAD_LEN))
  {
    PcDebug_SendData((uint8_t *)("ERR:014"), 7, DBG_MSG_TYPE_ANYDATA);
    return M2M_NOK;
  }

  //==校验和判断=====================================
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
  case M2M_MSG_TYPE_CMD_REQ: // 平台发来的命令请求
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

  pThis->frame_status = im2m_CheckRecvMsg(pThis); // 校验
  if (pThis->frame_status == M2M_OK)
  {
    //_m2m_disable_interrutps();
    frame_len = im2m_ProcessRecvMsg(pThis); // 处理
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
 * M2M周期性调用函数,1S调用一次,无阻塞
*******************************************************************************/
void M2M_Do1sTasks(void)
{

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




