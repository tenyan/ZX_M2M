/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: Collect.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version:  V1.0
* @Date:     2020-6-4
* @brief:
******************************************************************************/
#ifndef _COLLECT_H_
#define _COLLECT_H_

/******************************************************************************
 * Pre-Processor Defines
 ******************************************************************************/


/******************************************************************************
 * Macros
 ******************************************************************************/


/******************************************************************************
 * Data Types
 ******************************************************************************/
//采集状态
typedef struct
{
  uint8_t acc_off_event_flag; // ACC由开到关事件标志: 0=未发生, 1=发生
  
  uint8_t m2m_online_flag;  // M2M连接上线标志: 0=未上线, 1=已上线
  uint8_t hjep_online_flag; // HJEP连接上线标志: 0=未上线, 1=已上线
  uint8_t gbep_online_flag; // GBEP连接上线标志: 0=未上线, 1=已上线
  
  uint8_t powerr_off_timer; // 设备收到关机命令后等待关机的时间,定位5秒
  uint8_t restart_timer;    // 当服务器或者短信下发复位命令时, 需要延时x秒后复位系统
  uint8_t wdt_status;  // 单片机内部看门狗状态:0x00=未开启,0xAA=已开启
  uint8_t tobx_state;  // T-Box工作状态:0=正常, 1=休眠, 2=掉电, 3=运输
  uint32_t total_work_time;  // 累计工作时间,单位:s

  uint8_t save_offline_time_flag;  // 存储离线时间标志
  uint32_t total_offline_time;  // 累计不上线时间

  uint16_t vbat;            // 内部锂电池电压,单位10mV
  uint16_t vraw;            // 外电源电压,单位10mV
  int8_t int_temp;         // 内部温度
  uint8_t switch1;// 开关量采集1
  //B0：报警开关    0=低电平,1=高电平
  //B1：钥匙开关ACC 0=关闭,1=打开
  //B2：小时计开关  0=关闭,1=打开；
  //B3：外接电正极状态 0=断开,1=联通
  //B4：外接电负极状态 0=联通,1=断开
  //B5：搭铁极状态     0=联通,1=断开
  //B6：GSM天线报警    0=无报警, 1=报警
  //B7：保留位

  uint8_t switch2;//开关量采集2
  //B0：继电器1控制状态 0=未锁车;1=锁车
  //B1：继电器2控制状态 0=未锁车;1=锁车
  //B2：保留
  //B3：开盒报警 0=正常;1=报警
  //B4：保留
  //B5：RS232通讯状态 0= 未通讯；1=通讯；
  //B6：RS232通讯仪表提示状态 0= 未通讯；1=通讯
  //B7：CAN通讯状态 0= 未通讯；1=通讯；
  uint8_t switch3;//开关量采集3
  //B0：供电状态         0=外电供电；1=电池供电
  //B1：外电馈电状态     0=不馈电；1=馈电
  //B2：内部电池充电状态 0=未充电,1=充电
  //B3：车辆行驶速度报警 0=速度正常；1=超速报警
  //B4：车辆位置越界报警 0=不越界；1= 越界报警
  //B5：内置电池电压低   0=不馈电；1=馈电
  //B6：保留
  //B7：保留

  uint8_t alarm;	// 防拆类开关量
  //B0：ST看门狗状态:    0=未开启, 1=开启
  //B1：开盒报警1       0=正常,     1=报警
  //B2：开盒报警2       0=正常,     1=报警
  //B3：GSM天线报警,Reserved
  //B4: GSM卡盖检测     0=正常,  1=拆除
  //B5: GPS天线短路		  0=短路,  1=正常
  //B6: GPS天线开路		  0=正常,  1=开路
  //B7: Reserved
  
  uint8_t open_box_flag;  // 0:未开盖, 1:已开盖
  
  uint8_t gpsAntShortStatus; // 0:正常, 1:短路
  uint8_t gpsAntOpenStatus;  // 0:正常, 1:开路
  uint8_t gpsAntStatus;      // 0:正常, 1:故障
  uint8_t gpsFixValid;    // 0:未定位，1:已定位

  uint8_t correct_rtc_flag; // RTC校时标志: 0=未校时, 1=已校时
  uint8_t sim_card_status; // SIM卡状态: 0=未识别, 1=已识别
  uint8_t net_4g_status;  // 4G模块数据状态: 0=未联网, 1=已联网
  uint8_t gps_4g_status;  // 4G模块上的GPS定位状态: 0=未定位, 1=已定位
  uint8_t wifi_status;  // WIFI工作状态: 0=未工作, 1=已工作
  uint8_t eth_status;  // ETH工作状态: 0=未工作, 1=已工作
  uint8_t rfu_flag;  // 4G模块下载固件(RFU)标志: 0=未升级, 1=正在升级
}colt_info_t;
extern colt_info_t colt_info;

enum
{
  GPS_OK = 0x00,
  GPS_NOK = 0x01
};

enum
{
  RTC_NOK = 0x00,
  RTC_OK = 0x01
};

/******************************************************************************
 * Application Specific Globals
 ******************************************************************************/


/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void get_switchs_status(void);
void get_lipo_battery_status(void);
void get_main_power_status(void);
void get_micro_temperature(void);
void control_battery_charge(void);

#endif
