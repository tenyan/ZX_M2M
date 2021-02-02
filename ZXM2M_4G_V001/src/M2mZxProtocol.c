/*****************************************************************************
* @FileName: M2mZxProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-8
* @brief     重型专用TLV创建
******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "M2mZxProtocol.h"
#include "config.h"

/******************************************************************************
 * Typedef
 ******************************************************************************/
typedef uint16_t (*iZxm2m_BuildTlvMsgFun)(uint8_t *pbuf);
//typedef uint16_t (*iZxm2m_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  iZxm2m_BuildTlvMsgFun pfun_build;
  //iZxm2m_AnalyzeTlvMsgFun pfun_analyze;
}iZxm2m_CmdTlv_t;

/******************************************************************************
 * 外部函数
 ******************************************************************************/
extern uint16_t im2m_BuildTlvMsg_0111(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_100D(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_2101(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3000(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3004(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3005(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3007(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3008(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3016(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3017(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3018(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_3019(uint8_t *pbuf);
extern uint16_t im2m_BuildTlvMsg_301A(uint8_t *pbuf);
extern uint8_t im2m_BuildMsgHead(uint8_t *pbuf, im2m_msg_type_t msgType, uint16_t msgBodyLen, uint8_t flag, uint16_t SerialNumber);
extern uint8_t im2m_CalcSumCheck(uint8_t* pbuf,uint16_t len);

void ZxM2mBlindZone_Save(void);

/******************************************************************************
 * Macros
 ******************************************************************************/
//==盲区补偿定义==
#define ZXM2M_BZ_DEBUG    1  // 1-使能, 0-禁止
#define ZXM2M_BZ_SAVE_PERIOD_SP  590 // 1分钟
#define ZXM2M_BLIND_ZONE_PACKET_SIZE  1024
#define ZXM2M_BDZE_WRITE_ERROR_SP  6
#define ZXM2M_BDZE_READ_ERROR_SP   6

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
// TLV状态有效标志
bittype2 zxinfo_tlv_flag;
bittype2 zxup_tlv_flag1;
bittype2 zxup_tlv_flag2;
bittype2 zxup_tlv_flag3;
bittype2 zxdown_tlv_flag1;
bittype2 zxdown_tlv_flag2;
bittype2 zxengine_tlv_flag;
bittype2 zxversion_tlv_flag;
bittype2 zxstatistics_tlv_flag;

uint8_t zxinfo_buffer[SIZE_OF_ZXINFO_BUFFER]; /// 基本信息
uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// 上车数据缓存
uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// 下车底盘数据缓存
uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// 下车发动机数据缓存
uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///统计数据缓存
uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// 版本信息缓存

zxtcw_context_t zxtcw_context;

//==盲区补偿变量==
blind_zone_t zxm2m_blind_zone;
blind_zone_para_t zxm2m_blind_zone_para;
static uint8_t zxm2m_blind_zone_buffer[ZXM2M_BLIND_ZONE_PACKET_SIZE];
static uint16_t zxm2m_blind_zone_length;

/******************************************************************************
 * TLV组包策略
 ******************************************************************************/
// 上车作业TLV--机械/液控
uint16_t zxup_jxyk_tlv_table[] = {0xA5A0, 0xA5C8};
#define NUM_OF_ZXUP_JXYK_TLV  (sizeof(zxup_jxyk_tlv_table)/sizeof(zxup_jxyk_tlv_table[0]))

//============================================================================================
// 上车作业TLV--电控绳排
uint16_t zxup_dksp_tlv_table[] = {
  0xA5A0, 0xA5A3, 0xA5C8, 0xA5A6, 0xA5A7, 0xA5A9, 0xA5AB, 0xA5AC,
  0xA5AD, 0xA5AE, 0xA5AF, 0xA5B0, 0xA5B3, 0xA5B4, 0xA5B5, 0xA5BF,
  0xA5C0, 0xA5C1 };
#define NUM_OF_ZXUP_DKSP_TLV  (sizeof(zxup_dksp_tlv_table)/sizeof(zxup_dksp_tlv_table[0]))

// 上车作业TLV--开式单缸
uint16_t zxup_ksdg_tlv_table[] = {
  0xA5A0, 0xA5A3, 0xA5C8, 0xA5A6, 0xA5A7, 0xA5A9, 0xA5AB, 0xA5AC,
  0xA5AD, 0xA5AE, 0xA5AF, 0xA5B0, 0xA5B3, 0xA5B4, 0xA5B5, 0xA5B7,
  0xA5B8, 0xA5B9, 0xA5BA, 0xA5BC, 0xA5BF, 0xA5C0, 0xA5C1, 0xA5A5,
  0xA5A1, 0xA5A8, 0xA5B1, 0xA5C3, 0xA5C4, 0xA5A2, 0xA5B2, 0xA5BD,
  0xA5BE,0xA5AA };
#define NUM_OF_ZXUP_KSDG_TLV  (sizeof(zxup_ksdg_tlv_table)/sizeof(zxup_ksdg_tlv_table[0]))

// 上车作业TLV--闭式单缸
uint16_t zxup_bsdg_tlv_table[] = {
  0xA5A0, 0xA5A3, 0xA5A4, 0xA5C8, 0xA5A6, 0xA5A7, 0xA5A9, 0xA5AB,
  0xA5AC, 0xA5AD, 0xA5AE, 0xA5AF, 0xA5B3, 0xA5B4, 0xA5B6, 0xA5B7,
  0xA5B8, 0xA5B9, 0xA5BA, 0xA5BB, 0xA5BC, 0xA5BF, 0xA5C0, 0xA5C2,
  0xA5A5, 0xA5A1, 0xA5A8, 0xA5B1, 0xA5C3, 0xA5C4, 0xA5A2, 0xA5B2,
  0xA5BD, 0xA5BE, 0xA5AA };
#define NUM_OF_ZXUP_BSDG_TLV  (sizeof(zxup_bsdg_tlv_table)/sizeof(zxup_bsdg_tlv_table[0]))

//============================================================================================
// 下车作业TLV--全地面底盘
uint16_t zxdown_ag_tlv_table[] = {
  0xA5EF, 0xA5F0, 0xA5F1, 0xA5F2, 0xA5E0, 0xA5E1, 0xA5E4, 0xA5E6,
  0xA5E7, 0xA5E9, 0xA5E8, 0xA5E2, 0xA5E3, 0xA5EC, 0xA5ED, 0xA5EE,
  0xA5EA, 0xA5EB, 0xA502 };
#define NUM_OF_ZXDOWN_AG_TLV  (sizeof(zxdown_ag_tlv_table)/sizeof(zxdown_ag_tlv_table[0]))

// 下车作业TLV--汽车底盘
uint16_t zxdown_ac_tlv_table[] = {
  0xA5EF, 0xA5F0, 0xA5F1, 0xA5F2, 0xA5E0, 0xA5E1, 0xA5E5, 0xA5E6,
  0xA5E7, 0xA5E9, 0xA5EB, 0xA502 };
#define NUM_OF_ZXDOWN_AC_TLV  (sizeof(zxdown_ac_tlv_table)/sizeof(zxdown_ac_tlv_table[0]))


/******************************************************************************
* 创建TLV, 返回信息长度
******************************************************************************/
uint16_t iZxM2m_BuildTlvMsg(uint8_t *pbuf, uint8_t tlv_valid_flag, uint16_t tag, uint16_t length, uint8_t* pValue)
{
  uint16_t len = 0;
  if (tlv_valid_flag)
  {
    pbuf[len++] = (tag>>8) & 0xFF; // TAG
    pbuf[len++] = tag & 0xFF;
    pbuf[len++] = (length>>8) & 0xFF; // LENGTH
    pbuf[len++] = length & 0xFF;
    memcpy(&pbuf[len], pValue, length); // VALUE
    len += length;
  }

  return len;
}

#if (PART("ZxM2m车型配置TLV信息"))
/******************************************************************************
* 创建车型配置信息TLV
******************************************************************************/
//==TAG-A501通用状态字2(重型专用)=================================================
uint16_t iZxM2m_BuildTlvMsg_A501(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a501_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0x01;
    pbuf[len++] = (SIZE_OF_ZXINFO_A501>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXINFO_A501 & 0xFF;
    memcpy(&pbuf[len], zxinfo_buffer_a501, SIZE_OF_ZXINFO_A501); // VALUE
    len += SIZE_OF_ZXINFO_A501;
  }

  return len;
}

//==TAG-A504采集协议信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A504(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a504_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0x04;
    pbuf[len++] = (SIZE_OF_ZXINFO_A504>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXINFO_A504 & 0xFF;
    memcpy(&pbuf[len], zxinfo_buffer_a504, SIZE_OF_ZXINFO_A504); // VALUE
    len += SIZE_OF_ZXINFO_A504;
  }

  return len;
}

//==TAG-3004外部电源电压==========================================================
uint16_t iZxM2m_BuildTlvMsg_3004(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x04;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR]; // VALUE
  pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];

  return len;
}

//==TAG-3005终端内置电池电压==========================================================
uint16_t iZxM2m_BuildTlvMsg_3005(uint8_t *pbuf)
{
  uint16_t len = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x05;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS2_ADDR]; // VALUE
  pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS2_ADDR+1];

  return len;
}
#endif
#if (PART("ZxM2m上车TLV信息"))
/******************************************************************************
* 创建上车通信TLV
******************************************************************************/
//==TLV-A5A0力限器工况信息(单缸)==================================================
uint16_t iZxM2m_BuildTlvMsg_A5A0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a0, SIZE_OF_ZXUP_A5A0); // VALUE
    len += SIZE_OF_ZXUP_A5A0;
  }

  return len;
}

//==TLV-A5A1超起工况信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A1(uint8_t *pbuf)
{
  uint16_t len = 0;
  if (tlv_a5a1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a1, SIZE_OF_ZXUP_A5A1); // VALUE
    len += SIZE_OF_ZXUP_A5A1;
  }

  return len;
}

//==TLV-A5A2塔臂工况信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a2, SIZE_OF_ZXUP_A5A2); // VALUE
    len += SIZE_OF_ZXUP_A5A2;
  }

  return len;
}

//==TLV-A5A5上车发动机信息========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA5;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A5 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a5, SIZE_OF_ZXUP_A5A5); // VALUE
    len += SIZE_OF_ZXUP_A5A5;
  }

  return len;
}

//==TLV-A5A6手柄信息==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5A6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA6;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A6 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a6, SIZE_OF_ZXUP_A5A6); // VALUE
    len += SIZE_OF_ZXUP_A5A6;
  }

  return len;
}

//==TLV-A5A7显示器1信息===========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA7;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A7 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a7, SIZE_OF_ZXUP_A5A7); // VALUE
    len += SIZE_OF_ZXUP_A5A7;
  }

  return len;
}

//==TLV-A5A8显示器2信息===========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a8, SIZE_OF_ZXUP_A5A8); // VALUE
    len += SIZE_OF_ZXUP_A5A8;
  }

  return len;
}

//==TLV-A5A9总线面板信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5A9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5A9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5a9, SIZE_OF_ZXUP_A5A9); // VALUE
    len += SIZE_OF_ZXUP_A5A9;
  }

  return len;
}

//==TLV-A5AA无线操控信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5aa_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAA;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AA & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5aa, SIZE_OF_ZXUP_A5AA); // VALUE
    len += SIZE_OF_ZXUP_A5AA;
  }

  return len;
}

//==TLV-A5AB网络拓扑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ab_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAB;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AB & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ab, SIZE_OF_ZXUP_A5AB); // VALUE
    len += SIZE_OF_ZXUP_A5AB;
  }

  return len;
}

//==TLV-A5AC伸缩逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ac_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAC;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AC & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ac, SIZE_OF_ZXUP_A5AC); // VALUE
    len += SIZE_OF_ZXUP_A5AC;
  }

  return len;
}

//==TLV-A5AD变幅逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AD(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ad_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAD;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AD>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AD & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ad, SIZE_OF_ZXUP_A5AD); // VALUE
    len += SIZE_OF_ZXUP_A5AD;
  }

  return len;
}

//==TLV-A5AE回转逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ae_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAE;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AE & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ae, SIZE_OF_ZXUP_A5AE); // VALUE
    len += SIZE_OF_ZXUP_A5AE;
  }

  return len;
}

//==TLV-A5AF主卷扬逻辑信息========================================================
uint16_t iZxM2m_BuildTlvMsg_A5AF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5af_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xAF;
    pbuf[len++] = (SIZE_OF_ZXUP_A5AF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5AF & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5af, SIZE_OF_ZXUP_A5AF); // VALUE
    len += SIZE_OF_ZXUP_A5AF;
  }

  return len;
}

//==TLV-A5B0副卷扬逻辑信息========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b0, SIZE_OF_ZXUP_A5B0); // VALUE
    len += SIZE_OF_ZXUP_A5B0;
  }

  return len;
}

//==TLV-A5B1超起逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b1, SIZE_OF_ZXUP_A5B1); // VALUE
    len += SIZE_OF_ZXUP_A5B1;
  }

  return len;
}

//==TLV-A5B2塔臂逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b2, SIZE_OF_ZXUP_A5B2); // VALUE
    len += SIZE_OF_ZXUP_A5B2;
  }

  return len;
}

//==TLV-A5B3液压油温度============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b3, SIZE_OF_ZXUP_A5B3); // VALUE
    len += SIZE_OF_ZXUP_A5B3;
  }

  return len;
}

//==TLV-A5B4主泵信息==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b4, SIZE_OF_ZXUP_A5B4); // VALUE
    len += SIZE_OF_ZXUP_A5B4;
  }

  return len;
}

//==TLV-A5B5主阀1 XHVME4400P1=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5B5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB5;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B5 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b5, SIZE_OF_ZXUP_A5B5); // VALUE
    len += SIZE_OF_ZXUP_A5B5;
  }

  return len;
}

//==TLV-A5B6主阀3 大吨位==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB6;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B6 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b6, SIZE_OF_ZXUP_A5B6); // VALUE
    len += SIZE_OF_ZXUP_A5B6;
  }

  return len;
}

//==TLV-A5B7主泵信息==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5B7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB7;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B7 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b7, SIZE_OF_ZXUP_A5B7); // VALUE
    len += SIZE_OF_ZXUP_A5B7;
  }

  return len;
}

//==TLV-A5B8塔臂逻辑信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b8, SIZE_OF_ZXUP_A5B8); // VALUE
    len += SIZE_OF_ZXUP_A5B8;
  }

  return len;
}

//==TLV-A5B9缸臂销控制阀==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5B9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5b9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xB9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5B9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5B9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5b9, SIZE_OF_ZXUP_A5B9); // VALUE
    len += SIZE_OF_ZXUP_A5B9;
  }

  return len;
}

//==TLV-A5BA变幅平衡阀============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ba_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBA;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BA & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5ba, SIZE_OF_ZXUP_A5BA); // VALUE
    len += SIZE_OF_ZXUP_A5BA;
  }

  return len;
}

//==TLV-A5BB主卷泵================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bb_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBB;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BB & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bb, SIZE_OF_ZXUP_A5BB); // VALUE
    len += SIZE_OF_ZXUP_A5BB;
  }

  return len;
}

//==TLV-A5BC主卷马达==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bc_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBC;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BC & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bc, SIZE_OF_ZXUP_A5BC); // VALUE
    len += SIZE_OF_ZXUP_A5BC;
  }

  return len;
}

//==TLV-A5BD塔卷泵================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BD(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bd_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBD;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BD>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BD & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bd, SIZE_OF_ZXUP_A5BD); // VALUE
    len += SIZE_OF_ZXUP_A5BD;
  }

  return len;
}

//==TLV-A5BE塔卷马达==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5BE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5be_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBE;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BE & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5be, SIZE_OF_ZXUP_A5BE); // VALUE
    len += SIZE_OF_ZXUP_A5BE;
  }

  return len;
}

//==TLV-A5BF回转泵================================================================
uint16_t iZxM2m_BuildTlvMsg_A5BF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5bf_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xBF;
    pbuf[len++] = (SIZE_OF_ZXUP_A5BF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5BF & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5bf, SIZE_OF_ZXUP_A5BF); // VALUE
    len += SIZE_OF_ZXUP_A5BF;
  }

  return len;
}

//==TLV-A5C0回转制动阀============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC0;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C0 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c0, SIZE_OF_ZXUP_A5C0); // VALUE
    len += SIZE_OF_ZXUP_A5C0;
  }

  return len;
}

//==TLV-A5C1辅助阀1===============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC1;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C1 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c1, SIZE_OF_ZXUP_A5C1); // VALUE
    len += SIZE_OF_ZXUP_A5C1;
  }

  return len;
}

//==TLV-A5C2辅助阀2(超大吨位)=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5C2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC2;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C2 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c2, SIZE_OF_ZXUP_A5C2); // VALUE
    len += SIZE_OF_ZXUP_A5C2;
  }

  return len;
}

//==TLV-A5C3左超起阀组============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC3;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C3 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c3, SIZE_OF_ZXUP_A5C3); // VALUE
    len += SIZE_OF_ZXUP_A5C3;
  }

  return len;
}

//==TLV-A5C4右超起阀组============================================================
uint16_t iZxM2m_BuildTlvMsg_A5C4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC4;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C4 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c4, SIZE_OF_ZXUP_A5C4); // VALUE
    len += SIZE_OF_ZXUP_A5C4;
  }

  return len;
}

//==TLV-A5C8作业油耗信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5C8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC8;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C8 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c8, SIZE_OF_ZXUP_A5C8); // VALUE
    len += SIZE_OF_ZXUP_A5C8;
  }

  return len;
}

//==TLV-A5C9 ECU响应锁车CAN帧=====================================================
uint16_t iZxM2m_BuildTlvMsg_A5C9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC9;
    pbuf[len++] = (SIZE_OF_ZXUP_A5C9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXUP_A5C9 & 0xFF;
    memcpy(&pbuf[len], zxup_buffer_a5c9, SIZE_OF_ZXUP_A5C9); // VALUE
    len += SIZE_OF_ZXUP_A5C9;
  }

  return len;
}
#endif
#if (PART("ZxM2m下车TLV信息"))
/******************************************************************************
* 创建下车通信TLV
******************************************************************************/
//==TLV-A5E0节点状态==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE0;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E0 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e0, SIZE_OF_ZXDOWN_A5E0); // VALUE
    len += SIZE_OF_ZXDOWN_A5E0;
  }

  return len;
}

//==TLV-A5E1传动系统==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE1;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E1 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e1, SIZE_OF_ZXDOWN_A5E1); // VALUE
    len += SIZE_OF_ZXDOWN_A5E1;
  }

  return len;
}

//==TLV-A5E2支腿相关信息==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5E2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE2;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E2 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e2, SIZE_OF_ZXDOWN_A5E2); // VALUE
    len += SIZE_OF_ZXDOWN_A5E2;
  }

  return len;
}

//==TLV-A5E3悬挂系统==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE3;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E3 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer, SIZE_OF_ZXDOWN_A5E3); // VALUE
    len += SIZE_OF_ZXDOWN_A5E3;
  }

  return len;
}

//==TLV-A5E4转向系统（全地面）====================================================
uint16_t iZxM2m_BuildTlvMsg_A5E4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE4;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E4 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e4, SIZE_OF_ZXDOWN_A5E4); // VALUE
    len += SIZE_OF_ZXDOWN_A5E4;
  }

  return len;
}

//==TLV-A5E5转向系统（汽车）======================================================
uint16_t iZxM2m_BuildTlvMsg_A5E5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE5;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E5 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e5, SIZE_OF_ZXDOWN_A5E5); // VALUE
    len += SIZE_OF_ZXDOWN_A5E5;
  }

  return len;
}

//==TLV-A5E6制动系统==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE6;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E6 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e6, SIZE_OF_ZXDOWN_A5E6); // VALUE
    len += SIZE_OF_ZXDOWN_A5E6;
  }

  return len;
}

//==TLV-A5E7动力系统==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE7;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E7 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e7, SIZE_OF_ZXDOWN_A5E7); // VALUE
    len += SIZE_OF_ZXDOWN_A5E7;
  }

  return len;
}

//==TLV-A5E8单发取力系统==========================================================
uint16_t iZxM2m_BuildTlvMsg_A5E8(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e8_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE8;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E8>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E8 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e8, SIZE_OF_ZXDOWN_A5E8); // VALUE
    len += SIZE_OF_ZXDOWN_A5E8;
  }

  return len;
}

//==TLV-A5E9液压系统==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5E9(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5e9_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xE9;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5E9>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5E9 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5e9, SIZE_OF_ZXDOWN_A5E9); // VALUE
    len += SIZE_OF_ZXDOWN_A5E9;
  }

  return len;
}

//==TLV-A5EA双动力驱动系统========================================================
uint16_t iZxM2m_BuildTlvMsg_A5EA(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ea_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEA;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5EA>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5EA & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5ea, SIZE_OF_ZXDOWN_A5EA); // VALUE
    len += SIZE_OF_ZXDOWN_A5EA;
  }

  return len;
}

//==TLV-A5EB轮胎胎压==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5EB(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5eb_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEB;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5EB>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5EB & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5eb, SIZE_OF_ZXDOWN_A5EB); // VALUE
    len += SIZE_OF_ZXDOWN_A5EB;
  }

  return len;
}

//==TLV-A5EC左支腿面板============================================================
uint16_t iZxM2m_BuildTlvMsg_A5EC(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ec_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEC;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5EC>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5EC & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5ec, SIZE_OF_ZXDOWN_A5EC); // VALUE
    len += SIZE_OF_ZXDOWN_A5EC;
  }

  return len;
}

//==TLV-A5ED右支腿面板============================================================
uint16_t iZxM2m_BuildTlvMsg_A5ED(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ed_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xED;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5ED>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5ED & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5ed, SIZE_OF_ZXDOWN_A5ED); // VALUE
    len += SIZE_OF_ZXDOWN_A5ED;
  }

  return len;
}

//==TLV-A5EE中控台输入信息========================================================
uint16_t iZxM2m_BuildTlvMsg_A5EE(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ee_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEE;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5EE>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5EE & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5ee, SIZE_OF_ZXDOWN_A5EE); // VALUE
    len += SIZE_OF_ZXDOWN_A5EE;
  }

  return len;
}

//==TLV-A5A3支腿作业信息=========================================================
uint16_t iZxM2m_BuildTlvMsg_A5A3(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a3_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA3;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5A3>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5A3 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5a3, SIZE_OF_ZXDOWN_A5A3); // VALUE
    len += SIZE_OF_ZXDOWN_A5A3;
  }

  return len;
}

//==TLV-A5A4辅助支腿作业信息======================================================
uint16_t iZxM2m_BuildTlvMsg_A5A4(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5a4_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xA4;
    pbuf[len++] = (SIZE_OF_ZXDOWN_A5A4>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXDOWN_A5A4 & 0xFF;
    memcpy(&pbuf[len], zxdown_buffer_a5a4, SIZE_OF_ZXDOWN_A5A4); // VALUE
    len += SIZE_OF_ZXDOWN_A5A4;
  }

  return len;
}
#endif
#if (PART("ZxM2m底盘发动机TLV信息"))
/******************************************************************************
* 创建底盘发动机TLV
******************************************************************************/
//==TAG-A502环境信息(发动机的数据)================================================
uint16_t iZxM2m_BuildTlvMsg_A502(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a502_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0x02;
    pbuf[len++] = (SIZE_OF_ZXENGINE_A502>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXENGINE_A502 & 0xFF;
    memcpy(&pbuf[len], zxengine_buffer_a502, SIZE_OF_ZXENGINE_A502); // VALUE
    len += SIZE_OF_ZXENGINE_A502;
  }

  return len;
}

//==TLV-A5EF发动机运行参数(国四、国五、国六)======================================
uint16_t iZxM2m_BuildTlvMsg_A5EF(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5ef_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xEF;
    pbuf[len++] = (SIZE_OF_ZXENGINE_A5EF>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXENGINE_A5EF & 0xFF;
    memcpy(&pbuf[len], zxengine_buffer_a5ef, SIZE_OF_ZXENGINE_A5EF); // VALUE
    len += SIZE_OF_ZXENGINE_A5EF;
  }

  return len;
}

//==TLV-A5F0行驶油耗==============================================================
uint16_t iZxM2m_BuildTlvMsg_A5F0(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f0_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF0;
    pbuf[len++] = (SIZE_OF_ZXENGINE_A5F0>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXENGINE_A5F0 & 0xFF;
    memcpy(&pbuf[len], zxengine_buffer_a5f0, SIZE_OF_ZXENGINE_A5F0); // VALUE
    len += SIZE_OF_ZXENGINE_A5F0;
  }

  return len;
}

//==TLV-A5F1 SCR参数（国五）======================================================
uint16_t iZxM2m_BuildTlvMsg_A5F1(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f1_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF1;
    pbuf[len++] = (SIZE_OF_ZXENGINE_A5F1>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXENGINE_A5F1 & 0xFF;
    memcpy(&pbuf[len], zxengine_buffer_a5f1, SIZE_OF_ZXENGINE_A5F1); // VALUE
    len += SIZE_OF_ZXENGINE_A5F1;
  }

  return len;
}

//==TLV-A5F2 DPF参数(国六）=======================================================
uint16_t iZxM2m_BuildTlvMsg_A5F2(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5f2_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xF2;
    pbuf[len++] = (SIZE_OF_ZXENGINE_A5F2>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXENGINE_A5F2 & 0xFF;
    memcpy(&pbuf[len], zxengine_buffer_a5f2, SIZE_OF_ZXENGINE_A5F2); // VALUE
    len += SIZE_OF_ZXENGINE_A5F2;
  }

  return len;
}
#endif
#if (PART("ZxM2m版本TLV信息"))
/******************************************************************************
* 版本信息
******************************************************************************/
//==TLV-A505 上车系统版本=======================================================
uint16_t iZxM2m_BuildTlvMsg_A505(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a505_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0x05;
    pbuf[len++] = (SIZE_OF_ZXVERSION_A505>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXVERSION_A505 & 0xFF;
    memcpy(&pbuf[len], zxversion_buffer_a505, SIZE_OF_ZXVERSION_A505); // VALUE
    len += SIZE_OF_ZXVERSION_A505;
  }

  return len;
}

//==TLV-A506 下车系统版本=======================================================
uint16_t iZxM2m_BuildTlvMsg_A506(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a506_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0x06;
    pbuf[len++] = (SIZE_OF_ZXVERSION_A506>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXVERSION_A506 & 0xFF;
    memcpy(&pbuf[len], zxversion_buffer_a506, SIZE_OF_ZXVERSION_A506); // VALUE
    len += SIZE_OF_ZXVERSION_A506;
  }

  return len;
}
#endif
#if (PART("ZxM2m频次统计TLV信息"))
/******************************************************************************
* 频次统计信息
******************************************************************************/
//==TLV-A5C5 动作频次统计1=======================================================
uint16_t iZxM2m_BuildTlvMsg_A5C5(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c5_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC5;
    pbuf[len++] = (SIZE_OF_ZXSTATISTICS_A5C5>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXSTATISTICS_A5C5 & 0xFF;
    memcpy(&pbuf[len], zxstatistics_buffer_a5c5, SIZE_OF_ZXSTATISTICS_A5C5); // VALUE
    len += SIZE_OF_ZXSTATISTICS_A5C5;
  }

  return len;
}

//==TLV-A5C6 动作频次统计2=======================================================
uint16_t iZxM2m_BuildTlvMsg_A5C6(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c6_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC6;
    pbuf[len++] = (SIZE_OF_ZXSTATISTICS_A5C6>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXSTATISTICS_A5C6 & 0xFF;
    memcpy(&pbuf[len], zxstatistics_buffer_a5c6, SIZE_OF_ZXSTATISTICS_A5C6); // VALUE
    len += SIZE_OF_ZXSTATISTICS_A5C6;
  }

  return len;
}

//==TLV-A5C7 安全统计===========================================================
uint16_t iZxM2m_BuildTlvMsg_A5C7(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_a5c7_valid_flag)
  {
    pbuf[len++] = 0xA5; // TAG
    pbuf[len++] = 0xC7;
    pbuf[len++] = (SIZE_OF_ZXSTATISTICS_A5C7>>8) & 0xFF; // LENGTH
    pbuf[len++] = SIZE_OF_ZXSTATISTICS_A5C7 & 0xFF;
    memcpy(&pbuf[len], zxstatistics_buffer_a5c7, SIZE_OF_ZXSTATISTICS_A5C7); // VALUE
    len += SIZE_OF_ZXSTATISTICS_A5C7;
  }

  return len;
}
#endif

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
//==type按照递增顺序填写=================================================
iZxm2m_CmdTlv_t iZxm2m_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/

  /*****************车型配置信息TLV**********************************/
  {0xA501, iZxM2m_BuildTlvMsg_A501},// TAG-A501通用状态字2(重型专用)
  {0xA504, iZxM2m_BuildTlvMsg_A504},// TAG-A504采集协议信息
  {0x3004, iZxM2m_BuildTlvMsg_3004},// TAG-3004外部电源电压
  {0x3005, iZxM2m_BuildTlvMsg_3005},// TAG-3005终端内置电池电压

  /*****************创建上车通信TLV**********************************/
  {0xA5A0, iZxM2m_BuildTlvMsg_A5A0},// TLV-A5A0力限器工况信息(单缸)
  {0xA5A1, iZxM2m_BuildTlvMsg_A5A1},// TLV-A5A1超起工况信息
  {0xA5A2, iZxM2m_BuildTlvMsg_A5A2},// TLV-A5A2塔臂工况信息
  {0xA5A5, iZxM2m_BuildTlvMsg_A5A5},// TLV-A5A5上车发动机信息
  {0xA5A6, iZxM2m_BuildTlvMsg_A5A6},// TLV-A5A6手柄信息
  {0xA5A7, iZxM2m_BuildTlvMsg_A5A7},// TLV-A5A7显示器1信息
  {0xA5A8, iZxM2m_BuildTlvMsg_A5A8},// TLV-A5A8显示器2信息
  {0xA5A9, iZxM2m_BuildTlvMsg_A5A9},// TTLV-A5A9总线面板信息
  {0xA5AA, iZxM2m_BuildTlvMsg_A5AA},// TLV-A5AA无线操控信息
  {0xA5AB, iZxM2m_BuildTlvMsg_A5AB},// TLV-A5AB网络拓扑信息
  {0xA5AC, iZxM2m_BuildTlvMsg_A5AC},// TLV-A5AC伸缩逻辑信息
  {0xA5AD, iZxM2m_BuildTlvMsg_A5AD},// TLV-A5AD变幅逻辑信息  
  {0xA5AE, iZxM2m_BuildTlvMsg_A5AE},// TLV-A5AE回转逻辑信息
  {0xA5AF, iZxM2m_BuildTlvMsg_A5AF},// TLV-A5AF主卷扬逻辑信息
  {0xA5B0, iZxM2m_BuildTlvMsg_A5B0},// TLV-A5B0副卷扬逻辑信息
  {0xA5B1, iZxM2m_BuildTlvMsg_A5B1},// TLV-A5B1超起逻辑信息
  {0xA5B2, iZxM2m_BuildTlvMsg_A5B2},// TLV-A5B2塔臂逻辑信息
  {0xA5B3, iZxM2m_BuildTlvMsg_A5B3},// TLV-A5B3液压油温度  
  {0xA5B4, iZxM2m_BuildTlvMsg_A5B4},// TLV-A5B4主泵信息
  {0xA5B5, iZxM2m_BuildTlvMsg_A5B5},// TLV-A5B5主阀1 XHVME4400P1
  {0xA5B6, iZxM2m_BuildTlvMsg_A5B6},// TLV-A5B6主阀3 大吨位
  {0xA5B7, iZxM2m_BuildTlvMsg_A5B7},// TLV-A5B7主泵信息
  {0xA5B8, iZxM2m_BuildTlvMsg_A5B8},// TLV-A5B8塔臂逻辑信息
  {0xA5B9, iZxM2m_BuildTlvMsg_A5B9},// TLV-A5B9缸臂销控制阀  
  {0xA5BA, iZxM2m_BuildTlvMsg_A5BA},// TLV-A5BA变幅平衡阀
  {0xA5BB, iZxM2m_BuildTlvMsg_A5BB},// TLV-A5BB主卷泵
  {0xA5BC, iZxM2m_BuildTlvMsg_A5BC},// TLV-A5BC主卷马达
  {0xA5BD, iZxM2m_BuildTlvMsg_A5BD},// TLV-A5BD塔卷泵
  {0xA5BE, iZxM2m_BuildTlvMsg_A5BE},// TLV-A5BE塔卷马达 
  {0xA5BF, iZxM2m_BuildTlvMsg_A5BF},// TLV-A5BF回转泵
  {0xA5C0, iZxM2m_BuildTlvMsg_A5C0},// TLV-A5C0回转制动阀
  {0xA5C1, iZxM2m_BuildTlvMsg_A5C1},// TLV-A5C1辅助阀1
  {0xA5C2, iZxM2m_BuildTlvMsg_A5C2},// TLV-A5C2辅助阀2(超大吨位)
  {0xA5C3, iZxM2m_BuildTlvMsg_A5C3},// TLV-A5C3左超起阀组
  {0xA5C4, iZxM2m_BuildTlvMsg_A5C4},// TLV-A5C4右超起阀组  
  {0xA5C8, iZxM2m_BuildTlvMsg_A5C8},// TLV-A5C8作业油耗信息
  {0xA5C9, iZxM2m_BuildTlvMsg_A5C9},// TLV-A5C9 ECU响应锁车CAN帧

  /*****************创建下车通信TLV**********************************/
  {0xA5E0, iZxM2m_BuildTlvMsg_A5E0},// TLV-A5E0节点状态
  {0xA5E1, iZxM2m_BuildTlvMsg_A5E1},// TLV-A5E1传动系统
  {0xA5E2, iZxM2m_BuildTlvMsg_A5E2},// TLV-A5E2支腿相关信息
  {0xA5E3, iZxM2m_BuildTlvMsg_A5E3},// TLV-A5E3悬挂系统  
  {0xA5E4, iZxM2m_BuildTlvMsg_A5E4},// TLV-A5E4转向系统(全地面)
  {0xA5E5, iZxM2m_BuildTlvMsg_A5E5},// TLV-A5E5转向系统(汽车)
  {0xA5E6, iZxM2m_BuildTlvMsg_A5E6},// TLV-A5E6制动系统
  {0xA5E7, iZxM2m_BuildTlvMsg_A5E7},// TLV-A5E7动力系统
  {0xA5E8, iZxM2m_BuildTlvMsg_A5E8},// TLV-A5E8单发取力系统
  {0xA5E9, iZxM2m_BuildTlvMsg_A5E9},// TLV-A5E9液压系统
  {0xA5EA, iZxM2m_BuildTlvMsg_A5EA},// TLV-A5EA双动力驱动系统
  {0xA5EB, iZxM2m_BuildTlvMsg_A5EB},// TLV-A5EB轮胎胎压
  {0xA5EC, iZxM2m_BuildTlvMsg_A5EC},// TLV-A5EC左支腿面板
  {0xA5ED, iZxM2m_BuildTlvMsg_A5ED},// TLV-A5ED右支腿面板
  {0xA5EE, iZxM2m_BuildTlvMsg_A5EE},// TLV-A5EE中控台输入信息
  {0xA5A3, iZxM2m_BuildTlvMsg_A5A3},// TLV-A5A3支腿作业信息
  {0xA5A4, iZxM2m_BuildTlvMsg_A5A4},// TLV-A5A4辅助支腿作业信息

  /*****************创建底盘发动机TLV*********************************/
  {0xA502, iZxM2m_BuildTlvMsg_A502},// TAG-A502环境信息(发动机的数据)
  {0xA5EF, iZxM2m_BuildTlvMsg_A5EF},// TLV-A5EF发动机运行参数(国四、国五、国六)
  {0xA5F0, iZxM2m_BuildTlvMsg_A5F0},// TLV-A5F0行驶油耗
  {0xA5F1, iZxM2m_BuildTlvMsg_A5F1},// TLV-A5F1 SCR参数(国五)
  {0xA5F2, iZxM2m_BuildTlvMsg_A5F2},// TLV-A5F2 DPF参数(国六)

  /*****************创建版本信息TLV*********************************/
  {0xA505, iZxM2m_BuildTlvMsg_A505},// TLV-A505 上车系统版本
  {0xA506, iZxM2m_BuildTlvMsg_A506},// TLV-A506 下车系统版本

  /*****************创建频次统计TLV*********************************/
  {0xA5C5, iZxM2m_BuildTlvMsg_A5C5},// TLV-A5C5 动作频次统计1
  {0xA5C6, iZxM2m_BuildTlvMsg_A5C6},// TLV-A5C6 动作频次统计2
  {0xA5C7, iZxM2m_BuildTlvMsg_A5C7},// TLV-A5C7 安全统计
};
#define NUM_OF_ZXM2M_CMD_DEAL   (sizeof(iZxm2m_CmdDealTbl)/sizeof(iZxm2m_CmdDealTbl))

#if (PART("ZxM2m解析RC控制命令"))
//==重型专用:绑定与解绑======================================================
uint16_t iZxM2m_AnalyzeTlvMsg_A510(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;

  }

  return retVal;
}

//==重型专用:锁车与解锁======================================================
uint16_t iZxM2m_AnalyzeTlvMsg_A511(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;

  }

  return retVal;
}

//==重型专用:环保协议类型======================================================
uint16_t iZxM2m_AnalyzeTlvMsg_A512(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;

  }

  return retVal;
}

//==重型专用:VIN码设置=======================================================
uint16_t iZxM2m_AnalyzeTlvMsg_A513(uint8_t* pValue, uint16_t len)
{
  uint16_t retVal = 0;
  //uint8_t tempVal;

  if (0==len)
  {
    retVal = 1;

  }

  return retVal;
}
#endif

//==TLV-0x100F 协处理器版本===================================================
uint16_t im2m_BuildTlvMsg_100F(uint8_t *pbuf)
{
  uint16_t len =0;
  uint16_t st_version;

  st_version = COLT_GetStVersion();
  pbuf[len++] = 0x10;  // TAG
  pbuf[len++] = 0x0F;
  pbuf[len++] = 0x00;  // LENGTH
  pbuf[len++] = 0x02;
  pbuf[len++] = (uint8_t)(st_version>>8);  // VALUE
  pbuf[len++] = (uint8_t)st_version;

  return len;
}

//==TLV-终端休眠时间(0x301E)===================================================
uint16_t im2m_BuildTlvMsg_301E(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint32_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x1E;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x08;
  //temp_val = Cellura_GetLacID();
  pbuf[len++] = (temp_val>>24) & 0xFF;  // 总休眠时间
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;
  //temp_val = Cellura_GetCellID();
  pbuf[len++] = (temp_val>>24) & 0xFF;  // 本次休眠时间
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;

  return len;
}


//==TLV-4G模块小区信息及位置码(0x301F)==========================================
uint16_t im2m_BuildTlvMsg_301F(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint32_t temp_val = 0;

  pbuf[len++] = 0x30; // TAG
  pbuf[len++] = 0x1F;
  pbuf[len++] = 0x00; // LENGTH
  pbuf[len++] = 0x08;
  temp_val = Cellura_GetLacID();
  pbuf[len++] = (temp_val>>24) & 0xFF;  // VALUE
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;
  temp_val = Cellura_GetCellID();
  pbuf[len++] = (temp_val>>24) & 0xFF;  // VALUE
  pbuf[len++] = (temp_val>>16) & 0xFF;
  pbuf[len++] = (temp_val>>8) & 0xFF;
  pbuf[len++] = temp_val & 0xFF;

  return len;
}

//=============================================================================
uint16_t iZxM2m_BuildTcwData(uint8_t *pbuf, uint16_t tag)
{
  uint16_t len = 0;
  uint16_t it = 0;
  iZxm2m_CmdTlv_t* p_CmdTlvDeal = NULL;

  for (it = 0; it<NUM_OF_ZXM2M_CMD_DEAL; it++)
  {
    p_CmdTlvDeal = &iZxm2m_CmdDealTbl[it];
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

//==TLV-创建重型TCW工况TLV信息===================================================
uint16_t iZxM2m_BuildTlvMsg_TCW(uint8_t *pbuf, zxtcw_context_t* pZxtcw_ctx)
{
  uint8_t tlvNum;
  uint16_t it = 0;
  uint16_t tag = 0;
  uint16_t tlvLen = 0;
  uint16_t len = 0; // TLV消息累计长度

  pZxtcw_ctx->valid_tlv_num = 0;
  len = 0;

  //==上车TLV===============================================================
  it = 0;
  tlvNum = pZxtcw_ctx->zxup_tlv_num;  // 上车TLV数量
  while (tlvNum)
  {
    tag = pZxtcw_ctx->zxup_tlv_table[it++];  // 获取TLV的TAG
    tlvLen = iZxM2m_BuildTcwData(&pbuf[len], tag);  // 根据TAG查找TLV信息组建函数并创建信息
    if (tlvLen>0)
    {
      pZxtcw_ctx->valid_tlv_num++;
      len += tlvLen;
    }
    tlvNum--;
  }

  //==下车(底盘+发动机)TLV===============================================================
  it = 0;
  tlvNum = pZxtcw_ctx->zxdown_tlv_num;  // 下车TLV数量
  while (tlvNum)
  {
    tag = pZxtcw_ctx->zxdown_tlv_table[it++];  // 获取TLV的TAG
    tlvLen = iZxM2m_BuildTcwData(&pbuf[len], tag);  // 根据TAG查找TLV信息组建函数并创建信息
    if (tlvLen>0)
    {
      pZxtcw_ctx->valid_tlv_num++;
      len += tlvLen;
    }
    tlvNum--;
  }

  return len;
}

#if (PART("ZxM2m消息组建函数"))
/*************************************************************************
 *
*************************************************************************/
//==构建TCS消息体========================================================
uint16_t iZxM2m_BuildTcsBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x03;
  pbuf[len++] = 'T';  // 数据类型
  pbuf[len++] = 'C';
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

  /// 休眠时间统计数据
  temp_val = im2m_BuildTlvMsg_301E(&pbuf[len]);
  len += temp_val;
  tlv_num += ((temp_val == 0)? 0: 1);

  /// Cellular ID,终端设备所在的小区标识
  temp_val = im2m_BuildTlvMsg_301F(&pbuf[len]);
  len += temp_val;
  tlv_num++;

  /// 重型通用状态字2
  temp_val = iZxM2m_BuildTlvMsg_A501(&pbuf[len]);
  len += temp_val;
  tlv_num += ((temp_val == 0)? 0: 1);
  
  /// 填充TCS数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}

//==构建TCW消息体=========================================================
uint16_t iZxM2m_BuildTcwBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x03;
  pbuf[len++] = 'T';  // 数据类型
  pbuf[len++] = 'C';
  pbuf[len++] = 'W';

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

  /// 重型工况数据
  temp_val = iZxM2m_BuildTlvMsg_TCW(&pbuf[len], &zxtcw_context);
  len += temp_val;
  tlv_num += zxtcw_context.valid_tlv_num;

  /// 填充TCB数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}

//==构建TCB(版本)消息体=========================================================
uint16_t iZxM2m_BuildTcbBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x03;
  pbuf[len++] = 'T';  // 数据类型
  pbuf[len++] = 'C';
  pbuf[len++] = 'B';

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

  /// 采集协议信息(0xA504)
  temp_val = iZxM2m_BuildTlvMsg_A504(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 上车系统版本(0xA505)
  temp_val = iZxM2m_BuildTlvMsg_A505(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 下车系统版本(0xA506)
  temp_val = iZxM2m_BuildTlvMsg_A5C6(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 填充TCB数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}

//==构建TCT(统计)消息体=========================================================
uint16_t iZxM2m_BuildTctBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x03;
  pbuf[len++] = 'T';  // 数据类型
  pbuf[len++] = 'C';
  pbuf[len++] = 'B';

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

  /// 动作频次统计1(0xA5C5)
  temp_val = iZxM2m_BuildTlvMsg_A5C5(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 动作频次统计2(0xA5C6)
  temp_val = iZxM2m_BuildTlvMsg_A5C6(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 安全统计(0xA5C7)
  temp_val = iZxM2m_BuildTlvMsg_A5C7(&pbuf[len]);
  if(temp_val!=0)
  {
    len += temp_val;
    tlv_num++;
  }

  /// 填充TCT数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}

//==构建TCD消息体=========================================================
uint16_t iZxM2m_BuildTcdBody(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint8_t tlv_num = 0;
  uint16_t temp_val;

  //==数据类型=========================================
  pbuf[len++] = 0x00; // 数据类型长度
  pbuf[len++] = 0x03;
  pbuf[len++] = 'T';  // 数据类型
  pbuf[len++] = 'C';
  pbuf[len++] = 'D';

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

  /// 填充TCD数据内容长度及tlv计数
  temp_val = len - 6;
  pbuf[4] = (temp_val >> 8) & 0xFF; // 数据内容长度
  pbuf[5] = temp_val & 0xFF;
  pbuf[6] = tlv_num; // TLV个数

  return len;
}
#endif

#if (PART("发送ZxM2m网络数据"))
/******************************************************************************
 *
*******************************************************************************/
//==发送重型ConnectRequest消息=================================================
void iZxM2m_SendConnenctMsg(m2m_context_t* pThis)
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

  /// TLV-0x100F 协处理器版本
  len += im2m_BuildTlvMsg_100F(&pbuf[len]);

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

//==发送重型数据消息============================================================
void iZxM2m_SendTcMsg(m2m_context_t* pThis, uint8_t msg_type)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  //==创建报文体==========================================
  if(msg_type==ZXTC_MSG_TYPE_TCW)
  {
    msg_body_len = iZxM2m_BuildTcwBody(&pbuf[M2M_MSG_HEAD_LEN]);
  }
  else if(msg_type==ZXTC_MSG_TYPE_TCS)
  {
    msg_body_len = iZxM2m_BuildTcsBody(&pbuf[M2M_MSG_HEAD_LEN]);
  }
  else if(msg_type==ZXTC_MSG_TYPE_TCB)
  {
    msg_body_len = iZxM2m_BuildTcbBody(&pbuf[M2M_MSG_HEAD_LEN]);
  }
  else if(msg_type==ZXTC_MSG_TYPE_TCT)
  {
    msg_body_len = iZxM2m_BuildTctBody(&pbuf[M2M_MSG_HEAD_LEN]);
  }

  else if(msg_type==ZXTC_MSG_TYPE_TCD)
  {
    msg_body_len = iZxM2m_BuildTcdBody(&pbuf[M2M_MSG_HEAD_LEN]);
  }
  else
  {
    msg_body_len = 0;
  }

  if(msg_body_len==0)  // 空消息
  {
    return;
  }

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

//==上发盲区补偿数据==============================================================================
void iZxM2m_SendBzData(m2m_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint16_t stack_size = 0x00;
  uint8_t check_sum;
  uint8_t* pbuf = pThis->tx_data;

  stack_size = BlindZone_GetStackSize(&zxm2m_blind_zone);
  if (stack_size > 0)
  {
    //==读取报文体==========================================
    BlindZone_PopData(&zxm2m_blind_zone, &pbuf[M2M_MSG_HEAD_LEN], &msg_body_len);
    ZxM2mBlindZone_Save(); // 存入栈参数
    if (msg_body_len == 0x00)
    {
#if ZXM2M_BZ_DEBUG
      PcDebug_Printf("ZxM2mBzPop:Err\r\n");
#endif
      return;
    }

    //==创建报文头==========================================
    pThis->upload_sn++;
    pThis->ss_req.sn = pThis->upload_sn;
    msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_PUSH_DATA, msg_body_len, 0x08, pThis->upload_sn); // 盲区补偿数据

    //==计算校验字==========================================
    msg_len = msg_body_len + msg_header_len;
    check_sum = im2m_CalcSumCheck(pbuf, msg_len);
    pbuf[msg_len++] = check_sum;
    pThis->tx_size = msg_len;
    
    im2m_SendNetData(pThis->tx_data, pThis->tx_size); // 平台

#if ZXM2M_BZ_DEBUG
    PcDebug_Printf("ZxM2mBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
  }
}

#endif

#if (PART("ZxM2m盲区补偿"))
/******************************************************************************
* 补发重型M2M盲区数据
******************************************************************************/
void ZxM2mBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_ZXM2M_BZ_PARA_ADDR, &zxm2m_blind_zone_para); // 读取FLASH中的参数
  zxm2m_blind_zone.wr_error_cnt = zxm2m_blind_zone_para.wr_error_cnt;
  zxm2m_blind_zone.rd_error_cnt = zxm2m_blind_zone_para.rd_error_cnt;
  zxm2m_blind_zone.top = zxm2m_blind_zone_para.top;
  zxm2m_blind_zone.bottom = zxm2m_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    zxm2m_blind_zone.data[i] = zxm2m_blind_zone_para.data[i];
  }

  zxm2m_blind_zone.timer_100ms = ZXM2M_BZ_SAVE_PERIOD_SP;
  zxm2m_blind_zone.file_name = FILE_ZXM2M_BZ_DATA_ADDR; // 文件名
  zxm2m_blind_zone.frame_size = ZXM2M_BLIND_ZONE_PACKET_SIZE; // 数据帧固定长度
  pthread_mutex_init(&zxm2m_blind_zone.file_mutex, NULL);

#if ZXM2M_BZ_DEBUG
  PcDebug_Printf("ZxM2mBzInit:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
}

//=====================================================================================
void ZxM2mBlindZone_Save(void)
{
  uint16_t i;

  pthread_mutex_lock(&zxm2m_blind_zone.file_mutex);

  zxm2m_blind_zone_para.wr_error_cnt = zxm2m_blind_zone.wr_error_cnt;
  zxm2m_blind_zone_para.rd_error_cnt = zxm2m_blind_zone.rd_error_cnt;
  zxm2m_blind_zone_para.top = zxm2m_blind_zone.top;
  zxm2m_blind_zone_para.bottom = zxm2m_blind_zone.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    zxm2m_blind_zone_para.data[i] = zxm2m_blind_zone.data[i];
  }

  BlindZone_SaveParameter(FILE_ZXM2M_BZ_PARA_ADDR, &zxm2m_blind_zone_para);

  pthread_mutex_unlock(&zxm2m_blind_zone.file_mutex);
}

//==GPS及采集数据(NB)===================================================================
uint16_t ZxBlindZone_BuildTcwData(uint8_t* pdata)
{
  uint16_t msg_body_len;

  msg_body_len = iZxM2m_BuildTcwBody(pdata); // 盲区只存储重型TCW数据
  return msg_body_len;
}

//==1s调用一次============================================================================
void ZxM2mBlindZone_Service(void)
{
  // 定位有效、有工况数据
  if (COLT_GetAccStatus()==1) // ACC打开
  {
    if (zxm2m_blind_zone.timer_100ms)
      zxm2m_blind_zone.timer_100ms--;
    else
    {
      zxm2m_blind_zone.timer_100ms = ZXM2M_BZ_SAVE_PERIOD_SP; // x分钟一条
      if (im2m_GetLinkState() != SOCKET_LINK_STATE_READY) // 未连接
      {
        if (GPS_GetPositioningStatus()==1) // 终端已定位
        {
          memset(zxm2m_blind_zone_buffer,0xFF,ZXM2M_BLIND_ZONE_PACKET_SIZE); // 清空缓存
          zxm2m_blind_zone_length = ZxBlindZone_BuildTcwData(zxm2m_blind_zone_buffer); // 创建盲区数据包: GPS及ECU数据
          if (zxm2m_blind_zone_length <= ZXM2M_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&zxm2m_blind_zone, zxm2m_blind_zone_buffer, zxm2m_blind_zone_length); // 存入数据帧
            ZxM2mBlindZone_Save(); // 存入栈参数

#if ZXM2M_BZ_DEBUG
            PcDebug_Printf("ZxM2mBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC关闭
  {
    zxm2m_blind_zone.timer_100ms = ZXM2M_BZ_SAVE_PERIOD_SP; // x分钟一条
  }

  // 出现读写错误,复位栈为0
  if ((zxm2m_blind_zone.rd_error_cnt > ZXM2M_BDZE_READ_ERROR_SP) || (zxm2m_blind_zone.wr_error_cnt > ZXM2M_BDZE_WRITE_ERROR_SP))
  {
#if ZXM2M_BZ_DEBUG
    PcDebug_Printf("ZxM2mBzErr:Ewr=%d,Erd=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt);
#endif

    zxm2m_blind_zone.wr_error_cnt = 0;
    zxm2m_blind_zone.rd_error_cnt = 0;
    zxm2m_blind_zone.top = 0;
    zxm2m_blind_zone.bottom = 0;
    memset(zxm2m_blind_zone.data, 0x00, BLIND_ZONE_STACK_MAX_SIZE);
    ZxM2mBlindZone_Save(); // 存入栈参数
  }
}
#endif

