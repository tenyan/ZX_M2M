/*****************************************************************************
* @FileName: M2mZxProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-18
* @brief     重型专用TLV创建
******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "M2mZxProtocol.h"
#include "config.h"

/******************************************************************************
 * Typedef
 ******************************************************************************/
typedef uint16_t (*Zxm2m_BuildTlvMsgFun)(uint8_t *pbuf);
//typedef uint16_t (*Zxm2m_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  Zxm2m_BuildTlvMsgFun pfun_build;
  //Zxm2m_AnalyzeTlvMsgFun pfun_analyze;
}Zxm2m_CmdTlv_t;


/******************************************************************************
 *   Macros
 ******************************************************************************/


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

  if (tlv_3004_valid_flag)
  {
    pbuf[len++] = 0x30; // TAG
    pbuf[len++] = 0x04;
    pbuf[len++] = 0x00; // LENGTH
    pbuf[len++] = 0x02;
    pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR]; // VALUE
    pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];
  }

  return len;
}

//==TAG-3005终端内置电池电压==========================================================
uint16_t iZxM2m_BuildTlvMsg_3005(uint8_t *pbuf)
{
  uint16_t len = 0;

  if (tlv_3005_valid_flag)
  {
    pbuf[len++] = 0x30; // TAG
    pbuf[len++] = 0x05;
    pbuf[len++] = 0x00; // LENGTH
    pbuf[len++] = 0x02;
    pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS2_ADDR]; // VALUE
    pbuf[len++] = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS2_ADDR+1];
  }

  return len;
}

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

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
//==type按照递增顺序填写=================================================
Zxm2m_CmdTlv_t Zxm2m_CmdDealTbl[]=
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
//#define NUM_OF_ZXM2M_CMD_DEAL   (sizeof(Zxm2m_CmdDealTbl)/sizeof(Zxm2m_CmdDealTbl))



