/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: tcw.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2019-10-29
* @brief     徐工重型起重机CAN通信协议(上车(CAN2)、下车(CAN1))
* @Device ID: 2181193799(全地面)  2181193800(轮式KXCT)
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"
#include "tcw.h"


/******************************************************************************
* Define
******************************************************************************/


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void ZxSts_SetNumberFlag(zxsts_context_t* pThis);
void ZxSts_SetStartFlag(zxsts_context_t* pThis);
void ZxSts_SetStopFlag(zxsts_context_t* pThis);
void ZxSts_ResetCanMsgTimer(zxsts_context_t* pThis);

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

// 数据缓存
uint8_t zxinfo_buffer[SIZE_OF_ZXINFO_BUFFER]; /// 基本信息
uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// 上车数据缓存
uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// 下车底盘数据缓存
uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// 下车发动机数据缓存
uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///统计数据缓存
uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// 版本信息缓存

zxsts_context_t zxsts_context[NUMBER_OF_ZXSTS_TYPES]; /// 频次统计
bittype2 zxsts_flag1,zxsts_flag2;

/*************************************************************************
 * 处理接收到的uCAN报文(上车)
*************************************************************************/
uint8_t CAN_ProcessRecvUpMsg(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  bittype temp_flag;
  uint8_t retval = 0x00;
  uint16_t torque_percent;
  //uint32_t tempVal;

  switch (canId) // 上车通信
  {
  //==TAG-A504采集协议信息==========================================================
  case 0x574:  // 上车类型配置说明
   zxinfo_buffer_a504[ZXINFO_A504_POS1_ADDR] = pdata[0]; // 车型配置
   zxinfo_buffer_a504[ZXINFO_A504_POS2_ADDR] = pdata[1]; // 上车CAN协议
   break;

  //==A5A0====================================================================================
  case 0x565:
    zxup_buffer_a5a0[ZXUP_A5A0_POS1] = pdata[0]; // 力限器配置
    zxup_buffer_a5a0[ZXUP_A5A0_POS1+1] = pdata[1];
    break;

  case 0x285:
    zxup_buffer_a5a0[ZXUP_A5A0_POS2] = pdata[1]; // 工况代码(ABCDEFGH)
    zxup_buffer_a5a0[ZXUP_A5A0_POS2+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS2+2] = pdata[2];
    zxup_buffer_a5a0[ZXUP_A5A0_POS3] = pdata[0]; // 倍率
    zxup_buffer_a5a0[ZXUP_A5A0_POS6] = pdata[4]; // 主臂长度
    zxup_buffer_a5a0[ZXUP_A5A0_POS6+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS10] = pdata[6]; // 副臂长度
    zxup_buffer_a5a0[ZXUP_A5A0_POS10+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x195:
    zxup_buffer_a5a0[ZXUP_A5A0_POS4] = pdata[4]; // 主臂头部角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS4+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS5] = pdata[2]; // 主臂根部角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS5+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS15] = pdata[6]; // 臂头高度
    zxup_buffer_a5a0[ZXUP_A5A0_POS15+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x183:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS8], pdata, 8); // 节臂百分比
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1D1:
    //==风速超限=============================================
    temp_flag.byte = pdata[0];
    if(temp_flag.b.bit0)  // 风速超限
    {  zxsts_lmi_wind_speed_flag = ZXSTS_TRUE;}
    else
    {  zxsts_lmi_wind_speed_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FSCX]);
    //=======================================================
    retval = 0x01;
    break;

  case 0x283:
    zxup_buffer_a5a0[ZXUP_A5A0_POS9] = pdata[0]; // 缸臂销锁死
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x295:
    zxup_buffer_a5a0[ZXUP_A5A0_POS11] = pdata[0]; // 副臂角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS11+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS12] = pdata[2]; // 副臂/塔臂根部角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS12+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS13] = pdata[4]; // 副臂/塔臂头部角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS13+1] = pdata[5];
    zxup_buffer_a5a2[ZXUP_A5A2_POS1] = pdata[6]; // 塔臂拉力左
    zxup_buffer_a5a2[ZXUP_A5A2_POS1+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1D5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS14], pdata, 8); // 腔压力
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x185:
    //==频次统计:超载==========================
    torque_percent = ((uint16_t)(pdata[5]<<8) + pdata[4])/10;
    if(torque_percent > 100) // 超载值判断
    {  zxsts_overload_flag = ZXSTS_TRUE;}
    else
    {  zxsts_overload_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_LMI]);
    //=========================================
    
    zxup_buffer_a5a0[ZXUP_A5A0_POS16] = pdata[0]; // 额定重量
    zxup_buffer_a5a0[ZXUP_A5A0_POS16+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS17] = pdata[2]; // 实际重量
    zxup_buffer_a5a0[ZXUP_A5A0_POS17+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS18] = pdata[4]; // 力矩百分比
    zxup_buffer_a5a0[ZXUP_A5A0_POS18+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS19] = pdata[6]; // 工作幅度
    zxup_buffer_a5a0[ZXUP_A5A0_POS19+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x385:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS20], pdata, 8); // LMI故障代码1-4
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x395:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS21], pdata, 8); // LMI故障代码5-8
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3F5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS22], pdata, 4); // 非对称故障代码
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3E5:
    memcpy(&zxup_buffer_a5a0[ZXUP_A5A0_POS23], pdata, 4); // LMI运行时间
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C5:
    if (pdata[4]&BIT(0)) // 高度限位
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(0); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(0); // 清0

    zxup_buffer_a5a2[ZXUP_A5A2_POS2] = pdata[0]; // 塔臂拉力右
    zxup_buffer_a5a2[ZXUP_A5A2_POS2+1] = pdata[1];
    tlv_a5a0_valid_flag = 1;
    tlv_a5a2_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x463:
    //==频次统计:拆装开关、高限强制、变幅起强制、三圈强制、总强制====
    temp_flag.byte = pdata[2];
    if(temp_flag.b.bit0)  // 拆装开关
    {  zxsts_setup_flag = ZXSTS_TRUE;}
    else
    {  zxsts_setup_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_CZKG]);

    if(temp_flag.b.bit1)  // 高限强制
    {  zxsts_a2b_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_GXQZ]);

    if(temp_flag.b.bit2)  // 变幅起强制
    {  zxsts_luff_up_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_up_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFQQZ]);

    if(temp_flag.b.bit3)  // 三圈强制
    {  zxsts_od_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_SQQZ]);

    if(temp_flag.b.bit4)  // 总强制
    {  zxsts_lmi_force_flag = ZXSTS_TRUE;}
    else
    {  zxsts_lmi_force_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZQZ]);
    //========================================================
    
    if (pdata[2]&BIT(4)) // LMI强制
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(1); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(1); // 清0

    if (pdata[2]&BIT(0)) // 拆装工况
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(2); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(2); // 清0

    if (pdata[2]&BIT(2)) // 变幅起强制
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(3); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(3); // 清0

    if (pdata[2]&BIT(3)) // 过放强制
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(4); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(4); // 清0

    if (pdata[2]&BIT(1)) // 高度限位强制
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] |= BIT(5); // 置1
    else
      zxup_buffer_a5a0[ZXUP_A5A0_POS24] &= ~BIT(5); // 清0

    if (pdata[7]&BIT(7)) // 回油阻塞报警
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] |= BIT(0); // 置1
    else
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] &= ~BIT(0); // 清0

    if (pdata[7]&BIT(6)) // 进油堵塞报警
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] |= BIT(1); // 置1
    else
      zxup_buffer_a5b3[ZXUP_A5B3_POS3] &= ~BIT(1); // 清0

    tlv_a5b3_valid_flag = 1;
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x263:
    zxup_buffer_a5a0[ZXUP_A5A0_POS25] = pdata[0]; // 水平仪X
    zxup_buffer_a5a0[ZXUP_A5A0_POS25+1] = pdata[1];
    zxup_buffer_a5a0[ZXUP_A5A0_POS26] = pdata[2]; // 水平仪Y
    zxup_buffer_a5a0[ZXUP_A5A0_POS26+1] = pdata[3];
    zxup_buffer_a5a0[ZXUP_A5A0_POS27] = pdata[4]; // 风速
    zxup_buffer_a5a0[ZXUP_A5A0_POS27+1] = pdata[5];
    zxup_buffer_a5a0[ZXUP_A5A0_POS28] = pdata[6]; // 回转角度
    zxup_buffer_a5a0[ZXUP_A5A0_POS28+1] = pdata[7];
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x203:
    zxup_buffer_a5a0[ZXUP_A5A0_POS29] = pdata[1]; // 强制与救援
    tlv_a5a0_valid_flag = 1;
    retval = 0x01;
    break;


  //==A5A1====================================================================================
  case 0x1E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS1] = pdata[0]; // 左超起卷扬角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS1+1] = pdata[1];
    zxup_buffer_a5a1[ZXUP_A5A1_POS5] = pdata[2]; // 右超起卷扬角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS5+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS3] = pdata[4]; // 左超起展开角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS3+1] = pdata[5];
    zxup_buffer_a5a1[ZXUP_A5A1_POS7] = pdata[6]; // 右超起展开角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS7+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x2E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS2] = pdata[0]; // 左超起拉力
    zxup_buffer_a5a1[ZXUP_A5A1_POS2+1] = pdata[1];
    zxup_buffer_a5a1[ZXUP_A5A1_POS6] = pdata[2]; // 右超起拉力
    zxup_buffer_a5a1[ZXUP_A5A1_POS6+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS13] = pdata[4]; // 左超起张紧缸长度
    zxup_buffer_a5a1[ZXUP_A5A1_POS13+1] = pdata[5];
    zxup_buffer_a5a1[ZXUP_A5A1_POS14] = pdata[6]; // 右超起张紧缸长度
    zxup_buffer_a5a1[ZXUP_A5A1_POS14+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x4E3:
    zxup_buffer_a5a1[ZXUP_A5A1_POS4] = pdata[2]; // 左超起变幅角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS4+1] = pdata[3];
    zxup_buffer_a5a1[ZXUP_A5A1_POS8] = pdata[4]; // 右超起变幅角度
    zxup_buffer_a5a1[ZXUP_A5A1_POS8+1] = pdata[5];
    zxup_buffer_a5b1[ZXUP_A5B1_POS3] = pdata[6]; // 左超起张紧缸长度 *********
    zxup_buffer_a5b1[ZXUP_A5B1_POS3+1] = pdata[7];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x3E3:
    if (pdata[5]&BIT(0)) // 左超起卷扬锁止
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(0); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 左超起卷扬解锁
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(1); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(4)) // 左超起起竖到位
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(2); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(2); // 清0

    if (pdata[6]&BIT(1)) // 左超起张紧缸解锁
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(5); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(5); // 清0

    if (pdata[6]&BIT(0)) // 左超起张紧缸锁止
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(6); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(6); // 清0

    if (pdata[6]&BIT(4)) // 左超起张紧缸全缩到底
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] |= BIT(7); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS9] &= ~BIT(7); // 清0

    if (pdata[5]&BIT(2)) // 右超起卷扬锁止
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(0); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(3)) // 右超起卷扬解锁
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(1); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(5)) // 右超起起竖到位
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(2); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(2); // 清0

    if (pdata[6]&BIT(3)) // 右超起张紧缸解锁
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(5); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(5); // 清0

    if (pdata[6]&BIT(2)) // 右超起张紧缸锁止
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(6); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(6); // 清0

    if (pdata[6]&BIT(5)) // 右超起张紧缸全缩到底
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] |= BIT(7); // 置1
    else
      zxup_buffer_a5a1[ZXUP_A5A1_POS10] &= ~BIT(7); // 清0

    zxup_buffer_a5a1[ZXUP_A5A1_POS11] = pdata[0]; // 左超起齿数
    zxup_buffer_a5a1[ZXUP_A5A1_POS12] = pdata[1]; // 右超起齿数

    if (pdata[7]&BIT(2)) // 左张紧拉力过小*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(0); // 置1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(0); // 清0

    if (pdata[7]&BIT(3)) // 左张紧拉力过大*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(1); // 置1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(1); // 清0

    if (pdata[7]&BIT(4)) // 右张紧拉力过小*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(2); // 置1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(2); // 清0

    if (pdata[7]&BIT(5)) // 右张紧拉力过大*********
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] |= BIT(3); // 置1
    else
      zxup_buffer_a5b1[ZXUP_A5B1_POS4] &= ~BIT(3); // 清0

    zxup_buffer_a5b1[ZXUP_A5B1_POS2] = pdata[2]; // 超起目标张紧角度*********
    zxup_buffer_a5b1[ZXUP_A5B1_POS2+1] = pdata[3];
    tlv_a5a1_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A2====================================================================================
  case 0x3F3:
    zxup_buffer_a5a2[ZXUP_A5A2_POS3] = pdata[4]; // 防后倾压力
    zxup_buffer_a5a2[ZXUP_A5A2_POS3+1] = pdata[5];
    zxup_buffer_a5a2[ZXUP_A5A2_POS4] = pdata[4]; // 前支架角度
    zxup_buffer_a5a2[ZXUP_A5A2_POS4+1] = pdata[5];
    zxup_buffer_a5be[ZXUP_A5BE_POS2] = pdata[6]; // 卷筒转速*****
    zxup_buffer_a5be[ZXUP_A5BE_POS2+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1F3:
    if (pdata[0]&BIT(3)) // 右超起卷扬锁止
      zxup_buffer_a5a2[ZXUP_A5A2_POS5] |= BIT(0); // 置1
    else
      zxup_buffer_a5a2[ZXUP_A5A2_POS5] &= ~BIT(0); // 清0

    if (pdata[2]&BIT(0)) // 制动电磁阀
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(1); // 置1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(1); // 清0

    if (pdata[2]&BIT(1)) // 压力继电器
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(2); // 置1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(2); // 清0

    zxup_buffer_a5be[ZXUP_A5BE_POS1] = pdata[6]; // 马达电流
    zxup_buffer_a5be[ZXUP_A5BE_POS1+1] = pdata[7];
    tlv_a5a2_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A5====================================================================================
  case 0x35A:
    zxup_buffer_a5a5[ZXUP_A5A5_POS1] = pdata[3]; // 发动机转速
    zxup_buffer_a5a5[ZXUP_A5A5_POS1+1] = pdata[4];
    zxup_buffer_a5a5[ZXUP_A5A5_POS2] = pdata[2]; // 实际扭矩百分比

    if (pdata[0]&BIT(0)) // 发动机扭矩模式(调速器类型)1
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(0); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(0); // 清0
      
    if (pdata[0]&BIT(1)) // 发动机扭矩模式(调速器类型)2
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(1); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(2)) // 发动机扭矩模式(调速器类型)3
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(2); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(3)) // 发动机扭矩模式(调速器类型)4
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] |= BIT(3); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS16] &= ~BIT(3); // 清0

    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35B:
    zxup_buffer_a5a5[ZXUP_A5A5_POS3] = pdata[0]; // 摩擦扭矩百分比
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x35C:
    zxup_buffer_a5a5[ZXUP_A5A5_POS4] = pdata[1]; // 进气歧管压力
    zxup_buffer_a5a5[ZXUP_A5A5_POS5] = pdata[2]; // 进气歧管温度
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35D:
    zxup_buffer_a5a5[ZXUP_A5A5_POS6] = pdata[0]; // 冷却液温度
    zxup_buffer_a5a5[ZXUP_A5A5_POS7] = pdata[2]; // 机油温度
    zxup_buffer_a5a5[ZXUP_A5A5_POS7+1] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35E:
    if (pdata[0]&BIT(0)) // 油中有水指示灯1
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] |= BIT(0); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 油中有水指示灯2
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] |= BIT(1); // 置1
    else
      zxup_buffer_a5a5[ZXUP_A5A5_POS11] &= ~BIT(1); // 清0

    zxup_buffer_a5a5[ZXUP_A5A5_POS8] = pdata[2]; // 机油液位
    zxup_buffer_a5a5[ZXUP_A5A5_POS9] = pdata[3]; // 机油压力
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x35F:
    zxup_buffer_a5a5[ZXUP_A5A5_POS10] = pdata[0]; // 发动机总运行时间
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+1] = pdata[1];
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+2] = pdata[2];
    zxup_buffer_a5a5[ZXUP_A5A5_POS10+3] = pdata[3];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36A:
    zxup_buffer_a5a5[ZXUP_A5A5_POS12] = pdata[1]; // 油门踏板百分比
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36B:
    zxup_buffer_a5a5[ZXUP_A5A5_POS13] = pdata[0]; // 发动机燃油消耗率
    zxup_buffer_a5a5[ZXUP_A5A5_POS13+1] = pdata[1];
    zxup_buffer_a5a5[ZXUP_A5A5_POS14] = pdata[4]; // 发动机平均燃油消耗率
    zxup_buffer_a5a5[ZXUP_A5A5_POS14+1] = pdata[5];
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x36C:
    zxup_buffer_a5a5[ZXUP_A5A5_POS15] = pdata[1]; // 燃油液位
    tlv_a5a5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A6====================================================================================
  case 0x464:
    //==作业统计:主卷起和主卷落、变幅起和变幅落、副卷起和副卷落、左回转和右回转==
    temp_flag.byte = pdata[3];
    if(temp_flag.b.bit0)  // 主卷起操作
    {  zxsts_main_hoist_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_main_hoist_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit1)  // 主卷落操作
    {  zxsts_main_hoist_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_main_hoist_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJDW]);

    if(temp_flag.b.bit2)  // 左回转操作
    {  zxsts_slew_left_flag = ZXSTS_TRUE;}
    else
    {  zxsts_slew_left_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit3)  // 右回转操作
    {  zxsts_slew_right_flag = ZXSTS_TRUE;}
    else
    {  zxsts_slew_right_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZHR]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_YHR]);

    if(temp_flag.b.bit4)  // 变幅起操作
    {  zxsts_luff_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit5)  // 变幅落操作
    {  zxsts_luff_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_luff_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_BFDW]);

    if(temp_flag.b.bit6)  // 副卷起操作
    {  zxsts_deputy_hoist_up_flag = ZXSTS_TRUE;}
    else
    {  zxsts_deputy_hoist_up_flag = ZXSTS_FALSE;}

    if(temp_flag.b.bit7)  // 副卷落操作
    {  zxsts_deputy_hoist_down_flag = ZXSTS_TRUE;}
    else
    {  zxsts_deputy_hoist_down_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJUP]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJDW]);
    //=========================================================

    if (pdata[1]&BIT(0)) // 左手柄中位
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(0); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // 左手柄左开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(5); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(5); // 清0

    if (pdata[1]&BIT(2)) // 左手柄右开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(6); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(6); // 清0

    if (pdata[1]&BIT(3)) // 左手柄先导开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(7); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(7); // 清0

    if (pdata[1]&BIT(4)) // 右手柄中位
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(0); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(5)) // 右手柄左开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(5); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(5); // 清0

    if (pdata[1]&BIT(6)) // 右手柄右开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(6); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(6); // 清0

    if (pdata[1]&BIT(7)) // 右手柄先导开关
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(7); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(7); // 清0

    zxup_buffer_a5b3[ZXUP_A5B3_POS2] = pdata[2];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4E4:
    if (pdata[0]&BIT(4)) // 左手柄X正向
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(1); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(5)) // 左手柄X负向
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(2); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(6)) // 左手柄Y正向
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(3); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(3); // 清0

    if (pdata[0]&BIT(7)) // 左手柄Y负向
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] |= BIT(4); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS1] &= ~BIT(4); // 清0

    if (pdata[0]&BIT(0)) // 右手柄X正向
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(1); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(1)) // 右手柄X负向
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(2); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(2)) // 右手柄Y正向
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(3); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(3); // 清0

    if (pdata[0]&BIT(3)) // 右手柄Y负向
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] |= BIT(4); // 置1
    else
      zxup_buffer_a5a6[ZXUP_A5A6_POS4] &= ~BIT(4); // 清0

    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x363:
    zxup_buffer_a5a6[ZXUP_A5A6_POS2] = pdata[4]; // 左手柄X输出
    zxup_buffer_a5a6[ZXUP_A5A6_POS2+1] = pdata[5];
    zxup_buffer_a5a6[ZXUP_A5A6_POS3] = pdata[6]; // 左手柄Y输出
    zxup_buffer_a5a6[ZXUP_A5A6_POS3+1] = pdata[7];
    zxup_buffer_a5a6[ZXUP_A5A6_POS5] = pdata[0]; // 右手柄X输出
    zxup_buffer_a5a6[ZXUP_A5A6_POS5+1] = pdata[1];
    zxup_buffer_a5a6[ZXUP_A5A6_POS6] = pdata[2]; // 右手柄Y输出
    zxup_buffer_a5a6[ZXUP_A5A6_POS6+1] = pdata[3];
    tlv_a5a6_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A7====================================================================================
  case 0x561:
    zxup_buffer_a5a7[ZXUP_A5A7_POS1] = pdata[0]; // 产品配置(1-4)
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+1] = pdata[1];
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+2] = pdata[2];
    zxup_buffer_a5a7[ZXUP_A5A7_POS1+3] = pdata[3];
    zxversion_buffer_a505[ZXVERSION_A505_POS3] = pdata[4];  // 显示器底层版本
    zxversion_buffer_a505[ZXVERSION_A505_POS3+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS3+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;
  
  case 0x181:
    zxup_buffer_a5a7[ZXUP_A5A7_POS2] = pdata[1]; // 工况代码(ABCDEF)
    zxup_buffer_a5a7[ZXUP_A5A7_POS2+1] = pdata[3];
    zxup_buffer_a5a7[ZXUP_A5A7_POS2+2] = pdata[2];
    zxup_buffer_a5a7[ZXUP_A5A7_POS11] = pdata[4]; // 目标组合
    zxup_buffer_a5a7[ZXUP_A5A7_POS14] = pdata[5]; // 伸缩模式
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3A1:
    zxup_buffer_a5a7[ZXUP_A5A7_POS3] = pdata[0]; // 主卷起速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS4] = pdata[6]; // 主卷落速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS5] = pdata[4]; // 副卷起速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS6] = pdata[5]; // 副卷落速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS7] = pdata[2]; // 变幅起速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS8] = pdata[3]; // 变幅落速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS9] = pdata[1]; // 左回转速度
    zxup_buffer_a5a7[ZXUP_A5A7_POS10] = pdata[7]; // 右回转速度
    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x191:
    if (pdata[0]&BIT(0)) // 卷扬随动1
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(1); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(1)) // 卷扬随动2
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(2); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(4)) // 主/副卷随动
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(3); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(3); // 清0

    if (pdata[0]&BIT(3)) // 变幅补偿
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(4); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(4); // 清0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x482:
    if (pdata[2]&BIT(0)) // 取力使能
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(0); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(0); // 清0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x291:
    if (pdata[0]&BIT(0)) // 回转防摆
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(5); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(5); // 清0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x381:
    if (pdata[0]&BIT(0)) // 强制1
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(3); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(3); // 清0

    if (pdata[0]&BIT(1)) // 强制2
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(4); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(4); // 清0

    if (pdata[0]&BIT(2)) // 故障救援
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(5); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(5); // 清0
    
    if (pdata[3]&BIT(0)) // 塔臂拆装工况1
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(6); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(6); // 清0

    if (pdata[3]&BIT(1)) // 塔臂拆装工况2
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] |= BIT(7); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS12] &= ~BIT(7); // 清0

    if (pdata[3]&BIT(2)) // 塔臂工况变换1
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(0); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(0); // 清0

    if (pdata[3]&BIT(3)) // 塔臂工况变换2
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(1); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(1); // 清0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x391:
    if (pdata[0]&BIT(0)) // 塔臂自动
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] |= BIT(2); // 置1
    else
      zxup_buffer_a5a7[ZXUP_A5A7_POS13] &= ~BIT(2); // 清0

    tlv_a5a7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A8====================================================================================
  case 0x382:
    memcpy(&zxup_buffer_a5a8[ZXUP_A5A8_POS1], pdata, 8); // 超起操控
    zxup_buffer_a5b1[ZXUP_A5B1_POS1] = pdata[3]; // 超起目标展开角度
    tlv_a5a8_valid_flag = 1;
    tlv_a5b1_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x4C1:
    zxup_buffer_a5a8[ZXUP_A5A8_POS2] = pdata[0]; // 超起维修1+2
    zxup_buffer_a5a8[ZXUP_A5A8_POS2+1] = pdata[1];
    tlv_a5a8_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A9====================================================================================
  case 0x1B1:
    zxup_buffer_a5a9[ZXUP_A5A9_POS1] = pdata[0]; // 面板1
    zxup_buffer_a5a9[ZXUP_A5A9_POS1+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B2:
    zxup_buffer_a5a9[ZXUP_A5A9_POS2] = pdata[0]; // 面板2
    zxup_buffer_a5a9[ZXUP_A5A9_POS2+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x1B3:
    zxup_buffer_a5a9[ZXUP_A5A9_POS3] = pdata[0]; // 面板3
    zxup_buffer_a5a9[ZXUP_A5A9_POS3+1] = pdata[1];
    tlv_a5a9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AA====================================================================================
  case 0x188:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS1], pdata, 8); // Msg1
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x288:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS2], pdata, 8); // Msg2
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x388:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS3], pdata, 8); // Msg3
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x488:
    memcpy(&zxup_buffer_a5aa[ZXUP_A5AA_POS4], pdata, 8); // Msg4
    tlv_a5aa_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AB====================================================================================
  case 0x343:
    memcpy(&zxup_buffer_a5ab[ZXUP_A5AB_POS1], pdata, 8); // 节点状态
    tlv_a5ab_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AC====================================================================================
  case 0x493:
    memcpy(&zxup_buffer_a5ac[ZXUP_A5AC_POS1], pdata, 6); // 伸缩限制和解除
    zxup_buffer_a5b2[ZXUP_A5B2_POS3] = pdata[6]; // 塔臂限制伸臂***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS4] = pdata[7]; // 塔臂限制缩臂***********
    tlv_a5ac_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AD====================================================================================
  case 0x4C3:
    memcpy(&zxup_buffer_a5ad[ZXUP_A5AD_POS1], pdata, 8); // 起落限制和解除
    zxup_buffer_a5b2[ZXUP_A5B2_POS5] = pdata[6]; // 塔臂限制变幅起***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS6] = pdata[7]; // 塔臂限制变幅落***********
    tlv_a5ad_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AE====================================================================================
  case 0x4B3:
    zxup_buffer_a5ae[ZXUP_A5AE_POS1] = pdata[0]; // 左回限制1
    zxup_buffer_a5ae[ZXUP_A5AE_POS2] = pdata[4]; // 左回限制2
    zxup_buffer_a5ae[ZXUP_A5AE_POS3] = pdata[1]; // 左回解除
    zxup_buffer_a5ae[ZXUP_A5AE_POS4] = pdata[2]; // 右回限制1
    zxup_buffer_a5ae[ZXUP_A5AE_POS5] = pdata[5]; // 右回限制2
    zxup_buffer_a5ae[ZXUP_A5AE_POS6] = pdata[3]; // 右回解除
    tlv_a5ae_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5AF====================================================================================
  case 0x4A3:
    memcpy(&zxup_buffer_a5af[ZXUP_A5AF_POS1], pdata, 5); // 主卷扬起落限制和解除
    zxup_buffer_a5b2[ZXUP_A5B2_POS7] = pdata[5]; // 塔臂限制主卷起***********
    zxup_buffer_a5b2[ZXUP_A5B2_POS8] = pdata[6]; // 塔臂限制主卷落***********
    tlv_a5af_valid_flag = 1;
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

    //==A5B0====================================================================================
  case 0x4D3:
    memcpy(&zxup_buffer_a5b0[ZXUP_A5B0_POS1], &pdata[3], 4); // 副卷扬起落限制和解除
    tlv_a5b0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B1====================================================================================
  // zxup_buffer

  //==A5B2====================================================================================
  case 0x4F3:
    zxup_buffer_a5b2[ZXUP_A5B2_POS1] = pdata[0]; // 塔卷起限制
    zxup_buffer_a5b2[ZXUP_A5B2_POS1+1] = pdata[1];
    zxup_buffer_a5b2[ZXUP_A5B2_POS2] = pdata[2]; // 塔卷落限制
    zxup_buffer_a5b2[ZXUP_A5B2_POS2+1] = pdata[3];
    tlv_a5b2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B3====================================================================================
  case 0x564:
    zxup_buffer_a5b3[ZXUP_A5B3_POS1] = pdata[0]; // 液压油温度
    zxup_buffer_a5b3[ZXUP_A5B3_POS1+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS10] = pdata[4]; // LS1压力*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS10+1] = pdata[5];
    zxup_buffer_a5b5[ZXUP_A5B5_POS12] = pdata[6]; // LS2压力*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS12+1] = pdata[7];
    tlv_a5b3_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B4====================================================================================
  case 0x1A3:
    zxup_buffer_a5b4[ZXUP_A5B4_POS1] = pdata[0]; // 1#主泵电磁阀
    zxup_buffer_a5b4[ZXUP_A5B4_POS1+1] = pdata[1];
    zxup_buffer_a5b4[ZXUP_A5B4_POS2] = pdata[2]; // 2#主泵电磁阀
    zxup_buffer_a5b4[ZXUP_A5B4_POS2+1] = pdata[3];

    zxup_buffer_a5b5[ZXUP_A5B5_POS5] = pdata[0]; // 主卷起电磁阀*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS5+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS6] = pdata[2]; // 主卷落电磁阀*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS6+1] = pdata[3];

    zxup_buffer_a5bb[ZXUP_A5BB_POS1] = pdata[0]; // 起升电磁阀*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS1+1] = pdata[1];
    zxup_buffer_a5bb[ZXUP_A5BB_POS2] = pdata[2]; // 下落电磁阀*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS2+1] = pdata[3];

    zxup_buffer_a5bc[ZXUP_A5BC_POS1] = pdata[4]; // 马达电流
    zxup_buffer_a5bc[ZXUP_A5BC_POS1+1] = pdata[5];
    zxup_buffer_a5bc[ZXUP_A5BC_POS2] = pdata[6]; // 卷筒转速
    zxup_buffer_a5bc[ZXUP_A5BC_POS2+1] = pdata[7];
    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2A3:
    zxup_buffer_a5b4[ZXUP_A5B4_POS3] = pdata[0]; // 主泵压力
    zxup_buffer_a5b4[ZXUP_A5B4_POS3+1] = pdata[1];

    zxup_buffer_a5b5[ZXUP_A5B5_POS9] = pdata[0]; // MP1压力*****
    zxup_buffer_a5b5[ZXUP_A5B5_POS9+1] = pdata[1];

    zxup_buffer_a5bb[ZXUP_A5BB_POS3] = pdata[0]; // 油泵压力*****
    zxup_buffer_a5bb[ZXUP_A5BB_POS3+1] = pdata[1];

    if (pdata[6]&BIT(0)) // 制动电磁阀*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(1); // 置1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(1); // 清0

    if (pdata[6]&BIT(1)) // 压力继电器*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(2); // 置1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(2); // 清0

    tlv_a5b4_valid_flag = 1;
    tlv_a5b5_valid_flag = 1;
    tlv_a5bb_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B5====================================================================================
  case 0x393:
    zxup_buffer_a5b5[ZXUP_A5B5_POS1] = pdata[0]; // 伸电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS1+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS2] = pdata[2]; // 缩电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS2+1] = pdata[3];
    zxup_buffer_a5b5[ZXUP_A5B5_POS11] = pdata[4]; // MP2压力
    zxup_buffer_a5b5[ZXUP_A5B5_POS11+1] = pdata[5];
    zxup_buffer_a5b7[ZXUP_A5B7_POS1] = pdata[6]; // 伸缩缸压力*******
    zxup_buffer_a5b7[ZXUP_A5B7_POS1+1] = pdata[7];
    tlv_a5b5_valid_flag = 1;
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1C3:
    zxup_buffer_a5b5[ZXUP_A5B5_POS3] = pdata[0]; // 变幅起电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS3+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS4] = pdata[2]; // 变幅落电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS4+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x3D3:
    zxup_buffer_a5b5[ZXUP_A5B5_POS7] = pdata[0]; // 副卷落电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS7+1] = pdata[1];
    zxup_buffer_a5b5[ZXUP_A5B5_POS8] = pdata[2]; // 副卷落电磁阀
    zxup_buffer_a5b5[ZXUP_A5B5_POS8+1] = pdata[3];
    tlv_a5b5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x443:
    //==频次统计:主卷和副卷三圈保护、主臂和副臂高限触发======
    temp_flag.byte = pdata[0];
    if(temp_flag.b.bit3)  // 主卷三圈保护
    {  zxsts_od_main_hoist_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_main_hoist_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZJSQ]);

    if(temp_flag.b.bit4)  // 副卷三圈保护
    {  zxsts_od_deputy_hoist_flag = ZXSTS_TRUE;}
    else
    {  zxsts_od_deputy_hoist_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FJSQ]);

    temp_flag.byte = pdata[1];
    if(temp_flag.b.bit0)  // 主臂高限触发
    {  zxsts_a2b_main_arm_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_main_arm_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_ZBGX]);
    
    if(temp_flag.b.bit3)  // 副臂高限触发
    {  zxsts_a2b_deputy_arm_flag = ZXSTS_TRUE;}
    else
    {  zxsts_a2b_deputy_arm_flag = ZXSTS_FALSE;}
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_FBGX]);
    //===================================================

    if (pdata[3]&BIT(3)) // 合流电磁阀
      zxup_buffer_a5b5[ZXUP_A5B5_POS13] |= BIT(0); // 置1
    else
      zxup_buffer_a5b5[ZXUP_A5B5_POS13] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(0)) // 马达失速检测*****
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] |= BIT(0); // 置1
    else
      zxup_buffer_a5bc[ZXUP_A5BC_POS3] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 马达失速检测*****
      zxup_buffer_a5be[ZXUP_A5BE_POS3] |= BIT(0); // 置1
    else
      zxup_buffer_a5be[ZXUP_A5BE_POS3] &= ~BIT(0); // 清0

    tlv_a5b5_valid_flag = 1;
    tlv_a5bc_valid_flag = 1;
    tlv_a5be_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B6====================================================================================
  case 0x264:
    if (pdata[0]&BIT(0)) // 32MPa压力
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(0); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 24MPa压力1
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(1); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(1); // 清0
      
    if (pdata[0]&BIT(2)) // 24MPa压力2
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(3); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(3); // 清0
      
    if (pdata[0]&BIT(3)) // 1Mpa压力
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(4); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(4); // 清0

    tlv_a5b6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x193:
    //==频次统计:空缸伸和空缸缩、带臂伸和带臂缩===============
    temp_flag.byte = pdata[4];
    if((temp_flag.b.bit0==0) && (temp_flag.b.bit4==0) && (temp_flag.b.bit5==1)) // 空缸判断
    {  zxsts_empty_cylinder_flag = ZXSTS_TRUE;}
    else
    {  zxsts_empty_cylinder_flag = ZXSTS_FALSE;}

    temp_flag.byte = pdata[4];
    if((temp_flag.b.bit0==1) && (temp_flag.b.bit1==0)) // 带臂标志
    {  zxsts_arm_work_flag = ZXSTS_TRUE;}
    else
    {  zxsts_arm_work_flag = ZXSTS_FALSE;}
    
    temp_flag.byte = pdata[3];
    if(temp_flag.b.bit0)  // 伸缩缸缩操作
    {  zxsts_cylinder_shrink_flag = ZXSTS_TRUE;}
    else
    {  zxsts_cylinder_shrink_flag = ZXSTS_FALSE;}
    
    if(temp_flag.b.bit1)  // 伸缩缸伸操作
    {  zxsts_cylinder_extend_flag = ZXSTS_TRUE;}
    else
    {  zxsts_cylinder_extend_flag = ZXSTS_FALSE;}

    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_KGEX]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_KGS]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_DBEX]);
    ZxSts_ResetCanMsgTimer(&zxsts_context[ZXSTS_TYPE_DBS]);
    //=========================================
  
    if (pdata[5]&BIT(5)) // 伸电磁阀
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(2); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(6)) // 缩电磁阀
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(5); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(5); // 清0

    ///////////////////////////////////////////////////////////////////////
    if (pdata[0]&BIT(0)) // 后缸销锁死
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(0); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 后缸销解锁
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(1); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(5)) // 后臂销锁死
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(2); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(4)) // 后臂销解锁
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(3); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(3); // 清0

    if (pdata[4]&BIT(0)) // 前缸销锁死
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(4); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(4); // 清0

    if (pdata[4]&BIT(1)) // 前缸销解锁
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(5); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(5); // 清0

    if (pdata[4]&BIT(5)) // 前臂销锁死
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(6); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(6); // 清0

    if (pdata[4]&BIT(4)) // 前臂销解锁
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] |= BIT(7); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS1] &= ~BIT(7); // 清0

    if (pdata[0]&BIT(6)) // 前缸头体标志
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] |= BIT(0); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(7)) // 后缸头体标志
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] |= BIT(1); // 置1
    else
      zxup_buffer_a5b8[ZXUP_A5B8_POS2] &= ~BIT(1); // 清0

    zxup_buffer_a5b8[ZXUP_A5B8_POS5] = pdata[1]; // 臂位

    if (pdata[5]&BIT(0)) // 缸臂销供油电磁阀
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] |= BIT(0); // 置1
    else
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 缸臂销切换电磁阀
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] |= BIT(1); // 置1
    else
      zxup_buffer_a5b9[ZXUP_A5B9_POS1] &= ~BIT(1); // 清0

    tlv_a5b6_valid_flag = 1;
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2C3:
    if (pdata[2]&BIT(0)) // 变幅起电磁阀
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(6); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(6); // 清0

    if (pdata[2]&BIT(3)) // 变幅动力下落阀
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] |= BIT(7); // 置1
    else
      zxup_buffer_a5b6[ZXUP_A5B6_POS1] &= ~BIT(7); // 清0

    zxup_buffer_a5ba[ZXUP_A5BA_POS1] = pdata[6]; // 左变幅平衡阀电流
    zxup_buffer_a5ba[ZXUP_A5BA_POS1+1] = pdata[7];
    tlv_a5b6_valid_flag = 1;
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B7====================================================================================
  case 0x293:
    zxup_buffer_a5b7[ZXUP_A5B7_POS2] = pdata[2]; // 伸缩缸长度
    zxup_buffer_a5b7[ZXUP_A5B7_POS2+1] = pdata[3];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x394:
    zxup_buffer_a5b7[ZXUP_A5B7_POS3] = pdata[6]; // 电控平衡阀
    zxup_buffer_a5b7[ZXUP_A5B7_POS3+1] = pdata[7];
    tlv_a5b7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B8====================================================================================
  case 0x1E9:
    zxup_buffer_a5b8[ZXUP_A5B8_POS3] = pdata[0]; // 前缸头体检测开关8
    zxup_buffer_a5b8[ZXUP_A5B8_POS4] = pdata[1]; // 后缸头体检测开关8
    zxup_buffer_a5b9[ZXUP_A5B9_POS2] = pdata[6]; // 蓄能器压力
    zxup_buffer_a5b9[ZXUP_A5B9_POS2+1] = pdata[7];
    tlv_a5b8_valid_flag = 1;
    tlv_a5b9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5B9====================================================================================

  //==A5BA====================================================================================
  case 0x3C3:
    zxup_buffer_a5ba[ZXUP_A5BA_POS2] = pdata[0]; // 右变幅平衡阀电流
    zxup_buffer_a5ba[ZXUP_A5BA_POS2+1] = pdata[1];
    tlv_a5ba_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BB====================================================================================
  //==A5BC====================================================================================

  //==A5BD====================================================================================
  case 0x2F3:
    zxup_buffer_a5bd[ZXUP_A5BD_POS1] = pdata[0]; // 起升电磁阀
    zxup_buffer_a5bd[ZXUP_A5BD_POS1+1] = pdata[1];
    zxup_buffer_a5bd[ZXUP_A5BD_POS2] = pdata[2]; // 下落电磁阀
    zxup_buffer_a5bd[ZXUP_A5BD_POS2+1] = pdata[3];
    zxup_buffer_a5bd[ZXUP_A5BD_POS3] = pdata[4]; // 油泵压力
    zxup_buffer_a5bd[ZXUP_A5BD_POS3+1] = pdata[5];
    tlv_a5bd_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5BE====================================================================================
  // zxup_buffer_a5be[ZXUP_A5BE_POS1]

  //==A5BF====================================================================================
  case 0x2B3:
    zxup_buffer_a5bf[ZXUP_A5BF_POS1] = pdata[0]; // 左回转电磁阀
    zxup_buffer_a5bf[ZXUP_A5BF_POS1+1] = pdata[1];
    zxup_buffer_a5bf[ZXUP_A5BF_POS2] = pdata[2]; // 右回转电磁阀
    zxup_buffer_a5bf[ZXUP_A5BF_POS2+1] = pdata[3];
    zxup_buffer_a5bf[ZXUP_A5BF_POS4] = pdata[6]; // 油泵压力（回转压力检测）
    zxup_buffer_a5bf[ZXUP_A5BF_POS4+1] = pdata[7];
    tlv_a5bf_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C0====================================================================================
  case 0x3B3:
    if (pdata[0]&BIT(0)) // 制动控制阀
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] |= BIT(0); // 置1
    else
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 压力检测
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] |= BIT(1); // 置1
    else
      zxup_buffer_a5c0[ZXUP_A5C0_POS1] &= ~BIT(1); // 清0

    zxup_buffer_a5c0[ZXUP_A5C0_POS2] = pdata[6]; // 回转制动阀-工作压力
    zxup_buffer_a5c0[ZXUP_A5C0_POS2+1] = pdata[7];
    tlv_a5c0_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C1====================================================================================
  case 0x364:
    zxup_buffer_a5c1[ZXUP_A5C1_POS1] = pdata[4];
    zxup_buffer_a5c1[ZXUP_A5C1_POS2] = pdata[5];
    tlv_a5c1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C2====================================================================================
  case 0x3E4:
    zxup_buffer_a5c2[ZXUP_A5C2_POS1] = pdata[6];  // 压力选择
    zxup_buffer_a5c2[ZXUP_A5C2_POS1+1] = pdata[7];

    if (pdata[5]&BIT(0)) // 切换阀（Y35）
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(0); // 置1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 切换阀Ａ(Y33A)
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(1); // 置1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(2)) // 切换阀Ｂ(Y33B)
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] |= BIT(2); // 置1
    else
      zxup_buffer_a5c2[ZXUP_A5C2_POS2] &= ~BIT(2); // 清0
    
    tlv_a5c2_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5C3====================================================================================
  //==A5C4====================================================================================
  case 0x273:
    if (pdata[0]&BIT(0)) // 左-超起变幅起
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(0); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 左-超起变幅落
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(1); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(1); // 清0
    
    if (pdata[0]&BIT(2)) // 左-超起展开
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(2); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(2); // 清0
    
    if (pdata[0]&BIT(3)) // 左-超起收回
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(3); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(3); // 清0
    
    zxup_buffer_a5c3[ZXUP_A5C3_POS1] = pdata[2]; // 左-卷扬起
    zxup_buffer_a5c3[ZXUP_A5C3_POS1+1] = pdata[3];
    zxup_buffer_a5c3[ZXUP_A5C3_POS2] = pdata[6]; // 左-卷扬落
    zxup_buffer_a5c3[ZXUP_A5C3_POS2+1] = pdata[7];
    zxup_buffer_a5c3[ZXUP_A5C3_POS7] = pdata[4]; // 左-张紧缸缩
    zxup_buffer_a5c3[ZXUP_A5C3_POS7+1] = pdata[5];
    tlv_a5c3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x373:
    if (pdata[0]&BIT(7)) // 左-卷扬压力选择
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(4); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(4); // 清0

    if (pdata[1]&BIT(3)) // 左-卷扬制动
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(5); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(5); // 清0
    
    if (pdata[1]&BIT(1)) // 左-棘轮解锁
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(6); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(6); // 清0
    
    if (pdata[1]&BIT(0)) // 左-棘轮上锁
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] |= BIT(7); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS4] &= ~BIT(7); // 清0
    
    if (pdata[1]&BIT(2)) // 左-张紧缸解锁
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] |= BIT(0); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] &= ~BIT(0); // 清0
    
    if (pdata[1]&BIT(4)) // 左-超起卷扬浮动
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] |= BIT(1); // 置1
    else
      zxup_buffer_a5c3[ZXUP_A5C3_POS5] &= ~BIT(1); // 清0
    
    zxup_buffer_a5c4[ZXUP_A5C4_POS1] = pdata[2]; // 右-卷扬起
    zxup_buffer_a5c4[ZXUP_A5C4_POS1+1] = pdata[3];
    zxup_buffer_a5c4[ZXUP_A5C4_POS2] = pdata[6]; // 右-卷扬落
    zxup_buffer_a5c4[ZXUP_A5C4_POS2+1] = pdata[7];
    zxup_buffer_a5c4[ZXUP_A5C4_POS7] = pdata[4]; // 右-张紧缸缩
    zxup_buffer_a5c4[ZXUP_A5C4_POS7+1] = pdata[5];
    
    if (pdata[0]&BIT(0)) // 右-超起变幅起
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(0); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // 右-超起变幅落
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(1); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(1); // 清0
    
    if (pdata[0]&BIT(2)) // 右-超起展开
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(2); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(2); // 清0
    
    if (pdata[0]&BIT(3)) // 右-超起收回
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(3); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(3); // 清0

    if (pdata[0]&BIT(7)) // 右-卷扬压力选择
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(4); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(4); // 清0

    if (pdata[1]&BIT(3)) // 右-卷扬制动
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(5); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(5); // 清0
    
    if (pdata[1]&BIT(1)) // 右-棘轮解锁
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(6); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(6); // 清0
    
    if (pdata[1]&BIT(0)) // 右-棘轮上锁
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] |= BIT(7); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS4] &= ~BIT(7); // 清0
    
    if (pdata[1]&BIT(2)) // 右-张紧缸解锁
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] |= BIT(0); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] &= ~BIT(0); // 清0
    
    if (pdata[1]&BIT(4)) // 右-超起卷扬浮动
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] |= BIT(1); // 置1
    else
      zxup_buffer_a5c4[ZXUP_A5C4_POS5] &= ~BIT(1); // 清0
    
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x504:
    zxup_buffer_a5c3[ZXUP_A5C3_POS3] = pdata[0]; // 左-超起马达变量
    zxup_buffer_a5c3[ZXUP_A5C3_POS3+1] = pdata[1];
    zxup_buffer_a5c4[ZXUP_A5C4_POS3] = pdata[2]; // 右-超起马达变量
    zxup_buffer_a5c4[ZXUP_A5C4_POS3+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x473:
    zxup_buffer_a5c3[ZXUP_A5C3_POS6] = pdata[0]; // 左-张紧缸伸
    zxup_buffer_a5c3[ZXUP_A5C3_POS6+1] = pdata[1];
    zxup_buffer_a5c4[ZXUP_A5C4_POS6] = pdata[2]; // 右-张紧缸伸
    zxup_buffer_a5c4[ZXUP_A5C4_POS6+1] = pdata[3];
    tlv_a5c3_valid_flag = 1;
    tlv_a5c4_valid_flag = 1;
    retval = 0x01;
    break;

  //==上车系统版本====================================================================================
  case 0x562:
    zxversion_buffer_a505[ZXVERSION_A505_POS7] = pdata[0];  // 显示器2底层
    zxversion_buffer_a505[ZXVERSION_A505_POS7+1] = pdata[1];
    zxversion_buffer_a505[ZXVERSION_A505_POS7+2] = pdata[2];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x571:
    zxversion_buffer_a505[ZXVERSION_A505_POS2] = pdata[4];  // 显示器1
    zxversion_buffer_a505[ZXVERSION_A505_POS2+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS2+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x572:
    zxversion_buffer_a505[ZXVERSION_A505_POS6] = pdata[4];  // 显示器2
    zxversion_buffer_a505[ZXVERSION_A505_POS6+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS6+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;

  case 0x573:
    zxversion_buffer_a505[ZXVERSION_A505_POS5] = pdata[4];  // 控制器
    zxversion_buffer_a505[ZXVERSION_A505_POS5+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS5+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x00;
    break;
    
  case 0x575:
    zxversion_buffer_a505[ZXVERSION_A505_POS1] = pdata[4];  // 力矩限制器
    zxversion_buffer_a505[ZXVERSION_A505_POS1+1] = pdata[5];
    zxversion_buffer_a505[ZXVERSION_A505_POS1+2] = pdata[6];
    tlv_a505_valid_flag = 1;
    retval = 0x01;
    break;

  default:
    break;
  }

  return retval;
}

/*************************************************************************
 * 处理接收到的dCAN报文(下车)(汽车式起重机Autocrane )
*************************************************************************/
uint8_t CAN_ProcessRecvDownMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // 下车通信
  {
  //==A5E0===================================================================================
  case 0x1CF23B21:
    zxdown_buffer_a5e0[ZXDOWN_A5E0_POS1] = pdata[0]; // 节点状态1
    
    if (pdata[1]&BIT(0)) // 变矩器总线信号
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // ABS总线信号
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(4)) // 电涡流温度高检测
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(3)) // 制动器磨损检测
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(2); // 清0
    
    if (pdata[1]&BIT(3)) // 制动蹄片磨损检测开关 ?????????????????????
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(3); // 清0
    
    if (pdata[1]&BIT(2)) // ABS激活状态
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(6); // 清0
    
    tlv_a5e0_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E1====================================================================================
  case 0x1CF21D21:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS1] = pdata[7];
    if (pdata[3]&BIT(4)) // 变速箱空档检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(0); // 清0
    
    if (pdata[3]&BIT(5)) // 变速箱倒档检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(0)) // 二轴驱动检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(2); // 清0
    
    if (pdata[1]&BIT(1)) // 分动箱高档检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(3); // 清0
    
    if (pdata[1]&BIT(2)) // 分动箱低档检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(4); // 清0

    if (pdata[1]&BIT(3)) // 轮间差速检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(5); // 清0
    
    if (pdata[1]&BIT(4)) // 轴间差速检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(6); // 清0

    if (pdata[6]&BIT(6)) // 手刹检测(手制动检测开关)
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(0); // 清0
    
    if (pdata[3]&BIT(6)) // 行车制动检测开关
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(4); // 清0
    
    if (pdata[2]&BIT(5)) // 低气压报警
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(5); // 清0
    
    if (pdata[6]&BIT(7)) // PTO检测
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(0); // 清0

    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x1CF25E21:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3] = pdata[0];  // 变速箱油温
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3+1] = pdata[1];
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS4] = pdata[3];  // 变速箱进气气压
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS5] = pdata[4];  // 离合器位置 
    
    if (pdata[5]&BIT(0)) // 发动机制动
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(7); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(7); // 清0
    
    if (pdata[5]&BIT(1)) // 电涡流制动1挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(0); // 清0
    
    if (pdata[5]&BIT(2)) // 电涡流制动2挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(3)) // 电涡流制动3挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(2); // 清0
    
    if (pdata[5]&BIT(4)) // 电涡流制动4挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(3); // 清0
    
    if (pdata[5]&BIT(5)) // 发动机熄火输入
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(6)) // 发动机熄火输出
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(2); // 清0
    
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5] = pdata[0];  // 缓速器油温
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS6] = pdata[2];  // 缓速力矩百分比

    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E5====================================================================================
  case 0x1CF22D21:
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS1] = pdata[0];  // 当前一轴转角
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS1+1] = pdata[1];
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF28B21:
    if (pdata[1]&BIT(0)) // 转向系统报警指示灯E101
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // 中位指示灯
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e5[ZXDOWN_A5E5_POS6] &= ~BIT(1); // 清0
    
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS2] = pdata[4];  // 转向锁死压力(bar)
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS2+1] = pdata[5];
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS4] = pdata[2];  // 程序标记位
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS5] = pdata[0];  // 阀字节状态位1
    tlv_a5e5_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF25C21:
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS3] = pdata[0];  // 目标转向模式
    zxdown_buffer_a5e5[ZXDOWN_A5E5_POS3+1] = pdata[1];// 当前转向模式
    tlv_a5e5_valid_flag = 1;

    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2] = pdata[6]; // 液压系统压力
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2+1] = pdata[7];
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5E6====================================================================================
  case 0x1CF25D21:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1] = pdata[0];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2] = pdata[2];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2+1] = pdata[3];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3] = pdata[4];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3+1] = pdata[5];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4] = pdata[6];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4+1] = pdata[7];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF23A21:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7] = pdata[4];  // 变矩器油温
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7+1] = pdata[5];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E7====================================================================================
  // 分散

  //==A5E9====================================================================================
  case 0x1CF24B21:
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS1] = pdata[6]; // 液压油温度
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5EB====================================================================================
  case 0x1CF28C21:
    if(pdata[0]>0 && pdata[0]<19) // 1号轮胎到18号轮胎
    {
      zxdown_buffer_a5eb[ZXDOWN_A5EB_POS1+pdata[0]-1] = pdata[1];
      tlv_a5eb_valid_flag = 1;
    }
    retval = 0x01;
    break;

  //==下车系统版本====================================================================================
  case 0x1CF20F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS3] = pdata[0];  // P1应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS3+1] = pdata[1];
    zxversion_buffer_a506[ZXVERSION_A506_POS3+2] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS5] = pdata[3];  // P2应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS5+1] = pdata[4];
    zxversion_buffer_a506[ZXVERSION_A506_POS5+2] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS7] = pdata[6];  // P3应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS7+1] = pdata[7];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF21F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS7+2] = pdata[0];  // P3应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS9] = pdata[1];    // P4应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS9+1] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS9+2] = pdata[3];
    zxversion_buffer_a506[ZXVERSION_A506_POS11] = pdata[4];   // P5应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS11+1] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS11+2] = pdata[6];
    zxversion_buffer_a506[ZXVERSION_A506_POS17] = pdata[7];   // P8应用层
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x1CF22F21:
    zxversion_buffer_a506[ZXVERSION_A506_POS17+1] = pdata[0];   // P8应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS17+2] = pdata[1];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x1CF29F21:
    if(pdata[0]==1)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS4] = pdata[1];  // P1底层
      zxversion_buffer_a506[ZXVERSION_A506_POS4+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS4+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==2)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS6] = pdata[1];  // P2底层
      zxversion_buffer_a506[ZXVERSION_A506_POS6+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS6+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==3)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS8] = pdata[1];  // P3底层
      zxversion_buffer_a506[ZXVERSION_A506_POS8+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS8+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==4)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS10] = pdata[1];  // P4底层
      zxversion_buffer_a506[ZXVERSION_A506_POS10+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS10+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==5)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS12] = pdata[1];  // P5底层
      zxversion_buffer_a506[ZXVERSION_A506_POS12+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS12+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==6)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS14] = pdata[1];  // P6底层
      zxversion_buffer_a506[ZXVERSION_A506_POS14+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS14+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==7)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS16] = pdata[1];  // P7底层
      zxversion_buffer_a506[ZXVERSION_A506_POS16+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS16+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==8)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS18] = pdata[1];  // P8底层
      zxversion_buffer_a506[ZXVERSION_A506_POS18+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS18+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    retval = 0x01;
    break;


  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * 处理接收到的dCAN报文(下车)(全地面起重机All-Terrain Crane)
*************************************************************************/
uint8_t CAN_ProcessRecvDownMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // 下车通信
  {
  //==A5A3====================================================================================
  case 0x24D:
    memcpy(&zxdown_buffer_a5a3[ZXDOWN_A5A3_POS1], pdata, 8); // 水平支腿长度
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x22A:
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2] = pdata[0];   // 左前支腿压力
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+1] = pdata[1];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+2] = pdata[2]; // 右前支腿压力
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+3] = pdata[3];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+4] = pdata[4]; // 左后支腿压力
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+5] = pdata[5];
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+6] = pdata[6]; // 右后支腿压力
    zxdown_buffer_a5a3[ZXDOWN_A5A3_POS2+7] = pdata[7];
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x24E:
    memcpy(&zxdown_buffer_a5a3[ZXDOWN_A5A3_POS3], pdata, 8); // 摆缸长度
    tlv_a5a3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5A4====================================================================================
  case 0x24B:
    if (pdata[1]&BIT(4)) // 液压离合器状态
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] &= ~BIT(1); // 清0
    
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS1] = pdata[0]; // 辅助支腿状态
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS1] = pdata[6]; // 液压油温度
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS8] = pdata[2]; // 散热器液压驱动泵控制电流
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS8+1] = pdata[3];
    tlv_a5a4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x28D:
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS2] = pdata[0]; // 左前辅助支腿压力
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS2+1] = pdata[1];
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS3] = pdata[2]; // 右前辅助支腿压力
    zxdown_buffer_a5a4[ZXDOWN_A5A4_POS3+1] = pdata[3]; 
    tlv_a5a4_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E0===================================================================================
  case 0x23B:
    zxdown_buffer_a5e0[ZXDOWN_A5E0_POS1] = pdata[0]; // 节点状态1
    
    if (pdata[1]&BIT(0)) // 变矩器总线信号
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // ABS总线信号
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e0[ZXDOWN_A5E0_POS2] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(4)) // 电涡流温度高检测
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(3)) // 制动器磨损检测
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(2); // 清0
    
    if (pdata[1]&BIT(3)) // 制动蹄片磨损检测开关 ?????????????????????
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(3); // 清0
    
    if (pdata[1]&BIT(2)) // ABS激活状态
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(6); // 清0
    
    tlv_a5e0_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E1====================================================================================
  case 0x21D:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS1] = pdata[7];
    if (pdata[0]&BIT(0)) // 分动箱空挡
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(1)) // 取力standby
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(1); // 清0
    
    if (pdata[3]&BIT(0)) // bit0:进入ECO模式
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(2); // 清0
    
    if (pdata[0]&BIT(2)) // 取力正常
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e8[ZXDOWN_A5E8_POS2] &= ~BIT(3); // 清0
    
    if (pdata[3]&BIT(4)) // 变速箱空档检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(0); // 清0
    
    if (pdata[3]&BIT(5)) // 变速箱倒档检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(1); // 清0
    
    if (pdata[1]&BIT(0)) // 二轴驱动检测 校对
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(2); // 清0
    
    if (pdata[1]&BIT(1)) // 分动箱高档检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(3); // 清0
    
    if (pdata[1]&BIT(2)) // 分动箱低档检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(4); // 清0

    if (pdata[1]&BIT(3)) // 轮间差速检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(5); // 清0
    
    if (pdata[1]&BIT(4)) // 轴间差速检测
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e1[ZXDOWN_A5E1_POS2] &= ~BIT(6); // 清0
    
    if (pdata[6]&BIT(2)) // 分动箱低档检测
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] |= BIT(0); // 置1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS2] &= ~BIT(0); // 清0
    
    if (pdata[6]&BIT(6)) // 手刹检测(手制动检测开关)
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(0); // 清0
    
    if (pdata[3]&BIT(6)) // 行车制动检测开关
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(4); // 清0
    
    if (pdata[2]&BIT(5)) // 低气压报警
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(5); // 清0
    
    if (pdata[6]&BIT(7)) // PTO检测
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(0); // 清0

    zxdown_buffer_a5e8[ZXDOWN_A5E8_POS1] = pdata[2]; // 取力挂接(字节状态位)
    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5e8_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25E:
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3] = pdata[0];  // 变速箱油温
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS3+1] = pdata[1];
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS4] = pdata[3];  // 变速箱进气气压
    zxdown_buffer_a5e1[ZXDOWN_A5E1_POS5] = pdata[4];  // 离合器位置 
    
    if (pdata[5]&BIT(0)) // 发动机制动
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] |= BIT(7); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS8] &= ~BIT(7); // 清0
    
    if (pdata[5]&BIT(1)) // 电涡流制动1挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(0); // 清0
    
    if (pdata[5]&BIT(2)) // 电涡流制动2挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(3)) // 电涡流制动3挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(2); // 清0
    
    if (pdata[5]&BIT(4)) // 电涡流制动4挡
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e6[ZXDOWN_A5E6_POS9] &= ~BIT(3); // 清0
    
    if (pdata[5]&BIT(5)) // 发动机熄火输入
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(6)) // 发动机熄火输出
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1] &= ~BIT(2); // 清0
    
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5] = pdata[0];  // 缓速器油温
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS5+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS6] = pdata[2];  // 缓速力矩百分比
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS7] = pdata[6]; // 马达控制电流
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS7+1] = pdata[7];
    tlv_a5e1_valid_flag = 1;
    tlv_a5e6_valid_flag = 1;
    tlv_a5e7_valid_flag = 1;
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E2====================================================================================
  case 0x24F:
    zxdown_buffer_a5e2[ZXDOWN_A5E2_POS1] = pdata[7]; // 支腿垂直状态位
    zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] = pdata[5]; // 支腿水平状态位
    
    if (pdata[6]&BIT(2)) // 右前摆臂伸
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(0); // 清0
    
    if (pdata[6]&BIT(3)) // 右前摆臂缩
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(1); // 清0
    
    if (pdata[6]&BIT(0)) // 左前摆臂伸
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS3] &= ~BIT(2); // 清0
    
    if (pdata[6]&BIT(1)) // 左前摆臂缩
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(3); // 清0
    
    if (pdata[6]&BIT(6)) // 右后摆臂伸
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(4); // 清0
    
    if (pdata[6]&BIT(7)) // 右后摆臂缩
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(5); // 清0
    
    if (pdata[6]&BIT(4)) // 左后摆臂伸
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(6); // 清0
    
    if (pdata[6]&BIT(5)) // 左后摆臂缩
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] |= BIT(7); // 置1
    else
      zxdown_buffer_a5e2[ZXDOWN_A5E2_POS2] &= ~BIT(7); // 清0
    
    tlv_a5e2_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E3====================================================================================
  case 0x22B:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS1], pdata, 8); // 悬挂压力
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x22C:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS2], pdata, 8); // 悬挂行程
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21E:
    memcpy(&zxdown_buffer_a5e3[ZXDOWN_A5E3_POS3], pdata, 4); // 整车重量
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25F:
    if (pdata[1]&BIT(0)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(1); // 清0
    
    if (pdata[0]&BIT(0)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(2); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(1)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(3); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(3); // 清0
    
    if (pdata[0]&BIT(2)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(4); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(4); // 清0
    
    if (pdata[0]&BIT(3)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(5); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(5); // 清0
    
    if (pdata[0]&BIT(4)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(6); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(6); // 清0
    
    if (pdata[0]&BIT(5)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] |= BIT(7); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS4] &= ~BIT(7); // 清0
    
    if (pdata[0]&BIT(6)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] |= BIT(0); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(7)) // 右前摆臂伸
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] |= BIT(1); // 置1
    else
      zxdown_buffer_a5e3[ZXDOWN_A5E3_POS5] &= ~BIT(1); // 清0
    
    tlv_a5e3_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5E4====================================================================================
  case 0x22D:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS1], pdata, 8); // 一二轴转向角度
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22E:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2] = pdata[0];  // 电控三轴转向角度
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+1] = pdata[1];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+2] = pdata[2]; // 电控四轴转向角度
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+3] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25A:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+4] = pdata[0]; // 电控五轴转向角度
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+5] = pdata[1];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+6] = pdata[2]; // 电控六轴转向角度
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS2+7] = pdata[3];
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BB:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS3], pdata, 8); // 一二轴传感器电流
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;    

  case 0x2BC:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS4], pdata, 8); // 三四五六轴传感器电流
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x25C:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS5] = pdata[1];  // 当前转向模式
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS6] = pdata[0];  // 目标转向模式
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS7] = pdata[2];  // 转向系统压力
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS7+1] = pdata[3];
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS8] = pdata[4];  // 比例调压阀电流
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS8+1] = pdata[5];
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2] = pdata[6]; // 液压系统压力
    zxdown_buffer_a5e9[ZXDOWN_A5E9_POS2+1] = pdata[7];
    tlv_a5e4_valid_flag = 1;
    tlv_a5e9_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2BD:
    zxdown_buffer_a5e4[ZXDOWN_A5E4_POS11] = pdata[0];  // 桥锁止阀
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS9], &pdata[2], 6); // 123桥左右转阀占空比
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2BE:
    memcpy(&zxdown_buffer_a5e4[ZXDOWN_A5E4_POS10], pdata, 6); // 456桥左右转阀占空比
    tlv_a5e4_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5E6====================================================================================
  case 0x25D:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1] = pdata[0];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS1+1] = pdata[1];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2] = pdata[2];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS2+1] = pdata[3];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3] = pdata[4];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS3+1] = pdata[5];
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4] = pdata[6];  // 回路一气压
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS4+1] = pdata[7];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x23A:
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7] = pdata[4];  // 变矩器油温
    zxdown_buffer_a5e6[ZXDOWN_A5E6_POS7+1] = pdata[5];
    tlv_a5e6_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5E7====================================================================================
  // 分散
  //==A5E8====================================================================================
  // 分散
  //==A5E9====================================================================================
  // 分散
    
  //==A5EA====================================================================================
  case 0x26A:
    if (pdata[0]&BIT(0)) // 上车液压驱动确认信息
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS1] |= BIT(0); // 置1
    else
      zxdown_buffer_a5ea[ZXDOWN_A5EA_POS1] &= ~BIT(0); // 清0
    
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS3] = pdata[1]; // 上车发动机水温
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS4] = pdata[2]; // 上车发动机机油压力
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS4+1] = pdata[3];
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS5] = pdata[4]; // 上车发动机转速
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS5+1] = pdata[5];
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS6] = pdata[6]; // 上车泵出口压力
    zxdown_buffer_a5ea[ZXDOWN_A5EA_POS6+1] = pdata[7];
    tlv_a5ea_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5EB====================================================================================
  case 0x28C:
    if(pdata[0]>0 && pdata[0]<19) // 1号轮胎到18号轮胎
    {
      zxdown_buffer_a5eb[ZXDOWN_A5EB_POS1+pdata[0]-1] = pdata[1];
      tlv_a5eb_valid_flag = 1;
    }
    retval = 0x01;
    break;

  //==A5EC====================================================================================
  case 0x2AB:
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS1] = pdata[0]; // 左支腿面板 - 字节状态位1
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS2] = pdata[1]; // 左支腿面板 - 字节状态位2
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS3] = pdata[2]; // 左支腿面板 - 字节状态位3
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS4] = pdata[3]; // 左支腿面板 - 字节状态位4
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS5] = pdata[4]; // 左支腿面板 - 字节状态位5
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS6] = pdata[5]; // 左支腿面板 - 预留
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS7] = pdata[6]; // 左支腿面板 - DIR
    zxdown_buffer_a5ec[ZXDOWN_A5EC_POS8] = pdata[7]; // 左支腿面板 - 调平盒心跳
    tlv_a5ec_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5ED====================================================================================
  case 0x2AC:
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS1] = pdata[0]; // 右支腿面板 - 字节状态位1
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS2] = pdata[1]; // 右支腿面板 - 字节状态位2
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS3] = pdata[2]; // 右支腿面板 - 字节状态位3
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS4] = pdata[3]; // 右支腿面板 - 字节状态位4
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS5] = pdata[4]; // 右支腿面板 - 字节状态位5
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS6] = pdata[5]; // 右支腿面板 - 预留
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS7] = pdata[6]; // 右支腿面板 - DIR
    zxdown_buffer_a5ed[ZXDOWN_A5ED_POS8] = pdata[7]; // 右支腿面板 - 调平盒心跳
    tlv_a5ed_valid_flag = 1;
    retval = 0x01;
    break;
    
  //==A5EE====================================================================================
  case 0x2AD:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS1] = pdata[0]; // 中控台输入信息 - 字节状态位1
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS2] = pdata[1]; // 中控台输入信息 - 字节状态位2
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x2AE:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS3] = pdata[0]; // 中控台输入信息 - 字节状态位3
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS4] = pdata[1]; // 中控台输入信息 - 字节状态位4
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x2AF:
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS5] = pdata[0]; // 中控台输入信息 - 字节状态位5
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS6] = pdata[1]; // 中控台输入信息 - 字节状态位6
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS7] = pdata[2]; // 中控台输入信息 - 字节状态位7
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS8] = pdata[3]; // 后桥转向角度
    zxdown_buffer_a5ee[ZXDOWN_A5EE_POS9] = pdata[4]; // 字节状态位8
    tlv_a5ee_valid_flag = 1;
    retval = 0x01;
    break;

  //==下车系统版本====================================================================================
  case 0x20F:
    zxversion_buffer_a506[ZXVERSION_A506_POS3] = pdata[0];  // P1应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS3+1] = pdata[1];
    zxversion_buffer_a506[ZXVERSION_A506_POS3+2] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS5] = pdata[3];  // P2应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS5+1] = pdata[4];
    zxversion_buffer_a506[ZXVERSION_A506_POS5+2] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS7] = pdata[6];  // P3应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS7+1] = pdata[7];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x21F:
    zxversion_buffer_a506[ZXVERSION_A506_POS7+2] = pdata[0];  // P3应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS9] = pdata[1];    // P4应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS9+1] = pdata[2];
    zxversion_buffer_a506[ZXVERSION_A506_POS9+2] = pdata[3];
    zxversion_buffer_a506[ZXVERSION_A506_POS11] = pdata[4];   // P5应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS11+1] = pdata[5];
    zxversion_buffer_a506[ZXVERSION_A506_POS11+2] = pdata[6];
    zxversion_buffer_a506[ZXVERSION_A506_POS17] = pdata[7];   // P8应用层
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x22F:
    zxversion_buffer_a506[ZXVERSION_A506_POS17+1] = pdata[0];   // P8应用层
    zxversion_buffer_a506[ZXVERSION_A506_POS17+2] = pdata[1];
    tlv_a506_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x29F:
    if(pdata[0]==1)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS4] = pdata[1];  // P1底层
      zxversion_buffer_a506[ZXVERSION_A506_POS4+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS4+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==2)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS6] = pdata[1];  // P2底层
      zxversion_buffer_a506[ZXVERSION_A506_POS6+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS6+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==3)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS8] = pdata[1];  // P3底层
      zxversion_buffer_a506[ZXVERSION_A506_POS8+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS8+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==4)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS10] = pdata[1];  // P4底层
      zxversion_buffer_a506[ZXVERSION_A506_POS10+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS10+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==5)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS12] = pdata[1];  // P5底层
      zxversion_buffer_a506[ZXVERSION_A506_POS12+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS12+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==6)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS14] = pdata[1];  // P6底层
      zxversion_buffer_a506[ZXVERSION_A506_POS14+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS14+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==7)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS16] = pdata[1];  // P7底层
      zxversion_buffer_a506[ZXVERSION_A506_POS16+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS16+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    else if(pdata[0]==8)
    {
      zxversion_buffer_a506[ZXVERSION_A506_POS18] = pdata[1];  // P8底层
      zxversion_buffer_a506[ZXVERSION_A506_POS18+1] = pdata[2];
      zxversion_buffer_a506[ZXVERSION_A506_POS18+2] = pdata[3];
      tlv_a506_valid_flag = 1;
    }
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * 处理接收到的CAN报文(汽车底盘发动机)
*************************************************************************/
uint8_t CAN_ProcessRecvEngineMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // 上车通信
  {
  //==A5EF===================================================================================
  case 0x0CF00300:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS11] = pdata[1]; // 油门踏板百分比
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x0CF00400:
    if (pdata[0]&BIT(0)) // 发动机扭矩模式1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(1)) // 发动机扭矩模式2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(1); // 清0
    
    if (pdata[0]&BIT(2)) // 发动机扭矩模式3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(2); // 清0
    
    if (pdata[0]&BIT(3)) // 发动机扭矩模式4
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(3); // 清0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1] = pdata[3]; // 发动机转速
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1+1] = pdata[4];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS2] = pdata[2]; // 实际扭矩百分比
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEE00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS6] = pdata[0]; // 冷却液温度
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7] = pdata[2]; // 机油温度
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEEF00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS8] = pdata[2]; // 机油液位
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS9] = pdata[3]; // 机油压力
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
 
  case 0x18FEF600:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS4] = pdata[1]; // 进气歧管压力
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS5] = pdata[2]; // 进气歧管温度
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
   
  case 0x18FEE500:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10] = pdata[0]; // 发动机总运行时间
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEDF00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS3] = pdata[0]; // 摩擦扭矩百分比
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18FEF100:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS13] = pdata[3];
    
    if (pdata[4]&BIT(0)) // 巡航设置开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(0); // 清0
    
    if (pdata[4]&BIT(1)) // 巡航设置开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(1); // 清0
    
    if (pdata[4]&BIT(2)) // 巡航减速开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(2); // 清0
    
    if (pdata[4]&BIT(3)) // 巡航减速开关2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(3); // 清0
    
    if (pdata[4]&BIT(6)) // 巡航控制加速开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(4); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(4); // 清0
    
    if (pdata[4]&BIT(7)) // 巡航控制加速开关2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(5); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(5); // 清0
    
    if (pdata[6]&BIT(5)) // 巡航控制状态1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(4); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(4); // 清0

    if (pdata[6]&BIT(6)) // 巡航控制状态2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(5); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(5); // 清0

    if (pdata[6]&BIT(7)) // 巡航控制状态3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(6); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(6); // 清0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12] = pdata[1]; // 车速
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12+1] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS15] = pdata[5]; // 巡航设定速度
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEF200:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16] = pdata[0]; // 发动机燃油消耗率
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17] = pdata[4]; // 发动机平均燃油消耗率
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17+1] = pdata[5];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE900:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18] = pdata[4]; // 燃油总油耗量
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEE000:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19] = pdata[4]; // 总行驶里程
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFC17:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS20] = pdata[1]; // 燃油液位1
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS21] = pdata[6]; // 燃油液位2
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FD0700:
    if (pdata[0]&BIT(2)) // 驾驶员报警灯DWL-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(3)) // 驾驶员报警灯DWL-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(1); // 清0
    
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x18FEFF00:
    if (pdata[0]&BIT(0)) // 油中有水指示灯-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(1)) // 油中有水指示灯-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(3); // 清0

    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x18F00A00:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24] = pdata[2];  // 进气流量
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F0===================================================================================
  //GPS计算，根据发动机PTO区分上下车工作

  //==A502===================================================================================
  case 0x18FEF500:
    zxengine_buffer_a502[ZXENGINE_A502_POS1_ADDR] = pdata[0];// 大气压力
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR] = pdata[3];// 环境温度
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR+1] = pdata[4];
    tlv_a502_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5F1===================================================================================
  case 0x18FF203D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS1] = pdata[0]; // 尿素喷射状态
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS2] = pdata[5]; // T15_DCU
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3] = pdata[6]; // 尿素泵压力
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE563D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS4] = pdata[0]; // 尿素箱液位
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS5] = pdata[1]; // 尿素箱温度
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x0CF0233D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6] = pdata[6]; // 尿素喷射量
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6+1] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00E51:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7] = pdata[0]; // SCR上游NOx浓度
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18F00F52:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8] = pdata[0]; // SCR下游NOx浓度
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD3E3D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9] = pdata[0]; // SCR上游排气温度(T6温度)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9+1] = pdata[1];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10] = pdata[3]; // SCR下游排气温度(T7温度)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10+1] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD9BA3:
    if (pdata[3]&BIT(0)) // 品质温度传感器FMI (SPN 3519)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(0); // 清0

    if (pdata[3]&BIT(1)) // 品质温度传感器FMI (SPN 3519)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(1); // 清0

    if (pdata[3]&BIT(2)) // 品质温度传感器FMI (SPN 3519)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(2); // 清0

    if (pdata[3]&BIT(3)) // 品质温度传感器FMI (SPN 3519)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(3); // 清0

    if (pdata[3]&BIT(4)) // 品质温度传感器FMI (SPN 3519)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(4); // 清0

    if (pdata[4]&BIT(0)) // 品质传感器FMI (SPN3520)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(0); // 清0

    if (pdata[4]&BIT(1)) // 品质传感器FMI (SPN3520)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(1); // 清0

    if (pdata[4]&BIT(2)) // 品质传感器FMI (SPN3520)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(2); // 清0

    if (pdata[4]&BIT(3)) // 品质传感器FMI (SPN3520)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(3); // 清0

    if (pdata[4]&BIT(4)) // 品质传感器FMI (SPN3520)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(4); // 清0

    if (pdata[5]&BIT(0)) // 催化剂试剂类型(SPN3521)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 催化剂试剂类型(SPN3521)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(2)) // 催化剂试剂类型(SPN3521)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(3)) // 催化剂试剂类型(SPN3521)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(3); // 清0

    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS13] = pdata[1]; // 尿素品质传感器温度(SPN 3515)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS11] = pdata[1]; // 尿素浓度(SPN 3516)
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FCBD3D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12] = pdata[4]; // 累计尿素消耗量
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+1] = pdata[5];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+2] = pdata[6];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+3] = pdata[7];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FE56A3:
    if (pdata[4]&BIT(0)) // 尿素箱液位传感器失效模式FMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(0); // 清0

    if (pdata[4]&BIT(1)) // 尿素箱液位传感器失效模式FMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(1); // 清0

    if (pdata[4]&BIT(2)) // 尿素箱液位传感器失效模式FMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(2); // 清0

    if (pdata[4]&BIT(3)) // 尿素箱液位传感器失效模式FMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(3); // 清0

    if (pdata[4]&BIT(4)) // 尿素箱液位传感器失效模式FMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(4); // 清0

    if (pdata[5]&BIT(0)) // 尿素箱温度传感器失效模式FMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 尿素箱温度传感器失效模式FMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(2)) // 尿素箱温度传感器失效模式FMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(3)) // 尿素箱温度传感器失效模式FMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(3); // 清0

    if (pdata[5]&BIT(4)) // 尿素箱温度传感器失效模式FMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(4); // 清0
    
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FEDF3D:
    if (pdata[7]&BIT(0)) // Nox传感器露点状态1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(0); // 清0

    if (pdata[7]&BIT(1)) // Nox传感器露点状态2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(1); // 清0

    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F2===================================================================================
  case 0x18FD2000:
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1] = pdata[0]; // DOC上游排气温度
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1+1] = pdata[1];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2] = pdata[2]; // DPF上游排气温度
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2+1] = pdata[3];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FF1400:
    if (pdata[1]&BIT(0)) // DPF再生激活状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(0); // 清0

    if (pdata[1]&BIT(1)) // DPF再生激活状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(1); // 清0

    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS3] = pdata[0]; // DPF碳载量负荷率
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FDB200:
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4] = pdata[0]; // DPF压差
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4+1] = pdata[1];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18FD7C00:
    if (pdata[0]&BIT(0)) // DPF再生指示灯状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // DPF再生指示灯状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(2)) // DPF再生指示灯状态3
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(2); // 清0

    if (pdata[2]&BIT(0)) // DPF再生禁止状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(0); // 清0

    if (pdata[2]&BIT(1)) // DPF再生禁止状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(1); // 清0
      
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x18E00021:
    if (pdata[5]&BIT(0)) // DPF再生开关状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // DPF再生开关状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(2)) // DPF再生禁止开关状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(3)) // DPF再生禁止开关状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(3); // 清0

    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

/*************************************************************************
 * 处理接收到的CAN报文(全地面底盘发动机)
*************************************************************************/
uint8_t CAN_ProcessRecvEngineMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size)
{
  uint8_t retval = 0x00;
  //uint32_t tempVal;

  switch (canId) // 下车通信
  {
  //==A5EF发动机运行参数(国四、国五、国六)===================================================
  case 0x20B:
    if (pdata[0]&BIT(0)) // 发动机扭矩模式1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(1)) // 发动机扭矩模式2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(1); // 清0
    
    if (pdata[0]&BIT(2)) // 发动机扭矩模式3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(2); // 清0
    
    if (pdata[0]&BIT(3)) // 发动机扭矩模式4
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22] &= ~BIT(3); // 清0

    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1] = pdata[3]; // 发动机转速
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1+1] = pdata[4];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS2] = pdata[2]; // 实际扭矩百分比
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x40D:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS3] = pdata[7]; // 摩擦扭矩百分比
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16] = pdata[0]; // 发动机燃油消耗率
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16+1] = pdata[1];
    zxengine_buffer_a502[ZXENGINE_A502_POS1_ADDR] = pdata[4];// 大气压力
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR] = pdata[5];// 环境温度
    zxengine_buffer_a502[ZXENGINE_A502_POS2_ADDR+1] = pdata[6];
    tlv_a502_valid_flag = 1;
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20E:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS4] = pdata[1]; // 进气歧管压力
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS5] = pdata[2]; // 进气歧管温度
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS6] = pdata[0]; // 冷却液温度
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7] = pdata[2]; // 机油温度
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS7+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x20D:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS8] = pdata[2]; // 机油液位
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS9] = pdata[3]; // 机油压力
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x40B:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10] = pdata[0]; // 发动机总运行时间
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;   
    
  case 0x20A:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS11] = pdata[1]; // 油门踏板百分比
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break; 
    
  case 0x40C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS13] = pdata[7]; // 巡航/刹车/离合状态
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12] = pdata[4]; // 车速
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18] = pdata[0]; // 燃油总油耗量
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+1] = pdata[1];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+2] = pdata[2];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+3] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
 
  case 0x41A:
    if (pdata[4]&BIT(0)) // 巡航设置开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(0); // 清0
    
    if (pdata[4]&BIT(1)) // 巡航设置开关2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(1); // 清0
    
    if (pdata[4]&BIT(2)) // 巡航减速开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(2); // 清0
    
    if (pdata[4]&BIT(3)) // 巡航减速开关2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(3); // 清0
    
    if (pdata[4]&BIT(6)) // 巡航控制加速开关1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(4); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(4); // 清0
    
    if (pdata[4]&BIT(7)) // 巡航控制加速开关2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] |= BIT(5); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS14] &= ~BIT(5); // 清0
    
    if (pdata[6]&BIT(5)) // 巡航控制状态1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(4); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(4); // 清0

    if (pdata[6]&BIT(6)) // 巡航控制状态2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(5); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(5); // 清0

    if (pdata[6]&BIT(7)) // 巡航控制状态3
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(6); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(6); // 清0
    
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS15] = pdata[5]; // 巡航设定速度 
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS20] = pdata[0]; // 燃油液位1
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS21] = pdata[1]; // 燃油液位2
    
    //===========================================================================
    if (pdata[2]&BIT(0)) // 尿素箱液位传感器失效模式FMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(0); // 清0

    if (pdata[2]&BIT(1)) // 尿素箱液位传感器失效模式FMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(1); // 清0

    if (pdata[2]&BIT(2)) // 尿素箱液位传感器失效模式FMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(2); // 清0

    if (pdata[2]&BIT(3)) // 尿素箱液位传感器失效模式FMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(3); // 清0

    if (pdata[2]&BIT(4)) // 尿素箱液位传感器失效模式FMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS17] &= ~BIT(4); // 清0

    if (pdata[3]&BIT(0)) // 尿素箱温度传感器失效模式FMI-1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(0); // 清0

    if (pdata[3]&BIT(1)) // 尿素箱温度传感器失效模式FMI-2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(1); // 清0

    if (pdata[3]&BIT(2)) // 尿素箱温度传感器失效模式FMI-3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(2); // 清0

    if (pdata[3]&BIT(3)) // 尿素箱温度传感器失效模式FMI-4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(3); // 清0

    if (pdata[3]&BIT(4)) // 尿素箱温度传感器失效模式FMI-5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS18] &= ~BIT(4); // 清0
    
    tlv_a5ef_valid_flag = 1;
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x42C:
    if (pdata[0]&BIT(0)) // 油中有水指示灯-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(2); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(2); // 清0

    if (pdata[0]&BIT(1)) // 油中有水指示灯-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(3); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(3); // 清0
    
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17] = pdata[4]; // 发动机平均燃油消耗率
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS17+1] = pdata[5];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
    
  case 0x42B:
    if (pdata[0]&BIT(2)) // 驾驶员报警灯DWL-1
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(0); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(0); // 清0
    
    if (pdata[0]&BIT(3)) // 驾驶员报警灯DWL-2
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] |= BIT(1); // 置1
    else
      zxengine_buffer_a5ef[ZXENGINE_A5EF_POS23] &= ~BIT(1); // 清0
    
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;

  case 0x46C:
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24] = pdata[2];  // 进气流量
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24+1] = pdata[3];
    tlv_a5ef_valid_flag = 1;
    retval = 0x01;
    break;
  
  //==A5F0===================================================================================
  //GPS计算，根据发动机PTO区分上下车工作
  
  //==A5F1(SCR参数)==========================================================================
  case 0x41B:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS1] = pdata[0]; // 尿素喷射状态
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS2] = pdata[5]; // T15_DCU
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3] = pdata[6]; // 尿素泵压力
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS3+1] = pdata[7];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12] = pdata[1]; // 累计尿素消耗量
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+1] = pdata[2];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+2] = pdata[3];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+3] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x40A:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS4] = pdata[0]; // 尿素箱液位
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS5] = pdata[1]; // 尿素箱温度
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6] = pdata[2]; // 尿素喷射量
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6+1] = pdata[3];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19] = pdata[4]; // 总行驶里程
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+1] = pdata[5];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+2] = pdata[6];
    zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+3] = pdata[7];
    tlv_a5ef_valid_flag = 1;
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46A:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7] = pdata[0]; // SCR上游NOx浓度
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46B:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8] = pdata[0]; // SCR下游NOx浓度
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8+1] = pdata[1];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x46D:
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9] = pdata[0]; // SCR上游排气温度(T6温度)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9+1] = pdata[1];
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10] = pdata[3]; // SCR下游排气温度(T7温度)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10+1] = pdata[4];
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x41C:
    if (pdata[3]&BIT(0)) // 品质温度传感器FMI (SPN 3519)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(0); // 清0

    if (pdata[3]&BIT(1)) // 品质温度传感器FMI (SPN 3519)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(1); // 清0

    if (pdata[3]&BIT(2)) // 品质温度传感器FMI (SPN 3519)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(2); // 清0

    if (pdata[3]&BIT(3)) // 品质温度传感器FMI (SPN 3519)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(3); // 清0

    if (pdata[3]&BIT(4)) // 品质温度传感器FMI (SPN 3519)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS14] &= ~BIT(4); // 清0

    if (pdata[4]&BIT(0)) // 品质传感器FMI (SPN3520)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(0); // 清0

    if (pdata[4]&BIT(1)) // 品质传感器FMI (SPN3520)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(1); // 清0

    if (pdata[4]&BIT(2)) // 品质传感器FMI (SPN3520)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(2); // 清0

    if (pdata[4]&BIT(3)) // 品质传感器FMI (SPN3520)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(3); // 清0

    if (pdata[4]&BIT(4)) // 品质传感器FMI (SPN3520)5
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] |= BIT(4); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS15] &= ~BIT(4); // 清0

    if (pdata[5]&BIT(0)) // 催化剂试剂类型(SPN3521)1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // 催化剂试剂类型(SPN3521)2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(1); // 清0

    if (pdata[5]&BIT(2)) // 催化剂试剂类型(SPN3521)3
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(3)) // 催化剂试剂类型(SPN3521)4
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS16] &= ~BIT(3); // 清0

    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS13] = pdata[1]; // 尿素品质传感器温度(SPN 3515)
    zxengine_buffer_a5f1[ZXENGINE_A5F1_POS11] = pdata[1]; // 尿素浓度(SPN 3516)
    tlv_a5f1_valid_flag = 1;
    retval = 0x01;
    break;

  //==A5F2===================================================================================
  case 0x41D:
    if (pdata[5]&BIT(0)) // DPF再生激活状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // DPF再生激活状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS5] &= ~BIT(1); // 清0
    
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1] = pdata[0]; // DOC上游排气温度
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS1+1] = pdata[1];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2] = pdata[2]; // DPF上游排气温度
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2+1] = pdata[3];
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS3] = pdata[4]; // DPF碳载量负荷率
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4] = pdata[6]; // DPF压差
    zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4+1] = pdata[7];
    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  case 0x42A:
    if (pdata[7]&BIT(0)) // Nox传感器露点状态1
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(0); // 清0

    if (pdata[7]&BIT(1)) // Nox传感器露点状态2
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f1[ZXENGINE_A5F1_POS19] &= ~BIT(1); // 清0

    tlv_a5f1_valid_flag = 1;
  
    if (pdata[0]&BIT(0)) // DPF再生指示灯状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(0); // 清0

    if (pdata[0]&BIT(1)) // DPF再生指示灯状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(1); // 清0

    if (pdata[0]&BIT(2)) // DPF再生指示灯状态3
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS6] &= ~BIT(2); // 清0
 
    if (pdata[2]&BIT(0)) // DPF再生禁止状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(0); // 清0

    if (pdata[2]&BIT(1)) // DPF再生禁止状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS7] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(0)) // DPF再生禁止开关状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(0); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(0); // 清0

    if (pdata[5]&BIT(1)) // DPF再生禁止开关状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(1); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(1); // 清0
    
    if (pdata[5]&BIT(2)) // DPF再生开关状态1
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(2); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(2); // 清0

    if (pdata[5]&BIT(3)) // DPF再生开关状态2
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] |= BIT(3); // 置1
    else
      zxengine_buffer_a5f2[ZXENGINE_A5F2_POS8] &= ~BIT(3); // 清0

    tlv_a5f2_valid_flag = 1;
    retval = 0x01;
    break;
  
  default:
    retval = 0x00;
    break;
  }
  
  return retval;
}

#if (PART("作业统计"))
/*************************************************************************
 * 作业统计实现机制
*************************************************************************/
//==设置计数标志==============================================================
void ZxSts_SetNumberFlag(zxsts_context_t* pThis)
{
  pThis->number_flag = 0x01; // 需要计数
}

//==设置开始计时标志==========================================================
void ZxSts_SetStartFlag(zxsts_context_t* pThis)
{
  pThis->work_flag = 0x01; // 开始
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==设置停止计时标志==========================================================
void ZxSts_SetStopFlag(zxsts_context_t* pThis)
{
  pThis->work_flag = 0x00; // 停止
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==重置CAN帧消失去耦时间=====================================================
void ZxSts_ResetCanMsgTimer(zxsts_context_t* pThis)
{
  pThis->debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
}

//==累计工作时间=============================================================
void ZxSts_Accumulate(zxsts_context_t* pThis)
{
  //==时间累计===================================
  if(pThis->work_flag)  // 设备工作中
  {
    pThis->timer_100ms++;  // 时间累计
    if(pThis->timer_100ms >= ZXSTS_STEP_SP)
    {
      pThis->timer_100ms -= ZXSTS_STEP_SP;
      pThis->total_work_time++; // 工作时间累加0.05h(3分钟)
    }
  }
  //==频次累计===================================
  if(pThis->number_flag)  // 次数累计
  {
    pThis->number_flag = ZXSTS_FALSE;
    pThis->total_work_number++;  // 频次加1
  }
}

//==作业统计状态机============================================================
void ZxSts_AccumulateAll(void)
{
  uint8_t it;

  for(it=0x00; it<NUMBER_OF_ZXSTS_TYPES; it++)
  {
    ZxSts_Accumulate(&zxsts_context[it]);  // 作业统计
  }
}

//==状态判断==================================================================
void ZxSts_ServiceInput(zxsts_context_t* pThis, uint8_t current_state)
{
  pThis->current_state = current_state;  // 读取当前状态

  if(pThis->debounce_timer)  // CAN帧消失去耦时间
    pThis->debounce_timer--;
  else
  {
    pThis->current_state = ZXSTS_FALSE;
  }

  if(pThis->current_state) // 开始计时  
  {  pThis->work_flag = ZXSTS_TRUE;}
  else // 停止计时
  {  pThis->work_flag = ZXSTS_FALSE;}
  
  if((pThis->previous_state == ZXSTS_FALSE)&&(pThis->current_state == ZXSTS_TRUE)) // 记一次数
  {
    pThis->number_flag = ZXSTS_TRUE;
  }
  
  pThis->previous_state = pThis->current_state;
}

//==作业统计状态机(100ms周期)=======================================================
void ZxSts_StateMachine(void)
{
  uint8_t current_state;

  //==空缸伸===========================================
  if((zxsts_empty_cylinder_flag==ZXSTS_TRUE) && (zxsts_cylinder_extend_flag==ZXSTS_TRUE)) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_KGEX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_KGEX]);
    
  //==空缸缩===========================================
  if((zxsts_empty_cylinder_flag==ZXSTS_TRUE) && (zxsts_cylinder_shrink_flag==ZXSTS_TRUE)) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_KGS], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_KGS]);

  //==带臂伸===========================================
  if((zxsts_arm_work_flag==ZXSTS_TRUE) && (zxsts_cylinder_extend_flag==ZXSTS_TRUE)) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_DBEX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_DBEX]);

  //==带臂缩===========================================
  if((zxsts_arm_work_flag==ZXSTS_TRUE) && (zxsts_cylinder_shrink_flag==ZXSTS_TRUE)) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_DBS], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_DBS]);

  //==变幅起===========================================
  if(zxsts_luff_up_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFUP]);

  //==变幅落===========================================
  if(zxsts_luff_down_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFDW]);

  //==主卷起升=========================================
  if(zxsts_main_hoist_up_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJUP]);

  //==主卷下落=========================================
  if(zxsts_main_hoist_down_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJDW]);

  //==副卷起升=========================================
  if(zxsts_deputy_hoist_up_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJUP], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJUP]);

  //==副卷下落=========================================
  if(zxsts_deputy_hoist_down_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJDW], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJDW]);
  
  //==左回转===========================================
  if(zxsts_slew_left_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZHR], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZHR]);
  
  //==右回转===========================================
  if(zxsts_slew_right_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_YHR], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_YHR]);

  ///////////////////////////////////////////////////////////////////////////
  // 安全统计
  ///////////////////////////////////////////////////////////////////////////
  //==超载==============================================
  if(zxsts_overload_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_LMI], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_LMI]);

  //==主卷三圈==========================================
  if(zxsts_od_main_hoist_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZJSQ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZJSQ]);

  //==副卷三圈(备用)====================================
  if(zxsts_od_deputy_hoist_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FJSQ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FJSQ]);

  //==主臂高限==========================================
  if(zxsts_a2b_main_arm_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZBGX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZBGX]);

  //==副臂高限(备用)====================================
  if(zxsts_a2b_deputy_arm_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FBGX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FBGX]);

  //==总强制============================================
  if(zxsts_lmi_force_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_ZQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_ZQZ]);

  //==拆装开关==========================================
  if(zxsts_setup_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_CZKG], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_CZKG]);
  
  //==变幅起强制========================================
  if(zxsts_luff_up_force_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_BFQQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_BFQQZ]);

  //==高限强制==========================================
  if(zxsts_a2b_force_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_GXQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_GXQZ]);

  //==三圈强制==========================================
  if(zxsts_od_force_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_SQQZ], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_SQQZ]);

  //==风速超限==========================================
  if(zxsts_lmi_wind_speed_flag==ZXSTS_TRUE) // 读取当前状态
  {  current_state = ZXSTS_TRUE;}
  else
  {  current_state = ZXSTS_FALSE;}
  ZxSts_ServiceInput(&zxsts_context[ZXSTS_TYPE_FSCX], current_state);
  ZxSts_Accumulate(&zxsts_context[ZXSTS_TYPE_FSCX]);
}

//==初始化作业统计参数=======================================================
void ZxSts_Initialize(void)
{
  uint8_t it;

  for(it=0x00; it<NUMBER_OF_ZXSTS_TYPES; it++)
  {
    zxsts_context[it].previous_state = 0x00;
    zxsts_context[it].current_state = 0x00;
    zxsts_context[it].number_flag = 0x00;
    zxsts_context[it].work_flag = 0x00;
    zxsts_context[it].debounce_timer = ZXSTS_DEBOUNCE_TIME_SP;
  }
  zxsts_flag1.word = 0x00;
  zxsts_flag2.word = 0x00;
}

#endif

#if (PART("发动机工作统计"))
/*************************************************************************
 * 发动机工作统计实现机制(ACC开且发动机转速大于300)
*************************************************************************/
//==发动机工作统计状态机(1s周期)==========================================
void ZxStsEngine_StateMachine(zxsts_engine_context_t* pThis)
{
  uint8_t acc_state;
  bittype temp_flag;
  static uint8_t divide_for_1min = 59;
  static uint8_t divide_for_5min = 4;
  static uint8_t DO_1MIN_FLAG = ZXSTS_FALSE;
  static uint8_t DO_5MIN_FLAG = ZXSTS_FALSE;

  //==状态获取==========
  acc_state = COLT_GetAccStatus();
  temp_flag.byte = zxdown_buffer_a5e7[ZXDOWN_A5E7_POS1]; // 动力系统: bit0-PTO检测
  if(temp_flag.b.bit0)
  {  pThis->pto_status = 0x01;}  // 上车工作
  else
  {  pThis->pto_status = 0x02;}  // 下车工作

  temp_flag.byte = zxinfo_buffer_a504[ZXINFO_A504_POS2_ADDR]; // 0x574.byte1.bit3
  if(temp_flag.b.bit3)
  {  pThis->engine_type = 0x02;}  // 双发动机
  else
  {  pThis->engine_type = 0x01;}  // 单发动机

  //==确认发动机工作状态=======
  if (acc_state==1) // ACC开和发动机发动
  {
    if (pThis->engine_type==0x01) // 单发=0x01
    {
      if (pThis->pto_status==0x01) // 0x01=作业
      {
        pThis->up_work_flag = 0x01;
        pThis->down_work_flag = 0x00;
      }
      else if (pThis->pto_status==0x02) // 0x02=行驶
      {
        pThis->up_work_flag = 0x00;
        pThis->down_work_flag = 0x01;
      }
      else
      {
        pThis->up_work_flag = 0x00;
        pThis->down_work_flag = 0x00;
      }
    }
    else if (pThis->engine_type==0x02)// 双发=0x02
    {
      pThis->up_work_flag = 0x01;
      pThis->down_work_flag = 0x01;
    }
    else
    {
      return;
    }
    
    //==时间分时器==========
    if (!divide_for_1min)
    {
      divide_for_1min = 59;  // 基于1s
      DO_1MIN_FLAG = ZXSTS_TRUE;

      if (!divide_for_5min)
      {
        divide_for_5min = 4;
        DO_5MIN_FLAG = ZXSTS_TRUE;
      }
      else divide_for_5min--;
    }
    else divide_for_1min--;
  }
  else
  {
    divide_for_1min = 59;
    divide_for_5min = 4;
    DO_1MIN_FLAG = ZXSTS_FALSE;
    DO_5MIN_FLAG = ZXSTS_FALSE;
  }

  //==一分钟任务================================
  if (DO_1MIN_FLAG==ZXSTS_TRUE)
  {
    DO_1MIN_FLAG = ZXSTS_FALSE;

    //==上车工作==============================
    if (pThis->up_work_flag)
    {
      //==工时统计=====
      pThis->current_twt_up = CAN_GetUpEngineTwt(); // 上车总工作时间
      if (pThis->current_twt_up > pThis->previous_twt_up)
      {
        pThis->tdv_up = pThis->current_twt_up - pThis->previous_twt_up;
      }
      else
      {
        pThis->tdv_up = 0;
      }
      pThis->twt_up += pThis->tdv_up;  // 累计工时
      pThis->previous_twt_up = pThis->current_twt_up;

      //==油耗统计=====
      pThis->current_tfc_up = CAN_GetUpEngineTfc(); // 上车总油耗
      if (pThis->current_tfc_up > pThis->previous_tfc_up)
      {
        pThis->fdv_up = pThis->current_tfc_up - pThis->previous_tfc_up;
      }
      else
      {
        pThis->fdv_up = 0;
      }
      pThis->tfc_up += pThis->fdv_up;  // 累计油耗
      pThis->previous_tfc_up = pThis->current_tfc_up;
    } // end if (pThis->up_work_flag)

    //==下车工作==============================
    if (pThis->down_work_flag)
    {
      //==工时统计=====
      pThis->current_twt_down = CAN_GetDownEngineTwt(); // 下车总工作时间
      if (pThis->current_twt_down > pThis->previous_twt_down)
      {
        pThis->tdv_down = pThis->current_twt_down - pThis->previous_twt_down;
      }
      else
      {
        pThis->tdv_down = 0;
      }
      pThis->twt_down += pThis->tdv_down;  // 累计工时
      pThis->previous_twt_down = pThis->current_twt_down;

      //==油耗统计=====
      pThis->current_tfc_down = CAN_GetDownEngineTfc(); // 下车总油耗
      if (pThis->current_tfc_down > pThis->previous_tfc_down)
      {
        pThis->fdv_down = pThis->current_tfc_down - pThis->previous_tfc_down;
      }
      else
      {
        pThis->fdv_down = 0;
      }
      pThis->tfc_down += pThis->fdv_down;  // 累计油耗
      pThis->previous_tfc_down = pThis->current_tfc_down;
    } // end if (pThis->down_work_flag)
  } // end if (DO_1MIN_FLAG==ZXSTS_TRUE)

  //==5分钟任务===========================
  if (DO_5MIN_FLAG==ZXSTS_TRUE)
  {
    DO_5MIN_FLAG = ZXSTS_FALSE;

    //==上车平均油耗=====
    if (pThis->twt_up > 0)
    {
      pThis->afc_up = (uint16_t)(pThis->tfc_up / pThis->twt_up);
    }
    //==下车平均油耗=====
    if (pThis->twt_down > 0)
    {
      pThis->afc_down = (uint16_t)(pThis->tfc_down / pThis->twt_down);
    }
  }
}

//==初始化发动机工作统计参数=================================================
void ZxStsEngine_Initialize(zxsts_engine_context_t* pThis)
{
  pThis->pto_status = 0x00;
  pThis->up_work_flag = 0x00;
  pThis->down_work_flag = 0x00;
  pThis->tdv_down = 0x00;
  pThis->tdv_up = 0x00;
  pThis->fdv_up = 0x00;
  pThis->fdv_down = 0x00;

  // 从EEPROM内获取
  //pThis->engine_type = ;
  //pThis->twt_up = ;
  //pThis->twt_down = ;
  //pThis->tfc_up = ;
  //pThis->tfc_down = ;
  //pThis->afc_up = ;
  //pThis->afc_down = ;
  
  pThis->current_twt_up = pThis->twt_up;
  pThis->previous_twt_up = pThis->twt_up;
  pThis->current_twt_down = pThis->twt_down;
  pThis->previous_twt_down = pThis->twt_down;
  pThis->current_tfc_up = pThis->tfc_up;
  pThis->previous_tfc_up = pThis->tfc_up;
  pThis->current_tfc_down = pThis->tfc_down;
  pThis->previous_tfc_down = pThis->tfc_down;
}

#endif


