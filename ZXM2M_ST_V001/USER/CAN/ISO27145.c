/*****************************************************************************
* @FileName: ISO27145.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-10-25
* @brief:    ISO-27145协议获取OBD信息相关的.C文件
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
#define ISO27145_RESPONSE_TIMEOUT_SP  25  // 2.5S
#define ISO27145_REQUEST_PERIOD_SP    50  // 5S
#define ISO27145_DEBUG                0

/******************************************************************************
* Data Types and Globals
******************************************************************************/
iso27145_context_t iso27145_ctx;
iso_request_state_t iso_request_state;

/*************************************************************************
 * 初始化函数
*************************************************************************/
void EP_ObdInfoDataInit(void)
{
  obd_info.protocol_type = 0xfe;
  obd_info.mil_status = 0xfe;

  obd_info.diag_valid_flag = 0;
  obd_info.diag_supported_status = 0x00;
  obd_info.diag_readiness_status = 0x00;

  obd_info.vin_valid_flag = 0;
  memset(obd_info.vin,'0',17);

  obd_info.calid_valid_flag = 0;
  memset(obd_info.calid,'0',18);

  obd_info.cvn_valid_flag = 0;
  memset(obd_info.cvn,'0',18);

  obd_info.iupr_valid_flag = 0;
  memset(obd_info.iupr,'0',36);
}

/*************************************************************************
 *
*************************************************************************/
void dCan_ISO27145Init(iso27145_context_t* pThis)
{
  pThis->engine_type = CAN_GetEngineType(); // 发动机类型:潍柴/杭发/重型
  pThis->requested_pid_flag = FALSE;
  pThis->requested_vin_flag = FALSE;
  pThis->requested_calid_flag = FALSE;
  pThis->requested_cvn_flag = FALSE;
  pThis->requested_iupr_flag = FALSE;
  pThis->requested_readiness_flag = FALSE;
  pThis->frame_index = 1;
  pThis->first_frame_flag = FALSE;

  pThis->request_state = ISO_REQUEST_STATE_INIT;
  pThis->state_transition = TRUE;
}

/*************************************************************************
 *
*************************************************************************/
void ISO27145_RequestStateOff(iso27145_context_t* pThis)
{
  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    EP_ObdInfoDataInit();

    pThis->engine_type = CAN_GetEngineType(); // 发动机类型:潍柴/杭发/重型
    pThis->requested_pid_flag = FALSE;
    pThis->requested_vin_flag = FALSE;
    pThis->requested_calid_flag = FALSE;
    pThis->requested_cvn_flag = FALSE;
    pThis->requested_iupr_flag = FALSE;
    pThis->requested_readiness_flag = FALSE;
    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: Off!\r\n");
#endif
  }
  else
  {
    if (COLT_GetAccStatus()==1 && (CAN_GetEpDataState()==1)) // ACC开和有下车CAN数据
    {
      if (CAN_GetEngineSpeed() > 300) // 下车发动机发动
      {
        pThis->request_state = ISO_REQUEST_STATE_IDLE;
        pThis->state_transition = TRUE;
      }
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void ISO27145_RequestStateIdle(iso27145_context_t* pThis)
{
  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;
    pThis->timer_100ms = ISO27145_REQUEST_PERIOD_SP; // 请求周期5s

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: Idle!\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_PID_F810;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * Protocol Identification
*************************************************************************/
void ISO27145_SendRequest_S22_F810(iso27145_context_t* pThis)
{
  if (pThis->requested_pid_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDBI_VIN_F802;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF8\x10\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF8\x10\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F810\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_VIN_F802;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * Vehicle Identification Number
*************************************************************************/
void ISO27145_SendRequest_S22_F802(iso27145_context_t* pThis)
{
  if (pThis->requested_vin_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDBI_SCI_F804;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF8\x02\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF8\x02\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F802\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_SCI_F804;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * Software Calibration Identification
*************************************************************************/
void ISO27145_SendRequest_S22_F804(iso27145_context_t* pThis)
{
  if (pThis->requested_calid_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDBI_CVN_F806;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF8\x04\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF8\x04\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F804\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_CVN_F806;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * Calibration Verification Number
*************************************************************************/
void ISO27145_SendRequest_S22_F806(iso27145_context_t* pThis)
{
  if (pThis->requested_cvn_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDBI_IUPR_F80B;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF8\x06\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF8\x06\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F806\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_IUPR_F80B;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * In Use (Monitor) Performance Ratio
*************************************************************************/
void ISO27145_SendRequest_S22_F80B(iso27145_context_t* pThis)
{
  if (pThis->requested_iupr_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDBI_READINESS_F401;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF8\x0B\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF8\x0B\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F80B\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDBI_READINESS_F401;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * the readiness of the OBD system
*************************************************************************/
void ISO27145_SendRequest_S22_F401(iso27145_context_t* pThis)
{
  if (pThis->requested_readiness_flag)
  {
    pThis->request_state = ISO_REQUEST_STATE_RDTCI_S19_42;
    pThis->state_transition = TRUE;
    return;
  }

  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x03\x22\xF4\x01\xAA\xAA\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x03\x22\xF4\x01\x00\x00\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=22,DID=F401\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_RDTCI_S19_42;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * reportWWHOBDDTCByMaskRecord
*************************************************************************/
void ISO27145_SendRequest_S19_42(iso27145_context_t* pThis)
{
  if (pThis->state_transition == TRUE)
  {
    pThis->state_transition = FALSE;
    pThis->state_started_delay = FALSE;
    pThis->engine_type = CAN_GetEngineType(); // 发动机厂家
  }

  if (pThis->state_started_delay == FALSE)
  {
    pThis->state_started_delay = TRUE;

    pThis->frame_index = 1;
    pThis->first_frame_flag = FALSE;
    if (pThis->engine_type == ENGINE_TYPE_WEICHAI)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x05\x19\x42\x33\x0C\x1E\xAA\xAA"); // 潍柴
    }
    else if (pThis->engine_type == ENGINE_TYPE_HANGFA)
    {
      CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x05\x19\x42\x33\x0C\x1E\x00\x00"); // 杭发
    }
    pThis->timer_100ms = ISO27145_RESPONSE_TIMEOUT_SP;

#if ISO27145_DEBUG
    PcDebug_SendString("ISO27145: SID=19,FID=42\r\n");
#endif
  }
  else
  {
    if (pThis->timer_100ms == 0)
    {
      pThis->request_state = ISO_REQUEST_STATE_IDLE;
      pThis->state_transition = TRUE;
    }
  }
}

/*************************************************************************
 * 发送流控帧
*************************************************************************/
void ISO27145_SendFlowControl(uint8_t engine_type)
{
  if (engine_type == ENGINE_TYPE_WEICHAI)
  {
    CAN1_TransmitData(CAN_FRAME_TYPE_EXT, 0x18DA00F1, 8, (uint8_t*)"\x30\x00\x00\xAA\xAA\xAA\xAA\xAA"); // 潍柴
  }
  else if (engine_type == ENGINE_TYPE_HANGFA)
  {
    CAN1_TransmitData(CAN_FRAME_TYPE_STD, 0x7E0, 8, (uint8_t*)"\x30\x00\x00\x00\x00\x00\x00\x00"); // 杭发
  }

  // PC_DebugPrintf("ISO27145: FlowControl\r\n");
}

/*************************************************************************
 * 发送ISO27145请求帧,此函数每100ms调用一次
 * 杭发和潍柴的CAN报文格式是相同的,不同的只有报文ID。
 * 请求CNA ID: 标准帧0x7E0(杭发)、扩展帧0x18DA00F1(潍柴)
 * 响应CAN ID: 标准帧0x7E8(杭发)、扩展帧0x18DAF100(潍柴)
*************************************************************************/
void dCan_SendISO27145Request(iso27145_context_t* pThis)
{
  switch (pThis->request_state)
  {
  case ISO_REQUEST_STATE_INIT:
    pThis->state_transition = TRUE;
    pThis->request_state = ISO_REQUEST_STATE_OFF;
    break;

  case ISO_REQUEST_STATE_OFF:
    ISO27145_RequestStateOff(pThis);
    break;

  case ISO_REQUEST_STATE_IDLE:
    ISO27145_RequestStateIdle(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_PID_F810: // Protocol Identification
    ISO27145_SendRequest_S22_F810(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_VIN_F802: // Vehicle Identification Number
    ISO27145_SendRequest_S22_F802(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_SCI_F804: // Software Calibration Identification
    ISO27145_SendRequest_S22_F804(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_CVN_F806: // Calibration Verification Number
    ISO27145_SendRequest_S22_F806(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_IUPR_F80B: // In Use (Monitor) Performance Ratio
    ISO27145_SendRequest_S22_F80B(pThis);
    break;

  case ISO_REQUEST_STATE_RDBI_READINESS_F401: // the readiness of the OBD system
    ISO27145_SendRequest_S22_F401(pThis);
    break;

  case ISO_REQUEST_STATE_RDTCI_S19_42: // = reportWWHOBDDTCByMaskRecord
    ISO27145_SendRequest_S19_42(pThis);
    break;

  default:
    pThis->state_transition = TRUE;
    pThis->request_state = ISO_REQUEST_STATE_INIT;
    break;
  }

  if (pThis->first_frame_flag) // 收到首帧,发送流控帧
  {
    pThis->first_frame_flag = FALSE;
    ISO27145_SendFlowControl(pThis->engine_type);
  }

  if (pThis->timer_100ms)
    pThis->timer_100ms--;

  if ((pThis->request_state>ISO_REQUEST_STATE_OFF) && (COLT_GetAccStatus()==0)) // ACC关闭
  {
    pThis->state_transition = TRUE;
    pThis->request_state = ISO_REQUEST_STATE_OFF;
  }
}

#define DTC_MESSAGE_BUFFER_SIZE    (MAX_DTC_TABLE_SIZE*5+6)
/*************************************************************************
 * 处理ISO27145响应帧
*************************************************************************/
void dCan_ProcessISO27145Response(iso27145_context_t* pThis,uint8_t *pdata,uint8_t size)
{
  uint16_t MID = 0x0000;
  static uint8_t rsp_message_length = 0x00;
  uint8_t i;

  // 故障码变量
  uint32_t dtc_code;
  uint8_t dtc_pos;
  static uint8_t dtc_total; // 故障码总数量
  static uint8_t dtc_cumulated_address; // 故障码信息累包计接收地址
  static uint8_t dtc_message_buf[DTC_MESSAGE_BUFFER_SIZE]; // 故障码信息缓存

  if (pThis->request_state <= ISO_REQUEST_STATE_IDLE) // 没有任何请求
  {
    return;
  }

  if ((pdata[0]&0xF0) == 0)	// 单帧
  {
    MID = (uint16_t)(pdata[2]<<8) + pdata[3];
  }
  else if ((pdata[0]&0xF0) == 0x10) // 首帧
  {
    MID = (uint16_t)(pdata[3]<<8) + pdata[4];
    pThis->first_frame_flag = 1;
  }

  switch (pThis->request_state)
  {
    // 解析参考SAEJ1979-DA 表G16 第161页
  case ISO_REQUEST_STATE_RDBI_PID_F810: // Protocol Identification
    // 1 byte unsigned numeric: 0x01 ISO 27145-4
    if ((pdata[0]==0x07) && (MID==0xF810) && (pThis->frame_index==1)) // 单帧
    {
      obd_info.protocol_type = pdata[4]; // 协议类型
      pThis->requested_pid_flag = 1; // 请求成功标志位
      pThis->timer_100ms = 10; // 1s后发送下一个请求
    }
    break;

    // 解析参考ISO 15031-5 2006 表G.2 第178页
  case ISO_REQUEST_STATE_RDBI_VIN_F802: // Vehicle Identification Number
    // 17 ASCII characters
    if (pdata[0]==0x10 && MID==0xF802 && pThis->frame_index==1)	// 首帧
    {
      memcpy(&obd_info.vin[0],&pdata[5],3);
      rsp_message_length = pdata[1]; // 消息长度
      pThis->frame_index = 2;
    }
    else if (pdata[0]==0x21 && pThis->frame_index==2) // 连续帧
    {
      memcpy(&obd_info.vin[3],&pdata[1],7);
      pThis->frame_index = 3;
    }
    else if (pdata[0]==0x22 && pThis->frame_index==3) // 连续帧
    {
      memcpy(&obd_info.vin[10],&pdata[1],7);
      pThis->frame_index = 4;
    }

    if (pThis->frame_index==4)
    {
      pThis->frame_index = 1;
      pThis->requested_vin_flag = 1; // 请求成功标志位
      obd_info.vin_valid_flag = 1; // VIN有效
      pThis->timer_100ms = 10; // 1s后发送下一个请求
      // 拷贝VIN码
    }
    break;

    // 解析参考ISO 15031-5 2006 表G.4 第179页
  case ISO_REQUEST_STATE_RDBI_SCI_F804: // Software Calibration Identification
    // 16 ASCII characters
    if (pdata[0]==0x10 && MID==0xF804 && pThis->frame_index==1)	// 首帧
    {
      memcpy(&obd_info.calid[0],&pdata[5],3);
      rsp_message_length = pdata[1]; // 消息长度
      pThis->frame_index = 2;
    }
    else if (pdata[0]==0x21 && pThis->frame_index==2) // 连续帧
    {
      memcpy(&obd_info.calid[3],&pdata[1],7);
      pThis->frame_index = 3;
    }
    else if (pdata[0]==0x22 && pThis->frame_index==3) // 连续帧
    {
      memcpy(&obd_info.calid[10],&pdata[1],6);
      pThis->frame_index = 4;
    }

    if (pThis->frame_index==4)
    {
      pThis->frame_index = 1;
      pThis->requested_calid_flag = 1; // 请求成功标志位
      obd_info.calid_valid_flag = 1; // calid有效
      pThis->timer_100ms = 10; // 1s后发送下一个请求
    }
    break;

    // 解析参考ISO 15031-5 2006 表G.6 第180页
  case ISO_REQUEST_STATE_RDBI_CVN_F806: // Calibration Verification Number
    // 4 byte hex: 07 62 F8 06 C9 73 E3 D8
    if (pdata[0]==0x07 && MID==0xF806 && pThis->frame_index==1)	// 单帧
    {
      memcpy(&obd_info.cvn[0],&pdata[4],4);
      pThis->requested_cvn_flag = 1; // 请求成功标志位
      obd_info.cvn_valid_flag = 1; // cvn有效
      pThis->timer_100ms = 10; // 1s后发送下一个请求
    }
    break;

    // 解析参考SAEJ1979-DA 表G11 第157页
  case ISO_REQUEST_STATE_RDBI_IUPR_F80B: // In Use (Monitor) Performance Ratio
    // 16 or 18 counters(32 or 36 Bytes)
    if (pdata[0]==0x10 && MID==0xF80B && pThis->frame_index==1)	//首帧
    {
      memcpy(&obd_info.iupr[0],&pdata[5],3);
      rsp_message_length = pdata[1]; // 消息长度
      pThis->frame_index = 2;
    }
    else if (pdata[0]==0x21 && pThis->frame_index==2) // 连续帧
    {
      memcpy(&obd_info.iupr[3],&pdata[1],7);
      pThis->frame_index = 3;
    }
    else if (pdata[0]==0x22 && pThis->frame_index==3) // 连续帧
    {
      memcpy(&obd_info.iupr[10],&pdata[1],7);
      pThis->frame_index = 4;
    }
    else if (pdata[0]==0x23 && pThis->frame_index==4) // 连续帧
    {
      memcpy(&obd_info.iupr[17],&pdata[1],7);
      pThis->frame_index = 5;
    }
    else if (pdata[0]==0x24 && pThis->frame_index==5) // 连续帧
    {
      memcpy(&obd_info.iupr[24],&pdata[1],7);
      pThis->frame_index = 6;
    }
    else if (pdata[0]==0x25 && pThis->frame_index==6) // 连续帧
    {
      pThis->frame_index = 7;
      if (rsp_message_length == (36+3))
        memcpy(&obd_info.iupr[31],&pdata[1],5); // 36 Bytes
      else
        memcpy(&obd_info.iupr[31],&pdata[1],1); // 32 Bytes
    }

    if (pThis->frame_index==7)
    {
      pThis->frame_index = 1;
      pThis->requested_iupr_flag = 1; // 请求成功标志位
      obd_info.iupr_valid_flag = 1; // iupr有效
      pThis->timer_100ms = 10; // 1s后发送下一个请求
    }
    break;

    // 解析参考SAEJ1979-DA 表B2 第8页
  case ISO_REQUEST_STATE_RDBI_READINESS_F401: // the readiness of the OBD system
    if (pdata[0]==0x07 && MID==0xF401 && pThis->frame_index==1)	// 单帧
    {
      bittype temp_val;
      bittype2 obd_supported_status; // 诊断支持状态
      bittype2 obd_readiness_status; // 就绪就绪状态

      obd_info.dtc_cnt = pdata[4] & 0x7F;
      obd_info.mil_status = (pdata[4]>>7) & 0x01; // 0=MIL_OFF,1=MIL_ON

      temp_val.byte = pdata[5]; ////////
      obd_supported_status.w.bit13 = temp_val.b.bit0; // Misfire monitoring supported
      obd_supported_status.w.bit14 = temp_val.b.bit1; // Fuel system monitoring supported
      obd_supported_status.w.bit15 = temp_val.b.bit2; // Comprehensive component monitoring supported
      //obd_supported_status.w.bitx = temp_val.b.bit3; // Compression ignition monitoring supported
      obd_readiness_status.w.bit13 = temp_val.b.bit4; // Misfire monitoring ready
      obd_readiness_status.w.bit14 = temp_val.b.bit5; // Fuel system monitoring ready
      obd_readiness_status.w.bit15 = temp_val.b.bit6; // Comprehensive component monitoring ready

      temp_val.byte = pdata[6]; //  compression ignition vehicles
      obd_supported_status.w.bit12 = temp_val.b.bit0; // NMHC catalyst monitoring supported
      obd_supported_status.w.bit11 = temp_val.b.bit1; // NOx/SCR aftertreatment monitoring supported
      //obd_supported_status.w.bitx = temp_val.b.bit2;// ISO/SAE reserved
      obd_supported_status.w.bit9 = temp_val.b.bit3; // Boost pressure system monitoring supported
      obd_supported_status.w.bit4 = temp_val.b.bit4; // A/C system refrigerant monitoring supported
      obd_supported_status.w.bit5 = temp_val.b.bit5; // Exhaust gas sensor monitoring supported
      obd_supported_status.w.bit10 = temp_val.b.bit6; // PM filter monitoring supported
      obd_supported_status.w.bit7 = temp_val.b.bit7; // EGR and/or VVT system monitoring supported

      temp_val.byte = pdata[7]; //  compression ignition vehicles
      obd_readiness_status.w.bit12 = temp_val.b.bit0; // NMHC catalyst monitoring  ready
      obd_readiness_status.w.bit11 = temp_val.b.bit1; // NOx/SCR aftertreatment monitoring ready
      obd_readiness_status.w.bit9 = temp_val.b.bit3; // Boost pressure system monitoring ready
      obd_readiness_status.w.bit4 = temp_val.b.bit4; // A/C system refrigerant monitoring ready
      obd_readiness_status.w.bit5 = temp_val.b.bit5; // Exhaust gas sensor monitoring ready
      obd_readiness_status.w.bit10 = temp_val.b.bit6; // PM filter monitoring ready
      obd_readiness_status.w.bit7 = temp_val.b.bit7; // EGR and/or VVT system monitoring ready

      obd_info.diag_supported_status = obd_supported_status.word;
      obd_info.diag_readiness_status = obd_readiness_status.word;
      obd_info.diag_valid_flag = 1; // diag支持和准备状态有效

      pThis->requested_readiness_flag = 1;
      pThis->timer_100ms = 10; // 1s后发送下一个请求
    }
    break;

    // 解析参考ISO 27145-3 2012 表39和表41 第39页
    // DCT格式是SAE J1979-DA-compliant format.
  case ISO_REQUEST_STATE_RDTCI_S19_42: // = reportWWHOBDDTCByMaskRecord
    if (pdata[0]==0x10 && pdata[2]==0x59 && pdata[3]==42 && pThis->frame_index==1)	// 首帧  10 24 59 42 33 5C FE 04
    {
      memset(dtc_message_buf, 0xFF, DTC_MESSAGE_BUFFER_SIZE);

      rsp_message_length = pdata[1]; // 消息长度
      if (rsp_message_length <= DTC_MESSAGE_BUFFER_SIZE)
      {
        memcpy(dtc_message_buf,&pdata[2],6); // 缓存DTC数据包
        dtc_cumulated_address = 6;
        pThis->frame_index = 2;
        dtc_total = (rsp_message_length-6)/5; // 故障码总数
      }
    }
    else if ((pdata[0]>=0x21) && (pThis->frame_index==2))	// 连续帧
    {
      if ((dtc_cumulated_address+7) >= rsp_message_length) // 最后一帧
      {
        memcpy((uint8_t*)(dtc_message_buf+dtc_cumulated_address),&pdata[1],(rsp_message_length-dtc_cumulated_address)); // 缓存最后一帧数据包
        for (i=0; i<dtc_total; i++) // 提取DTC
        {
          dtc_pos = 6 + i*5;
          dtc_code = (uint32_t)(dtc_message_buf[dtc_pos]) + (uint32_t)(dtc_message_buf[dtc_pos+1]<<8) + (uint32_t)(dtc_message_buf[dtc_pos+2]<<16) + (uint32_t)(dtc_message_buf[dtc_pos+3]<<24);
          if ((dtc_code!=0xFFFFFFFF) && (dtc_code!=0x00000000))
          {
            uint8_t val = DTC_SaveCode(&dtc_27145, dtc_code); // 存入故障码

            if (val == 1) // 新故障码
            {  dtc_27145.new_flag = 1;}
          }
        }
        dtc_total = 0;
        pThis->frame_index = 1;
      }
      else // 中间帧
      {
        memcpy((uint8_t*)(dtc_message_buf+dtc_cumulated_address),&pdata[1],7); // 缓存DTC数据包
        dtc_cumulated_address += 7;
      }
      return;
    }
    break;

  default:
    break;
  } // end switch (pThis->request_state)
}
