/*****************************************************************************
* @FileName: AuxCom.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modem模块和Micro辅助通信协议实现
******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "AuxComHW.h"
#include "config.h"

/******************************************************************************
 * Typedef
 ******************************************************************************/


/******************************************************************************
 *   Macros
 ******************************************************************************/
#define AuxCom_Receive     USART6_ReceiveData
#define AuxCom_Transmit    USART6_TransmitData

#define AUX_MAX_TXBUF_SIZE  USART6_TX_BUFFER_SIZE
#define AUX_MAX_RXBUF_SIZE  USART6_RX_BUFFER_SIZE
#define aux_rx_buffer       usart6_rx_buffer
#define aux_rx_size         usart6_rx_size

#define AUX_DELAY_MS(ms)   do { osDelay(ms);} while(0)
#define PART(x)     1

// FSRV协议定义
#define FCLIT_MSG_HEAD            0x7E    // 帧头
#define FCLIT_MSG_TAIL            0x0D0A  // 帧尾
#define FCLIT_SEND_COMMAND_ID     0x84
#define FCLIT_RECV_COMMAND_ID     0x04
#define FCLIT_PACKET_SIZE         RFU_BUFFER_SIZE

#define FCLIT_TIMEOUT_SP          50  // 基于100ms
#define FCLIT_RETRY_SP            2   // 超时重试次数

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(1B)+数据包(NB)+校验(1B)+帧尾(2B)
#define FCLIT_FRAME_STX_FIELD        0x00
#define FCLIT_FRAME_SIZE_HIGH_FIELD  0x01
#define FCLIT_FRAME_SIZE_LOW_FIELD   0x02
#define FCLIT_FUNCTION_CODE_FIELD    0x03
#define FCLIT_SERIAL_NUMBER_FIELD    0x04
#define FCLIT_DATA_START_FIELD       0x05

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
extern volatile uint16_t usart6_rx_size;
extern uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];

static uint8_t aux_tx_buffer[AUX_MAX_TXBUF_SIZE];
static uint8_t aux_com_buffer[AUX_MAX_RXBUF_SIZE]; // AuxCom任务数据缓存

rfu_context_t rfu_context;
fclit_context_t fclit_context;

uint8_t rfu_data_buffer[RFU_BUFFER_SIZE];

/******************************************************************************
 * RTOS相关
 ******************************************************************************/
//=消费者==============================================================
#define AppThreadPriority_AuxComProcess   osPriorityHigh2
osThreadId_t tid_AuxComProcess;

const osThreadAttr_t AppThreadAttr_AuxComProcess =
{
  .priority = AppThreadPriority_AuxComProcess,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // 字节
};

//=生产者==============================================================
#define AppThreadPriority_AuxComProduce   osPriorityHigh1
osThreadId_t tid_AuxComProduce;

const osThreadAttr_t AppThreadAttr_AuxComProduce =
{
  .priority = AppThreadPriority_AuxComProduce,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // 字节
};

/******************************************************************************
 * 计算校验和
*******************************************************************************/
uint8_t iFCLIT_CalcSumCheck(uint8_t* pbuf,uint16_t len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }

  return sum;
}

/******************************************************************************
 * FCLIT周期性调用函数(周期100mS,无阻塞)
*******************************************************************************/
void FCLIT_Do100msTasks(void)
{
  //==重试次数判断===========================================
  if(fclit_context.retry_cnt > fclit_context.retry_sp)
  {
    fclit_context.retry_cnt = 0x00;
    if(fclit_context.state < FCLIT_STATE_SEND_UR_REQ)
    {
      fclit_context.ud_result = 0x03;  // 未成功
      fclit_context.state = FCLIT_STATE_SEND_UR_REQ;  // 向4G上报升级失败
    }
    else
    {
      fclit_context.state = FCLIT_STATE_IDLE;  // 退出升级
    }
  }

  //==超时重试判断===========================================
  if(fclit_context.timeout_100ms)
  {
    fclit_context.timeout_100ms--;
    if(fclit_context.timeout_100ms==0x00)
    {
      if(fclit_context.state==FCLIT_STATE_WAIT_UD_RSP)
      {
        fclit_context.state = FCLIT_STATE_SEND_UD_REQ;
      }
      else if(fclit_context.state==FCLIT_STATE_WAIT_UR_RSP)
      {
        fclit_context.state = FCLIT_STATE_SEND_UR_REQ;
      }
      else
      {
        fclit_context.state = FCLIT_STATE_IDLE;  // 退出升级
      }
    }
  }
}

/******************************************************************************
 * 初始化FSRV参数
*******************************************************************************/
void FCLIT_Init(void)
{
  fclit_context.state = FCLIT_STATE_IDLE;
  fclit_context.dev_id = 0x00;  // 无效
  fclit_context.msg_sn = 0x00;
  fclit_context.fw_packet_index = 0x00;
  fclit_context.fw_block_size = FCLIT_PACKET_SIZE;
  fclit_context.start_address = 0x00;
  fclit_context.ending_address = 0x00;
  fclit_context.total_block_count = 0x00;
  fclit_context.percent = 0x00;
  //fclit_context.result = 0x00;

  fclit_context.tx_size = 0x00;
  fclit_context.tx_data = aux_tx_buffer;

  fclit_context.rx_size = 0x00;
  fclit_context.rx_data = aux_rx_buffer;

  fclit_context.retry_sp = FCLIT_RETRY_SP;
  fclit_context.retry_cnt = 0;

  fclit_context.timeout_100ms_sp = FCLIT_TIMEOUT_SP;
  fclit_context.timeout_100ms = fclit_context.timeout_100ms_sp;
}

#if (PART("发送FCLIT数据"))
/*************************************************************************
 * 发送固件升级相关消息
*************************************************************************/
//==向4G模块发送固件升级通知的应答(ST->4G)================================
void iFCLIT_SendUnRspMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUn:%d!\n", pThis->un_result);
#endif

  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->rx_msg_sn;  // 响应流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = pThis->un_result;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);

  if (pThis->un_result == 0x00) // 执行成功
  {
    pThis->state = FCLIT_STATE_START;  // 启动升级
  }
  else
  {
    pThis->state = FCLIT_STATE_IDLE;  // 结束升级
  }
}

//==启动下载任务=============================================================
void iFCLIT_start(fclit_context_t* pThis)
{
#if FCLIT_DEBUG
  PcDebug_Printf("FclitSt!\n");
#endif

  rfu_EraseFlashHexFile();  // 擦除Flash
  pThis->retry_cnt = 0;
  pThis->ud_result = 0x01;  // 未成功
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->fw_packet_index = 0x00;
}

//==固件升级下载请求(ST->4G)==============================================
void iFCLIT_SendUdReqMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUd!\n");
#endif

  pThis->msg_sn++;
  pThis->retry_cnt++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x02;  // 命令标识
  pdata[pos++] = (uint8_t)(pThis->fw_packet_index>>8);  // 下载升级包序号
  pdata[pos++] = (uint8_t)pThis->fw_packet_index;
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==向4G上报升级结果(ST->4G)==============================================
void iFCLIT_SendUrReqMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUr:%d!\n", pThis->ud_result);
#endif

  pThis->msg_sn++;
  pThis->retry_cnt++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x03;  // 命令标识
  pdata[pos++] = pThis->dev_id;  // 升级目标设备
  pdata[pos++] = pThis->ud_result;  // 下载结果
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

/*************************************************************************
 * 100ms调用一次
*************************************************************************/
void FCLIT_ProduceSendMsg(fclit_context_t* pThis)
{
  if (pThis->state==FCLIT_STATE_IDLE) /// 空闲
  {
    return;
  }

  switch (pThis->state)
  {
  case FCLIT_STATE_NOTIFIED: /// 向4G发送升级通知响应
    iFCLIT_SendUnRspMsg(pThis);
    pThis->state = FCLIT_STATE_START;
    break;

  case FCLIT_STATE_START: /// 启动升级(初始化参数)
    iFCLIT_start(pThis);
    pThis->state = FCLIT_STATE_SEND_UD_REQ;
    break;

  case FCLIT_STATE_SEND_UD_REQ:  /// 向4G发送下载数据请求
    iFCLIT_SendUdReqMsg(pThis);
    pThis->state = FCLIT_STATE_WAIT_UD_RSP;
    break;

  case FCLIT_STATE_SEND_UR_REQ:  /// 向4G发送升级结果请求
    iFCLIT_SendUrReqMsg(pThis);
    pThis->state = FCLIT_STATE_WAIT_UR_RSP;
    break;

  default:
    break;
  }

  FCLIT_Do100msTasks();
}
#endif

/******************************************************************************
 * 处理收到4G模块消息
*******************************************************************************/
#if (PART("接收FCLIT数据"))
//==判断MOMI信息校验和==========================================================
uint8_t iFCLIT_CheckMsg(fclit_context_t* pThis)
{
  uint8_t *pdata = pThis->rx_data;
  uint16_t size = pThis->rx_size;
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;
  uint16_t msg_tail;

  if (pdata[0] != FCLIT_MSG_HEAD) // 帧头错误
  {
    return FCLIT_NOK;
  }

  msg_tail = (uint16_t)(pdata[size-2]<<8) + pdata[size-1]; // 获取帧尾
  if (msg_tail != FCLIT_MSG_TAIL) // 帧尾错误
  {
    return FCLIT_NOK;
  }

  if (pdata[3] != FCLIT_RECV_COMMAND_ID) // 命令字段错误
  {
    return FCLIT_NOK;
  }

  msg_len = size-6; // 不含帧头(1B)+数据长度(2B)+检验和(1B)+帧尾(2B)
  received_check_sum = pdata[size-3]; // 接收到的检验和
  calculated_check_sum = iFCLIT_CalcSumCheck(&pdata[FCLIT_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
  if (received_check_sum==calculated_check_sum) //check if crc's match
  {
    return FCLIT_OK;
  }
  else
  {
    return FCLIT_NOK;
  }
}

//==解析升级通知指令(4G->ST)===================================================
void iFCLIT_AnalyzeUnReqMsg(fclit_context_t* pThis)
{
  uint8_t *pdata;
  uint16_t pos;
  uint16_t msg_len;
  uint16_t msg_sn;
  uint8_t rfu_type;
  uint8_t rfu_dev;

  if (pThis->state != FCLIT_STATE_IDLE) // 升级忙,忽略此次升级
  {
    return;
  }

  msg_sn = pThis->rx_data[FCLIT_SERIAL_NUMBER_FIELD];  // 消息流水号
  pThis->rx_msg_sn = msg_sn;  // 流水号
  pThis->un_result = 0x00;  // 执行成功
  pThis->state = FCLIT_STATE_NOTIFIED;  // 状态跳转(发送升级通知响应)

  msg_len = pThis->rx_data[FCLIT_FRAME_SIZE_HIGH_FIELD];  // 数据长度
  msg_len <<= 8;
  msg_len += pThis->rx_data[FCLIT_FRAME_SIZE_LOW_FIELD];
  if (msg_len < 20) // 无数据
  {
    pThis->un_result = 0x01;  // 格式异常
    return;
  }
  msg_len -= 3;  // 数据包的长度(不包含功能码(1B)+流水号(1B)+校验(1B))

  //==数据内容======================================================
  pos = 0x00;
  pdata = &pThis->rx_data[FCLIT_DATA_START_FIELD];
  pos++;  // 命令标识
  rfu_type = pdata[pos++];  // 升级类型
  rfu_dev = pdata[pos++];  // 升级目标设备
  if (rfu_dev > 3 || rfu_dev==0x00) // 0x01=ECU控制器, 0x02=显示器, 0x03=ST协处理器
  {
    pThis->un_result = 0x02;  // 设备不支持
    return;
  }
  //==数据内容======================================================
  rfu_context.type = rfu_type;  // 升级类型
  rfu_context.dev = rfu_dev;  // 升级目标设备
  pThis->dev_id = rfu_dev;

  rfu_context.file_name_length = pdata[pos++];  // 固件名称长度
  memcpy(rfu_context.file_name, &pdata[pos], rfu_context.file_name_length);  // 固件名称(ASCII)
  pos += rfu_context.file_name_length;

  rfu_context.file_version_length = pdata[pos++];  // 固件版本长度
  memcpy(rfu_context.file_version, &pdata[pos], rfu_context.file_version_length);  // 固件版本(ASCII)
  pos += rfu_context.file_version_length;

  rfu_context.file_length = pdata[pos++];  // 固件大小(4B)
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];

  rfu_context.total_block_count = pdata[pos++];  // 固件总包数(2B)
  rfu_context.total_block_count <<= 8;
  rfu_context.total_block_count += pdata[pos++];

  rfu_context.block = 0x00;  // 初始化
  rfu_context.cumulated_address = 0x00;
  rfu_context.ending_address = rfu_context.file_length;

  rfu_context.plain_crc32val = pdata[pos++];  // 固件CRC校验(4B)
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];

  pThis->fw_block_size = pdata[pos++];  // 单包大小(固定值1024)
  pThis->fw_block_size <<= 8;
  pThis->fw_block_size += pdata[pos++];

  if (pThis->fw_block_size >= FCLIT_FIRMWARE_PACKET_LEN) // 单包大小判断
  {
    pThis->un_result = 0x03;  // 其他
    return;
  }
  //===============================================================
}

//pThis->retry_cnt = 0x00;  // 重试计数器
//pThis->timeout_100ms = pThis->timeout_100ms_sp;  // 复位超时计时器
//pThis->msg_sn = 0x00;  // 清零流水号

//==解析升级下载响应指令(4G->ST)===================================================
void iFCLIT_AnalyzeUdRspMsg(fclit_context_t* pThis)
{
  uint8_t *pdata;
  uint16_t pos;
  uint16_t current_block_index;  // 当前包序
  uint16_t total_block_num;  // 总包数量
  uint16_t packet_len;
  uint8_t rfu_status;

  if (pThis->rx_size < 6) // 无数据
  {
    pThis->ud_result = 0x01;  // 格式异常
    return;
  }

  //==数据内容======================================================
  pos = 0x00;
  pdata = &pThis->rx_data[FCLIT_DATA_START_FIELD];
  pos++;  // 命令标识

  current_block_index = pdata[pos++];  //==当前包序列号
  current_block_index <<= 8;
  current_block_index += pdata[pos++];
  if (current_block_index != rfu_context.block)
  {
    return;
  }

  total_block_num = pdata[pos++];  //==文件总包数
  total_block_num <<= 8;
  total_block_num += pdata[pos++];
  if (total_block_num != rfu_context.total_block_count)
  {
    return;
  }

  packet_len = pdata[pos++];  //==本包固件数据长度
  packet_len <<= 8;
  packet_len += pdata[pos++];
  if (packet_len == 0)
  {
    return;
  }

  rfu_SaveFlashHexFile(&rfu_context, &pdata[pos], packet_len);  // 将BIN文件写入SPI FLASH
  rfu_context.block++;
  rfu_context.percent = rfu_context.block * 100L / rfu_context.total_block_count;
  if (current_block_index==(total_block_num-1)) // 传输结束(最后一包)
  {
    rfu_status = rfu_CheckNewFirmware(&rfu_context, rfu_data_buffer, RFU_BUFFER_SIZE);
    if (RFU_OK==rfu_status)
    {
      pThis->ud_result = 0;
      if (rfu_context.dev==0x01) // 控制器
      {
        PcDebug_SendString("RfuCrc:Ctl-Ok!\n");
      }
      else if (rfu_context.dev==0x02) // 显示器
      {
        PcDebug_SendString("RfuCrc:LCD-Ok!\n");
      }
      else if (rfu_context.dev==0x03) // 终端协处理器
      {
        PcDebug_SendString("RfuCrc:ST-Ok!\n");
      }
      else
      {
        PcDebug_SendString("RfuCrc:Ok!\n");
      }
    }
    else
    {
      pThis->ud_result = 1;
      PcDebug_SendString("RfuCrc:Err!\n");
    }

    pThis->state = FCLIT_STATE_SEND_UR_REQ;
    rfu_context.cumulated_address = 0;
    rfu_context.block = 0;
    return ;
  }

  pThis->retry_cnt = 0;
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->state = FCLIT_STATE_SEND_UD_REQ;

  return ;
}

//==解析升级上报响应指令(4G->ST)==============================================
void iFCLIT_AnalyzeUrRspMsg(fclit_context_t* pThis)
{
  if (0==pThis->ud_result)  // 固件下载成功
  {
    pThis->state = FCLIT_STATE_IDLE;
    if ((pThis->dev_id==0x03) || (memcmp(rfu_context.file_name, "ZXM2M_ST", 8)==0))
    {
      Tbox_SetMachineState(TBOX_STATE_IAP); // T-BOX进入升级模式
    }
    // 在此处增加对外部控制器或显示器升级处理
  }
}

//============================================================================
static void FCLIT_ProcessRecvMsg(fclit_context_t* pThis)
{
  uint8_t msg_status;
  uint8_t command;

  msg_status = iFCLIT_CheckMsg(pThis);
  if (msg_status==FCLIT_OK)
  {
    command = pThis->rx_data[FCLIT_DATA_START_FIELD];  // 命令标识
    switch (command)
    {
    case FCLIT_COMMAND_TYPE_NOTIFIE:  // 升级通知
      iFCLIT_AnalyzeUnReqMsg(pThis);
      break;

    case FCLIT_COMMAND_TYPE_DOWNLOAD:  // 下载数据
      iFCLIT_AnalyzeUdRspMsg(pThis);
      break;

    case FCLIT_COMMAND_TYPE_REPORT:  // 上报结果
      iFCLIT_AnalyzeUrRspMsg(pThis);
      break;

    default:
      break;
    }
  }
}
#endif

/******************************************************************************
 * 辅助串口数据发送进程
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 100m执行一次
  {
    AUX_DELAY_MS(OS_TICKS_PER_SEC/10); // 100ms执行一次
    FCLIT_ProduceSendMsg(&fclit_context); // 发送MOMI消息
  }
}

/******************************************************************************
 * 处理通过辅助串口发来的M2M透传指令
*******************************************************************************/
void AuxCom_ProcessM2mMsg(uint8_t *pdata, uint16_t size)
{
  if (pdata[0]==0x05 && (size>=10))
  {
    memcpy(aux_com_buffer, pdata, size);
    SbusMsg_AuxCom.data_size = size;  // 数据内容长度
    SbusMsg_AuxCom.data = aux_com_buffer;  // 获取数据地址
    SYSBUS_PutMbox(SbusMsg_AuxCom);  // 发送消息
  }
}

/******************************************************************************
 * 辅助串口数据接收进程
*******************************************************************************/
void AppTask_AuxComProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (AuxCom_Receive(&pdata, &len)) // 等待信号量(阻塞)
    {
      if (pdata[0]==0x05) // 透传M2M命令请求CMD_REQ指令
      {
        if ( (pdata[15]=='P' && pdata[16]=='W') || (pdata[15]=='R' && pdata[16]=='C') ) // PW指令和RC指令
        {
          AuxCom_ProcessM2mMsg(pdata, len);
        }
      }
      else
      {
        fclit_context.rx_data = pdata;
        fclit_context.rx_size = len;
        FCLIT_ProcessRecvMsg(&fclit_context);
      }
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceInit(void)
{
  USART6_Initialize(AUXCOM_UART_BAUDRATE);
  FCLIT_Init();
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceStart(void)
{
  tid_AuxComProcess = osThreadNew(AppTask_AuxComProcess, NULL, &AppThreadAttr_AuxComProcess);
  tid_AuxComProduce = osThreadNew(AppTask_AuxComProduce, NULL, &AppThreadAttr_AuxComProduce);
}

//-----文件AuxCom.c结束---------------------------------------------

