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

#define FCLIT_TIMEOUT_SP          30  // 基于100ms
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
//static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

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


#define FCLIT_FRAME_STX_FIELD        0x00
#define FCLIT_FRAME_SIZE_HIGH_FIELD  0x01
#define FCLIT_FRAME_SIZE_LOW_FIELD   0x02
#define FCLIT_FUNCTION_CODE_FIELD    0x03
#define FCLIT_SERIAL_NUMBER_FIELD    0x04
#define FCLIT_DATA_START_FIELD       0x05

/*************************************************************************
 * 固件升级相关消息
*************************************************************************/
//==计算校验和================================================================
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

//==向4G模块发送固件升级通知的应答(ST->4G)================================
void iFCLIT_SendUnRspMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
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

//==固件升级下载请求(ST->4G)==============================================
void iFCLIT_SendUdReqMsg(m2m_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
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
void iFCLIT_SendUrReqMsg(m2m_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
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
 * 10ms调用一次
*************************************************************************/
void FCLIT_ProduceSendMsg(void)
{
//  static uint8_t divide_for_300ms = 30;

}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void FCLIT_ProcessRecvMsg(uint8_t *pdata, uint16_t size)
{
#if 0
  uint8_t retVal = 0;
  uint8_t msg_status;
  uint16_t msg_len;
  uint16_t msg_sn;
  uint8_t tlvNum = 0;
  uint16_t tag;
  uint16_t length;
  
  uint16_t pos = 0;
  uint8_t* pbuf = &pdata[MOMI_DATA_START_FIELD];
  uint8_t fail_tag_num = 0;

  msg_status = Momi_CheckMsg(pdata, size);
  if(msg_status==MOMI_OK)
  {
    msg_len = pdata[MOMI_FRAME_SIZE_HIGH_FIELD];  // 数据长度
    msg_len <<= 8;
    msg_len += pdata[MOMI_FRAME_SIZE_LOW_FIELD];
    if(msg_len < 5)  // 无数据
    {
      return;
    }
    msg_len -= 4;  // 数据包的长度(不包含功能码(1B)+流水号(2B)+校验(1B))
    msg_sn = pdata[MOMI_SN_HIGH_FIELD];  // 消息流水号
    msg_sn <<= 8;
    msg_sn += pdata[MOMI_SN_LOW_FIELD];
    
    pos = 0;
    tlvNum = pbuf[pos++];  // TLV个数
    while (tlvNum) // TLV解析
    {
      if (pos >= msg_len) // 长度判断
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
      retVal = iMomi_AnalyzeTlvData(tag, length, &pbuf[pos]); // 解析
      if(MOMI_NOK==retVal)
      {
        fail_tag_num++;

        // 输出调试信息
      }
      pos += length; // 指向下一个TLV
      tlvNum--;
    }
  }
#endif
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
    FCLIT_ProduceSendMsg();  // 发送MOMI消息
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
    if (AuxCom_ReceiveData(&pdata, &len)) // 等待信号量(阻塞)
    {
      FCLIT_ProcessRecvMsg(pdata, len);
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceInit(void)
{
  USART6_Initialize(AUXCOM_UART_BAUDRATE);
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


