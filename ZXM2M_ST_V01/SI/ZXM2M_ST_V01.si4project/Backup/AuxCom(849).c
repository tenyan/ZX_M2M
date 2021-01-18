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
#define AuxCom_ReceiveData     USART6_ReceiveData
#define AuxCom_TransmitData    USART6_TransmitData

#define AUX_MAX_TXBUF_SIZE  USART6_TX_BUFFER_SIZE
#define AUX_MAX_RXBUF_SIZE  USART6_RX_BUFFER_SIZE
#define aux_rx_buffer       usart6_rx_buffer
#define aux_rx_size         usart6_rx_size

#define AUX_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

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

/******************************************************************************
 * 消息队列定义
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x01
#define MOMI_RECV_COMMAND_ID     0x02
#define MOMI_MSG_QUEUE_MAX_SIZE  1024
#define ONE_MOMI_CMD_MAX_SIZE    256
#define MOMI_MSG_HEAD            0x7E      // 帧头
#define MOMI_MSG_TAIL            0xFFFCFFFF  // 帧尾

/*************************************************************************
 * 10ms调用一次
*************************************************************************/
void AuxCom_ProduceSendMsg(void)
{
  static uint8_t divide_for_300ms = 30;
  static uint8_t divide_for_500ms = 50;
  static uint8_t divide_for_1second = 100;

  if(Modem_GetState()==MODEM_STATE_DATA_READY) // 模块处于已开机状态
  {
    // 终端信息(周期型300ms)
    if(divide_for_300ms)
    {  divide_for_300ms--;}
    else
    {
      divide_for_300ms = 30;  // 300ms任务
      iMomi_SendColtMsg();  // F001
    }

    // 车辆工况信息(周期型500ms)
    if(divide_for_500ms)
    {  divide_for_500ms--;}
    else
    {
      divide_for_500ms = 50;  // 500ms任务
      iMomi_SendVehicleMsg(); // F010, F011, F012
    }

    // GPS位置信息(周期型1s)
    if(divide_for_1second)
    {  divide_for_1second--;}
    else
    {
      divide_for_1second = 100;  // 1s任务
      //iMomi_SendGpsMsg();
    }

    // 故障信息(事件型)
    // iMomi_SendDtcMsg();
    
    // CAN帧信息(事件型)
    //iMomi_SendCanMsg();

    // 版本信息(上电发一次)

    //iMomi_SendVersionMsg();

    // 统计类消息(ACC关发送一次)
    //iMomi_SendStatisticsMsg();
  }
  else
  {
    divide_for_300ms = 30;
    divide_for_500ms = 50;
    divide_for_1second = 100;
  }
}
#endif

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void AuxCom_ProcessRecvData(uint8_t *pdata, uint16_t size)
{
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
}
#endif

/******************************************************************************
 * 4G模块主进程,负责对模块的所有操作
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 10m执行一次
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/100); // 10ms执行一次
    AuxCom_ProduceSendMsg();  // 发送MOMI消息
  }
}

/******************************************************************************
 * CAT1模块数据接收进程
*******************************************************************************/
void AppTask_AuxComProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (AuxCom_ReceiveData(&pdata, &len)) // 等待信号量(阻塞)
    {
      AuxCom_ProcessRecvData(pdata, len);
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


