/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modem模块和Micro通信协议实现
******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "MomiHW.h"
#include "config.h"

/******************************************************************************
 * Typedef
 ******************************************************************************/
typedef uint16_t (*iMomi_BuildTlvMsgFun)(uint8_t *pbuf);
typedef uint16_t (*iMomi_AnalyzeTlvMsgFun)(uint8_t* pValue, uint16_t len);
typedef struct
{
  uint16_t type;
  iMomi_BuildTlvMsgFun pfun_build;
  iMomi_AnalyzeTlvMsgFun pfun_analyze;
}iMomi_CmdTlv_t;

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MOMI_ReceiveData      USART3_ReceiveData
#define MOMI_TransmitData     USART3_TransmitData

#define MOMI_MAX_TXBUF_SIZE  USART3_TX_BUFFER_SIZE
#define MOMI_MAX_RXBUF_SIZE  USART3_RX_BUFFER_SIZE
#define momi_rx_buffer       usart3_rx_buffer
#define momi_rx_size          usart3_rx_size

#define MODEM_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

#define momiMsgQueueENTER_CRITICAL() USART_ITConfig(USART3, USART_IT_IDLE, DISABLE) // 关闭串口空闲接收中断
#define momiMsgQueueEXIT_CRITICAL()  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE) // 开启串口空闲接收中断

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
modem_state_t modem_state;

extern volatile uint16_t usart3_rx_size;
extern uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

/******************************************************************************
 * RTOS相关
 ******************************************************************************/
//=消费者==============================================================
#define AppThreadPriority_ModemProcess   osPriorityHigh2
osThreadId_t tid_ModemProcess;

const osThreadAttr_t AppThreadAttr_ModemProcess =
{
  .priority = AppThreadPriority_ModemProcess,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // 字节
};

 //=生产者==============================================================
#define AppThreadPriority_ModemProduce   osPriorityHigh1
osThreadId_t tid_ModemProduce;

const osThreadAttr_t AppThreadAttr_ModemProduce =
{
  .priority = AppThreadPriority_ModemProduce,
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

// 消息队列定义
typedef struct
{
  uint16_t head;  // 队列头
  uint16_t tail;  // 队列尾
  uint8_t data[MOMI_MSG_QUEUE_MAX_SIZE];  // 数据缓冲区
}momi_msg_queue_t;

static momi_msg_queue_t moni_msg_queue = {0,0,0};
static uint32_t momi_msg_state = 0;
static uint16_t momi_msg_pos = 0;
uint8_t moni_msg_buffer[ONE_MOMI_CMD_MAX_SIZE];

#if (PART("MOMI消息队列"))
//==清空队列数据==============================================================
void momi_msg_queue_reset()
{
  moni_msg_queue.head = 0;
  moni_msg_queue.tail = 0;
  momi_msg_state = 0;
  momi_msg_pos = 0;
}

//==将收到的数据放入队列中=====================================================
void momi_msg_queue_push(uint8_t dat)
{
  uint16_t pos;

  pos = (moni_msg_queue.head + 1) % MOMI_MSG_QUEUE_MAX_SIZE;

  if(pos != moni_msg_queue.tail) // 非满状态
  {
    moni_msg_queue.data[moni_msg_queue.head] = dat;
    moni_msg_queue.head = pos;
  }
}

//==从队列中取一个数据=========================================================
static void momi_msg_queue_pop(uint8_t* p_data)
{
  if(moni_msg_queue.tail != moni_msg_queue.head) //非空状态
  {
    *p_data = moni_msg_queue.data[moni_msg_queue.tail];
    moni_msg_queue.tail = (moni_msg_queue.tail + 1) % MOMI_MSG_QUEUE_MAX_SIZE;
  }
}

//==获取队列中有效数据个数=====================================================
static uint16_t momi_msg_queue_size(void)
{
  return ((moni_msg_queue.head + MOMI_MSG_QUEUE_MAX_SIZE - moni_msg_queue.tail) % MOMI_MSG_QUEUE_MAX_SIZE);
}

/******************************************************************************
*\brief  从队列中取出一条完整的消息
*\param  pbuffer 指令接收缓存区
*\param  size 指令接收缓存区大小
*\return  指令长度，0表示队列中无完整指令
******************************************************************************/
uint16_t momi_msg_queue_find(uint8_t *buffer, uint16_t size)
{
  uint16_t cmd_size = 0;
  uint8_t _data = 0;

  while(momi_msg_queue_size() > 0)
  {
    momiMsgQueueENTER_CRITICAL();
    momi_msg_queue_pop(&_data);  //取一个数据
    momiMsgQueueEXIT_CRITICAL();

    if((momi_msg_pos == 0)&& (_data != MOMI_MSG_HEAD)) //指令第一个字节必须是帧头,否则跳过
    {
      continue;
    }
    
    if(momi_msg_pos < size) // 防止缓冲区溢出
    {
      buffer[momi_msg_pos++] = _data;
    }
    
    momi_msg_state = ((momi_msg_state << 8) | _data); // 拼接最后4个字节,组成一个32位整数
    if(momi_msg_state == MOMI_MSG_TAIL)  //最后4个字节与帧尾匹配,得到完整帧
    {
      cmd_size = momi_msg_pos;  //指令字节长度
      momi_msg_state = 0;       //重新检测帧尾巴
      momi_msg_pos = 0;         //复位指令指针
      return cmd_size;
    }
  }
	
  return 0; // 没有完整数据帧
}
#endif

#if (PART("MODEM开关机处理"))
/******************************************************************************
* 
******************************************************************************/
modem_state_t Modem_GetState(void)
{
  return modem_state;
}

void Modem_SetState(modem_state_t state)
{
  modem_state = state;
}

/******************************************************************************
 * 功能: 使4G模块关机,并断电20秒
*******************************************************************************/
void Modem_PowerOff(void)
{
  uint8_t i;
  uint8_t j;
  uint8_t flag = 0;

  for (i=0; i<3; i++)
  {
    PcDebug_SendString("Modem PowerOff\n");

    MODEM_PWRKEY_H(); // 拉高1s
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_L(); // 拉低4s
    MODEM_DELAY_MS(OS_TICKS_PER_SEC*4);

    MODEM_PWRKEY_H(); //拉高
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    for (j=0; j<30; j++)
    {
      MODEM_DELAY_MS(OS_TICKS_PER_SEC);
      if (0==MODEM_STATUS_IN()) // 低电平: 模块Power off
      {
        flag = 1;
        break;
      }
    }

    if (1==flag)
    {
      break;
    }
  }
  DISABLE_MODEM_PWR();	// 关闭模块供电
  MODEM_DELAY_MS(OS_TICKS_PER_SEC*2); // 等待2s
}

/******************************************************************************
 * 功能: 使4G模块开机
*******************************************************************************/
void Modem_PowerOn(void)
{
  uint8_t i;
  uint8_t j;
  uint8_t flag = 0;
  uint32_t status = 0;

  MODEM_RESET_H(); // 释放模块复位
  MODEM_DTR_L(); // 使模块退出休眠模式
  ENABLE_MODEM_PWR();  // 给模块供电
  MODEM_DELAY_MS(OS_TICKS_PER_SEC*2); // 延时2s

  for (i=0; i<3; i++)
  {
    PcDebug_SendString("Modem PowerOn\n");

    MODEM_PWRKEY_H(); // 拉高1S
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_L(); // 拉低1S
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_H(); // 拉高
    MODEM_DELAY_MS(OS_TICKS_PER_SEC*3);

    for (j=0; j<15; j++) // 等待15s
    {
      MODEM_DELAY_MS(OS_TICKS_PER_SEC);
      status = MODEM_STATUS_IN();
      if (0 != status) // 高电平: 模块Power on and firmware ready
      {
        flag = 1;
        break;
      }
    }

    if (1==flag)
    {
      break;
    }
  }
}

/******************************************************************************
 * 模块状态机(10ms调用一次)
*******************************************************************************/
void Modem_StateManageService(void)               /// 阻塞
{
  uint8_t imodem_state;

  imodem_state = Modem_GetState();
  switch (imodem_state)
  {
  case MODEM_STATE_OFF:  // 关机后,无操作
    break;
    
  case MODEM_STATE_POWER_OFF:  // 模块关机
    Modem_PowerOff();
    Modem_SetState(MODEM_STATE_OFF);
    PcDebug_SendString("Modem Off!\n");
    break;

  case MODEM_STATE_POWER_ON:  // 模块开机
    Modem_PowerOn();
    Modem_SetState(MODEM_STATE_DATA_READY); // 转数据模式
    break;

  case MODEM_STATE_REPOWER_OFF:
    Modem_PowerOff();
    Modem_SetState(MODEM_STATE_SILENCE);
    break;
  
  case MODEM_STATE_DATA_READY:  // 模块可通信
    break; 

  case MODEM_STATE_SILENCE:  // 模块静默5分钟
    PcDebug_SendString("Modem Rest 10min\n");
    MODEM_DELAY_MS((300*OS_TICKS_PER_SEC));
    Modem_SetState(MODEM_STATE_POWER_ON); // 重新开机
    break;

  default:
    Modem_SetState(MODEM_STATE_REPOWER_OFF);
    break;
  }
}
#endif

/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI构建TLV"))

//==终端自身采集信息====================================================
static uint16_t iMomi_BuildTlvMsg_F001(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x01;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==GPS采集信息====================================================
static uint16_t iMomi_BuildTlvMsg_F002(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x02;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==起重机上车信息=====================================================
static uint16_t iMomi_BuildTlvMsg_F010(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXUP_BUFFER+6;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x10;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxup_tlv_flag1.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxup_tlv_flag1.word & 0xFF;
  pbuf[len++] = (zxup_tlv_flag2.word>>8) & 0xFF;
  pbuf[len++] = zxup_tlv_flag2.word & 0xFF;
  pbuf[len++] = (zxup_tlv_flag3.word>>8) & 0xFF;
  pbuf[len++] = zxup_tlv_flag3.word & 0xFF;
  memcpy(&pbuf[len], zxup_buffer, SIZE_OF_ZXUP_BUFFER);
  len += SIZE_OF_ZXUP_BUFFER;

  return len;
}

//==起重机底盘信息=====================================================
static uint16_t iMomi_BuildTlvMsg_F011(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXDOWN_BUFFER+4;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x11;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxdown_tlv_flag1.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxdown_tlv_flag1.word & 0xFF;
  pbuf[len++] = (zxdown_tlv_flag2.word>>8) & 0xFF;
  pbuf[len++] = zxdown_tlv_flag2.word & 0xFF;
  memcpy(&pbuf[len], zxdown_buffer, SIZE_OF_ZXDOWN_BUFFER);
  len += SIZE_OF_ZXDOWN_BUFFER;

  return len;
}

//==起重机下车发动机信息=================================================
static uint16_t iMomi_BuildTlvMsg_F012(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x12;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==故障类消息信息=====================================================
static uint16_t iMomi_BuildTlvMsg_F020(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x20;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==统计类消息信息=====================================================
static uint16_t iMomi_BuildTlvMsg_F030(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x30;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==车身ECU版本信息====================================================
static uint16_t iMomi_BuildTlvMsg_F040(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x40;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==CAN帧信息======================================================
static uint16_t iMomi_BuildTlvMsg_F050(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x50;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

#endif


//////////////////////////////////////////////////////////////////////////////////////////////
// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI构建和发送协议数据"))
//==计算校验和================================================================
uint8_t iMomi_CalcSumCheck(uint8_t* pbuf,uint16 len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  return sum;
}

//==发送终端采集信息===========================================================
void iMomi_SendColtMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数
  msg_len = iMomi_BuildTlvMsg_F001(&pdata[pos]); // 终端自身采集信息
  pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送GPS位置信息============================================================
void iMomi_SendGpsMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F002(&pdata[pos]); // GPS采集信息
  pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送车辆工况信息===========================================================
void iMomi_SendVehicleMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x03;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F010(&pdata[pos]); // 起重机上车信息
  pos += msg_len;

  msg_len = iMomi_BuildTlvMsg_F011(&pdata[pos]); // 起重机底盘信息
  pos += msg_len;

  msg_len = iMomi_BuildTlvMsg_F012(&pdata[pos]); // 起重机下车发动机信息
  pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送DTC故障信息============================================================
void iMomi_SendDtcMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F020(&pdata[pos]); // 故障类消息信息
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F021(&pdata[pos]); // 故障类消息信息
  //pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F022(&pdata[pos]); // 故障类消息信息
  //pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送统计类信息=============================================================
void iMomi_SendStatisticsMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F030(&pdata[pos]); // 起重机上车信息
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F031(&pdata[pos]); // 起重机上车信息
  //pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送版本信息===============================================================
void iMomi_SendVersionMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F040(&pdata[pos]); // 上车统计类消息信息
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F041(&pdata[pos]); // 下车统计类消息信息
  //pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==发送CAN帧信息==============================================================
void iMomi_SendCanMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV个数

  msg_len = iMomi_BuildTlvMsg_F050(&pdata[pos]); // CAN帧信息
  pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[2] = (msg_len>>8) & 0xFF; // 长度
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // 帧尾(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

/*************************************************************************
 * 10ms调用一次
*************************************************************************/
void Momi_ProduceSendMsg(void)
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
      iMomi_SendColtMsg();
    }

    // 车辆工况信息(周期型500ms)
    if(divide_for_500ms)
    {  divide_for_500ms--;}
    else
    {
      divide_for_500ms = 50;  // 500ms任务
      iMomi_SendVehicleMsg();
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

//////////////////////////////////////////////////////////////////////////////////////////////
// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI接收和解析协议数据"))
//==判断MOMI信息校验和==========================================================
uint8_t Momi_CheckMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;

  if(pdata[3] != MOMI_RECV_COMMAND_ID) // 命令字段错误
  {
    return MOMI_NOK;
  }

  msg_len = size-8;
  received_check_sum = pdata[size-5]; // 检验和
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // 计算校验值:从命令号开始计算
  if(received_check_sum==calculated_check_sum)  //check if crc's match
  {
    return MOMI_OK;
  }
  else
  {
    return MOMI_NOK;
  }
}

//==处理收到的TLV数据=============================================================
uint16_t iMomi_AnalyzeTlvData(uint16_t tag, uint16_t len, uint8_t *pValue)
{
  uint16_t retVal = 0;
  //uint16_t it = 0;
  //im2m_CmdTlv_t* p_CmdTlvDeal = NULL;

  //for (it = 0; it<NUM_OF_M2M_CMD_DEAL; it++)
  //{
  //  p_CmdTlvDeal = &m2m_CmdDealTbl[it];
  //  if (p_CmdTlvDeal->type != tag)
  //  {
  //    continue;
  //  }

  //  if (p_CmdTlvDeal->pfun_analyze != NULL)
  //  {
  //    retVal = p_CmdTlvDeal->pfun_analyze(pValue, len);
  //  }

  //  break;
  //}

  return retVal;
}


/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void Momi_ProcessRecvData(uint8_t *pdata, uint16_t size)
{
  uint8_t retVal = 0;
  uint8_t msg_status;
  uint16_t msg_len;
  uint16_t msg_sn;
  uint8_t tlvNum = 0;
  uint16_t tag;
  uint16_t length;
  uint16_t pos = 0;
  uint8_t* pbuf = &pdata[6];
  uint8_t fail_tag_num = 0;

  msg_status = Momi_CheckMsg(pdata, size);
  if(msg_status==MOMI_OK)
  {
    msg_len = pdata[1];  // 数据长度
    msg_len <<= 8;
    msg_len += pdata[2];
    if(msg_len < 5)  // 无数据
    {
      return;
    }
    msg_len -= 4;  // 数据包的长度(不包含功能码(1B)+流水号(2B)+校验(1B))
    msg_sn = pdata[4];  // 消息流水号
    msg_sn <<= 8;
    msg_sn += pdata[5];
    
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
void AppTask_ModemProduce(void *argument)
{
  while (1)  // 10m执行一次
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/100); // 10ms执行一次
    Modem_StateManageService();  // modem管理
    Momi_ProduceSendMsg();  // 发送MOMI消息
  }
}

/******************************************************************************
 * CAT1模块数据接收进程
*******************************************************************************/
void AppTask_ModemProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;
  uint16_t cmd_size;

  while (1)
  {
    if (MOMI_ReceiveData(&pdata, &len)) // 等待信号量(阻塞)
    {
      while(momi_msg_queue_size()>0)
      {
        cmd_size = momi_msg_queue_find(moni_msg_buffer, ONE_MOMI_CMD_MAX_SIZE); // 查找MONI消息
        if(cmd_size>0) // 得到一条完整的MOMI消息
        {
          Momi_ProcessRecvData(moni_msg_buffer, cmd_size);
        }
      }
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceInit(void)
{
  Modem_GpioInitialize();
  USART3_Initialize(MODEM_UART_BAUDRATE);
  Modem_SetState(MODEM_STATE_POWER_OFF);
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceStart(void)
{
  tid_ModemProcess = osThreadNew(AppTask_ModemProcess, NULL, &AppThreadAttr_ModemProcess);  
  tid_ModemProduce = osThreadNew(AppTask_ModemProduce, NULL, &AppThreadAttr_ModemProduce);
}

#if 0
//==type按照递增顺序填写=================================================
iMomi_CmdTlv_t Momi_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/
  {0x0000, im2m_BuildTlvMsg_0000, im2m_AnalyzeTlvMsg_0000},/*设备终端编号*/
  {0x0001, im2m_BuildTlvMsg_0001, NULL},                   /*终端设备软件版本编号，只读*/
  {0x1002,                  NULL, im2m_AnalyzeTlvMsg_1002},//升级服务器IP
};
#define NUM_OF_MOMI_CMD_DEAL   (sizeof(Momi_CmdDealTbl)/sizeof(Momi_CmdDealTbl))
#endif

//-----文件CelluraCore.c结束---------------------------------------------

