/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modem模块和Micro通信协议实现
******************************************************************************/
//-----头文件调用------------------------------------------------------------
//#include "MomiHW.h"
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
#define MOMI_TransmitData     USART3_TransmitData

#define MOMI_MAX_TXBUF_SIZE  1500
#define MODEM_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

#define momiMsgQueueENTER_CRITICAL() ()
#define momiMsgQueueEXIT_CRITICAL()  ()

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

// TLV状态有效标志
extern bittype2 zxup_tlv_flag1;
extern bittype2 zxup_tlv_flag2;
extern bittype2 zxup_tlv_flag3;
extern bittype2 zxdown_tlv_flag1;
extern bittype2 zxdown_tlv_flag2;
extern bittype2 zxengine_tlv_flag;
extern bittype2 zxstatistics_tlv_flag;
extern bittype2 zxversion_tlv_flag;

uint8_t zxup_buffer[]; /// 上车数据缓存
uint8_t zxdown_buffer[]; /// 下车底盘数据缓存
uint8_t zxengine_buffer[]; /// 下车发动机数据缓存
uint8_t zxstatistics_buffer[]; ///统计数据缓存
uint8_t zxversion_buffer[]; /// 版本信息缓存



/******************************************************************************
 * Macros
 ******************************************************************************/
#define MOMI_UART_RX_BUFFER_SIZE   1500

#define MOMI_BUADRATE         B115200
#define MOMI_BUFFER_MAX_SIZE  MOMI_UART_RX_BUFFER_SIZE
#define MOMI_Transmit          MOMI_UartTransmitData
#define MOMI_Receive           MOMI_UartReceiveData

#define MOMI_TRANSMIT_MUTEX_LOCK()     do{pthread_mutex_lock(&mid_MomiTransmit);}while(0)
#define MOMI_TRANSMIT_MUTEX_UNLOCK()   do{pthread_mutex_unlock(&mid_MomiTransmit);}while(0)

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t momi_uart_rx_buffer[DBG_UART_RX_BUFFER_SIZE];
int fd_momi = -1;

static uint8_t pc_debug_buffer[PC_DEBUG_BUFFER_MAX_SIZE]; // PcDebug任务数据缓存

// 递归互斥
pthread_mutex_t mid_MomiTransmit;


#define DEV_TTY_MCU_CAN "/dev/ttyHS0"

/*****************************************************************************
 * 串口初始化函数
 ****************************************************************************/
void MOMI_UartInitialize(void)
{

  struct termios options;

  pthread_mutex_init(&gMcuCanSendMutex, NULL);   //

  /* open uart */
  McuCan_fd = open(DEV_TTY_MCU_CAN, O_RDWR|O_NOCTTY);

  if (McuCan_fd < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY_MCU_CAN, McuCan_fd);
    close(McuCan_fd);
  }
  else
  {

  }
  printf("McuCan_fd: %d \n", McuCan_fd);

  /* configure uart */
  tcgetattr(McuCan_fd, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VTIME] = 1; // read timeout 单位*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, B115200);//波特率设置
  cfsetospeed(&options, B115200);//波特率设置
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(McuCan_fd, TCIFLUSH);
  tcsetattr(McuCan_fd, TCSANOW, &options);
}

//===================================================================================
BOOL MOMI_UartReceiveData(uint8 **data, uint16* len)
{
  int ret;
  memset(MCU_Can_Recv_Buff,0,MCU_CAN_RECV_BUFF_LEN);
  if ((ret = read(McuCan_fd, MCU_Can_Recv_Buff, MCU_CAN_RECV_BUFF_LEN-1)) > 0)
  {
    *data = MCU_Can_Recv_Buff;
    *len = (uint16)ret;
    return TRUE;
  }
  else
    return FALSE;
}

//===================================================================================
uint16 MOMI_UartTransmitData(uint8_t *data, uint16_t size)
{
  int ret;

  if ((0==Len) || (NULL==data))
    return 0;

  /* write uart */
  pthread_mutex_lock(&mid_MomiTransmit);       //互斥锁
  ret = write(McuCan_fd, data, Len);
  pthread_mutex_unlock(&mid_MomiTransmit);     //互斥锁解锁

  if (ret != Len)
  {
    printf("ERROR MCU_CAN write ret=%d\n", ret);
    return 0;
  }
  else
    return Len;
}









/******************************************************************************
 * 消息队列定义
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x02
#define MOMI_RECV_COMMAND_ID     0x01
#define MOMI_MSG_QUEUE_MAX_SIZE  4096
#define ONE_MOMI_CMD_MAX_SIZE    1500
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

#if (PART("MOMI环形数组"))
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

/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI解析TLV"))

//==终端自身采集信息====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXINFO_BUFFER+2))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // 有效标志
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxinfo_tlv_flag.word = tlv_flag;

    memcpy(zxinfo_buffer, &pValue[pos], SIZE_OF_ZXINFO_BUFFER);
  }

  return retVal;
}

//==GPS采集信息====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F002(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;
    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==起重机上车信息=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F010(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXUP_BUFFER+6))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // 有效标志1
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag1.word = tlv_flag;

    tlv_flag = pValue[pos++]; // 有效标志2
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag2.word = tlv_flag;

    tlv_flag = pValue[pos++]; // 有效标志3
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag3.word = tlv_flag;

    memcpy(zxup_buffer, &pValue[pos], SIZE_OF_ZXUP_BUFFER);
  }

  return retVal;
}

//==起重机底盘信息=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F011(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXDOWN_BUFFER+4))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // 有效标志1
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxdown_tlv_flag1.word = tlv_flag;

    tlv_flag = pValue[pos++]; // 有效标志2
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxdown_tlv_flag2.word = tlv_flag;

    memcpy(zxdown_buffer, &pValue[pos], SIZE_OF_ZXDOWN_BUFFER);
  }

  return retVal;
}

//==起重机下车发动机信息=================================================
static uint8_t iMomi_AnalyzeTlvMsg_F012(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER+2))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // 有效标志
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxengine_tlv_flag.word = tlv_flag;

    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==故障类消息信息=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F020(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;

    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==统计类消息信息=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F030(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXSTATISTICS_BUFFER+2))
  {
    retVal = 1;
    tlv_flag = pValue[pos++]; // 有效标志
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxstatistics_tlv_flag.word = tlv_flag;
    
    memcpy(zxstatistics_buffer, &pValue[pos], SIZE_OF_ZXSTATISTICS_BUFFER);
  }

  return retVal;
}

//==车身ECU版本信息====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F040(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXVERSION_BUFFER+2))
  {
    retVal = 1;
    tlv_flag = pValue[pos++]; // 有效标志
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxversion_tlv_flag.word = tlv_flag;

    memcpy(zxversion_buffer, &pValue[pos], SIZE_OF_ZXVERSION_BUFFER);
  }

  return retVal;
}

//==CAN帧信息======================================================
static uint8_t iMomi_AnalyzeTlvMsg_F050(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint8_t can_num;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;
    can_num = pValue[pos++]; // 有效标志
    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
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
#if 0
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // 帧头(1B)
  pos += 2;             // 长度(命令号->校验和)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[pos++] = 0x01;  // 流水号
  pdata[pos++] = 0x01;

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
#endif
}

/*************************************************************************
 * 10ms调用一次
*************************************************************************/
void Momi_ProduceSendMsg(void)
{
  static uint8_t divide_for_300ms = 30;
  static uint8_t divide_for_500ms = 50;
  static uint8_t divide_for_1second = 100;

  // 终端信息(周期型300ms)
  if (divide_for_300ms)
  {
    divide_for_300ms--;
  }
  else
  {
    divide_for_300ms = 30;  // 300ms任务
    //iMomi_SendColtMsg();
  }

  // 车辆工况信息(周期型500ms)
  if (divide_for_500ms)
  {
    divide_for_500ms--;
  }
  else
  {
    divide_for_500ms = 50;  // 500ms任务
    //iMomi_SendVehicleMsg();
  }

  // GPS位置信息(周期型1s)
  if (divide_for_1second)
  {
    divide_for_1second--;
  }
  else
  {
    divide_for_1second = 100;  // 1s任务
    //iMomi_SendGpsMsg();
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
  
  switch()
  {
    case 0xF001: // 终端自身采集信息
      retVal = iMomi_AnalyzeTlvMsg_F001(pValue, len);
      break;
      
    case 0xF002: // GPS采集信息
      retVal = iMomi_AnalyzeTlvMsg_F002(pValue, len);
      break;

    case 0xF010: // 起重机上车信息
      retVal = iMomi_AnalyzeTlvMsg_F010(pValue, len);
      break;
      
    case 0xF011: // 起重机底盘信息
      retVal = iMomi_AnalyzeTlvMsg_F011(pValue, len);
      break;
      
    case 0xF012: // 起重机下车发动机信息
      retVal = iMomi_AnalyzeTlvMsg_F012(pValue, len);
      break;
     
    case 0xF020: //故障类消息信息
      retVal = iMomi_AnalyzeTlvMsg_F020(pValue, len);
      break;
      
    case 0xF030: // 统计类消息信息
      retVal = iMomi_AnalyzeTlvMsg_F030(pValue, len);
      break;
      
    case 0xF040: // 车身ECU版本信息
      retVal = iMomi_AnalyzeTlvMsg_F040(pValue, len);
      break;
      
    case 0xF050: // CAN帧信息
      retVal = iMomi_AnalyzeTlvMsg_F050(pValue, len);
      break;
  }

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
 * 将收到的数据放入队列
*******************************************************************************/
void MOMI_ReceiveData(uint8_t *data, uint16_t size)
{  
  uint16_t it;
  
  for (it = 0; it < size; it++)
  {
    momi_msg_queue_push(data[it]); // 将收到的数据放入队列
  }
}

/******************************************************************************
 * 4G模块发送报文给ST单片机
*******************************************************************************/
void pthread_MomiProduce(void *argument)
{
  while (1)  // 100m执行一次
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/10); // 100ms执行一次
    Momi_ProduceSendMsg();  // 发送MOMI消息
  }
}

/******************************************************************************
 * 处理ST单片机发给4G的报文
*******************************************************************************/
void pthread_MomiProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;
  uint16_t cmd_size;、

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
  //Modem_GpioInitialize();
  //USART3_Initialize(MODEM_UART_BAUDRATE);
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_MOMI_PROCESS_ID], NULL, pthread_MomiProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MOMI_PRODUCE_ID], NULL, pthread_MomiProduce, NULL);
  usleep(10);
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

