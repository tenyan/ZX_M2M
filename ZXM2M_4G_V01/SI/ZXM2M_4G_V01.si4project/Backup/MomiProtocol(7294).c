/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modem模块和Micro通信协议实现
******************************************************************************/
//-----头文件调用------------------------------------------------------------
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
#define DEV_TTY_MOMI_UART         "/dev/ttyHS0"
#define MOMI_UART_RX_BUFFER_SIZE  1500
#define MOMI_UART_BAUDRATE        B115200
#define MOMI_BUFFER_MAX_SIZE      MOMI_UART_RX_BUFFER_SIZE
#define MOMI_TransmitData         MOMI_UartTransmitData
#define MOMI_ReceiveData          MOMI_UartReceiveData

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t momi_uart_rx_buffer[MOMI_UART_RX_BUFFER_SIZE];
int fd_momi = -1;
pthread_mutex_t mid_MomiTransmit; // 递归互斥


#define MOMI_MAX_TXBUF_SIZE  1500
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

#if 0
// TLV状态有效标志
extern bittype2 zxup_tlv_flag1;
extern bittype2 zxup_tlv_flag2;
extern bittype2 zxup_tlv_flag3;
extern bittype2 zxdown_tlv_flag1;
extern bittype2 zxdown_tlv_flag2;
extern bittype2 zxengine_tlv_flag;
extern bittype2 zxstatistics_tlv_flag;
extern bittype2 zxversion_tlv_flag;

extern uint8_t zxup_buffer[]; /// 上车数据缓存
extern uint8_t zxdown_buffer[]; /// 下车底盘数据缓存
extern uint8_t zxengine_buffer[]; /// 下车发动机数据缓存
extern uint8_t zxstatistics_buffer[]; ///统计数据缓存
extern uint8_t zxversion_buffer[]; /// 版本信息缓存
#endif

rtc_date_t st_rtc_data;
can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];

/******************************************************************************
 * 消息队列定义
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x02
#define MOMI_RECV_COMMAND_ID     0x01
#define MOMI_MSG_QUEUE_MAX_SIZE  8192
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

static momi_msg_queue_t moni_msg_queue = {0,0};
static uint32_t momi_msg_state = 0;
static uint16_t momi_msg_pos = 0;
uint8_t moni_msg_buffer[ONE_MOMI_CMD_MAX_SIZE];

#if (PART("MOMI环形数组"))
//==清空队列数据==============================================================
void momi_msg_queue_reset(void)
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
    //momiMsgQueueENTER_CRITICAL();
    momi_msg_queue_pop(&_data);  //取一个数据
    //momiMsgQueueEXIT_CRITICAL();

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

/*****************************************************************************
 * 串口初始化函数
 ****************************************************************************/
void MOMI_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  pthread_mutex_init(&mid_MomiTransmit, NULL);
  fd_momi = open(DEV_TTY_MOMI_UART, O_RDWR|O_NOCTTY); // open uart
  if (fd_momi < 0) // 创建失败,如何处理??
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY_MOMI_UART, fd_momi);
    close(fd_momi);
    return;
  }

  printf("fd_momi: %d \n", fd_momi);
  // configure uart
  tcgetattr(fd_momi, &options);
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
  tcflush(fd_momi, TCIFLUSH);
  tcsetattr(fd_momi, TCSANOW, &options);
}

//==将收到的数据放入队列========================================================
uint8_t MOMI_UartReceiveData(void)
{
  uint16_t it;
  int retVal;
  
  retVal = read(fd_momi, momi_uart_rx_buffer, (MOMI_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
  if (retVal > 0)
  {
    for (it = 0; it < retVal; it++)
    {
      momi_msg_queue_push(momi_uart_rx_buffer[it]); // 将收到的数据放入队列
    }
    PcDebug_SendData(momi_uart_rx_buffer, retVal, DBG_MSG_TYPE_CAN);
    return TRUE;
  }

  return FALSE;
}

//===================================================================================
uint16_t MOMI_UartTransmitData(uint8_t *data, uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_momi<0))
  {
    return 0;
  }

  pthread_mutex_lock(&mid_MomiTransmit);  // 上锁
  retVal = write(fd_momi, data, size);
  pthread_mutex_unlock(&mid_MomiTransmit);  // 解锁
  if (retVal != size)
  {
    printf("Err Momi write ret=%d\n", retVal);
    return 0;
  }

  return size;
}

/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI解析TLV"))
//==解析终端自身采集信息==================================================
void Momi_AnalyzeAdSwitch(void)
{
  uint16_t tempVal;

  if (tlv_a5ff_valid_flag)
  {
    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR];  // 外部电源电压
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];
    colt_info.vraw = tempVal;

    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR];  // 内置电池电压
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];
    colt_info.vbat = tempVal;

    colt_info.switch1 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS3_ADDR];  // 开关量采集1
    colt_info.switch2 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS4_ADDR];  // 开关量采集2
    colt_info.switch3 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS5_ADDR];  // 开关量采集3
    colt_info.alarm = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS6_ADDR];  // 防拆类开关量
    colt_info.can_status = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS7_ADDR];  // CAN通信状态
    
    st_rtc_data.year = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR]; // RTC时间
    st_rtc_data.month = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+1];
    st_rtc_data.day = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+2];
    st_rtc_data.hour = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+3];
    st_rtc_data.minute = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+4];
    st_rtc_data.second = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+5];
    
    colt_info.st_tbox_mode = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS9_ADDR]; // 工作模式

    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS10_ADDR];  // 协处理器版本信息
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS10_ADDR+1];
    colt_info.st_version = tempVal;
  }
  else
  {
    
  }
}

//==终端自身采集信息====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXINFO_BUFFER+2))
  {
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F001\n");
//#endif
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
  //uint16_t pos = 0;
  //uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F002\n");
//#endif
    retVal = 1;
    //memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F010\n");
//#endif

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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F011\n");
//#endif

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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F012\n");
//#endif

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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F020\n");
//#endif

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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F030\n");
//#endif

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
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F040\n");
//#endif

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
  //uint16_t pos = 0;
  //uint8_t can_num;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
//#if MOMI_DEBUG
//    PcDebug_Printf("Momi:F050\n");
//#endif

    retVal = 1;
    //can_num = pValue[pos++]; // 有效标志
    //memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI构建和发送协议数据"))
//==计算校验和================================================================
uint8_t iMomi_CalcSumCheck(uint8_t* pbuf,uint16_t len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  
  return sum;
}


#define MOMI_E001_POS1    0                   // 面板1
#define MOMI_E001_POS2    (MOMI_E001_POS1+2)  // 面板2
#define MOMI_E001_POS3    (MOMI_E001_POS2+2)  // 面板3
#define MOMI_E001_POS4    (MOMI_E001_POS32)  // 面板3
#define MOMI_E001_POS5    (MOMI_E001_POS42)  // 面板3
#define MOMI_E001_POS6    (MOMI_E001_POS2+2)  // 面板3
#define MOMI_E001_POS7    (MOMI_E001_POS2+2)  // 面板3
#define MOMI_E001_POS8   (MOMI_E001_POS2+2)  // 面板3
#define MOMI_E001_POS9   (MOMI_E001_POS2+2)  // 面板3
#define MOMI_E001_POS10  (MOMI_E001_POS2+2)  // 面板3
#define SIZE_OF_MOMI_E001 (ZXUP_A5A9_POS3+2)  // 总字节数(6B)


//==4G工作状态和GPS时间信息====================================================
static uint16_t iMomi_BuildTlvMsg_E001(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXINFO_BUFFER+2;
  utc_time_t utc_time;
  rtc_date_t bj_time;

  pbuf[len++] = 0xE0; // TAG
  pbuf[len++] = 0x01;

  len += 2;  // LENGTH

  // VALUE
  if(M2M_GetConnStatus()==M2M_TRUE)  // 网络工作状态
  {pbuf[len++] = 0x01;}
  else
  {pbuf[len++] = 0x00;}
  tlv_length++;
  
  pbuf[len++] = GPS_GetPositioningStatus();  // GPS定位状态
  tlv_length++;
  
  pbuf[len++] = 0x00;  // WIFI工作状态
  tlv_length++;
  
  pbuf[len++] = 0x00;  // ETH工作状态
  tlv_length++;

  if(M2M_GetRfuStatus() > M2M_UPDATE_STATE_IDLE)  // 远程升级标志
  {  pbuf[len++] = 0x00;}
  else
  {  pbuf[len++] = 0x00;}
  tlv_length++;

  if(GPS_GetPositioningStatus())
  {
    utc_time = GPS_GetUtcTime();
    RTC_CovertUtcToBjt(&utc_time, &bj_time);
    pbuf[len++] = bj_time.year;  // GPS时间
    pbuf[len++] = bj_time.month;
    pbuf[len++] = bj_time.day;
    pbuf[len++] = bj_time.hour;
    pbuf[len++] = bj_time.minute;
    pbuf[len++] = bj_time.second;
  }
  tlv_length += 6;

  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;

  return len;
}

//==发送4G工作状态和GPS时间信息==================================================
void iMomi_Send4GStatusMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[MOMI_FRAME_STX_FIELD] = MOMI_MSG_HEAD;  // 帧头(1B)
  pdata[MOMI_FUNCTION_CODE_FIELD] = MOMI_SEND_COMMAND_ID;  // 命令号
  pdata[MOMI_SN_HIGH_FIELD] = 0x01;  // 流水号
  pdata[MOMI_SN_LOW_FIELD] = 0x01;
  pdata[MOMI_DATA_START_FIELD] = 0x01;  // TLV个数
  pos = MOMI_DATA_START_FIELD + 0x01;

  msg_len = iMomi_BuildTlvMsg_E001(&pdata[pos]); // 终端自身采集信息
  pos += msg_len;

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iMomi_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[MOMI_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[MOMI_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

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
    iMomi_Send4GStatusMsg();
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
    //PcDebug_SendString("MomiProduce!\n");
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)+帧尾(4B)
// 数据长度 = LengthOf{功能码(1B)+流水号(2B)+数据包(NB)+校验(1B)}
// 校验 = sum{功能码(1B)+流水号(2B)+数据包(NB)}
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI接收和解析协议数据"))
//==判断MOMI信息校验和==========================================================
uint8_t Momi_CheckMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;

  if(pdata[MOMI_FUNCTION_CODE_FIELD] != MOMI_RECV_COMMAND_ID) // 命令字段错误
  {
    //PcDebug_Printf("MomiCmdErr\n");
    return MOMI_NOK;
  }

  msg_len = size-8;  // 去掉{帧头(1B)+数据长度(2B)+校验(1B)+帧尾(4B)
  received_check_sum = pdata[size-5]; // 检验和
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[MOMI_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
  if(received_check_sum==calculated_check_sum)  //check if crc's match
  {
    //PcDebug_Printf("MomiCrcOK\n");
    return MOMI_OK;
  }
  else
  {
    //PcDebug_Printf("MomiCrcErr\n");
    return MOMI_NOK;
  }
}

//==处理收到的TLV数据=============================================================
uint16_t iMomi_AnalyzeTlvData(uint16_t tag, uint16_t len, uint8_t *pValue)
{
  uint16_t retVal = 0;
  
  switch(tag)
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
        PcDebug_Printf("MomiErr:Tlv=%x\n", tag); // 输出16进制
        // 输出调试信息
      }
      pos += length; // 指向下一个TLV
      tlvNum--;
    }
  }
}
#endif

/******************************************************************************
 * 4G模块发送报文给ST单片机
*******************************************************************************/
void* pthread_MomiProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms执行一次
    Momi_ProduceSendMsg();  // 发送MOMI消息
  }
}

/******************************************************************************
 * 处理ST单片机发给4G的报文
*******************************************************************************/
void* pthread_MomiProcess(void *argument)
{
  uint16_t cmd_size;

  while (1)
  {
    if (MOMI_ReceiveData()) // 等待数据(阻塞)
    {
      while(momi_msg_queue_size()>0)
      {
        cmd_size = momi_msg_queue_find(moni_msg_buffer, ONE_MOMI_CMD_MAX_SIZE); // 查找MONI消息
        if(cmd_size>0) // 得到一条完整的MOMI消息
        {
          //PcDebug_Printf("MomiQ:head=%d,tail=%d\n",moni_msg_queue.head, moni_msg_queue.tail);
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
  momi_msg_queue_reset();
  MOMI_UartInitialize(MOMI_UART_BAUDRATE);
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

