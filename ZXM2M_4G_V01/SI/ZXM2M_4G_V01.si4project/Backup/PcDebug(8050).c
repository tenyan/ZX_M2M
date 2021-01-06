/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebug.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-5-29
 * @brief     本文件为PcDebug功能模块协议层处理的文件
 ********************************************************************************/
#include "config.h"

static uint8_t public_buffer[1200];

/******************************************************************************
 * Macros
 ******************************************************************************/
#define DBG_UART_RX_BUFFER_SIZE   1500

#define PC_DEBUG_BUADRATE         B115200
#define PC_DEBUG_BUFFER_MAX_SIZE  DBG_UART_RX_BUFFER_SIZE
#define PcDebug_Transmit          DBG_UartTransmitData
#define PcDebug_Receive           DBG_UartReceiveData

#define PRINTF_MUTEX_LOCK()     do{pthread_mutex_lock(&mid_PcDebugPrintf);}while(0)
#define PRINTF_MUTEX_UNLOCK()   do{pthread_mutex_unlock(&mid_PcDebugPrintf);}while(0)

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t dbg_uart_rx_buffer[DBG_UART_RX_BUFFER_SIZE];
int fd_pcdebug = -1;

static uint8_t pc_debug_buffer[PC_DEBUG_BUFFER_MAX_SIZE]; // PcDebug任务数据缓存

// 递归互斥
pthread_mutex_t mid_PcDebugPrintf;

static bittype enable_dbg_flag = {.byte = 0x00};
#define ENABLE_DBG_TYPE_MODEM_FLAG  enable_dbg_flag.b.bit0
#define ENABLE_DBG_TYPE_GPS_FLAG    enable_dbg_flag.b.bit1
#define ENABLE_DBG_TYPE_CAN_FLAG    enable_dbg_flag.b.bit2
#define ENABLE_DBG_TYPE_SYS_FLAG    enable_dbg_flag.b.bit3
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit4
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit5
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit6
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit7

/*****************************************************************************
 * 串口初始化函数
 ****************************************************************************/
void DBG_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  fd_pcdebug = open(DEV_TTYGS0_CDC_DEBUG, O_RDWR|O_NOCTTY); // open uart 
  if (fd_pcdebug < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTYGS0_CDC_DEBUG, fd_pcdebug);
    close(fd_pcdebug);
    fd_pcdebug = -1;
    return ;
  }

  printf("fd_pcdebug=%d\n", fd_pcdebug);

  // configure uart
  tcgetattr(fd_pcdebug, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VTIME] = 1; // read timeout 单位*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate); // 波特率设置
  cfsetospeed(&options, baudrate); // 波特率设置
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(fd_pcdebug, TCIFLUSH);
  tcsetattr(fd_pcdebug, TCSANOW, &options);
}

//===================================================================================
uint8_t DBG_UartReceiveData(uint8_t **data, uint16_t* size)
{
  int retVal;

  if (fd_pcdebug<0)
  {
    return FALSE;
  }

  retVal = read(fd_pcdebug, dbg_uart_rx_buffer, (DBG_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
  if (retVal > 0)
  {
    *data = dbg_uart_rx_buffer;
    *size = (uint16_t)retVal;
    
    return TRUE;
  }

  return FALSE;
}

//===================================================================================
uint16_t DBG_UartTransmitData(uint8_t *data,uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_pcdebug<0))
  {
    return 0;
  }
 
  retVal = write(fd_pcdebug, data, size); // 写串口数据
  if (retVal != size)
  {
    printf("ERROR Debug write ret=%d\n", retVal);
    close(fd_pcdebug);
    DBG_UartInitialize(B115200);
    return 0;
  }

  return size;
}

/*****************************************************************************
 * 调试状状态:
 * Bit3: 1=输出SYS调试信息,0=不输出
 * Bit2: 1=输出CAN调试信息,0=不输出
 * Bit1: 1=输出GPS调试信息,0=不输出
 * Bit0: 1=输出Modem调试信息,0=不输出
*****************************************************************************/
void PcDebug_SetStatus(uint8_t enable_type)
{
  enable_dbg_flag.byte = enable_type;
}

/******************************************************************************
 * 功能: 调试输出数据信息的函数,可以被不同模块调用
 * 输入: pdata:指针，指向调试输出数据的指针;
         size:要输出数据的长度;
         msg_types:当前该调试函数被哪个模块所调用,模块的编号
         1=GSM模块;2=GPS模块;3=MCU模块;4=可以输出任意数据
 * 输出: 无
*******************************************************************************/
void PcDebug_SendData(uint8_t* pdata,uint16_t size,uint8_t msg_types)
{
  //ENABLE_DBG_TYPE_MODEM_FLAG = 1;
  //ENABLE_DBG_TYPE_GPS_FLAG = 1;

  if (ENABLE_DBG_TYPE_MODEM_FLAG)
  {
    if (msg_types == DBG_MSG_TYPE_MODEM)
    {
      PcDebug_Transmit(pdata, size);
    }
  }
  else if (ENABLE_DBG_TYPE_GPS_FLAG)
  {
    if (msg_types == DBG_MSG_TYPE_GPS)
    {
      PcDebug_Transmit(pdata, size);
    }
  }
  else if (ENABLE_DBG_TYPE_CAN_FLAG)
  {
    if (msg_types == DBG_MSG_TYPE_CAN)
    {
      PcDebug_Transmit(pdata, size);
    }
  }
  else if (ENABLE_DBG_TYPE_SYS_FLAG)
  {
    if (msg_types == DBG_MSG_TYPE_SYS)
    {
      PcDebug_Transmit(pdata, size);
    }
  }

  if (msg_types == DBG_MSG_TYPE_ANYDATA) //直接输出数据
  {
    PcDebug_Transmit(pdata, size);
  }
}

//--DMA方式发送字符串-----------------------------------------------------------
void PcDebug_SendString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// 获取字符串字符数量
  PcDebug_Transmit((uint8_t*)pstr, size);
}

//uint16_t length;
//static char _dbg_TXBuff[256];
//--DMA方式的printf-------------------------------------------------------------
void PcDebug_Printf(const char *format,...)
{
  uint16_t length;
  char _dbg_TXBuff[256];

  PRINTF_MUTEX_LOCK(); // 上锁

  va_list args;

  va_start(args, format);
  length = vsnprintf((char*)_dbg_TXBuff, sizeof(_dbg_TXBuff), (char*)format, args);
  va_end(args);
  
  PcDebug_Transmit((uint8_t*)_dbg_TXBuff, length); // DMA
  PRINTF_MUTEX_UNLOCK();  // 解锁
}

/**********************************************************************************
 * 功能: 对数据包进行转义 0x7B7B-->ESCESC和0x7B7B; 0x7D7D-->ESCESC和0x7D7D
 * 输入: src=待转译的数据包
 * 输出: des=转译后的数据包
 * 返回: 转译后的数据包长度
*********************************************************************************/
uint16_t PcDebug_EncodeM2mData(uint8_t* src,uint16_t srcLen)
{
  uint16_t i;
  uint16_t desLen = 0;

  uint8_t *pDes;
  uint8_t *pSrc;
  uint8_t *temp = public_buffer;

  pSrc = src;
  pDes = &temp[0];
  desLen = srcLen;
  for (i = 0; i < srcLen; i++)
  {
    if ((0x7B==*pSrc) && (0x7B==*(pSrc+1))) // 0x7B7B-->ESCESC和0x7B7B
    {
      *pDes++ = 0x1B;
      *pDes++ = 0x1B;
      *pDes++ = 0x7B;
      *pDes++ = 0x7B;
      pSrc += 2;
      desLen += 2;
    }
    else if ((0x7D==*pSrc)&&(0x7D==*(pSrc+1)) ) // 0x7D7D-->ESCESC和0x7D7D
    {
      *pDes++ = 0x1B;
      *pDes++ = 0x1B;
      *pDes++ = 0x7D;
      *pDes++ = 0x7D;
      pSrc += 2;
      desLen += 2;
    }
    else
    {
      *pDes++ = *pSrc++;
    }
  }
  memcpy(src, temp, desLen);
  return desLen;
}

/******************************************************************************
 * 发送M2M协议命令响应
*******************************************************************************/
void PcDebug_SendM2mRsp(uint8_t *pdata, uint16_t size)
{
  uint16_t len = 0;
  uint8_t* pbuf = public_buffer;

  len = PcDebug_EncodeM2mData(pdata, size); // 反转义
  pbuf[0] = 0x7B;     // 包头
  pbuf[1] = 0x7B;
  memcpy(&pbuf[2], pdata,len);
  len += 2;
  pbuf[len++] = 0x7D; // 包尾
  pbuf[len++] = 0x7D;

  PcDebug_Transmit(pbuf, len);
}

/*****************************************************************************
 * 功能: 反转义 ESCESC0x7B7B-->0x7B7B; ESCESC0x7D7D-->7D7D
 * 输入: Ptr,需要进行转义数据指向的指针地址  
         len 需要转义的数据的长度
 * 输出: 转义后的长度
*****************************************************************************/
uint16_t PcDebug_DecodeM2mData(uint8_t *pbuf, uint16_t len)
{
  uint16_t it=0;

  for(it=0; it< (len-3); it++)
  {
    if((pbuf[it]==0x1b)&&(pbuf[it+1]==0x1b)&&(pbuf[it+2]==0x7b)&&(pbuf[it+3]==0x7b))
    {
      memmove(&pbuf[it],&pbuf[it+2],len-2-it);
      len -= 2;
    }
    else if((pbuf[it]==0x1b)&&(pbuf[it+1]==0x1b)&&(pbuf[it+2]==0x7d)&&(pbuf[it+3]==0x7d))
    {
      memmove(&pbuf[it],&pbuf[it+2],len-2-it);
      len -= 2; 
    }
    else 
    {
    
    }
  }

  return len;
}

/*************************************************************************
 *
*************************************************************************/
void PcDebug_Service(uint8_t *pdata, uint16_t len)
{
  uint16_t frame_len;
  
  if ((pdata[0]==0x7B)&&(pdata[1]==0x7B)&&(pdata[len-2]==0x7D)&&(pdata[len-1]==0x7D)) // 帧头(0x7B0x7B)和帧尾(0x7D0x7D)
  {
    if (len >= 17)
    {
      len -= 4; // 去掉帧头和帧尾
      memcpy(pc_debug_buffer, &pdata[2], len); // 得到M2M数据帧
      
      frame_len = PcDebug_DecodeM2mData(pc_debug_buffer, len); // 反转义
      UNUSED(frame_len);
      
      // frame_len;  // 数据内容长度
      // pc_debug_buffer; // 获取数据地址
#if 0
      if (pc_debug_buffer[0]==0x05&&((pc_debug_buffer[15]=='P'&&pc_debug_buffer[16]=='W')||(pc_debug_buffer[15]=='R'&&pc_debug_buffer[16]=='C')))
      {
        A5_UART0_Write(pc_debug_buffer, frame_len);
      }
      DealSerCmd(PCSendBuff, usLen,SRCDEVICE_ID_SETTOOL,0);
#endif
    }
  }
}

/******************************************************************************
* 与PC机通信的调试任务
*******************************************************************************/
void* pthread_PcDebug(void *argument)
{
  uint16_t size; // 收到的数据长度
  uint8_t *pdata; // 数据起始地址

  while (1)
  {
    if (PcDebug_Receive(&pdata,&size)) // 等待数据(阻塞)
    {
      PcDebug_Service(pdata,size);
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void PcDebug_ServiceInit(void)
{
  pthread_mutex_init(&mid_PcDebugPrintf, NULL);
  DBG_UartInitialize(PC_DEBUG_BUADRATE);
}

/*************************************************************************
 *
*************************************************************************/
void PcDebug_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_PC_DEBUG_ID], NULL, pthread_PcDebug, NULL);
  usleep(10);
}

//-----文件PcDebug.c结束---------------------------------------------

