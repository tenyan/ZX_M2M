/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    本文件为辅助通信模块的c文件
 ********************************************************************************/
#include "config.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define AUX_UART_RX_BUFFER_SIZE  1500

#define AUX_COM_BUADRATE         B115200
#define AUX_COM_BUFFER_MAX_SIZE  AUX_UART_RX_BUFFER_SIZE


//#define PRINTF_MUTEX_LOCK()     do{pthread_mutex_lock(&mid_PcDebugPrintf);}while(0)
//#define PRINTF_MUTEX_UNLOCK()   do{pthread_mutex_unlock(&mid_PcDebugPrintf);}while(0)

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t aux_uart_rx_buffer[AUX_UART_RX_BUFFER_SIZE];
int fd_auxcom = -1;

static uint8_t aux_com_buffer[AUX_COM_BUFFER_MAX_SIZE]; // PcDebug任务数据缓存

// 递归互斥
pthread_mutex_t mid_AuxComTx;

/*****************************************************************************
 * 串口初始化函数
 ****************************************************************************/
void AUX_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  fd_auxcom = open(DEV_TTYS4_UART0, O_RDWR|O_NOCTTY); // open uart 
  if (fd_auxcom < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTYS4_UART0, fd_auxcom);
    close(fd_auxcom);
    fd_auxcom = -1;
    return ;
  }

  printf("fd_auxcom=%d\n", fd_auxcom);

  // configure uart
  tcgetattr(fd_auxcom, &options);
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
  tcflush(fd_auxcom, TCIFLUSH);
  tcsetattr(fd_auxcom, TCSANOW, &options);
}

//===================================================================================
uint8_t AUX_UartReceiveData(uint8_t **data, uint16_t* size)
{
  int retVal;

  if (fd_auxcom<0)
  {
    return FALSE;
  }

  retVal = read(fd_auxcom, aux_uart_rx_buffer, (AUX_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
  if (retVal > 0)
  {
    *data = dbg_uart_rx_buffer;
    *size = (uint16_t)retVal;
    
    return TRUE;
  }

  return FALSE;
}

//===================================================================================
uint16_t AUX_UartTransmitData(uint8_t *data,uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_auxcom<0))
  {
    return 0;
  }
 
  retVal = write(fd_auxcom, data, size); // 写串口数据
  if (retVal != size)
  {
    printf("ERROR Debug write ret=%d\n", retVal);
    close(fd_auxcom);
    AUX_UartInitialize(B115200);
    return 0;
  }

  return size;
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

/*************************************************************************
 *
*************************************************************************/
void AuxCom_Service(uint8_t *pdata, uint16_t len)
{
  uint16_t frame_len;
  
  if ((pdata[0]==0x7B)&&(pdata[1]==0x7B)&&(pdata[len-2]==0x7D)&&(pdata[len-1]==0x7D)) // 帧头(0x7B0x7B)和帧尾(0x7D0x7D)
  {
    if (len >= 17)
    {
      len -= 4; // 去掉帧头和帧尾
      memcpy(pc_debug_buffer, &pdata[2], len); // 得到M2M数据帧
      frame_len = PcDebug_DecodeM2mData(pc_debug_buffer, len); // 反转义

      m2m_context.rx_size = frame_len;  // 数据长度
      m2m_context.rx_data = pc_debug_buffer;  // 数据地址
      m2m_context.rx_from = SYSBUS_DEVICE_TYPE_PCDEBUG;
      frame_len = M2M_ProcessRecvMsg(&m2m_context); // 处理M2M数据

      if (pc_debug_buffer[0]==0x05)
      {
        if( (pc_debug_buffer[15]=='P'&&pc_debug_buffer[16]=='W') || (pc_debug_buffer[15]=='R'&&pc_debug_buffer[16]=='C') )
        {
          // A5_UART0_Write(pc_debug_buffer, frame_len);
        }
      }
    }
  }
}

/******************************************************************************
* 辅助UART通信
*******************************************************************************/
void* pthread_AuxCom(void *argument)
{
  uint16_t size; // 收到的数据长度
  uint8_t *pdata; // 数据起始地址

  while (1)
  {
    if (AuxCom_Receive(&pdata,&size)) // 等待数据(阻塞)
    {
      AuxCom_Service(pdata,size);
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceInit(void)
{
  //pthread_mutex_init(&mid_PcDebugPrintf, NULL);
  AUX_UartInitialize(AUX_COM_BUADRATE);
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_AUX_COM_ID], NULL, pthread_AuxCom, NULL);
  usleep(10);
}

//-----文件PcDebug.c结束---------------------------------------------

