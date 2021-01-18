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

/*************************************************************************
 * 10ms调用一次(固件服务器FSRV = Firmware Server)
*************************************************************************/
void FSRV_ProduceSendMsg(void)
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


/******************************************************************************
 * 辅助UART通信--发送报文给ST单片机
*******************************************************************************/
void* pthread_AuxComProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms执行一次
    FSRV_ProduceSendMsg();  // 发送升级消息
  }
}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void FSRV_ProcessRecvMsg(uint8_t *pdata, uint16_t size)
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


/******************************************************************************
* 辅助UART通信--处理ST单片机发给4G的报文
*******************************************************************************/
void* pthread_AuxComProcess(void *argument)
{
  uint16_t size; // 收到的数据长度
  uint8_t *pdata; // 数据起始地址

  while (1)
  {
    if (AuxCom_Receive(&pdata,&size)) // 等待数据(阻塞)
    {
      FSRV_ProcessRecvMsg(pdata, size);
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
  pthread_create(&pthreads[PTHREAD_AUX_COM_PROCESS_ID], NULL, pthread_AuxComProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_AUX_COM_PRODUCE_ID], NULL, pthread_AuxComProduce, NULL);
  usleep(10);

}

//-----文件PcDebug.c结束---------------------------------------------

