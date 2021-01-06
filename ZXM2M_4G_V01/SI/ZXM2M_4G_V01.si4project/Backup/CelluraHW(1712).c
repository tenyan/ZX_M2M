/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-6-5
 * @brief:
 *******************************************************************************/
//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "CelluraHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart3ReceiveSem = NULL;  // 调试串口接收信号量
uint8_t usart3_dma_tx_buffer[USART3_TX_BUFFER_SIZE];

volatile uint16_t usart3_rx_size;
osSemaphoreId_t sid_Usart3TransmitSem = NULL;  // 调试串口发送信号量
uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
uint8_t usart3_dma_rx_buffer[USART3_RX_BUFFER_SIZE];


/*****************************************************************************
 * USART3的TX/RX在PB10(TXD)和PB11(RXD)
 ****************************************************************************/
void USART3_Initialize(uint32_t baudrate)
{

}


/* ================================================================== */
void USART3_TransmitData(uint8_t *data,uint16_t size)
{
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart3_dma_tx_buffer),data,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // 开启DMA发送
}

/* ================================================================== */
void USART3_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// 获取字符串字符数量
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // 等待信号量
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // 禁用DMA通道
  memcpy((uint8_t*)(usart3_dma_tx_buffer),pstr,size); // 拷贝数据
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // 修改DMA 发送数据数量
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // 开启DMA发送
}

//===================================================================
uint8_t USART3_ReceiveData(uint8_t **data, uint16_t* size)
{
  osStatus_t stat;

  stat = osSemaphoreAcquire(sid_Usart3ReceiveSem, osWaitForever);
  if (stat == osOK)
  {
    *data = usart3_rx_buffer;
    *size = usart3_rx_size;
    return 1;
  }

  return 0;
}


#define MODEM_DEV_TTY "/dev/smd8"


#define MODEM_UART_RX_BUFFER_SIZE  1500

volatile uint16_t modem_uart_rx_size;
uint8_t modem_uart_rx_buffer[MODEM_UART_RX_BUFFER_SIZE];
int fd_modem = -1;

/*************************************************************************
 *
*************************************************************************/
void Modem_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  // 打开失败，如何处理?
  fd_modem = open(MODEM_DEV_TTY, O_RDWR|O_NOCTTY);
  if (fd_modem < 0)
  {
    printf("ERROR open %s ret=%d\n\r", MODEM_DEV_TTY, fd_modem);
    return ;
  }
  printf("fd_modem=%d\n", fd_modem);

  tcgetattr(fd_modem, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VTIME] = 10; // read timeout 10*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate);
  cfsetospeed(&options, baudrate);
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(fd_modem, TCIFLUSH);
  tcsetattr(fd_modem, TCSANOW, &options);
}

//===================================================================================
int Modem_UartReceiveData(uint8_t **data, uint16_t* size)
{
  int ret = 0;

  if (fd_modem<0)
  {
    return FALSE;
  }

  ret = read(fd_modem, modem_uart_rx_buffer, (MODEM_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
  if (ret > 0)
  {
    *data = modem_uart_rx_buffer;
    *size = (uint16_t)ret;
    
    return TRUE;
  }

  return FALSE;
}


//===================================================================================
int Modem_UartTransmitData(uint8_t *data, uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_modem<0))
  {
    return 0;
  }

  retVal = write(fd_modem, data, size);  // 写串口数据
  if (retVal != size)
  {
    printf("ERROR Modem write ret=%d\n", retVal);
    close(fd_modem);
    Modem_UartInitialize(B115200);
    return 0;
  }
  
  PcDebug_SendData(data, size, DBG_MSG_TYPE_AT); // 调试信息

  return size;
}


//-----文件CelluraHW.c结束---------------------------------------------
