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


#define DEV_TTY "/dev/smd8"

//-----外部变量定义------------------------------------------------------------
#define GSM_UART_Rx_BSIZE        1500

uint8 GSM_UART_Rx_Buff[GSM_UART_Rx_BSIZE];
uint16 GSM_UART_Rx_Cnt;
int32 fd_modem = -1;

/*************************************************************************
 *
*************************************************************************/
int32 OpenGsmAT_DEV_TTY(void)
void Modem_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  //打开失败，如何处理?
  fd_modem = open(DEV_TTY, O_RDWR|O_NOCTTY);
  if (fd_modem < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY, fd_modem);
    return fd_modem;
  }
  printf("fd_modem: %d \n", fd_modem);

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
  return fd_modem;
}


//-----文件CelluraHW.c结束---------------------------------------------
