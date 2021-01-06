/*******************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: CelluraHW.c
 * @Engineer: TenYan
 * @version:  V1.0
 * @Date:     2020-6-5
 * @brief:
 *******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "CelluraHW.h"

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
osSemaphoreId_t sid_Usart3ReceiveSem = NULL;  // ���Դ��ڽ����ź���
uint8_t usart3_dma_tx_buffer[USART3_TX_BUFFER_SIZE];

volatile uint16_t usart3_rx_size;
osSemaphoreId_t sid_Usart3TransmitSem = NULL;  // ���Դ��ڷ����ź���
uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
uint8_t usart3_dma_rx_buffer[USART3_RX_BUFFER_SIZE];


/*****************************************************************************
 * USART3��TX/RX��PB10(TXD)��PB11(RXD)
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
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart3_dma_tx_buffer),data,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // ����DMA����
}

/* ================================================================== */
void USART3_TransmitString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// ��ȡ�ַ����ַ�����
  if ((size==0)||(size>USART3_TX_BUFFER_SIZE))
  {
    return;
  }
  osSemaphoreAcquire(sid_Usart3TransmitSem, osWaitForever); // �ȴ��ź���
  DMA_Cmd(DMA1_Stream3, DISABLE);                     // ����DMAͨ��
  memcpy((uint8_t*)(usart3_dma_tx_buffer),pstr,size); // ��������
  DMA_SetCurrDataCounter(DMA1_Stream3, size);         // �޸�DMA ������������
  DMA_Cmd(DMA1_Stream3, ENABLE);                      // ����DMA����
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

//-----�ⲿ��������------------------------------------------------------------
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

  //��ʧ�ܣ���δ���?
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


//-----�ļ�CelluraHW.c����---------------------------------------------
