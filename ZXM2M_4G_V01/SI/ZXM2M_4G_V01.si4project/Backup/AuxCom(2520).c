/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    ���ļ�Ϊ����ͨ��ģ���c�ļ�
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

static uint8_t aux_com_buffer[AUX_COM_BUFFER_MAX_SIZE]; // PcDebug�������ݻ���

// �ݹ黥��
pthread_mutex_t mid_AuxComTx;

/*****************************************************************************
 * ���ڳ�ʼ������
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
  options.c_cc[VTIME] = 1; // read timeout ��λ*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate); // ����������
  cfsetospeed(&options, baudrate); // ����������
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

  retVal = read(fd_auxcom, aux_uart_rx_buffer, (AUX_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
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
 
  retVal = write(fd_auxcom, data, size); // д��������
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
 * ����: �������������Ϣ�ĺ���,���Ա���ͬģ�����
 * ����: pdata:ָ�룬ָ�����������ݵ�ָ��;
         size:Ҫ������ݵĳ���;
         msg_types:��ǰ�õ��Ժ������ĸ�ģ��������,ģ��ı��
         1=GSMģ��;2=GPSģ��;3=MCUģ��;4=���������������
 * ���: ��
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

  if (msg_types == DBG_MSG_TYPE_ANYDATA) //ֱ���������
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
  
  if ((pdata[0]==0x7B)&&(pdata[1]==0x7B)&&(pdata[len-2]==0x7D)&&(pdata[len-1]==0x7D)) // ֡ͷ(0x7B0x7B)��֡β(0x7D0x7D)
  {
    if (len >= 17)
    {
      len -= 4; // ȥ��֡ͷ��֡β
      memcpy(pc_debug_buffer, &pdata[2], len); // �õ�M2M����֡
      frame_len = PcDebug_DecodeM2mData(pc_debug_buffer, len); // ��ת��

      m2m_context.rx_size = frame_len;  // ���ݳ���
      m2m_context.rx_data = pc_debug_buffer;  // ���ݵ�ַ
      m2m_context.rx_from = SYSBUS_DEVICE_TYPE_PCDEBUG;
      frame_len = M2M_ProcessRecvMsg(&m2m_context); // ����M2M����

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
* ����UARTͨ��
*******************************************************************************/
void* pthread_AuxCom(void *argument)
{
  uint16_t size; // �յ������ݳ���
  uint8_t *pdata; // ������ʼ��ַ

  while (1)
  {
    if (AuxCom_Receive(&pdata,&size)) // �ȴ�����(����)
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

//-----�ļ�PcDebug.c����---------------------------------------------

