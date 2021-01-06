/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebug.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2020-5-29
 * @brief     ���ļ�ΪPcDebug����ģ��Э��㴦����ļ�
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

static uint8_t pc_debug_buffer[PC_DEBUG_BUFFER_MAX_SIZE]; // PcDebug�������ݻ���

// �ݹ黥��
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
 * ���ڳ�ʼ������
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
  options.c_cc[VTIME] = 1; // read timeout ��λ*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate); // ����������
  cfsetospeed(&options, baudrate); // ����������
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

  retVal = read(fd_pcdebug, dbg_uart_rx_buffer, (DBG_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
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
 
  retVal = write(fd_pcdebug, data, size); // д��������
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
 * ����״״̬:
 * Bit3: 1=���SYS������Ϣ,0=�����
 * Bit2: 1=���CAN������Ϣ,0=�����
 * Bit1: 1=���GPS������Ϣ,0=�����
 * Bit0: 1=���Modem������Ϣ,0=�����
*****************************************************************************/
void PcDebug_SetStatus(uint8_t enable_type)
{
  enable_dbg_flag.byte = enable_type;
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

//--DMA��ʽ�����ַ���-----------------------------------------------------------
void PcDebug_SendString(const char *pstr)
{
  uint16_t size;

  size = strlen(pstr);	// ��ȡ�ַ����ַ�����
  PcDebug_Transmit((uint8_t*)pstr, size);
}

//uint16_t length;
//static char _dbg_TXBuff[256];
//--DMA��ʽ��printf-------------------------------------------------------------
void PcDebug_Printf(const char *format,...)
{
  uint16_t length;
  char _dbg_TXBuff[256];

  PRINTF_MUTEX_LOCK(); // ����

  va_list args;

  va_start(args, format);
  length = vsnprintf((char*)_dbg_TXBuff, sizeof(_dbg_TXBuff), (char*)format, args);
  va_end(args);
  
  PcDebug_Transmit((uint8_t*)_dbg_TXBuff, length); // DMA
  PRINTF_MUTEX_UNLOCK();  // ����
}

/**********************************************************************************
 * ����: �����ݰ�����ת�� 0x7B7B-->ESCESC��0x7B7B; 0x7D7D-->ESCESC��0x7D7D
 * ����: src=��ת������ݰ�
 * ���: des=ת�������ݰ�
 * ����: ת�������ݰ�����
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
    if ((0x7B==*pSrc) && (0x7B==*(pSrc+1))) // 0x7B7B-->ESCESC��0x7B7B
    {
      *pDes++ = 0x1B;
      *pDes++ = 0x1B;
      *pDes++ = 0x7B;
      *pDes++ = 0x7B;
      pSrc += 2;
      desLen += 2;
    }
    else if ((0x7D==*pSrc)&&(0x7D==*(pSrc+1)) ) // 0x7D7D-->ESCESC��0x7D7D
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
 * ����M2MЭ��������Ӧ
*******************************************************************************/
void PcDebug_SendM2mRsp(uint8_t *pdata, uint16_t size)
{
  uint16_t len = 0;
  uint8_t* pbuf = public_buffer;

  len = PcDebug_EncodeM2mData(pdata, size); // ��ת��
  pbuf[0] = 0x7B;     // ��ͷ
  pbuf[1] = 0x7B;
  memcpy(&pbuf[2], pdata,len);
  len += 2;
  pbuf[len++] = 0x7D; // ��β
  pbuf[len++] = 0x7D;

  PcDebug_Transmit(pbuf, len);
}

/*****************************************************************************
 * ����: ��ת�� ESCESC0x7B7B-->0x7B7B; ESCESC0x7D7D-->7D7D
 * ����: Ptr,��Ҫ����ת������ָ���ָ���ַ  
         len ��Ҫת������ݵĳ���
 * ���: ת���ĳ���
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
  
  if ((pdata[0]==0x7B)&&(pdata[1]==0x7B)&&(pdata[len-2]==0x7D)&&(pdata[len-1]==0x7D)) // ֡ͷ(0x7B0x7B)��֡β(0x7D0x7D)
  {
    if (len >= 17)
    {
      len -= 4; // ȥ��֡ͷ��֡β
      memcpy(pc_debug_buffer, &pdata[2], len); // �õ�M2M����֡
      
      frame_len = PcDebug_DecodeM2mData(pc_debug_buffer, len); // ��ת��
      UNUSED(frame_len);
      
      // frame_len;  // �������ݳ���
      // pc_debug_buffer; // ��ȡ���ݵ�ַ
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
* ��PC��ͨ�ŵĵ�������
*******************************************************************************/
void* pthread_PcDebug(void *argument)
{
  uint16_t size; // �յ������ݳ���
  uint8_t *pdata; // ������ʼ��ַ

  while (1)
  {
    if (PcDebug_Receive(&pdata,&size)) // �ȴ�����(����)
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

//-----�ļ�PcDebug.c����---------------------------------------------

