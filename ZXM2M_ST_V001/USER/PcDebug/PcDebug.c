/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: PcDebug.c
 * @Engineer: TenYan
 * @Company:  �칤��Ϣ����Ӳ����
 * @version:  V1.0
 * @Date:     2020-5-29
 * @brief:    ���ļ�ΪPcDebug����ģ��Э��㴦����ļ�
 ********************************************************************************/
#include "PcDebugHW.h"
#include "config.h"

//extern STUSystem g_stuSystem;

static uint8_t public_buffer[1200];

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define PC_DEBUG_BUADRATE         115200
#define PC_DEBUG_BUFFER_MAX_SIZE  USART1_RX_BUFFER_SIZE
#define PcDebug_Transmit          USART1_TransmitData
#define PcDebug_Receive           USART1_ReceiveData

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
#define AppThreadPriority_PcDebug	  osPriorityNormal5
osThreadId_t tid_PcDebug;

const osThreadAttr_t AppThreadAttr_PcDebug =
{
  .priority = AppThreadPriority_PcDebug,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // �ֽ�
};

static uint8_t pc_debug_buffer[UART0_RECV_BUFF_LENGTH]; // PcDebug�������ݻ���

// �ݹ黥��
osMutexId_t mid_PcDebugPrintf;

static bittype enable_dbg_flag = {.byte = 0x00,};
#define ENABLE_DBG_TYPE_MODEM_FLAG  enable_dbg_flag.b.bit0
#define ENABLE_DBG_TYPE_GPS_FLAG    enable_dbg_flag.b.bit1
#define ENABLE_DBG_TYPE_CAN_FLAG    enable_dbg_flag.b.bit2
#define ENABLE_DBG_TYPE_SYS_FLAG    enable_dbg_flag.b.bit3
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit4
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit5
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit6
//#define ENABLE_DBG_TYPE_  enable_dbg_flag.b.bit7

/*****************************************************************************
 * ����Printf�ݹ黥��
 ****************************************************************************/
void PcDebugPrintf_CreatMutex(void)
{
  const osMutexAttr_t attr =
  {
    .attr_bits = osMutexRecursive
  };
    
  mid_PcDebugPrintf = osMutexNew(&attr);
}

#define PRINTF_MUTEX_LOCK()     do{osMutexAcquire(mid_PcDebugPrintf, osWaitForever);}while(0)
#define PRINTF_MUTEX_UNLOCK()   do{osMutexRelease(mid_PcDebugPrintf);}while(0)

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

uint16_t length;
static char _dbg_TXBuff[256];
//--DMA��ʽ��printf-------------------------------------------------------------
void PcDebug_Printf(const char *format,...)
{
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
      SbusMsg_PcDebug.data_size = frame_len;  // �������ݳ���
      SbusMsg_PcDebug.data = pc_debug_buffer; // ��ȡ���ݵ�ַ
      SYSBUS_PutMbox(SbusMsg_PcDebug);        // ������Ϣ
    }
  }
}

/******************************************************************************
* ��PC��ͨ�ŵĵ�������
*******************************************************************************/
void AppThread_PcDebug(void *argument)
{
  uint16_t size;
  uint8_t *pdata;

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
  USART1_Initialize(PC_DEBUG_BUADRATE);
}

/*************************************************************************
 *
*************************************************************************/
void PcDebug_ServiceStart(void)
{
  tid_PcDebug = osThreadNew(AppThread_PcDebug, NULL, &AppThreadAttr_PcDebug);
}

//-----�ļ�PcDebug.c����---------------------------------------------
