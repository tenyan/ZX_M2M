/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modemģ���Microͨ��Э��ʵ��
******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "MomiHW.h"
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
#define MOMI_ReceiveData      USART3_ReceiveData
#define MOMI_TransmitData     USART3_TransmitData

#define MOMI_MAX_TXBUF_SIZE  USART3_TX_BUFFER_SIZE
#define MOMI_MAX_RXBUF_SIZE  USART3_RX_BUFFER_SIZE
#define momi_rx_buffer       usart3_rx_buffer
#define momi_rx_size          usart3_rx_size

#define MODEM_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

#define momiMsgQueueENTER_CRITICAL() USART_ITConfig(USART3, USART_IT_IDLE, DISABLE) // �رմ��ڿ��н����ж�
#define momiMsgQueueEXIT_CRITICAL()  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE) // �������ڿ��н����ж�

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
modem_state_t modem_state;

extern volatile uint16_t usart3_rx_size;
extern uint8_t usart3_rx_buffer[USART3_RX_BUFFER_SIZE];
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

/******************************************************************************
 * RTOS���
 ******************************************************************************/
//=������==============================================================
#define AppThreadPriority_ModemProcess   osPriorityHigh2
osThreadId_t tid_ModemProcess;

const osThreadAttr_t AppThreadAttr_ModemProcess =
{
  .priority = AppThreadPriority_ModemProcess,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // �ֽ�
};

 //=������==============================================================
#define AppThreadPriority_ModemProduce   osPriorityHigh1
osThreadId_t tid_ModemProduce;

const osThreadAttr_t AppThreadAttr_ModemProduce =
{
  .priority = AppThreadPriority_ModemProduce,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // �ֽ�
};

/******************************************************************************
 * ��Ϣ���ж���
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x01
#define MOMI_RECV_COMMAND_ID     0x02
#define MOMI_MSG_QUEUE_MAX_SIZE  1024
#define ONE_MOMI_CMD_MAX_SIZE    256
#define MOMI_MSG_HEAD            0x7E      // ֡ͷ
#define MOMI_MSG_TAIL            0xFFFCFFFF  // ֡β

// ��Ϣ���ж���
typedef struct
{
  uint16_t head;  // ����ͷ
  uint16_t tail;  // ����β
  uint8_t data[MOMI_MSG_QUEUE_MAX_SIZE];  // ���ݻ�����
}momi_msg_queue_t;

static momi_msg_queue_t moni_msg_queue = {0,0,0};
static uint32_t momi_msg_state = 0;
static uint16_t momi_msg_pos = 0;
uint8_t moni_msg_buffer[ONE_MOMI_CMD_MAX_SIZE];

#if (PART("MOMI��Ϣ����"))
//==��ն�������==============================================================
void momi_msg_queue_reset()
{
  moni_msg_queue.head = 0;
  moni_msg_queue.tail = 0;
  momi_msg_state = 0;
  momi_msg_pos = 0;
}

//==���յ������ݷ��������=====================================================
void momi_msg_queue_push(uint8_t dat)
{
  uint16_t pos;

  pos = (moni_msg_queue.head + 1) % MOMI_MSG_QUEUE_MAX_SIZE;

  if(pos != moni_msg_queue.tail) // ����״̬
  {
    moni_msg_queue.data[moni_msg_queue.head] = dat;
    moni_msg_queue.head = pos;
  }
}

//==�Ӷ�����ȡһ������=========================================================
static void momi_msg_queue_pop(uint8_t* p_data)
{
  if(moni_msg_queue.tail != moni_msg_queue.head) //�ǿ�״̬
  {
    *p_data = moni_msg_queue.data[moni_msg_queue.tail];
    moni_msg_queue.tail = (moni_msg_queue.tail + 1) % MOMI_MSG_QUEUE_MAX_SIZE;
  }
}

//==��ȡ��������Ч���ݸ���=====================================================
static uint16_t momi_msg_queue_size(void)
{
  return ((moni_msg_queue.head + MOMI_MSG_QUEUE_MAX_SIZE - moni_msg_queue.tail) % MOMI_MSG_QUEUE_MAX_SIZE);
}

/******************************************************************************
*\brief  �Ӷ�����ȡ��һ����������Ϣ
*\param  pbuffer ָ����ջ�����
*\param  size ָ����ջ�������С
*\return  ָ��ȣ�0��ʾ������������ָ��
******************************************************************************/
uint16_t momi_msg_queue_find(uint8_t *buffer, uint16_t size)
{
  uint16_t cmd_size = 0;
  uint8_t _data = 0;

  while(momi_msg_queue_size() > 0)
  {
    momiMsgQueueENTER_CRITICAL();
    momi_msg_queue_pop(&_data);  //ȡһ������
    momiMsgQueueEXIT_CRITICAL();

    if((momi_msg_pos == 0)&& (_data != MOMI_MSG_HEAD)) //ָ���һ���ֽڱ�����֡ͷ,��������
    {
      continue;
    }
    
    if(momi_msg_pos < size) // ��ֹ���������
    {
      buffer[momi_msg_pos++] = _data;
    }
    
    momi_msg_state = ((momi_msg_state << 8) | _data); // ƴ�����4���ֽ�,���һ��32λ����
    if(momi_msg_state == MOMI_MSG_TAIL)  //���4���ֽ���֡βƥ��,�õ�����֡
    {
      cmd_size = momi_msg_pos;  //ָ���ֽڳ���
      momi_msg_state = 0;       //���¼��֡β��
      momi_msg_pos = 0;         //��λָ��ָ��
      return cmd_size;
    }
  }
	
  return 0; // û����������֡
}
#endif

#if (PART("MODEM���ػ�����"))
/******************************************************************************
* 
******************************************************************************/
modem_state_t Modem_GetState(void)
{
  return modem_state;
}

void Modem_SetState(modem_state_t state)
{
  modem_state = state;
}

/******************************************************************************
 * ����: ʹ4Gģ��ػ�,���ϵ�20��
*******************************************************************************/
void Modem_PowerOff(void)
{
  uint8_t i;
  uint8_t j;
  uint8_t flag = 0;

  for (i=0; i<3; i++)
  {
    PcDebug_SendString("Modem PowerOff\n");

    MODEM_PWRKEY_H(); // ����1s
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_L(); // ����4s
    MODEM_DELAY_MS(OS_TICKS_PER_SEC*4);

    MODEM_PWRKEY_H(); //����
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    for (j=0; j<30; j++)
    {
      MODEM_DELAY_MS(OS_TICKS_PER_SEC);
      if (0==MODEM_STATUS_IN()) // �͵�ƽ: ģ��Power off
      {
        flag = 1;
        break;
      }
    }

    if (1==flag)
    {
      break;
    }
  }
  DISABLE_MODEM_PWR();	// �ر�ģ�鹩��
  MODEM_DELAY_MS(OS_TICKS_PER_SEC*2); // �ȴ�2s
}

/******************************************************************************
 * ����: ʹ4Gģ�鿪��
*******************************************************************************/
void Modem_PowerOn(void)
{
  uint8_t i;
  uint8_t j;
  uint8_t flag = 0;
  uint32_t status = 0;

  MODEM_RESET_H(); // �ͷ�ģ�鸴λ
  MODEM_DTR_L(); // ʹģ���˳�����ģʽ
  ENABLE_MODEM_PWR();  // ��ģ�鹩��
  MODEM_DELAY_MS(OS_TICKS_PER_SEC*2); // ��ʱ2s

  for (i=0; i<3; i++)
  {
    PcDebug_SendString("Modem PowerOn\n");

    MODEM_PWRKEY_H(); // ����1S
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_L(); // ����1S
    MODEM_DELAY_MS(OS_TICKS_PER_SEC);

    MODEM_PWRKEY_H(); // ����
    MODEM_DELAY_MS(OS_TICKS_PER_SEC*3);

    for (j=0; j<15; j++) // �ȴ�15s
    {
      MODEM_DELAY_MS(OS_TICKS_PER_SEC);
      status = MODEM_STATUS_IN();
      if (0 != status) // �ߵ�ƽ: ģ��Power on and firmware ready
      {
        flag = 1;
        break;
      }
    }

    if (1==flag)
    {
      break;
    }
  }
}

/******************************************************************************
 * ģ��״̬��(10ms����һ��)
*******************************************************************************/
void Modem_StateManageService(void)               /// ����
{
  uint8_t imodem_state;

  imodem_state = Modem_GetState();
  switch (imodem_state)
  {
  case MODEM_STATE_OFF:  // �ػ���,�޲���
    break;
    
  case MODEM_STATE_POWER_OFF:  // ģ��ػ�
    Modem_PowerOff();
    Modem_SetState(MODEM_STATE_OFF);
    PcDebug_SendString("Modem Off!\n");
    break;

  case MODEM_STATE_POWER_ON:  // ģ�鿪��
    Modem_PowerOn();
    Modem_SetState(MODEM_STATE_DATA_READY); // ת����ģʽ
    break;

  case MODEM_STATE_REPOWER_OFF:
    Modem_PowerOff();
    Modem_SetState(MODEM_STATE_SILENCE);
    break;
  
  case MODEM_STATE_DATA_READY:  // ģ���ͨ��
    break; 

  case MODEM_STATE_SILENCE:  // ģ�龲Ĭ5����
    PcDebug_SendString("Modem Rest 10min\n");
    MODEM_DELAY_MS((300*OS_TICKS_PER_SEC));
    Modem_SetState(MODEM_STATE_POWER_ON); // ���¿���
    break;

  default:
    Modem_SetState(MODEM_STATE_REPOWER_OFF);
    break;
  }
}
#endif

/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI����TLV"))

//==�ն�����ɼ���Ϣ====================================================
static uint16_t iMomi_BuildTlvMsg_F001(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x01;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==GPS�ɼ���Ϣ====================================================
static uint16_t iMomi_BuildTlvMsg_F002(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x02;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==���ػ��ϳ���Ϣ=====================================================
static uint16_t iMomi_BuildTlvMsg_F010(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXUP_BUFFER+6;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x10;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxup_tlv_flag1.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxup_tlv_flag1.word & 0xFF;
  pbuf[len++] = (zxup_tlv_flag2.word>>8) & 0xFF;
  pbuf[len++] = zxup_tlv_flag2.word & 0xFF;
  pbuf[len++] = (zxup_tlv_flag3.word>>8) & 0xFF;
  pbuf[len++] = zxup_tlv_flag3.word & 0xFF;
  memcpy(&pbuf[len], zxup_buffer, SIZE_OF_ZXUP_BUFFER);
  len += SIZE_OF_ZXUP_BUFFER;

  return len;
}

//==���ػ�������Ϣ=====================================================
static uint16_t iMomi_BuildTlvMsg_F011(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXDOWN_BUFFER+4;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x11;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxdown_tlv_flag1.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxdown_tlv_flag1.word & 0xFF;
  pbuf[len++] = (zxdown_tlv_flag2.word>>8) & 0xFF;
  pbuf[len++] = zxdown_tlv_flag2.word & 0xFF;
  memcpy(&pbuf[len], zxdown_buffer, SIZE_OF_ZXDOWN_BUFFER);
  len += SIZE_OF_ZXDOWN_BUFFER;

  return len;
}

//==���ػ��³���������Ϣ=================================================
static uint16_t iMomi_BuildTlvMsg_F012(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x12;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==��������Ϣ��Ϣ=====================================================
static uint16_t iMomi_BuildTlvMsg_F020(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x20;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==ͳ������Ϣ��Ϣ=====================================================
static uint16_t iMomi_BuildTlvMsg_F030(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x30;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==����ECU�汾��Ϣ====================================================
static uint16_t iMomi_BuildTlvMsg_F040(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x40;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

//==CAN֡��Ϣ======================================================
static uint16_t iMomi_BuildTlvMsg_F050(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_ZXENGINE_BUFFER+2;

  pbuf[len++] = 0xF0; // TAG
  pbuf[len++] = 0x50;
  
  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;
  
  pbuf[len++] = (zxengine_tlv_flag.word>>8) & 0xFF; // VALUE
  pbuf[len++] = zxengine_tlv_flag.word & 0xFF;
  memcpy(&pbuf[len], zxengine_buffer, SIZE_OF_ZXENGINE_BUFFER);
  len += SIZE_OF_ZXENGINE_BUFFER;

  return len;
}

#endif


//////////////////////////////////////////////////////////////////////////////////////////////
// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI�����ͷ���Э������"))
//==����У���================================================================
uint8_t iMomi_CalcSumCheck(uint8_t* pbuf,uint16 len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  return sum;
}

//==�����ն˲ɼ���Ϣ===========================================================
void iMomi_SendColtMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����
  msg_len = iMomi_BuildTlvMsg_F001(&pdata[pos]); // �ն�����ɼ���Ϣ
  pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==����GPSλ����Ϣ============================================================
void iMomi_SendGpsMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F002(&pdata[pos]); // GPS�ɼ���Ϣ
  pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==���ͳ���������Ϣ===========================================================
void iMomi_SendVehicleMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x03;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F010(&pdata[pos]); // ���ػ��ϳ���Ϣ
  pos += msg_len;

  msg_len = iMomi_BuildTlvMsg_F011(&pdata[pos]); // ���ػ�������Ϣ
  pos += msg_len;

  msg_len = iMomi_BuildTlvMsg_F012(&pdata[pos]); // ���ػ��³���������Ϣ
  pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==����DTC������Ϣ============================================================
void iMomi_SendDtcMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F020(&pdata[pos]); // ��������Ϣ��Ϣ
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F021(&pdata[pos]); // ��������Ϣ��Ϣ
  //pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F022(&pdata[pos]); // ��������Ϣ��Ϣ
  //pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==����ͳ������Ϣ=============================================================
void iMomi_SendStatisticsMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F030(&pdata[pos]); // ���ػ��ϳ���Ϣ
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F031(&pdata[pos]); // ���ػ��ϳ���Ϣ
  //pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==���Ͱ汾��Ϣ===============================================================
void iMomi_SendVersionMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F040(&pdata[pos]); // �ϳ�ͳ������Ϣ��Ϣ
  pos += msg_len;

  //msg_len = iMomi_BuildTlvMsg_F041(&pdata[pos]); // �³�ͳ������Ϣ��Ϣ
  //pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

//==����CAN֡��Ϣ==============================================================
void iMomi_SendCanMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;
  pdata[pos++] = 0x01;  // TLV����

  msg_len = iMomi_BuildTlvMsg_F050(&pdata[pos]); // CAN֡��Ϣ
  pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[2] = (msg_len>>8) & 0xFF; // ����
  pdata[3] = msg_len & 0xFF;
  
  pdata[pos++] = (MOMI_MSG_TAIL>>24) & 0xFF; // ֡β(4B)
  pdata[pos++] = (MOMI_MSG_TAIL>>16) & 0xFF;
  pdata[pos++] = (MOMI_MSG_TAIL>>8) & 0xFF;
  pdata[pos++] = MOMI_MSG_TAIL & 0xFF;

  MOMI_TransmitData(pdata, pos);
}

/*************************************************************************
 * 10ms����һ��
*************************************************************************/
void Momi_ProduceSendMsg(void)
{
  static uint8_t divide_for_300ms = 30;
  static uint8_t divide_for_500ms = 50;
  static uint8_t divide_for_1second = 100;

  if(Modem_GetState()==MODEM_STATE_DATA_READY) // ģ�鴦���ѿ���״̬
  {
    // �ն���Ϣ(������300ms)
    if(divide_for_300ms)
    {  divide_for_300ms--;}
    else
    {
      divide_for_300ms = 30;  // 300ms����
      iMomi_SendColtMsg();
    }

    // ����������Ϣ(������500ms)
    if(divide_for_500ms)
    {  divide_for_500ms--;}
    else
    {
      divide_for_500ms = 50;  // 500ms����
      iMomi_SendVehicleMsg();
    }

    // GPSλ����Ϣ(������1s)
    if(divide_for_1second)
    {  divide_for_1second--;}
    else
    {
      divide_for_1second = 100;  // 1s����
      //iMomi_SendGpsMsg();
    }

    // ������Ϣ(�¼���)
    // iMomi_SendDtcMsg();
    
    // CAN֡��Ϣ(�¼���)
    //iMomi_SendCanMsg();

    // �汾��Ϣ(�ϵ緢һ��)

    //iMomi_SendVersionMsg();

    // ͳ������Ϣ(ACC�ط���һ��)
    //iMomi_SendStatisticsMsg();
  }
  else
  {
    divide_for_300ms = 30;
    divide_for_500ms = 50;
    divide_for_1second = 100;
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI���պͽ���Э������"))
//==�ж�MOMI��ϢУ���==========================================================
uint8_t Momi_CheckMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;

  if(pdata[3] != MOMI_RECV_COMMAND_ID) // �����ֶδ���
  {
    return MOMI_NOK;
  }

  msg_len = size-8;
  received_check_sum = pdata[size-5]; // �����
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[5], msg_len); // ����У��ֵ:������ſ�ʼ����
  if(received_check_sum==calculated_check_sum)  //check if crc's match
  {
    return MOMI_OK;
  }
  else
  {
    return MOMI_NOK;
  }
}

//==�����յ���TLV����=============================================================
uint16_t iMomi_AnalyzeTlvData(uint16_t tag, uint16_t len, uint8_t *pValue)
{
  uint16_t retVal = 0;
  //uint16_t it = 0;
  //im2m_CmdTlv_t* p_CmdTlvDeal = NULL;

  //for (it = 0; it<NUM_OF_M2M_CMD_DEAL; it++)
  //{
  //  p_CmdTlvDeal = &m2m_CmdDealTbl[it];
  //  if (p_CmdTlvDeal->type != tag)
  //  {
  //    continue;
  //  }

  //  if (p_CmdTlvDeal->pfun_analyze != NULL)
  //  {
  //    retVal = p_CmdTlvDeal->pfun_analyze(pValue, len);
  //  }

  //  break;
  //}

  return retVal;
}


/******************************************************************************
 * ����: �����յ�4Gģ����Ϣ
 * ����: pdata-���ݰ�ָ��;  size-���ݰ�����
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
  uint8_t* pbuf = &pdata[6];
  uint8_t fail_tag_num = 0;

  msg_status = Momi_CheckMsg(pdata, size);
  if(msg_status==MOMI_OK)
  {
    msg_len = pdata[1];  // ���ݳ���
    msg_len <<= 8;
    msg_len += pdata[2];
    if(msg_len < 5)  // ������
    {
      return;
    }
    msg_len -= 4;  // ���ݰ��ĳ���(������������(1B)+��ˮ��(2B)+У��(1B))
    msg_sn = pdata[4];  // ��Ϣ��ˮ��
    msg_sn <<= 8;
    msg_sn += pdata[5];
    
    tlvNum = pbuf[pos++];  // TLV����
    while (tlvNum) // TLV����
    {
      if (pos >= msg_len) // �����ж�
      {
        break; // ���ȴ���,�˳�ѭ��
      }
      
      //==��ȡTAG==================================
      tag = pbuf[pos++];
      tag <<= 8;
      tag += pbuf[pos++];

      //==��ȡLENGTH===============================
      length = pbuf[pos++];
      length <<= 8;
      length += pbuf[pos++];

      //==��ȡVALUE================================
      retVal = iMomi_AnalyzeTlvData(tag, length, &pbuf[pos]); // ����
      if(MOMI_NOK==retVal)
      {
        fail_tag_num++;

        // ���������Ϣ
      }
      pos += length; // ָ����һ��TLV
      tlvNum--;
    }
  }
}
#endif

/******************************************************************************
 * 4Gģ��������,�����ģ������в���
*******************************************************************************/
void AppTask_ModemProduce(void *argument)
{
  while (1)  // 10mִ��һ��
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/100); // 10msִ��һ��
    Modem_StateManageService();  // modem����
    Momi_ProduceSendMsg();  // ����MOMI��Ϣ
  }
}

/******************************************************************************
 * CAT1ģ�����ݽ��ս���
*******************************************************************************/
void AppTask_ModemProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;
  uint16_t cmd_size;

  while (1)
  {
    if (MOMI_ReceiveData(&pdata, &len)) // �ȴ��ź���(����)
    {
      while(momi_msg_queue_size()>0)
      {
        cmd_size = momi_msg_queue_find(moni_msg_buffer, ONE_MOMI_CMD_MAX_SIZE); // ����MONI��Ϣ
        if(cmd_size>0) // �õ�һ��������MOMI��Ϣ
        {
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
  Modem_GpioInitialize();
  USART3_Initialize(MODEM_UART_BAUDRATE);
  Modem_SetState(MODEM_STATE_POWER_OFF);
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceStart(void)
{
  tid_ModemProcess = osThreadNew(AppTask_ModemProcess, NULL, &AppThreadAttr_ModemProcess);  
  tid_ModemProduce = osThreadNew(AppTask_ModemProduce, NULL, &AppThreadAttr_ModemProduce);
}

#if 0
//==type���յ���˳����д=================================================
iMomi_CmdTlv_t Momi_CmdDealTbl[]=
{
  /*TLV type ,build hdl ,analyze hdl*/
  {0x0000, im2m_BuildTlvMsg_0000, im2m_AnalyzeTlvMsg_0000},/*�豸�ն˱��*/
  {0x0001, im2m_BuildTlvMsg_0001, NULL},                   /*�ն��豸����汾��ţ�ֻ��*/
  {0x1002,                  NULL, im2m_AnalyzeTlvMsg_1002},//����������IP
};
#define NUM_OF_MOMI_CMD_DEAL   (sizeof(Momi_CmdDealTbl)/sizeof(Momi_CmdDealTbl))
#endif

//-----�ļ�CelluraCore.c����---------------------------------------------

