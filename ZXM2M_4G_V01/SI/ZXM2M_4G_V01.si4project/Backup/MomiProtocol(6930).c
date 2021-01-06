/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modemģ���Microͨ��Э��ʵ��
******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
//#include "MomiHW.h"
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
#define MOMI_TransmitData     USART3_TransmitData

#define MOMI_MAX_TXBUF_SIZE  1500
#define MODEM_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

#define momiMsgQueueENTER_CRITICAL() ()
#define momiMsgQueueEXIT_CRITICAL()  ()

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

// TLV״̬��Ч��־
extern bittype2 zxup_tlv_flag1;
extern bittype2 zxup_tlv_flag2;
extern bittype2 zxup_tlv_flag3;
extern bittype2 zxdown_tlv_flag1;
extern bittype2 zxdown_tlv_flag2;
extern bittype2 zxengine_tlv_flag;
extern bittype2 zxstatistics_tlv_flag;
extern bittype2 zxversion_tlv_flag;

uint8_t zxup_buffer[]; /// �ϳ����ݻ���
uint8_t zxdown_buffer[]; /// �³��������ݻ���
uint8_t zxengine_buffer[]; /// �³����������ݻ���
uint8_t zxstatistics_buffer[]; ///ͳ�����ݻ���
uint8_t zxversion_buffer[]; /// �汾��Ϣ����



/******************************************************************************
 * Macros
 ******************************************************************************/
#define MOMI_UART_RX_BUFFER_SIZE   1500

#define MOMI_BUADRATE         B115200
#define MOMI_BUFFER_MAX_SIZE  MOMI_UART_RX_BUFFER_SIZE
#define MOMI_Transmit          MOMI_UartTransmitData
#define MOMI_Receive           MOMI_UartReceiveData

#define MOMI_TRANSMIT_MUTEX_LOCK()     do{pthread_mutex_lock(&mid_MomiTransmit);}while(0)
#define MOMI_TRANSMIT_MUTEX_UNLOCK()   do{pthread_mutex_unlock(&mid_MomiTransmit);}while(0)

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t momi_uart_rx_buffer[DBG_UART_RX_BUFFER_SIZE];
int fd_momi = -1;

static uint8_t pc_debug_buffer[PC_DEBUG_BUFFER_MAX_SIZE]; // PcDebug�������ݻ���

// �ݹ黥��
pthread_mutex_t mid_MomiTransmit;


#define DEV_TTY_MCU_CAN "/dev/ttyHS0"

/*****************************************************************************
 * ���ڳ�ʼ������
 ****************************************************************************/
void MOMI_UartInitialize(void)
{

  struct termios options;

  pthread_mutex_init(&gMcuCanSendMutex, NULL);   //

  /* open uart */
  McuCan_fd = open(DEV_TTY_MCU_CAN, O_RDWR|O_NOCTTY);

  if (McuCan_fd < 0)
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY_MCU_CAN, McuCan_fd);
    close(McuCan_fd);
  }
  else
  {

  }
  printf("McuCan_fd: %d \n", McuCan_fd);

  /* configure uart */
  tcgetattr(McuCan_fd, &options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VTIME] = 1; // read timeout ��λ*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, B115200);//����������
  cfsetospeed(&options, B115200);//����������
  options.c_cflag |= (CLOCAL | CREAD);
  tcflush(McuCan_fd, TCIFLUSH);
  tcsetattr(McuCan_fd, TCSANOW, &options);
}

//===================================================================================
BOOL MOMI_UartReceiveData(uint8 **data, uint16* len)
{
  int ret;
  memset(MCU_Can_Recv_Buff,0,MCU_CAN_RECV_BUFF_LEN);
  if ((ret = read(McuCan_fd, MCU_Can_Recv_Buff, MCU_CAN_RECV_BUFF_LEN-1)) > 0)
  {
    *data = MCU_Can_Recv_Buff;
    *len = (uint16)ret;
    return TRUE;
  }
  else
    return FALSE;
}

//===================================================================================
uint16 MOMI_UartTransmitData(uint8_t *data, uint16_t size)
{
  int ret;

  if ((0==Len) || (NULL==data))
    return 0;

  /* write uart */
  pthread_mutex_lock(&mid_MomiTransmit);       //������
  ret = write(McuCan_fd, data, Len);
  pthread_mutex_unlock(&mid_MomiTransmit);     //����������

  if (ret != Len)
  {
    printf("ERROR MCU_CAN write ret=%d\n", ret);
    return 0;
  }
  else
    return Len;
}









/******************************************************************************
 * ��Ϣ���ж���
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x02
#define MOMI_RECV_COMMAND_ID     0x01
#define MOMI_MSG_QUEUE_MAX_SIZE  4096
#define ONE_MOMI_CMD_MAX_SIZE    1500
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

#if (PART("MOMI��������"))
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

/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI����TLV"))

//==�ն�����ɼ���Ϣ====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXINFO_BUFFER+2))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // ��Ч��־
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxinfo_tlv_flag.word = tlv_flag;

    memcpy(zxinfo_buffer, &pValue[pos], SIZE_OF_ZXINFO_BUFFER);
  }

  return retVal;
}

//==GPS�ɼ���Ϣ====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F002(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;
    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==���ػ��ϳ���Ϣ=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F010(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXUP_BUFFER+6))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // ��Ч��־1
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag1.word = tlv_flag;

    tlv_flag = pValue[pos++]; // ��Ч��־2
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag2.word = tlv_flag;

    tlv_flag = pValue[pos++]; // ��Ч��־3
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxup_tlv_flag3.word = tlv_flag;

    memcpy(zxup_buffer, &pValue[pos], SIZE_OF_ZXUP_BUFFER);
  }

  return retVal;
}

//==���ػ�������Ϣ=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F011(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXDOWN_BUFFER+4))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // ��Ч��־1
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxdown_tlv_flag1.word = tlv_flag;

    tlv_flag = pValue[pos++]; // ��Ч��־2
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxdown_tlv_flag2.word = tlv_flag;

    memcpy(zxdown_buffer, &pValue[pos], SIZE_OF_ZXDOWN_BUFFER);
  }

  return retVal;
}

//==���ػ��³���������Ϣ=================================================
static uint8_t iMomi_AnalyzeTlvMsg_F012(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER+2))
  {
    retVal = 1;
    
    tlv_flag = pValue[pos++]; // ��Ч��־
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxengine_tlv_flag.word = tlv_flag;

    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==��������Ϣ��Ϣ=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F020(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;

    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

//==ͳ������Ϣ��Ϣ=====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F030(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXSTATISTICS_BUFFER+2))
  {
    retVal = 1;
    tlv_flag = pValue[pos++]; // ��Ч��־
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxstatistics_tlv_flag.word = tlv_flag;
    
    memcpy(zxstatistics_buffer, &pValue[pos], SIZE_OF_ZXSTATISTICS_BUFFER);
  }

  return retVal;
}

//==����ECU�汾��Ϣ====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F040(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXVERSION_BUFFER+2))
  {
    retVal = 1;
    tlv_flag = pValue[pos++]; // ��Ч��־
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxversion_tlv_flag.word = tlv_flag;

    memcpy(zxversion_buffer, &pValue[pos], SIZE_OF_ZXVERSION_BUFFER);
  }

  return retVal;
}

//==CAN֡��Ϣ======================================================
static uint8_t iMomi_AnalyzeTlvMsg_F050(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint8_t can_num;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
    retVal = 1;
    can_num = pValue[pos++]; // ��Ч��־
    memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
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
#if 0
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[pos++] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pos += 2;             // ����(�����->У���)
  
  pdata[pos++] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[pos++] = 0x01;  // ��ˮ��
  pdata[pos++] = 0x01;

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
#endif
}

/*************************************************************************
 * 10ms����һ��
*************************************************************************/
void Momi_ProduceSendMsg(void)
{
  static uint8_t divide_for_300ms = 30;
  static uint8_t divide_for_500ms = 50;
  static uint8_t divide_for_1second = 100;

  // �ն���Ϣ(������300ms)
  if (divide_for_300ms)
  {
    divide_for_300ms--;
  }
  else
  {
    divide_for_300ms = 30;  // 300ms����
    //iMomi_SendColtMsg();
  }

  // ����������Ϣ(������500ms)
  if (divide_for_500ms)
  {
    divide_for_500ms--;
  }
  else
  {
    divide_for_500ms = 50;  // 500ms����
    //iMomi_SendVehicleMsg();
  }

  // GPSλ����Ϣ(������1s)
  if (divide_for_1second)
  {
    divide_for_1second--;
  }
  else
  {
    divide_for_1second = 100;  // 1s����
    //iMomi_SendGpsMsg();
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
  
  switch()
  {
    case 0xF001: // �ն�����ɼ���Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F001(pValue, len);
      break;
      
    case 0xF002: // GPS�ɼ���Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F002(pValue, len);
      break;

    case 0xF010: // ���ػ��ϳ���Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F010(pValue, len);
      break;
      
    case 0xF011: // ���ػ�������Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F011(pValue, len);
      break;
      
    case 0xF012: // ���ػ��³���������Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F012(pValue, len);
      break;
     
    case 0xF020: //��������Ϣ��Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F020(pValue, len);
      break;
      
    case 0xF030: // ͳ������Ϣ��Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F030(pValue, len);
      break;
      
    case 0xF040: // ����ECU�汾��Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F040(pValue, len);
      break;
      
    case 0xF050: // CAN֡��Ϣ
      retVal = iMomi_AnalyzeTlvMsg_F050(pValue, len);
      break;
  }

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
 * ���յ������ݷ������
*******************************************************************************/
void MOMI_ReceiveData(uint8_t *data, uint16_t size)
{  
  uint16_t it;
  
  for (it = 0; it < size; it++)
  {
    momi_msg_queue_push(data[it]); // ���յ������ݷ������
  }
}

/******************************************************************************
 * 4Gģ�鷢�ͱ��ĸ�ST��Ƭ��
*******************************************************************************/
void pthread_MomiProduce(void *argument)
{
  while (1)  // 100mִ��һ��
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/10); // 100msִ��һ��
    Momi_ProduceSendMsg();  // ����MOMI��Ϣ
  }
}

/******************************************************************************
 * ����ST��Ƭ������4G�ı���
*******************************************************************************/
void pthread_MomiProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;
  uint16_t cmd_size;��

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
  //Modem_GpioInitialize();
  //USART3_Initialize(MODEM_UART_BAUDRATE);
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_MOMI_PROCESS_ID], NULL, pthread_MomiProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MOMI_PRODUCE_ID], NULL, pthread_MomiProduce, NULL);
  usleep(10);
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

