/*****************************************************************************
* @FileName: MomiProtocol.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-12-10
* @brief     Modemģ���Microͨ��Э��ʵ��
* TLV˵��:
*  TAGF001--�ն�����ɼ���Ϣ(yes)
*  TAGF002--GPS��λ��Ϣ(no)
*  TAGF010--�ϳ�������Ϣ(yes)
*  TAGF011--�³�������Ϣ(yes)
*  TAGF012--�³���������Ϣ(yes)
*  TAGF020--������Ϣ(no)
*  TAGF030--ͳ����Ϣ(yes)
*  TAGF040--�汾��Ϣ(yes)
*  TAGF050--CAN֡��Ϣ(no)
******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
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
#define DEV_TTY_MOMI_UART         "/dev/ttyHS0"
#define MOMI_UART_RX_BUFFER_SIZE  1500
#define MOMI_UART_BAUDRATE        B115200
#define MOMI_BUFFER_MAX_SIZE      MOMI_UART_RX_BUFFER_SIZE
#define MOMI_TransmitData         MOMI_UartTransmitData
#define MOMI_ReceiveData          MOMI_UartReceiveData

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
uint8_t momi_uart_rx_buffer[MOMI_UART_RX_BUFFER_SIZE];
int fd_momi = -1;
pthread_mutex_t mid_MomiTransmit; // �ݹ黥��


#define MOMI_MAX_TXBUF_SIZE  1500
static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

#if 0
// TLV״̬��Ч��־
extern bittype2 zxup_tlv_flag1;
extern bittype2 zxup_tlv_flag2;
extern bittype2 zxup_tlv_flag3;
extern bittype2 zxdown_tlv_flag1;
extern bittype2 zxdown_tlv_flag2;
extern bittype2 zxengine_tlv_flag;
extern bittype2 zxstatistics_tlv_flag;
extern bittype2 zxversion_tlv_flag;

extern uint8_t zxup_buffer[]; /// �ϳ����ݻ���
extern uint8_t zxdown_buffer[]; /// �³��������ݻ���
extern uint8_t zxengine_buffer[]; /// �³����������ݻ���
extern uint8_t zxstatistics_buffer[]; ///ͳ�����ݻ���
extern uint8_t zxversion_buffer[]; /// �汾��Ϣ����
#endif

rtc_date_t st_rtc_data;
can_frame_t can_frame_table[MAX_CAN_FRAME_NUM];
uint8_t ep_data_buffer[NUMBER_OF_EP_POS];
dtc_context_t dtc_1939;
dtc_context_t dtc_27145;

obd_info_t obd_info = {
  .protocol_type = 1,  // OBD���Э������
  .mil_status = 1,  // ���ϵ�
  .diag_valid_flag = 1,
  .diag_supported_status = 0xFFFF, // ���֧��״̬
  .diag_readiness_status = 0xFFFF, // ��Ͼ���״̬
  .vin_valid_flag = 0,  // VIN����Ч
  .vin = "LXGBPA123test0088",  // VIN��(HJ)
  //.vin = "LXGBPA123test0036",  // VIN��(GB)
  .calid_valid_flag = 1,
  .calid = "666666666666666666",
  .cvn_valid_flag = 1,
  .cvn = "888888888888888888",
  .iupr_valid_flag = 1,
  .iupr ="333333333333333333333333333333333333",
  .dtc_num = 3,
  .dtc ={0x00112233,0x00445566,0x00778899},
  };

/******************************************************************************
 * ��Ϣ���ж���
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x02
#define MOMI_RECV_COMMAND_ID     0x01
#define MOMI_MSG_QUEUE_MAX_SIZE  8192
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

static momi_msg_queue_t moni_msg_queue = {0,0};
static uint32_t momi_msg_state = 0;
static uint16_t momi_msg_pos = 0;
uint8_t moni_msg_buffer[ONE_MOMI_CMD_MAX_SIZE];

#if (PART("MOMI��������"))
//==��ն�������==============================================================
void momi_msg_queue_reset(void)
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
    //momiMsgQueueENTER_CRITICAL();
    momi_msg_queue_pop(&_data);  //ȡһ������
    //momiMsgQueueEXIT_CRITICAL();

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

/*****************************************************************************
 * ���ڳ�ʼ������
 ****************************************************************************/
void MOMI_UartInitialize(uint32_t baudrate)
{
  struct termios options;

  pthread_mutex_init(&mid_MomiTransmit, NULL);
  fd_momi = open(DEV_TTY_MOMI_UART, O_RDWR|O_NOCTTY); // open uart
  if (fd_momi < 0) // ����ʧ��,��δ���??
  {
    printf("ERROR open %s ret=%d\n\r", DEV_TTY_MOMI_UART, fd_momi);
    close(fd_momi);
    return;
  }

  printf("fd_momi: %d \n", fd_momi);
  // configure uart
  tcgetattr(fd_momi, &options);
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
  tcflush(fd_momi, TCIFLUSH);
  tcsetattr(fd_momi, TCSANOW, &options);
}

//==���յ������ݷ������========================================================
uint8_t MOMI_UartReceiveData(void)
{
  uint16_t it;
  int retVal;
  
  retVal = read(fd_momi, momi_uart_rx_buffer, (MOMI_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
  if (retVal > 0)
  {
    for (it = 0; it < retVal; it++)
    {
      momi_msg_queue_push(momi_uart_rx_buffer[it]); // ���յ������ݷ������
    }
    //PcDebug_SendData(momi_uart_rx_buffer, retVal, DBG_MSG_TYPE_CAN);
    return TRUE;
  }

  return FALSE;
}

//===================================================================================
uint16_t MOMI_UartTransmitData(uint8_t *data, uint16_t size)
{
  int retVal;

  if ((0==size) || (NULL==data) || (fd_momi<0))
  {
    return 0;
  }

  pthread_mutex_lock(&mid_MomiTransmit);  // ����
  retVal = write(fd_momi, data, size);
  pthread_mutex_unlock(&mid_MomiTransmit);  // ����
  if (retVal != size)
  {
    printf("Err Momi write ret=%d\n", retVal);
    return 0;
  }

  return size;
}

/*************************************************************************
 * ����������С�ˣ���ɽ�����Ǵ��
*************************************************************************/
void Momi_FillEpData(void)
{
  //��������������Ϣ����
  ep_data_buffer[EP_POS1_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12+1];// ����(2B)
  ep_data_buffer[EP_POS1_ADDRESS+1] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS12];
  
  ep_data_buffer[EP_POS2_ADDRESS] = zxengine_buffer_a502[ZXENGINE_A502_POS1_ADDR]; // ����ѹ��(1B)
  
  ep_data_buffer[EP_POS3_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS2]; // ʵ��Ť�ذٷֱ�(1B)
  
  ep_data_buffer[EP_POS4_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS3]; // Ħ��Ť�ذٷֱ�(1B)
  
  ep_data_buffer[EP_POS5_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1+1]; // ������ת��(2B)
  ep_data_buffer[EP_POS5_ADDRESS+1] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS1];
  
  ep_data_buffer[EP_POS6_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16+1]; // ������ȼ������(2B)
  ep_data_buffer[EP_POS6_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS16];
  
  ep_data_buffer[EP_POS7_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7+1]; // �������ε���Ũ��(2B)
  ep_data_buffer[EP_POS7_ADDRESS+1] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS7];
  
  ep_data_buffer[EP_POS8_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8+1]; // �������ε���Ũ��(2B)
  ep_data_buffer[EP_POS8_ADDRESS+1] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS8];
  
  ep_data_buffer[EP_POS9_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS4]; // ������Һλ(1B)
  
  ep_data_buffer[EP_POS10_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24+1]; // ��������(2B)
  ep_data_buffer[EP_POS10_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS24];
  
  ep_data_buffer[EP_POS11_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9+1]; // �������������¶�(2B)
  ep_data_buffer[EP_POS11_ADDRESS+1] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS9];
  
  ep_data_buffer[EP_POS12_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10+1]; // �������������¶�(2B)
  ep_data_buffer[EP_POS12_ADDRESS+1] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS10];
  
  ep_data_buffer[EP_POS13_ADDRESS] = zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4+1]; // (DPF/POC)����ѹ��(2B)
  ep_data_buffer[EP_POS13_ADDRESS+1] = zxengine_buffer_a5f2[ZXENGINE_A5F2_POS4];
  
  ep_data_buffer[EP_POS14_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS6]; // ��ȴҺ�¶�(1B)
  ep_data_buffer[EP_POS15_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS21]; // ����Һλ(1B)

  // ��������������
  ep_data_buffer[EP_POS16_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS22]; // ������Ť��ģʽ(1B)
  ep_data_buffer[EP_POS17_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS11]; // ����̤��(1B)
  
  ep_data_buffer[EP_POS18_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+3]; // ���ͺ�(4B)
  ep_data_buffer[EP_POS18_ADDRESS+1] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+2];
  ep_data_buffer[EP_POS18_ADDRESS+2] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18+1];
  ep_data_buffer[EP_POS18_ADDRESS+3] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS18];
  
  ep_data_buffer[EP_POS19_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS5]; // �������¶�(1B)
  
  ep_data_buffer[EP_POS20_ADDRESS] = 0x00; // ʵ������������(4B)
  ep_data_buffer[EP_POS20_ADDRESS+1] = 0x00;
  ep_data_buffer[EP_POS20_ADDRESS+2] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6+1];
  ep_data_buffer[EP_POS20_ADDRESS+3] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS6];
  
  ep_data_buffer[EP_POS21_ADDRESS] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+3]; // ����������(4B)
  ep_data_buffer[EP_POS21_ADDRESS+1] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+2];
  ep_data_buffer[EP_POS21_ADDRESS+2] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12+1];
  ep_data_buffer[EP_POS21_ADDRESS+3] = zxengine_buffer_a5f1[ZXENGINE_A5F1_POS12];
  
  ep_data_buffer[EP_POS22_ADDRESS] = zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2+1]; // DPF�����¶�(2B)
  ep_data_buffer[EP_POS22_ADDRESS+1] = zxengine_buffer_a5f2[ZXENGINE_A5F2_POS2];

  // ������Ϣ
  ep_data_buffer[EP_POS23_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+3]; // ����ʻ���(4B)
  ep_data_buffer[EP_POS23_ADDRESS+1] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+2];
  ep_data_buffer[EP_POS23_ADDRESS+2] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19+1];
  ep_data_buffer[EP_POS23_ADDRESS+3] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS19];
  
  // ep_data_buffer[EP_POS24_ADDRESS] = ; // MIL��״̬(1B)
  
  // ep_data_buffer[EP_POS25_ADDRESS] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+3]; // ������������ʱ��(4B)
  // ep_data_buffer[EP_POS25_ADDRESS+1] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+2];
  // ep_data_buffer[EP_POS25_ADDRESS+2] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+1];
  // ep_data_buffer[EP_POS25_ADDRESS+3] = zxengine_buffer_a5ef[ZXENGINE_A5EF_POS10+0];
}


/*************************************************************************
 *
*************************************************************************/
#if (PART("MOMI����TLV"))
//==�����ն�����ɼ���Ϣ==================================================
void Momi_AnalyzeAdSwitch(void)
{
  uint16_t tempVal;

  if (tlv_a5ff_valid_flag)
  {
    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR];  // �ⲿ��Դ��ѹ
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];
    colt_info.vraw = tempVal;

    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR];  // ���õ�ص�ѹ
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS1_ADDR+1];
    colt_info.vbat = tempVal;

    colt_info.switch1 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS3_ADDR];  // �������ɼ�1
    colt_info.switch2 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS4_ADDR];  // �������ɼ�2
    colt_info.switch3 = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS5_ADDR];  // �������ɼ�3
    colt_info.alarm = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS6_ADDR];  // �����࿪����
    colt_info.can_status = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS7_ADDR];  // CANͨ��״̬
    
    st_rtc_data.year = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR]; // RTCʱ��
    st_rtc_data.month = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+1];
    st_rtc_data.day = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+2];
    st_rtc_data.hour = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+3];
    st_rtc_data.minute = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+4];
    st_rtc_data.second = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS8_ADDR+5];
    
    colt_info.st_tbox_mode = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS9_ADDR]; // ����ģʽ

    tempVal = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS10_ADDR];  // Э�������汾��Ϣ
    tempVal <<= 8;
    tempVal += zxinfo_buffer_a5ff[ZXINFO_A5FF_POS10_ADDR+1];
    colt_info.st_version = tempVal;
  }
  else
  {
    
  }
}

//==�ն�����ɼ���Ϣ====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F001(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  uint16_t pos = 0;
  uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXINFO_BUFFER+2))
  {
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F001\n");
#endif
    retVal = 1;
    tlv_flag = pValue[pos++]; // ��Ч��־
    tlv_flag <<= 8;
    tlv_flag += pValue[pos++];
    zxinfo_tlv_flag.word = tlv_flag;
    memcpy(zxinfo_buffer, &pValue[pos], SIZE_OF_ZXINFO_BUFFER);
    ZxM2m_UpdatePidInfo(); // ���³���������Ϣ
    zxtcw_context.lvc_binded_flag = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS11_ADDR];  // ��״̬
    zxtcw_context.tbox_state = zxinfo_buffer_a5ff[ZXINFO_A5FF_POS12_ADDR];  // ST����״̬
  }

  return retVal;
}

//==GPS�ɼ���Ϣ====================================================
static uint8_t iMomi_AnalyzeTlvMsg_F002(uint8_t* pValue, uint16_t len)
{
  uint8_t retVal = 0;
  //uint16_t pos = 0;
  //uint16_t tlv_flag;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F002\n");
#endif
    retVal = 1;
    //memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F010\n");
#endif

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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F011\n");
#endif

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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F012\n");
#endif

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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F020\n");
#endif

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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F030\n");
#endif

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
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F040\n");
#endif

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
  //uint16_t pos = 0;
  //uint8_t can_num;

  if (len==(SIZE_OF_ZXENGINE_BUFFER))
  {
#if MOMI_DEBUG
    PcDebug_Printf("Momi:F050\n");
#endif

    retVal = 1;
    //can_num = pValue[pos++]; // ��Ч��־
    //memcpy(zxengine_buffer, &pValue[pos], SIZE_OF_ZXENGINE_BUFFER);
  }

  return retVal;
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI�����ͷ���Э������"))
//==����У���================================================================
uint8_t iMomi_CalcSumCheck(uint8_t* pbuf,uint16_t len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  
  return sum;
}

//==4G����״̬��GPSʱ����Ϣ====================================================
static uint16_t iMomi_BuildTlvMsg_E001(uint8_t *pbuf)
{
  uint16_t len = 0;
  uint16_t tlv_length = SIZE_OF_MOMI_E001;
  utc_time_t utc_time;
  rtc_date_t bj_time;

  pbuf[len++] = 0xE0; // TAG
  pbuf[len++] = 0x01;

  pbuf[len++] = (tlv_length>>8) & 0xFF; // LENGTH
  pbuf[len++] = tlv_length & 0xFF;

  if(M2M_GetConnStatus()==M2M_TRUE)  // ���繤��״̬        // VALUE
  {pbuf[len++] = 0x01;}
  else
  {pbuf[len++] = 0x00;}

  if(Cellura_GetSimCardState()==1)  // SIM��ʶ��״̬: 1=����; 0,����
  {pbuf[len++] = 0x00;}
  else
  {pbuf[len++] = 0x01;}

  pbuf[len++] = GPS_GetPositioningStatus();  // GPS��λ״̬

  pbuf[len++] = 0x00;  // WIFI����״̬

  pbuf[len++] = 0x00;  // ETH����״̬

  if(M2M_GetRfuStatus() > M2M_UPDATE_STATE_IDLE)  // Զ��������־
  {  pbuf[len++] = 0x01;}
  else
  {  pbuf[len++] = 0x00;}

  if(GPS_GetPositioningStatus())
  {
    utc_time = GPS_GetUtcTime();
    RTC_CovertUtcToBjt(&utc_time, &bj_time);
    pbuf[len++] = bj_time.year;  // GPSʱ��
    pbuf[len++] = bj_time.month;
    pbuf[len++] = bj_time.day;
    pbuf[len++] = bj_time.hour;
    pbuf[len++] = bj_time.minute;
    pbuf[len++] = bj_time.second;
  }
  
  return len;
}

//==����4G����״̬��GPSʱ����Ϣ==================================================
void iMomi_Send4GStatusMsg(void)
{
  uint8_t* pdata = momi_tx_buffer;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

  pdata[MOMI_FRAME_STX_FIELD] = MOMI_MSG_HEAD;  // ֡ͷ(1B)
  pdata[MOMI_FUNCTION_CODE_FIELD] = MOMI_SEND_COMMAND_ID;  // �����
  pdata[MOMI_SN_HIGH_FIELD] = 0x01;  // ��ˮ��
  pdata[MOMI_SN_LOW_FIELD] = 0x01;
  pdata[MOMI_DATA_START_FIELD] = 0x01;  // TLV����
  pos = MOMI_DATA_START_FIELD + 0x01;

  msg_len = iMomi_BuildTlvMsg_E001(&pdata[pos]); // �ն�����ɼ���Ϣ
  pos += msg_len;

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iMomi_CalcSumCheck(&pdata[3], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[MOMI_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[MOMI_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

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
    iMomi_Send4GStatusMsg();
  }

  // GPSλ����Ϣ(������1s)
  if (divide_for_1second)
  {
    divide_for_1second--;
  }
  else
  {
    divide_for_1second = 100;  // 1s����
    Momi_FillEpData();
    //iMomi_SendGpsMsg();
    //PcDebug_SendString("MomiProduce!\n");
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)+֡β(4B)
// ���ݳ��� = LengthOf{������(1B)+��ˮ��(2B)+���ݰ�(NB)+У��(1B)}
// У�� = sum{������(1B)+��ˮ��(2B)+���ݰ�(NB)}
//////////////////////////////////////////////////////////////////////////////////////////////
#if (PART("MOMI���պͽ���Э������"))
//==�ж�MOMI��ϢУ���==========================================================
uint8_t Momi_CheckMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;

  if(pdata[MOMI_FUNCTION_CODE_FIELD] != MOMI_RECV_COMMAND_ID) // �����ֶδ���
  {
    //PcDebug_Printf("MomiCmdErr\n");
    return MOMI_NOK;
  }

  msg_len = size-8;  // ȥ��{֡ͷ(1B)+���ݳ���(2B)+У��(1B)+֡β(4B)
  received_check_sum = pdata[size-5]; // �����
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[MOMI_FUNCTION_CODE_FIELD], msg_len); // ����У��ֵ:������ſ�ʼ����
  if(received_check_sum==calculated_check_sum)  //check if crc's match
  {
    //PcDebug_Printf("MomiCrcOK\n");
    return MOMI_OK;
  }
  else
  {
    //PcDebug_Printf("MomiCrcErr\n");
    return MOMI_NOK;
  }
}

//==�����յ���TLV����=============================================================
uint16_t iMomi_AnalyzeTlvData(uint16_t tag, uint16_t len, uint8_t *pValue)
{
  uint16_t retVal = 0;

  switch(tag)
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

    case 0xF020: // ��������Ϣ��Ϣ
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
  uint8_t* pbuf = &pdata[MOMI_DATA_START_FIELD];
  uint8_t fail_tag_num = 0;

  msg_status = Momi_CheckMsg(pdata, size);
  if(msg_status==MOMI_OK)
  {
    msg_len = pdata[MOMI_FRAME_SIZE_HIGH_FIELD];  // ���ݳ���
    msg_len <<= 8;
    msg_len += pdata[MOMI_FRAME_SIZE_LOW_FIELD];
    if(msg_len < 5)  // ������
    {
      return;
    }
    msg_len -= 4;  // ���ݰ��ĳ���(������������(1B)+��ˮ��(2B)+У��(1B))
    msg_sn = pdata[MOMI_SN_HIGH_FIELD];  // ��Ϣ��ˮ��
    msg_sn <<= 8;
    msg_sn += pdata[MOMI_SN_LOW_FIELD];

    pos = 0;
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
        PcDebug_Printf("MomiErr:Tlv=%x\n", tag); // ���16����
        // ���������Ϣ
      }
      pos += length; // ָ����һ��TLV
      tlvNum--;
    }
  }
}
#endif

/******************************************************************************
 * 4Gģ�鷢�ͱ��ĸ�ST��Ƭ��
*******************************************************************************/
void* pthread_MomiProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10msִ��һ��
    Momi_ProduceSendMsg();  // ����MOMI��Ϣ
  }
}

/******************************************************************************
 * ����ST��Ƭ������4G�ı���
*******************************************************************************/
void* pthread_MomiProcess(void *argument)
{
  uint16_t cmd_size;

  while (1)
  {
    if (MOMI_ReceiveData()) // �ȴ�����(����)
    {
      while(momi_msg_queue_size()>0)
      {
        cmd_size = momi_msg_queue_find(moni_msg_buffer, ONE_MOMI_CMD_MAX_SIZE); // ����MONI��Ϣ
        if(cmd_size>0) // �õ�һ��������MOMI��Ϣ
        {
          //PcDebug_Printf("MomiQ:head=%d,tail=%d\n",moni_msg_queue.head, moni_msg_queue.tail);
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
  momi_msg_queue_reset();
  MOMI_UartInitialize(MOMI_UART_BAUDRATE);
}

/*************************************************************************
 *
*************************************************************************/
void Momi_ServiceStart(void)
{
  pthread_attr_t thread_attr;
  int ret ,stacksize = DEFAULT_THREAD_STACK_SIZE; // thread��ջ����Ϊ40KB

  pthread_attr_init(&thread_attr);
  ret = pthread_attr_setstacksize(&thread_attr,stacksize);
  if(ret!=0)
  {
    printf("Set StackSize Error!\n");
  }

  pthread_create(&pthreads[PTHREAD_MOMI_PROCESS_ID], &thread_attr, pthread_MomiProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_MOMI_PRODUCE_ID], &thread_attr, pthread_MomiProduce, NULL);
  usleep(10);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// DTC����
////////////////////////////////////////////////////////////////////////////////////////////////////////
/************************************************************************
* ��ȡ�Ƿ����µ�DTC��Ҫ�ϱ�
*************************************************************************/
uint8_t DTC_GetNewFlag(dtc_context_t* pThis)
{
  return pThis->new_flag;
}

/************************************************************************
* �����DTC��־
*************************************************************************/
void DTC_ClearNewFlag(dtc_context_t* pThis)
{
  pThis->new_flag = 0;
}

/******************************************************************************************
* �յ�CAN�����������������������б��Ƿ��иù����룬���������¸ù�����ļ�ʱ,
* ���û�У�����Ϊ���²����Ĺ����룬��ӵ�����������б�,������;
* ����б�������������ֱ���б��пյ�Ԫ
* ����: 0-û�������������������б�������1-�����˹�����
*******************************************************************************************/
uint8_t DTC_SaveCode(dtc_context_t* pThis, uint32_t dtcode)
{
  uint8_t i;

  if (dtcode == 0x00000000)
  {
    return 0;
  }

  dtcode &= 0x00FFFFFF;
  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i] == dtcode) // ��ʷ������
    {
      pThis->debounce_tmr[i] = DTC_DEBOUNCE_TIME_SP;
      return 0;
    }
  }

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++) // ����������
  {
    if (pThis->code[i] == 0)
    {
      pThis->code[i] = dtcode;
      pThis->debounce_tmr[i] = DTC_DEBOUNCE_TIME_SP;
      pThis->total_num++;
      //PcDebug_Printf("SaveNewDTC!\r\n");
      return 1;
    }
  }
  return 0;	// �б�����
}

/*****************************************************************************************
* �˺���ÿ1������һ�Σ��Թ������б��й�������ڵ�ʱ����е���ʱ������ʱ��0ʱ��
* ��Ϊ�ù������Ѿ���ʧ���Ըù���������
* DTC(Diagnostic Trouble Code)
******************************************************************************************/
void DTC_DebounceCode(dtc_context_t* pThis)
{
  uint8_t i;

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i])
    {
      if (pThis->debounce_tmr[i])
        pThis->debounce_tmr[i]--;
      else
      {
        pThis->code[i] = 0;
        pThis->total_num--;
        //PcDebug_Printf("ClearOldDTC!\r\n");
      }
    }
  }
}

/******************************************************************************************
* ��ȡ�������������
* pBuf - ָ�������������ݻ����ָ��
* ����: ��������������0��ʾû�й������������
******************************************************************************************/
uint8_t DTC_GetCode(dtc_context_t* pThis, uint8_t *pBuf)
{
  uint8_t i;
  uint8_t dtc_cnt=0;

  for (i=0; i<MAX_DTC_TABLE_SIZE; i++)
  {
    if (pThis->code[i])
    {
      *pBuf++ = pThis->code[i] & 0xFF;
      *pBuf++ = (pThis->code[i] >> 8) & 0xFF;
      *pBuf++ = (pThis->code[i] >> 16) & 0xFF;
      *pBuf++ = (pThis->code[i] >> 24) & 0xFF;
      dtc_cnt++;
    }
  }

  if (dtc_cnt == 0)
  {
    pThis->total_num = 0x00;
  }
  //PcDebug_Printf("GetDTC:%d!\r\n",dtc_cnt);
  return dtc_cnt;
}

//========================================================================================
uint8_t DTC_GetTotalNumber(dtc_context_t* pThis)
{
  return pThis->total_num;
}

/******************************************************************************************
* ��ȡ�������������
* pBuf - ָ�������������ݻ����ָ��
* ����: ��������������0��ʾû�й������������
******************************************************************************************/
void OBD_GetDtcCode(obd_info_t* pThis, uint8_t *pBuf)
{
  uint8_t i;
  uint8_t pos=0;

  for (i=0; i<pThis->dtc_num; i++)
  {
    pBuf[pos++] = pThis->dtc[i] & 0xFF;
    pBuf[pos++] = (pThis->dtc[i] >> 8) & 0xFF;
    pBuf[pos++] = (pThis->dtc[i] >> 16) & 0xFF;
    pBuf[pos++] = (pThis->dtc[i] >> 24) & 0xFF;
  }
}

//-----�ļ�CelluraCore.c����---------------------------------------------

