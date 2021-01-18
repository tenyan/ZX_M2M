/*****************************************************************************
* @FileName: AuxCom.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modemģ���Micro����ͨ��Э��ʵ��
******************************************************************************/
//-----ͷ�ļ�����------------------------------------------------------------
#include "AuxComHW.h"
#include "config.h"

/******************************************************************************
 * Typedef
 ******************************************************************************/


/******************************************************************************
 *   Macros
 ******************************************************************************/
#define AuxCom_Receive     USART6_ReceiveData
#define AuxCom_Transmit    USART6_TransmitData

#define AUX_MAX_TXBUF_SIZE  USART6_TX_BUFFER_SIZE
#define AUX_MAX_RXBUF_SIZE  USART6_RX_BUFFER_SIZE
#define aux_rx_buffer       usart6_rx_buffer
#define aux_rx_size         usart6_rx_size

#define AUX_DELAY_MS(ms)   do { osDelay(ms);} while(0)
#define PART(x)     1

// FSRVЭ�鶨��
#define FCLIT_MSG_HEAD            0x7E    // ֡ͷ
#define FCLIT_MSG_TAIL            0x0D0A  // ֡β
#define FCLIT_SEND_COMMAND_ID     0x84
#define FCLIT_RECV_COMMAND_ID     0x04
#define FCLIT_PACKET_SIZE         RFU_BUFFER_SIZE

#define FCLIT_TIMEOUT_SP          50  // ����100ms
#define FCLIT_RETRY_SP            2   // ��ʱ���Դ���

// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(1B)+���ݰ�(NB)+У��(1B)+֡β(2B)
#define FCLIT_FRAME_STX_FIELD        0x00
#define FCLIT_FRAME_SIZE_HIGH_FIELD  0x01
#define FCLIT_FRAME_SIZE_LOW_FIELD   0x02
#define FCLIT_FUNCTION_CODE_FIELD    0x03
#define FCLIT_SERIAL_NUMBER_FIELD    0x04
#define FCLIT_DATA_START_FIELD       0x05

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
extern volatile uint16_t usart6_rx_size;
extern uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];

static uint8_t aux_tx_buffer[AUX_MAX_TXBUF_SIZE];
static uint8_t aux_com_buffer[AUX_MAX_RXBUF_SIZE]; // AuxCom�������ݻ���

rfu_context_t rfu_context;
fclit_context_t fclit_context;

uint8_t rfu_data_buffer[RFU_BUFFER_SIZE];

/******************************************************************************
 * RTOS���
 ******************************************************************************/
//=������==============================================================
#define AppThreadPriority_AuxComProcess   osPriorityHigh2
osThreadId_t tid_AuxComProcess;

const osThreadAttr_t AppThreadAttr_AuxComProcess =
{
  .priority = AppThreadPriority_AuxComProcess,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // �ֽ�
};

//=������==============================================================
#define AppThreadPriority_AuxComProduce   osPriorityHigh1
osThreadId_t tid_AuxComProduce;

const osThreadAttr_t AppThreadAttr_AuxComProduce =
{
  .priority = AppThreadPriority_AuxComProduce,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // �ֽ�
};

/******************************************************************************
 * ����У���
*******************************************************************************/
uint8_t iFCLIT_CalcSumCheck(uint8_t* pbuf,uint16_t len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }

  return sum;
}

/******************************************************************************
 * FCLIT�����Ե��ú���(����100mS,������)
*******************************************************************************/
void FCLIT_Do100msTasks(void)
{
  //==���Դ����ж�===========================================
  if(fclit_context.retry_cnt > fclit_context.retry_sp)
  {
    fclit_context.retry_cnt = 0x00;
    if(fclit_context.state < FCLIT_STATE_SEND_UR_REQ)
    {
      fclit_context.ud_result = 0x03;  // δ�ɹ�
      fclit_context.state = FCLIT_STATE_SEND_UR_REQ;  // ��4G�ϱ�����ʧ��
    }
    else
    {
      fclit_context.state = FCLIT_STATE_IDLE;  // �˳�����
    }
  }

  //==��ʱ�����ж�===========================================
  if(fclit_context.timeout_100ms)
  {
    fclit_context.timeout_100ms--;
    if(fclit_context.timeout_100ms==0x00)
    {
      if(fclit_context.state==FCLIT_STATE_WAIT_UD_RSP)
      {
        fclit_context.state = FCLIT_STATE_SEND_UD_REQ;
      }
      else if(fclit_context.state==FCLIT_STATE_WAIT_UR_RSP)
      {
        fclit_context.state = FCLIT_STATE_SEND_UR_REQ;
      }
      else
      {
        fclit_context.state = FCLIT_STATE_IDLE;  // �˳�����
      }
    }
  }
}

/******************************************************************************
 * ��ʼ��FSRV����
*******************************************************************************/
void FCLIT_Init(void)
{
  fclit_context.state = FCLIT_STATE_IDLE;
  fclit_context.dev_id = 0x00;  // ��Ч
  fclit_context.msg_sn = 0x00;
  fclit_context.fw_packet_index = 0x00;
  fclit_context.fw_block_size = FCLIT_PACKET_SIZE;
  fclit_context.start_address = 0x00;
  fclit_context.ending_address = 0x00;
  fclit_context.total_block_count = 0x00;
  fclit_context.percent = 0x00;
  //fclit_context.result = 0x00;

  fclit_context.tx_size = 0x00;
  fclit_context.tx_data = aux_tx_buffer;

  fclit_context.rx_size = 0x00;
  fclit_context.rx_data = aux_rx_buffer;

  fclit_context.retry_sp = FCLIT_RETRY_SP;
  fclit_context.retry_cnt = 0;

  fclit_context.timeout_100ms_sp = FCLIT_TIMEOUT_SP;
  fclit_context.timeout_100ms = fclit_context.timeout_100ms_sp;
}

#if (PART("����FCLIT����"))
/*************************************************************************
 * ���͹̼����������Ϣ
*************************************************************************/
//==��4Gģ�鷢�͹̼�����֪ͨ��Ӧ��(ST->4G)================================
void iFCLIT_SendUnRspMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUn:%d!\n", pThis->un_result);
#endif

  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->rx_msg_sn;  // ��Ӧ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x01;  //�����ʶ
  pdata[pos++] = pThis->un_result;  // Ӧ����(0=�ɹ�, 1=��ʽ�쳣,2=�豸��֧��,3=����)
  //===============================================================

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // ֡β(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);

  if (pThis->un_result == 0x00) // ִ�гɹ�
  {
    pThis->state = FCLIT_STATE_START;  // ��������
  }
  else
  {
    pThis->state = FCLIT_STATE_IDLE;  // ��������
  }
}

//==������������=============================================================
void iFCLIT_start(fclit_context_t* pThis)
{
#if FCLIT_DEBUG
  PcDebug_Printf("FclitSt!\n");
#endif

  rfu_EraseFlashHexFile();  // ����Flash
  pThis->retry_cnt = 0;
  pThis->ud_result = 0x01;  // δ�ɹ�
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->fw_packet_index = 0x00;
}

//==�̼�������������(ST->4G)==============================================
void iFCLIT_SendUdReqMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUd!\n");
#endif

  pThis->msg_sn++;
  pThis->retry_cnt++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x02;  // �����ʶ
  pdata[pos++] = (uint8_t)(pThis->fw_packet_index>>8);  // �������������
  pdata[pos++] = (uint8_t)pThis->fw_packet_index;
  //===============================================================

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // ֡β(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==��4G�ϱ��������(ST->4G)==============================================
void iFCLIT_SendUrReqMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUr:%d!\n", pThis->ud_result);
#endif

  pThis->msg_sn++;
  pThis->retry_cnt++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x03;  // �����ʶ
  pdata[pos++] = pThis->dev_id;  // ����Ŀ���豸
  pdata[pos++] = pThis->ud_result;  // ���ؽ��
  //===============================================================

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // ֡β(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

/*************************************************************************
 * 100ms����һ��
*************************************************************************/
void FCLIT_ProduceSendMsg(fclit_context_t* pThis)
{
  if (pThis->state==FCLIT_STATE_IDLE) /// ����
  {
    return;
  }

  switch (pThis->state)
  {
  case FCLIT_STATE_NOTIFIED: /// ��4G��������֪ͨ��Ӧ
    iFCLIT_SendUnRspMsg(pThis);
    pThis->state = FCLIT_STATE_START;
    break;

  case FCLIT_STATE_START: /// ��������(��ʼ������)
    iFCLIT_start(pThis);
    pThis->state = FCLIT_STATE_SEND_UD_REQ;
    break;

  case FCLIT_STATE_SEND_UD_REQ:  /// ��4G����������������
    iFCLIT_SendUdReqMsg(pThis);
    pThis->state = FCLIT_STATE_WAIT_UD_RSP;
    break;

  case FCLIT_STATE_SEND_UR_REQ:  /// ��4G���������������
    iFCLIT_SendUrReqMsg(pThis);
    pThis->state = FCLIT_STATE_WAIT_UR_RSP;
    break;

  default:
    break;
  }

  FCLIT_Do100msTasks();
}
#endif

/******************************************************************************
 * �����յ�4Gģ����Ϣ
*******************************************************************************/
#if (PART("����FCLIT����"))
//==�ж�MOMI��ϢУ���==========================================================
uint8_t iFCLIT_CheckMsg(fclit_context_t* pThis)
{
  uint8_t *pdata = pThis->rx_data;
  uint16_t size = pThis->rx_size;
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;
  uint16_t msg_tail;

  if (pdata[0] != FCLIT_MSG_HEAD) // ֡ͷ����
  {
    return FCLIT_NOK;
  }

  msg_tail = (uint16_t)(pdata[size-2]<<8) + pdata[size-1]; // ��ȡ֡β
  if (msg_tail != FCLIT_MSG_TAIL) // ֡β����
  {
    return FCLIT_NOK;
  }

  if (pdata[3] != FCLIT_RECV_COMMAND_ID) // �����ֶδ���
  {
    return FCLIT_NOK;
  }

  msg_len = size-6; // ����֡ͷ(1B)+���ݳ���(2B)+�����(1B)+֡β(2B)
  received_check_sum = pdata[size-3]; // ���յ��ļ����
  calculated_check_sum = iFCLIT_CalcSumCheck(&pdata[FCLIT_FUNCTION_CODE_FIELD], msg_len); // ����У��ֵ:������ſ�ʼ����
  if (received_check_sum==calculated_check_sum) //check if crc's match
  {
    return FCLIT_OK;
  }
  else
  {
    return FCLIT_NOK;
  }
}

//==��������ָ֪ͨ��(4G->ST)===================================================
void iFCLIT_AnalyzeUnReqMsg(fclit_context_t* pThis)
{
  uint8_t *pdata;
  uint16_t pos;
  uint16_t msg_len;
  uint16_t msg_sn;
  uint8_t rfu_type;
  uint8_t rfu_dev;

  if (pThis->state != FCLIT_STATE_IDLE) // ����æ,���Դ˴�����
  {
    return;
  }

  msg_sn = pThis->rx_data[FCLIT_SERIAL_NUMBER_FIELD];  // ��Ϣ��ˮ��
  pThis->rx_msg_sn = msg_sn;  // ��ˮ��
  pThis->un_result = 0x00;  // ִ�гɹ�
  pThis->state = FCLIT_STATE_NOTIFIED;  // ״̬��ת(��������֪ͨ��Ӧ)

  msg_len = pThis->rx_data[FCLIT_FRAME_SIZE_HIGH_FIELD];  // ���ݳ���
  msg_len <<= 8;
  msg_len += pThis->rx_data[FCLIT_FRAME_SIZE_LOW_FIELD];
  if (msg_len < 20) // ������
  {
    pThis->un_result = 0x01;  // ��ʽ�쳣
    return;
  }
  msg_len -= 3;  // ���ݰ��ĳ���(������������(1B)+��ˮ��(1B)+У��(1B))

  //==��������======================================================
  pos = 0x00;
  pdata = &pThis->rx_data[FCLIT_DATA_START_FIELD];
  pos++;  // �����ʶ
  rfu_type = pdata[pos++];  // ��������
  rfu_dev = pdata[pos++];  // ����Ŀ���豸
  if (rfu_dev > 3 || rfu_dev==0x00) // 0x01=ECU������, 0x02=��ʾ��, 0x03=STЭ������
  {
    pThis->un_result = 0x02;  // �豸��֧��
    return;
  }
  //==��������======================================================
  rfu_context.type = rfu_type;  // ��������
  rfu_context.dev = rfu_dev;  // ����Ŀ���豸
  pThis->dev_id = rfu_dev;

  rfu_context.file_name_length = pdata[pos++];  // �̼����Ƴ���
  memcpy(rfu_context.file_name, &pdata[pos], rfu_context.file_name_length);  // �̼�����(ASCII)
  pos += rfu_context.file_name_length;

  rfu_context.file_version_length = pdata[pos++];  // �̼��汾����
  memcpy(rfu_context.file_version, &pdata[pos], rfu_context.file_version_length);  // �̼��汾(ASCII)
  pos += rfu_context.file_version_length;

  rfu_context.file_length = pdata[pos++];  // �̼���С(4B)
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];
  rfu_context.file_length <<= 8;
  rfu_context.file_length += pdata[pos++];

  rfu_context.total_block_count = pdata[pos++];  // �̼��ܰ���(2B)
  rfu_context.total_block_count <<= 8;
  rfu_context.total_block_count += pdata[pos++];

  rfu_context.block = 0x00;  // ��ʼ��
  rfu_context.cumulated_address = 0x00;
  rfu_context.ending_address = rfu_context.file_length;

  rfu_context.plain_crc32val = pdata[pos++];  // �̼�CRCУ��(4B)
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];
  rfu_context.plain_crc32val <<= 8;
  rfu_context.plain_crc32val += pdata[pos++];

  pThis->fw_block_size = pdata[pos++];  // ������С(�̶�ֵ1024)
  pThis->fw_block_size <<= 8;
  pThis->fw_block_size += pdata[pos++];

  if (pThis->fw_block_size >= FCLIT_FIRMWARE_PACKET_LEN) // ������С�ж�
  {
    pThis->un_result = 0x03;  // ����
    return;
  }
  //===============================================================
}

//pThis->retry_cnt = 0x00;  // ���Լ�����
//pThis->timeout_100ms = pThis->timeout_100ms_sp;  // ��λ��ʱ��ʱ��
//pThis->msg_sn = 0x00;  // ������ˮ��

//==��������������Ӧָ��(4G->ST)===================================================
void iFCLIT_AnalyzeUdRspMsg(fclit_context_t* pThis)
{
  uint8_t *pdata;
  uint16_t pos;
  uint16_t current_block_index;  // ��ǰ����
  uint16_t total_block_num;  // �ܰ�����
  uint16_t packet_len;
  uint8_t rfu_status;

  if (pThis->rx_size < 6) // ������
  {
    pThis->ud_result = 0x01;  // ��ʽ�쳣
    return;
  }

  //==��������======================================================
  pos = 0x00;
  pdata = &pThis->rx_data[FCLIT_DATA_START_FIELD];
  pos++;  // �����ʶ

  current_block_index = pdata[pos++];  //==��ǰ�����к�
  current_block_index <<= 8;
  current_block_index += pdata[pos++];
  if (current_block_index != rfu_context.block)
  {
    return;
  }

  total_block_num = pdata[pos++];  //==�ļ��ܰ���
  total_block_num <<= 8;
  total_block_num += pdata[pos++];
  if (total_block_num != rfu_context.total_block_count)
  {
    return;
  }

  packet_len = pdata[pos++];  //==�����̼����ݳ���
  packet_len <<= 8;
  packet_len += pdata[pos++];
  if (packet_len == 0)
  {
    return;
  }

  rfu_SaveFlashHexFile(&rfu_context, &pdata[pos], packet_len);  // ��BIN�ļ�д��SPI FLASH
  rfu_context.block++;
  rfu_context.percent = rfu_context.block * 100L / rfu_context.total_block_count;
  if (current_block_index==(total_block_num-1)) // �������(���һ��)
  {
    rfu_status = rfu_CheckNewFirmware(&rfu_context, rfu_data_buffer, RFU_BUFFER_SIZE);
    if (RFU_OK==rfu_status)
    {
      pThis->ud_result = 0;
      if (rfu_context.dev==0x01) // ������
      {
        PcDebug_SendString("RfuCrc:Ctl-Ok!\n");
      }
      else if (rfu_context.dev==0x02) // ��ʾ��
      {
        PcDebug_SendString("RfuCrc:LCD-Ok!\n");
      }
      else if (rfu_context.dev==0x03) // �ն�Э������
      {
        PcDebug_SendString("RfuCrc:ST-Ok!\n");
      }
      else
      {
        PcDebug_SendString("RfuCrc:Ok!\n");
      }
    }
    else
    {
      pThis->ud_result = 1;
      PcDebug_SendString("RfuCrc:Err!\n");
    }

    pThis->state = FCLIT_STATE_SEND_UR_REQ;
    rfu_context.cumulated_address = 0;
    rfu_context.block = 0;
    return ;
  }

  pThis->retry_cnt = 0;
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->state = FCLIT_STATE_SEND_UD_REQ;

  return ;
}

//==���������ϱ���Ӧָ��(4G->ST)==============================================
void iFCLIT_AnalyzeUrRspMsg(fclit_context_t* pThis)
{
  if (0==pThis->ud_result)  // �̼����سɹ�
  {
    pThis->state = FCLIT_STATE_IDLE;
    if ((pThis->dev_id==0x03) || (memcmp(rfu_context.file_name, "ZXM2M_ST", 8)==0))
    {
      Tbox_SetMachineState(TBOX_STATE_IAP); // T-BOX��������ģʽ
    }
    // �ڴ˴����Ӷ��ⲿ����������ʾ����������
  }
}

//============================================================================
static void FCLIT_ProcessRecvMsg(fclit_context_t* pThis)
{
  uint8_t msg_status;
  uint8_t command;

  msg_status = iFCLIT_CheckMsg(pThis);
  if (msg_status==FCLIT_OK)
  {
    command = pThis->rx_data[FCLIT_DATA_START_FIELD];  // �����ʶ
    switch (command)
    {
    case FCLIT_COMMAND_TYPE_NOTIFIE:  // ����֪ͨ
      iFCLIT_AnalyzeUnReqMsg(pThis);
      break;

    case FCLIT_COMMAND_TYPE_DOWNLOAD:  // ��������
      iFCLIT_AnalyzeUdRspMsg(pThis);
      break;

    case FCLIT_COMMAND_TYPE_REPORT:  // �ϱ����
      iFCLIT_AnalyzeUrRspMsg(pThis);
      break;

    default:
      break;
    }
  }
}
#endif

/******************************************************************************
 * �����������ݷ��ͽ���
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 100mִ��һ��
  {
    AUX_DELAY_MS(OS_TICKS_PER_SEC/10); // 100msִ��һ��
    FCLIT_ProduceSendMsg(&fclit_context); // ����MOMI��Ϣ
  }
}

/******************************************************************************
 * ����ͨ���������ڷ�����M2M͸��ָ��
*******************************************************************************/
void AuxCom_ProcessM2mMsg(uint8_t *pdata, uint16_t size)
{
  if (pdata[0]==0x05 && (size>=10))
  {
    memcpy(aux_com_buffer, pdata, size);
    SbusMsg_AuxCom.data_size = size;  // �������ݳ���
    SbusMsg_AuxCom.data = aux_com_buffer;  // ��ȡ���ݵ�ַ
    SYSBUS_PutMbox(SbusMsg_AuxCom);  // ������Ϣ
  }
}

/******************************************************************************
 * �����������ݽ��ս���
*******************************************************************************/
void AppTask_AuxComProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (AuxCom_Receive(&pdata, &len)) // �ȴ��ź���(����)
    {
      if (pdata[0]==0x05) // ͸��M2M��������CMD_REQָ��
      {
        if ( (pdata[15]=='P' && pdata[16]=='W') || (pdata[15]=='R' && pdata[16]=='C') ) // PWָ���RCָ��
        {
          AuxCom_ProcessM2mMsg(pdata, len);
        }
      }
      else
      {
        fclit_context.rx_data = pdata;
        fclit_context.rx_size = len;
        FCLIT_ProcessRecvMsg(&fclit_context);
      }
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceInit(void)
{
  USART6_Initialize(AUXCOM_UART_BAUDRATE);
  FCLIT_Init();
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceStart(void)
{
  tid_AuxComProcess = osThreadNew(AppTask_AuxComProcess, NULL, &AppThreadAttr_AuxComProcess);
  tid_AuxComProduce = osThreadNew(AppTask_AuxComProduce, NULL, &AppThreadAttr_AuxComProduce);
}

//-----�ļ�AuxCom.c����---------------------------------------------

