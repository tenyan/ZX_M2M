/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    ���ļ�Ϊ����ͨ��ģ���c�ļ�
 ********************************************************************************/
#include "config.h"

extern uint8_t im2m_BuildMsgHead(uint8_t *pbuf, im2m_msg_type_t msgType, uint16_t msgBodyLen, uint8_t flag, uint16_t SerialNumber);
extern uint8_t im2m_CalcSumCheck(uint8_t* pbuf,uint16_t len);

/******************************************************************************
 * Macros
 ******************************************************************************/
#define AUX_UART_RX_BUFFER_SIZE  1500

#define AUX_COM_BUADRATE         B115200
#define AUX_COM_BUFFER_MAX_SIZE  AUX_UART_RX_BUFFER_SIZE

// FSRVЭ�鶨��
#define FSRV_MSG_HEAD            0x7E    // ֡ͷ
#define FSRV_MSG_TAIL            0x0D0A  // ֡β
#define FSRV_SEND_COMMAND_ID     0x04
#define FSRV_RECV_COMMAND_ID     0x84
#define FSRV_PACKET_SIZE         RFU_BUFFER_SIZE

#define FSRV_TIMEOUT_SP          30  // ����100ms
#define FSRV_RETRY_SP            2   // ��ʱ���Դ���


// ֡ͷ(1B)+���ݳ���(2B)+������(1B)+��ˮ��(1B)+���ݰ�(NB)+У��(1B)+֡β(2B)
#define FSRV_FRAME_STX_FIELD        0x00
#define FSRV_FRAME_SIZE_HIGH_FIELD  0x01
#define FSRV_FRAME_SIZE_LOW_FIELD   0x02
#define FSRV_FUNCTION_CODE_FIELD    0x03
#define FSRV_SERIAL_NUMBER_FIELD    0x04
#define FSRV_DATA_START_FIELD       0x05

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
int fd_auxcom = -1;
static uint8_t aux_rx_buffer[AUX_UART_RX_BUFFER_SIZE];
static uint8_t aux_tx_buffer[AUX_COM_BUFFER_MAX_SIZE];

fsrv_context_t fsrv_context;

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

  retVal = read(fd_auxcom, aux_rx_buffer, (AUX_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
  if (retVal > 0)
  {
    *data = aux_rx_buffer;
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
 *
******************************************************************************/
/*
//���ڷ��ʹ������
uslen :��Ҫ���������峤�� (������Command��SN��Checksum 3���ֽڳ���)
*/
void AuxCom_SendData(uint8_t type, uint8_t *pdata, uint16_t size, uint8_t sn)
{
#if 0
  uint8 SendBuff[1200] = {0};

  SendBuff[0] = 0x7E;
  SendBuff[1] = (uint8)((uslen+3)>>8);
  SendBuff[2] = (uint8)((uslen+3)&0xFF);
  SendBuff[3] = ucCommand;            //Command

  SendBuff[4] = ucSN;
  memcpy(&SendBuff[5],pdata,uslen);

  SendBuff[5+uslen] = SumCalc(&SendBuff[3],uslen+2);

  SendBuff[6+uslen] = 0x0D;
  SendBuff[7+uslen] = 0x0A;

  AuxCom_Transmit(SendBuff, 8+uslen);
#endif
}

//==����У���================================================================
uint8_t iFSRV_CalcSumCheck(uint8_t* pbuf,uint16_t len)
{
  uint16_t i;
  uint8_t sum = 0;

  for (i = 0; i < len; i++)
  {
    sum += pbuf[i];
  }
  
  return sum;
}

//==������������=============================================================
void iFSRV_start(fsrv_context_t* pThis)
{
  pThis->retry_cnt = 0;
  pThis->result = 0x00;
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->state = FSRV_STATE_SEND_UN;
}

//==����UN��Ϣ��Э������======================================================
void iFSRV_SendUnMsg(fsrv_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FSRV_DEBUG
  PcDebug_Printf("FsrvUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FSRV_FRAME_STX_FIELD] = FSRV_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FSRV_FUNCTION_CODE_FIELD] = FSRV_SEND_COMMAND_ID;  // �����
  pdata[FSRV_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FSRV_DATA_START_FIELD;
  
  //==��������======================================================
  pdata[pos++] = 0x01;  //�����ʶ
  pdata[pos++] = rfu_context.type;  // ��������
  pdata[pos++] = rfu_context.dev;  // ����Ŀ���豸
  
  pdata[pos++] = rfu_context.file_name_length;  // �̼����Ƴ���
  memcpy(&pdata[pos], rfu_context.file_name, rfu_context.file_name_length);  // �̼�����(ASCII)
  pos += rfu_context.file_name_length;
  
  pdata[pos++] = rfu_context.file_version_length;  // �̼��汾����
  memcpy(&pdata[pos], rfu_context.file_version, rfu_context.file_version_length);  // �̼��汾(ASCII)
  pos += rfu_context.file_version_length;
  
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>24);  // �̼���С(4B)
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>16);
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>8);
  pdata[pos++] = (uint8_t)(rfu_context.file_length);

  pdata[pos++] = (uint8_t)(rfu_context.total_block_count>>8);  // �̼��ܰ���
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count);

  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>24);  // �̼�CRCУ��(4B)
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>16);
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>8);
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val);

  pdata[pos++] = (uint8_t)(FSRV_PACKET_SIZE>>8);  // ������С(�̶�ֵ1024)
  pdata[pos++] = (uint8_t)FSRV_PACKET_SIZE;  // ������С(�̶�ֵ1024)
  //===============================================================

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iFSRV_CalcSumCheck(&pdata[FSRV_FUNCTION_CODE_FIELD], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[FSRV_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[FSRV_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FSRV_MSG_TAIL>>8) & 0xFF;  // ֡β(2B)
  pdata[pos++] = FSRV_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==��ȡ�̼��ְ�===============================================================
uint16_t fsrv_BuildFwData(fsrv_context_t* pThis, uint8_t* pbuf)
{
  uint16_t size = FSRV_PACKET_SIZE;

  pThis->start_address = pThis->fw_packet_index * FSRV_PACKET_SIZE;
  if ((pThis->start_address + size) > pThis->ending_address)
  {
    size = pThis->ending_address - pThis->start_address;
  }

  pbuf[0] = (uint8_t)(size>>8);  // �̼����ݳ���
  pbuf[1] = (uint8_t)(size);
  rfu_ReadFlashHexFile(pThis->dev_id,pThis->start_address, &pbuf[2], size); // ��ȡ��������
  
  //pThis->block++;
  //pThis->percent = pThis->fw_packet_index * 100L / pThis->total_block_count;
  return (size+2);
}

//==����UD��Ϣ��Э������======================================================
void iFSRV_SendUdMsg(fsrv_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FSRV_DEBUG
  PcDebug_Printf("FsrvUD:%d!\n",pThis->fw_packet_index);
#endif

  pThis->msg_sn++;
  pdata[FSRV_FRAME_STX_FIELD] = FSRV_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FSRV_FUNCTION_CODE_FIELD] = FSRV_SEND_COMMAND_ID;  // �����
  pdata[FSRV_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FSRV_DATA_START_FIELD;
  
  //==��������======================================================
  pdata[pos++] = 0x02;  //�����ʶ
  pdata[pos++] = (uint8_t)(pThis->fw_packet_index>>8);  //��ǰ���������
  pdata[pos++] = (uint8_t)pThis->fw_packet_index;
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count>>8);  // �̼��ܰ���
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count);
  
  msg_len = fsrv_BuildFwData(pThis, &pdata[pos]);  // ��ȡ�ļ�����
  pos += msg_len;
  //===============================================================

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  check_sum = iFSRV_CalcSumCheck(&pdata[FSRV_FUNCTION_CODE_FIELD], msg_len); // ����У��ֵ:������ſ�ʼ����
  pdata[pos++] = check_sum;  // У���

  msg_len = pos - 3; // ����֡ͷ�����ݳ���
  pdata[FSRV_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // ����
  pdata[FSRV_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FSRV_MSG_TAIL>>8) & 0xFF;  // ֡β(2B)
  pdata[pos++] = FSRV_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==��ƽ̨�ϱ��������(�ն�->ƽ̨)==============================================
uint16_t iFSRV_SendUrMsg(fsrv_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = m2m_context.tx_data;
  uint16_t len;

#if FSRV_DEBUG
  PcDebug_Printf("FsrvUR:%d!\n", pThis->result);
#endif

  //==����������==========================================
  len = M2M_MSG_HEAD_LEN;

  //==��������========
  pbuf[len++] = 0x00;  // �������ͳ���
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // ����
  pbuf[len++] = 'R';

  //==��������========
  pbuf[len++] = pThis->result; // �������
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==��������ͷ==========================================
  m2m_context.upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, m2m_context.upload_sn); // M2mUploadSN

  //==����У����==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  m2m_context.tx_size = msg_len;
  im2m_SendNetData(m2m_context.tx_data, m2m_context.tx_size); // ƽ̨

  return msg_len;
}

/******************************************************************************
 * FSRV�����Ե��ú���(����100mS,������)
*******************************************************************************/
void FSRV_Do100msTasks(void)
{
  // ��ʱ�ж�
  if(fsrv_context.timeout_100ms)
    fsrv_context.timeout_100ms--;
  else
  {
#if FSRV_DEBUG
    PcDebug_Printf("FsrvTO!\n");
#endif
    fsrv_context.timeout_100ms = fsrv_context.timeout_100ms_sp;
    fsrv_context.retry_cnt++;
    if(fsrv_context.retry_cnt < fsrv_context.retry_sp) // ���Դ���
    {
      fsrv_context.state = FSRV_STATE_SEND_UN;  // ���·�������֪ͨ
    }
    else
    {
      fsrv_context.state = FSRV_STATE_REPORT;  // �ϱ�����ʧ�ܽ��

      if(fsrv_context.dev_id==3)  // Э������
      {  fsrv_context.result = 0x01;}
      else if(fsrv_context.dev_id==1)  // �ⲿECU
      {  fsrv_context.result = 0x01;}  // 4=����������ʧ��
      else
      {  fsrv_context.state = FSRV_STATE_IDLE;}  // ��������˳�����
    }
  }
}

/*************************************************************************
 * 100ms����һ��(�̼�������FSRV = Firmware Server)
*************************************************************************/
void FSRV_ProduceSendMsg(fsrv_context_t* pThis)
{
  if(pThis->state==FSRV_STATE_IDLE)  /// ����
  {  return;}

  switch(pThis->state)
  {
  case FSRV_STATE_START: /// ��������(��ʼ������)
    iFSRV_start(pThis);
    break;

  case FSRV_STATE_SEND_UN:  /// ����֪ͨ����
    iFSRV_SendUnMsg(pThis);
    pThis->state = FSRV_STATE_WAIT_REQ;
    break;

  case FSRV_STATE_WAIT_REQ:  /// �ȴ�ST�������Ӧ
    break;

  case FSRV_STATE_SEND_UD:  /// ������������(��ӦST����)
    iFSRV_SendUdMsg(pThis);
    pThis->state = FSRV_STATE_WAIT_REQ;
    break;

  case FSRV_STATE_REPORT:  /// ��������ϱ��������
    iFSRV_SendUrMsg(pThis);
    pThis->state = FSRV_STATE_IDLE;
    break;

  default:
    break;
  }

  FSRV_Do100msTasks();
}

/******************************************************************************
 * ����: �����յ�4Gģ����Ϣ
 * ����: pdata-���ݰ�ָ��;  size-���ݰ�����
*******************************************************************************/
static void FSRV_ProcessRecvMsg(uint8_t *pdata, uint16_t size)
{
#if 0
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
#endif
}

/******************************************************************************
 * ��ʼ��FSRV����
*******************************************************************************/
void FSRV_Init(void)
{
  fsrv_context.state = FSRV_STATE_IDLE;
  fsrv_context.dev_id = 0x00;  // ��Ч
  fsrv_context.msg_sn = 0x00;
  fsrv_context.fw_packet_index = 0x00;
  fsrv_context.fw_packet_size = FSRV_PACKET_SIZE;
  fsrv_context.start_address = 0x00;
  fsrv_context.ending_address = 0x00;
  fsrv_context.total_block_count = 0x00;
  fsrv_context.percent = 0x00;
  fsrv_context.result = 0x00;

  fsrv_context.tx_size = 0x00;
  fsrv_context.tx_data = aux_tx_buffer;

  fsrv_context.rx_size = 0x00;
  fsrv_context.rx_data = aux_rx_buffer;

  fsrv_context.retry_sp = FSRV_RETRY_SP;
  fsrv_context.retry_cnt = 0;
  
  fsrv_context.timeout_100ms_sp = FSRV_TIMEOUT_SP;
  fsrv_context.timeout_100ms = fsrv_context.timeout_100ms_sp;
}

/******************************************************************************
 * ����UARTͨ��--���ͱ��ĸ�ST��Ƭ��
*******************************************************************************/
void* pthread_AuxComProduce(void *argument)
{
  while (1)
  {
    msleep(100); // 100msִ��һ��
    FSRV_ProduceSendMsg(&fsrv_context);  // ����������Ϣ
  }
}

/******************************************************************************
* ����UARTͨ��--����ST��Ƭ������4G�ı���
*******************************************************************************/
void* pthread_AuxComProcess(void *argument)
{
  uint16_t size; // �յ������ݳ���
  uint8_t *pdata; // ������ʼ��ַ

  while (1)
  {
    if (AuxCom_Receive(&pdata,&size)) // �ȴ�����(����)
    {
      FSRV_ProcessRecvMsg(pdata, size);
    }
  }
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceInit(void)
{
  AUX_UartInitialize(AUX_COM_BUADRATE);
  FSRV_Init();
}

/*************************************************************************
 *
*************************************************************************/
void AuxCom_ServiceStart(void)
{
  pthread_create(&pthreads[PTHREAD_AUX_COM_PROCESS_ID], NULL, pthread_AuxComProcess, NULL);
  usleep(10);
  pthread_create(&pthreads[PTHREAD_AUX_COM_PRODUCE_ID], NULL, pthread_AuxComProduce, NULL);
  usleep(10);
}

//-----�ļ�PcDebug.c����---------------------------------------------
