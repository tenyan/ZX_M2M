/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    本文件为辅助通信模块的c文件
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

// FSRV协议定义
#define FSRV_MSG_HEAD            0x7E    // 帧头
#define FSRV_MSG_TAIL            0x0D0A  // 帧尾
#define FSRV_SEND_COMMAND_ID     0x04
#define FSRV_RECV_COMMAND_ID     0x84
#define FSRV_PACKET_SIZE         RFU_BUFFER_SIZE

#define FSRV_TIMEOUT_SP          30  // 基于100ms
#define FSRV_RETRY_SP            2   // 超时重试次数


// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(1B)+数据包(NB)+校验(1B)+帧尾(2B)
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

// 递归互斥
pthread_mutex_t mid_AuxComTx;

/*****************************************************************************
 * 串口初始化函数
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
  options.c_cc[VTIME] = 1; // read timeout 单位*100ms
  options.c_cc[VMIN]  = 0;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  options.c_iflag &= ~(ICRNL | IXON);
  cfsetispeed(&options, baudrate); // 波特率设置
  cfsetospeed(&options, baudrate); // 波特率设置
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

  retVal = read(fd_auxcom, aux_rx_buffer, (AUX_UART_RX_BUFFER_SIZE-1)); // 阻塞等待数据
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
 
  retVal = write(fd_auxcom, data, size); // 写串口数据
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
//串口发送打包函数
uslen :需要发送数据体长度 (不包含Command、SN、Checksum 3个字节长度)
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

//==计算校验和================================================================
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

//==启动升级任务=============================================================
void iFSRV_start(fsrv_context_t* pThis)
{
  pThis->retry_cnt = 0;
  pThis->result = 0x00;
  pThis->timeout_100ms = pThis->timeout_100ms_sp;
  pThis->state = FSRV_STATE_SEND_UN;
}

//==发送UN信息给协处理器======================================================
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
  pdata[FSRV_FRAME_STX_FIELD] = FSRV_MSG_HEAD;  // 帧头(1B)
  pdata[FSRV_FUNCTION_CODE_FIELD] = FSRV_SEND_COMMAND_ID;  // 命令号
  pdata[FSRV_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = MOMI_DATA_START_FIELD;
  
  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = rfu_context.type;  // 升级类型
  pdata[pos++] = rfu_context.dev;  // 升级目标设备
  
  pdata[pos++] = rfu_context.file_name_length;  // 固件名称长度
  memcpy(&pdata[pos], rfu_context.file_name, rfu_context.file_name_length);  // 固件名称(ASCII)
  pos += rfu_context.file_name_length;
  
  pdata[pos++] = rfu_context.file_version_length;  // 固件版本长度
  memcpy(&pdata[pos], rfu_context.file_version, rfu_context.file_version_length);  // 固件版本(ASCII)
  pos += rfu_context.file_version_length;
  
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>24);  // 固件大小(4B)
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>16);
  pdata[pos++] = (uint8_t)(rfu_context.file_length>>8);
  pdata[pos++] = (uint8_t)(rfu_context.file_length);

  pdata[pos++] = (uint8_t)(rfu_context.total_block_count>>8);  // 固件总包数
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count);

  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>24);  // 固件CRC校验(4B)
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>16);
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val>>8);
  pdata[pos++] = (uint8_t)(rfu_context.plain_crc32val);

  pdata[pos++] = (uint8_t)(FSRV_PACKET_SIZE>>8);  // 单包大小(固定值1024)
  pdata[pos++] = (uint8_t)FSRV_PACKET_SIZE;  // 单包大小(固定值1024)
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFSRV_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FSRV_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FSRV_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FSRV_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FSRV_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==读取固件分包===============================================================
uint16_t fsrv_BuildFwData(fsrv_context_t* pThis, uint8_t* pbuf)
{
  uint16_t size = FSRV_PACKET_SIZE;

  pThis->start_address = pThis->fw_packet_index * FSRV_PACKET_SIZE;
  if ((pThis->start_address + size) > pThis->ending_address)
  {
    size = pThis->ending_address - pThis->start_address;
  }

  pbuf[0] = (uint8_t)(size>>8);  // 固件数据长度
  pbuf[1] = (uint8_t)(size);
  rfu_ReadFlashHexFile(pThis->dev_id,pThis->start_address, &pbuf[2], size); // 读取数据内容
  
  //pThis->block++;
  //pThis->percent = pThis->fw_packet_index * 100L / pThis->total_block_count;
  return (size+2);
}

//==发送UD信息给协处理器======================================================
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
  pdata[FSRV_FRAME_STX_FIELD] = FSRV_MSG_HEAD;  // 帧头(1B)
  pdata[FSRV_FUNCTION_CODE_FIELD] = FSRV_SEND_COMMAND_ID;  // 命令号
  pdata[FSRV_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = MOMI_DATA_START_FIELD;
  
  //==数据内容======================================================
  pdata[pos++] = 0x02;  //命令标识
  pdata[pos++] = (uint8_t)(pThis->fw_packet_index>>8);  //当前升级包序号
  pdata[pos++] = (uint8_t)pThis->fw_packet_index;
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count>>8);  // 固件总包数
  pdata[pos++] = (uint8_t)(rfu_context.total_block_count);
  
  msg_len = fsrv_BuildFwData(pThis, &pdata[pos]);  // 读取文件数据
  pos += msg_len;
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFSRV_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FSRV_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FSRV_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FSRV_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FSRV_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==向平台上报升级结果(终端->平台)==============================================
uint16_t iFSRV_SendUrMsg(fsrv_context_t* pThis)
{
  uint16_t msg_len, msg_body_len, msg_header_len;
  uint8_t check_sum;
  uint8_t* pbuf = m2m_context.tx_data;
  uint16_t len;

#if FSRV_DEBUG
  PcDebug_Printf("FsrvUR:%d!\n", pThis->result);
#endif

  //==创建报文体==========================================
  len = M2M_MSG_HEAD_LEN;

  //==命令类型========
  pbuf[len++] = 0x00;  // 命令类型长度
  pbuf[len++] = 0x02;
  pbuf[len++] = 'U';  // 命令
  pbuf[len++] = 'R';

  //==命令内容========
  pbuf[len++] = pThis->result; // 升级结果
  msg_body_len = len - M2M_MSG_HEAD_LEN;

  //==创建报文头==========================================
  m2m_context.upload_sn++;
  msg_header_len = im2m_BuildMsgHead(pbuf, M2M_MSG_TYPE_UPDATE, msg_body_len, 0, m2m_context.upload_sn); // M2mUploadSN

  //==计算校验字==========================================
  msg_len = msg_body_len + msg_header_len;
  check_sum = im2m_CalcSumCheck(pbuf, msg_len);
  pbuf[msg_len++] = check_sum;
  m2m_context.tx_size = msg_len;
  im2m_SendNetData(m2m_context.tx_data, m2m_context.tx_size); // 平台

  return msg_len;
}

/******************************************************************************
 * FSRV周期性调用函数(周期100mS,无阻塞)
*******************************************************************************/
void FSRV_Do100msTasks(void)
{
  // 超时判断
  if(fsrv_context.timeout_100ms)
    fsrv_context.timeout_100ms--;
  else
  {
#if FSRV_DEBUG
    PcDebug_Printf("FsrvTO!\n");
#endif
    fsrv_context.timeout_100ms = fsrv_context.timeout_100ms_sp;
    fsrv_context.retry_cnt++;
    if(fsrv_context.retry_cnt < fsrv_context.retry_sp) // 重试次数
    {
      fsrv_context.state = FSRV_STATE_SEND_UN;  // 重新发送升级通知
    }
    else
    {
      fsrv_context.state = FSRV_STATE_REPORT;  // 上报升级失败结果

      if(fsrv_context.dev_id==3)  // 协处理器
      {  fsrv_context.result = 0x01;}
      else if(fsrv_context.dev_id==1)  // 外部ECU
      {  fsrv_context.result = 0x01;}  // 4=控制器升级失败
      else
      {  fsrv_context.state = FSRV_STATE_IDLE;}  // 其他情况退出升级
    }
  }
}

/*************************************************************************
 * 100ms调用一次(固件服务器FSRV = Firmware Server)
*************************************************************************/
void FSRV_ProduceSendMsg(fsrv_context_t* pThis)
{
  if(pThis->state==FSRV_STATE_IDLE)  /// 空闲
  {  return;}

  switch(pThis->state)
  {
  case FSRV_STATE_START: /// 启动升级(初始化参数)
    iFSRV_start(pThis);
    break;

  case FSRV_STATE_SEND_UN:  /// 发送通知升级
    iFSRV_SendUnMsg(pThis);
    pThis->state = FSRV_STATE_WAIT_REQ;
    break;
    
  case FSRV_STATE_WAIT_REQ:  /// 等待ST请求和响应
    break;
    
  case FSRV_STATE_SEND_UD:  /// 发送升级数据(响应ST请求)
    iFSRV_SendUdMsg(pThis);
    pThis->state = FSRV_STATE_WAIT_REQ;
    break;
    
  case FSRV_STATE_REPORT:  /// 向服务器上报升级结果
    iFSRV_SendUrMsg(pThis);
    pThis->state = FSRV_STATE_IDLE;
    break;

  default:
    break;
  }

  FSRV_Do100msTasks();
}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
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
    msg_len = pdata[MOMI_FRAME_SIZE_HIGH_FIELD];  // 数据长度
    msg_len <<= 8;
    msg_len += pdata[MOMI_FRAME_SIZE_LOW_FIELD];
    if(msg_len < 5)  // 无数据
    {
      return;
    }
    msg_len -= 4;  // 数据包的长度(不包含功能码(1B)+流水号(2B)+校验(1B))
    msg_sn = pdata[MOMI_SN_HIGH_FIELD];  // 消息流水号
    msg_sn <<= 8;
    msg_sn += pdata[MOMI_SN_LOW_FIELD];

    pos = 0;
    tlvNum = pbuf[pos++];  // TLV个数
    while (tlvNum) // TLV解析
    {
      if (pos >= msg_len) // 长度判断
      {
        break; // 长度错误,退出循环
      }
      
      //==获取TAG==================================
      tag = pbuf[pos++];
      tag <<= 8;
      tag += pbuf[pos++];

      //==获取LENGTH===============================
      length = pbuf[pos++];
      length <<= 8;
      length += pbuf[pos++];

      //==获取VALUE================================
      retVal = iMomi_AnalyzeTlvData(tag, length, &pbuf[pos]); // 解析
      if(MOMI_NOK==retVal)
      {
        fail_tag_num++;
        PcDebug_Printf("MomiErr:Tlv=%x\n", tag); // 输出16进制
        // 输出调试信息
      }
      pos += length; // 指向下一个TLV
      tlvNum--;
    }
  }
#endif
}

/******************************************************************************
 * 初始化FSRV参数
*******************************************************************************/
void FSRV_Init(void)
{
  fsrv_context.state = FSRV_STATE_IDLE;
  fsrv_context.dev_id = 0x00;  // 无效
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
 * 辅助UART通信--发送报文给ST单片机
*******************************************************************************/
void* pthread_AuxComProduce(void *argument)
{
  while (1)
  {
    msleep(100); // 100ms执行一次
    FSRV_ProduceSendMsg(&fsrv_context);  // 发送升级消息
  }
}

/******************************************************************************
* 辅助UART通信--处理ST单片机发给4G的报文
*******************************************************************************/
void* pthread_AuxComProcess(void *argument)
{
  uint16_t size; // 收到的数据长度
  uint8_t *pdata; // 数据起始地址

  while (1)
  {
    if (AuxCom_Receive(&pdata,&size)) // 等待数据(阻塞)
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

//-----文件PcDebug.c结束---------------------------------------------

