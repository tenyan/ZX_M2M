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

#define FSRV_TIMEOUT_SP          50  // 基于100ms
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
  pos = FSRV_DATA_START_FIELD;
  
  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = rfu_context.type;  // 升级类型
  pdata[pos++] = rfu_context.dev;  // 升级目标设备
  pThis->dev_id = rfu_context.dev;
  
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

  pdata[pos++] = (uint8_t)(pThis->fw_block_size>>8);  // 单包大小(固定值1024)
  pdata[pos++] = (uint8_t)pThis->fw_block_size;
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFSRV_CalcSumCheck(&pdata[FSRV_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
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
  uint16_t size = pThis->fw_block_size;

  pThis->start_address = pThis->fw_packet_index * pThis->fw_block_size;
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
  pos = FSRV_DATA_START_FIELD;

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
  check_sum = iFSRV_CalcSumCheck(&pdata[FSRV_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FSRV_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FSRV_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FSRV_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FSRV_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==向平台上报升级结果(终端->平台)=============================================
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
      {  fsrv_context.result = 0x04;}  // 4=控制器升级失败
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

#if (PART("接收FSRV数据"))
//==判断MOMI信息校验和==========================================================
uint8_t iFSRV_CheckMsg(fsrv_context_t* pThis)
{
  uint8_t *pdata = pThis->rx_data;
  uint16_t size = pThis->rx_size;
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;
  uint16_t msg_tail;

  if (pdata[0] != FSRV_MSG_HEAD) // 帧头错误
  {
    return FSRV_NOK;
  }

  msg_tail = (uint16_t)(pdata[size-2]<<8) + pdata[size-1]; // 获取帧尾
  if (msg_tail != FSRV_MSG_TAIL) // 帧尾错误
  {
    return FSRV_NOK;
  }

  if (pdata[3] != FSRV_RECV_COMMAND_ID) // 命令字段错误
  {
    return FSRV_NOK;
  }

  msg_len = size-6; // 不含帧头(1B)+数据长度(2B)+检验和(1B)+帧尾(2B)
  received_check_sum = pdata[size-3]; // 接收到的检验和
  calculated_check_sum = iFSRV_CalcSumCheck(&pdata[FSRV_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
  if (received_check_sum==calculated_check_sum) //check if crc's match
  {
    return FSRV_OK;
  }
  else
  {
    return FSRV_NOK;
  }
}

//==解析升级通知指令(4G->ST)===================================================
void iFSRV_AnalyzeUnRspMsg(fsrv_context_t* pThis)
{
  uint8_t command;

  if(pThis->state==FSRV_STATE_WAIT_REQ)
  {
    pThis->retry_cnt = 0x00;  // 复位重试计数器
    pThis->timeout_100ms = pThis->timeout_100ms_sp;  // 复位超时计时器
  }
  else
  {
    pThis->state = FSRV_STATE_IDLE;
  }
}

//pThis->retry_cnt = 0x00;  // 重试计数器
//pThis->timeout_100ms = pThis->timeout_100ms_sp;  // 复位超时计时器
//pThis->msg_sn = 0x00;  // 清零流水号

//==解析升级下载响应指令(4G->ST)===================================================
void iFSRV_AnalyzeUdReqMsg(fsrv_context_t* pThis)
{
  uint8_t *pdata;
  uint16_t pos;
  uint16_t current_block_index;  // 当前包序

  //==数据内容======================================================
  pos = 0x00;
  pdata = &pThis->rx_data[FSRV_DATA_START_FIELD];
  pos++;  // 命令标识

  current_block_index = pdata[pos++];  //==当前包序列号
  current_block_index <<= 8;
  current_block_index += pdata[pos++];
  
  pThis->fw_packet_index = current_block_index;
  pThis->timeout_100ms = pThis->timeout_100ms_sp;  // 复位超时计时器
}

//==解析升级上报请求指令(ST->4G)==============================================
void iFSRV_AnalyzeUrReqMsg(fsrv_context_t* pThis)
{
  if (0==pThis->ud_result)  // 固件下载成功
  {
    pThis->state = FCLIT_STATE_IDLE;
    if ((pThis->dev_id==0x03) || (memcmp(rfu_context.file_name, "ZXM2M_ST", 8)==0))
    {
      Tbox_SetMachineState(TBOX_STATE_IAP); // T-BOX进入升级模式
    }
    // 在此处增加对外部控制器或显示器升级处理
  }
}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void FSRV_ProcessRecvMsg(fsrv_context_t *pThis)
{
  uint8_t msg_status;
  uint8_t command;

  msg_status = iFSRV_CheckMsg(pThis);
  if (msg_status==FSRV_OK)
  {
    command = pThis->rx_data[FSRV_DATA_START_FIELD];  // 命令标识
    switch (command)
    {
    case FSRV_COMMAND_TYPE_NOTIFIE:  // 升级通知
      iFSRV_AnalyzeUnRspMsg(pThis);
      break;

    case FSRV_COMMAND_TYPE_DOWNLOAD:  // 下载数据
      iFSRV_AnalyzeUdReqMsg(pThis);
      break;

    case FSRV_COMMAND_TYPE_REPORT:  // 上报结果
      iFSRV_AnalyzeUrReqMsg(pThis);
      break;

    default:
      break;
    }
  }
}
#endif

/******************************************************************************
 * 初始化FSRV参数
*******************************************************************************/
void FSRV_Init(void)
{
  fsrv_context.state = FSRV_STATE_IDLE;
  fsrv_context.dev_id = 0x00;  // 无效
  fsrv_context.msg_sn = 0x00;
  fsrv_context.fw_packet_index = 0x00;
  fsrv_context.start_address = 0x00;
  fsrv_context.ending_address = 0x00;
  fsrv_context.fw_block_size = FSRV_PACKET_SIZE;
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
      fsrv_context.rx_data = pdata;
      fsrv_context.rx_size = len;
      FSRV_ProcessRecvMsg(&fsrv_context);
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

