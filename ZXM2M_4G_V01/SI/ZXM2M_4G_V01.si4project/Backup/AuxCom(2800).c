/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    本文件为辅助通信模块的c文件
 ********************************************************************************/
#include "config.h"

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
static uint8_t aux_tx_buffer[AUX_COM_BUFFER_MAX_SIZE]; // PcDebug任务数据缓存

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
void AuxCom_SendData(uint8 ucCommand, uint16 uslen, uint8* pdata,uint8 ucSN)
{
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
  //PC_SendDebugData(SendBuff, 8+uslen, DEBUG_ANYDATA);
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

//==发送UN信息给协处理器======================================================
void iFSRV_SendUnMsg(fsrv_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

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

  pdata[pos++] = FSRV_PACKET_SIZE;  // 单包大小(固定值1024)
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

/* ================================================================== */
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


  uint8 buff[1200] = {0};
  static uint8 ucSN = 0;
  uint16 uslen = 0;

  ucSN++;
  buff[0] = 0x02;    //命令标识
  buff[1] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn>>8);   //当前升级包序号
  buff[2] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn&0xFF); //
  buff[3] = (uint8)(FirmwareUpdate.usPackets>>8);           //升级总包数
  buff[4] = (uint8)(FirmwareUpdate.usPackets&0xFF);
  /*
  if(FirmwareUpdate.usRequestPacketSn==0)
  {
     //关闭升级文件
  }
  */
  if (stu_ExtMCUUpgrade.usRequestPacketSn==FirmwareUpdate.usPackets-1)
    uslen = FirmwareUpdate.usLastPacketLen;
  else
    uslen = 1024;
  
  buff[5] = (uint8)(uslen>>8);
  buff[6] = (uint8)(uslen&0xFF);
  // ReadFromWholFileFlash(FLASH_FIRMWARE_Auxi_MCU, FIRMWARE_PACKET_LEN, &buff[7]);
  if (FirmwareUpdate.ucdev==3) //协处理器
    ReadFromlseekFlash(FLASH_FIRMWARE_Auxi_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]);
  else if (FirmwareUpdate.ucdev==1)  //车辆控制器
    ReadFromlseekFlash(FLASH_FIRMWARE_Vehicle_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]);

  AuxCom_SendData(0x04, 7+uslen, buff, ucSN);
}

//==向平台上报升级结果====================================================
void iFSRV_SendUrMsg(fsrv_context_t* pThis)
{
  uint8 *p = &aMsgSendData[MSG_HEAD_LEN];
  uint16 usTemp;

  if (!g_stuSystem.ucOnline)
    return;

  *p++ = 0;
  *p++ = 2;
  *p++ = 'U';
  *p++ = 'R';
  *p++ = stu_ExtMCUUpgrade.ucResult;
  /*
  	if(stu_McuFirmware.ucUploadResult==0)
  	    *p++ = 4;
  	else if(stu_McuFirmware.ucUploadResult==1)
  		*p++ = 2;
  */
  f_usUpLoadCmdSn++;
  usTemp = BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, 5, 0, f_usUpLoadCmdSn);
  usTemp += 5;
  aMsgSendData[usTemp] = SumCalc(aMsgSendData, usTemp);

  GSM_SendGprs(aMsgSendData, usTemp+1, 0);
}

/*************************************************************************
 * 10ms调用一次(固件服务器FSRV = Firmware Server)
*************************************************************************/
void FSRV_ProduceSendMsg(fsrv_context_t* pThis)
{
  static uint8_t divide_for_300ms = 30;

  if(pThis->state==FSRV_STATE_IDLE)  /// 空闲
  {  return;}

  switch(pThis->state)
  {
  case FSRV_STATE_SEND_UN:  /// 发送通知升级
    iFSRV_SendUnMsg(pThis);
    break;
    
  case FSRV_STATE_WAIT_REQ:  /// 等待ST请求和响应
    break;
    
  case FSRV_STATE_SEND_UD:  /// 发送升级数据(响应ST请求)
    iFSRV_SendUdMsg(pThis);
    break;
    
  case FSRV_STATE_REPORT_REQ:  /// 向服务器上报升级结果
    iFSRV_SendUrMsg(pThis);
    break;

  default:
    break;
  }

  if(pThis->rsp_timer)
    pThis->rsp_timer--;
}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
static void FSRV_ProcessRecvMsg(uint8_t *pdata, uint16_t size)
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
}

/******************************************************************************
 * 辅助UART通信--发送报文给ST单片机
*******************************************************************************/
void* pthread_AuxComProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10ms执行一次
    FSRV_ProduceSendMsg();  // 发送升级消息
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

