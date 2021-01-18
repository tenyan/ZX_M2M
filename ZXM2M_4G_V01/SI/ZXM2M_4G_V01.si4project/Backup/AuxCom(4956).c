/********************************************************************************
 * Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
 * @FileName: AuxCom.c
 * @Engineer: TenYan
 * @version   V1.0
 * @Date:     2021-1-8
 * @brief:    ���ļ�Ϊ����ͨ��ģ���c�ļ�
 ********************************************************************************/
#include "config.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
#define AUX_UART_RX_BUFFER_SIZE  1500

#define AUX_COM_BUADRATE         B115200
#define AUX_COM_BUFFER_MAX_SIZE  AUX_UART_RX_BUFFER_SIZE

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
 int fd_auxcom = -1;
uint8_t aux_uart_rx_buffer[AUX_UART_RX_BUFFER_SIZE];
static uint8_t aux_com_buffer[AUX_COM_BUFFER_MAX_SIZE]; // PcDebug�������ݻ���

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

  retVal = read(fd_auxcom, aux_uart_rx_buffer, (AUX_UART_RX_BUFFER_SIZE-1)); // �����ȴ�����
  if (retVal > 0)
  {
    *data = dbg_uart_rx_buffer;
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
//==����NU��Ϣ��Э������======================================================
void iFSRV_SendUnMsg(fsrv_context_t* pThis)
{
  uint8 buff[100] = {0};
  static uint8 ucSN = 0;
  uint8 uclen = 0;

  ucSN++;
  buff[0] = 0x01;    //�����ʶ
  buff[1] = FirmwareUpdate.uctype;   //��������
  buff[2] = FirmwareUpdate.ucdev;//����Ŀ���豸
  buff[3] = FirmwareUpdate.ucSWNameLen;
  memcpy(&buff[4],FirmwareUpdate.aucSWName,FirmwareUpdate.ucSWNameLen);
  uclen = FirmwareUpdate.ucSWNameLen;
  buff[4+uclen] = FirmwareUpdate.ucSWVersionLen;
  memcpy(&buff[5+uclen],FirmwareUpdate.aucSWVersion,FirmwareUpdate.ucSWVersionLen);
  uclen += FirmwareUpdate.ucSWVersionLen;
  buff[5+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>24);
  buff[6+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>16);
  buff[7+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>8);
  buff[8+uclen] = (uint8)(FirmwareUpdate.uiSWSize&0xFF);
  buff[9+uclen] = (uint8)(FirmwareUpdate.usPackets>>8);
  buff[10+uclen] = (uint8)(FirmwareUpdate.usPackets&0xFF);
  buff[11+uclen] = (uint8)(FirmwareUpdate.uiCrc>>24);
  buff[12+uclen] = (uint8)(FirmwareUpdate.uiCrc>>16);
  buff[13+uclen] = (uint8)(FirmwareUpdate.uiCrc>>8);
  buff[14+uclen] = (uint8)(FirmwareUpdate.uiCrc&0xFF);

  A5Uart0_DataSend(0x04, 15+uclen, buff, ucSN);
}


//��MCU�������������� (��ʼ��ȡ�ļ�����ǰȷ���ļ��Ѿ��ر�״̬)
//�����ظ���ȡ�Ŀ���?
void iFSRV_SendUdMsg(fsrv_context_t* pThis)
{
  uint8 buff[1200] = {0};
  static uint8 ucSN = 0;
  uint16 uslen = 0;

  ucSN++;
  buff[0] = 0x02;    //�����ʶ
  buff[1] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn>>8);   //��ǰ���������
  buff[2] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn&0xFF); //
  buff[3] = (uint8)(FirmwareUpdate.usPackets>>8);           //�����ܰ���
  buff[4] = (uint8)(FirmwareUpdate.usPackets&0xFF);
  /*
  if(FirmwareUpdate.usRequestPacketSn==0)
  {
     //�ر������ļ�
  }
  */
  if (stu_ExtMCUUpgrade.usRequestPacketSn==FirmwareUpdate.usPackets-1)
    uslen = FirmwareUpdate.usLastPacketLen;
  else
    uslen = 1024;
  buff[5] = (uint8)(uslen>>8);
  buff[6] = (uint8)(uslen&0xFF);
  // ReadFromWholFileFlash(FLASH_FIRMWARE_Auxi_MCU, FIRMWARE_PACKET_LEN, &buff[7]);
  if (FirmwareUpdate.ucdev==3) //Э������
    ReadFromlseekFlash(FLASH_FIRMWARE_Auxi_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]);
  else if (FirmwareUpdate.ucdev==1)  //����������
    ReadFromlseekFlash(FLASH_FIRMWARE_Vehicle_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]);

  A5Uart0_DataSend(0x04, 7+uslen, buff, ucSN);
}

//==��ƽ̨�ϱ��������====================================================
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
 * 10ms����һ��(�̼�������FSRV = Firmware Server)
*************************************************************************/
void FSRV_ProduceSendMsg(fsrv_context_t* pThis)
{
  static uint8_t divide_for_300ms = 30;

  if(pThis->state==FSRV_STATE_IDLE)  /// ����
  {  return;}

  switch(pThis->state)
  {
  case FSRV_STATE_SEND_UN:  /// ����֪ͨ����
    break;
    
  case FSRV_STATE_WAIT_REQ:  /// �ȴ�ST�������Ӧ
    break;
    
  case FSRV_STATE_SEND_UD:  /// ������������(��ӦST����)
    break;
    
  case FSRV_STATE_REPORT_REQ:  /// ��������ϱ��������
    break;

  default:
    break;
  }

  if(pThis->rsp_timer)
    pThis->rsp_timer--;
}

/******************************************************************************
 * ����: �����յ�4Gģ����Ϣ
 * ����: pdata-���ݰ�ָ��;  size-���ݰ�����
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

/******************************************************************************
 * ����UARTͨ��--���ͱ��ĸ�ST��Ƭ��
*******************************************************************************/
void* pthread_AuxComProduce(void *argument)
{
  while (1)
  {
    msleep(10); // 10msִ��һ��
    FSRV_ProduceSendMsg();  // ����������Ϣ
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

