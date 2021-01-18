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

#define FCLIT_TIMEOUT_SP          30  // ����100ms
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
//static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

static uint8_t aux_com_buffer[USART6_RX_BUFFER_SIZE]; // AuxCom�������ݻ���

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

/*************************************************************************
 * �̼����������Ϣ
*************************************************************************/
//==����У���================================================================
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

//==��4Gģ�鷢�͹̼�����֪ͨ��Ӧ��(ST->4G)================================
void iFCLIT_SendUnRspMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x01;  //�����ʶ
  pdata[pos++] = 0x00;  // Ӧ����(0=�ɹ�, 1=��ʽ�쳣,2=�豸��֧��,3=����)
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

//==�̼�������������(ST->4G)==============================================
void iFCLIT_SendUdReqMsg(fclit_context_t* pThis)
{
  uint8_t* pdata = pThis->tx_data;
  uint16_t pos = 0;
  uint16_t msg_len;
  uint8_t check_sum;

#if FCLIT_DEBUG
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x01;  //�����ʶ
  pdata[pos++] = 0x00;  // Ӧ����(0=�ɹ�, 1=��ʽ�쳣,2=�豸��֧��,3=����)
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
  PcDebug_Printf("FclitUN!\n");
#endif

  pThis->msg_sn++;
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // ֡ͷ(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // �����
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // ��ˮ��
  pos = FCLIT_DATA_START_FIELD;

  //==��������======================================================
  pdata[pos++] = 0x01;  //�����ʶ
  pdata[pos++] = 0x00;  // Ӧ����(0=�ɹ�, 1=��ʽ�쳣,2=�豸��֧��,3=����)
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

/******************************************************************************
 * FCLIT�����Ե��ú���(����100mS,������)
*******************************************************************************/
void FCLIT_Do100msTasks(void)
{

}


/*************************************************************************
 * 10ms����һ��
*************************************************************************/
void FCLIT_ProduceSendMsg(void)
{
//  static uint8_t divide_for_300ms = 30;

}

/******************************************************************************
 * ����: �����յ�4Gģ����Ϣ
 * ����: pdata-���ݰ�ָ��;  size-���ݰ�����
*******************************************************************************/
//==�ж�MOMI��ϢУ���==========================================================
uint8_t iFCLIT_CheckMsg(uint8_t *pdata, uint16_t size)
{
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
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[FCLIT_FUNCTION_CODE_FIELD], msg_len); // ����У��ֵ:������ſ�ʼ����
  if (received_check_sum==calculated_check_sum) //check if crc's match
  {
    return FCLIT_OK;
  }
  else
  {
    return FCLIT_NOK;
  }
}

/***********************************************************************
** ��������: ����Э������Ӧ��������������
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
************************************************************************/
void Ext_Mcu_UpgradeNotice_RecVProcess(uint8* ptr, uint16 uslen)
{
  uint8 buff[50] = {0};
  static uint8 ucSN = 0;
  uint16 usSendlen = 0;

  ucSN++;
  buff[0] = 0x01;    //�����ʶ
  buff[1] = 0;       //�ɹ�
  buff[2] = 0x04;
  buff[3] = 0x00;
  usSendlen = 4;
  if (uslen<17)
    buff[1] = 1;   //��ʽ�쳣

  FirmwareUpdate.uctype = *ptr++;   //��������
  FirmwareUpdate.ucdev = *ptr++;    //����Ŀ���豸
  if (FirmwareUpdate.ucdev>3)
    buff[1] = 2;   //�豸��֧��

  FirmwareUpdate.ucSWNameLen = *ptr++;
  memcpy(FirmwareUpdate.aucSWName, ptr, FirmwareUpdate.ucSWNameLen);
  ptr += FirmwareUpdate.ucSWNameLen;
  FirmwareUpdate.ucSWVersionLen = *ptr++;
  memcpy(FirmwareUpdate.aucSWVersion, ptr, FirmwareUpdate.ucSWVersionLen);
  ptr += FirmwareUpdate.ucSWVersionLen;
  FirmwareUpdate.uiSWSize = (*ptr<<24) + (*(ptr+1)<<16)+ (*(ptr+2)<<8)+ *(ptr+3);
  ptr += 4;
  FirmwareUpdate.usPackets = (*ptr<<8) + (*(ptr+1));
  ptr += 2;
  FirmwareUpdate.uiCrc = (*ptr<<24) + (*(ptr+1)<<16)+ (*(ptr+2)<<8)+ *(ptr+3);

  Ext_McuUart_DataSend(0x04, usSendlen, buff, ucSN);
  printf("usPackets = %d\n",FirmwareUpdate.usPackets);

  if (buff[1]!=0)
    stu_ExtMCUUpgrade.ucUpgradeStep = 0;//�����˳�
  else
  {
    stu_ExtMCUUpgrade.ucUpgradeStep = 2;
    stu_ExtMCUUpgrade.usTimeOut = 600;
  }
  FirmwareUpdate.usRequestPacketSn = 0;
  stu_ExtMCUUpgrade.ucTimer = 0;
  stu_ExtMCUUpgrade.ucRepeats = 0;
}

/***********************************************************************
** ��������: ���յ�����������
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
************************************************************************/
uint8 Ext_Mcu_Upgradepackage_DownloadProcess(uint8* p, uint16 uslen)
{
  uint16 usPackSn, usTotalPacks, usPackLen;
  uint32 uiTemp;

  if (uslen<6)
    stu_ExtMCUUpgrade.ucUpgradeStep = 0; //�����˳�


  usPackSn = (*p<<8) + *(p+1);
  p += 2;
  usTotalPacks = (*p<<8) + *(p+1);
  p += 2;
  usPackLen = (*p<<8) + *(p+1);
  p += 2;

  if (usPackSn != FirmwareUpdate.usRequestPacketSn)
    return 0;
  if (usTotalPacks!= FirmwareUpdate.usPackets)
    return 0;
  if (usPackLen == 0) //?
    return 0;

  if (FirmwareUpdate.ucdev==3)
    WriteToFlash(FLASH_PAGEADDR_UPGRADE +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
  else
    WriteToFlash(FLASH_Firmware_MCU +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);

  FirmwareUpdate.usRequestPacketSn++;
  if (FirmwareUpdate.usRequestPacketSn==FirmwareUpdate.usPackets)//�������
  {
    FirmwareUpdate.usLastPacketLen = usPackLen;
    uiTemp = GetCrc32();
    if (uiTemp==FirmwareUpdate.uiCrc)
    {
      stu_ExtMCUUpgrade.ucResult = 0;
      if (FirmwareUpdate.ucdev==1)
      {
        stu_McuFirmware.ucRcvPackflag = 1;
        stu_McuFirmware.ucLoadStep = 1;
        stu_McuFirmware.usReadFlashSN = 0;
        //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
        stu_McuFirmware.usMcuUPdateTimerOut = 600;   //10����
        PC_SendDebugData((uint8 *)("MCULoad OK"), 10, DEBUG_ANYDATA);
      }
      else if (FirmwareUpdate.ucdev==4)
      {
        stu_KCMCUDownload.ucRcvPackflag = 1;
        stu_KCMCUDownload.ucLoadStep = 0xFF;
        //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
        stu_KCMCUDownload.usMcuUPdateTimerOut = 1200;   //20����
        PC_SendDebugData((uint8 *)("KCMCULoad OK"), 10, DEBUG_ANYDATA);

      }
      else
        PC_SendDebugData((uint8 *)("ST Load OK"), 10, DEBUG_ANYDATA);
    }
    else
    {
      stu_ExtMCUUpgrade.ucResult = 1;
      stu_McuFirmware.ucRcvPackflag = 0;
      stu_McuFirmware.ucLoadStep = 0;
      //stu_ExtMCUUpgrade.ucUpgradeStep = 3;
      PC_SendDebugData((uint8 *)("FM CRC ERR"), 10, DEBUG_ANYDATA);
    }
    stu_ExtMCUUpgrade.ucUpgradeStep = 3;
    FirmwareUpdate.usRequestPacketSn = 0;
    stu_ExtMCUUpgrade.ucTimer = 1;
    stu_ExtMCUUpgrade.ucRepeats = 0;
    return 0;
  }
  stu_ExtMCUUpgrade.ucUpgradeStep = 2;
  stu_ExtMCUUpgrade.ucTimer = 0;
  stu_ExtMCUUpgrade.ucRepeats = 0;
  return 0;
}

/***********************************************************************
** ��������: Ext_Mcu_Upgradepackage_Request
** ��������: ����Э������Ӧ��������������
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
************************************************************************/
void Ext_Mcu_UpgradeRequest_RecVProcess(uint8* ptr, uint16 uslen)
{

  stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //�����˳�
  if (stu_McuFirmware.ucLoadStep == 7)
  {
    stu_McuFirmware.ucLoadStep = 0;
    stu_McuFirmware.ucRcvPackflag = 0;
  }
  if (FirmwareUpdate.ucdev!=3||(memcmp(FirmwareUpdate.aucSWName,"AUX_MCU",7)!=0))
    return;

  CPU_IntDis();
  IAP_Start();            /*Enable interrupt and reset system    */
  CPU_IntEn();
}

//============================================================================
static void FCLIT_ProcessRecvMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t msg_status;
  uint16_t msg_len;
  uint16_t msg_sn;
  uint16_t pos = 0;
  uint8_t* pbuf = &pdata[FCLIT_DATA_START_FIELD];
  uint8_t command;

  msg_status = iFCLIT_CheckMsg(pdata, size);
  if (msg_status==FCLIT_OK)
  {
    msg_len = pdata[FCLIT_FRAME_SIZE_HIGH_FIELD];  // ���ݳ���
    msg_len <<= 8;
    msg_len += pdata[FCLIT_FRAME_SIZE_LOW_FIELD];
    if (msg_len < 5) // ������
    {
      return;
    }
    msg_len -= 3;  // ���ݰ��ĳ���(������������(1B)+��ˮ��(1B)+У��(1B))
    msg_sn = pdata[FCLIT_SERIAL_NUMBER_FIELD];  // ��Ϣ��ˮ��
    command = pdata[FCLIT_DATA_START_FIELD];  // �����ʶ
    switch ()
    {
    case FCLIT_COMMAND_TYPE_NOTIFIE:  // ����֪ͨ
      break;

    case FCLIT_COMMAND_TYPE_DOWNLOAD:  // ��������
      break;

    case FCLIT_COMMAND_TYPE_REPORT:  // �ϱ����
      break;

    default:
      break;
    }
  }
}

void Ext_McuUartMessage(uint8 *ptr, uint16 uslen)
{
  uint8 ucSumNum = 0;
  uint16 usDatalen = 0;
  uint8 ucCommand = 0, ucSubCommand = 0;
  uint8 ucSN,ucResult;

  if ((ptr[0]==0x7E)&&(ptr[uslen-2]==0x0D)&&(ptr[uslen-1]==0x0A)&&uslen>8)
  {

    usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
    ucCommand = ptr[3];
    ucSN = ptr[4];
    ucSubCommand = ptr[5];    //���豸��
    ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    if (ucSumNum!=ptr[uslen-3])
      return;

    if (ucCommand==0x04)
    {
      switch (ucSubCommand)
      {
      case 1:           //�յ����İ巢�͵�����֪ͨ
        Ext_Mcu_UpgradeNotice_RecVProcess(&ptr[6], usDatalen-3);
        break;
      case 2:           //�յ����İ巢�͵�����������
        Ext_Mcu_Upgradepackage_DownloadProcess(&ptr[6], usDatalen-3);
        break;
      case 3:           //�յ����İ巢�͵��������Ӧ��
        Ext_Mcu_UpgradeRequest_RecVProcess(&ptr[6], usDatalen-3);
        break;
      default:
        break;
      }
    }
    /*  else if(ucCommand==0x83)      //MCU�Ժ��İ巢�������Ӧ��
    {
          ucResult = ptr[5];       // 0-�ɹ���1-�쳣�� ʧ�ܿ��Կ����ط�
    }
    */
  }
  else   //����͸������ ���ó���
  {

  }
}


/******************************************************************************
 * �����������ݷ��ͽ���
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 100mִ��һ��
  {
    AUX_DELAY_MS(OS_TICKS_PER_SEC/10); // 100msִ��һ��
    FCLIT_ProduceSendMsg(); // ����MOMI��Ϣ
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
        FCLIT_ProcessRecvMsg(pdata, len);
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

