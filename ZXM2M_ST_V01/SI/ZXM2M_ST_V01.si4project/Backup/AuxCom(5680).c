/*****************************************************************************
* @FileName: AuxCom.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-12
* @brief     Modem模块和Micro辅助通信协议实现
******************************************************************************/
//-----头文件调用------------------------------------------------------------
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


// FSRV协议定义
#define FCLIT_MSG_HEAD            0x7E    // 帧头
#define FCLIT_MSG_TAIL            0x0D0A  // 帧尾
#define FCLIT_SEND_COMMAND_ID     0x84
#define FCLIT_RECV_COMMAND_ID     0x04
#define FCLIT_PACKET_SIZE         RFU_BUFFER_SIZE

#define FCLIT_TIMEOUT_SP          30  // 基于100ms
#define FCLIT_RETRY_SP            2   // 超时重试次数

// 帧头(1B)+数据长度(2B)+功能码(1B)+流水号(1B)+数据包(NB)+校验(1B)+帧尾(2B)
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

static uint8_t aux_com_buffer[USART6_RX_BUFFER_SIZE]; // AuxCom任务数据缓存

/******************************************************************************
 * RTOS相关
 ******************************************************************************/
//=消费者==============================================================
#define AppThreadPriority_AuxComProcess   osPriorityHigh2
osThreadId_t tid_AuxComProcess;

const osThreadAttr_t AppThreadAttr_AuxComProcess =
{
  .priority = AppThreadPriority_AuxComProcess,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // 字节
};

//=生产者==============================================================
#define AppThreadPriority_AuxComProduce   osPriorityHigh1
osThreadId_t tid_AuxComProduce;

const osThreadAttr_t AppThreadAttr_AuxComProduce =
{
  .priority = AppThreadPriority_AuxComProduce,
  .attr_bits = osThreadDetached,
  .stack_size = 1024, // 字节
};

/*************************************************************************
 * 固件升级相关消息
*************************************************************************/
//==计算校验和================================================================
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

//==向4G模块发送固件升级通知的应答(ST->4G)================================
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
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

//==固件升级下载请求(ST->4G)==============================================
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
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);

}

//==向4G上报升级结果(ST->4G)==============================================
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
  pdata[FCLIT_FRAME_STX_FIELD] = FCLIT_MSG_HEAD;  // 帧头(1B)
  pdata[FCLIT_FUNCTION_CODE_FIELD] = FCLIT_SEND_COMMAND_ID;  // 命令号
  pdata[FCLIT_SERIAL_NUMBER_FIELD] = pThis->msg_sn;  // 流水号
  pos = FCLIT_DATA_START_FIELD;

  //==数据内容======================================================
  pdata[pos++] = 0x01;  //命令标识
  pdata[pos++] = 0x00;  // 应答结果(0=成功, 1=格式异常,2=设备不支持,3=其他)
  //===============================================================

  msg_len = pos - 3; // 不含帧头和数据长度
  check_sum = iFCLIT_CalcSumCheck(&pdata[3], msg_len); // 计算校验值:从命令号开始计算
  pdata[pos++] = check_sum;  // 校验和

  msg_len = pos - 3; // 不含帧头和数据长度
  pdata[FCLIT_FRAME_SIZE_HIGH_FIELD] = (msg_len>>8) & 0xFF; // 长度
  pdata[FCLIT_FRAME_SIZE_LOW_FIELD] = msg_len & 0xFF;

  pdata[pos++] = (FCLIT_MSG_TAIL>>8) & 0xFF;  // 帧尾(2B)
  pdata[pos++] = FCLIT_MSG_TAIL & 0xFF;

  AuxCom_Transmit(pdata, pos);
}

/******************************************************************************
 * FCLIT周期性调用函数(周期100mS,无阻塞)
*******************************************************************************/
void FCLIT_Do100msTasks(void)
{

}


/*************************************************************************
 * 10ms调用一次
*************************************************************************/
void FCLIT_ProduceSendMsg(void)
{
//  static uint8_t divide_for_300ms = 30;

}

/******************************************************************************
 * 功能: 处理收到4G模块消息
 * 输入: pdata-数据包指针;  size-数据包长度
*******************************************************************************/
//==判断MOMI信息校验和==========================================================
uint8_t iFCLIT_CheckMsg(uint8_t *pdata, uint16_t size)
{
  uint8_t received_check_sum;
  uint8_t calculated_check_sum;
  uint16_t msg_len;
  uint16_t msg_tail;

  if (pdata[0] != FCLIT_MSG_HEAD) // 帧头错误
  {
    return FCLIT_NOK;
  }

  msg_tail = (uint16_t)(pdata[size-2]<<8) + pdata[size-1]; // 获取帧尾
  if (msg_tail != FCLIT_MSG_TAIL) // 帧尾错误
  {
    return FCLIT_NOK;
  }

  if (pdata[3] != FCLIT_RECV_COMMAND_ID) // 命令字段错误
  {
    return FCLIT_NOK;
  }

  msg_len = size-6; // 不含帧头(1B)+数据长度(2B)+检验和(1B)+帧尾(2B)
  received_check_sum = pdata[size-3]; // 接收到的检验和
  calculated_check_sum = iMomi_CalcSumCheck(&pdata[FCLIT_FUNCTION_CODE_FIELD], msg_len); // 计算校验值:从命令号开始计算
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
** 功能描述: 处理协处理器应答升级请求命令
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
************************************************************************/
void Ext_Mcu_UpgradeNotice_RecVProcess(uint8* ptr, uint16 uslen)
{
  uint8 buff[50] = {0};
  static uint8 ucSN = 0;
  uint16 usSendlen = 0;

  ucSN++;
  buff[0] = 0x01;    //命令标识
  buff[1] = 0;       //成功
  buff[2] = 0x04;
  buff[3] = 0x00;
  usSendlen = 4;
  if (uslen<17)
    buff[1] = 1;   //格式异常

  FirmwareUpdate.uctype = *ptr++;   //升级类型
  FirmwareUpdate.ucdev = *ptr++;    //升级目标设备
  if (FirmwareUpdate.ucdev>3)
    buff[1] = 2;   //设备不支持

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
    stu_ExtMCUUpgrade.ucUpgradeStep = 0;//升级退出
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
** 功能描述: 接收到升级包数据
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
************************************************************************/
uint8 Ext_Mcu_Upgradepackage_DownloadProcess(uint8* p, uint16 uslen)
{
  uint16 usPackSn, usTotalPacks, usPackLen;
  uint32 uiTemp;

  if (uslen<6)
    stu_ExtMCUUpgrade.ucUpgradeStep = 0; //升级退出


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
  if (FirmwareUpdate.usRequestPacketSn==FirmwareUpdate.usPackets)//传输结束
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
        stu_McuFirmware.usMcuUPdateTimerOut = 600;   //10分钟
        PC_SendDebugData((uint8 *)("MCULoad OK"), 10, DEBUG_ANYDATA);
      }
      else if (FirmwareUpdate.ucdev==4)
      {
        stu_KCMCUDownload.ucRcvPackflag = 1;
        stu_KCMCUDownload.ucLoadStep = 0xFF;
        //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
        stu_KCMCUDownload.usMcuUPdateTimerOut = 1200;   //20分钟
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
** 函数名称: Ext_Mcu_Upgradepackage_Request
** 功能描述: 处理协处理器应答升级请求命令
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
************************************************************************/
void Ext_Mcu_UpgradeRequest_RecVProcess(uint8* ptr, uint16 uslen)
{

  stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //升级退出
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
    msg_len = pdata[FCLIT_FRAME_SIZE_HIGH_FIELD];  // 数据长度
    msg_len <<= 8;
    msg_len += pdata[FCLIT_FRAME_SIZE_LOW_FIELD];
    if (msg_len < 5) // 无数据
    {
      return;
    }
    msg_len -= 3;  // 数据包的长度(不包含功能码(1B)+流水号(1B)+校验(1B))
    msg_sn = pdata[FCLIT_SERIAL_NUMBER_FIELD];  // 消息流水号
    command = pdata[FCLIT_DATA_START_FIELD];  // 命令标识
    switch ()
    {
    case FCLIT_COMMAND_TYPE_NOTIFIE:  // 升级通知
      break;

    case FCLIT_COMMAND_TYPE_DOWNLOAD:  // 下载数据
      break;

    case FCLIT_COMMAND_TYPE_REPORT:  // 上报结果
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
    ucSubCommand = ptr[5];    //子设备号
    ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    if (ucSumNum!=ptr[uslen-3])
      return;

    if (ucCommand==0x04)
    {
      switch (ucSubCommand)
      {
      case 1:           //收到核心板发送的升级通知
        Ext_Mcu_UpgradeNotice_RecVProcess(&ptr[6], usDatalen-3);
        break;
      case 2:           //收到核心板发送的升级包数据
        Ext_Mcu_Upgradepackage_DownloadProcess(&ptr[6], usDatalen-3);
        break;
      case 3:           //收到核心板发送的升级结果应答
        Ext_Mcu_UpgradeRequest_RecVProcess(&ptr[6], usDatalen-3);
        break;
      default:
        break;
      }
    }
    /*  else if(ucCommand==0x83)      //MCU对核心板发送命令的应答
    {
          ucResult = ptr[5];       // 0-成功；1-异常。 失败可以考虑重发
    }
    */
  }
  else   //考虑透传数据 设置程序
  {

  }
}


/******************************************************************************
 * 辅助串口数据发送进程
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 100m执行一次
  {
    AUX_DELAY_MS(OS_TICKS_PER_SEC/10); // 100ms执行一次
    FCLIT_ProduceSendMsg(); // 发送MOMI消息
  }
}

/******************************************************************************
 * 处理通过辅助串口发来的M2M透传指令
*******************************************************************************/
void AuxCom_ProcessM2mMsg(uint8_t *pdata, uint16_t size)
{
  if (pdata[0]==0x05 && (size>=10))
  {
    memcpy(aux_com_buffer, pdata, size);
    SbusMsg_AuxCom.data_size = size;  // 数据内容长度
    SbusMsg_AuxCom.data = aux_com_buffer;  // 获取数据地址
    SYSBUS_PutMbox(SbusMsg_AuxCom);  // 发送消息
  }
}

/******************************************************************************
 * 辅助串口数据接收进程
*******************************************************************************/
void AppTask_AuxComProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (AuxCom_Receive(&pdata, &len)) // 等待信号量(阻塞)
    {
      if (pdata[0]==0x05) // 透传M2M命令请求CMD_REQ指令
      {
        if ( (pdata[15]=='P' && pdata[16]=='W') || (pdata[15]=='R' && pdata[16]=='C') ) // PW指令和RC指令
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

//-----文件AuxCom.c结束---------------------------------------------

