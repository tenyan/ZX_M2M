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
#define AuxCom_ReceiveData     USART6_ReceiveData
#define AuxCom_TransmitData    USART6_TransmitData

#define AUX_MAX_TXBUF_SIZE  USART6_TX_BUFFER_SIZE
#define AUX_MAX_RXBUF_SIZE  USART6_RX_BUFFER_SIZE
#define aux_rx_buffer       usart6_rx_buffer
#define aux_rx_size         usart6_rx_size

#define AUX_DELAY_MS(ms)   do { osDelay(ms); } while(0)

#define PART(x)     1

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
extern volatile uint16_t usart6_rx_size;
extern uint8_t usart6_rx_buffer[USART6_RX_BUFFER_SIZE];
//static uint8_t momi_tx_buffer[MOMI_MAX_TXBUF_SIZE];

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
 * ��Ϣ���ж���
 ******************************************************************************/
#define MOMI_SEND_COMMAND_ID     0x01
#define MOMI_RECV_COMMAND_ID     0x02
#define MOMI_MSG_QUEUE_MAX_SIZE  1024
#define ONE_MOMI_CMD_MAX_SIZE    256
#define MOMI_MSG_HEAD            0x7E      // ֡ͷ
#define MOMI_MSG_TAIL            0xFFFCFFFF  // ֡β

/*************************************************************************
 * 10ms����һ��
*************************************************************************/
void AuxCom_ProduceSendMsg(void)
{
  static uint8_t divide_for_300ms = 30;
  static uint8_t divide_for_500ms = 50;
  static uint8_t divide_for_1second = 100;

  if(Modem_GetState()==MODEM_STATE_DATA_READY) // ģ�鴦���ѿ���״̬
  {
    // �ն���Ϣ(������300ms)
    if(divide_for_300ms)
    {  divide_for_300ms--;}
    else
    {
      divide_for_300ms = 30;  // 300ms����
      iMomi_SendColtMsg();  // F001
    }

    // ����������Ϣ(������500ms)
    if(divide_for_500ms)
    {  divide_for_500ms--;}
    else
    {
      divide_for_500ms = 50;  // 500ms����
      iMomi_SendVehicleMsg(); // F010, F011, F012
    }

    // GPSλ����Ϣ(������1s)
    if(divide_for_1second)
    {  divide_for_1second--;}
    else
    {
      divide_for_1second = 100;  // 1s����
      //iMomi_SendGpsMsg();
    }

    // ������Ϣ(�¼���)
    // iMomi_SendDtcMsg();
    
    // CAN֡��Ϣ(�¼���)
    //iMomi_SendCanMsg();

    // �汾��Ϣ(�ϵ緢һ��)

    //iMomi_SendVersionMsg();

    // ͳ������Ϣ(ACC�ط���һ��)
    //iMomi_SendStatisticsMsg();
  }
  else
  {
    divide_for_300ms = 30;
    divide_for_500ms = 50;
    divide_for_1second = 100;
  }
}
#endif

/******************************************************************************
 * ����: �����յ�4Gģ����Ϣ
 * ����: pdata-���ݰ�ָ��;  size-���ݰ�����
*******************************************************************************/
static void AuxCom_ProcessRecvData(uint8_t *pdata, uint16_t size)
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

        // ���������Ϣ
      }
      pos += length; // ָ����һ��TLV
      tlvNum--;
    }
  }
}
#endif

/******************************************************************************
 * 4Gģ��������,�����ģ������в���
*******************************************************************************/
void AppTask_AuxComProduce(void *argument)
{
  while (1)  // 10mִ��һ��
  {
    MODEM_DELAY_MS(OS_TICKS_PER_SEC/100); // 10msִ��һ��
    AuxCom_ProduceSendMsg();  // ����MOMI��Ϣ
  }
}

/******************************************************************************
 * CAT1ģ�����ݽ��ս���
*******************************************************************************/
void AppTask_AuxComProcess(void *argument)
{
  uint8_t *pdata = NULL;
  uint16_t len;

  while (1)
  {
    if (AuxCom_ReceiveData(&pdata, &len)) // �ȴ��ź���(����)
    {
      AuxCom_ProcessRecvData(pdata, len);
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


