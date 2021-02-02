/*****************************************************************************
* @FileName: BlindZone.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version:  V1.0
* @Date:     2021-1-19
* @brief:    ä�����ݲ�������
*
* ���ԭ��
* ��Ϣ����=������(�ϳ����³�����Ч��λ)�����ݰ�
* ʵ�ʹ����У��ն��ڲ�4Gģ��������վ�л��������źŲ��ã��ᵼ�������жϣ��޷��Ѳɼ�������
* ���͵�Զ�˷��������ն���Ҫ��������Ϣ���浽���ش洢���ϣ��ȴ������ɹ��󣬽���������Ϣ����
* �ϴ�����������
* ����豸ʵ�������ä����ʱ�䳤���ǲ���֪�������ݳ���Ҳ��δ֪�ġ����ǱȽϹ������µ����ݣ�
* ���洢�����޵ģ�������Ҫ�ڴ������ݱ����ʱ�򣬰Ѿɵ����ݲ�ͣ�ĸ��ǵ���ֻ�������µ����ݣ�
* ���ҵ�һʱ���ϱ����µ����ݣ�����һ��������(First Input Last Output)�Ĵ洢��ʽ��
* ����һ������������ѭ������(ջ�׿ɱ仯)��ջ�����ڻ���ÿһ������֡�ĳ��ȡ�
* ���ջ����洢��һ��PARA�ļ��У������У��ֵ��
* ����֡���洢��һ��DATA�ļ��У��ļ�Ϊÿ������֡����̶�����N Bytes,
* ÿ������֡�洢����ʼ��ַ����ջ��ֵ���Թ̶����ȡ�
******************************************************************************/
#include "BlindZone.h"

/******************************************************************************
* Macros
******************************************************************************/
#define BLIND_ZONE_PARA_HEADER  0x55AA5AA5

/******************************************************************************
 *   Data Types
 ******************************************************************************/


/*************************************************************************
 * �������У��ֵ
*************************************************************************/
uint8_t BlindZone_CalcXorCheck(uint8_t *buffer, uint16_t size)
{
  uint8_t sum = 0;
  uint16_t i;

  for (i = 0; i < size; i++)
  {
    sum ^= buffer[i];
  }
  return sum;
}

#if (PART("FILO����"))
/******************************************************************************
* ����ä��������FLASH
******************************************************************************/
void BlindZone_SaveParameter(const char *file_name, blind_zone_para_t* pThis)
{
  __blind_zone_para_t* p_blind_zone_para = (__blind_zone_para_t*) pThis;

  pThis->header = BLIND_ZONE_PARA_HEADER;
  pThis->crc = BlindZone_CalcXorCheck(p_blind_zone_para->chMask, (BLIND_ZONE_PARA_SIZE - 1));  // ����CRC
  FileIO_Write(file_name, p_blind_zone_para->chMask, BLIND_ZONE_PARA_SIZE);
}

/******************************************************************************
* ��FLASH�ж�ȡä������
******************************************************************************/
void BlindZone_ReadParameter(const char *file_name, blind_zone_para_t* pThis)
{
  const char *zxm2m_file_name = FILE_ZXM2M_BZ_PARA_ADDR;
  const char *hjep_file_name = FILE_HJEP_BZ_PARA_ADDR;
  const char *gbep_file_name = FILE_GBEP_BZ_PARA_ADDR;

  __blind_zone_para_t* p_blind_zone_para = (__blind_zone_para_t*)pThis;

  // ��Flash�ж�������
  FileIO_Read(file_name, p_blind_zone_para->chMask, BLIND_ZONE_PARA_SIZE);

  // У��CRC���ļ�ͷ
  if (BlindZone_CalcXorCheck(p_blind_zone_para->chMask, (BLIND_ZONE_PARA_SIZE - 1)) != pThis->crc || (pThis->header != BLIND_ZONE_PARA_HEADER))
  {
    // If the CRCs did not match, load defaults and store into bothmemory locations
    pThis->header = BLIND_ZONE_PARA_HEADER;; // У���ֽ�

    pThis->wr_error_cnt = 0;
    pThis->rd_error_cnt = 0;
    pThis->top = 0;
    pThis->bottom = 0;
    memset(pThis->data, 0x00, BLIND_ZONE_STACK_MAX_SIZE);

    BlindZone_SaveParameter(file_name, pThis); // Calculates the CRC and saves the data
    if (file_name==zxm2m_file_name)
    {
      PcDebug_Printf("ZxM2MBz:RdParaErr!\r\n");
    }
    else if (file_name==hjep_file_name)
    {
      PcDebug_Printf("HjepBz:RdParaErr!\r\n");
    }
    else if (file_name==gbep_file_name)
    {
      PcDebug_Printf("GbepBz:RdParaErr!\r\n");
    }
  }
}

/******************************************************************************
* ������ջ
******************************************************************************/
void BlindZone_PushData(blind_zone_t* pThis, uint8_t* p_data, uint16_t size)
{
  uint16_t pos;
  uint8_t ret_val;
  uint32_t offset_addr;

  if (p_data==NULL || size==0)
  {
    return;
  }

  pthread_mutex_lock(&pThis->file_mutex);

  pos	= (pThis->top + 1) % BLIND_ZONE_STACK_MAX_SIZE; // ջ����λ��
  offset_addr = pos * pThis->frame_size; // ֡��ʼ��ַ
  ret_val = FileIO_RandomWrite(pThis->file_name, offset_addr, p_data, pThis->frame_size); // д��ä��������
  if (ret_val==1) // д��ʧ��
  {
    pThis->wr_error_cnt++;
    return;
  }

  pThis->wr_error_cnt = 0;
  if (pos == pThis->bottom) // ��״̬,ɾ�������ݼ�¼
  {
    pThis->bottom	= (pThis->bottom + 1) % BLIND_ZONE_STACK_MAX_SIZE; // ����ջ��λ��
  }

  pThis->top = pos; // ����ջ��ֵ
  pThis->data[pThis->top] = size; // ���볤��

  pthread_mutex_unlock(&pThis->file_mutex);
}

/******************************************************************************
* ���ݳ�ջ
******************************************************************************/
void BlindZone_PopData(blind_zone_t* pThis, uint8_t* p_data, uint16_t* psize)
{
  uint32_t offset_addr;
  uint8_t ret_val;

  pthread_mutex_lock(&pThis->file_mutex);

  if (pThis->top == pThis->bottom) // ��״̬
  {
    *psize = 0x0000; // ���ݳ�����0
  }
  else // �ǿ�״̬,��ȡ����
  {
    *psize = pThis->data[pThis->top]; // ��ȡ����֡����
    if (*psize == 0)
    {
      pThis->data[pThis->top] = 0x00; // ��������
      pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // ����ջ��λ��
      return;
    }
    else
    {
      offset_addr = pThis->top * pThis->frame_size; // ֡��ʼ��ַ
      ret_val = FileIO_RandomRead(pThis->file_name, offset_addr,p_data, *psize); // ��ȡä��������
      if (ret_val==1) // ��ȡʧ��
      {
        pThis->rd_error_cnt++;
        *psize = 0x0000; // ���ݳ�����0
        pThis->data[pThis->top] = 0x00; // ��������
        pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // ����ջ��λ��
        return;
      }
      pThis->rd_error_cnt = 0x00;
      pThis->data[pThis->top] = 0x00; // ��������
      pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // ����ջ��λ��
    }
  }

  pthread_mutex_unlock(&pThis->file_mutex);
}

/******************************************************************************
* ��ȡջ��ʹ������
******************************************************************************/
uint16_t BlindZone_GetStackSize(blind_zone_t* pThis)
{
  return (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - pThis->bottom) % BLIND_ZONE_STACK_MAX_SIZE);
}
#endif

#if 0//(PART("ZxM2mä������"))

blind_zone_t zxm2m_blind_zone;
blind_zone_para_t zxm2m_blind_zone_para;
static uint8_t zxm2m_blind_zone_buffer[ZXM2M_BLIND_ZONE_PACKET_SIZE];
static uint16_t zxm2m_blind_zone_length;
#define ZXM2M_BZ_DEBUG    0  // 1-ʹ��, 0-��ֹ
#define ZXM2M_BZ_SAVE_PERIOD_SP  59 // 5����
//#define ZXM2M_BZ_SAVE_PERIOD_SP    29 // 30��  ������
#define ZXM2M_BLIND_ZONE_PACKET_SIZE  512
#define ZXM2M_BDZE_WRITE_ERROR_SP  6
#define ZXM2M_BDZE_READ_ERROR_SP   6

/******************************************************************************
* ��������M2Mä������
******************************************************************************/
void ZxM2mBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_ZXM2M_BZ_PARA_ADDR, &zxm2m_blind_zone_para); // ��ȡFLASH�еĲ���
  zxm2m_blind_zone.wr_error_cnt = zxm2m_blind_zone_para.wr_error_cnt;
  zxm2m_blind_zone.rd_error_cnt = zxm2m_blind_zone_para.rd_error_cnt;
  zxm2m_blind_zone.top = zxm2m_blind_zone_para.top;
  zxm2m_blind_zone.bottom = zxm2m_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    zxm2m_blind_zone.data[i] = zxm2m_blind_zone_para.data[i];
  }

  zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP;
  zxm2m_blind_zone.file_name = FILE_ZXM2M_BZ_DATA_ADDR; // �ļ���
  zxm2m_blind_zone.frame_size = ZXM2M_BLIND_ZONE_PACKET_SIZE; // ����֡�̶�����
  pthread_mutex_init(&zxm2m_blind_zone.file_mutex, NULL);

#if ZXM2M_BZ_DEBUG
  PcDebug_Printf("ZxM2mBzInit:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
}

//=====================================================================================
void ZxM2mBlindZone_Save(void)
{
  uint16_t i;

  pthread_mutex_lock(&zxm2m_blind_zone.file_mutex);

  zxm2m_blind_zone_para.wr_error_cnt = zxm2m_blind_zone.wr_error_cnt;
  zxm2m_blind_zone_para.rd_error_cnt = zxm2m_blind_zone.rd_error_cnt;
  zxm2m_blind_zone_para.top = zxm2m_blind_zone.top;
  zxm2m_blind_zone_para.bottom = zxm2m_blind_zone.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    zxm2m_blind_zone_para.data[i] = zxm2m_blind_zone.data[i];
  }

  BlindZone_SaveParameter(FILE_ZXM2M_BZ_PARA_ADDR, &zxm2m_blind_zone_para);

  pthread_mutex_unlock(&zxm2m_blind_zone.file_mutex);
}

//==GPS���ɼ�����(NB)===================================================================
uint16_t ZxBlindZone_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;
  uint8_t* downCanLenPos;

  pdata[len++] = 1;  // GPS��Ϣ������1B��,�̶�Ϊ1
  memcpy(&pdata[len], (uint8_t*)&g_stuZXGPSPosition, 25);  // GPSλ����Ϣ��ʽ(25B)
  len += 25;

  memcpy(&pdata[len], (uint8_t*)&stu_ZXVehicleState, 5);  // ����״̬��Ϣ1-5(5B)
  len += 5;

  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>24) & 0xFF;  // ���(4B)
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>16) & 0xFF;
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>8) & 0xFF;
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage & 0xFF);

  // �³���Ϣ�ֽ���N(1B) +�³���Ϣ(NB)
  downCanLenPos = &pdata[len];  // �����³���Ϣ�ֽ��������ݰ��еĵ�ַ
  len++;  // ָ���³���Ϣ
  *downCanLenPos = g_stuZXMcuData.ucMcuDownDataLen; //�³�ECU�����ֽ���N
  if (g_stuZXMcuData.ucMcuDownDataLen)
  {
    memcpy(&pdata[len],g_stuZXMcuData.aucMcuDownData,g_stuZXMcuData.ucMcuDownDataLen);  // �³���Ϣ
    len += g_stuZXMcuData.ucMcuDownDataLen;
  }

  if (g_stuZXMcuData.ucKxctAgFlag == CAN_DOWN_KXCT) // KXCT�³����ݵĹ��ϲ�����
  {
    g_stuZXMcuData.ucFaultDataLen = KXCT_FD_ReadAllFaultPacket(g_stuZXMcuData.aucFaultData,sfault_packet,mfault_packet);  // �³����ϲ���
    memcpy(&pdata[len],g_stuZXMcuData.aucFaultData,g_stuZXMcuData.ucFaultDataLen);  // ���ϲ���
    len += g_stuZXMcuData.ucFaultDataLen;
    *downCanLenPos += g_stuZXMcuData.ucFaultDataLen;
  }

  // �ϳ���ҵ�����ֽ���M(1B)+�ϳ���ҵ����(MB)
  uCAN_AttachFirmwareVersion(); // �������̼��汾��
  pdata[len++] = g_stuZXMcuData.ucMcuUpDataLen;  //�ϳ���ҵ�����ֽ���M
  if (g_stuZXMcuData.ucMcuUpDataLen)
  {
    if (stu_SYSZXParamSet.stu_Sys_ZXReport.ucReturnMode==0xc0 && g_stuZXMcuData.aucMcuUpDataPre[0]) // �ش�ģʽ
    {
      uCAN_ReadTypicalWorkData();
      MCU_ClearPercentageMomentData();
    }
    memcpy(&pdata[len], g_stuZXMcuData.aucMcuUpData,g_stuZXMcuData.ucMcuUpDataLen); // �ϳ���ҵ����
    len += g_stuZXMcuData.ucMcuUpDataLen;
  }

  return len;
}

//==1s����һ��============================================================================
void ZxM2mBlindZone_Service(void)
{
  // ��λ��Ч���й�������
  if (COLT_GetAccStatus()==1) // ACC��
  {
    if (zxm2m_blind_zone.timer_1s)
      zxm2m_blind_zone.timer_1s--;
    else
    {
      zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP; // �����һ��
      if (NetSocket_GetLinkState(&zxm2m_socket) != SOCKET_LINK_STATE_READY) // δ����
      {
        if (GPS_GetPositioningStatus()==1) // �ն��Ѷ�λ
        {
          memset(zxm2m_blind_zone_buffer,0xFF,ZXM2M_BLIND_ZONE_PACKET_SIZE); // ��ջ���
          zxm2m_blind_zone_length = ZxBlindZone_BuildEngData(zxm2m_blind_zone_buffer); // ����ä�����ݰ�: GPS��ECU����
          if (zxm2m_blind_zone_length <= ZXM2M_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&zxm2m_blind_zone, zxm2m_blind_zone_buffer, zxm2m_blind_zone_length); // ��������֡
            ZxM2mBlindZone_Save(); // ����ջ����

#if ZXM2M_BZ_DEBUG
            PcDebug_Printf("ZxM2mBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC�ر�
  {
    zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP; // �����һ��
  }

  // ���ֶ�д����,��λջΪ0
  if ((zxm2m_blind_zone.rd_error_cnt > ZXM2M_BDZE_READ_ERROR_SP) || (zxm2m_blind_zone.wr_error_cnt > ZXM2M_BDZE_WRITE_ERROR_SP))
  {
#if ZXM2M_BZ_DEBUG
    PcDebug_Printf("ZxM2mBzErr:Ewr=%d,Erd=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt);
#endif

    zxm2m_blind_zone.wr_error_cnt = 0;
    zxm2m_blind_zone.rd_error_cnt = 0;
    zxm2m_blind_zone.top = 0;
    zxm2m_blind_zone.bottom = 0;
    memset(zxm2m_blind_zone.data, 0x00, BLIND_ZONE_STACK_MAX_SIZE);
    ZxM2mBlindZone_Save(); // ����ջ����
  }
}

//==�Ϸ�ä����������==============================================================================
void ZxM2mBlindZone_SendData(void)
{
  uint16_t len = 0;
  uint8_t* pdata = aMsgSendData;
  uint16_t stack_size = 0x00;
  uint16_t bind_zone_data_size = 0x00;
  uint16_t gprs_data_size = 0x00;
  uint16_t sms_data_size = 0x00;

  stack_size = BlindZone_GetStackSize(&zxm2m_blind_zone);
  if (stack_size > 0)
  {
    // ����GPRS��ʽ: ��ʼλ(1B) + ���ݸ�ʽ(1B) + ����(2B) + ��������(1B) + ��������(nB) + ����λ(2B)
    pdata[0] = 0x68; // ��ʼλ
    pdata[1] = 'B'; // ���ݸ�ʽ
    pdata[4] = 0;  // ��������,0x00��ʾ����ΪGPS������Ϣ�ն���Ϣ

    /*************************���ж��Ÿ�ʽ(��������)��ʼ*************************/
    len = 5;
    pdata[len++] = 0x28; // ������
    pdata[len++] = 0x50; // �豸����
    pdata[len++] = 0x18; // �ش���Ϣ��ʶ

    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID & 0xFF);  // GPS������Ϣ�ն����к�
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>8) & 0xFF;
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>16) & 0xFF;
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>24) & 0xFF;

    // GPS������Ϣ�ն˵Ĳ������룬��¼���豸ʱ���룬�������Ĭ��Ϊ��Qh��
    memcpy(&pdata[len], (uint8_t*)&stu_SYSZXParamSet.aucLoginPassword[0], stu_SYSZXParamSet.ucLoginPasswordLen); // ���루2B��
    len += stu_SYSZXParamSet.ucLoginPasswordLen;

    pdata[len++] = 0x12;  // �ش������־:�Ϸ�ä����������

    g_stuZXSystem.usSerialnumber++;
    pdata[len++] = (uint8_t)(g_stuZXSystem.usSerialnumber & 0xFF); // ������ˮ��
    pdata[len++] = (uint8_t)(g_stuZXSystem.usSerialnumber >> 8);

    // GPS���ɼ�����(NB)
    BlindZone_PopData(&zxm2m_blind_zone, &pdata[len], &bind_zone_data_size);
    ZxM2mBlindZone_Save(); // ����ջ����
    if (bind_zone_data_size == 0x00)
    {
#if ZXM2M_BZ_DEBUG
      PcDebug_Printf("ZxM2mBzPop:Err\r\n");
#endif
      return;
    }

    len += bind_zone_data_size;

    sms_data_size = len - 5;
    pdata[len++] = XOR_Check(&pdata[5],sms_data_size); // У����(1B)
    sms_data_size += 1;
    /*************************���ж��Ÿ�ʽ(��������)����*************************/

    gprs_data_size = (1 + sms_data_size + 2); // ����: ��������(1B)+��������(nB)+����λ(2B)
    pdata[2] = (uint8_t)(gprs_data_size & 0xFF);
    pdata[3] = (uint8_t)(gprs_data_size>>8) & 0xFF;

    pdata[len++] = 0x0D; // ����λ
    pdata[len++] = 0x0A;
    NetSocket_Send(&zxm2m_socket, pdata, len);

#if ZXM2M_BZ_DEBUG
    PcDebug_Printf("ZxM2mBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif

  }
}
#endif

