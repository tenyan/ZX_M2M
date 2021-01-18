/*****************************************************************************
* @FileName: parameters.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-10-31
* @brief     �����洢����
******************************************************************************/
#include "config.h"
#include "sfud.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define PARM_DELAY(ms)    do { osDelay(ms); } while(0)

extern uint16_t im2m_AnalyzeParaData(uint16_t tag, uint16_t len, uint8_t *pValue);
extern uint16_t im2m_BuildParaData(uint8_t *pbuf, uint16_t tag);


/*************************************************************************
 * �������У��ֵ
*************************************************************************/
uint8_t iParm_CalcXorValue(uint8_t *p, uint16_t len)
{
  uint8_t sum = 0;
  uint16_t i;

  for (i = 0; i < len; i++)
  {
    sum ^= p[i];
  }
  
  return sum;
}

/******************************************************************************
* ������Ϣ
******************************************************************************/
void Parm_SaveLvcInfo(void)
{
  sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO1_ADDRESS,EMAP_LVC_INFO1_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO2_ADDRESS,EMAP_LVC_INFO2_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms

  //Calculate the Xor value so it is stored into memory
  lvc_context.xor_value = iParm_CalcXorValue((uint8_t *) &lvc_context, (SIZEOF_LVC_CONTEXT - 1));

  sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
  PARM_DELAY(2); // ��ʱ2ms
  sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
}

//============================================================================
void Parm_ReadLvcInfo(void)
{
  lvc_context_t lvc_ctx1;
  lvc_context_t lvc_ctx2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  sfud_read(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_ctx1); // ��ȡ������
  if (iParm_CalcXorValue((uint8_t *) &lvc_ctx1, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx1.xor_value && (lvc_ctx1.header == PARM_DATA_HEADER))
  {
    main_is_ok = PARM_TRUE;
  }

  sfud_read(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_ctx2); // ��ȡ������
  if (iParm_CalcXorValue((uint8_t *) &lvc_ctx2, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx2.xor_value && (lvc_ctx2.header == PARM_DATA_HEADER))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    memcpy(&lvc_context, &lvc_ctx1, SIZEOF_LVC_CONTEXT);
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc2Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO2_ADDRESS,EMAP_LVC_INFO2_SIZE); // ����
      PARM_DELAY(1); // ��ʱ1ms
      sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
    }

    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    memcpy(&lvc_context, &lvc_ctx2, SIZEOF_LVC_CONTEXT);
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc1Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO1_ADDRESS,EMAP_LVC_INFO1_SIZE); // ����
      PARM_DELAY(1); // ��ʱ1ms
      sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
    }
    return;
  }

  PcDebug_SendString("ParmLvc1_2Err!\n"); // �洢������
}

/******************************************************************************
* �ۼƹ���ʱ��
******************************************************************************/
void Parm_SaveTotalWorkTimeInfo(void)
{
  uint8_t buf[7];

  sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO1_ADDRESS,EMAP_WORK_TIME_INFO1_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO2_ADDRESS,EMAP_WORK_TIME_INFO2_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms

  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_work_time>>24) & 0xFF);  // ����
  buf[3] = (uint8_t)((colt_info.total_work_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_work_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_work_time & 0xFF);
  buf[6] = iParm_CalcXorValue(buf, 6);  // У��

  sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, buf);
  PARM_DELAY(2); // ��ʱ2ms
  sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, buf);
}

//============================================================================
void Parm_ReadTotalWorkTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  sfud_read(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, m_buf); // ��ȡ������
  if (iParm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  sfud_read(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, r_buf); // ��ȡ������
  if (iParm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    colt_info.total_work_time = m_buf[2];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += m_buf[3];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += m_buf[4];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += m_buf[5];
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmWkt2Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO2_ADDRESS,EMAP_WORK_TIME_INFO2_SIZE); // ����
      PARM_DELAY(1); // ��ʱ1ms
      sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, m_buf);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    colt_info.total_work_time = r_buf[2];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += r_buf[3];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += r_buf[4];
    colt_info.total_work_time <<= 8;
    colt_info.total_work_time += r_buf[5];
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmWkt1Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO1_ADDRESS,EMAP_WORK_TIME_INFO1_SIZE); // ����
      PARM_DELAY(1); // ��ʱ1ms
      sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, r_buf);
    }
    return;
  }

  colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // �洢������
}

//==���湤��Сʱ��RTCRAM=====================================================
void BKP_SaveTotalWorkTimeInfo(void)
{
  RTC_WriteBackupRegister(RTC_BKP_DR0, 0x55AA5AA5);
  RTC_WriteBackupRegister(RTC_BKP_DR1, colt_info.total_work_time);
}

/******************************************************************************
* M2MЭ�����ò���
******************************************************************************/
void Parm_SaveM2mAssetData(void)
{
  uint8_t *pbuf = public_data_buffer;
  uint16_t len = 0;
  uint8_t it;
  uint16_t tempVal;
  uint8_t xor_value;
  uint16_t tag[] = { 0x0000,0x0002,0x0003,0x0004,0x0005,0x0006,0x0007,0x0008,0x0009,0x000A,
                     0x000b,0x000c,0x0106,0x0107,0x0108,0x010b,0x010c,0x010d,0x0110,0x0113,
                     0x0114,0x0115,0x0116,0x0117,0x0118,0x0119,0x011a,0xA1FE };
  uint8_t totalTlvNum = (sizeof(tag)/sizeof(tag[0]));
  uint8_t tlvNum = 0;
                   
  PcDebug_SendString("SaveM2mParm!\n");
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,EMAP_M2M_ASSET_DATA1_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA2_ADDRESS,EMAP_M2M_ASSET_DATA2_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms

  pbuf[len++] = 0xA5; // ����ͷ
  pbuf[len++] = 0x5A;
  //==����================================
  len += 2;  // ���ݳ���
  len += 1;  // TLV����
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // ��������TLV
    if(tempVal!=0x00) // TLV��Ч
    {
      len += tempVal;
      tlvNum++;
    }
  }
  
  pbuf[2] = (len>>8) & 0xFF; // ����
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV����
  //======================================
  xor_value = iParm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // У��ֵ

  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, len, pbuf); // д��
  PARM_DELAY(2); // ��ʱ2ms
  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, len, pbuf); // д��
}

//============================================================================
void Parm_ReadM2mAssetData(void)
{
  uint8_t *pbuf = public_data_buffer;
  uint16_t pos = 0;
  uint8_t parm_is_ok = PARM_FALSE;
  uint16_t len;
  uint16_t header;
  uint8_t xor_value;
  uint8_t tlvNum;
  uint8_t retVal;
  uint16_t tag;
  uint16_t length;

  sfud_read(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, 1024, pbuf); // ��ȡ������
  header = pbuf[pos++]; // У��ͷ
  header <<= 8;;
  header += pbuf[pos++];
  len = pbuf[pos++]; // ����(����У��ͷ2B+����2B)
  len <<= 8;
  len += pbuf[pos++];
  tlvNum = pbuf[pos++]; // tlv����
  if ((header==0xA55A) && (len <= 1024)) // У��ͷ�Ͳ�������
  {
    xor_value = iParm_CalcXorValue(pbuf, len);
    if (xor_value==pbuf[len]) // У��ֵ
    {
      while (tlvNum) // TLV����
      {
        if (pos > len) // �����ж�
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
        retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // ����
        if (M2M_FALSE == retVal) // ����ʧ��
        {
          PcDebug_SendString("ParmM2mTlv1Err!\n");
          //break;
        }
        pos += length; // ָ����һ��TLV
        tlvNum--;
      }
      parm_is_ok = PARM_TRUE;
    }
  }

  if (parm_is_ok == PARM_FALSE)
  {
    PcDebug_SendString("ParmM2mAD1Err!\n");
    sfud_read(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, 1024, pbuf); // ��ȡ������
    header = pbuf[pos++]; // У��ͷ
    header <<= 8;;
    header += pbuf[pos++];
    len = pbuf[pos++]; // ����(����У��ͷ2B+����2B)
    len <<= 8;
    len += pbuf[pos++];
    tlvNum = pbuf[pos++]; // tlv����
    if ((header==0xA55A) && (len <= 1024)) // У��ͷ�Ͳ�������
    {
      xor_value = iParm_CalcXorValue(pbuf, len);
      if (xor_value==pbuf[len]) // У��ֵ
      {
        while (tlvNum) // TLV����
        {
          if (pos > len) // �����ж�
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
          retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // ����
          if (M2M_FALSE == retVal) // ����ʧ��
          {
            PcDebug_SendString("ParmM2mTlv2Err!\n");
          }
          pos += length; // ָ����һ��TLV
          tlvNum--;
        }
        parm_is_ok = PARM_TRUE;
        sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,1024); // ����
        PARM_DELAY(2); // ��ʱ2ms
        sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, 1024, pbuf); // д��
      }
    }

    if (parm_is_ok == PARM_FALSE)
    {
      PcDebug_SendString("ParmM2mAD2Err!\n");
    }
  }// end if (parm_is_ok == PARM_FALSE)

  memcpy(m2m_context.srv_ip, m2m_asset_data.main_srv_ip, 4);
  m2m_context.srv_port = m2m_asset_data.main_srv_port;
  m2m_context.srv_protocol = m2m_asset_data.main_srv_protocol;
  //enter_sleep_timer_sp = m2m_asset_data.wakeup_work_time_sp;
}

//==�ָ���������==============================================================
void Parm_ResetM2mAssetDataToFactory(void)
{
  uint8_t *pbuf = public_data_buffer;
  uint16_t len = 0;
  uint8_t it;
  uint16_t tempVal;
  uint8_t xor_value;
  
  // �ն�ID��IP����������
  uint16_t tag[] = { 0x0000,0x0002,0x0006,0x0007,0x0008,0x0009,0x0113,0x0114,0x0119,0x011A };
  uint8_t totalTlvNum = (sizeof(tag)/sizeof(tag[0]));
  uint8_t tlvNum = 0;
                   
  PcDebug_SendString("SaveM2mParm!\n");
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,EMAP_M2M_ASSET_DATA1_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA2_ADDRESS,EMAP_M2M_ASSET_DATA2_SIZE); // ����
  PARM_DELAY(2); // ��ʱ2ms

  pbuf[len++] = 0xA5; // ����ͷ
  pbuf[len++] = 0x5A;
  //==����================================
  len += 2;  // ���ݳ���
  len += 1;  // TLV����
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // ��������TLV
    if(tempVal!=0x00) // TLV��Ч
    {
      len += tempVal;
      tlvNum++;
    }
  }
  
  pbuf[2] = (len>>8) & 0xFF; // ����
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV����
  //======================================
  xor_value = iParm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // У��ֵ

  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, len, pbuf); // д��
  PARM_DELAY(2); // ��ʱ2ms
  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, len, pbuf); // д��
}

/******************************************************************************
* Զ�̹̼�����
******************************************************************************/
void rfu_EraseFlashHexFile(void)
{
  sfud_erase(&sfud_mx25l3206e,EMAP_HEX_FILE_ADDRESS,EMAP_HEX_FILE_SIZE); // ����
}

/* ================================================================== */
void rfu_SaveFlashHexFile(rfu_context_t* pThis, uint8_t *buf, uint16_t length)
{
  if ((pThis->cumulated_address+length) < EMAP_HEX_FILE_SIZE)
  {
    sfud_write(&sfud_mx25l3206e, (EMAP_HEX_FILE_ADDRESS + pThis->cumulated_address), length, buf);
    pThis->cumulated_address += length;
  }
}

/* ================================================================== */
void rfu_ReadFlashHexFile(uint32_t address, uint8_t *buf, uint32_t cnt)
{
  sfud_read(&sfud_mx25l3206e, (EMAP_HEX_FILE_ADDRESS + address), cnt, buf);
}

/* ================================================================== */
uint8_t rfu_CheckNewFirmware(rfu_context_t* pThis,uint8_t* buffer, uint16_t bufferSize)
{
  uint32_t size = bufferSize;

  pThis->block = 0;
  pThis->cumulated_address = 0;
  pThis->ending_address = pThis->file_length;
  pThis->total_block_count = (pThis->file_length / size) +1;
  pThis->crc32val = GetCrc32_Stream_Init();

  while (pThis->cumulated_address < pThis->ending_address)
  {
    if (pThis->cumulated_address + size > pThis->ending_address)
    {
      size = pThis->ending_address - pThis->cumulated_address;
    }

    rfu_ReadFlashHexFile(pThis->cumulated_address,buffer,size);
    pThis->crc32val = GetCrc32_Stream_Update(pThis->crc32val,buffer,size);

    pThis->cumulated_address += size;
    pThis->block++;
    pThis->percent = pThis->block * 100L / pThis->total_block_count;
  }

  // ����õ�SPI FLASH��HEX�ļ���CRCֵ
  pThis->crc32val = GetCrc32_Stream_Final(pThis->crc32val);
  if (pThis->crc32val != pThis->plain_crc32val)
  {
    return RFU_NOK;
  }

  return RFU_OK;
}


