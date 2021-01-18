/*****************************************************************************
* @FileName: parameters.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-10-31
* @brief     参数存储管理
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
 * 计算异或校验值
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
* 锁车信息
******************************************************************************/
void Parm_SaveLvcInfo(void)
{
  sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO1_ADDRESS,EMAP_LVC_INFO1_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO2_ADDRESS,EMAP_LVC_INFO2_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms

  //Calculate the Xor value so it is stored into memory
  lvc_context.xor_value = iParm_CalcXorValue((uint8_t *) &lvc_context, (SIZEOF_LVC_CONTEXT - 1));

  sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
  PARM_DELAY(2); // 延时2ms
  sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
}

//============================================================================
void Parm_ReadLvcInfo(void)
{
  lvc_context_t lvc_ctx1;
  lvc_context_t lvc_ctx2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  sfud_read(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_ctx1); // 读取主备份
  if (iParm_CalcXorValue((uint8_t *) &lvc_ctx1, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx1.xor_value && (lvc_ctx1.header == PARM_DATA_HEADER))
  {
    main_is_ok = PARM_TRUE;
  }

  sfud_read(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_ctx2); // 读取副备份
  if (iParm_CalcXorValue((uint8_t *) &lvc_ctx2, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx2.xor_value && (lvc_ctx2.header == PARM_DATA_HEADER))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // 主备份OK
  {
    memcpy(&lvc_context, &lvc_ctx1, SIZEOF_LVC_CONTEXT);
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc2Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO2_ADDRESS,EMAP_LVC_INFO2_SIZE); // 擦除
      PARM_DELAY(1); // 延时1ms
      sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO2_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
    }

    return;
  }

  if (redundant_is_ok==PARM_TRUE) // 副备份OK
  {
    memcpy(&lvc_context, &lvc_ctx2, SIZEOF_LVC_CONTEXT);
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc1Err!\n");
      sfud_erase(&sfud_mx25l3206e,EMAP_LVC_INFO1_ADDRESS,EMAP_LVC_INFO1_SIZE); // 擦除
      PARM_DELAY(1); // 延时1ms
      sfud_write(&sfud_mx25l3206e, EMAP_LVC_INFO1_ADDRESS, SIZEOF_LVC_CONTEXT, (uint8_t *)&lvc_context);
    }
    return;
  }

  PcDebug_SendString("ParmLvc1_2Err!\n"); // 存储都出错
}

/******************************************************************************
* 累计工作时间
******************************************************************************/
void Parm_SaveTotalWorkTimeInfo(void)
{
  uint8_t buf[7];

  sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO1_ADDRESS,EMAP_WORK_TIME_INFO1_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO2_ADDRESS,EMAP_WORK_TIME_INFO2_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms

  buf[0] = 0xA5;  //  校验头
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_work_time>>24) & 0xFF);  // 参数
  buf[3] = (uint8_t)((colt_info.total_work_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_work_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_work_time & 0xFF);
  buf[6] = iParm_CalcXorValue(buf, 6);  // 校验

  sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, buf);
  PARM_DELAY(2); // 延时2ms
  sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, buf);
}

//============================================================================
void Parm_ReadTotalWorkTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  sfud_read(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, m_buf); // 读取主备份
  if (iParm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  sfud_read(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, r_buf); // 读取副备份
  if (iParm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // 主备份OK
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
      sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO2_ADDRESS,EMAP_WORK_TIME_INFO2_SIZE); // 擦除
      PARM_DELAY(1); // 延时1ms
      sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO2_ADDRESS, 7, m_buf);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // 副备份OK
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
      sfud_erase(&sfud_mx25l3206e,EMAP_WORK_TIME_INFO1_ADDRESS,EMAP_WORK_TIME_INFO1_SIZE); // 擦除
      PARM_DELAY(1); // 延时1ms
      sfud_write(&sfud_mx25l3206e, EMAP_WORK_TIME_INFO1_ADDRESS, 7, r_buf);
    }
    return;
  }

  colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // 存储都出错
}

//==保存工作小时到RTCRAM=====================================================
void BKP_SaveTotalWorkTimeInfo(void)
{
  RTC_WriteBackupRegister(RTC_BKP_DR0, 0x55AA5AA5);
  RTC_WriteBackupRegister(RTC_BKP_DR1, colt_info.total_work_time);
}

/******************************************************************************
* M2M协议设置参数
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
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,EMAP_M2M_ASSET_DATA1_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA2_ADDRESS,EMAP_M2M_ASSET_DATA2_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms

  pbuf[len++] = 0xA5; // 检验头
  pbuf[len++] = 0x5A;
  //==参数================================
  len += 2;  // 数据长度
  len += 1;  // TLV个数
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // 创建参数TLV
    if(tempVal!=0x00) // TLV有效
    {
      len += tempVal;
      tlvNum++;
    }
  }
  
  pbuf[2] = (len>>8) & 0xFF; // 长度
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV个数
  //======================================
  xor_value = iParm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // 校验值

  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, len, pbuf); // 写入
  PARM_DELAY(2); // 延时2ms
  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, len, pbuf); // 写入
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

  sfud_read(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, 1024, pbuf); // 读取主备份
  header = pbuf[pos++]; // 校验头
  header <<= 8;;
  header += pbuf[pos++];
  len = pbuf[pos++]; // 长度(包含校验头2B+长度2B)
  len <<= 8;
  len += pbuf[pos++];
  tlvNum = pbuf[pos++]; // tlv数量
  if ((header==0xA55A) && (len <= 1024)) // 校验头和参数长度
  {
    xor_value = iParm_CalcXorValue(pbuf, len);
    if (xor_value==pbuf[len]) // 校验值
    {
      while (tlvNum) // TLV解析
      {
        if (pos > len) // 长度判断
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
        retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // 解析
        if (M2M_FALSE == retVal) // 解析失败
        {
          PcDebug_SendString("ParmM2mTlv1Err!\n");
          //break;
        }
        pos += length; // 指向下一个TLV
        tlvNum--;
      }
      parm_is_ok = PARM_TRUE;
    }
  }

  if (parm_is_ok == PARM_FALSE)
  {
    PcDebug_SendString("ParmM2mAD1Err!\n");
    sfud_read(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, 1024, pbuf); // 读取副备份
    header = pbuf[pos++]; // 校验头
    header <<= 8;;
    header += pbuf[pos++];
    len = pbuf[pos++]; // 长度(包含校验头2B+长度2B)
    len <<= 8;
    len += pbuf[pos++];
    tlvNum = pbuf[pos++]; // tlv数量
    if ((header==0xA55A) && (len <= 1024)) // 校验头和参数长度
    {
      xor_value = iParm_CalcXorValue(pbuf, len);
      if (xor_value==pbuf[len]) // 校验值
      {
        while (tlvNum) // TLV解析
        {
          if (pos > len) // 长度判断
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
          retVal = im2m_AnalyzeParaData(tag, length, &pbuf[pos]); // 解析
          if (M2M_FALSE == retVal) // 解析失败
          {
            PcDebug_SendString("ParmM2mTlv2Err!\n");
          }
          pos += length; // 指向下一个TLV
          tlvNum--;
        }
        parm_is_ok = PARM_TRUE;
        sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,1024); // 擦除
        PARM_DELAY(2); // 延时2ms
        sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, 1024, pbuf); // 写入
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

//==恢复出厂设置==============================================================
void Parm_ResetM2mAssetDataToFactory(void)
{
  uint8_t *pbuf = public_data_buffer;
  uint16_t len = 0;
  uint8_t it;
  uint16_t tempVal;
  uint8_t xor_value;
  
  // 终端ID、IP、域名不变
  uint16_t tag[] = { 0x0000,0x0002,0x0006,0x0007,0x0008,0x0009,0x0113,0x0114,0x0119,0x011A };
  uint8_t totalTlvNum = (sizeof(tag)/sizeof(tag[0]));
  uint8_t tlvNum = 0;
                   
  PcDebug_SendString("SaveM2mParm!\n");
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA1_ADDRESS,EMAP_M2M_ASSET_DATA1_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms
  sfud_erase(&sfud_mx25l3206e,EMAP_M2M_ASSET_DATA2_ADDRESS,EMAP_M2M_ASSET_DATA2_SIZE); // 擦除
  PARM_DELAY(2); // 延时2ms

  pbuf[len++] = 0xA5; // 检验头
  pbuf[len++] = 0x5A;
  //==参数================================
  len += 2;  // 数据长度
  len += 1;  // TLV个数
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // 创建参数TLV
    if(tempVal!=0x00) // TLV有效
    {
      len += tempVal;
      tlvNum++;
    }
  }
  
  pbuf[2] = (len>>8) & 0xFF; // 长度
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV个数
  //======================================
  xor_value = iParm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // 校验值

  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA1_ADDRESS, len, pbuf); // 写入
  PARM_DELAY(2); // 延时2ms
  sfud_write(&sfud_mx25l3206e, EMAP_M2M_ASSET_DATA2_ADDRESS, len, pbuf); // 写入
}

/******************************************************************************
* 远程固件更新
******************************************************************************/
void rfu_EraseFlashHexFile(void)
{
  sfud_erase(&sfud_mx25l3206e,EMAP_HEX_FILE_ADDRESS,EMAP_HEX_FILE_SIZE); // 擦除
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

  // 计算得到SPI FLASH中HEX文件的CRC值
  pThis->crc32val = GetCrc32_Stream_Final(pThis->crc32val);
  if (pThis->crc32val != pThis->plain_crc32val)
  {
    return RFU_NOK;
  }

  return RFU_OK;
}


