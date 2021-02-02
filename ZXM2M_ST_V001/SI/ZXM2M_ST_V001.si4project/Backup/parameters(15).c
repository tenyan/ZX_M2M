/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: parameters.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-10-31
* @brief     �����洢����
******************************************************************************/
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define PARM_DELAY(ms)    do { osDelay(ms); } while(0)

extern uint16_t im2m_AnalyzeParaData(uint16_t tag, uint16_t len, uint8_t *pValue);
extern uint16_t im2m_BuildParaData(uint8_t *pbuf, uint16_t tag);

/******************************************************************************
 * Data Types and Globals
 ******************************************************************************/
//==default KV nodes======
static struct fdb_default_kv_node default_kv_table[] = {
  {"ZxSts",      zxstatistics_buffer,      (SIZE_OF_ZXSTATISTICS_BUFFER - SIZE_OF_ZXSTATISTICS_301E)}, // ͳ��������(�����ն�����ͳ��)
  {"TboxSts",    zxstatistics_buffer_301e, SIZE_OF_ZXSTATISTICS_301E}, // �ն�����ͳ��
  {"ZxEngineUpSts", zxup_buffer_a5c8,      SIZE_OF_ZXUP_A5C8}, // ��ҵ�ͺ�ͳ��
  {"ZxEngineDwSts", zxengine_buffer_a5f0,  SIZE_OF_ZXENGINE_A5F0},    // ��ʻ�ͺ�ͳ��
};

static struct fdb_kvdb kvdb = { 0 };  // KVDB object
osSemaphoreId_t sid_FlashDbSem = NULL;  // KV���ݿ��ź���

/******************************************************************************
* FlashDB��ʼ��
******************************************************************************/
//==���ݿ�����====================================
static void fdb_lock(fdb_db_t* db)
{
  osSemaphoreAcquire(sid_FlashDbSem, osWaitForever); // �ȴ��ź���
}

//==���ݿ�ȥ��====================================
static void fdb_unlock(fdb_db_t* db)
{
  osSemaphoreRelease(sid_FlashDbSem); // �ͷ��ź���
}

//==��ʼ��Flash���ݿ�=============================
int32_t Parm_FlashDbInit(void)
{
  fdb_err_t result;
  struct fdb_default_kv default_kv;

  if(sid_FlashDbSem==NULL)
  {
    sid_FlashDbSem = osSemaphoreNew(1, 1, NULL);
  }

  default_kv.kvs = default_kv_table;
  default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);
  FlashDB_KvdbControl(&kvdb, FDB_KVDB_CTRL_SET_LOCK, fdb_lock);
  FlashDB_KvdbControl(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, fdb_unlock);
  
  /* Key-Value database initialization
   *       &kvdb: database object
   *       "env": database name
   * "fdb_kvdb1": The flash partition name base on FAL. Please make sure it's in FAL partition table.
   *              Please change to YOUR partition name.
   * &default_kv: The default KV nodes. It will auto add to KVDB when first initialize successfully.
   *        NULL: The user data if you need, now is empty.
   */
  result = FlashDB_KvdbInit(&kvdb, "env", "fdb_kvdb1", &default_kv, NULL);

  if (result != FDB_NO_ERR)
  {
    return -1;
  }
  
  return 0;
}

#if 0
static uint32_t boot_count = 0;
static uint32_t boot_time[10] = {0, 1, 2, 3};
//==default KV nodes======
static struct fdb_default_kv_node default_kv_table[] = {
  {"username", "armink", 0}, // string KV
  {"password", "123456", 0}, // string KV
  {"boot_count", &boot_count, sizeof(boot_count)}, // int type KV
  {"boot_time", &boot_time, sizeof(boot_time)},    // int array type KV
};

#define FDB_LOG_TAG "[sample][kvdb][blob]"
void kvdb_type_blob_sample(fdb_kvdb_t kvdb)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;


  FDB_INFO("==================== kvdb_type_blob_sample ====================\n");

  { // CREATE new Key-Value
    int temp_data = 36;

    /* It will create new KV node when "temp" KV not in database.
     * fdb_blob_make: It's a blob make function, and it will return the blob when make finish.
     */
    p_blob = FlashDB_BlobMake(&blob, &temp_data, sizeof(temp_data));
    FlashDB_KvSetBlob(kvdb, "temp", p_blob);
    FDB_INFO("create the 'temp' blob KV, value is: %d\n", temp_data);
  }

  { // GET the KV value
    int temp_data = 0;

    /* get the "temp" KV value */
    p_blob = FlashDB_BlobMake(&blob, &temp_data, sizeof(temp_data));
    FalshDB_KvGetBlob(kvdb, "temp", p_blob);
    /* the blob.saved.len is more than 0 when get the value successful */
    if (blob.saved.len > 0)
    {
      FDB_INFO("get the 'temp' value is: %d\n", temp_data);
    }
  }

  { // CHANGE the KV value
    int temp_data = 38;

    /* change the "temp" KV's value to 38 */
    p_blob = FlashDB_BlobMake(&blob, &temp_data, sizeof(temp_data));
    FlashDB_KvSetBlob(kvdb, "temp", p_blob);
    FDB_INFO("set 'temp' value to %d\n", temp_data);
  }

  { // DELETE the KV by name
    FlashDB_KvDel(kvdb, "temp");
    FDB_INFO("delete the 'temp' finish\n");
  }

  FDB_INFO("===========================================================\n");
}
#endif /* FDB_USING_KVDB */

/******************************************************************************
* ��ҵͳ����
******************************************************************************/
void Parm_SaveZxStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;
  
  p_blob = FlashDB_BlobMake(&blob, zxstatistics_buffer, (SIZE_OF_ZXSTATISTICS_BUFFER - SIZE_OF_ZXSTATISTICS_301E));
  FlashDB_KvSetBlob(&kvdb, "ZxSts", p_blob);
}

//============================================================================
void Parm_ReadZxStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;

  p_blob = FlashDB_BlobMake(&blob, zxstatistics_buffer, (SIZE_OF_ZXSTATISTICS_BUFFER - SIZE_OF_ZXSTATISTICS_301E));
  FalshDB_KvGetBlob(&kvdb, "ZxSts", p_blob);
  if (blob.saved.len > 0) // the blob.saved.len is more than 0 when get the value successful
  {
    //FDB_INFO("get the 'temp' value is: %d\n", temp_data);
  }
}

/******************************************************************************
* �ն�����ͳ��
******************************************************************************/
void Parm_SaveTboxStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;
  
  p_blob = FlashDB_BlobMake(&blob, zxstatistics_buffer_301e, SIZE_OF_ZXSTATISTICS_301E);
  FlashDB_KvSetBlob(&kvdb, "TboxSts", p_blob);
}

//============================================================================
void Parm_ReadTboxStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;

  p_blob = FlashDB_BlobMake(&blob, zxstatistics_buffer_301e, SIZE_OF_ZXSTATISTICS_301E);
  FalshDB_KvGetBlob(&kvdb, "TboxSts", p_blob);
  if (blob.saved.len > 0) // the blob.saved.len is more than 0 when get the value successful
  {
    //FDB_INFO("get the 'temp' value is: %d\n", temp_data);
  }
}

/******************************************************************************
* ��ҵ�ͺ�ͳ��
******************************************************************************/
void Parm_SaveZxEngineUpStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;
  
  p_blob = FlashDB_BlobMake(&blob, zxup_buffer_a5c8, SIZE_OF_ZXUP_A5C8);
  FlashDB_KvSetBlob(&kvdb, "ZxEngineUpSts", p_blob);
}

//============================================================================
void Parm_ReadZxEngineUpStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;
  
  p_blob = FlashDB_BlobMake(&blob, zxup_buffer_a5c8, SIZE_OF_ZXUP_A5C8);
  FalshDB_KvGetBlob(&kvdb, "ZxEngineUpSts", p_blob);
  if (blob.saved.len > 0) // the blob.saved.len is more than 0 when get the value successful
  {
    //FDB_INFO("get the 'temp' value is: %d\n", temp_data);
  }
}

/******************************************************************************
* ��ʻ�ͺ�ͳ��
******************************************************************************/
void Parm_SaveZxEngineDwStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;
  
  p_blob = FlashDB_BlobMake(&blob, zxengine_buffer_a5f0, SIZE_OF_ZXENGINE_A5F0);
  FlashDB_KvSetBlob(&kvdb, "ZxEngineDwSts", p_blob);
}

//============================================================================
void Parm_ReadZxEngineDwStsInfo(void)
{
  fdb_blob_t blob;
  fdb_blob_t* p_blob;

  p_blob = FlashDB_BlobMake(&blob, zxengine_buffer_a5f0, SIZE_OF_ZXENGINE_A5F0);
  FalshDB_KvGetBlob(&kvdb, "ZxEngineDwSts", p_blob);
  if (blob.saved.len > 0) // the blob.saved.len is more than 0 when get the value successful
  {
    //FDB_INFO("get the 'temp' value is: %d\n", temp_data);
  }
}

/******************************************************************************
 * �������У��ֵ
******************************************************************************/
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

#if 0
//==���湤��Сʱ��RTCRAM=====================================================
void BKP_SaveTotalWorkTimeInfo(void)
{
  RTC_WriteBackupRegister(RTC_BKP_DR0, 0x55AA5AA5);
  RTC_WriteBackupRegister(RTC_BKP_DR1, colt_info.total_work_time);
}
#endif

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
void rfu_EraseFlashHexFile(uint8_t fileType)
{
  if(fileType==0x03) // Э������
  {
    sfud_erase(&sfud_mx25l3206e, EMAP_HEX_FILE_ADDRESS, EMAP_HEX_FILE_SIZE); // ����
  }
  else // ECU or LCD
  {
    sfud_erase(&sfud_mx25l3206e, EMAP_ECU_HEX_FILE_ADDRESS, EMAP_ECU_HEX_FILE_SIZE); // ����
  }
}

/* ================================================================== */
void rfu_SaveFlashHexFile(uint8_t fileType, uint32_t address, uint8_t *buf, uint16_t length)
{
  if(fileType==0x03) // Э������
  {
    if ((address + length) < EMAP_HEX_FILE_SIZE)
    {
      sfud_write(&sfud_mx25l3206e, (EMAP_HEX_FILE_ADDRESS + address), length, buf);
    }
  }
  else // ECU or LCD
  {
    if ((address + length) < EMAP_ECU_HEX_FILE_SIZE)
    {
      sfud_write(&sfud_mx25l3206e, (EMAP_ECU_HEX_FILE_ADDRESS + address), length, buf);
    }
  }
}

/* ================================================================== */
void rfu_ReadFlashHexFile(uint8_t fileType, uint32_t address, uint8_t *buf, uint16_t bufferSize)
{
  if(fileType==0x03) // Э������
  {
    sfud_read(&sfud_mx25l3206e, (EMAP_HEX_FILE_ADDRESS + address), bufferSize, buf);
  }
  else // ECU or LCD
  {
    sfud_read(&sfud_mx25l3206e, (EMAP_ECU_HEX_FILE_ADDRESS + address), bufferSize, buf);
  }
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
    rfu_ReadFlashHexFile(pThis->dev, pThis->cumulated_address, buffer, size);
    pThis->crc32val = GetCrc32_Stream_Update(pThis->crc32val, buffer, size);
    
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


