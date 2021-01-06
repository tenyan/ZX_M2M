/*****************************************************************************
* @FileName: Parameters.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-1
* @brief     参数存储管理
* @技术点:     sync命令用于强制被改变的内容立刻写入磁盘，更新超块信息。
* 在Linux/Unix系统中,在文件或数据处理过程中一般先放到内存缓冲区中,
* 等到适当的时候再写入磁盘,以提高系统的运行效率。
* sync命令则可用来强制将内存缓冲区中的数据立即写入磁盘中。
* 用户通常不需执行sync命令,系统会自动执行update或bdflush操作,
* 将缓冲区的数据写入磁盘。
* 只有在update或bdflush无法执行或用户需要非正常关机时,才需手动执行sync命令。
******************************************************************************/
#include "config.h"
#include "sfud.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define PARM_DELAY(ms)    msleep(ms)

extern uint16_t im2m_AnalyzeParaData(uint16_t tag, uint16_t len, uint8_t *pValue);
extern uint16_t im2m_BuildParaData(uint8_t *pbuf, uint16_t tag);


/******************************************************************************
 * read data from file (Random Access File)
 *
 * @param file_name file path
 * @param offset_addr start address
 * @param pbuf read data pointer
 * @param size read size
 *
 * @return 0-操作成功 -1-操作失败
 ******************************************************************************/
int8_t FileIO_RandomRead(const char *file_name, uint32_t offset_addr, uint8_t *pbuf, uint32_t size)
{
  int fd_rd;
  int new_offset;

  fd_rd = open(file_name, O_RDONLY); // 以只读方式打开文件
  if (fd_rd < 0)
  {
    printf("ERROR R_open %s fd=%d\n\r", file_name, fd_rd);
    close(fd_rd);
    return -1;
  }

  // 设置新的读写位置为从文件开头算起，偏移offset_addr个字节
  new_offset = lseek(fd_rd, offset_addr, SEEK_SET); // 设置文件的起始读写位置
  if (new_offset == -1)
  {
    printf("ERROR R_lseek %s fd=%d\n\r", file_name, offset_addr);
    close(fd_rd);
    return -1;
  }

  read(fd_rd, pbuf, size); // 读取文件
  close(fd_rd); // 关闭文件

  return 0;
}

/******************************************************************************
 * write data to file (Random Access File)
 *
 * @param file_name file path
 * @param offset_addr start address
 * @param pdata write data
 * @param size write size
 *
 * @return 0-操作成功 -1-操作失败
 ******************************************************************************/
int8_t FileIO_RandomWrite(const char *file_name, uint32_t offset_addr, uint8_t *pdata, uint32_t size)
{
  int fd_wr;
  int new_offset;

  fd_wr = open(file_name, O_RDWR|O_CREAT); // 打开文件
  if (fd_wr < 0)
  {
    printf("ERROR W_open %s fd=%d\n\r", file_name, fd_wr);
    close(fd_wr);
    return -1;
  }

  // 设置新的读写位置为从文件开头算起，偏移addr字节
  new_offset = lseek(fd_wr, offset_addr, SEEK_SET); // 设置文件的起始读写位置
  if (new_offset == -1)
  {
    printf("ERROR W_lseek %s fd=%d\n\r", file_name, offset_addr);
    close(fd_wr);
    return -1;
  }

  write(fd_wr, pdata, size); // 写入文件到缓存
  // fsync(fd_wr); // 同步文件
  close(fd_wr); // 关闭文件

  return 0;
}

/******************************************************************************
 * write data to file
 *
 * @param file_name file path
 * @param pdata write data
 * @param size write size
 *
 * @return 0-操作成功 -1-操作失败
 ******************************************************************************/
int8_t FileIO_Write(const char *file_name, uint8_t *pdata, uint32_t size)
{
  int fd_wr;

  fd_wr = open(file_name, O_RDWR|O_CREAT);
  if (fd_wr < 0)
  {
    printf("ERROR Wopen %s fd=%d\n\r", file_name, fd_wr);
    close(fd_wr);
    return -1;
  }

  write(fd_wr, pdata, size);
  fsync(fd_wr);  // 同步文件(等待写磁盘操作结束)
  close(fd_wr);
  return 0;
}

/******************************************************************************
 * read data from file
 *
 * @param file_name file path
 * @param pbuf read data pointer
 * @param size read size
 *
 * @return 0-操作成功 -1-操作失败
 ******************************************************************************/
int8_t FileIO_Read(const char *file_name, uint8_t *pbuf, uint32_t size)
{
  int fd_rd;

  fd_rd = open(file_name, O_RDONLY); // 以只读方式打开文件
  if (fd_rd < 0)
  {
    printf("ERROR Ropen %s fd=%d\n\r", file_name, fd_rd);
    close(fd_rd);
    return -1;
  }

  read(fd_rd, pbuf,size);
  close(fd_rd);

  return 0;
}

/*************************************************************************
 * 计算异或校验值
*************************************************************************/
uint8_t Parm_CalcXorValue(uint8_t *p, uint16_t len)
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
  //Calculate the Xor value so it is stored into memory
  lvc_context.xor_value = Parm_CalcXorValue((uint8_t *) &lvc_context, (SIZEOF_LVC_CONTEXT - 1));

  FileIO_Write(FILE_USER_LVC_CPY1, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
  PARM_DELAY(5); // 延时5ms
  FileIO_Write(FILE_USER_LVC_CPY2, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
}

//============================================================================
void Parm_ReadLvcInfo(void)
{
  lvc_context_t lvc_ctx1;
  lvc_context_t lvc_ctx2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_USER_LVC_CPY1, &lvc_ctx1, SIZEOF_LVC_CONTEXT); // 读取主备份
  if (Parm_CalcXorValue((uint8_t *) &lvc_ctx1, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx1.xor_value && (lvc_ctx1.header == PARM_DATA_HEADER))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_USER_LVC_CPY2, &lvc_ctx2, SIZEOF_LVC_CONTEXT); // 读取副备份
  if (Parm_CalcXorValue((uint8_t *) &lvc_ctx2, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx2.xor_value && (lvc_ctx2.header == PARM_DATA_HEADER))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // 主备份OK
  {
    memcpy(&lvc_context, &lvc_ctx1, SIZEOF_LVC_CONTEXT);
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc2Err!\n");
      FileIO_Write(FILE_USER_LVC_CPY2, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
    }

    return;
  }

  if (redundant_is_ok==PARM_TRUE) // 副备份OK
  {
    memcpy(&lvc_context, &lvc_ctx2, SIZEOF_LVC_CONTEXT);
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc1Err!\n");
      FileIO_Write(FILE_USER_LVC_CPY1, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
    }
    return;
  }

  PcDebug_SendString("ParmLvc1_2Err!\n"); // 存储都出错
}

/******************************************************************************
* VIN码信息
******************************************************************************/
void Parm_SaveVinInfo(void)
{
  uint8_t buf[25];
  
  buf[0] = 0xA5;  //  校验头
  buf[1] = 0x5A;
  buf[2] = VinValidFlag; // 激活或关闭
  buf[3] = VinLen;       // VIN长度
  memcpy(&buf[4], Vin, VIN_SIZE); // 17个字节VIN码
  buf[24] = Parm_CalcXorValue(buf, 24); // 校验和

  FileIO_Write(FILE_USER_VIN_CPY1, buf, 25);
  PARM_DELAY(5); // 延时5ms
  FileIO_Write(FILE_USER_VIN_CPY2, buf, 25);
}

//============================================================================
void Parm_ReadVinInfo(void)
{
  uint8_t m_buf[25];
  uint8_t xor_value1;
  uint8_t r_buf[25];
  uint8_t xor_value2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_USER_VIN_CPY1, m_buf, 25); // 读取主备份
  xor_value1 = Parm_CalcXorValue(m_buf, 24);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[24]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_USER_VIN_CPY2, r_buf, 25); // 读取副备份
  xor_value2 = Parm_CalcXorValue(r_buf, 24);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[24]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // 主备份OK
  {
    VinValidFlag = m_buf[2]; // 激活或关闭
    VinLen = m_buf[3];       // VIN长度
    memcpy(Vin, &m_buf[4], VIN_SIZE); // 17个字节VIN码
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmVin2Err!\n");
      FileIO_Write(FILE_USER_VIN_CPY2, m_buf, 25);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // 副备份OK
  {
    VinValidFlag = r_buf[2]; // 激活或关闭
    VinLen = r_buf[3];       // VIN长度
    memcpy(Vin, &r_buf[4], VIN_SIZE); // 17个字节VIN码
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmVin1Err!\n");
      FileIO_Write(FILE_USER_VIN_CPY1, r_buf, 25);
    }
    return;
  }

  PcDebug_SendString("ParmVin1_2Err!\n"); // 存储都出错
}

/******************************************************************************
* 累计不上线时间
******************************************************************************/
void Parm_SaveOfflineTimeInfo(void)
{
  uint8_t buf[7];

  buf[0] = 0xA5;  //  校验头
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_offline_time>>24) & 0xFF);  // 参数
  buf[3] = (uint8_t)((colt_info.total_offline_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_offline_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_offline_time & 0xFF);
  buf[6] = Parm_CalcXorValue(buf, 6);  // 校验

  FileIO_Write(FILE_OFFLINE_TIME_CPY1, buf, 7);
  PARM_DELAY(5); // 延时5ms
  FileIO_Write(FILE_OFFLINE_TIME_CPY2, buf, 7);
}

//============================================================================
void Parm_ReadOfflineTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_OFFLINE_TIME_CPY1, m_buf, 7); // 读取主备份
  if (Parm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_OFFLINE_TIME_CPY2, r_buf, 7); // 读取副备份
  if (Parm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
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
      FileIO_Write(FILE_OFFLINE_TIME_CPY2, m_buf, 7);
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
      FileIO_Write(FILE_OFFLINE_TIME_CPY1, r_buf, 7);
    }
    return;
  }
  //colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // 存储都出错
}

/******************************************************************************
* 累计工作时间
******************************************************************************/
void Parm_SaveWorkTimeInfo(void)
{
  uint8_t buf[7];

  buf[0] = 0xA5;  //  校验头
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_work_time>>24) & 0xFF);  // 参数
  buf[3] = (uint8_t)((colt_info.total_work_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_work_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_work_time & 0xFF);
  buf[6] = Parm_CalcXorValue(buf, 6);  // 校验

  FileIO_Write(FILE_WORKTIME_CPY1, buf, 7);
  PARM_DELAY(5); // 延时5ms
  FileIO_Write(FILE_WORKTIME_CPY2, buf, 7);
}

//============================================================================
void Parm_ReadWorkTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_WORKTIME_CPY1, m_buf, 7); // 读取主备份
  if (Parm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_WORKTIME_CPY2, r_buf, 7); // 读取副备份
  if (Parm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
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
      FileIO_Write(FILE_WORKTIME_CPY2, m_buf, 7);
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
      FileIO_Write(FILE_WORKTIME_CPY1, r_buf, 7);
    }
    return;
  }

  // colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // 存储都出错
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
                     0x0114,0x0115,0x0116,0x0117,0x0118,0x0119,0x011a,0xA1FE
                   };
  uint8_t totalTlvNum = (sizeof(tag)/sizeof(tag[0]));
  uint8_t tlvNum = 0;

  PcDebug_SendString("SaveM2mParm!\n");

  pbuf[len++] = 0xA5; // 检验头
  pbuf[len++] = 0x5A;
  //==参数================================
  len += 2;  // 数据长度
  len += 1;  // TLV个数
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // 创建参数TLV
    if (tempVal!=0x00) // TLV有效
    {
      len += tempVal;
      tlvNum++;
    }
  }

  pbuf[2] = (len>>8) & 0xFF; // 长度
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV个数
  //======================================
  xor_value = Parm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // 校验值

  FileIO_Write(FILE_M2M_PARA_CPY1, pbuf, len);
  usleep(10); // 延时10us
  FileIO_Write(FILE_M2M_PARA_CPY2, pbuf, len);
  usleep(10); // 延时10us
  system("sync");
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

  FileIO_Read(FILE_M2M_PARA_CPY1, pbuf, 1024); // 读取主备份
  header = pbuf[pos++]; // 校验头
  header <<= 8;;
  header += pbuf[pos++];
  len = pbuf[pos++]; // 长度(包含校验头2B+长度2B)
  len <<= 8;
  len += pbuf[pos++];
  tlvNum = pbuf[pos++]; // tlv数量
  if ((header==0xA55A) && (len <= 1024)) // 校验头和参数长度
  {
    xor_value = Parm_CalcXorValue(pbuf, len);
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
    FileIO_Read(FILE_M2M_PARA_CPY2, pbuf, 1024); // 读取副备份
    header = pbuf[pos++]; // 校验头
    header <<= 8;;
    header += pbuf[pos++];
    len = pbuf[pos++]; // 长度(包含校验头2B+长度2B)
    len <<= 8;
    len += pbuf[pos++];
    tlvNum = pbuf[pos++]; // tlv数量
    if ((header==0xA55A) && (len <= 1024)) // 校验头和参数长度
    {
      xor_value = Parm_CalcXorValue(pbuf, len);
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
        FileIO_Write(FILE_M2M_PARA_CPY1, pbuf, 1024);
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

  pbuf[len++] = 0xA5; // 检验头
  pbuf[len++] = 0x5A;
  //==参数================================
  len += 2;  // 数据长度
  len += 1;  // TLV个数
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // 创建参数TLV
    if (tempVal!=0x00) // TLV有效
    {
      len += tempVal;
      tlvNum++;
    }
  }

  pbuf[2] = (len>>8) & 0xFF; // 长度
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV个数
  //======================================
  xor_value = Parm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // 校验值

  FileIO_Write(FILE_M2M_PARA_CPY1, pbuf, len);
  usleep(10); // 延时10us
  FileIO_Write(FILE_M2M_PARA_CPY2, pbuf, len);
  usleep(10); // 延时10us
  // system("sync");
}

/******************************************************************************
* 远程固件更新
******************************************************************************/
void rfu_EraseFlashHexFile(rfu_context_t* pThis)
{
  if(pThis->dev==0x00)  // 目标设备: 0x00=终端,0x01=控制器,0x02=显示器,0x03=协处理器
  {
    system("rm -f /data/helloworld_bak");  // 删除原有升级文件
    system("rm -f /cache/helloworld_bak_ver.txt");  // 删除备份区文件
    system("rm -f /cache/helloworld_bak");
  }
  else if(pThis->dev==0x01)
  {
    system("rm -f /media/card/ecu_firmware");  // 删除原有外部MCU器升级文件
  }
  else if(pThis->dev==0x03)
  {
    system("rm -f /data/st_firmware");  // 删除原有协处理器升级文件
  }
  system("sync");
  sleep(1);
}

/* ================================================================== */
void rfu_SaveFlashHexFile(rfu_context_t* pThis, uint8_t *buf, uint16_t length)
{
  if(pThis->dev==0x00) // 0x00=终端
  {
    FileIO_RandomWrite(FILE_4G_APP_FIRMWARE, pThis->cumulated_address, buf, length);
  }
  else if(pThis->dev==0x01) // 0x01=控制器
  {
    FileIO_RandomWrite(FILE_ECU_FIRMWARE, pThis->cumulated_address, buf, length);
  }
  else if(pThis->dev==0x03) // 0x03=协处理器
  {
    FileIO_RandomWrite(FILE_ST_FIRMWARE, pThis->cumulated_address, buf, length);
  }
}

/* ================================================================== */
void rfu_ReadFlashHexFile(rfu_context_t* pThis, uint32_t address, uint8_t *buf, uint32_t cnt)
{
  if(pThis->dev==0x00) // 0x00=终端
  {
    FileIO_RandomRead(FILE_4G_APP_FIRMWARE, address, buf, length);
  }
  else if(pThis->dev==0x01) // 0x01=控制器
  {
    FileIO_RandomRead(FILE_ECU_FIRMWARE, address, buf, length);
  }
  else if(pThis->dev==0x03) // 0x03=协处理器
  {
    FileIO_RandomRead(FILE_ST_FIRMWARE, address, buf, length);
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

    rfu_ReadFlashHexFile(pThis, pThis->cumulated_address, buffer, size);
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


