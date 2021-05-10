/*****************************************************************************
* @FileName: Parameters.c
* @Engineer: TenYan
* @version   V1.0
* @Date:     2021-1-1
* @brief     �����洢����
* @������:     sync��������ǿ�Ʊ��ı����������д����̣����³�����Ϣ��
* ��Linux/Unixϵͳ��,���ļ������ݴ��������һ���ȷŵ��ڴ滺������,
* �ȵ��ʵ���ʱ����д�����,�����ϵͳ������Ч�ʡ�
* sync�����������ǿ�ƽ��ڴ滺�����е���������д������С�
* �û�ͨ������ִ��sync����,ϵͳ���Զ�ִ��update��bdflush����,
* ��������������д����̡�
* ֻ����update��bdflush�޷�ִ�л��û���Ҫ�������ػ�ʱ,�����ֶ�ִ��sync���
******************************************************************************/
#include "config.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define PARM_DELAY(ms)    msleep(ms)

extern uint16_t im2m_AnalyzeParaData(uint16_t tag, uint16_t len, uint8_t *pValue);
extern uint16_t im2m_BuildParaData(uint8_t *pbuf, uint16_t tag);

uint8_t public_data_buffer[1460];

/******************************************************************************
 * read data from file (Random Access File)
 *
 * @param file_name file path
 * @param offset_addr start address
 * @param pbuf read data pointer
 * @param size read size
 *
 * @return 0-�����ɹ� -1-����ʧ��
 ******************************************************************************/
int8_t FileIO_RandomRead(const char *file_name, uint32_t offset_addr, uint8_t *pbuf, uint32_t size)
{
  int fd_rd;
  int new_offset;

  fd_rd = open(file_name, O_RDONLY); // ��ֻ����ʽ���ļ�
  if (fd_rd < 0)
  {
    printf("ERROR R_open %s fd=%d\n\r", file_name, fd_rd);
    close(fd_rd);
    return -1;
  }

  // �����µĶ�дλ��Ϊ���ļ���ͷ����ƫ��offset_addr���ֽ�
  new_offset = lseek(fd_rd, offset_addr, SEEK_SET); // �����ļ�����ʼ��дλ��
  if (new_offset == -1)
  {
    printf("ERROR R_lseek %s fd=%d\n\r", file_name, offset_addr);
    close(fd_rd);
    return -1;
  }

  read(fd_rd, pbuf, size); // ��ȡ�ļ�
  close(fd_rd); // �ر��ļ�

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
 * @return 0-�����ɹ� -1-����ʧ��
 ******************************************************************************/
int8_t FileIO_RandomWrite(const char *file_name, uint32_t offset_addr, uint8_t *pdata, uint32_t size)
{
  int fd_wr;
  int new_offset;

  fd_wr = open(file_name, O_RDWR|O_CREAT); // ���ļ�
  if (fd_wr < 0)
  {
    printf("ERROR W_open %s fd=%d\n\r", file_name, fd_wr);
    close(fd_wr);
    return -1;
  }

  // �����µĶ�дλ��Ϊ���ļ���ͷ����ƫ��addr�ֽ�
  new_offset = lseek(fd_wr, offset_addr, SEEK_SET); // �����ļ�����ʼ��дλ��
  if (new_offset == -1)
  {
    printf("ERROR W_lseek %s fd=%d\n\r", file_name, offset_addr);
    close(fd_wr);
    return -1;
  }

  write(fd_wr, pdata, size); // д���ļ�������
  // fsync(fd_wr); // ͬ���ļ�
  close(fd_wr); // �ر��ļ�

  return 0;
}

/******************************************************************************
 * write data to file
 *
 * @param file_name file path
 * @param pdata write data
 * @param size write size
 *
 * @return 0-�����ɹ� -1-����ʧ��
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
  fsync(fd_wr);  // ͬ���ļ�(�ȴ�д���̲�������)
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
 * @return 0-�����ɹ� -1-����ʧ��
 ******************************************************************************/
int8_t FileIO_Read(const char *file_name, uint8_t *pbuf, uint32_t size)
{
  int fd_rd;

  fd_rd = open(file_name, O_RDONLY); // ��ֻ����ʽ���ļ�
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
 * �������У��ֵ
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
* ������Ϣ
******************************************************************************/
void Parm_SaveLvcInfo(void)
{
#if 0
  //Calculate the Xor value so it is stored into memory
  lvc_context.xor_value = Parm_CalcXorValue((uint8_t *) &lvc_context, (SIZEOF_LVC_CONTEXT - 1));

  FileIO_Write(FILE_USER_LVC_CPY1, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_USER_LVC_CPY2, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
#endif
}

//============================================================================
void Parm_ReadLvcInfo(void)
{
#if 0
  lvc_context_t lvc_ctx1;
  lvc_context_t lvc_ctx2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_USER_LVC_CPY1, &lvc_ctx1, SIZEOF_LVC_CONTEXT); // ��ȡ������
  if (Parm_CalcXorValue((uint8_t *) &lvc_ctx1, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx1.xor_value && (lvc_ctx1.header == PARM_DATA_HEADER))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_USER_LVC_CPY2, &lvc_ctx2, SIZEOF_LVC_CONTEXT); // ��ȡ������
  if (Parm_CalcXorValue((uint8_t *) &lvc_ctx2, (SIZEOF_LVC_CONTEXT-1)) == lvc_ctx2.xor_value && (lvc_ctx2.header == PARM_DATA_HEADER))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    memcpy(&lvc_context, &lvc_ctx1, SIZEOF_LVC_CONTEXT);
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc2Err!\n");
      FileIO_Write(FILE_USER_LVC_CPY2, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
    }

    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    memcpy(&lvc_context, &lvc_ctx2, SIZEOF_LVC_CONTEXT);
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmLvc1Err!\n");
      FileIO_Write(FILE_USER_LVC_CPY1, (uint8_t *)&lvc_context, SIZEOF_LVC_CONTEXT);
    }
    return;
  }

  PcDebug_SendString("ParmLvc1_2Err!\n"); // �洢������
#endif
}

extern uint16_t hjep_login_sn;
extern uint16_t hjep_send_data_sn;

extern uint16_t gbep_login_sn;
extern uint16_t gbep_send_data_sn;

/******************************************************************************
* ��ˮ����Ϣ
******************************************************************************/
void Parm_SaveSnInfo(void)
{
  uint8_t buf[5];
  uint8_t len = 0;
  uint8_t check_sum;
  uint16_t days;
  rtc_date_t time;

  time = RTC_GetBjTime(); // ��ȡ��ǰʱ��
  RTC_ConvertDataTimeToDays(&time, &days); // ��ȡ��ǰ����
  
  buf[len++] = 0xA5;  //  У��ͷ
  buf[len++] = 0x5A;
  
  buf[len++] = (uint8_t)(days>>8);  // ��ǰ����
  buf[len++] = (uint8_t)days;

  buf[len++] = (uint8_t)(m2m_context.upload_sn>>8);  // M2MЭ�鱨����ˮ��
  buf[len++] = (uint8_t)m2m_context.upload_sn;

  buf[len++] = (uint8_t)(hjep_login_sn>>8);  // HJ������¼��ˮ��
  buf[len++] = (uint8_t)hjep_login_sn;

  buf[len++] = (uint8_t)(hjep_send_data_sn>>8);  // HJ����������ˮ��
  buf[len++] = (uint8_t)hjep_send_data_sn;

  buf[len++] = (uint8_t)(gbep_login_sn>>8);  // GB������¼��ˮ��
  buf[len++] = (uint8_t)gbep_login_sn;

  buf[len++] = (uint8_t)(gbep_send_data_sn>>8);  // GB����������ˮ��
  buf[len++] = (uint8_t)gbep_send_data_sn;

  check_sum = Parm_CalcXorValue(buf, len);
  buf[len++] = check_sum; // У���

  FileIO_Write(FILE_SERIAL_NUMBER_CPY1, buf, len);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_SERIAL_NUMBER_CPY2, buf, len);
}

//============================================================================
void Parm_ReadSnInfo(void)
{
  uint8_t m_buf[20];
  uint8_t xor_value1;
  uint8_t r_buf[20];
  uint8_t xor_value2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_SERIAL_NUMBER_CPY1, m_buf, 15); // ��ȡ������
  xor_value1 = Parm_CalcXorValue(m_buf, 14);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[14]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_SERIAL_NUMBER_CPY2, r_buf, 15); // ��ȡ������
  xor_value2 = Parm_CalcXorValue(r_buf, 14);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[14]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.ep_valid_flag = m_buf[2]; // ����������Ч��־
    zxtcw_context.ep_type = m_buf[3];  // ��������
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmEpType2Err!\n");
      FileIO_Write(FILE_SERIAL_NUMBER_CPY2, m_buf, 5);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.ep_valid_flag = r_buf[2]; // ����������Ч��־
    zxtcw_context.ep_type = r_buf[3];  // ��������
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmEpType1Err!\n");
      FileIO_Write(FILE_SERIAL_NUMBER_CPY1, r_buf, 5);
    }
    return;
  }

  zxtcw_context.ep_valid_flag = EP_ENABLE;
  zxtcw_context.ep_type = EP_TYPE_HJ;
  PcDebug_SendString("ParmEpType1_2Err!\n"); // �洢������
}


/******************************************************************************
* ����������Ϣ
******************************************************************************/
void Parm_SaveEpTypeInfo(void)
{
  uint8_t buf[5];
  
  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;
  buf[2] = zxtcw_context.bk_ep_valid_flag;  // ����������Ч��־
  buf[3] = zxtcw_context.bk_ep_type;  // ��������
  buf[4] = Parm_CalcXorValue(buf, 4); // У���

  FileIO_Write(FILE_USER_EP_TYPE_CPY1, buf, 5);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_USER_EP_TYPE_CPY2, buf, 5);
  PARM_DELAY(5); // ��ʱ5ms
}

//============================================================================
void Parm_ReadEpTypeInfo(void)
{
  uint8_t m_buf[5];
  uint8_t xor_value1;
  uint8_t r_buf[5];
  uint8_t xor_value2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_USER_EP_TYPE_CPY1, m_buf, 5); // ��ȡ������
  xor_value1 = Parm_CalcXorValue(m_buf, 4);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[4]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_USER_EP_TYPE_CPY2, r_buf, 5); // ��ȡ������
  xor_value2 = Parm_CalcXorValue(r_buf, 4);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[4]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.ep_valid_flag = m_buf[2]; // ����������Ч��־
    zxtcw_context.ep_type = m_buf[3];  // ��������
    zxtcw_context.bk_ep_valid_flag = m_buf[2]; // ����������Ч��־
    zxtcw_context.bk_ep_type = m_buf[3];  // ��������
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmEpType2Err!\n");
      FileIO_Write(FILE_USER_EP_TYPE_CPY2, m_buf, 5);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.ep_valid_flag = r_buf[2]; // ����������Ч��־
    zxtcw_context.ep_type = r_buf[3];  // ��������
    zxtcw_context.bk_ep_valid_flag = r_buf[2]; // ����������Ч��־
    zxtcw_context.bk_ep_type = r_buf[3];  // ��������
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmEpType1Err!\n");
      FileIO_Write(FILE_USER_EP_TYPE_CPY1, r_buf, 5);
    }
    return;
  }

  zxtcw_context.ep_valid_flag = EP_ENABLE;
  zxtcw_context.ep_type = EP_TYPE_HJ;
  zxtcw_context.bk_ep_valid_flag = EP_ENABLE;
  zxtcw_context.bk_ep_type = EP_TYPE_HJ;
  PcDebug_SendString("ParmEpType1_2Err!\n"); // �洢������
}

/******************************************************************************
* ������λ��Ϣ
******************************************************************************/
void Parm_SaveGpsNavInfo(void)
{
  uint8_t buf[15];
  
  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;

  buf[2] = nav_data.ns;
  buf[3] = (uint8_t)((nav_data.lat>>24) & 0xFF);  /// γ�� ��λ:�����֮һ��
  buf[4] = (uint8_t)((nav_data.lat>>16) & 0xFF);
  buf[5] = (uint8_t)((nav_data.lat>>8) & 0xFF);
  buf[6] = (uint8_t)( nav_data.lat & 0xFF );

  buf[7] = nav_data.ew;
  buf[8] = (uint8_t)((nav_data.lon>>24) & 0xFF);  /// ���� ��λ:�����֮һ��
  buf[9] = (uint8_t)((nav_data.lon>>16) & 0xFF);
  buf[10] = (uint8_t)((nav_data.lon>>8) & 0xFF);
  buf[11] = (uint8_t)( nav_data.lon & 0xFF );

  buf[12] = (uint8_t)((nav_data.alt>>8) & 0xFF);  /// ���θ߶�(��),��ƽ������Ϊ����
  buf[13] = (uint8_t)( nav_data.alt & 0xFF );
  buf[14] = Parm_CalcXorValue(buf, 14); // У���

  FileIO_Write(FILE_GPS_NAV_INFO_CPY1, buf, 15);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_GPS_NAV_INFO_CPY2, buf, 15);
  PARM_DELAY(5); // ��ʱ5ms
}

//============================================================================
void Parm_ReadGpsNavInfo(void)
{
  uint8_t m_buf[15];
  uint8_t xor_value1;
  uint8_t r_buf[15];
  uint8_t xor_value2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_GPS_NAV_INFO_CPY1, m_buf, 15); // ��ȡ������
  xor_value1 = Parm_CalcXorValue(m_buf, 14);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[14]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_GPS_NAV_INFO_CPY2, r_buf, 15); // ��ȡ������
  xor_value2 = Parm_CalcXorValue(r_buf, 14);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[14]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    nav_data.ns = m_buf[2];
    nav_data.lat = m_buf[3];
    nav_data.lat <<= 8;
    nav_data.lat += m_buf[4];
    nav_data.lat <<= 8;
    nav_data.lat += m_buf[5];
    nav_data.lat <<= 8;
    nav_data.lat += m_buf[6];

    nav_data.ew = m_buf[7];
    nav_data.lon = m_buf[8];
    nav_data.lon <<= 8;
    nav_data.lon += m_buf[9];
    nav_data.lon <<= 8;
    nav_data.lon += m_buf[10];
    nav_data.lon <<= 8;
    nav_data.lon += m_buf[11];

    nav_data.alt = m_buf[12];
    nav_data.alt <<= 8;
    nav_data.alt += m_buf[13];
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmGpsNavInfo2Err!\n");
      FileIO_Write(FILE_GPS_NAV_INFO_CPY2, m_buf, 15);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    nav_data.ns = r_buf[2];
    nav_data.lat = r_buf[3];
    nav_data.lat <<= 8;
    nav_data.lat += r_buf[4];
    nav_data.lat <<= 8;
    nav_data.lat += r_buf[5];
    nav_data.lat <<= 8;
    nav_data.lat += r_buf[6];

    nav_data.ew = r_buf[7];
    nav_data.lon = r_buf[8];
    nav_data.lon <<= 8;
    nav_data.lon += r_buf[9];
    nav_data.lon <<= 8;
    nav_data.lon += r_buf[10];
    nav_data.lon <<= 8;
    nav_data.lon += r_buf[11];

    nav_data.alt = r_buf[12];
    nav_data.alt <<= 8;
    nav_data.alt += r_buf[13];
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmGpsNavInfo1Err!\n");
      FileIO_Write(FILE_GPS_NAV_INFO_CPY1, r_buf, 15);
    }
    return;
  }

  PcDebug_SendString("ParmGpsNavInfo1_2Err!\n"); // �洢������
}

/******************************************************************************
* VIN����Ϣ
******************************************************************************/
void Parm_SaveVinInfo(void)
{
  uint8_t buf[25];
  
  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;
  buf[2] = zxtcw_context.vin_valid_flag; // �����ر�
  buf[3] = zxtcw_context.vin_size;       // VIN����
  memcpy(&buf[4], zxtcw_context.vin, zxtcw_context.vin_size); // 17���ֽ�VIN��
  buf[24] = Parm_CalcXorValue(buf, 24); // У���

  FileIO_Write(FILE_USER_VIN_CPY1, buf, 25);
  PARM_DELAY(5); // ��ʱ5ms
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

  FileIO_Read(FILE_USER_VIN_CPY1, m_buf, 25); // ��ȡ������
  xor_value1 = Parm_CalcXorValue(m_buf, 24);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[24]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_USER_VIN_CPY2, r_buf, 25); // ��ȡ������
  xor_value2 = Parm_CalcXorValue(r_buf, 24);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[24]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    if(m_buf[3] <= VIN_BUFFER_SIZE)
    {
      zxtcw_context.vin_valid_flag = m_buf[2]; // �����ر�
      zxtcw_context.vin_size = m_buf[3];  // VIN����
      memcpy(zxtcw_context.vin, &m_buf[4], zxtcw_context.vin_size); // 17���ֽ�VIN��
      if (redundant_is_ok==PARM_FALSE)
      {
        PcDebug_SendString("ParmVin2Err!\n");
        FileIO_Write(FILE_USER_VIN_CPY2, m_buf, 25);
      }
      return;
    }
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    if(m_buf[3] <= VIN_BUFFER_SIZE)
    {
      zxtcw_context.vin_valid_flag = r_buf[2]; // �����ر�
      zxtcw_context.vin_size = r_buf[3];       // VIN����
      memcpy(zxtcw_context.vin, &r_buf[4], VIN_BUFFER_SIZE); // 17���ֽ�VIN��
      if (main_is_ok==PARM_FALSE)
      {
        PcDebug_SendString("ParmVin1Err!\n");
        FileIO_Write(FILE_USER_VIN_CPY1, r_buf, 25);
      }
      return;
    }
  }

  PcDebug_SendString("ParmVin1_2Err!\n"); // �洢������
}

/******************************************************************************
* ����������Ϣ��Ϣ
******************************************************************************/
void Parm_SavePidInfo(void)
{
  uint8_t buf[20];
  uint8_t len = 0;
  uint8_t check_sum;

  buf[len++] = 0xA5;  //  У��ͷ
  buf[len++] = 0x5A;
  buf[len++] = zxtcw_context.pid_up_type;      // �ϳ�����״̬��
  buf[len++] = zxtcw_context.pid_up_config1;   // �ϳ�����״̬��1
  buf[len++] = zxtcw_context.pid_up_config2;   // �ϳ�����״̬��2
  buf[len++] = zxtcw_context.pid_up_can;       // �ϳ�Э������״̬��
  buf[len++] = zxtcw_context.pid_down_type;    // ��������״̬��
  buf[len++] = zxtcw_context.pid_down_config1; // ��������״̬��1
  buf[len++] = zxtcw_context.pid_down_config2; // ��������״̬��2
  buf[len++] = zxtcw_context.pid_down_can;     // ����CANЭ��
  check_sum = Parm_CalcXorValue(buf, len);
  buf[len++] = check_sum; // У���

  FileIO_Write(FILE_PID_CPY1, buf, len);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_PID_CPY2, buf, len);
}

//============================================================================
void Parm_ReadPidInfo(void)
{
  uint8_t m_buf[25];
  uint8_t xor_value1;
  uint8_t r_buf[25];
  uint8_t xor_value2;
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_PID_CPY1, m_buf, 11); // ��ȡ������
  xor_value1 = Parm_CalcXorValue(m_buf, 10);
  if ((m_buf[0] == 0xA5) && (m_buf[1] == 0x5A) && (xor_value1== m_buf[10]))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_PID_CPY2, r_buf, 11); // ��ȡ������
  xor_value2 = Parm_CalcXorValue(r_buf, 10);
  if ((r_buf[0] == 0xA5) && (r_buf[1] == 0x5A) && (xor_value2== r_buf[10]))
  {
    redundant_is_ok = PARM_TRUE;
  }

  if (main_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.pid_up_type = m_buf[2];      // �ϳ�����״̬��
    zxtcw_context.pid_up_config1 = m_buf[3];   // �ϳ�����״̬��1
    zxtcw_context.pid_up_config2 = m_buf[4];   // �ϳ�����״̬��2
    zxtcw_context.pid_up_can = m_buf[5];       // �ϳ�Э������״̬��
    zxtcw_context.pid_down_type = m_buf[6];    // ��������״̬��
    zxtcw_context.pid_down_config1 = m_buf[7]; // ��������״̬��1
    zxtcw_context.pid_down_config2 = m_buf[8]; // ��������״̬��2
    zxtcw_context.pid_down_can = m_buf[9];     // ����CANЭ��
    if (redundant_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmPid2Err!\n");
      FileIO_Write(FILE_PID_CPY2, m_buf, 11);
    }
    return;
  }

  if (redundant_is_ok==PARM_TRUE) // ������OK
  {
    zxtcw_context.pid_up_type = r_buf[2];      // �ϳ�����״̬��
    zxtcw_context.pid_up_config1 = r_buf[3];   // �ϳ�����״̬��1
    zxtcw_context.pid_up_config2 = r_buf[4];   // �ϳ�����״̬��2
    zxtcw_context.pid_up_can = r_buf[5];       // �ϳ�Э������״̬��
    zxtcw_context.pid_down_type = r_buf[6];    // ��������״̬��
    zxtcw_context.pid_down_config1 = r_buf[7]; // ��������״̬��1
    zxtcw_context.pid_down_config2 = r_buf[8]; // ��������״̬��2
    zxtcw_context.pid_down_can = r_buf[9];     // ����CANЭ��
    if (main_is_ok==PARM_FALSE)
    {
      PcDebug_SendString("ParmPid1Err!\n");
      FileIO_Write(FILE_PID_CPY1, r_buf, 11);
    }
    return;
  }

  PcDebug_SendString("ParmPid1_2Err!\n"); // �洢������
}


/******************************************************************************
* �ۼƲ�����ʱ��
******************************************************************************/
void Parm_SaveOfflineTimeInfo(void)
{
  uint8_t buf[7];

  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_offline_time>>24) & 0xFF);  // ����
  buf[3] = (uint8_t)((colt_info.total_offline_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_offline_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_offline_time & 0xFF);
  buf[6] = Parm_CalcXorValue(buf, 6);  // У��

  FileIO_Write(FILE_OFFLINE_TIME_CPY1, buf, 7);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_OFFLINE_TIME_CPY2, buf, 7);
}

//============================================================================
void Parm_ReadOfflineTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_OFFLINE_TIME_CPY1, m_buf, 7); // ��ȡ������
  if (Parm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_OFFLINE_TIME_CPY2, r_buf, 7); // ��ȡ������
  if (Parm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
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
      FileIO_Write(FILE_OFFLINE_TIME_CPY2, m_buf, 7);
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
      FileIO_Write(FILE_OFFLINE_TIME_CPY1, r_buf, 7);
    }
    return;
  }
  //colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // �洢������
}

/******************************************************************************
* �ۼƹ���ʱ��
******************************************************************************/
void Parm_SaveWorkTimeInfo(void)
{
  uint8_t buf[7];

  buf[0] = 0xA5;  //  У��ͷ
  buf[1] = 0x5A;
  buf[2] = (uint8_t)((colt_info.total_work_time>>24) & 0xFF);  // ����
  buf[3] = (uint8_t)((colt_info.total_work_time>>16) & 0xFF);
  buf[4] = (uint8_t)((colt_info.total_work_time>>8) & 0xFF);
  buf[5] = (uint8_t)(colt_info.total_work_time & 0xFF);
  buf[6] = Parm_CalcXorValue(buf, 6);  // У��

  FileIO_Write(FILE_WORKTIME_CPY1, buf, 7);
  PARM_DELAY(5); // ��ʱ5ms
  FileIO_Write(FILE_WORKTIME_CPY2, buf, 7);
}

//============================================================================
void Parm_ReadWorkTimeInfo(void)
{
  uint8_t m_buf[7];
  uint8_t r_buf[7];
  uint8_t main_is_ok = PARM_FALSE;
  uint8_t redundant_is_ok = PARM_FALSE;

  FileIO_Read(FILE_WORKTIME_CPY1, m_buf, 7); // ��ȡ������
  if (Parm_CalcXorValue(m_buf, 6) == m_buf[6] && (m_buf[0] == 0xA5)&&(m_buf[1] == 0x5A))
  {
    main_is_ok = PARM_TRUE;
  }

  FileIO_Read(FILE_WORKTIME_CPY2, r_buf, 7); // ��ȡ������
  if (Parm_CalcXorValue(r_buf, 6) == r_buf[6] && (r_buf[0] == 0xA5)&&(r_buf[1] == 0x5A))
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
      FileIO_Write(FILE_WORKTIME_CPY2, m_buf, 7);
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
      FileIO_Write(FILE_WORKTIME_CPY1, r_buf, 7);
    }
    return;
  }

  // colt_info.total_work_time = 0;
  PcDebug_SendString("ParmWkt1_2Err!\n"); // �洢������
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
                     0x0114,0x0115,0x0116,0x0117,0x0118,0x0119,0x011a,0xA1FE
                   };
  uint8_t totalTlvNum = (sizeof(tag)/sizeof(tag[0]));
  uint8_t tlvNum = 0;

  PcDebug_SendString("SaveM2mParm!\n");

  pbuf[len++] = 0xA5; // ����ͷ
  pbuf[len++] = 0x5A;
  //==����================================
  len += 2;  // ���ݳ���
  len += 1;  // TLV����
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // ��������TLV
    if (tempVal!=0x00) // TLV��Ч
    {
      len += tempVal;
      tlvNum++;
    }
  }

  pbuf[2] = (len>>8) & 0xFF; // ����
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV����
  //======================================
  xor_value = Parm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // У��ֵ

  FileIO_Write(FILE_M2M_PARA_CPY1, pbuf, len);
  usleep(10); // ��ʱ10us
  FileIO_Write(FILE_M2M_PARA_CPY2, pbuf, len);
  usleep(10); // ��ʱ10us
  //system("sync");
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

  FileIO_Read(FILE_M2M_PARA_CPY1, pbuf, 1024); // ��ȡ������
  header = pbuf[pos++]; // У��ͷ
  header <<= 8;;
  header += pbuf[pos++];
  len = pbuf[pos++]; // ����(����У��ͷ2B+����2B)
  len <<= 8;
  len += pbuf[pos++];
  tlvNum = pbuf[pos++]; // tlv����
  if ((header==0xA55A) && (len <= 1024)) // У��ͷ�Ͳ�������
  {
    xor_value = Parm_CalcXorValue(pbuf, len);
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
    FileIO_Read(FILE_M2M_PARA_CPY2, pbuf, 1024); // ��ȡ������
    header = pbuf[pos++]; // У��ͷ
    header <<= 8;;
    header += pbuf[pos++];
    len = pbuf[pos++]; // ����(����У��ͷ2B+����2B)
    len <<= 8;
    len += pbuf[pos++];
    tlvNum = pbuf[pos++]; // tlv����
    if ((header==0xA55A) && (len <= 1024)) // У��ͷ�Ͳ�������
    {
      xor_value = Parm_CalcXorValue(pbuf, len);
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

  pbuf[len++] = 0xA5; // ����ͷ
  pbuf[len++] = 0x5A;
  //==����================================
  len += 2;  // ���ݳ���
  len += 1;  // TLV����
  for (it=0; it<totalTlvNum; it++)
  {
    tempVal = im2m_BuildParaData(&pbuf[len], tag[it]);  // ��������TLV
    if (tempVal!=0x00) // TLV��Ч
    {
      len += tempVal;
      tlvNum++;
    }
  }

  pbuf[2] = (len>>8) & 0xFF; // ����
  pbuf[3] = len & 0xFF;
  pbuf[4] = tlvNum;  // TLV����
  //======================================
  xor_value = Parm_CalcXorValue(pbuf, len);
  pbuf[len++] = xor_value; // У��ֵ

  FileIO_Write(FILE_M2M_PARA_CPY1, pbuf, len);
  usleep(10); // ��ʱ10us
  FileIO_Write(FILE_M2M_PARA_CPY2, pbuf, len);
  usleep(10); // ��ʱ10us
  // system("sync");
}

/******************************************************************************
* Զ�̹̼�����
******************************************************************************/
void rfu_EraseFlashHexFile(rfu_context_t* pThis)
{
  pThis->cumulated_address = 0x00;

  if(pThis->dev==0x00)  // Ŀ���豸: 0x00=�ն�,0x01=������,0x02=��ʾ��,0x03=Э������
  {
    system("rm -f /data/helloworld_bak");  // ɾ��ԭ�������ļ�
    system("rm -f /cache/helloworld_bak_ver.txt");  // ɾ���������ļ�
    system("rm -f /cache/helloworld_bak");
  }
  else if(pThis->dev==0x01)
  {
    system("rm -f /media/card/ecu_firmware");  // ɾ��ԭ���ⲿMCU�������ļ�
  }
  else if(pThis->dev==0x03)
  {
    system("rm -f /data/st_firmware");  // ɾ��ԭ��Э�����������ļ�
  }
  system("sync");
  sleep(1);
}

/* ================================================================== */
void rfu_SaveFlashHexFile(rfu_context_t* pThis, uint8_t *buf, uint16_t length)
{
  if(pThis->dev==0x00) // 0x00=�ն�
  {
    FileIO_RandomWrite(FILE_4G_APP_FIRMWARE, pThis->cumulated_address, buf, length);
  }
  else if(pThis->dev==0x01) // 0x01=������
  {
    FileIO_RandomWrite(FILE_ECU_FIRMWARE, pThis->cumulated_address, buf, length);
  }
  else if(pThis->dev==0x03) // 0x03=Э������
  {
    FileIO_RandomWrite(FILE_ST_FIRMWARE, pThis->cumulated_address, buf, length);
  }

  pThis->cumulated_address += length;
}

/* ================================================================== */
void rfu_ReadFlashHexFile(uint8_t file_dev, uint32_t address, uint8_t *buf, uint16_t length)
{
  if(file_dev==0x00) // 0x00=�ն�
  {
    FileIO_RandomRead(FILE_4G_APP_FIRMWARE, address, buf, length);
  }
  else if(file_dev==0x01) // 0x01=������
  {
    FileIO_RandomRead(FILE_ECU_FIRMWARE, address, buf, length);
  }
  else if(file_dev==0x03) // 0x03=Э������
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

    rfu_ReadFlashHexFile(pThis->dev, pThis->cumulated_address, buffer, size);
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


