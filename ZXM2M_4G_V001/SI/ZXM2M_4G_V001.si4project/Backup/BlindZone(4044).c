/*****************************************************************************
* @FileName: BlindZone.c
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version:  V1.0
* @Date:     2021-1-19
* @brief:    盲区数据补发功能
*
* 设计原理
* 信息数据=带工况(上车或下车或有效定位)的数据包
* 实际工作中，终端内部4G模块遇到基站切换、网络信号不好，会导致网络中断，无法把采集的数据
* 发送到远端服务器，终端需要将数据信息缓存到本地存储器上，等待联网成功后，将补报的信息数据
* 上传到服务器。
* 针对设备实际情况，盲区的时间长度是不可知，即数据长度也是未知的。我们比较关心最新的数据，
* 而存储是有限的，所以需要在大量数据保存的时候，把旧的数据不停的覆盖掉，只保留最新的数据，
* 并且第一时间上报最新的数据，这是一种先入后出(First Input Last Output)的存储方式。
* 创建一个先入后出、可循环覆盖(栈底可变化)的栈，用于缓存每一条数据帧的长度。
* 这个栈还需存储在一个PARA文件中，并添加校验值。
* 数据帧被存储在一个DATA文件中，文件为每条数据帧分配固定长度N Bytes,
* 每条数据帧存储的起始地址等于栈顶值乘以固定长度。
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
 * 计算异或校验值
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

#if (PART("FILO队列"))
/******************************************************************************
* 保存盲区参数到FLASH
******************************************************************************/
void BlindZone_SaveParameter(const char *file_name, blind_zone_para_t* pThis)
{
  __blind_zone_para_t* p_blind_zone_para = (__blind_zone_para_t*) pThis;

  pThis->header = BLIND_ZONE_PARA_HEADER;
  pThis->crc = BlindZone_CalcXorCheck(p_blind_zone_para->chMask, (BLIND_ZONE_PARA_SIZE - 1));  // 计算CRC
  FileIO_Write(file_name, p_blind_zone_para->chMask, BLIND_ZONE_PARA_SIZE);
}

/******************************************************************************
* 从FLASH中读取盲区参数
******************************************************************************/
void BlindZone_ReadParameter(const char *file_name, blind_zone_para_t* pThis)
{
  const char *zxm2m_file_name = FILE_ZXM2M_BZ_PARA_ADDR;
  const char *hjep_file_name = FILE_HJEP_BZ_PARA_ADDR;
  const char *gbep_file_name = FILE_GBEP_BZ_PARA_ADDR;

  __blind_zone_para_t* p_blind_zone_para = (__blind_zone_para_t*)pThis;

  // 从Flash中读出数据
  FileIO_Read(file_name, p_blind_zone_para->chMask, BLIND_ZONE_PARA_SIZE);

  // 校验CRC和文件头
  if (BlindZone_CalcXorCheck(p_blind_zone_para->chMask, (BLIND_ZONE_PARA_SIZE - 1)) != pThis->crc || (pThis->header != BLIND_ZONE_PARA_HEADER))
  {
    // If the CRCs did not match, load defaults and store into bothmemory locations
    pThis->header = BLIND_ZONE_PARA_HEADER;; // 校验字节

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
* 数据入栈
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

  pos	= (pThis->top + 1) % BLIND_ZONE_STACK_MAX_SIZE; // 栈顶新位置
  offset_addr = pos * pThis->frame_size; // 帧起始地址
  ret_val = FileIO_RandomWrite(pThis->file_name, offset_addr, p_data, pThis->frame_size); // 写入盲区的数据
  if (ret_val==1) // 写入失败
  {
    pThis->wr_error_cnt++;
    return;
  }

  pThis->wr_error_cnt = 0;
  if (pos == pThis->bottom) // 满状态,删除旧数据记录
  {
    pThis->bottom	= (pThis->bottom + 1) % BLIND_ZONE_STACK_MAX_SIZE; // 更新栈底位置
  }

  pThis->top = pos; // 更新栈顶值
  pThis->data[pThis->top] = size; // 存入长度

  pthread_mutex_unlock(&pThis->file_mutex);
}

/******************************************************************************
* 数据出栈
******************************************************************************/
void BlindZone_PopData(blind_zone_t* pThis, uint8_t* p_data, uint16_t* psize)
{
  uint32_t offset_addr;
  uint8_t ret_val;

  pthread_mutex_lock(&pThis->file_mutex);

  if (pThis->top == pThis->bottom) // 空状态
  {
    *psize = 0x0000; // 数据长度置0
  }
  else // 非空状态,获取数据
  {
    *psize = pThis->data[pThis->top]; // 获取数据帧长度
    if (*psize == 0)
    {
      pThis->data[pThis->top] = 0x00; // 长度清零
      pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // 更新栈顶位置
      return;
    }
    else
    {
      offset_addr = pThis->top * pThis->frame_size; // 帧起始地址
      ret_val = FileIO_RandomRead(pThis->file_name, offset_addr,p_data, *psize); // 读取盲区的数据
      if (ret_val==1) // 读取失败
      {
        pThis->rd_error_cnt++;
        *psize = 0x0000; // 数据长度置0
        pThis->data[pThis->top] = 0x00; // 长度清零
        pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // 更新栈顶位置
        return;
      }
      pThis->rd_error_cnt = 0x00;
      pThis->data[pThis->top] = 0x00; // 长度清零
      pThis->top = (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - 1) % BLIND_ZONE_STACK_MAX_SIZE); // 更新栈顶位置
    }
  }

  pthread_mutex_unlock(&pThis->file_mutex);
}

/******************************************************************************
* 获取栈的使用数量
******************************************************************************/
uint16_t BlindZone_GetStackSize(blind_zone_t* pThis)
{
  return (( pThis->top + BLIND_ZONE_STACK_MAX_SIZE - pThis->bottom) % BLIND_ZONE_STACK_MAX_SIZE);
}
#endif

#if 0//(PART("ZxM2m盲区补偿"))

blind_zone_t zxm2m_blind_zone;
blind_zone_para_t zxm2m_blind_zone_para;
static uint8_t zxm2m_blind_zone_buffer[ZXM2M_BLIND_ZONE_PACKET_SIZE];
static uint16_t zxm2m_blind_zone_length;
#define ZXM2M_BZ_DEBUG    0  // 1-使能, 0-禁止
#define ZXM2M_BZ_SAVE_PERIOD_SP  59 // 5分钟
//#define ZXM2M_BZ_SAVE_PERIOD_SP    29 // 30秒  测试用
#define ZXM2M_BLIND_ZONE_PACKET_SIZE  512
#define ZXM2M_BDZE_WRITE_ERROR_SP  6
#define ZXM2M_BDZE_READ_ERROR_SP   6

/******************************************************************************
* 补发重型M2M盲区数据
******************************************************************************/
void ZxM2mBlindZone_Init(void)
{
  uint16_t i;

  BlindZone_ReadParameter(FILE_ZXM2M_BZ_PARA_ADDR, &zxm2m_blind_zone_para); // 读取FLASH中的参数
  zxm2m_blind_zone.wr_error_cnt = zxm2m_blind_zone_para.wr_error_cnt;
  zxm2m_blind_zone.rd_error_cnt = zxm2m_blind_zone_para.rd_error_cnt;
  zxm2m_blind_zone.top = zxm2m_blind_zone_para.top;
  zxm2m_blind_zone.bottom = zxm2m_blind_zone_para.bottom;
  for (i=0; i<BLIND_ZONE_STACK_MAX_SIZE; i++)
  {
    zxm2m_blind_zone.data[i] = zxm2m_blind_zone_para.data[i];
  }

  zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP;
  zxm2m_blind_zone.file_name = FILE_ZXM2M_BZ_DATA_ADDR; // 文件名
  zxm2m_blind_zone.frame_size = ZXM2M_BLIND_ZONE_PACKET_SIZE; // 数据帧固定长度
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

//==GPS及采集数据(NB)===================================================================
uint16_t ZxBlindZone_BuildEngData(uint8_t* pdata)
{
  uint16_t len = 0;
  uint8_t* downCanLenPos;

  pdata[len++] = 1;  // GPS信息条数（1B）,固定为1
  memcpy(&pdata[len], (uint8_t*)&g_stuZXGPSPosition, 25);  // GPS位置信息格式(25B)
  len += 25;

  memcpy(&pdata[len], (uint8_t*)&stu_ZXVehicleState, 5);  // 车辆状态信息1-5(5B)
  len += 5;

  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>24) & 0xFF;  // 里程(4B)
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>16) & 0xFF;
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage>>8) & 0xFF;
  pdata[len++] = (uint8_t)(stu_SYSZXAlarmPosition.uiTravelMileage & 0xFF);

  // 下车信息字节数N(1B) +下车信息(NB)
  downCanLenPos = &pdata[len];  // 保存下车信息字节数再数据包中的地址
  len++;  // 指向下车信息
  *downCanLenPos = g_stuZXMcuData.ucMcuDownDataLen; //下车ECU参数字节数N
  if (g_stuZXMcuData.ucMcuDownDataLen)
  {
    memcpy(&pdata[len],g_stuZXMcuData.aucMcuDownData,g_stuZXMcuData.ucMcuDownDataLen);  // 下车信息
    len += g_stuZXMcuData.ucMcuDownDataLen;
  }

  if (g_stuZXMcuData.ucKxctAgFlag == CAN_DOWN_KXCT) // KXCT下车数据的故障参数段
  {
    g_stuZXMcuData.ucFaultDataLen = KXCT_FD_ReadAllFaultPacket(g_stuZXMcuData.aucFaultData,sfault_packet,mfault_packet);  // 下车故障参数
    memcpy(&pdata[len],g_stuZXMcuData.aucFaultData,g_stuZXMcuData.ucFaultDataLen);  // 故障参数
    len += g_stuZXMcuData.ucFaultDataLen;
    *downCanLenPos += g_stuZXMcuData.ucFaultDataLen;
  }

  // 上车作业参数字节数M(1B)+上车作业参数(MB)
  uCAN_AttachFirmwareVersion(); // 填入程序固件版本号
  pdata[len++] = g_stuZXMcuData.ucMcuUpDataLen;  //上车作业参数字节数M
  if (g_stuZXMcuData.ucMcuUpDataLen)
  {
    if (stu_SYSZXParamSet.stu_Sys_ZXReport.ucReturnMode==0xc0 && g_stuZXMcuData.aucMcuUpDataPre[0]) // 回传模式
    {
      uCAN_ReadTypicalWorkData();
      MCU_ClearPercentageMomentData();
    }
    memcpy(&pdata[len], g_stuZXMcuData.aucMcuUpData,g_stuZXMcuData.ucMcuUpDataLen); // 上车作业参数
    len += g_stuZXMcuData.ucMcuUpDataLen;
  }

  return len;
}

//==1s调用一次============================================================================
void ZxM2mBlindZone_Service(void)
{
  // 定位有效、有工况数据
  if (COLT_GetAccStatus()==1) // ACC打开
  {
    if (zxm2m_blind_zone.timer_1s)
      zxm2m_blind_zone.timer_1s--;
    else
    {
      zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP; // 五分钟一条
      if (NetSocket_GetLinkState(&zxm2m_socket) != SOCKET_LINK_STATE_READY) // 未连接
      {
        if (GPS_GetPositioningStatus()==1) // 终端已定位
        {
          memset(zxm2m_blind_zone_buffer,0xFF,ZXM2M_BLIND_ZONE_PACKET_SIZE); // 清空缓存
          zxm2m_blind_zone_length = ZxBlindZone_BuildEngData(zxm2m_blind_zone_buffer); // 创建盲区数据包: GPS及ECU数据
          if (zxm2m_blind_zone_length <= ZXM2M_BLIND_ZONE_PACKET_SIZE)
          {
            BlindZone_PushData(&zxm2m_blind_zone, zxm2m_blind_zone_buffer, zxm2m_blind_zone_length); // 存入数据帧
            ZxM2mBlindZone_Save(); // 存入栈参数

#if ZXM2M_BZ_DEBUG
            PcDebug_Printf("ZxM2mBzPush:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif
          }
        }
      }
    }
  }
  else // ACC关闭
  {
    zxm2m_blind_zone.timer_1s = ZXM2M_BZ_SAVE_PERIOD_SP; // 五分钟一条
  }

  // 出现读写错误,复位栈为0
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
    ZxM2mBlindZone_Save(); // 存入栈参数
  }
}

//==上发盲区补偿数据==============================================================================
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
    // 上行GPRS格式: 起始位(1B) + 数据格式(1B) + 长度(2B) + 数据类型(1B) + 数据内容(nB) + 结束位(2B)
    pdata[0] = 0x68; // 起始位
    pdata[1] = 'B'; // 数据格式
    pdata[4] = 0;  // 数据类型,0x00表示数据为GPS车载信息终端信息

    /*************************上行短信格式(数据内容)开始*************************/
    len = 5;
    pdata[len++] = 0x28; // 引导符
    pdata[len++] = 0x50; // 设备代码
    pdata[len++] = 0x18; // 回传信息标识

    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID & 0xFF);  // GPS车载信息终端序列号
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>8) & 0xFF;
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>16) & 0xFF;
    pdata[len++] = (uint8_t)(stu_SYSZXParamSet.uiDeviceID>>24) & 0xFF;

    // GPS车载信息终端的操作密码，在录入设备时输入，密码出厂默认为“Qh”
    memcpy(&pdata[len], (uint8_t*)&stu_SYSZXParamSet.aucLoginPassword[0], stu_SYSZXParamSet.ucLoginPasswordLen); // 密码（2B）
    len += stu_SYSZXParamSet.ucLoginPasswordLen;

    pdata[len++] = 0x12;  // 回传命令标志:上发盲区补偿数据

    g_stuZXSystem.usSerialnumber++;
    pdata[len++] = (uint8_t)(g_stuZXSystem.usSerialnumber & 0xFF); // 命令流水号
    pdata[len++] = (uint8_t)(g_stuZXSystem.usSerialnumber >> 8);

    // GPS及采集数据(NB)
    BlindZone_PopData(&zxm2m_blind_zone, &pdata[len], &bind_zone_data_size);
    ZxM2mBlindZone_Save(); // 存入栈参数
    if (bind_zone_data_size == 0x00)
    {
#if ZXM2M_BZ_DEBUG
      PcDebug_Printf("ZxM2mBzPop:Err\r\n");
#endif
      return;
    }

    len += bind_zone_data_size;

    sms_data_size = len - 5;
    pdata[len++] = XOR_Check(&pdata[5],sms_data_size); // 校验码(1B)
    sms_data_size += 1;
    /*************************上行短信格式(数据内容)结束*************************/

    gprs_data_size = (1 + sms_data_size + 2); // 长度: 数据类型(1B)+数据内容(nB)+结束位(2B)
    pdata[2] = (uint8_t)(gprs_data_size & 0xFF);
    pdata[3] = (uint8_t)(gprs_data_size>>8) & 0xFF;

    pdata[len++] = 0x0D; // 结束位
    pdata[len++] = 0x0A;
    NetSocket_Send(&zxm2m_socket, pdata, len);

#if ZXM2M_BZ_DEBUG
    PcDebug_Printf("ZxM2mBzPop:Ewr=%d,Erd=%d,Top=%d,Bot=%d\r\n",zxm2m_blind_zone.wr_error_cnt,zxm2m_blind_zone.rd_error_cnt,zxm2m_blind_zone.top,zxm2m_blind_zone.bottom);
#endif

  }
}
#endif

