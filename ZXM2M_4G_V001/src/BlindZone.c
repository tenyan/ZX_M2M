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

