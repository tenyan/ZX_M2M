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

