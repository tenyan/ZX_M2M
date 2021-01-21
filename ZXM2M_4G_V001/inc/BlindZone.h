/*****************************************************************************
* @FileName: BlindZone.h
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2021-01-19
* @brief     ä�����ݲ�������
******************************************************************************/
#ifndef _BLIND_ZONE_H_
#define _BLIND_ZONE_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "config.h"

/******************************************************************************
* Macros
******************************************************************************/
#define BLIND_ZONE_STACK_MAX_SIZE  8400

/******************************************************************************
 * Data Types
 ******************************************************************************/
// error code
enum
{
  FILE_IO_SUCCESS = 0, // success
  FILE_IO_ERROR = 1,   // fail
};

// ���籣�����
typedef struct
{
  uint32_t header; // У���ֶ�  0x55AA5AA5

  uint8_t wr_error_cnt; // ���������
  uint8_t rd_error_cnt; // ���������
  uint16_t top;      // stack top
  uint16_t bottom;   // stack bottom
  uint16_t data[BLIND_ZONE_STACK_MAX_SIZE];  // ���ݻ�����(����֡����)

  uint8_t crc;  // Add a CRC to the assest data structure
}blind_zone_para_t;
#define BLIND_ZONE_PARA_SIZE  sizeof(blind_zone_para_t)

// ��Ĥ�ṹ��
typedef struct
{
  uint8_t chMask[BLIND_ZONE_PARA_SIZE];
}__blind_zone_para_t;

// ä�������ṹ��
typedef struct
{
  uint16_t timer_100ms;

  uint16_t frame_size;
  const char *file_name;

  uint8_t wr_error_cnt;
  uint8_t rd_error_cnt;
  uint16_t top;     // stack top
  uint16_t bottom;  // stack bottom
  uint16_t data[BLIND_ZONE_STACK_MAX_SIZE];  // ���ݻ�����(����֡����)

  pthread_mutex_t file_mutex;
}blind_zone_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void BlindZone_SaveParameter(const char *file_name, blind_zone_para_t* pThis);
void BlindZone_ReadParameter(const char *file_name, blind_zone_para_t* pThis);

void BlindZone_PushData(blind_zone_t* pThis, uint8_t* p_data, uint16_t size);
void BlindZone_PopData(blind_zone_t* pThis, uint8_t* p_data, uint16_t* psize);

uint16_t BlindZone_GetStackSize(blind_zone_t* pThis);

#endif /* _BLIND_ZONE_H_ */

