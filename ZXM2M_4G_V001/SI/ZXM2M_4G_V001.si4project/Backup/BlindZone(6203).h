/*****************************************************************************
* @FileName: BlindZone.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2021-01-19
* @brief     盲区数据补发定义
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
#define BLIND_ZONE_STACK_MAX_SIZE         840
#define ZXM2M_BZ_SAVE_PERIOD_SP  60 // 5分钟
#define HJEP_BZ_SAVE_PERIOD_SP  60 // 2分钟
#define GBEP_BZ_SAVE_PERIOD_SP  60 // 2分钟

//#define ZXM2M_BZ_SAVE_PERIOD_SP    30 // 30秒  测试用
//#define HJEP_BZ_SAVE_PERIOD_SP  20 // 20秒 测试用

#define ZXM2M_BZ_DEBUG    0  // 1-使能, 0-禁止
#define HJEP_BZ_DEBUG    0  // 1-使能, 0-禁止
#define GBEP_BZ_DEBUG    0  // 1-使能, 0-禁止

/******************************************************************************
 * Data Types
 ******************************************************************************/
// error code
enum
{
  FILE_IO_SUCCESS = 0, // success
  FILE_IO_ERROR = 1,   // fail
};

// 盲区补发结构体
typedef struct
{
  uint16_t timer_1s;

  uint16_t frame_size;
  const char *file_name;

  uint8_t wr_error_cnt;
  uint8_t rd_error_cnt;
  uint16_t top;     // stack top
  uint16_t bottom;  // stack bottom
  uint16_t data[BLIND_ZONE_STACK_MAX_SIZE];  // 数据缓冲区(数据帧长度)

  pthread_mutex_t file_mutex;
}blind_zone_t;
extern blind_zone_t zxm2m_blind_zone;
extern blind_zone_t hjep_blind_zone;
extern blind_zone_t gbep_blind_zone;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void ZxM2mBlindZone_Init(void);
void ZxM2mBlindZone_Service(void);
void ZxM2mBlindZone_SendData(void);

void HjepBlindZone_Init(void);
void HjepBlindZone_Service(void);
int32_t HjepBlindZone_SendData(void);

void GbepBlindZone_Init(void);
void GbepBlindZone_Service(void);
int32_t GbepBlindZone_SendData(void);

#endif /* _BLIND_ZONE_H_ */

