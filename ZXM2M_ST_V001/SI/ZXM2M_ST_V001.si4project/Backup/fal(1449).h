/*****************************************************************************
* @FileName: fal.h
* @Engineer: armink & TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2021-1-27
* @brief     This file is part of FAL (Flash Abstraction Layer) package.
******************************************************************************/
#ifndef _FAL_H_
#define _FAL_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "types.h"

/******************************************************************************
 * 移植定义
 ******************************************************************************/
#define FAL_DEBUG  1

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define FAL_PRINTF  PcDebug_Printf

#if FAL_DEBUG // 调试信息(带颜色特效控制)
 #define FAL_ASSERT(EXPR)  if (!(EXPR)) {  FAL_PRINTF("(%s) has assert failed at %s.\n", #EXPR, __FUNCTION__); while (1);}
 #define FAL_LOG_D(...)  do{ FAL_PRINTF("[D/FAL] (%s:%d) ", __FUNCTION__, __LINE__); FAL_PRINTF(__VA_ARGS__); FAL_PRINTF("\n");}while(0)// debug level log
 #define FAL_LOG_E(...)  do{ FAL_PRINTF("\033[31;22m[E/FAL] (%s:%d) ", __FUNCTION__, __LINE__); FAL_PRINTF(__VA_ARGS__); FAL_PRINTF("\033[0m\n");}while(0)  // error level log
 #define FAL_LOG_I(...)  do{ FAL_PRINTF("\033[32;22m[I/FAL] ");FAL_PRINTF(__VA_ARGS__);FAL_PRINTF("\033[0m\n");}while(0)  // info level log
#else
 #define FAL_ASSERT(EXPR)  ((void)0)
 #define FAL_LOG_D(...)  ((void)0)
 #define FAL_LOG_E(...)  ((void)0)
 #define FAL_LOG_I(...)  ((void)0)
#endif

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// FAL flash and partition device name max length
#define FAL_DEV_NAME_MAX  24
typedef struct
{
  char name[FAL_DEV_NAME_MAX];
  uint32_t addr; // flash device start address
  size_t len; // flash device len
  size_t blk_size; // the block size in the flash for erase minimum granularity
  struct
  {
    int (*init)(void);
    int (*read)(uint32_t offset, uint8_t *buf, size_t size);
    int (*write)(uint32_t offset, const uint8_t *buf, size_t size);
    int (*erase)(uint32_t offset, size_t size);
  } ops;
  size_t write_gran; // write minimum granularity, unit: bit. 1(nor flash)/ 8(stm32f4)/ 32(stm32f1)/ 64(stm32l4).0 will not take effect.
}fal_flash_dev_t;

//==FAL partition======
typedef struct
{
  uint32_t magic_word;
  char name[FAL_DEV_NAME_MAX]; // partition name
  char flash_name[FAL_DEV_NAME_MAX]; // flash device name for partition
  uint32_t offset; // partition offset address on flash device
  size_t len;
  uint32_t reserved;
}fal_partition_t;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
int FAL_Init(void);
const fal_flash_dev_t* FAL_FlashDeviceFind(const char *name);
const fal_partition_t* FAL_PartitionFind(const char *name);

int FAL_PartitionRead(const fal_partition_t *part, uint32_t addr, uint8_t *buf, size_t size);
int FAL_PartitionWrite(const fal_partition_t *part, uint32_t addr, const uint8_t *buf, size_t size);
int FAL_PartitionErase(const fal_partition_t *part, uint32_t addr, size_t size);
int FAL_PartitionEraseAll(const fal_partition_t *part);

//const fal_partition_t* FAL_GetPartitionTable(size_t *len);
//void FAL_SetPartitionTableTemp(fal_partition_t *table, size_t len);
void FAL_ShowPartTalbe(void);

#endif /* _FAL_H_ */

