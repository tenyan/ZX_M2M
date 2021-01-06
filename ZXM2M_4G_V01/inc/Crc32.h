/*****************************************************************************
* FileName:  Crc32.h
* Engineer:  TenYan
* Date:      2021-01-02
******************************************************************************/
#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>


/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
uint32_t GetCrc32(const void* data, uint16_t length);

uint32_t GetCrc32_Stream_Init(void);
uint32_t GetCrc32_Stream_Update(uint32_t crc32val,const void* data, uint16_t length);
uint32_t GetCrc32_Stream_Final(uint32_t crc32val);
#endif

