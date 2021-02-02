/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName:  md5.h
* @Engineer:  TenYan
* @Company:  徐工信息智能硬件部
* @Date:      2020-07-09
* @brief:     MD5 (Message-Digest Algorithm)
******************************************************************************/
#ifndef _MD5_H_
#define _MD5_H_

// Dependencies
#include <stdint.h>
#include <stdio.h>

/******************************************************************************
 * Macros
 ******************************************************************************/
#define MD5_BLOCK_SIZE     64  // MD5 block size
#define MD5_DIGEST_SIZE    16  // MD5 digest size
#define MD5_MIN_PAD_SIZE   9   // Minimum length of the padding string

// Rotate left operation
#define ROL8(a, n)  (((a) << (n)) | ((a) >> (8 - (n))))
#define ROL16(a, n) (((a) << (n)) | ((a) >> (16 - (n))))
#define ROL32(a, n) (((a) << (n)) | ((a) >> (32 - (n))))
#define ROL64(a, n) (((a) << (n)) | ((a) >> (64 - (n))))

// Rotate right operation
#define ROR8(a, n)  (((a) >> (n)) | ((a) << (8 - (n))))
#define ROR16(a, n) (((a) >> (n)) | ((a) << (16 - (n))))
#define ROR32(a, n) (((a) >> (n)) | ((a) << (32 - (n))))
#define ROR64(a, n) (((a) >> (n)) | ((a) << (64 - (n))))

// Shift left operation
#define SHL8(a, n)  ((a) << (n))
#define SHL16(a, n) ((a) << (n))
#define SHL32(a, n) ((a) << (n))
#define SHL64(a, n) ((a) << (n))

// Shift right operation
#define SHR8(a, n)  ((a) >> (n))
#define SHR16(a, n) ((a) >> (n))
#define SHR32(a, n) ((a) >> (n))
#define SHR64(a, n) ((a) >> (n))

// Micellaneous macros
#define _U8(x)  ((uint8_t)  (x))
#define _U16(x) ((uint16_t) (x))
#define _U32(x) ((uint32_t) (x))
#define _U64(x) ((uint64_t) (x))

// Swap a 32-bit integer
#define SWAPINT32(x) ( \
   (((uint32_t)(x) & 0x000000FFUL) << 24) | \
   (((uint32_t)(x) & 0x0000FF00UL) << 8) | \
   (((uint32_t)(x) & 0x00FF0000UL) >> 8) | \
   (((uint32_t)(x) & 0xFF000000UL) >> 24))

// 大小端
#ifdef _CPU_BIG_ENDIAN //Big-endian machine?
  #define htole32(value) SWAPINT32((uint32_t) (value)) // Host byte order to little-endian byte order
  #define letoh32(value) SWAPINT32((uint32_t) (value)) // Little-endian byte order to host byte order
#else // Little-endian machine?
  #define htole32(value) ((uint32_t) (value))
  #define letoh32(value) ((uint32_t) (value))
#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

/******************************************************************************
 *   Data Types
 ******************************************************************************/
 // MD5 algorithm context
#pragma anon_unions 
typedef struct
{
  union
  {
    uint32_t h[4];
    uint8_t digest[16];
  };
  union
  {
    uint32_t x[16];
    uint8_t buffer[64];
  };
  size_t size;
  uint64_t totalSize;
} md5_context_t;
extern md5_context_t md5_context;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void md5_Compute(const void *data, size_t length, uint8_t *digest);
void md5_Init(md5_context_t *pThis);
void md5_Update(md5_context_t *pThis, const void *data, size_t length);
void md5_Final(md5_context_t *pThis, uint8_t *digest);
void md5_FinalRaw(md5_context_t *pThis, uint8_t *digest);

#endif
