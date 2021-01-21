/*****************************************************************************
* @FileName: iap.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-11-10
* @brief
******************************************************************************/
#ifndef _IAP_H_
#define	_IAP_H_

#include <stdint.h>

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define MICRO_FLASH_BASE                  0x08000000  // FLASHÆðÊ¼µØÖ·
#define MICRO_FLASH_SECTORS               (0x06)
#define MICRO_FLASH_SIZE                  (0x40000)  // 512KB

#define MICRO_FLASH_APPLICATION_SECTORS   (MICRO_FLASH_SECTORS)
#define MICRO_FLASH_APPLICATION_MAX_SIZE  (MICRO_FLASH_SIZE) // 512KB
#define MICRO_FLASH_APPLICATION_ADDR      (0x08000000)

#define APP_START_ADDRESS                 (uint32_t)0x08000000 // Start address of application space in flash
#define MICRO_ST_BOOTLOADER_ADDRESS       (uint32_t)0x1FFF0000 // Address of System Memory (ST Bootloader)

#define FLASH_BUFFER_SIZE  1024

/******************************************************************************
 *   Data Types
 ******************************************************************************/
__align(8) typedef union
{
  uint8_t bytes[FLASH_BUFFER_SIZE];
  uint32_t words[FLASH_BUFFER_SIZE/4];
}bls_flash_buffer_t;
extern bls_flash_buffer_t bls_flash_buffer;
#define BLS_FLASH_BUFFER_SIZE  sizeof(bls_flash_buffer_t)

__align(8) typedef struct {
  uint8_t chMask[BLS_FLASH_BUFFER_SIZE];
}__bls_flash_buffer_t;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
void iap_main(void);

#endif

