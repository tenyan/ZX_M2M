/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: sfud.h
* @Engineer: armink & TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2020-7-6
* @brief:    This file is part of the Serial Flash Universal Driver Library.
******************************************************************************/
#ifndef _SFUD_H_
#define _SFUD_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 *   Macros
 ******************************************************************************/
// all defined supported command
#define SFUD_CMD_WRITE_ENABLE              0x06
#define SFUD_CMD_WRITE_DISABLE             0x04
#define SFUD_CMD_READ_STATUS_REGISTER      0x05
#define SFUD_VOLATILE_SR_WRITE_ENABLE      0x50
#define SFUD_CMD_WRITE_STATUS_REGISTER     0x01
#define SFUD_CMD_PAGE_PROGRAM              0x02
#define SFUD_CMD_AAI_WORD_PROGRAM          0xAD
#define SFUD_CMD_ERASE_CHIP                0xC7
#define SFUD_CMD_READ_DATA                   0x03
//#define SFUD_CMD_DUAL_OUTPUT_READ_DATA     0x3B
//#define SFUD_CMD_DUAL_IO_READ_DATA         0xBB
//#define SFUD_CMD_QUAD_IO_READ_DATA         0xEB
//#define SFUD_CMD_QUAD_OUTPUT_READ_DATA     0x6B
#define SFUD_CMD_MANUFACTURER_DEVICE_ID    0x90
#define SFUD_CMD_JEDEC_ID                  0x9F
#define SFUD_CMD_READ_UNIQUE_ID            0x4B
#define SFUD_CMD_READ_SFDP_REGISTER        0x5A
#define SFUD_CMD_ENABLE_RESET              0x66
#define SFUD_CMD_RESET                     0x99
#define SFUD_CMD_ENTER_4B_ADDRESS_MODE     0xB7
#define SFUD_CMD_EXIT_4B_ADDRESS_MODE      0xE9
#define SFUD_WRITE_MAX_PAGE_SIZE           256
#define SFUD_DUMMY_DATA                    0xFF

/* maximum number of erase type support on JESD216 (V1.0) */
#define SFUD_SFDP_ERASE_TYPE_MAX_NUM                      4


//_SFUD_FLASH_DEF_H_

// flash program(write) data mode
enum sfud_write_mode
{
  SFUD_WM_PAGE_256B = 1 << 0,     // write 1 to 256 bytes per page
  SFUD_WM_BYTE = 1 << 1,          // byte write
  SFUD_WM_AAI = 1 << 2,           // auto address increment
  SFUD_WM_DUAL_BUFFER = 1 << 3,   // dual-buffer write, like AT45DB series
};

/* SFUD support manufacturer JEDEC ID */
#define SFUD_MF_ID_CYPRESS                             0x01
#define SFUD_MF_ID_FUJITSU                             0x04
#define SFUD_MF_ID_EON                                 0x1C
#define SFUD_MF_ID_ATMEL                               0x1F
#define SFUD_MF_ID_MICRON                              0x20
#define SFUD_MF_ID_AMIC                                0x37
#define SFUD_MF_ID_SANYO                               0x62
#define SFUD_MF_ID_INTEL                               0x89
#define SFUD_MF_ID_ESMT                                0x8C
#define SFUD_MF_ID_FUDAN                               0xA1
#define SFUD_MF_ID_HYUNDAI                             0xAD
#define SFUD_MF_ID_SST                                 0xBF
#define SFUD_MF_ID_MICRONIX                            0xC2
#define SFUD_MF_ID_GIGADEVICE                          0xC8
#define SFUD_MF_ID_ISSI                                0xD5
#define SFUD_MF_ID_WINBOND                             0xEF

/* SFUD supported manufacturer information table */
#define SFUD_MF_TABLE                                     \
{                                                         \
    {"Cypress",    SFUD_MF_ID_CYPRESS},                   \
    {"Fujitsu",    SFUD_MF_ID_FUJITSU},                   \
    {"EON",        SFUD_MF_ID_EON},                       \
    {"Atmel",      SFUD_MF_ID_ATMEL},                     \
    {"Micron",     SFUD_MF_ID_MICRON},                    \
    {"AMIC",       SFUD_MF_ID_AMIC},                      \
    {"Sanyo",      SFUD_MF_ID_SANYO},                     \
    {"Intel",      SFUD_MF_ID_INTEL},                     \
    {"ESMT",       SFUD_MF_ID_ESMT},                      \
    {"Fudan",      SFUD_MF_ID_FUDAN},                     \
    {"Hyundai",    SFUD_MF_ID_HYUNDAI},                   \
    {"SST",        SFUD_MF_ID_SST},                       \
    {"GigaDevice", SFUD_MF_ID_GIGADEVICE},                \
    {"ISSI",       SFUD_MF_ID_ISSI},                      \
    {"Winbond",    SFUD_MF_ID_WINBOND},                   \
    {"Micronix",   SFUD_MF_ID_MICRONIX},                  \
}

#ifdef SFUD_USING_FLASH_INFO_TABLE
/* SFUD supported flash chip information table. If the flash not support JEDEC JESD216 standard,
 * then the SFUD will find the flash chip information by this table. You can add other flash to here then
 *  notice me for update it. The configuration information name and index reference the sfud_flash_chip_t structure.
 * | name | mf_id | type_id | capacity_id | capacity | write_mode | erase_gran | erase_gran_cmd |
 */
#define SFUD_FLASH_CHIP_TABLE                                                                                       \
{                                                                                                                   \
    {"AT45DB161E", SFUD_MF_ID_ATMEL, 0x26, 0x00, 2L*1024L*1024L, SFUD_WM_BYTE|SFUD_WM_DUAL_BUFFER, 512, 0x81},      \
    {"W25Q40BV", SFUD_MF_ID_WINBOND, 0x40, 0x13, 512L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                        \
    {"W25Q16BV", SFUD_MF_ID_WINBOND, 0x40, 0x15, 2L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                    \
    {"W25Q32BV", SFUD_MF_ID_WINBOND, 0x40, 0x16, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                    \
    {"W25Q64CV", SFUD_MF_ID_WINBOND, 0x40, 0x17, 8L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                    \
    {"W25Q64DW", SFUD_MF_ID_WINBOND, 0x60, 0x17, 8L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                    \
    {"W25Q128BV", SFUD_MF_ID_WINBOND, 0x40, 0x18, 16L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                  \
    {"W25Q256FV", SFUD_MF_ID_WINBOND, 0x40, 0x19, 32L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                  \
    {"SST25VF080B", SFUD_MF_ID_SST, 0x25, 0x8E, 1L*1024L*1024L, SFUD_WM_BYTE|SFUD_WM_AAI, 4096, 0x20},              \
    {"SST25VF016B", SFUD_MF_ID_SST, 0x25, 0x41, 2L*1024L*1024L, SFUD_WM_BYTE|SFUD_WM_AAI, 4096, 0x20},              \
    {"M25P32", SFUD_MF_ID_MICRON, 0x20, 0x16, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 64L*1024L, 0xD8},                  \
    {"M25P80", SFUD_MF_ID_MICRON, 0x20, 0x14, 1L*1024L*1024L, SFUD_WM_PAGE_256B, 64L*1024L, 0xD8},                  \
    {"M25P40", SFUD_MF_ID_MICRON, 0x20, 0x13, 512L*1024L, SFUD_WM_PAGE_256B, 64L*1024L, 0xD8},                      \
    {"EN25Q32B", SFUD_MF_ID_EON, 0x30, 0x16, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                        \
    {"GD25Q64B", SFUD_MF_ID_GIGADEVICE, 0x40, 0x17, 8L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                 \
    {"GD25Q16B", SFUD_MF_ID_GIGADEVICE, 0x40, 0x15, 2L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                 \
    {"S25FL216K", SFUD_MF_ID_CYPRESS, 0x40, 0x15, 2L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                   \
    {"S25FL032P", SFUD_MF_ID_CYPRESS, 0x02, 0x15, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                   \
    {"A25L080", SFUD_MF_ID_AMIC, 0x30, 0x14, 1L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20},                        \
    {"F25L004", SFUD_MF_ID_ESMT, 0x20, 0x13, 512L*1024L, SFUD_WM_BYTE|SFUD_WM_AAI, 4096, 0x20},                     \
    {"PCT25VF016B", SFUD_MF_ID_SST, 0x25, 0x41, 2L*1024L*1024L, SFUD_WM_BYTE|SFUD_WM_AAI, 4096, 0x20},              \
}
#endif /* SFUD_USING_FLASH_INFO_TABLE */


// _SFUD_DEF_H_

/**
 * retry process
 *
 * @param delay delay function for every retry. NULL will not delay for every retry.
 * @param retry retry counts
 * @param result SFUD_ERR_TIMEOUT: retry timeout
 */
#define SFUD_RETRY_PROCESS(delay, retry, result)        \
    void (*__delay_temp)(void) = (void (*)(void))delay; \
    if (retry == 0) {result = SFUD_ERR_TIMEOUT;break;}  \
    else {if (__delay_temp) {__delay_temp();} retry --;}

/**
 * status register bits
 */
enum
{
  SFUD_STATUS_REGISTER_BUSY = (1 << 0),  // busing
  SFUD_STATUS_REGISTER_WEL = (1 << 1),   // write enable latch
  SFUD_STATUS_REGISTER_SRP = (1 << 7),   // status register protect
};

/******************************************************************************
 *   Data Types
 ******************************************************************************/
// error code
typedef enum
{
  SFUD_SUCCESS = 0,               // success
  SFUD_ERR_NOT_FOUND = 1,         // not found or not supported
  SFUD_ERR_WRITE = 2,             // write error
  SFUD_ERR_READ = 3,              // read error
  SFUD_ERR_TIMEOUT = 4,           // timeout error
  SFUD_ERR_ADDR_OUT_OF_BOUND = 5, // address is out of flash bound
} sfud_status_t;

// SPI device
typedef struct __sfud_spi_t
{
  char *name; // SPI device name
  sfud_status_t (*rw)(const struct __sfud_spi_t *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,size_t read_size); // SPI bus write read data function
  void (*lock)(const struct __sfud_spi_t *spi); // lock SPI bus
  void (*unlock)(const struct __sfud_spi_t *spi); // unlock SPI bus
  void *user_data; // some user data
} sfud_spi_t;

// flash chip information
typedef struct
{
  char *name;               // flash chip name
  uint8_t mf_id;            // manufacturer ID
  uint8_t type_id;          // memory type ID
  uint8_t capacity_id;      // capacity ID
  uint32_t capacity;        // flash capacity (bytes
  uint16_t write_mode;      // write mode @see sfud_write_mode
  uint32_t erase_gran;      // erase granularity (bytes
  uint8_t erase_gran_cmd;   // erase granularity size block command
} sfud_flash_chip_t;

// serial flash device
typedef struct
{
  char *name;               // serial flash name
  size_t index;             // index of flash device information table  @see flash_table
  sfud_flash_chip_t chip;   // flash chip information
  sfud_spi_t spi;           // SPI device
  uint8_t init_ok;          // initialize OK flag
  uint8_t addr_in_4_byte;   // flash is in 4-Byte addressing
  struct
  {
    void (*delay)(void);    // every retry's delay
    size_t times;           // default times for error retry
  } retry;
  void *user_data;          // some user data
} sfud_flash_t;
extern sfud_flash_t sfud_mx25l3206e;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
sfud_status_t sfud_init(void);
sfud_status_t sfud_read(const sfud_flash_t *flash, uint32_t addr, size_t size, uint8_t *data);
sfud_status_t sfud_erase(const sfud_flash_t *flash, uint32_t addr, size_t size);
sfud_status_t sfud_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data);
sfud_status_t sfud_erase_write(const sfud_flash_t *flash, uint32_t addr, size_t size, const uint8_t *data);
sfud_status_t sfud_chip_erase(const sfud_flash_t *flash);
sfud_status_t sfud_read_status(const sfud_flash_t *flash, uint8_t *status);
sfud_status_t sfud_write_status(const sfud_flash_t *flash, bool is_volatile, uint8_t status);

#endif /* _SFUD_H_ */

