/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: iap.c
* @Engineer: TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @version   V1.0
* @Date:     2020-11-10
* @brief     ��������������
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "iap.h"
#include "sfud.h"
#include "types.h"
#include "parameters.h"
#include "stm32f2xx_flash.h"

/******************************************************************************
 *   Macros
 ******************************************************************************/
#define KR_KEY_Reload  ((uint16_t)0xAAAA)
#define CLR_IWDG()     IWDG->KR = KR_KEY_Reload

/******************************************************************************
* Data Types and Globals
******************************************************************************/
bls_flash_buffer_t bls_flash_buffer;
extern rfu_context_t rfu_context;

/* flash parameters that we should not really know */
static struct {
	uint32_t  sector_number;
	uint32_t  size;
} flash_sectors[] = {
  {0x00, 16 * 1024},  // Sector 0
  {0x01, 16 * 1024},  // Sector 1
  {0x02, 16 * 1024},  // Sector 2
  {0x03, 16 * 1024},  // Sector 3
  {0x04, 64 * 1024},  // Sector 4
  {0x05, 128 * 1024}, // Sector 5
  //{0x06, 128 * 1024}, // Sector 6
  //{0x07, 128 * 1024}, // Sector 7
  //{0x08, 128 * 1024}, // Sector 8
  //{0x09, 128 * 1024}, // Sector 9
  //{0x0a, 128 * 1024}, // Sector 10
  //{0x0b, 128 * 1024}, // Sector 11
};

/***********************************************************************************
 * FLASH������غ���
***********************************************************************************/
uint32_t flash_FuncGetSectorSize(uint16_t sector)
{
  if (sector < MICRO_FLASH_APPLICATION_SECTORS)
  {
    return flash_sectors[sector].size;;
  }

  return 0;
}

//===================================================================================
void flash_FuncWriteWord(uint32_t address, uint32_t word)
{
  FLASH_ProgramWord((address + MICRO_FLASH_APPLICATION_ADDR), word);
}

//===================================================================================
uint32_t flash_FuncReadWord(uint32_t address)
{
  return *(uint32_t *)(address + MICRO_FLASH_APPLICATION_ADDR);
}

//===================================================================================
void flash_FuncEraseSector(unsigned char sector)
{
  if (sector >= MICRO_FLASH_APPLICATION_SECTORS) {
    return;
  }

  /* Caculate the logical base address of the sector
   * flash_FuncReadWord will add MICRO_FLASH_APPLICATION_ADDR
   */
  uint32_t address;
  uint32_t size;
  uint32_t i;
  uint8_t blank = 1;

  address = 0;
  for (i=0; i<sector; i++) {
    address += flash_FuncGetSectorSize(i); // ����sector��ʼ��ַ
  }
  size = flash_FuncGetSectorSize(sector); // ����sector��С

  // blank-check the sector
  for (i=0; i<size; i += sizeof(uint32_t)) {
    if (flash_FuncReadWord(address + i) != 0xffffffff) {
      blank = false;
      break;
    }
  }

  // erase the sector if it failed the blank check
  if (!blank) {
    FLASH_EraseSector((uint32_t)(flash_sectors[sector].sector_number<<3), VoltageRange_3);  // FLASH_Sector_0
  }
}


//===================================================================================
uint8_t flash_FuncReadApp(uint32_t Address, bls_flash_buffer_t* pbuf, uint16_t size)
{
  // program bytes at current address
  uint32_t address = Address;

  if (size == 0) {
    return FALSE;
  }

  // sanity-check arguments
  if (size % 4) {
    return FALSE;
  }

  if (size > sizeof(bls_flash_buffer_t)) {
    return FALSE;
  }

  size /= 4;

  for (int i = 0; i < size; i++)
  {
    pbuf->words[i] = flash_FuncReadWord(address);  // read the word
    address += 4;
  }

  return TRUE;
}

//===================================================================================
uint8_t flash_FuncProgramApp(uint32_t Address, bls_flash_buffer_t* pbuf, uint16_t size)
{
  // program bytes at current address
  uint32_t address = Address;

  if (size == 0) {
    return FALSE;
  }

  // sanity-check arguments
  if (size % 4) {
    return FALSE;
  }

  if (size > sizeof(bls_flash_buffer_t)) {
    return FALSE;
  }

  size /= 4;

  for (uint16_t i = 0; i < size; i++)
  {
    // program the word
    flash_FuncWriteWord(address, pbuf->words[i]);

    // do immediate read-back verify
    if (flash_FuncReadWord(address) != pbuf->words[i])
    {
      return FALSE;;
    }

    address += 4;
  }

  return TRUE;
}

//===================================================================================
uint8_t flash_FuncEraseApp(void)
{
  //erase and prepare for programming
  uint32_t address;
  uint8_t flash_rc = TRUE;

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);

  // erase all sectors
  for (uint16_t i = 0; flash_FuncGetSectorSize(i)!=0; i++)
  {
    // ι���Ź�
    CLR_IWDG();
    //WDOG_LED_OFF();
    flash_FuncEraseSector(i);
    //WDOG_LED_ON();
  }

  // verify the erase
  for (address = 0; address < MICRO_FLASH_APPLICATION_MAX_SIZE; address += 4)
  {
    if (flash_FuncReadWord(address) != 0xFFFFFFFF)
    {
      flash_rc = FALSE;
      break;
    }
  }

  return flash_rc;
}

//======================================================================================
void iap_ReadFlashHexFile(uint32_t address, uint8_t *buf, uint32_t cnt)
{
  sfud_mx25l3206e.spi.lock = NULL; // ����IAPģʽ,���ɵȴ��ź���,��Ϊ������RAMִ��
  sfud_mx25l3206e.spi.unlock = NULL;
  sfud_read(&sfud_mx25l3206e, (EMAP_HEX_FILE_ADDRESS + address), cnt, buf);
}

/******************************************************************************************
 * ����Ѿ�����IAP,����Ҫ�ȴ��ź���
******************************************************************************************/
void iap_main(void)
{
  uint16_t bufferSize = FLASH_BUFFER_SIZE;
  uint8_t flash_rc;
  
  __disable_irq();      // �ر������ж�

  CLR_IWDG();  // ������Ź�

  FLASH_Unlock();
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);

  //==rfu_ProgramNewFirmware()============================================================
  rfu_context.block = 0;
  rfu_context.cumulated_address = 0;
  rfu_context.ending_address = rfu_context.file_length;
  rfu_context.total_block_count = (rfu_context.file_length / bufferSize) + 1;

  // �����ڲ�FLASH�̼�
  flash_rc = flash_FuncEraseApp();
  if (flash_rc == FALSE)
  {
    return;
  }

  while (rfu_context.cumulated_address < rfu_context.ending_address)
  {
    if (rfu_context.cumulated_address + bufferSize > rfu_context.ending_address)
    {
      bufferSize = rfu_context.ending_address - rfu_context.cumulated_address;
    }

    CLR_IWDG();  // ι���Ź�
    //WDOG_LED_OFF();  // ι���Ź�
    iap_ReadFlashHexFile(rfu_context.cumulated_address,bls_flash_buffer.bytes,bufferSize); // ��ȡOTA����
    //WDOG_LED_ON(); // ι���Ź�

    flash_rc = flash_FuncProgramApp(rfu_context.cumulated_address,&bls_flash_buffer,bufferSize); // д������
    if (flash_rc == FALSE)
    {
      return;
    }

    rfu_context.cumulated_address += bufferSize;
    rfu_context.block++;
    rfu_context.percent = rfu_context.block * 100L / rfu_context.total_block_count;
  }
  //=======================================================================================

  /* just for paranoia's sake */
  FLASH_Lock();
  NVIC_SystemReset(); // ��λ
  
  while (1)//�����������ϵͳ
  {

  }
}

//-----�ļ�iap.c����---------------------------------------------
