/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: fal.c
* @Engineer: armink & TenYan
* @Company:  �칤��Ϣ����Ӳ����
* @Date:     2021-1-26
* @brief     This file is part of FAL (Flash Abstraction Layer) package.
* Flash������ʵ��,�������Flash�豸��Flash����
******************************************************************************/
#include "fal.h"
#include "sfud.h"
#include "PcDebug.h"
#include "parameters.h"

/******************************************************************************
 * Macros
 ******************************************************************************/
//==partition magic word======
#define FAL_PART_MAGIC_WORD  0x45503130
#define FAL_PART_MAGIC_WORD_H  0x4550L
#define FAL_PART_MAGIC_WORD_L  0x3130L
#define FAL_PART_MAGIC_WROD  0x45503130

/******************************************************************************
 * FAL��ֲ:����flash�豸
 ******************************************************************************/
#if (PART("�û���ֲʵ��"))
static sfud_flash_t* sfud_dev = NULL;

static int init(void);
static int read(uint32_t offset, uint8_t *buf, size_t size);
static int write(uint32_t offset, const uint8_t *buf, size_t size);
static int erase(uint32_t offset, size_t size);

// Flash�豸����
#define FAL_USING_NOR_FLASH_DEV_NAME  "norflash0"
fal_flash_dev_t nor_flash0 =
{
  .name = FAL_USING_NOR_FLASH_DEV_NAME,  // Flash�豸������
  .addr = 0,  // ��Flash��������ʼ��ַ
  .len = 4 * 1024 * 1024,  // Flash���ܴ�С(4MB)
  .blk_size = 4096,  // Flash ��/������С
  .ops = {init, read, write, erase},  // Flash �Ĳ�������
  .write_gran = 1  // ����д����(��λbit)
};
#endif

/******************************************************************************
 * FAL��ֲ:����flash�豸��Ͷ���flash������
 ******************************************************************************/
#if (PART("�û���ֲʵ��"))
//==Flash device Configuration ========================
#define FAL_FLASH_DEV_TABLE  { &nor_flash0, }  // Flash�豸��

//== Partition Configuration ==========================
// �û���Ҫ�޸ĵķ�����������:�������ơ�������Flash�豸����ƫ�Ƶ�ַ(���Flash�豸�ڲ�)����С
// 1-��������֤�����ظ�
// 2-������Flash�豸��������Flash�豸���ж����,��������һ��,���������޷��ҵ�Flash�豸�Ĵ���
// 3-��������ʼ��ַ�ʹ�С���ܳ���Flash�豸�ĵ�ַ��Χ,����ᵼ�°���ʼ������
// 4-ÿ����������ʱ,������д������ܵĲ���������,����ǰ������FAL_PART_MAGIC_WORD����,ĩβ����0(Ŀǰ���ڱ�������)
#define FAL_PART_TABLE  \
{ \
    {FAL_PART_MAGIC_WROD,  "fdb_kvdb1",  "norflash0",   EMAP_KVDB_ADDRESS, EMAP_KVDB_SIZE, 0}, \
}  // flash ������

    //{FAL_PART_MAGIC_WROD,  "fdb_tsdb1",  "norflash0",   1024*1024, 1024*1024, 0},
#endif

/******************************************************************************
 * Data Types
 ******************************************************************************/
static uint8_t flash_dev_init_ok = 0;
static uint8_t flash_partition_init_ok = 0;
static uint8_t fal_init_ok = 0;

static const fal_flash_dev_t* const device_table[] = FAL_FLASH_DEV_TABLE;
static const uint16_t device_table_len = sizeof(device_table) / sizeof(device_table[0]);

static const fal_partition_t partition_table_def[] = FAL_PART_TABLE;
static fal_partition_t* partition_table = NULL;
static uint16_t partition_table_len = 0;

/******************************************************************************
 * FAL��ֲ:�ײ�ӿں���
 ******************************************************************************/
#if (PART("�û���ֲʵ��"))
//=====================================================================================
static int init(void)
{
  // bare metal platform
  extern sfud_flash_t sfud_mx25l3206e;
  sfud_dev = &sfud_mx25l3206e;

  if (NULL == sfud_dev)
  {
    return -1;
  }

  // update the flash chip information
  nor_flash0.blk_size = sfud_dev->chip.erase_gran;
  nor_flash0.len = sfud_dev->chip.capacity;

  return 0;
}

//=====================================================================================
static int read(uint32_t offset, uint8_t *buf, size_t size)
{
  FAL_ASSERT(sfud_dev);
  FAL_ASSERT(sfud_dev->init_ok);
  sfud_read(sfud_dev, nor_flash0.addr + offset, size, buf);

  return size; // ����ʵ�ʶ�ȡ�����ݴ�С
}

//=====================================================================================
static int write(uint32_t offset, const uint8_t *buf, size_t size)
{
  FAL_ASSERT(sfud_dev);
  FAL_ASSERT(sfud_dev->init_ok);
  if (sfud_write(sfud_dev, nor_flash0.addr + offset, size, buf) != SFUD_SUCCESS)
  {
    return -1;
  }

  return size; // ����ʵ��д������ݴ�С
}

//=====================================================================================
static int erase(uint32_t offset, size_t size)
{
  FAL_ASSERT(sfud_dev);
  FAL_ASSERT(sfud_dev->init_ok);
  if (sfud_erase(sfud_dev, nor_flash0.addr + offset, size) != SFUD_SUCCESS)
  {
    return -1;
  }

  return size; // ����ʵ�ʲ����������С
}
#endif

/***************************************************************************
 * print the partition table
 **************************************************************************/
void FAL_ShowPartTalbe(void)
{
  char *item1 = "name", *item2 = "flash_dev";
  size_t i, part_name_max = strlen(item1), flash_dev_name_max = strlen(item2);
  const fal_partition_t *part;

  if (partition_table_len)
  {
    for (i = 0; i < partition_table_len; i++)
    {
      part = &partition_table[i];
      if (strlen(part->name) > part_name_max)
      {
        part_name_max = strlen(part->name);
      }
      if (strlen(part->flash_name) > flash_dev_name_max)
      {
        flash_dev_name_max = strlen(part->flash_name);
      }
    }
  }
  FAL_LOG_I("==================== FAL partition table ====================");
  FAL_LOG_I("| %-*.*s | %-*.*s |   offset   |    length  |", part_name_max, FAL_DEV_NAME_MAX, item1, flash_dev_name_max, FAL_DEV_NAME_MAX, item2);
  FAL_LOG_I("-------------------------------------------------------------");
  for (i = 0; i < partition_table_len; i++)
  {
    part = &partition_table[i];
    FAL_LOG_I("| %-*.*s | %-*.*s | 0x%08lx | 0x%08x |", part_name_max, FAL_DEV_NAME_MAX, part->name, flash_dev_name_max, FAL_DEV_NAME_MAX, part->flash_name, part->offset, part->len);
  }
  FAL_LOG_I("=============================================================");
}

/***************************************************************************
 * Initialize all flash partition on FAL partition table
 * @return partitions total number
 **************************************************************************/
int iFAL_PartitionInit(void)
{
  size_t i;
  const fal_flash_dev_t *flash_dev = NULL;

  if (flash_partition_init_ok)
  {
    return partition_table_len;
  }

  partition_table = (fal_partition_t*) &partition_table_def[0];
  partition_table_len = sizeof(partition_table_def) / sizeof(partition_table_def[0]);
  for (i = 0; i < partition_table_len; i++) // check the partition table device exists
  {
    flash_dev = FAL_FlashDeviceFind(partition_table[i].flash_name);
    if (flash_dev == NULL)
    {
      FAL_LOG_D("Warning: Do NOT found the flash device(%s).", partition_table[i].flash_name);
      continue;
    }

    if (partition_table[i].offset >= (long)flash_dev->len)
    {
      FAL_LOG_E("Initialize failed! Partition(%s) offset address(%ld) out of flash bound(<%d).", partition_table[i].name, partition_table[i].offset, flash_dev->len);
      partition_table_len = 0;
      goto _exit;
    }
  }

  flash_partition_init_ok = 1;

_exit:

#if FAL_DEBUG
  FAL_ShowPartTalbe();
#endif

  return partition_table_len;
}

/***************************************************************************
 * find the partition by name
 * @param name partition name
 * @return != NULL: partition, NULL: not found
 **************************************************************************/
const fal_partition_t* FAL_PartitionFind(const char *name)
{
  FAL_ASSERT(flash_partition_init_ok);
  size_t i;

  for (i = 0; i < partition_table_len; i++)
  {
    if (!strcmp(name, partition_table[i].name))
    {
      return &partition_table[i];
    }
  }

  return NULL;
}

#if 0
/***************************************************************************
 * get the partition table
 * @param len return the partition table length
 * @return partition table
 **************************************************************************/
const fal_partition_t* FAL_GetPartitionTable(size_t *len)
{
  FAL_ASSERT(flash_partition_init_ok);
  FAL_ASSERT(len);

  *len = partition_table_len;

  return partition_table;
}

/***************************************************************************
 * set partition table temporarily
 * This setting will modify the partition table temporarily, the setting will be lost after restart.
 * @param table partition table
 * @param len partition table length
 **************************************************************************/
void FAL_SetPartitionTableTemp(fal_partition_t* table, size_t len)
{
  FAL_ASSERT(flash_partition_init_ok);
  FAL_ASSERT(table);

  partition_table_len = len;
  partition_table = table;
}
#endif

/***************************************************************************
 * read data from partition
 * @param part partition
 * @param addr relative address for partition
 * @param buf read buffer
 * @param size read size
 * @return >= 0: successful read data size, -1: error
 **************************************************************************/
int FAL_PartitionRead(const fal_partition_t *part, uint32_t addr, uint8_t *buf, size_t size)
{
  int ret = 0;
  const fal_flash_dev_t *flash_dev = NULL;

  FAL_ASSERT(part);
  FAL_ASSERT(buf);

  if (addr + size > part->len)
  {
    FAL_LOG_E("FalPartReadErr! Partition address out of bound.");
    return -1;
  }

  flash_dev = FAL_FlashDeviceFind(part->flash_name);
  if (flash_dev == NULL)
  {
    FAL_LOG_E("FalPartReadErr! Don't found flash device(%s) of the partition(%s).", part->flash_name, part->name);
    return -1;
  }

  ret = flash_dev->ops.read(part->offset + addr, buf, size);
  if (ret < 0)
  {
    FAL_LOG_E("FalPartReadErr! Flash device(%s) read error!", part->flash_name);
  }

  return ret;
}

/***************************************************************************
 * write data to partition
 * @param part partition
 * @param addr relative address for partition
 * @param buf write buffer
 * @param size write size
 * @return >= 0: successful write data size, -1: error
 **************************************************************************/
int FAL_PartitionWrite(const fal_partition_t *part, uint32_t addr, const uint8_t *buf, size_t size)
{
  int ret = 0;
  const fal_flash_dev_t *flash_dev = NULL;

  FAL_ASSERT(part);
  FAL_ASSERT(buf);

  if (addr + size > part->len)
  {
    FAL_LOG_E("FalPartWriteErr! Partition address out of bound.");
    return -1;
  }

  flash_dev = FAL_FlashDeviceFind(part->flash_name);
  if (flash_dev == NULL)
  {
    FAL_LOG_E("FalPartWriteErr!  Don't found flash device(%s) of the partition(%s).", part->flash_name, part->name);
    return -1;
  }

  ret = flash_dev->ops.write(part->offset + addr, buf, size);
  if (ret < 0)
  {
    FAL_LOG_E("FalPartWriteErr! Flash device(%s) write error!", part->flash_name);
  }

  return ret;
}

/***************************************************************************
 * erase partition data
 * @param part partition
 * @param addr relative address for partition
 * @param size erase size
 * @return >= 0: successful erased data size, -1: error
 **************************************************************************/
int FAL_PartitionErase(const fal_partition_t *part, uint32_t addr, size_t size)
{
  int ret = 0;
  const fal_flash_dev_t *flash_dev = NULL;

  FAL_ASSERT(part);

  if (addr + size > part->len)
  {
    FAL_LOG_E("FalPartEraseErr! Partition address out of bound.");
    return -1;
  }

  flash_dev = FAL_FlashDeviceFind(part->flash_name);
  if (flash_dev == NULL)
  {
    FAL_LOG_E("FalPartEraseErr! Don't found flash device(%s) of the partition(%s).", part->flash_name, part->name);
    return -1;
  }

  ret = flash_dev->ops.erase(part->offset + addr, size);
  if (ret < 0)
  {
    FAL_LOG_E("FalPartEraseErr! Flash device(%s) erase error!", part->flash_name);
  }

  return ret;
}

/***************************************************************************
 * erase partition all data
 * @param part partition
 * @return >= 0: successful erased data size, -1: error
 **************************************************************************/
int FAL_PartitionEraseAll(const fal_partition_t *part)
{
  return FAL_PartitionErase(part, 0, part->len);
}

/***************************************************************************
 * Initialize all flash device on FAL flash table
 * @return result
 **************************************************************************/
int iFAL_FlashInit(void)
{
  size_t i;
  const fal_flash_dev_t *dev;

  if (flash_dev_init_ok)
  {
    return 0;
  }

  for (i = 0; i < device_table_len; i++)
  {
    dev = device_table[i];
    FAL_ASSERT(device_table[i]->ops.read);
    FAL_ASSERT(device_table[i]->ops.write);
    FAL_ASSERT(device_table[i]->ops.erase);
    if (device_table[i]->ops.init)
    {
      device_table[i]->ops.init(); // init flash device on flash table
    }
    FAL_LOG_D("Flash device | %*.*s | addr: 0x%08lx | len: 0x%08x | blk_size: 0x%08x |initialized finish.",FAL_DEV_NAME_MAX, FAL_DEV_NAME_MAX, dev->name, dev->addr, dev->len, dev->blk_size);
  }
  
  UNUSED_VARIABLE(dev);
  flash_dev_init_ok = 1;
  return 0;
}

/***************************************************************************
 * find flash device by name
 * @param name flash device name
 * @return != NULL: flash device, NULL: not found
 **************************************************************************/
const fal_flash_dev_t* FAL_FlashDeviceFind(const char *name)
{
  FAL_ASSERT(flash_dev_init_ok);
  FAL_ASSERT(name);

  size_t i;

  for (i = 0; i < device_table_len; i++)
  {
    if (!strncmp(name, device_table[i]->name, FAL_DEV_NAME_MAX))
    {
      return device_table[i];
    }
  }

  return NULL;
}

/***************************************************************************
 * FAL (Flash Abstraction Layer) initialization.
 * It will initialize all flash device and all flash partition.
 * @return >= 0: partitions total number
 **************************************************************************/
int FAL_Init(void)
{
  int result;

  result = iFAL_FlashInit(); // initialize all flash device on FAL flash table
  if (result < 0)
  {
    goto __exit;
  }

  result = iFAL_PartitionInit(); // initialize all flash partition on FAL partition table

__exit:

  if ((result > 0) && (!fal_init_ok))
  {
    fal_init_ok = 1;
    FAL_LOG("FAL Init:OK.\n");
  }
  else if (result <= 0)
  {
    fal_init_ok = 0;
    FAL_LOG("FAL Init:NOK.\n");
  }

  return result;
}

/***************************************************************************
 * Check if the FAL is initialized successfully
 * @return 0: not init or init failed; 1: init success
 **************************************************************************/
int FAL_InitCheck(void)
{
  return fal_init_ok;
}

