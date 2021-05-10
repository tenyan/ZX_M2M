/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: FlashDB.c
* @Engineer: armink & TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2021-1-26
* @brief:    This file is part of the Flash Database Library.
******************************************************************************/
/*****************************************************************************
* 设计说明
*
* FAL底层将不同的Flash存储介质进行了统一封装,并提供了分区表机制,暴露给上层用户。
* FlashDB的每个数据库就是基于FAL提供的分区机制,每个数据库都坐落在某个FAL的分区上,
* 相当于一个分区对应一个数据库。
*
* 扇区大小与块大小
* FlashDB内部存储结构由N个扇区组成,每次格式化时是以扇区作为最小单位。
* 而一个扇区通常是Flash块大小的N倍,比如:NorFlash的块大小一般为 4096。
* 默认KVDB会使用1倍的块大小作为扇区大小,即:4096。此时,该KVDB无法存入超过4096长度的KV 。
* 如果想要存入比如:10K长度的KV,可以通过control函数,设置扇区大小为12K,或者更大大小即可。
*
* 主要特性
* 1-资源占用极低,内存占用几乎为0;
* 2-支持多分区,多实例。数据量大时,可细化分区,降低检索时间;
* 3-支持磨损平衡,延长Flash寿命;
* 4-支持掉电保护功能,可靠性高;
* 5-支持字符串及blob两种KV类型,方便用户操作;
* 6-支持KV增量升级,产品固件升级后,KVDB内容也支持自动升级；
* 7-支持修改每条TSDB记录的状态,方便用户进行管理;
*
* FlashDB提供两种数据库模式:
* 1-键值数据库KVDB:是一种非关系数据库,它将数据存储为键值(Key-Value)对集合,其中键作为唯一标识符。
*   KVDB操作简洁,可扩展性强。
* 2-时序数据库TSDB:时间序列数据库(Time Series Database),它将数据按照时间顺序存储。
*   TSDB数据具有时间戳,数据存储量大,插入及查询性能高。
******************************************************************************/
#include <string.h>
#include "cmsis_os2.h"

#include <inttypes.h>
#include <string.h>
#include <FlashDB.h>
#include "PcDebug.h"

/******************************************************************************
 * FlashDB Low Level
 ******************************************************************************/
#if (FDB_WRITE_GRAN == 1)
#define FDB_STATUS_TABLE_SIZE(status_number)  ((status_number * FDB_WRITE_GRAN + 7)/8)
#else
#define FDB_STATUS_TABLE_SIZE(status_number)  (((status_number - 1) * FDB_WRITE_GRAN + 7)/8)
#endif

/// Return the most contiguous size aligned at specified width. RT_ALIGN(13, 4) would return 16. 
#define FDB_ALIGN(size, align)  (((size) + (align) - 1) & ~((align) - 1))
#define FDB_WG_ALIGN(size)  (FDB_ALIGN(size, (FDB_WRITE_GRAN + 7)/8))  // align by write granularity

/// Return the down number of aligned at specified width. RT_ALIGN_DOWN(13, 4) would return 12.
#define FDB_ALIGN_DOWN(size, align)  ((size) & ~((align) - 1))
#define FDB_WG_ALIGN_DOWN(size)  (FDB_ALIGN_DOWN(size, (FDB_WRITE_GRAN + 7)/8))  // align down by write granularity

#define FDB_STORE_STATUS_TABLE_SIZE  FDB_STATUS_TABLE_SIZE(FDB_SECTOR_STORE_STATUS_NUM)
#define FDB_DIRTY_STATUS_TABLE_SIZE  FDB_STATUS_TABLE_SIZE(FDB_SECTOR_DIRTY_STATUS_NUM)

#define FDB_DATA_UNUSED  0xFFFFFFFF  // the data is unused

#ifdef FDB_USING_FILE_MODE
extern fdb_err_t _fdb_file_read(fdb_db_t* db, uint32_t addr, void *buf, size_t size);
extern fdb_err_t _fdb_file_write(fdb_db_t* db, uint32_t addr, const void *buf, size_t size);
extern fdb_err_t _fdb_file_erase(fdb_db_t* db, uint32_t addr, size_t size);
#endif

#if !defined(FDB_USING_FAL_MODE) && !defined(FDB_USING_FILE_MODE)
#error "Please defined the FDB_USING_FAL_MODE or FDB_USING_FILE_MODE macro"
#endif

#define FDB_NAME(db)  (((fdb_db_t*)db)->name)
#define FDB_INIT_OK(db)  (((fdb_db_t*)db)->init_ok)
#define FDB_SECTOR_SIZE(db)  (((fdb_db_t*)db)->sec_size)
#define FDB_MAX_SIZE(db)  (((fdb_db_t*)db)->max_size)
#define FDB_LOCK(db)  do {  if (((fdb_db_t*)db)->lock) ((fdb_db_t*)db)->lock((fdb_db_t*)db);} while(0);
#define FDB_UNLOCK(db)  do {   if (((fdb_db_t*)db)->unlock) ((fdb_db_t*)db)->unlock((fdb_db_t*)db);} while(0);

#define FAILED_ADDR  0xFFFFFFFF  // the next address is get failed

/******************************************************************************
 * KVDB Macros
 ******************************************************************************/
#if defined(FDB_USING_KVDB)

#ifndef FDB_WRITE_GRAN
#error "Please configure flash write granularity (in fdb_cfg.h)"
#endif

#if FDB_WRITE_GRAN != 1 && FDB_WRITE_GRAN != 8 && FDB_WRITE_GRAN != 32
#error "the write gran can be only setting as 1, 8 and 32"
#endif

#define KV_SECTOR_MAGIC_WORD  0x30424446  // magic word(`F`, `D`, `B`, `1`)
#define KV_MAGIC_WORD  0x3030564B  // magic word(`K`, `V`, `0`, `0`) 

/// the sector remain threshold before full status
#ifndef FDB_SEC_REMAIN_THRESHOLD
#define FDB_SEC_REMAIN_THRESHOLD  (KV_HDR_DATA_SIZE + FDB_KV_NAME_MAX)
#endif

/// the total remain empty sector threshold before GC
#ifndef FDB_GC_EMPTY_SEC_THRESHOLD
#define FDB_GC_EMPTY_SEC_THRESHOLD  1
#endif

/// the string KV value buffer size for legacy fdb_get_kv(db, ) function
#ifndef FDB_STR_KV_VALUE_MAX_SIZE
#define FDB_STR_KV_VALUE_MAX_SIZE  128
#endif

#if FDB_KV_CACHE_TABLE_SIZE > 0xFFFF
#error "The KV cache table size must less than 0xFFFF"
#endif

#define KV_SECTOR_NOT_COMBINED  0xFFFFFFFF  // the sector is not combined value
#define KV_STATUS_TABLE_SIZE  FDB_STATUS_TABLE_SIZE(FDB_KV_STATUS_NUM)
#define KV_SECTOR_HDR_DATA_SIZE  (FDB_WG_ALIGN(sizeof(kv_sector_hdr_data_t)))
#define KV_SECTOR_DIRTY_OFFSET  ((unsigned long)(&((kv_sector_hdr_data_t *)0)->status_table.dirty))
#define KV_HDR_DATA_SIZE  (FDB_WG_ALIGN(sizeof(kv_hdr_data_t)))
#define KV_MAGIC_OFFSET  ((unsigned long)(&((kv_hdr_data_t *)0)->magic))
#define KV_LEN_OFFSET  ((unsigned long)(&((kv_hdr_data_t *)0)->len))
#define KV_NAME_LEN_OFFSET  ((unsigned long)(&((kv_hdr_data_t *)0)->name_len))

#define VER_NUM_KV_NAME  "__ver_num__"
#define SECTOR_NUM  (FDB_MAX_SIZE(db) / FDB_SECTOR_SIZE(db))

#endif

/******************************************************************************
 * TSDB: Macros
 ******************************************************************************/
#define FDB_LOG_TAG "[tsl]"
/* rewrite log prefix */
#undef  FDB_LOG_PREFIX2
#define FDB_LOG_PREFIX2()  FDB_PRINT("[%s] ", FDB_NAME(db))

#if defined(FDB_USING_TSDB)

#define TS_SECTOR_MAGIC_WORD  0x304C5354 // magic word(`T`, `S`, `L`, `0`) 
#define TSL_STATUS_TABLE_SIZE  FDB_STATUS_TABLE_SIZE(FDB_TSL_STATUS_NUM)

#define TS_SECTOR_HDR_DATA_SIZE  (FDB_WG_ALIGN(sizeof(struct ts_sector_hdr_data)))
#define TS_LOG_IDX_DATA_SIZE  (FDB_WG_ALIGN(sizeof(struct ts_log_idx_data)))
#define TS_LOG_IDX_OFFSET  ((unsigned long)(&((struct ts_log_idx_data *)0)->time))
#define TS_SECTOR_MAGIC_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->magic))
#define TS_SECTOR_START_TIME_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->start_time))
#define TS_SECTOR_END0_TIME_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[0].time))
#define TS_SECTOR_END0_IDX_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[0].index))
#define TS_SECTOR_END0_STATUS_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[0].status))
#define TS_SECTOR_END1_TIME_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[1].time))
#define TS_SECTOR_END1_IDX_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[1].index))
#define TS_SECTOR_END1_STATUS_OFFSET  ((unsigned long)(&((struct ts_sector_hdr_data *)0)->end_info[1].status))

#define TSDB_WRITE_STATUS(db, addr, status_table, status_num, status_index)\
        do {\
        result = iFDB_WriteStatus((fdb_db_t*)db, addr, status_table, status_num, status_index);\
        if (result != FDB_NO_ERR) {return result;}\
    } while(0);

#define TSDB_FLASH_WRITE(db, addr, buf, size)\
    do {\
        result = iFDB_FlashWrite((fdb_db_t*)db, addr, buf, size);\
        if (result != FDB_NO_ERR) {return result;}\
    } while(0);

#endif

/******************************************************************************
* KVDB: Data Types and Globals
******************************************************************************/
#if defined(FDB_USING_KVDB)

typedef struct
{
  struct
  {
    uint8_t store[FDB_STORE_STATUS_TABLE_SIZE]; //< sector store status @see fdb_sector_store_status_t
    uint8_t dirty[FDB_DIRTY_STATUS_TABLE_SIZE]; //< sector dirty status @see fdb_sector_dirty_status_t
  } status_table;
  uint32_t magic; //< magic word(`E`, `F`, `4`, `0`)
  uint32_t combined; //< the combined next sector number, 0xFFFFFFFF: not combined
  uint32_t reserved;
}kv_sector_hdr_data_t;

typedef struct
{
  uint8_t status_table[KV_STATUS_TABLE_SIZE]; //< KV node status, @see fdb_kv_status_t
  uint32_t magic; //< magic word(`K`, `V`, `4`, `0`)
  uint32_t len; //< KV node total length (header + name + value), must align by FDB_WRITE_GRAN
  uint32_t crc32; //< KV node crc32(name_len + data_len + name + value)
  uint8_t name_len; //< name length
  uint32_t value_len; //< value length
}kv_hdr_data_t;

typedef struct alloc_kv_cb_args
{
  fdb_kvdb_t* db;
  size_t kv_size;
  uint32_t *empty_kv;
}alloc_kv_cb_args_t;

static void iKVDB_GcCollect(fdb_kvdb_t* db);

#endif

/******************************************************************************
* TSDB: Data Types and Globals
******************************************************************************/
#if defined(FDB_USING_TSDB)

typedef struct ts_sector_hdr_data
{
  uint8_t status[FDB_STORE_STATUS_TABLE_SIZE];  //< sector store status @see fdb_sector_store_status_t
  uint32_t magic;  //< magic word(`T`, `S`, `L`, `0`)
  fdb_time_t start_time;  //< the first start node's timestamp
  struct
  {
    fdb_time_t time;  //< the last end node's timestamp
    uint32_t index;  //< the last end node's index
    uint8_t status[TSL_STATUS_TABLE_SIZE];  //< end node status, @see fdb_tsl_status_t
  } end_info[2];
  uint32_t reserved;
}ts_sector_hdr_data_t;

// time series log node index data
typedef struct ts_log_idx_data
{
  uint8_t status_table[TSL_STATUS_TABLE_SIZE];  //< node status, @see fdb_tsl_status_t
  fdb_time_t time;  //< node timestamp
  uint32_t log_len;  //< node total length (header + name + value), must align by FDB_WRITE_GRAN
  uint32_t log_addr;  //< node address
}ts_log_idx_data_t;

struct ts_query_count_args
{
  fdb_tsl_status_t status;
  size_t count;
};

struct ts_check_sec_hdr_cb_args
{
  fdb_tsdb_t* db;
  bool check_failed;
  size_t empty_num;
  uint32_t empty_addr;
};

#endif

/******************************************************************************
* Crc32 Table
******************************************************************************/
static const uint32_t crc32_table[] =
{
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

/******************************************************************************
* Some utils for this library
******************************************************************************/
//====================================================================================
static bool iFDB_IsString(uint8_t *value, size_t len)
{
  #define __is_print(ch) ((unsigned int)((ch) - ' ') < 127u - ' ')
  size_t i;

  for (i = 0; i < len; i++)
  {
    if (!__is_print(value[i]))
    {
      return false;
    }
  }
  return true;
}

//====================================================================================
size_t iFDB_SetStatus(uint8_t status_table[], size_t status_num, size_t status_index)
{
  size_t byte_index = ~0UL;
  /*
   * | write garn |       status0       |       status1       |      status2         |
   * ---------------------------------------------------------------------------------
   * |    1bit    | 0xFF                | 0x7F                |  0x3F                |
   * |    8bit    | 0xFFFF              | 0x00FF              |  0x0000              |
   * |   32bit    | 0xFFFFFFFF FFFFFFFF | 0x00FFFFFF FFFFFFFF |  0x00FFFFFF 00FFFFFF |
   */
  memset(status_table, 0xFF, FDB_STATUS_TABLE_SIZE(status_num));
  if (status_index > 0)
  {
#if (FDB_WRITE_GRAN == 1)
    byte_index = (status_index - 1) / 8;
    status_table[byte_index] &= ~(0x80 >> ((status_index - 1) % 8));
#else
    byte_index = (status_index - 1) * (FDB_WRITE_GRAN / 8);
    status_table[byte_index] = 0x00;
#endif
  }

  return byte_index;
}

//====================================================================================
size_t iFDB_GetStatus(uint8_t status_table[], size_t status_num)
{
  size_t i = 0, status_num_bak = --status_num;

  while (status_num --)
  {
    // get the first 0 position from end address to start address
#if (FDB_WRITE_GRAN == 1)
    if ((status_table[status_num / 8] & (0x80 >> (status_num % 8))) == 0x00)
    {
      break;
    }
#else //  (FDB_WRITE_GRAN == 8) ||  (FDB_WRITE_GRAN == 32) ||  (FDB_WRITE_GRAN == 64)
    if (status_table[status_num * FDB_WRITE_GRAN / 8] == 0x00)
    {
      break;
    }
#endif
    i++;
  }

  return status_num_bak - i;
}

//====================================================================================
fdb_err_t iFDB_FlashRead(fdb_db_t* db, uint32_t addr, void *buf, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    return _fdb_file_read(db, addr, buf, size);
#else
    return FDB_READ_ERR;
#endif
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    FAL_PartitionRead(db->storage.part, addr, (uint8_t *) buf, size);
#endif
  }

  return result;
}

//====================================================================================
fdb_err_t iFDB_FlashErase(fdb_db_t* db, uint32_t addr, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    return _fdb_file_erase(db, addr, size);
#else
    return FDB_ERASE_ERR;
#endif
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    if (FAL_PartitionErase(db->storage.part, addr, size) < 0)
    {
      result = FDB_ERASE_ERR;
    }
#endif
  }

  return result;
}

//====================================================================================
fdb_err_t iFDB_FlashWrite(fdb_db_t* db, uint32_t addr, const void *buf, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    return _fdb_file_write(db, addr, buf, size);
#else
    return FDB_READ_ERR;
#endif
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    if (FAL_PartitionWrite(db->storage.part, addr, (uint8_t *)buf, size) < 0)
    {
      result = FDB_WRITE_ERR;
    }
#endif
  }

  return result;
}

//====================================================================================
fdb_err_t iFDB_WriteStatus(fdb_db_t* db, uint32_t addr, uint8_t status_table[], size_t status_num, size_t status_index)
{
  fdb_err_t result = FDB_NO_ERR;
  size_t byte_index;

  FDB_ASSERT(status_index < status_num);
  FDB_ASSERT(status_table);

  byte_index = iFDB_SetStatus(status_table, status_num, status_index); // set the status first
  if (byte_index == ~0UL) // the first status table value is all 1, so no need to write flash
  {
    return FDB_NO_ERR;
  }
#if (FDB_WRITE_GRAN == 1)
  result = iFDB_FlashWrite(db, addr + byte_index, (uint32_t *)&status_table[byte_index], 1);
#else // (FDB_WRITE_GRAN == 8) ||  (FDB_WRITE_GRAN == 32) ||  (FDB_WRITE_GRAN == 64)
  // write the status by write granularity some flash (like stm32 onchip) NOT supported repeated write before erase
  result = iFDB_FlashWrite(db, addr + byte_index, (uint32_t *) &status_table[byte_index], FDB_WRITE_GRAN / 8);
#endif // FDB_WRITE_GRAN == 1

  return result;
}

//====================================================================================
size_t iFDB_ReadStatus(fdb_db_t* db, uint32_t addr, uint8_t status_table[], size_t total_num)
{
  FDB_ASSERT(status_table);

  iFDB_FlashRead(db, addr, (uint32_t *) status_table, FDB_STATUS_TABLE_SIZE(total_num));

  return iFDB_GetStatus(status_table, total_num);
}

//==find the continue 0xFF flash address to end address================================
uint32_t iFDB_ContinueFfAddr(fdb_db_t* db, uint32_t start, uint32_t end)
{
  uint8_t buf[32], last_data = 0x00;
  size_t i, addr = start, read_size;

  for (; start < end; start += sizeof(buf))
  {
    if (start + sizeof(buf) < end)
    {
      read_size = sizeof(buf);
    }
    else
    {
      read_size = end - start;
    }
    iFDB_FlashRead(db, start, (uint32_t *) buf, read_size);
    for (i = 0; i < read_size; i++)
    {
      if (last_data != 0xFF && buf[i] == 0xFF)
      {
        addr = start + i;
      }
      last_data = buf[i];
    }
  }

  if (last_data == 0xFF)
  {
    return FDB_WG_ALIGN(addr);
  }
  else
  {
    return end;
  }
}

/****************************************************************************
 * Calculate the CRC32 value of a memory buffer.
 * @param crc accumulated CRC32 value, must be 0 on first call
 * @param buf buffer to calculate CRC32 value for
 * @param size bytes in buffer
 * @return calculated CRC32 value
 ***************************************************************************/
uint32_t iFDB_CalcCrc32(uint32_t crc, const void *buf, size_t size)
{
  const uint8_t *p;

  p = (const uint8_t *)buf;
  crc = crc ^ ~0U;

  while (size--)
  {
    crc = crc32_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
  }

  return crc ^ ~0U;
}

//====================================================================================
fdb_err_t iFDB_InitEx(fdb_db_t* db, const char *name, const char *part_name, fdb_db_type_t type, void *user_data)
{
  FDB_ASSERT(db);
  FDB_ASSERT(name);
  FDB_ASSERT(part_name);

  if (db->init_ok)
  {
    return FDB_NO_ERR;
  }

  db->name = name;
  db->type = type;
  db->user_data = user_data;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    FDB_ASSERT(db->sec_size != 0);  // must set when using file mode
    FDB_ASSERT(db->max_size != 0);
#ifdef FDB_USING_POSIX_MODE
    db->cur_file = -1;
#else
    db->cur_file = 0;
#endif
    db->storage.dir = part_name;
    FDB_ASSERT(strlen(part_name) != 0)
#endif
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    size_t block_size;

    FAL_Init();  // FAL (Flash Abstraction Layer) initialization
    if ((db->storage.part = FAL_PartitionFind(part_name)) == NULL)  // check the flash partition
    {
      FDB_INFO("Error: Partition (%s) not found.\n", part_name);
      return FDB_PART_NOT_FOUND;
    }

    block_size = FAL_FlashDeviceFind(db->storage.part->flash_name)->blk_size;
    if (db->sec_size == 0)
    {
      db->sec_size = block_size;
    }
    else
    {
      FDB_ASSERT(db->sec_size % block_size == 0);  // must be aligned with block size
    }

    db->max_size = db->storage.part->len;
#endif // FDB_USING_FAL_MODE
  }

  FDB_ASSERT(db->max_size % db->sec_size == 0);  // must align with sector size
  FDB_ASSERT(db->max_size / db->sec_size >= 2);  // must have more than or equal 2 sector

  return FDB_NO_ERR;
}

//====================================================================================
void iFDB_InitFinish(fdb_db_t* db, fdb_err_t result)
{
  static bool log_is_show = false;
  if (result == FDB_NO_ERR)
  {
    db->init_ok = true;
    if (!log_is_show)
    {
      FDB_INFO("FlashDB V%s Init OK.\n", FDB_SW_VERSION);
      log_is_show = true;
    }
  }
  else
  {
    FDB_INFO("Error: %s (%s) Init Fail (%d).\n", db->type == FDB_DB_TYPE_KV ? "KVDB" : "TSDB", db->name, (int)result);
  }
}

/****************************************************************************
 * Make a blob object.
 * @param blob blob object
 * @param value_buf value buffer
 * @param buf_len buffer length
 * @return new blob object
 ***************************************************************************/
fdb_blob_t* FlashDB_BlobMake(fdb_blob_t* blob, const void *value_buf, size_t buf_len)
{
  blob->buf = (void *)value_buf;
  blob->size = buf_len;

  return blob;
}

/****************************************************************************
 * Read the blob object in database.
 * @param db database object
 * @param blob blob object
 * @return read length
 ***************************************************************************/
size_t FlashDB_BlobRead(fdb_db_t* db, fdb_blob_t* blob)
{
  size_t read_len = blob->size;

  if (read_len > blob->saved.len)
  {
    read_len = blob->saved.len;
  }
  iFDB_FlashRead(db, blob->saved.addr, blob->buf, read_len);

  return read_len;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// KVDB feature: Key-Value Database feature implement source file
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(FDB_USING_KVDB)
#ifdef FDB_KV_USING_CACHE
/***************************************************************************
 * It's only caching the current using status sector's empty_addr
 ***************************************************************************/
static void iKVDB_UpdateSectorCache(fdb_kvdb_t* db, uint32_t sec_addr, uint32_t empty_addr)
{
  size_t i, empty_index = FDB_SECTOR_CACHE_TABLE_SIZE;

  for (i = 0; i < FDB_SECTOR_CACHE_TABLE_SIZE; i++)
  {
    if ((empty_addr > sec_addr) && (empty_addr < sec_addr + FDB_SECTOR_SIZE(db)))
    {
      if (db->sector_cache_table[i].addr == sec_addr)  // update the sector empty_addr in cache
      {
        db->sector_cache_table[i].addr = sec_addr;
        db->sector_cache_table[i].empty_addr = empty_addr;
        return;
      }
      else if ((db->sector_cache_table[i].addr == FDB_DATA_UNUSED) && (empty_index == FDB_SECTOR_CACHE_TABLE_SIZE))
      {
        empty_index = i;
      }
    }
    else if (db->sector_cache_table[i].addr == sec_addr)
    {
      db->sector_cache_table[i].addr = FDB_DATA_UNUSED;  // delete the sector which status is not current using
      return;
    }
  }

  if (empty_index < FDB_SECTOR_CACHE_TABLE_SIZE)  // add the sector empty_addr to cache
  {
    db->sector_cache_table[empty_index].addr = sec_addr;
    db->sector_cache_table[empty_index].empty_addr = empty_addr;
  }
}

/***************************************************************************
 * Get sector info from cache. It's return true when cache is hit.
 ***************************************************************************/
static bool iKVDB_GetSectorFromCache(fdb_kvdb_t* db, uint32_t sec_addr, uint32_t *empty_addr)
{
  size_t i;

  for (i = 0; i < FDB_SECTOR_CACHE_TABLE_SIZE; i++)
  {
    if (db->sector_cache_table[i].addr == sec_addr)
    {
      if (empty_addr)
      {
        *empty_addr = db->sector_cache_table[i].empty_addr;
      }
      return true;
    }
  }

  return false;
}

//====================================================================================
static void iKVDB_UpdataKvCache(fdb_kvdb_t* db, const char *name, size_t name_len, uint32_t addr)
{
  size_t i, empty_index = FDB_KV_CACHE_TABLE_SIZE, min_activity_index = FDB_KV_CACHE_TABLE_SIZE;
  uint16_t name_crc = (uint16_t) (iFDB_CalcCrc32(0, name, name_len) >> 16), min_activity = 0xFFFF;

  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    if (addr != FDB_DATA_UNUSED)
    {
      if (db->kv_cache_table[i].name_crc == name_crc)  // update the KV address in cache
      {
        db->kv_cache_table[i].addr = addr;
        return;
      }
      else if ((db->kv_cache_table[i].addr == FDB_DATA_UNUSED) && (empty_index == FDB_KV_CACHE_TABLE_SIZE))
      {
        empty_index = i;
      }
      else if (db->kv_cache_table[i].addr != FDB_DATA_UNUSED)
      {
        if (db->kv_cache_table[i].active > 0)
        {
          db->kv_cache_table[i].active--;
        }
        if (db->kv_cache_table[i].active < min_activity)
        {
          min_activity_index = i;
          min_activity = db->kv_cache_table[i].active;
        }
      }
    }
    else if (db->kv_cache_table[i].name_crc == name_crc)
    {
      db->kv_cache_table[i].addr = FDB_DATA_UNUSED;  // delete the KV
      db->kv_cache_table[i].active = 0;
      return;
    }
  }
  // add the KV to cache, using LRU (Least Recently Used) like algorithm
  if (empty_index < FDB_KV_CACHE_TABLE_SIZE)
  {
    db->kv_cache_table[empty_index].addr = addr;
    db->kv_cache_table[empty_index].name_crc = name_crc;
    db->kv_cache_table[empty_index].active = 0;
  }
  else if (min_activity_index < FDB_KV_CACHE_TABLE_SIZE)
  {
    db->kv_cache_table[min_activity_index].addr = addr;
    db->kv_cache_table[min_activity_index].name_crc = name_crc;
    db->kv_cache_table[min_activity_index].active = 0;
  }
}

/***************************************************************************
 * Get KV info from cache. It's return true when cache is hit.
 ***************************************************************************/
static bool iKVDB_GetKvFromCache(fdb_kvdb_t* db, const char *name, size_t name_len, uint32_t *addr)
{
  size_t i;
  uint16_t name_crc = (uint16_t) (iFDB_CalcCrc32(0, name, name_len) >> 16);

  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    if ((db->kv_cache_table[i].addr != FDB_DATA_UNUSED) && (db->kv_cache_table[i].name_crc == name_crc))
    {
      char saved_name[FDB_KV_NAME_MAX];
      iFDB_FlashRead((fdb_db_t*)db, db->kv_cache_table[i].addr + KV_HDR_DATA_SIZE, (uint32_t *) saved_name, FDB_KV_NAME_MAX);  // read the KV name in flash
      if (!strncmp(name, saved_name, name_len))
      {
        *addr = db->kv_cache_table[i].addr;
        if (db->kv_cache_table[i].active >= 0xFFFF - FDB_KV_CACHE_TABLE_SIZE)
        {
          db->kv_cache_table[i].active = 0xFFFF;
        }
        else
        {
          db->kv_cache_table[i].active += FDB_KV_CACHE_TABLE_SIZE;
        }
        return true;
      }
    }
  }

  return false;
}
#endif /* FDB_KV_USING_CACHE */

/***************************************************************************
 * find the next KV address by magic word on the flash
 ***************************************************************************/
static uint32_t iKVDB_FindNextKvAddr(fdb_kvdb_t* db, uint32_t start, uint32_t end)
{
  uint8_t buf[32];
  uint32_t start_bak = start, i;
  uint32_t magic;

#ifdef FDB_KV_USING_CACHE
  uint32_t empty_kv;
  if (iKVDB_GetSectorFromCache(db, FDB_ALIGN_DOWN(start, FDB_SECTOR_SIZE(db)), &empty_kv) && start == empty_kv)
  {
    return FAILED_ADDR;
  }
#endif

  for (; start < end && start + sizeof(buf) < end; start += (sizeof(buf) - sizeof(uint32_t)))
  {
    iFDB_FlashRead((fdb_db_t*)db, start, (uint32_t *) buf, sizeof(buf));
    for (i = 0; i < sizeof(buf) - sizeof(uint32_t) && start + i < end; i++)
    {
#ifndef FDB_BIG_ENDIAN 
      magic = buf[i] + (buf[i + 1] << 8) + (buf[i + 2] << 16) + (buf[i + 3] << 24);  /// Little Endian Order
#else 
      magic = buf[i + 3] + (buf[i + 2] << 8) + (buf[i + 1] << 16) + (buf[i] << 24);  /// Big Endian Order
#endif
      if (magic == KV_MAGIC_WORD && (start + i - KV_MAGIC_OFFSET) >= start_bak)
      {
        return start + i - KV_MAGIC_OFFSET;
      }
    }
  }

  return FAILED_ADDR;
}

//====================================================================================
static uint32_t iKVDB_GetNextKvAddr(fdb_kvdb_t* db, kvdb_sec_info_t* sector, fdb_kv_t* pre_kv)
{
  uint32_t addr = FAILED_ADDR;

  if (sector->status.store == FDB_SECTOR_STORE_EMPTY)
  {
    return FAILED_ADDR;
  }

  if (pre_kv->addr.start == FAILED_ADDR)
  {
    addr = sector->addr + KV_SECTOR_HDR_DATA_SIZE;  // the first KV address
  }
  else
  {
    if (pre_kv->addr.start <= sector->addr + FDB_SECTOR_SIZE(db))
    {
      if (pre_kv->crc_is_ok)
      {
        addr = pre_kv->addr.start + pre_kv->len;
      }
      else
      {
        // when pre_kv CRC check failed, maybe the flash has error data iKVDB_FindNextKvAddr after pre_kv address
        addr = pre_kv->addr.start + FDB_WG_ALIGN(1);
      }
      addr = iKVDB_FindNextKvAddr(db, addr, sector->addr + FDB_SECTOR_SIZE(db));  // check and find next KV address

      if (addr > sector->addr + FDB_SECTOR_SIZE(db) || pre_kv->len == 0)
      {
        //TODO 扇区连续模式
        return FAILED_ADDR;
      }
    }
    else
    {
      return FAILED_ADDR;  // no KV
    }
  }

  return addr;
}

//====================================================================================
static fdb_err_t iKVDB_ReadKv(fdb_kvdb_t* db, fdb_kv_t* kv)
{
  kv_hdr_data_t kv_hdr;
  uint8_t buf[32];
  uint32_t calc_crc32 = 0, crc_data_len, kv_name_addr;
  fdb_err_t result = FDB_NO_ERR;
  size_t len, size;

  iFDB_FlashRead((fdb_db_t*)db, kv->addr.start, (uint32_t *)&kv_hdr, sizeof(kv_hdr_data_t));  // read KV header raw data
  kv->status = (fdb_kv_status_t) iFDB_GetStatus(kv_hdr.status_table, FDB_KV_STATUS_NUM);
  kv->len = kv_hdr.len;

  if (kv->len == ~0UL || kv->len > FDB_MAX_SIZE(db) || kv->len < KV_NAME_LEN_OFFSET)
  {
    kv->len = KV_HDR_DATA_SIZE;  // the KV length was not write, so reserved the info for current KV
    if (kv->status != FDB_KV_ERR_HDR)
    {
      kv->status = FDB_KV_ERR_HDR;
      KVDB_DEBUG("Error: The KV @0x%08" PRIX32 " length has an error.\n", kv->addr.start);
      iFDB_WriteStatus((fdb_db_t*)db, kv->addr.start, kv_hdr.status_table, FDB_KV_STATUS_NUM, FDB_KV_ERR_HDR);
    }
    kv->crc_is_ok = false;
    return FDB_READ_ERR;
  }
  else if (kv->len > FDB_SECTOR_SIZE(db) - KV_SECTOR_HDR_DATA_SIZE && kv->len < FDB_MAX_SIZE(db))
  {
    //TODO 扇区连续模式，或者写入长度没有写入完整
    FDB_ASSERT(0);
  }

  crc_data_len = kv->len - KV_NAME_LEN_OFFSET;  // CRC32 data len(header.name_len + header.value_len + name + value)
  for (len = 0, size = 0; len < crc_data_len; len += size)  // calculate the CRC32 value
  {
    if (len + sizeof(buf) < crc_data_len)
    {
      size = sizeof(buf);
    }
    else
    {
      size = crc_data_len - len;
    }

    iFDB_FlashRead((fdb_db_t*)db, kv->addr.start + KV_NAME_LEN_OFFSET + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
    calc_crc32 = iFDB_CalcCrc32(calc_crc32, buf, size);
  }

  if (calc_crc32 != kv_hdr.crc32) // check CRC32
  {
    kv->crc_is_ok = false;
    result = FDB_READ_ERR;
  }
  else
  {
    kv->crc_is_ok = true;
    kv_name_addr = kv->addr.start + KV_HDR_DATA_SIZE;  // the name is behind aligned KV header
    iFDB_FlashRead((fdb_db_t*)db, kv_name_addr, (uint32_t *) kv->name, FDB_WG_ALIGN(kv_hdr.name_len));
    kv->addr.value = kv_name_addr + FDB_WG_ALIGN(kv_hdr.name_len);  // the value is behind aligned name
    kv->value_len = kv_hdr.value_len;
    kv->name_len = kv_hdr.name_len;
    if (kv_hdr.name_len >= sizeof(kv->name) / sizeof(kv->name[0]))
    {
      kv_hdr.name_len = sizeof(kv->name) / sizeof(kv->name[0]) - 1;
    }
    kv->name[kv_hdr.name_len] = '\0';
  }

  return result;
}

//====================================================================================
static fdb_err_t iKVDB_ReadSectorInfo(fdb_kvdb_t* db, uint32_t addr, kvdb_sec_info_t* sector, bool traversal)
{
  fdb_err_t result = FDB_NO_ERR;
  kv_sector_hdr_data_t sec_hdr = { 0 };

  FDB_ASSERT(addr % FDB_SECTOR_SIZE(db) == 0);
  FDB_ASSERT(sector);

  iFDB_FlashRead((fdb_db_t*)db, addr, (uint32_t *)&sec_hdr, sizeof(kv_sector_hdr_data_t));  // read sector header raw data

  sector->addr = addr;
  sector->magic = sec_hdr.magic;
  if (sector->magic != KV_SECTOR_MAGIC_WORD)  // check magic word
  {
    sector->check_ok = false;
    sector->combined = KV_SECTOR_NOT_COMBINED;
    return FDB_INIT_FAILED;
  }
  sector->check_ok = true;
  // get other sector info
  sector->combined = sec_hdr.combined;
  sector->status.store = (fdb_sector_store_status_t) iFDB_GetStatus(sec_hdr.status_table.store, FDB_SECTOR_STORE_STATUS_NUM);
  sector->status.dirty = (fdb_sector_dirty_status_t) iFDB_GetStatus(sec_hdr.status_table.dirty, FDB_SECTOR_DIRTY_STATUS_NUM);
  if (traversal)  // traversal all KV and calculate the remain space size
  {
    sector->remain = 0;
    sector->empty_kv = sector->addr + KV_SECTOR_HDR_DATA_SIZE;
    if (sector->status.store == FDB_SECTOR_STORE_EMPTY)
    {
      sector->remain = FDB_SECTOR_SIZE(db) - KV_SECTOR_HDR_DATA_SIZE;
    }
    else if (sector->status.store == FDB_SECTOR_STORE_USING)
    {
      fdb_kv_t kv_obj;

#ifdef FDB_KV_USING_CACHE
      if (iKVDB_GetSectorFromCache(db, addr, &sector->empty_kv))
      {
        sector->remain = FDB_SECTOR_SIZE(db) - (sector->empty_kv - sector->addr);
        return result;
      }
#endif

      sector->remain = FDB_SECTOR_SIZE(db) - KV_SECTOR_HDR_DATA_SIZE;
      kv_obj.addr.start = sector->addr + KV_SECTOR_HDR_DATA_SIZE;
      do
      {
        iKVDB_ReadKv(db, &kv_obj);
        if (!kv_obj.crc_is_ok)
        {
          if (kv_obj.status != FDB_KV_PRE_WRITE && kv_obj.status != FDB_KV_ERR_HDR)
          {
            KVDB_INFO("Error: The KV (@0x%08" PRIX32 ") CRC32 check failed!\n", kv_obj.addr.start);
            sector->remain = 0;
            result = FDB_READ_ERR;
            break;
          }
        }
        sector->empty_kv += kv_obj.len;
        sector->remain -= kv_obj.len;
      } while ((kv_obj.addr.start = iKVDB_GetNextKvAddr(db, sector, &kv_obj)) != FAILED_ADDR);
      
      // check the empty KV address by read continue 0xFF on flash
      {
        uint32_t ff_addr;

        ff_addr = iFDB_ContinueFfAddr((fdb_db_t*)db, sector->empty_kv, sector->addr + FDB_SECTOR_SIZE(db));
        if (sector->empty_kv != ff_addr)  // check the flash data is clean
        {
          // update the sector information
          sector->empty_kv = ff_addr;
          sector->remain = FDB_SECTOR_SIZE(db) - (ff_addr - sector->addr);
        }
      }

#ifdef FDB_KV_USING_CACHE
      iKVDB_UpdateSectorCache(db, sector->addr, sector->empty_kv);
#endif
    }
  }

  return result;
}

//====================================================================================
static uint32_t iKVDB_GetNextSectorAddr(fdb_kvdb_t* db, kvdb_sec_info_t* pre_sec)
{
  uint32_t next_addr;

  if (pre_sec->addr == FAILED_ADDR)
  {
    return 0;  // the next sector is on the top of the partition
  }
  else
  {
    if (pre_sec->combined == KV_SECTOR_NOT_COMBINED)  // check KV sector combined
    {
      next_addr = pre_sec->addr + FDB_SECTOR_SIZE(db);
    }
    else
    {
      next_addr = pre_sec->addr + pre_sec->combined * FDB_SECTOR_SIZE(db);
    }

    if (next_addr < FDB_MAX_SIZE(db))  // check range
    {
      return next_addr;
    }
    else
    {
      return FAILED_ADDR;  // no sector
    }
  }
}

//====================================================================================
static void iKVDB_KvIterator(fdb_kvdb_t* db, fdb_kv_t* kv, void *arg1, void *arg2, bool (*callback)(fdb_kv_t* kv, void *arg1, void *arg2))
{
  kvdb_sec_info_t sector;
  uint32_t sec_addr;

  sec_addr = 0;
  do  // search all sectors
  {
    if (iKVDB_ReadSectorInfo(db, sec_addr, &sector, false) != FDB_NO_ERR)
    {
      continue;
    }
    if (callback == NULL)
    {
      continue;
    }
    if (sector.status.store == FDB_SECTOR_STORE_USING || sector.status.store == FDB_SECTOR_STORE_FULL)  // sector has KV
    {
      kv->addr.start = sector.addr + KV_SECTOR_HDR_DATA_SIZE;
      do  // search all KV
      {
        iKVDB_ReadKv(db, kv);
        if (callback(kv, arg1, arg2))  // iterator is interrupted when callback return true
        {
          return;
        }
      } while ((kv->addr.start = iKVDB_GetNextKvAddr(db, &sector, kv)) != FAILED_ADDR);
    }
  } while ((sec_addr = iKVDB_GetNextSectorAddr(db, &sector)) != FAILED_ADDR);
}

//====================================================================================
static bool iKVDB_FindKvCb(fdb_kv_t* kv, void *arg1, void *arg2)
{
  const char *key = arg1;
  bool *find_ok = arg2;
  size_t key_len = strlen(key);

  if (key_len != kv->name_len)
  {
    return false;
  }
  if (kv->crc_is_ok && kv->status == FDB_KV_WRITE && !strncmp(kv->name, key, key_len)) // check KV
  {
    *find_ok = true;
    return true;
  }
  return false;
}

//====================================================================================
static bool iKVDB_FindKvNoCache(fdb_kvdb_t* db, const char *key, fdb_kv_t* kv)
{
  bool find_ok = false;

  iKVDB_KvIterator(db, kv, (void *)key, &find_ok, iKVDB_FindKvCb);

  return find_ok;
}

//====================================================================================
static bool iKVDB_FindKv(fdb_kvdb_t* db, const char *key, fdb_kv_t* kv)
{
  bool find_ok = false;

#ifdef FDB_KV_USING_CACHE
  size_t key_len = strlen(key);
  if (iKVDB_GetKvFromCache(db, key, key_len, &kv->addr.start))
  {
    iKVDB_ReadKv(db, kv);
    return true;
  }
#endif

  find_ok = iKVDB_FindKvNoCache(db, key, kv);

#ifdef FDB_KV_USING_CACHE
  if (find_ok)
  {
    iKVDB_UpdataKvCache(db, key, key_len, kv->addr.start);
  }
#endif

  return find_ok;
}

//====================================================================================
static size_t iKVDB_GetKv(fdb_kvdb_t* db, const char *key, void *value_buf, size_t buf_len, size_t *value_len)
{
  fdb_kv_t kv;
  size_t read_len = 0;

  if (iKVDB_FindKv(db, key, &kv))
  {
    if (value_len)
    {
      *value_len = kv.value_len;
    }
    if (buf_len > kv.value_len)
    {
      read_len = kv.value_len;
    }
    else
    {
      read_len = buf_len;
    }
    if (value_buf)
    {
      iFDB_FlashRead((fdb_db_t*)db, kv.addr.value, (uint32_t *) value_buf, read_len);
    }
  }
  else if (value_len)
  {
    *value_len = 0;
  }

  return read_len;
}

//====================================================================================
static fdb_err_t iKVDB_WriteKvHdr(fdb_kvdb_t* db, uint32_t addr, kv_hdr_data_t* kv_hdr)
{
  fdb_err_t result = FDB_NO_ERR;

  result = iFDB_WriteStatus((fdb_db_t*)db, addr, kv_hdr->status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_WRITE); // write the status will by write granularity
  if (result != FDB_NO_ERR)
  {
    return result;
  }
  result = iFDB_FlashWrite((fdb_db_t*)db, addr + KV_MAGIC_OFFSET, &kv_hdr->magic, sizeof(kv_hdr_data_t) - KV_MAGIC_OFFSET); // write other header data

  return result;
}

//====================================================================================
static fdb_err_t iKVDB_FormatSentor(fdb_kvdb_t* db, uint32_t addr, uint32_t combined_value)
{
  fdb_err_t result = FDB_NO_ERR;
  kv_sector_hdr_data_t sec_hdr = { 0 };

  FDB_ASSERT(addr % FDB_SECTOR_SIZE(db) == 0);

  result = iFDB_FlashErase((fdb_db_t*)db, addr, FDB_SECTOR_SIZE(db));
  if (result == FDB_NO_ERR)
  {
    // initialize the header data
    memset(&sec_hdr, 0xFF, sizeof(kv_sector_hdr_data_t));
    iFDB_SetStatus(sec_hdr.status_table.store, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_EMPTY);
    iFDB_SetStatus(sec_hdr.status_table.dirty, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_FALSE);
    sec_hdr.magic = KV_SECTOR_MAGIC_WORD;
    sec_hdr.combined = combined_value;
    sec_hdr.reserved = 0xFFFFFFFF;
    result = iFDB_FlashWrite((fdb_db_t*)db, addr, (uint32_t *)&sec_hdr, sizeof(kv_sector_hdr_data_t));  // save the header

#ifdef FDB_KV_USING_CACHE
    iKVDB_UpdateSectorCache(db, addr, addr + FDB_SECTOR_SIZE(db));  // delete the sector cache
#endif
  }

  return result;
}

//====================================================================================
static fdb_err_t iKVDB_UpdateSentorStatus(fdb_kvdb_t* db, kvdb_sec_info_t* sector, size_t new_kv_len, bool *is_full)
{
  uint8_t status_table[FDB_STORE_STATUS_TABLE_SIZE];
  fdb_err_t result = FDB_NO_ERR;

  if (sector->status.store == FDB_SECTOR_STORE_EMPTY)  // change the current sector status
  {
    result = iFDB_WriteStatus((fdb_db_t*)db, sector->addr, status_table, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_USING);  // change the sector status to using
  }
  else if (sector->status.store == FDB_SECTOR_STORE_USING)
  {
    if (sector->remain < FDB_SEC_REMAIN_THRESHOLD || sector->remain - new_kv_len < FDB_SEC_REMAIN_THRESHOLD)  // check remain size
    {
      result = iFDB_WriteStatus((fdb_db_t*)db, sector->addr, status_table, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_FULL); // change the sector status to full

#ifdef FDB_KV_USING_CACHE
      iKVDB_UpdateSectorCache(db, sector->addr, sector->addr + FDB_SECTOR_SIZE(db));  // delete the sector cache
#endif

      if (is_full)
      {
        *is_full = true;
      }
    }
    else if (is_full)
    {
      *is_full = false;
    }
  }

  return result;
}

//====================================================================================
static void iKVDB_SentorIterator(fdb_kvdb_t* db, kvdb_sec_info_t* sector, fdb_sector_store_status_t status, void *arg1, void *arg2,
                            bool (*callback)(kvdb_sec_info_t* sector, void *arg1, void *arg2), bool traversal_kv)
{
  uint32_t sec_addr;

  // search all sectors
  sec_addr = 0;
  do
  {
    iKVDB_ReadSectorInfo(db, sec_addr, sector, false);
    if (status == FDB_SECTOR_STORE_UNUSED || status == sector->status.store)
    {
      if (traversal_kv)
      {
        iKVDB_ReadSectorInfo(db, sec_addr, sector, true);
      }
      if (callback && callback(sector, arg1, arg2))  // iterator is interrupted when callback return true
      {
        return;
      }
    }
  } while ((sec_addr = iKVDB_GetNextSectorAddr(db, sector)) != FAILED_ADDR);
}

//====================================================================================
static bool iKVDB_SentorStatisticsCb(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  size_t *empty_sector = arg1, *using_sector = arg2;

  if (sector->check_ok && sector->status.store == FDB_SECTOR_STORE_EMPTY)
  {
    (*empty_sector)++;
  }
  else if (sector->check_ok && sector->status.store == FDB_SECTOR_STORE_USING)
  {
    (*using_sector)++;
  }

  return false;
}

//====================================================================================
static bool iKVDB_AllocKvCb(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  alloc_kv_cb_args_t *arg = arg1;

  // 1. sector has space
  // 2. the NO dirty sector
  // 3. the dirty sector only when the gc_request is false
  if (sector->check_ok && sector->remain > arg->kv_size && ((sector->status.dirty == FDB_SECTOR_DIRTY_FALSE)
      || (sector->status.dirty == FDB_SECTOR_DIRTY_TRUE && !arg->db->gc_request)))
  {
    *(arg->empty_kv) = sector->empty_kv;
    return true;
  }

  return false;
}

//====================================================================================
static uint32_t iKVDB_AllocKv(fdb_kvdb_t* db, kvdb_sec_info_t* sector, size_t kv_size)
{
  uint32_t empty_kv = FAILED_ADDR;
  size_t empty_sector = 0, using_sector = 0;
  alloc_kv_cb_args_t arg = {db, kv_size, &empty_kv};

  iKVDB_SentorIterator(db, sector, FDB_SECTOR_STORE_UNUSED, &empty_sector, &using_sector, iKVDB_SentorStatisticsCb, false);  // sector status statistics
  if (using_sector > 0)
  {
    iKVDB_SentorIterator(db, sector, FDB_SECTOR_STORE_USING, &arg, NULL, iKVDB_AllocKvCb, true);  // alloc the KV from the using status sector first
  }
  if (empty_sector > 0 && empty_kv == FAILED_ADDR)
  {
    if (empty_sector > FDB_GC_EMPTY_SEC_THRESHOLD || db->gc_request)
    {
      iKVDB_SentorIterator(db, sector, FDB_SECTOR_STORE_EMPTY, &arg, NULL, iKVDB_AllocKvCb, true);
    }
    else
    {
      KVDB_DEBUG("Trigger a GC check after alloc KV failed.\n");  // no space for new KV now will GC and retry
      db->gc_request = true;
    }
  }

  return empty_kv;
}

//====================================================================================
static fdb_err_t iKVDB_DelKv(fdb_kvdb_t* db, const char *key, fdb_kv_t* old_kv, bool complete_del)
{
  fdb_err_t result = FDB_NO_ERR;
  uint32_t dirty_status_addr;

  uint8_t status_table[KV_STATUS_TABLE_SIZE >= FDB_DIRTY_STATUS_TABLE_SIZE ? KV_STATUS_TABLE_SIZE : FDB_DIRTY_STATUS_TABLE_SIZE];

  if (!old_kv)  // need find KV
  {
    fdb_kv_t kv;
    if (iKVDB_FindKv(db, key, &kv))  // find KV
    {
      old_kv = &kv;
    }
    else
    {
      KVDB_DEBUG("Not found '%s' in KV.\n", key);
      return FDB_KV_NAME_ERR;
    }
  }

  if (!complete_del)  // change and save the new status
  {
    result = iFDB_WriteStatus((fdb_db_t*)db, old_kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_DELETE);
    db->last_is_complete_del = true;
  }
  else
  {
    result = iFDB_WriteStatus((fdb_db_t*)db, old_kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_DELETED);

    if (!db->last_is_complete_del && result == FDB_NO_ERR)
    {
#ifdef FDB_KV_USING_CACHE
      if (key != NULL)  // delete the KV in flash and cache
      {
        // when using iKVDB_DelKv(db, key, NULL, true) or iKVDB_DelKv(db, key, kv, true) in fdb_del_kv(db, ) and iKVDB_SetKv(db, )
        iKVDB_UpdataKvCache(db, key, strlen(key), FDB_DATA_UNUSED);
      }
      else if (old_kv != NULL)
      {
        // when using iKVDB_DelKv(db, NULL, kv, true) in iKVDB_MoveKv(db, )
        iKVDB_UpdataKvCache(db, old_kv->name, old_kv->name_len, FDB_DATA_UNUSED);
      }
#endif
    }

    db->last_is_complete_del = false;
  }

  dirty_status_addr = FDB_ALIGN_DOWN(old_kv->addr.start, FDB_SECTOR_SIZE(db)) + KV_SECTOR_DIRTY_OFFSET;
  // read and change the sector dirty status
  if (result == FDB_NO_ERR && (iFDB_ReadStatus((fdb_db_t*)db, dirty_status_addr, status_table, FDB_SECTOR_DIRTY_STATUS_NUM) == FDB_SECTOR_DIRTY_FALSE))
  {
    result = iFDB_WriteStatus((fdb_db_t*)db, dirty_status_addr, status_table, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_TRUE);
  }

  return result;
}

//==move the KV to new space==========================================================
static fdb_err_t iKVDB_MoveKv(fdb_kvdb_t* db, fdb_kv_t* kv)
{
  fdb_err_t result = FDB_NO_ERR;
  uint8_t status_table[KV_STATUS_TABLE_SIZE];
  uint32_t kv_addr;
  kvdb_sec_info_t sector;

  if (kv->status == FDB_KV_WRITE)  // prepare to delete the current KV
  {
    iKVDB_DelKv(db, NULL, kv, false);
  }

  if ((kv_addr = iKVDB_AllocKv(db, &sector, kv->len)) != FAILED_ADDR)
  {
    if (db->in_recovery_check)
    {
      fdb_kv_t kv_bak;
      char name[FDB_KV_NAME_MAX + 1] = { 0 };
      strncpy(name, kv->name, kv->name_len);
      if (iKVDB_FindKvNoCache(db, name, &kv_bak))  // check the KV in flash is already create success
      {
        result = FDB_NO_ERR;  // already create success, don't need to duplicate
        goto __exit;
      }
    }
  }
  else
  {
    return FDB_SAVED_FULL;
  }
  
  // start move the KV
  {
    uint8_t buf[32];
    size_t len, size, kv_len = kv->len;

    iKVDB_UpdateSentorStatus(db, &sector, kv->len, NULL);  // update the new KV sector status first
    iFDB_WriteStatus((fdb_db_t*)db, kv_addr, status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_WRITE);
    kv_len -= KV_MAGIC_OFFSET;
    for (len = 0, size = 0; len < kv_len; len += size)
    {
      if (len + sizeof(buf) < kv_len)
      {
        size = sizeof(buf);
      }
      else
      {
        size = kv_len - len;
      }
      iFDB_FlashRead((fdb_db_t*)db, kv->addr.start + KV_MAGIC_OFFSET + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
      result = iFDB_FlashWrite((fdb_db_t*)db, kv_addr + KV_MAGIC_OFFSET + len, (uint32_t *) buf, size);
    }
    iFDB_WriteStatus((fdb_db_t*)db, kv_addr, status_table, FDB_KV_STATUS_NUM, FDB_KV_WRITE);

#ifdef FDB_KV_USING_CACHE
    iKVDB_UpdateSectorCache(db, FDB_ALIGN_DOWN(kv_addr, FDB_SECTOR_SIZE(db)), (kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv->name_len) + FDB_WG_ALIGN(kv->value_len)));
    iKVDB_UpdataKvCache(db, kv->name, kv->name_len, kv_addr);
#endif
  }

  KVDB_DEBUG("Moved the KV (%.*s) from 0x%08" PRIX32 " to 0x%08" PRIX32 ".\n", kv->name_len, kv->name, kv->addr.start, kv_addr);

__exit:
  iKVDB_DelKv(db, NULL, kv, true);

  return result;
}

//====================================================================================
static uint32_t iKVDB_NewKv(fdb_kvdb_t* db, kvdb_sec_info_t* sector, size_t kv_size)
{
  bool already_gc = false;
  uint32_t empty_kv = FAILED_ADDR;

__retry:

  if ((empty_kv = iKVDB_AllocKv(db, sector, kv_size)) == FAILED_ADDR && db->gc_request && !already_gc)
  {
    KVDB_DEBUG("Warning: Alloc an KV (size %zu) failed when new KV. Now will GC then retry.\n", kv_size);
    iKVDB_GcCollect(db);
    already_gc = true;
    goto __retry;
  }

  return empty_kv;
}

//====================================================================================
static uint32_t iKVDB_NewKvEx(fdb_kvdb_t* db, kvdb_sec_info_t* sector, size_t key_len, size_t buf_len)
{
  size_t kv_len = KV_HDR_DATA_SIZE + FDB_WG_ALIGN(key_len) + FDB_WG_ALIGN(buf_len);

  return iKVDB_NewKv(db, sector, kv_len);
}

//====================================================================================
static bool iKVDB_GcCheckCb(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  size_t *empty_sec = arg1;

  if (sector->check_ok)
  {
    *empty_sec = *empty_sec + 1;
  }

  return false;
}

//====================================================================================
static bool iKVDB_DoGc(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  fdb_kv_t kv;
  fdb_kvdb_t* db = arg1;

  if (sector->check_ok && (sector->status.dirty == FDB_SECTOR_DIRTY_TRUE || sector->status.dirty == FDB_SECTOR_DIRTY_GC))
  {
    uint8_t status_table[FDB_DIRTY_STATUS_TABLE_SIZE];

    iFDB_WriteStatus((fdb_db_t*)db, sector->addr + KV_SECTOR_DIRTY_OFFSET, status_table, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_GC);  // change the sector status to GC
    kv.addr.start = sector->addr + KV_SECTOR_HDR_DATA_SIZE;  // search all KV
    do
    {
      iKVDB_ReadKv(db, &kv);
      if (kv.crc_is_ok && (kv.status == FDB_KV_WRITE || kv.status == FDB_KV_PRE_DELETE))
      {
        if (iKVDB_MoveKv(db, &kv) != FDB_NO_ERR)  // move the KV to new space
        {
          KVDB_DEBUG("Error: Moved the KV (%.*s) for GC failed.\n", kv.name_len, kv.name);
        }
      }
    } while ((kv.addr.start = iKVDB_GetNextKvAddr(db, sector, &kv)) != FAILED_ADDR);
    iKVDB_FormatSentor(db, sector->addr, KV_SECTOR_NOT_COMBINED);
    KVDB_DEBUG("Collect a sector @0x%08" PRIX32 "\n", sector->addr);
  }

  return false;
}

/**************************************************************************
 * The GC will be triggered on the following scene:
 * 1. alloc an KV when the flash not has enough space
 * 2. write an KV then the flash not has enough space
 **************************************************************************/
static void iKVDB_GcCollect(fdb_kvdb_t* db)
{
  kvdb_sec_info_t sector;
  size_t empty_sec = 0;

  iKVDB_SentorIterator(db, &sector, FDB_SECTOR_STORE_EMPTY, &empty_sec, NULL, iKVDB_GcCheckCb, false);  // GC check the empty sector number
  KVDB_DEBUG("The remain empty sector is %zu, GC threshold is %d.\n", empty_sec, FDB_GC_EMPTY_SEC_THRESHOLD);  // do GC collect
  if (empty_sec <= FDB_GC_EMPTY_SEC_THRESHOLD)
  {
    iKVDB_SentorIterator(db, &sector, FDB_SECTOR_STORE_UNUSED, db, NULL, iKVDB_DoGc, false);
  }

  db->gc_request = false;
}

//====================================================================================
static fdb_err_t iKVDB_AlignWrite(fdb_kvdb_t* db, uint32_t addr, const uint32_t *buf, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;
  size_t align_remain;

#if (FDB_WRITE_GRAN / 8 > 0)
  uint8_t align_data[FDB_WRITE_GRAN / 8];
  size_t align_data_size = sizeof(align_data);
#else
  uint8_t align_data_u8, *align_data = &align_data_u8;  // For compatibility with C89
  size_t align_data_size = 1;
#endif

  memset(align_data, 0xFF, align_data_size);
  result = iFDB_FlashWrite((fdb_db_t*)db, addr, buf, FDB_WG_ALIGN_DOWN(size));

  align_remain = size - FDB_WG_ALIGN_DOWN(size);
  if (result == FDB_NO_ERR && align_remain)
  {
    memcpy(align_data, (uint8_t *)buf + FDB_WG_ALIGN_DOWN(size), align_remain);
    result = iFDB_FlashWrite((fdb_db_t*)db, addr + FDB_WG_ALIGN_DOWN(size), (uint32_t *) align_data, align_data_size);
  }

  return result;
}

//====================================================================================
static fdb_err_t iKVDB_CreatKvBlob(fdb_kvdb_t* db, kvdb_sec_info_t* sector, const char *key, const void *value, size_t len)
{
  fdb_err_t result = FDB_NO_ERR;
  kv_hdr_data_t kv_hdr;
  bool is_full = false;
  uint32_t kv_addr = sector->empty_kv;

  if (strlen(key) > FDB_KV_NAME_MAX)
  {
    KVDB_INFO("Error: The KV name length is more than %d\n", FDB_KV_NAME_MAX);
    return FDB_KV_NAME_ERR;
  }

  memset(&kv_hdr, 0xFF, sizeof(kv_hdr_data_t));
  kv_hdr.magic = KV_MAGIC_WORD;
  kv_hdr.name_len = strlen(key);
  kv_hdr.value_len = len;
  kv_hdr.len = KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len) + FDB_WG_ALIGN(kv_hdr.value_len);

  if (kv_hdr.len > FDB_SECTOR_SIZE(db) - KV_SECTOR_HDR_DATA_SIZE)
  {
    KVDB_INFO("Error: The KV size is too big\n");
    return FDB_SAVED_FULL;
  }

  if (kv_addr != FAILED_ADDR || (kv_addr = iKVDB_NewKv(db, sector, kv_hdr.len)) != FAILED_ADDR)
  {
    size_t align_remain;

    if (result == FDB_NO_ERR)
    {
      result = iKVDB_UpdateSentorStatus(db, sector, kv_hdr.len, &is_full);  // update the sector status
    }
    if (result == FDB_NO_ERR)
    {
      uint8_t ff = 0xFF;
      kv_hdr.crc32 = iFDB_CalcCrc32(0, &kv_hdr.name_len, KV_HDR_DATA_SIZE - KV_NAME_LEN_OFFSET);  // start calculate CRC32
      kv_hdr.crc32 = iFDB_CalcCrc32(kv_hdr.crc32, key, kv_hdr.name_len);
      align_remain = FDB_WG_ALIGN(kv_hdr.name_len) - kv_hdr.name_len;
      while (align_remain--)
      {
        kv_hdr.crc32 = iFDB_CalcCrc32(kv_hdr.crc32, &ff, 1);
      }
      kv_hdr.crc32 = iFDB_CalcCrc32(kv_hdr.crc32, value, kv_hdr.value_len);
      align_remain = FDB_WG_ALIGN(kv_hdr.value_len) - kv_hdr.value_len;
      while (align_remain--)
      {
        kv_hdr.crc32 = iFDB_CalcCrc32(kv_hdr.crc32, &ff, 1);
      }
      result = iKVDB_WriteKvHdr(db, kv_addr, &kv_hdr);  // write KV header data
    }

    if (result == FDB_NO_ERR)
    {
      result = iKVDB_AlignWrite(db, kv_addr + KV_HDR_DATA_SIZE, (uint32_t *) key, kv_hdr.name_len);  // write key name

#ifdef FDB_KV_USING_CACHE
      if (!is_full)
      {
        iKVDB_UpdateSectorCache(db, sector->addr, kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len) + FDB_WG_ALIGN(kv_hdr.value_len));
      }
      iKVDB_UpdataKvCache(db, key, kv_hdr.name_len, kv_addr);
#endif
    }
    
    if (result == FDB_NO_ERR)
    {
      result = iKVDB_AlignWrite(db, kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len), value, kv_hdr.value_len);  // write value
    }

    if (result == FDB_NO_ERR)
    {
      result = iFDB_WriteStatus((fdb_db_t*)db, kv_addr, kv_hdr.status_table, FDB_KV_STATUS_NUM, FDB_KV_WRITE);  // change the KV status to KV_WRITE
    }

    if (result == FDB_NO_ERR && is_full)
    {
      KVDB_DEBUG("Trigger a GC check after created KV.\n");
      db->gc_request = true;  // trigger GC collect when current sector is full
    }
  }
  else
  {
    result = FDB_SAVED_FULL;
  }

  return result;
}

//====================================================================================
static fdb_err_t iKVDB_SetKv(fdb_kvdb_t* db, const char *key, const void *value_buf, size_t buf_len)
{
  fdb_err_t result = FDB_NO_ERR;
  bool kv_is_found = false;

  if (value_buf == NULL)
  {
    result = iKVDB_DelKv(db, key, NULL, true);
  }
  else
  {
    if (iKVDB_NewKvEx(db, &db->cur_sector, strlen(key), buf_len) == FAILED_ADDR)  // make sure the flash has enough space
    {
      return FDB_SAVED_FULL;
    }
    
    kv_is_found = iKVDB_FindKv(db, key, &db->cur_kv);
    if (kv_is_found)
    {
      result = iKVDB_DelKv(db, key, &db->cur_kv, false);  // prepare to delete the old KV
    }

    if (result == FDB_NO_ERR)
    {
      result = iKVDB_CreatKvBlob(db, &db->cur_sector, key, value_buf, buf_len);  // create the new KV
    }

    if (kv_is_found && result == FDB_NO_ERR)
    {
      result = iKVDB_DelKv(db, key, &db->cur_kv, true);  // delete the old KV
    }

    if (db->gc_request)
    {
      iKVDB_GcCollect(db);  // process the GC after set KV
    }
  }

  return result;
}

//====================================================================================
static bool iKVDB_PrintKvCb(fdb_kv_t* kv, void *arg1, void *arg2)
{
  bool value_is_str = true, print_value = false;
  size_t *using_size = arg1;
  fdb_kvdb_t* db = arg2;

  if (kv->crc_is_ok)
  {
    *using_size += kv->len;  // calculate the total using flash size
    if (kv->status == FDB_KV_WRITE)  // check KV
    {
      FDB_PRINT("%.*s=", kv->name_len, kv->name);

      if (kv->value_len < FDB_STR_KV_VALUE_MAX_SIZE )
      {
        uint8_t buf[32];
        size_t len, size;
__reload:

        for (len = 0, size = 0; len < kv->value_len; len += size)  // check the value is string
        {
          if (len + sizeof(buf) < kv->value_len)
          {
            size = sizeof(buf);
          }
          else
          {
            size = kv->value_len - len;
          }
          iFDB_FlashRead((fdb_db_t*)db, kv->addr.value + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
          if (print_value)
          {
            FDB_PRINT("%.*s", (int)size, buf);
          }
          else if (!iFDB_IsString(buf, size))
          {
            value_is_str = false;
            break;
          }
        }
      }
      else
      {
        value_is_str = false;
      }
      if (value_is_str && !print_value)
      {
        print_value = true;
        goto __reload;
      }
      else if (!value_is_str)
      {
        FDB_PRINT("blob @0x%08" PRIX32 " %" PRIu32 "bytes", kv->addr.value, kv->value_len);
      }
      FDB_PRINT("\n");
    }
  }

  return false;
}

#ifdef FDB_KV_AUTO_UPDATE
/*
 * Auto update KV to latest default when current setting version number is changed.
 */
static void iKVDB_KvAutoUpdate(fdb_kvdb_t* db)
{
  size_t saved_ver_num, setting_ver_num = db->ver_num;

  if (iKVDB_GetKv(db, VER_NUM_KV_NAME, &saved_ver_num, sizeof(size_t), NULL) > 0)
  {
    if (saved_ver_num != setting_ver_num)  // check version number
    {
      size_t i, value_len;
      KVDB_DEBUG("Update the KV from version %zu to %zu.\n", saved_ver_num, setting_ver_num);
      for (i = 0; i < db->default_kvs.num; i++)
      {
        if (!iKVDB_FindKv(db, db->default_kvs.kvs[i].key, &db->cur_kv))  // add a new KV when it's not found
        {
          // It seems to be a string when value length is 0.
          // This mechanism is for compatibility with older versions (less then V4.0).
          if (db->default_kvs.kvs[i].value_len == 0)
          {
            value_len = strlen(db->default_kvs.kvs[i].value);
          }
          else
          {
            value_len = db->default_kvs.kvs[i].value_len;
          }
          db->cur_sector.empty_kv = FAILED_ADDR;
          iKVDB_CreatKvBlob(db, &db->cur_sector, db->default_kvs.kvs[i].key, db->default_kvs.kvs[i].value, value_len);
        }
      }
    }
    else
    {
      return;  // version number not changed now return
    }
  }

  iKVDB_SetKv(db, VER_NUM_KV_NAME, &setting_ver_num, sizeof(size_t));
}
#endif // FDB_KV_AUTO_UPDATE

//====================================================================================
static bool iKVDB_CheckSecHdrCb(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  if (!sector->check_ok)
  {
    size_t *failed_count = arg1;
    fdb_kvdb_t* db = arg2;

    KVDB_DEBUG("Sector header info is incorrect. Auto format this sector (0x%08" PRIX32 ").\n", sector->addr);
    (*failed_count) ++;
    iKVDB_FormatSentor(db, sector->addr, KV_SECTOR_NOT_COMBINED);
  }

  return false;
}

//====================================================================================
static bool iKVDB_CheckAndRecoveryGcCb(kvdb_sec_info_t* sector, void *arg1, void *arg2)
{
  fdb_kvdb_t* db = arg1;

  if (sector->check_ok && sector->status.dirty == FDB_SECTOR_DIRTY_GC)
  {
    db->gc_request = true;  // make sure the GC request flag to true
    iKVDB_GcCollect(db);  // resume the GC operate
  }

  return false;
}

//====================================================================================
static bool iKVDB_CheckAndRecoveryKvCb(fdb_kv_t* kv, void *arg1, void *arg2)
{
  fdb_kvdb_t* db = arg1;

  if (kv->crc_is_ok && kv->status == FDB_KV_PRE_DELETE)  // recovery the prepare deleted KV
  {
    KVDB_INFO("Found an KV (%.*s) which has changed value failed. Now will recovery it.\n", kv->name_len, kv->name);
    if (iKVDB_MoveKv(db, kv) == FDB_NO_ERR)  // recovery the old KV
    {
      KVDB_DEBUG("Recovery the KV successful.\n");
    }
    else
    {
      KVDB_DEBUG("Warning: Moved an KV (size %" PRIu32 ") failed when recovery. Now will GC then retry.\n", kv->len);
      return true;
    }
  }
  else if (kv->status == FDB_KV_PRE_WRITE)
  {
    uint8_t status_table[KV_STATUS_TABLE_SIZE];
    // the KV has not write finish, change the status to error
    //TODO 绘制异常处理的状态装换图
    iFDB_WriteStatus((fdb_db_t*)db, kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_ERR_HDR);
    return true;
  }

  return false;
}

/***************************************************************************
 * Check and load the flash KV.
 * @return result
 **************************************************************************/
fdb_err_t iKVDB_KvLoad(fdb_kvdb_t* db)
{
  fdb_err_t result = FDB_NO_ERR;
  fdb_kv_t kv;
  kvdb_sec_info_t sector;
  size_t check_failed_count = 0;

  db->in_recovery_check = true;
  iKVDB_SentorIterator(db, &sector, FDB_SECTOR_STORE_UNUSED, &check_failed_count, db, iKVDB_CheckSecHdrCb, false);  // check all sector header
  if (check_failed_count == SECTOR_NUM)  // all sector header check failed
  {
    KVDB_INFO("All sector header is incorrect. Set it to default.\n");
    FlashDB_KvSetDefault(db);
  }

  FDB_LOCK(db);  // lock the KV cache
  iKVDB_SentorIterator(db, &sector, FDB_SECTOR_STORE_UNUSED, db, NULL, iKVDB_CheckAndRecoveryGcCb, false);  // check all sector header for recovery GC

__retry:
  iKVDB_KvIterator(db, &kv, db, NULL, iKVDB_CheckAndRecoveryKvCb);  // check all KV for recovery
  if (db->gc_request)
  {
    iKVDB_GcCollect(db);
    goto __retry;
  }

  db->in_recovery_check = false;

  FDB_UNLOCK(db);  // unlock the KV cache

  return result;
}

/***************************************************************************
 * Set a string KV. If it value is NULL, delete it.
 * If not find it in flash, then create it.
 * @param db database object
 * @param key KV name
 * @param value KV value
 * @return result
 **************************************************************************/
fdb_err_t FlashDB_KvSet(fdb_kvdb_t* db, const char *key, const char *value)
{
  fdb_blob_t blob;

  return FlashDB_KvSetBlob(db, key, FlashDB_BlobMake(&blob, value, strlen(value)));
}

/***************************************************************************
 * Get an KV value by key name.
 * @note this function is NOT supported reentrant(不可重入)
 * @note this function is DEPRECATED
 * @param db database object
 * @param key KV name
 * @return value
 **************************************************************************/
char* FlashDB_KvGet(fdb_kvdb_t* db, const char *key)
{
  static char value[FDB_STR_KV_VALUE_MAX_SIZE + 1];
  size_t get_size;
  fdb_blob_t blob;

  if ((get_size = FalshDB_KvGetBlob(db, key, FlashDB_BlobMake(&blob, value, FDB_STR_KV_VALUE_MAX_SIZE))) > 0)
  {
    if (iFDB_IsString((uint8_t *)value, get_size))  // the return value must be string
    {
      value[get_size] = '\0';
      return value;
    }
    else if (blob.saved.len > FDB_STR_KV_VALUE_MAX_SIZE)
    {
      KVDB_INFO("Warning: The default string KV value buffer length (%d) is too less (%zu).\n", FDB_STR_KV_VALUE_MAX_SIZE, blob.saved.len);
    }
    else
    {
      KVDB_INFO("Warning: The KV value isn't string. Could not be returned\n");
      return NULL;
    }
  }

  return NULL;
}

/***************************************************************************
 * Set a blob KV. If it blob value is NULL, delete it.
 * If not find it in flash, then create it.
 * @param db database object
 * @param key KV name
 * @param blob blob object
 * @return result
 **************************************************************************/
fdb_err_t FlashDB_KvSetBlob(fdb_kvdb_t* db, const char *key, fdb_blob_t* blob)
{
  fdb_err_t result = FDB_NO_ERR;

  if (!FDB_INIT_OK(db))
  {
    KVDB_INFO("Error: KV (%s) isn't initialize OK.\n", FDB_NAME(db));
    return FDB_INIT_FAILED;
  }

  FDB_LOCK(db);  // lock the KV cache
  result = iKVDB_SetKv(db, key, blob->buf, blob->size);
  FDB_UNLOCK(db);  // unlock the KV cache

  return result;
}

/****************************************************************************
 * Get a blob KV value by key name.
 * @param db database object
 * @param key KV name
 * @param blob blob object
 * @return the actually get size on successful
 ***************************************************************************/
size_t FalshDB_KvGetBlob(fdb_kvdb_t* db, const char *key, fdb_blob_t* blob)
{
  size_t read_len = 0;

  if (!FDB_INIT_OK(db))
  {
    KVDB_INFO("Error: KV (%s) isn't initialize OK.\n", FDB_NAME(db));
    return 0;
  }

  FDB_LOCK(db);  // lock the KV cache
  read_len = iKVDB_GetKv(db, key, blob->buf, blob->size, &blob->saved.len);
  FDB_UNLOCK(db);  // unlock the KV cache

  return read_len;
}

/***************************************************************************
 * Delete an KV.
 * @param db database object
 * @param key KV name
 * @return result
 **************************************************************************/
fdb_err_t FlashDB_KvDel(fdb_kvdb_t* db, const char *key)
{
  fdb_err_t result = FDB_NO_ERR;

  if (!FDB_INIT_OK(db))
  {
    KVDB_INFO("Error: KV (%s) isn't initialize OK.\n", FDB_NAME(db));
    return FDB_INIT_FAILED;
  }

  FDB_LOCK(db);  // lock the KV cache
  result = iKVDB_DelKv(db, key, NULL, true);
  FDB_UNLOCK(db);  // unlock the KV cache

  return result;
}

/****************************************************************************
 * Get a KV object by key name
 * @param db database object
 * @param key KV name
 * @param kv KV object
 * @return KV object when is not NULL
 ***************************************************************************/
fdb_kv_t* FlashDB_KvGetObj(fdb_kvdb_t* db, const char *key, fdb_kv_t* kv)
{
  bool find_ok = false;

  if (!FDB_INIT_OK(db))
  {
    KVDB_INFO("Error: KV (%s) isn't initialize OK.\n", FDB_NAME(db));
    return 0;
  }

  FDB_LOCK(db);  // lock the KV cache
  find_ok = iKVDB_FindKv(db, key, kv);
  FDB_UNLOCK(db);  // unlock the KV cache

  return find_ok ? kv : NULL;
}

/****************************************************************************
 * Convert the KV object to blob object
 * @param kv KV object
 * @param blob blob object
 * @return new blob object
 ***************************************************************************/
fdb_blob_t* FlashDB_KvToBlob(fdb_kv_t* kv, fdb_blob_t* blob)
{
  blob->saved.meta_addr = kv->addr.start;
  blob->saved.addr = kv->addr.value;
  blob->saved.len = kv->value_len;

  return blob;
}

/***************************************************************************
 * recovery all KV to default.
 * @param db database object
 * @return result
 **************************************************************************/
fdb_err_t FlashDB_KvSetDefault(fdb_kvdb_t* db)
{
  fdb_err_t result = FDB_NO_ERR;
  uint32_t addr, i, value_len;
  kvdb_sec_info_t sector;

  FDB_LOCK(db);  // lock the KV cache
  for (addr = 0; addr < FDB_MAX_SIZE(db); addr += FDB_SECTOR_SIZE(db))
  {
    result = iKVDB_FormatSentor(db, addr, KV_SECTOR_NOT_COMBINED);  // format all sectors
    if (result != FDB_NO_ERR)
    {
      goto __exit;
    }
  }
  
  for (i = 0; i < db->default_kvs.num; i++)  // create default KV
  {
    // It seems to be a string when value length is 0.
    // This mechanism is for compatibility with older versions (less then V4.0).
    if (db->default_kvs.kvs[i].value_len == 0)
    {
      value_len = strlen(db->default_kvs.kvs[i].value);
    }
    else
    {
      value_len = db->default_kvs.kvs[i].value_len;
    }
    sector.empty_kv = FAILED_ADDR;
    iKVDB_CreatKvBlob(db, &sector, db->default_kvs.kvs[i].key, db->default_kvs.kvs[i].value, value_len);
    if (result != FDB_NO_ERR)
    {
      goto __exit;
    }
  }

__exit:
  FDB_UNLOCK(db);  // unlock the KV cache

  return result;
}

/***************************************************************************
 * Print all KV.
 * @param db database object
 **************************************************************************/
void FlashDB_KvPrint(fdb_kvdb_t* db)
{
  fdb_kv_t kv;
  size_t using_size = 0;

  if (!FDB_INIT_OK(db))
  {
    KVDB_INFO("Error: KV (%s) isn't initialize OK.\n", FDB_NAME(db));
    return;
  }

  FDB_LOCK(db);  // lock the KV cache
  iKVDB_KvIterator(db, &kv, &using_size, db, iKVDB_PrintKvCb);
  FDB_PRINT("\nmode: next generation\n");
  FDB_PRINT("size: %zu/%zu bytes.\n", using_size + (size_t)((SECTOR_NUM - FDB_GC_EMPTY_SEC_THRESHOLD) * KV_SECTOR_HDR_DATA_SIZE),
            (size_t)(FDB_MAX_SIZE(db) - FDB_SECTOR_SIZE(db) * FDB_GC_EMPTY_SEC_THRESHOLD));
  FDB_UNLOCK(db);  // unlock the KV cache
}

/***************************************************************************
 * This function will get or set some options of the database
 * @param db database object
 * @param cmd the control command
 * @param arg the argument
 **************************************************************************/
void FlashDB_KvdbControl(fdb_kvdb_t* db, int cmd, void *arg)
{
  FDB_ASSERT(db);

  switch (cmd)
  {
  case FDB_KVDB_CTRL_SET_SEC_SIZE:
    FDB_ASSERT(db->parent.init_ok == false);  // this change MUST before database initialization
    db->parent.sec_size = *(uint32_t *)arg;
    break;
    
  case FDB_KVDB_CTRL_GET_SEC_SIZE:
    *(uint32_t *)arg = db->parent.sec_size;
    break;
  
  case FDB_KVDB_CTRL_SET_LOCK:
    db->parent.lock = (void (*)(fdb_db_t* db))arg;
    break;
  
  case FDB_KVDB_CTRL_SET_UNLOCK:
    db->parent.unlock = (void (*)(fdb_db_t* db))arg;
    break;
  
  case FDB_KVDB_CTRL_SET_FILE_MODE:
#ifdef FDB_USING_FILE_MODE
    FDB_ASSERT(db->parent.init_ok == false);  // this change MUST before database initialization
    db->parent.file_mode = *(bool *)arg;
#else
    KVDB_INFO("Error: set file mode Failed. Please defined the FDB_USING_FILE_MODE macro.");
#endif
    break;

  case FDB_KVDB_CTRL_SET_MAX_SIZE:
#ifdef FDB_USING_FILE_MODE
    FDB_ASSERT(db->parent.init_ok == false);  // this change MUST before database initialization
    db->parent.max_size = *(uint32_t *)arg;
#endif
    break;
  }
}

/***************************************************************************
 * The KV database initialization.
 * @param db database object
 * @param name database name
 * @param part_name partition name
 * @param default_kv the default KV set @see fdb_default_kv
 * @param user_data user data
 * @return result
 **************************************************************************/
fdb_err_t FlashDB_KvdbInit(fdb_kvdb_t* db, const char *name, const char *part_name, fdb_default_kv_t *default_kv, void *user_data)
{
  fdb_err_t result = FDB_NO_ERR;

#ifdef FDB_KV_USING_CACHE
  size_t i;
#endif

  FDB_ASSERT(default_kv);
  FDB_ASSERT((FDB_STR_KV_VALUE_MAX_SIZE * 8) % FDB_WRITE_GRAN == 0);  // must be aligned with write granularity

  result = iFDB_InitEx((fdb_db_t*)db, name, part_name, FDB_DB_TYPE_KV, user_data);
  if (result != FDB_NO_ERR)
  {
    goto __exit;
  }

  db->gc_request = false;
  db->in_recovery_check = false;
  db->last_is_complete_del = false;
  db->default_kvs = *default_kv;
  FDB_ASSERT((FDB_GC_EMPTY_SEC_THRESHOLD > 0 && FDB_GC_EMPTY_SEC_THRESHOLD < SECTOR_NUM));  // there is at least one empty sector for GC.

#ifdef FDB_KV_USING_CACHE
  for (i = 0; i < FDB_SECTOR_CACHE_TABLE_SIZE; i++)
  {
    db->sector_cache_table[i].addr = FDB_DATA_UNUSED;
  }
  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    db->kv_cache_table[i].addr = FDB_DATA_UNUSED;
  }
#endif

  KVDB_DEBUG("KVDB size is %u bytes.\n", FDB_MAX_SIZE(db));

  result = iKVDB_KvLoad(db);

#ifdef FDB_KV_AUTO_UPDATE
  if (result == FDB_NO_ERR)
  {
    iKVDB_KvAutoUpdate(db);
  }
#endif

__exit:

  iFDB_InitFinish((fdb_db_t*)db, result);

  return result;
}

/***************************************************************************
 * The KV database initialization.
 * @param itr iterator structure to be initialized
 * @return pointer to the iterator initialized.
 **************************************************************************/
fdb_kv_iterator_t* FlashDB_KvIteratorInit(fdb_kv_iterator_t* itr)
{
  itr->curr_kv.addr.start = 0;

  // If iterator statistics is needed
  itr->iterated_cnt = 0;
  itr->iterated_obj_bytes = 0;
  itr->iterated_value_bytes = 0;
  
  // Start from sector head
  itr->sector_addr = 0;
  return itr;
}

/***************************************************************************
 * The KV database iterator.
 * @param db database object
 * @param itr the iterator structure
 * @return false if iteration is ended, true if iteration is not ended.
 **************************************************************************/
bool FlashDB_KvIterate(fdb_kvdb_t* db, fdb_kv_iterator_t* itr)
{
  kvdb_sec_info_t sector;
  fdb_kv_t* kv = &(itr->curr_kv);
  do
  {
    if (iKVDB_ReadSectorInfo(db, itr->sector_addr, &sector, false) == FDB_NO_ERR)
    {
      if (sector.status.store == FDB_SECTOR_STORE_USING || sector.status.store == FDB_SECTOR_STORE_FULL)
      {
        if (kv->addr.start == 0)
        {
          kv->addr.start = sector.addr + KV_SECTOR_HDR_DATA_SIZE;
        }
        else if ((kv->addr.start = iKVDB_GetNextKvAddr(db, &sector, kv)) == FAILED_ADDR)
        {
          kv->addr.start = 0;
          continue;
        }
        do
        {
          iKVDB_ReadKv(db, kv);
          if (kv->status == FDB_KV_WRITE)
          {
            // We got a valid kv here.If iterator statistics is needed
            itr->iterated_cnt++;
            itr->iterated_obj_bytes += kv->len;
            itr->iterated_value_bytes += kv->value_len;
            return true;
          }
        } while ((kv->addr.start = iKVDB_GetNextKvAddr(db, &sector, kv)) != FAILED_ADDR);
      }
    }
    // Set kv->addr.start to 0 when we get into a new sector so that if we successfully get the next sector info,
    // the kv->addr.start is set to the new sector.addr + KV_SECTOR_HDR_DATA_SIZE.
    kv->addr.start = 0;
  } while ((itr->sector_addr = iKVDB_GetNextSectorAddr(db, &sector)) != FAILED_ADDR);
  // Finally we have iterated all the KVs.
  return false;
}

#endif /* defined(FDB_USING_KVDB) */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TSDB feature: Time series log (like TSDB) feature implement source file
// TSL is time series log, the TSDB saved many TSLs.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined(FDB_USING_TSDB)

//====================================================================================
static fdb_err_t iTSDB_ReadTsl(fdb_tsdb_t* db, fdb_tsl_t* tsl)
{
  struct ts_log_idx_data idx;
  
  // read TSL index raw data
  iFDB_FlashRead((fdb_db_t*)db, tsl->addr.index, (uint32_t *) &idx, sizeof(struct ts_log_idx_data));
  tsl->status = (fdb_tsl_status_t) iFDB_GetStatus(idx.status_table, FDB_TSL_STATUS_NUM);
  if (tsl->status == FDB_TSL_PRE_WRITE)
  {
    tsl->log_len = db->max_len;
    tsl->addr.log = FDB_DATA_UNUSED;
    tsl->time = 0;
  }
  else
  {
    tsl->log_len = idx.log_len;
    tsl->addr.log = idx.log_addr;
    tsl->time = idx.time;
  }

  return FDB_NO_ERR;
}

//====================================================================================
static uint32_t iTSDB_GetNextSectorAddr(fdb_tsdb_t* db, tsdb_sec_info_t* pre_sec, uint32_t traversed_len)
{
  if (traversed_len + FDB_SECTOR_SIZE(db) <= FDB_MAX_SIZE(db))
  {
    if (pre_sec->addr + FDB_SECTOR_SIZE(db) < FDB_MAX_SIZE(db))
    {
      return pre_sec->addr + FDB_SECTOR_SIZE(db);
    }
    else
    {
      return 0; // the next sector is on the top of the partition
    }
  }
  else
  {
    return FAILED_ADDR; // finished
  }
}

//====================================================================================
static uint32_t iTSDB_GetNextTslAddr(tsdb_sec_info_t* sector, fdb_tsl_t* pre_tsl)
{
  uint32_t addr = FAILED_ADDR;

  if (sector->status == FDB_SECTOR_STORE_EMPTY)
  {
    return FAILED_ADDR;
  }

  if (pre_tsl->addr.index + TS_LOG_IDX_DATA_SIZE <= sector->end_idx)
  {
    addr = pre_tsl->addr.index + TS_LOG_IDX_DATA_SIZE;
  }
  else
  {
    return FAILED_ADDR; // no TSL
  }

  return addr;
}

//====================================================================================
static fdb_err_t iTSDB_ReadSectorInfo(fdb_tsdb_t* db, uint32_t addr, tsdb_sec_info_t* sector, bool traversal)
{
  fdb_err_t result = FDB_NO_ERR;
  struct ts_sector_hdr_data sec_hdr;

  FDB_ASSERT(sector);

  iFDB_FlashRead((fdb_db_t*)db, addr, (uint32_t *)&sec_hdr, sizeof(struct ts_sector_hdr_data)); // read sector header raw data

  sector->addr = addr;
  sector->magic = sec_hdr.magic;
  if (sector->magic != TS_SECTOR_MAGIC_WORD) // check magic word
  {
    sector->check_ok = false;
    return FDB_INIT_FAILED;
  }
  sector->check_ok = true;
  sector->status = (fdb_sector_store_status_t) iFDB_GetStatus(sec_hdr.status, FDB_SECTOR_STORE_STATUS_NUM);
  sector->start_time = sec_hdr.start_time;
  sector->end_info_stat[0] = (fdb_tsl_status_t) iFDB_GetStatus(sec_hdr.end_info[0].status, FDB_TSL_STATUS_NUM);
  sector->end_info_stat[1] = (fdb_tsl_status_t) iFDB_GetStatus(sec_hdr.end_info[1].status, FDB_TSL_STATUS_NUM);
  if (sector->end_info_stat[0] == FDB_TSL_WRITE)
  {
    sector->end_time = sec_hdr.end_info[0].time;
    sector->end_idx = sec_hdr.end_info[0].index;
  }
  else if (sector->end_info_stat[1] == FDB_TSL_WRITE)
  {
    sector->end_time = sec_hdr.end_info[1].time;
    sector->end_idx = sec_hdr.end_info[1].index;
  }
  else if (sector->end_info_stat[0] == FDB_TSL_PRE_WRITE && sector->end_info_stat[1] == FDB_TSL_PRE_WRITE)
  {
    //TODO There is no valid end node info on this sector, need impl fast query this sector by FlashDB_TslIterByTime
    FDB_ASSERT(0);
  }
  // traversal all TSL and calculate the remain space size
  sector->empty_idx = sector->addr + TS_SECTOR_HDR_DATA_SIZE;
  sector->empty_data = sector->addr + FDB_SECTOR_SIZE(db);
  // the TSL's data is saved from sector bottom, and the TSL's index saved from the sector top
  sector->remain = sector->empty_data - sector->empty_idx;
  if (sector->status == FDB_SECTOR_STORE_USING && traversal)
  {
    struct fdb_tsl tsl;

    tsl.addr.index = sector->empty_idx;
    while (iTSDB_ReadTsl(db, &tsl) == FDB_NO_ERR)
    {
      if (tsl.status == FDB_TSL_UNUSED)
      {
        break;
      }
      sector->end_time = tsl.time;
      sector->end_idx = tsl.addr.index;
      sector->empty_idx += TS_LOG_IDX_DATA_SIZE;
      sector->empty_data -= FDB_WG_ALIGN(tsl.log_len);
      tsl.addr.index += TS_LOG_IDX_DATA_SIZE;
      if (sector->remain > TS_LOG_IDX_DATA_SIZE + FDB_WG_ALIGN(tsl.log_len))
      {
        sector->remain -= (TS_LOG_IDX_DATA_SIZE + FDB_WG_ALIGN(tsl.log_len));
      }
      else
      {
        TSDB_INFO("Error: this TSL (0x%08" PRIX32 ") size (%" PRIu32 ") is out of bound.\n", tsl.addr.index, tsl.log_len);
        sector->remain = 0;
        result = FDB_READ_ERR;
        break;
      }
    }
  }

  return result;
}

//====================================================================================
static fdb_err_t iTSDB_FormatSector(fdb_tsdb_t* db, uint32_t addr)
{
  fdb_err_t result = FDB_NO_ERR;
  struct ts_sector_hdr_data sec_hdr;

  FDB_ASSERT(addr % FDB_SECTOR_SIZE(db) == 0);

  result = iFDB_FlashErase((fdb_db_t*)db, addr, FDB_SECTOR_SIZE(db));
  if (result == FDB_NO_ERR)
  {
    TSDB_WRITE_STATUS(db, addr, sec_hdr.status, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_EMPTY);
    sec_hdr.magic = TS_SECTOR_MAGIC_WORD; // set the magic
    TSDB_FLASH_WRITE(db, addr + TS_SECTOR_MAGIC_OFFSET, &sec_hdr.magic, sizeof(sec_hdr.magic));
  }

  return result;
}

//====================================================================================
static void iTSDB_SectorIterator(fdb_tsdb_t* db, tsdb_sec_info_t* sector, fdb_sector_store_status_t status, void *arg1, void *arg2, bool (*callback)(tsdb_sec_info_t* sector, void *arg1, void *arg2), bool traversal)
{
  uint32_t sec_addr = sector->addr, traversed_len = 0;

  do // search all sectors
  {
    iTSDB_ReadSectorInfo(db, sec_addr, sector, false);
    if (status == FDB_SECTOR_STORE_UNUSED || status == sector->status)
    {
      if (traversal)
      {
        iTSDB_ReadSectorInfo(db, sec_addr, sector, true);
      }
      // iterator is interrupted when callback return true
      if (callback && callback(sector, arg1, arg2))
      {
        return;
      }
    }
    traversed_len += FDB_SECTOR_SIZE(db);
  } while ((sec_addr = iTSDB_GetNextSectorAddr(db, sector, traversed_len)) != FAILED_ADDR);
}

//====================================================================================
static fdb_err_t iTSDB_WriteTsl(fdb_tsdb_t* db, fdb_blob_t* blob, fdb_time_t time)
{
  fdb_err_t result = FDB_NO_ERR;
  struct ts_log_idx_data idx;
  uint32_t idx_addr = db->cur_sec.empty_idx;

  idx.log_len = blob->size;
  idx.time = time;
  idx.log_addr = db->cur_sec.empty_data - FDB_WG_ALIGN(idx.log_len);
  TSDB_WRITE_STATUS(db, idx_addr, idx.status_table, FDB_TSL_STATUS_NUM, FDB_TSL_PRE_WRITE); // write the status will by write granularity
  TSDB_FLASH_WRITE(db, idx_addr + TS_LOG_IDX_OFFSET, &idx.time,  sizeof(struct ts_log_idx_data) - TS_LOG_IDX_OFFSET); // write other index info
  TSDB_FLASH_WRITE(db, idx.log_addr, blob->buf, blob->size); // write blob data
  TSDB_WRITE_STATUS(db, idx_addr, idx.status_table, FDB_TSL_STATUS_NUM, FDB_TSL_WRITE); // write the status will by write granularity

  return result;
}

//====================================================================================
static fdb_err_t iTSDB_UpdateSecStatus(fdb_tsdb_t* db, tsdb_sec_info_t* sector, fdb_blob_t* blob, fdb_time_t cur_time)
{
  fdb_err_t result = FDB_NO_ERR;
  uint8_t status[FDB_STORE_STATUS_TABLE_SIZE];

  if (sector->status == FDB_SECTOR_STORE_USING && sector->remain < TS_LOG_IDX_DATA_SIZE + FDB_WG_ALIGN(blob->size))
  {
    if (db->rollover)
    {
      uint8_t end_status[TSL_STATUS_TABLE_SIZE];
      uint32_t end_index = sector->empty_idx - TS_LOG_IDX_DATA_SIZE, new_sec_addr, cur_sec_addr = sector->addr;
      // save the end node index and timestamp
      if (sector->end_info_stat[0] == FDB_TSL_UNUSED)
      {
        TSDB_WRITE_STATUS(db, cur_sec_addr + TS_SECTOR_END0_STATUS_OFFSET, end_status, FDB_TSL_STATUS_NUM, FDB_TSL_PRE_WRITE);
        TSDB_FLASH_WRITE(db, cur_sec_addr + TS_SECTOR_END0_TIME_OFFSET, (uint32_t * )&db->last_time, sizeof(fdb_time_t));
        TSDB_FLASH_WRITE(db, cur_sec_addr + TS_SECTOR_END0_IDX_OFFSET, &end_index, sizeof(end_index));
        TSDB_WRITE_STATUS(db, cur_sec_addr + TS_SECTOR_END0_STATUS_OFFSET, end_status, FDB_TSL_STATUS_NUM, FDB_TSL_WRITE);
      }
      else if (sector->end_info_stat[1] == FDB_TSL_UNUSED)
      {
        TSDB_WRITE_STATUS(db, cur_sec_addr + TS_SECTOR_END1_STATUS_OFFSET, end_status, FDB_TSL_STATUS_NUM, FDB_TSL_PRE_WRITE);
        TSDB_FLASH_WRITE(db, cur_sec_addr + TS_SECTOR_END1_TIME_OFFSET, (uint32_t * )&db->last_time, sizeof(fdb_time_t));
        TSDB_FLASH_WRITE(db, cur_sec_addr + TS_SECTOR_END1_IDX_OFFSET, &end_index, sizeof(end_index));
        TSDB_WRITE_STATUS(db, cur_sec_addr + TS_SECTOR_END1_STATUS_OFFSET, end_status, FDB_TSL_STATUS_NUM, FDB_TSL_WRITE);
      }
      TSDB_WRITE_STATUS(db, cur_sec_addr, status, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_FULL); // change current sector to full
      if (sector->addr + FDB_SECTOR_SIZE(db) < FDB_MAX_SIZE(db))
      {
        new_sec_addr = sector->addr + FDB_SECTOR_SIZE(db); // calculate next sector address
      }
      else
      {
        new_sec_addr = 0;
      }
      iTSDB_ReadSectorInfo(db, new_sec_addr, &db->cur_sec, false);
      if (sector->status != FDB_SECTOR_STORE_EMPTY)
      {
        // calculate the oldest sector address
        if (new_sec_addr + FDB_SECTOR_SIZE(db) < FDB_MAX_SIZE(db))
        {
          db->oldest_addr = new_sec_addr + FDB_SECTOR_SIZE(db);
        }
        else
        {
          db->oldest_addr = 0;
        }
        iTSDB_FormatSector(db, new_sec_addr);
        iTSDB_ReadSectorInfo(db, new_sec_addr, &db->cur_sec, false);
      }
    }
    else
    {
      return FDB_SAVED_FULL; // not rollover
    }
  }

  if (sector->status == FDB_SECTOR_STORE_EMPTY)
  {
    sector->status = FDB_SECTOR_STORE_USING; // change the sector to using
    sector->start_time = cur_time;
    TSDB_WRITE_STATUS(db, sector->addr, status, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_USING);
    TSDB_FLASH_WRITE(db, sector->addr + TS_SECTOR_START_TIME_OFFSET, (uint32_t *)&cur_time, sizeof(fdb_time_t)); // save the start timestamp
  }

  return result;
}

//====================================================================================
static fdb_err_t iTSDB_TslAppend(fdb_tsdb_t* db, fdb_blob_t* blob)
{
  fdb_err_t result = FDB_NO_ERR;
  fdb_time_t cur_time = db->get_time();

  FDB_ASSERT(blob->size <= db->max_len);

  result = iTSDB_UpdateSecStatus(db, &db->cur_sec, blob, cur_time);
  if (result != FDB_NO_ERR)
  {
    return result;
  }

  result = iTSDB_WriteTsl(db, blob, cur_time); // write the TSL node
  if (result != FDB_NO_ERR)
  {
    return result;
  }

  // recalculate the current using sector info
  db->cur_sec.end_idx = db->cur_sec.empty_idx;
  db->cur_sec.end_time = cur_time;
  db->cur_sec.empty_idx += TS_LOG_IDX_DATA_SIZE;
  db->cur_sec.empty_data -= FDB_WG_ALIGN(blob->size);
  db->cur_sec.remain -= TS_LOG_IDX_DATA_SIZE + FDB_WG_ALIGN(blob->size);

  if (cur_time >= db->last_time)
  {
    db->last_time = cur_time;
  }
  else
  {
    TSDB_INFO("Warning: current timestamp (%" PRIdMAX ") is less than the last save timestamp (%" PRIdMAX ")\n", (intmax_t)cur_time, (intmax_t)(db->last_time));
  }

  return result;
}

//====================================================================================
static bool iTSDB_QueryCountCb(fdb_tsl_t* tsl, void *arg)
{
  struct ts_query_count_args *args = arg;

  if (tsl->status == args->status)
  {
    args->count++;
  }

  return false;
}

//====================================================================================
static bool iTSDB_CheckSecHdrCb(tsdb_sec_info_t* sector, void *arg1, void *arg2)
{
  struct ts_check_sec_hdr_cb_args *arg = arg1;
  fdb_tsdb_t* db = arg->db;

  if (!sector->check_ok)
  {
    TSDB_INFO("Sector (0x%08" PRIu32 ") header info is incorrect.\n", sector->addr);
    (arg->check_failed) = true;
    return true;
  }
  else if (sector->status == FDB_SECTOR_STORE_USING)
  {
    if (db->cur_sec.addr == FDB_DATA_UNUSED)
    {
      memcpy(&db->cur_sec, sector, sizeof(struct tsdb_sec_info));
    }
    else
    {
      TSDB_INFO("Warning: Sector status is wrong, there are multiple sectors in use.\n");
      (arg->check_failed) = true;
      return true;
    }
  }
  else if (sector->status == FDB_SECTOR_STORE_EMPTY)
  {
    (arg->empty_num) += 1;
    arg->empty_addr = sector->addr;
    if ((arg->empty_num) == 1 && db->cur_sec.addr == FDB_DATA_UNUSED)
    {
      memcpy(&db->cur_sec, sector, sizeof(struct tsdb_sec_info));
    }
  }

  return false;
}

//====================================================================================
static bool iTSDB_FormatAllCb(tsdb_sec_info_t* sector, void *arg1, void *arg2)
{
  fdb_tsdb_t* db = arg1;

  iTSDB_FormatSector(db, sector->addr);

  return false;
}

//====================================================================================
static void iTSDB_TslFormatAll(fdb_tsdb_t* db)
{
  struct tsdb_sec_info sector;

  sector.addr = 0;
  iTSDB_SectorIterator(db, &sector, FDB_SECTOR_STORE_UNUSED, db, NULL, iTSDB_FormatAllCb, false);
  db->oldest_addr = 0;
  db->cur_sec.addr = 0;
  db->last_time = 0;
  iTSDB_ReadSectorInfo(db, db->cur_sec.addr, &db->cur_sec, false); // read the current using sector info

  TSDB_INFO("All sector format finished.\n");
}

/****************************************************************************
 * Append a new log to TSDB.
 * @param db database object
 * @param blob log blob data
 * @return result
 ***************************************************************************/
fdb_err_t FlashDB_TslAppend(fdb_tsdb_t* db, fdb_blob_t* blob)
{
  fdb_err_t result = FDB_NO_ERR;

  if (!FDB_INIT_OK(db))
  {
    TSDB_INFO("Error: TSL (%s) isn't initialize OK.\n", FDB_NAME(db));
    return FDB_INIT_FAILED;
  }

  FDB_LOCK(db);
  result = iTSDB_TslAppend(db, blob);
  FDB_UNLOCK(db);

  return result;
}

/****************************************************************************
 * The TSDB iterator for each TSL.
 * @param db database object
 * @param cb callback
 * @param arg callback argument
 ***************************************************************************/
void FlashDB_TslIter(fdb_tsdb_t* db, fdb_tsl_cb cb, void *arg)
{
  struct tsdb_sec_info sector;
  uint32_t sec_addr, traversed_len = 0;
  struct fdb_tsl tsl;

  if (!FDB_INIT_OK(db))
  {
    TSDB_INFO("Error: TSL (%s) isn't initialize OK.\n", FDB_NAME(db));
  }

  if (cb == NULL)
  {
    return;
  }

  sec_addr = db->oldest_addr;
  do // search all sectors
  {
    traversed_len += FDB_SECTOR_SIZE(db);
    if (iTSDB_ReadSectorInfo(db, sec_addr, &sector, false) != FDB_NO_ERR)
    {
      continue;
    }

    if (sector.status == FDB_SECTOR_STORE_USING || sector.status == FDB_SECTOR_STORE_FULL) // sector has TSL
    {
      if (sector.status == FDB_SECTOR_STORE_USING)
      {
        sector = db->cur_sec; // copy the current using sector status
      }
      tsl.addr.index = sector.addr + TS_SECTOR_HDR_DATA_SIZE;

      do // search all TSL
      {
        iTSDB_ReadTsl(db, &tsl);
        // iterator is interrupted when callback return true
        if (cb(&tsl, arg))
        {
          return;
        }
      } while ((tsl.addr.index = iTSDB_GetNextTslAddr(&sector, &tsl)) != FAILED_ADDR);
    }
  } while ((sec_addr = iTSDB_GetNextSectorAddr(db, &sector, traversed_len)) != FAILED_ADDR);
}

/****************************************************************************
 * The TSDB iterator for each TSL by timestamp.
 * @param db database object
 * @param from starting timestap
 * @param to ending timestap
 * @param cb callback
 * @param arg callback argument
 ***************************************************************************/
void FlashDB_TslIterByTime(fdb_tsdb_t* db, fdb_time_t from, fdb_time_t to, fdb_tsl_cb cb, void *cb_arg)
{
  struct tsdb_sec_info sector;
  uint32_t sec_addr, oldest_addr = db->oldest_addr, traversed_len = 0;
  struct fdb_tsl tsl;
  bool found_start_tsl = false;

  if (!FDB_INIT_OK(db))
  {
    TSDB_INFO("Error: TSL (%s) isn't initialize OK.\n", FDB_NAME(db));
  }

  if (cb == NULL)
  {
    return;
  }

  sec_addr = oldest_addr;
  do // search all sectors
  {
    traversed_len += FDB_SECTOR_SIZE(db);
    if (iTSDB_ReadSectorInfo(db, sec_addr, &sector, false) != FDB_NO_ERR)
    {
      continue;
    }

    if ((sector.status == FDB_SECTOR_STORE_USING || sector.status == FDB_SECTOR_STORE_FULL)) // sector has TSL 
    {
      if (sector.status == FDB_SECTOR_STORE_USING)
      {
        sector = db->cur_sec; // copy the current using sector status
      }
      
      if ((found_start_tsl) || (!found_start_tsl && ((from >= sector.start_time && from <= sector.end_time) || (sec_addr == oldest_addr && from <= sector.start_time))))
      {
        uint32_t start = sector.addr + TS_SECTOR_HDR_DATA_SIZE, end = sector.end_idx;

        found_start_tsl = true;
        while (start <= end) // search start TSL address, using binary search algorithm
        {
          tsl.addr.index = start + ((end - start) / 2 + 1) / TS_LOG_IDX_DATA_SIZE * TS_LOG_IDX_DATA_SIZE;
          iTSDB_ReadTsl(db, &tsl);
          if (tsl.time < from)
          {
            start = tsl.addr.index + TS_LOG_IDX_DATA_SIZE;
          }
          else
          {
            end = tsl.addr.index - TS_LOG_IDX_DATA_SIZE;
          }
        }
        tsl.addr.index = start;
        do // search all TSL
        {
          iTSDB_ReadTsl(db, &tsl);
          if (tsl.time >= from && tsl.time <= to)
          {
            // iterator is interrupted when callback return true
            if (cb(&tsl, cb_arg))
            {
              return;
            }
          }
          else
          {
            return;
          }
        } while ((tsl.addr.index = iTSDB_GetNextTslAddr(&sector, &tsl)) != FAILED_ADDR);
      }
    }
    else if (sector.status == FDB_SECTOR_STORE_EMPTY)
    {
      return;
    }
  } while ((sec_addr = iTSDB_GetNextSectorAddr(db, &sector, traversed_len)) != FAILED_ADDR);
}

/****************************************************************************
 * Query some TSL's count by timestamp and status.
 * @param db database object
 * @param from starting timestap
 * @param to ending timestap
 * @param status status
 ***************************************************************************/
size_t FlashDB_TslQueryCount(fdb_tsdb_t* db, fdb_time_t from, fdb_time_t to, fdb_tsl_status_t status)
{
  struct ts_query_count_args arg = { FDB_TSL_UNUSED, 0 };

  arg.status = status;
  if (!FDB_INIT_OK(db))
  {
    TSDB_INFO("Error: TSL (%s) isn't initialize OK.\n", FDB_NAME(db));
    return FDB_INIT_FAILED;
  }

  FlashDB_TslIterByTime(db, from, to, iTSDB_QueryCountCb, &arg);

  return arg.count;
}

/****************************************************************************
 * Set the TSL status.
 * @param db database object
 * @param tsl TSL object
 * @param status status
 * @return result
 ***************************************************************************/
fdb_err_t FlashDB_TslSetStatus(fdb_tsdb_t* db, fdb_tsl_t* tsl, fdb_tsl_status_t status)
{
  fdb_err_t result = FDB_NO_ERR;
  uint8_t status_table[TSL_STATUS_TABLE_SIZE];

  // write the status will by write granularity
  TSDB_WRITE_STATUS(db, tsl->addr.index, status_table, FDB_TSL_STATUS_NUM, status);

  return result;
}

/****************************************************************************
 * Convert the TSL object to blob object
 * @param tsl TSL object
 * @param blob blob object
 * @return new blob object
 ***************************************************************************/
fdb_blob_t* FlashDB_TslToBlob(fdb_tsl_t* tsl, fdb_blob_t* blob)
{
  blob->saved.addr = tsl->addr.log;
  blob->saved.meta_addr = tsl->addr.index;
  blob->saved.len = tsl->log_len;

  return blob;
}

/****************************************************************************
 * Clean all the data in the TSDB.
 * @note It's DANGEROUS. This operation is not reversible.
 * @param db database object
 ***************************************************************************/
void FlashDB_TslClean(fdb_tsdb_t* db)
{
  FDB_LOCK(db);
  iTSDB_TslFormatAll(db);
  FDB_UNLOCK(db);
}

/****************************************************************************
 * This function will get or set some options of the database
 * @param db database object
 * @param cmd the control command
 * @param arg the argument
 ***************************************************************************/
void FlashDB_TsdbControl(fdb_tsdb_t* db, int cmd, void *arg)
{
  FDB_ASSERT(db);

  switch (cmd)
  {
  case FDB_TSDB_CTRL_SET_SEC_SIZE:
    FDB_ASSERT(db->parent.init_ok == false); // this change MUST before database initialization
    db->parent.sec_size = *(uint32_t *)arg;
    break;
    
  case FDB_TSDB_CTRL_GET_SEC_SIZE:
    *(uint32_t *)arg = db->parent.sec_size;
    break;
    
  case FDB_TSDB_CTRL_SET_LOCK:
    db->parent.lock = (void (*)(fdb_db_t* db))arg;
    break;
    
  case FDB_TSDB_CTRL_SET_UNLOCK:
    db->parent.unlock = (void (*)(fdb_db_t* db))arg;
    break;
    
  case FDB_TSDB_CTRL_SET_ROLLOVER:
    db->rollover = *(bool *)arg;
    break;
    
  case FDB_TSDB_CTRL_GET_ROLLOVER:
    *(bool *)arg = db->rollover;
    break;
    
  case FDB_TSDB_CTRL_GET_LAST_TIME:
    *(fdb_time_t *)arg = db->last_time;
    break;
    
  case FDB_TSDB_CTRL_SET_FILE_MODE:
#ifdef FDB_USING_FILE_MODE
    FDB_ASSERT(db->parent.init_ok == false); // this change MUST before database initialization
    db->parent.file_mode = *(bool *)arg;
#else
    TSDB_INFO("Error: set file mode Failed. Please defined the FDB_USING_FILE_MODE macro.");
#endif
    break;

  case FDB_TSDB_CTRL_SET_MAX_SIZE:
#ifdef FDB_USING_FILE_MODE
    FDB_ASSERT(db->parent.init_ok == false); // this change MUST before database initialization
    db->parent.max_size = *(uint32_t *)arg;
#endif
    break;
  }
}

/****************************************************************************
 * The time series database initialization.
 * @param db database object
 * @param name database name
 * @param part_name partition name
 * @param get_time get current time function
 * @param max_len maximum length of each log
 * @param user_data user data
 * @return result
 ***************************************************************************/
fdb_err_t FlashDB_TsdbInit(fdb_tsdb_t* db, const char *name, const char *part_name, fdb_get_time get_time, size_t max_len, void *user_data)
{
  fdb_err_t result = FDB_NO_ERR;
  struct tsdb_sec_info sector;
  struct ts_check_sec_hdr_cb_args check_sec_arg = { db, false, 0, 0};

  FDB_ASSERT(get_time);

  result = iFDB_InitEx((fdb_db_t*)db, name, part_name, FDB_DB_TYPE_TS, user_data);
  if (result != FDB_NO_ERR)
  {
    goto __exit;
  }

  db->get_time = get_time;
  db->max_len = max_len;
  db->rollover = true; // default rollover flag is true
  db->oldest_addr = FDB_DATA_UNUSED;
  db->cur_sec.addr = FDB_DATA_UNUSED;
  FDB_ASSERT(max_len < FDB_SECTOR_SIZE(db)); // must less than sector size

  sector.addr = 0;
  iTSDB_SectorIterator(db, &sector, FDB_SECTOR_STORE_UNUSED, &check_sec_arg, NULL, iTSDB_CheckSecHdrCb, true); // check all sector header
  if (check_sec_arg.check_failed)
  {
    iTSDB_TslFormatAll(db); // format all sector when check failed
  }
  else
  {
    uint32_t latest_addr;
    if (check_sec_arg.empty_num > 0)
    {
      latest_addr = check_sec_arg.empty_addr;
    }
    else
    {
      latest_addr = db->cur_sec.addr;
    }
    // db->cur_sec is the latest sector, and the next is the oldest sector
    if (latest_addr + FDB_SECTOR_SIZE(db) >= FDB_MAX_SIZE(db))
    {
      db->oldest_addr = 0; // db->cur_sec is the the bottom of the partition
    }
    else
    {
      db->oldest_addr = latest_addr + FDB_SECTOR_SIZE(db);
    }
  }
  TSDB_DEBUG("TSDB (%s) oldest sectors is 0x%08" PRIX32 ", current using sector is 0x%08" PRIX32 ".\n", FDB_NAME(db), db->oldest_addr, db->cur_sec.addr);
  iTSDB_ReadSectorInfo(db, db->cur_sec.addr, &db->cur_sec, true); // read the current using sector info
  if (db->cur_sec.status == FDB_SECTOR_STORE_USING) // get last save time
  {
    db->last_time = db->cur_sec.end_time;
  }
  else if (db->cur_sec.status == FDB_SECTOR_STORE_EMPTY && db->oldest_addr != db->cur_sec.addr)
  {
    struct tsdb_sec_info sec;
    uint32_t addr = db->cur_sec.addr;

    if (addr == 0)
    {
      addr = FDB_MAX_SIZE(db) - FDB_SECTOR_SIZE(db);
    }
    else
    {
      addr -= FDB_SECTOR_SIZE(db);
    }
    iTSDB_ReadSectorInfo(db, addr, &sec, false);
    db->last_time = sec.end_time;
  }

__exit:

  iFDB_InitFinish((fdb_db_t*)db, result);

  return result;
}

#endif /* defined(FDB_USING_TSDB) */

