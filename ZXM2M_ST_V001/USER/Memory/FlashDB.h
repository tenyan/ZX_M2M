/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: FlashDB.h
* @Engineer: armink & TenYan
* @Company:  徐工信息智能硬件部
* @Date:     2021-1-26
* @brief:    This file is part of the Flash Data Base Library.
* 主要对Flash存储实现磨损均衡、掉电保护、增量升级、数据加密等功能
******************************************************************************/
#ifndef _FLASH_DB_H_
#define _FLASH_DB_H_

//==包含头文件============================================
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <fal.h>

/******************************************************************************
 * FlashDB功能配置
 ******************************************************************************/
/// using KVDB feature
#define FDB_USING_KVDB

/// Auto update KV to latest default when current KVDB version number is changed. @see fdb_kvdb.ver_num
#ifdef FDB_USING_KVDB
#define FDB_KV_AUTO_UPDATE  
#endif

/// using TSDB (Time series database) feature
#define FDB_USING_TSDB

/// Using FAL storage mode
#define FDB_USING_FAL_MODE

/// the flash write granularity, unit: bit
#ifdef FDB_USING_FAL_MODE
#define FDB_WRITE_GRAN   1  // only support 1(nor flash)/ 8(stm32f2/f4)/ 32(stm32f1)
#endif

/// Using file storage mode by LIBC file API, like fopen/fread/fwrte/fclose
//#define FDB_USING_FILE_LIBC_MODE 

/// Using file storage mode by POSIX file API, like open/read/write/close 
//#define FDB_USING_FILE_POSIX_MODE

/// MCU Endian Configuration, default is Little Endian Order. 
//#define FDB_BIG_ENDIAN  // stm32是小段模式,不使能

/// log print macro. default EF_PRINT macro is printf()
#define FDB_PRINT(...)  PcDebug_Printf(__VA_ARGS__)

/// print debug information
#define FDB_DEBUG_ENABLE  0

/******************************************************************************
 *   Macros
 ******************************************************************************/
/// software version number
#define FDB_SW_VERSION  "1.0.99"
#define FDB_SW_VERSION_NUM  0x10099
#define FDB_KV_NAME_MAX  32  /// the KV max name length must less then it

#define FDB_KV_CACHE_TABLE_SIZE  16  /// the KV cache table size, it will improve KV search speed when using cache
#define FDB_SECTOR_CACHE_TABLE_SIZE  4  /// the sector cache table size, it will improve KV save speed when using cache

#if (FDB_KV_CACHE_TABLE_SIZE > 0) && (FDB_SECTOR_CACHE_TABLE_SIZE > 0)
#define FDB_KV_USING_CACHE
#endif

#if defined(FDB_USING_FILE_LIBC_MODE) || defined(FDB_USING_FILE_POSIX_MODE)
#define FDB_USING_FILE_MODE
#endif

#define FDB_LOG_PREFIX()  FDB_PRINT("[FlashDB]")
#define KVDB_LOG_PREFIX()  FDB_PRINT("[KVDB][%s]", FDB_NAME(db))
#define TSDB_LOG_PREFIX()  FDB_PRINT("[TSDB][%s]", FDB_NAME(db))

#ifdef FDB_DEBUG_ENABLE
#define KVDB_DEBUG(...)  KVDB_LOG_PREFIX();FDB_PRINT("(%s:%d) ", __FILE__, __LINE__);FDB_PRINT(__VA_ARGS__)
#define TSDB_DEBUG(...)  TSDB_LOG_PREFIX();FDB_PRINT("(%s:%d) ", __FILE__, __LINE__);FDB_PRINT(__VA_ARGS__)
#define FDB_DEBUG(...)  FDB_LOG_PREFIX();FDB_PRINT("(%s:%d) ", __FILE__, __LINE__);FDB_PRINT(__VA_ARGS__)
#else
#define KVDB_DEBUG(...)
#define TSDB_DEBUG(...)
#define FDB_DEBUG(...)
#endif

#define FDB_INFO(...)  do{ FDB_LOG_PREFIX();FDB_PRINT(__VA_ARGS__);}while(0) /// routine print function. Must be implement by user. 
#define KVDB_INFO(...)  do{ KVDB_LOG_PREFIX();FDB_PRINT(__VA_ARGS__);}while(0) /// routine print function. Must be implement by user. 
#define TSDB_INFO(...)  do{ TSDB_LOG_PREFIX();FDB_PRINT(__VA_ARGS__);}while(0) /// routine print function. Must be implement by user. 
#define FDB_ASSERT(EXPR)  if (!(EXPR)) { FDB_DEBUG("(%s) has assert failed at %s.\n", #EXPR, __FUNCTION__);  while (1);} /// assert for developer. 

#define FDB_KVDB_CTRL_SET_SEC_SIZE  0x0 //< set sector size control command
#define FDB_KVDB_CTRL_GET_SEC_SIZE  0x1 //< get sector size control command
#define FDB_KVDB_CTRL_SET_LOCK      0x2 //< set lock function control command
#define FDB_KVDB_CTRL_SET_UNLOCK    0x3 //< set unlock function control command
#define FDB_KVDB_CTRL_SET_FILE_MODE 0x9 //< set file mode control command
#define FDB_KVDB_CTRL_SET_MAX_SIZE  0xA //< set database max size in file mode control command

#define FDB_TSDB_CTRL_SET_SEC_SIZE  0x0 //< set sector size control command
#define FDB_TSDB_CTRL_GET_SEC_SIZE  0x1 //< get sector size control command
#define FDB_TSDB_CTRL_SET_LOCK      0x2 //< set lock function control command
#define FDB_TSDB_CTRL_SET_UNLOCK    0x3 //< set unlock function control command
#define FDB_TSDB_CTRL_SET_ROLLOVER  0x4 //< set rollover control command
#define FDB_TSDB_CTRL_GET_ROLLOVER  0x5 //< get rollover control command
#define FDB_TSDB_CTRL_GET_LAST_TIME 0x6 //< get last save time control command
#define FDB_TSDB_CTRL_SET_FILE_MODE 0x9 //< set file mode control command
#define FDB_TSDB_CTRL_SET_MAX_SIZE  0xA //< set database max size in file mode control command

/******************************************************************************
 * FDB公用数据类型 Data Types
 ******************************************************************************/
 //==数据库类型======
 typedef enum
{
  FDB_DB_TYPE_KV,
  FDB_DB_TYPE_TS,
} fdb_db_type_t;
  
//==error code======
typedef enum
{
  FDB_NO_ERR,
  FDB_ERASE_ERR,
  FDB_READ_ERR,
  FDB_WRITE_ERR,
  FDB_PART_NOT_FOUND,
  FDB_KV_NAME_ERR,
  FDB_KV_NAME_EXIST,
  FDB_SAVED_FULL,
  FDB_INIT_FAILED,
} fdb_err_t;

//==the flash sector store status======
typedef enum fdb_sector_store_status
{
  FDB_SECTOR_STORE_UNUSED,
  FDB_SECTOR_STORE_EMPTY,
  FDB_SECTOR_STORE_USING,
  FDB_SECTOR_STORE_FULL,
  FDB_SECTOR_STORE_STATUS_NUM,
}fdb_sector_store_status_t;

//==the flash sector dirty status======
typedef enum fdb_sector_dirty_status
{
  FDB_SECTOR_DIRTY_UNUSED,
  FDB_SECTOR_DIRTY_FALSE,
  FDB_SECTOR_DIRTY_TRUE,
  FDB_SECTOR_DIRTY_GC,
  FDB_SECTOR_DIRTY_STATUS_NUM,
}fdb_sector_dirty_status_t;

//==database structure======
typedef struct fdb_db
{
  const char *name;  //< database name
  fdb_db_type_t type;  //< database type
  union
  {
#ifdef FDB_USING_FAL_MODE
    const fal_partition_t *part;  //< flash partition for saving database
#endif
#ifdef FDB_USING_FILE_MODE
    const char *dir;  //< directory path for saving database
#endif
  } storage;
  uint32_t sec_size;  //< flash section size. It's a multiple of block size
  uint32_t max_size;  //< database max size. It's a multiple of section size
  bool init_ok;  //< initialized successfully
  bool file_mode;  //< is file mode, default is false
#ifdef FDB_USING_FILE_MODE
#if defined(FDB_USING_FILE_POSIX_MODE)
  int cur_file;  //< current file object
#elif defined(FDB_USING_FILE_LIBC_MODE)
  FILE *cur_file;  //< current file object
#endif
  uint32_t cur_sec;  //< current operate sector address
#endif
  void (*lock)(struct fdb_db* db);  //< lock the database operate
  void (*unlock)(struct fdb_db* db);  //< unlock the database operate
  void *user_data;
}fdb_db_t;

//==blob structure======
typedef struct fdb_blob
{
  void *buf;  //< blob data buffer
  size_t size;  //< blob data buffer size
  struct
  {
    uint32_t meta_addr;  //< saved KV or TSL index address
    uint32_t addr;  //< blob data saved address
    size_t len;  //< blob data saved length
  } saved;
}fdb_blob_t;

/******************************************************************************
 * KVDB数据库数据类型定义
 ******************************************************************************/
// kv状态
 typedef enum fdb_kv_status
{
  FDB_KV_UNUSED,
  FDB_KV_PRE_WRITE,
  FDB_KV_WRITE,
  FDB_KV_PRE_DELETE,
  FDB_KV_DELETED,
  FDB_KV_ERR_HDR,
  FDB_KV_STATUS_NUM,
}fdb_kv_status_t;

// 默认KV节点
typedef struct fdb_default_kv_node
{
  char *key;
  void *value;
  size_t value_len;
}fdb_default_kv_node_t;

// 默认KV表
typedef struct fdb_default_kv
{
  struct fdb_default_kv_node *kvs;
  size_t num;
}fdb_default_kv_t;

//==key-value node object======
typedef struct fdb_kv
{
  fdb_kv_status_t status;  //< node status, @see fdb_kv_status_t
  bool crc_is_ok;  //< node CRC32 check is OK
  uint8_t name_len;  //< name length
  uint32_t magic;  //< magic word(`K`, `V`, `4`, `0`)
  uint32_t len;  //< node total length (header + name + value), must align by FDB_WRITE_GRAN
  uint32_t value_len;  //< value length
  char name[FDB_KV_NAME_MAX];  //< name
  struct
  {
    uint32_t start; //< node start address
    uint32_t value; //< value start address
  } addr;
}fdb_kv_t;

//==迭代器structure======
typedef struct fdb_kv_iterator
{
  struct fdb_kv curr_kv;  //< Current KV we get from the iterator
  uint32_t iterated_cnt;  //< How many KVs have we iterated already
  size_t iterated_obj_bytes;  //< Total storage size of KVs we have iterated.
  size_t iterated_value_bytes;  //< Total value size of KVs we have iterated.
  uint32_t sector_addr;  //< Current sector address we're iterating. DO NOT touch it.
}fdb_kv_iterator_t;

//==KVDB section information======
typedef struct kvdb_sec_info
{
  bool check_ok;  //< sector header check is OK
  struct
  {
    fdb_sector_store_status_t store;  //< sector store status @see fdb_sector_store_status_t
    fdb_sector_dirty_status_t dirty;  //< sector dirty status @see sector_dirty_status_t
  } status;
  uint32_t addr;  //< sector start address
  uint32_t magic;  //< magic word(`E`, `F`, `4`, `0`)
  uint32_t combined;  //< the combined next sector number, 0xFFFFFFFF: not combined
  size_t remain;  //< remain size
  uint32_t empty_kv;  //< the next empty KV node start address
}kvdb_sec_info_t;

//==KV cache节点======
typedef struct kv_cache_node
{
  uint16_t name_crc;  //< KV name's CRC32 low 16bit value
  uint16_t active;  //< KV node access active degree
  uint32_t addr;  //< KV node address
}kv_cache_node_t;

//==扇区 cache节点======
typedef struct sector_cache_node
{
  uint32_t addr;  //< sector start address
  uint32_t empty_addr;  //< sector empty address
}sector_cache_node_t;

//==KVDB structure======
typedef struct fdb_kvdb
{
  struct fdb_db parent;  //< inherit from fdb_db
  struct fdb_default_kv default_kvs;  //< default KV
  bool gc_request;  //< request a GC check
  bool in_recovery_check;  //< is in recovery check status when first reboot
  struct fdb_kv cur_kv;
  struct kvdb_sec_info cur_sector;
  bool last_is_complete_del;

#ifdef FDB_KV_USING_CACHE
  struct kv_cache_node kv_cache_table[FDB_KV_CACHE_TABLE_SIZE];  // KV cache table
  // sector cache table, it caching the sector info which status is current using
  struct sector_cache_node sector_cache_table[FDB_SECTOR_CACHE_TABLE_SIZE];
#endif

#ifdef FDB_KV_AUTO_UPDATE
  uint32_t ver_num;  //< setting version number for update
#endif

  void *user_data;
}fdb_kvdb_t;

/******************************************************************************
 * TSDB数据库数据类型定义
 ******************************************************************************/
typedef unsigned int fdb_time_t;

#ifdef FDB_USING_TIMESTAMP_64BIT
typedef int64_t fdb_time_t;
#endif

typedef fdb_time_t (*fdb_get_time)(void);

//==状态定义======
typedef enum fdb_tsl_status
{
  FDB_TSL_UNUSED,
  FDB_TSL_PRE_WRITE,
  FDB_TSL_WRITE,
  FDB_TSL_USER_STATUS1,
  FDB_TSL_DELETED,
  FDB_TSL_USER_STATUS2,
  FDB_TSL_STATUS_NUM,
}fdb_tsl_status_t;

//==time series log node object
typedef struct fdb_tsl
{
  fdb_tsl_status_t status;  //< node status, @see fdb_log_status_t
  fdb_time_t time;  //< node timestamp
  uint32_t log_len;  //< log length, must align by FDB_WRITE_GRAN
  struct
  {
    uint32_t index;  //< node index address
    uint32_t log;  //< log data address
  } addr;
}fdb_tsl_t;
typedef bool (*fdb_tsl_cb)(fdb_tsl_t* tsl, void *arg);

// TSDB section information
typedef struct tsdb_sec_info
{
  bool check_ok;  //< sector header check is OK
  fdb_sector_store_status_t status;  //< sector store status @see fdb_sector_store_status_t
  uint32_t addr;  //< sector start address
  uint32_t magic;  //< magic word(`T`, `S`, `L`, `0`)
  fdb_time_t start_time;  //< the first start node's timestamp, 0xFFFFFFFF: unused
  fdb_time_t end_time;  //< the last end node's timestamp, 0xFFFFFFFF: unused
  uint32_t end_idx;  //< the last end node's index, 0xFFFFFFFF: unused
  fdb_tsl_status_t end_info_stat[2];  //< the last end node's info status
  size_t remain;  //< remain size
  uint32_t empty_idx;  //< the next empty node index address
  uint32_t empty_data;  //< the next empty node's data end address
}tsdb_sec_info_t;

// TSDB structure
typedef struct fdb_tsdb
{
  struct fdb_db parent;  //< inherit from fdb_db
  struct tsdb_sec_info cur_sec;  //< current using sector
  fdb_time_t last_time;  //< last TSL timestamp
  fdb_get_time get_time;  //< the current timestamp get function
  size_t max_len;  //< the maximum length of each log
  uint32_t oldest_addr;  //< the oldest sector start address
  bool rollover;  //< the oldest data will rollover by newest data, default is true

  void *user_data;
}fdb_tsdb_t;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
/// Blob API
fdb_blob_t* FlashDB_BlobMake(fdb_blob_t* blob, const void *value_buf, size_t buf_len); // 构造blob对象
size_t FlashDB_BlobRead(fdb_db_t* db, fdb_blob_t* blob); // 读取blob数据

/// Key-Value API like a KVDB
fdb_err_t FlashDB_KvdbInit(fdb_kvdb_t* db, const char *name, const char *part_name, fdb_default_kv_t *default_kv, void *user_data); // 初始化KVDB
void FlashDB_KvdbControl(fdb_kvdb_t* db, int cmd, void *arg); // 控制KVDB
fdb_err_t FlashDB_KvSet(fdb_kvdb_t* db, const char *key, const char *value); // 设置string类型KV
char* FlashDB_KvGet(fdb_kvdb_t* db, const char *key); // 获取字符串类型KV
fdb_err_t FlashDB_KvSetBlob(fdb_kvdb_t* db, const char *key, fdb_blob_t* blob); // 设置blob类型KV
size_t FalshDB_KvGetBlob(fdb_kvdb_t* db, const char *key, fdb_blob_t* blob); // 获取blob类型KV
fdb_err_t FlashDB_KvDel(fdb_kvdb_t* db, const char *key); // 删除KV
fdb_kv_t* FlashDB_KvGetObj(fdb_kvdb_t* db, const char *key, fdb_kv_t* kv); // 获取KV对象
fdb_blob_t* FlashDB_KvToBlob(fdb_kv_t* kv, fdb_blob_t* blob); // KV对象转换为blob对象
fdb_err_t FlashDB_KvSetDefault(fdb_kvdb_t* db); // 重置KVDB
void FlashDB_KvPrint(fdb_kvdb_t* db); // 打印KVDB中的KV信息
fdb_kv_iterator_t* FlashDB_KvIteratorInit(fdb_kv_iterator_t* itr); // 初始化KV迭代器
bool FlashDB_KvIterate(fdb_kvdb_t* db, fdb_kv_iterator_t* itr); // 迭代KV

// Time series log API like a TSDB
fdb_err_t FlashDB_TsdbInit(fdb_tsdb_t* db, const char *name, const char *part_name, fdb_get_time get_time, size_t max_len, void *user_data); // 初始化TSDB
void FlashDB_TsdbControl(fdb_tsdb_t* db, int cmd, void *arg); // 控制TSDB
fdb_err_t FlashDB_TslAppend(fdb_tsdb_t* db, fdb_blob_t* blob); // 追加TSL
void FlashDB_TslIter(fdb_tsdb_t* db, fdb_tsl_cb cb, void *cb_arg); // 迭代TSL
void FlashDB_TslIterByTime(fdb_tsdb_t* db, fdb_time_t from, fdb_time_t to, fdb_tsl_cb cb, void *cb_arg); // 按时间段迭代TSL
size_t FlashDB_TslQueryCount(fdb_tsdb_t* db, fdb_time_t from, fdb_time_t to, fdb_tsl_status_t status); //查询TSL的数量
fdb_err_t FlashDB_TslSetStatus(fdb_tsdb_t* db, fdb_tsl_t* tsl, fdb_tsl_status_t status); // 设置TSL状态
void FlashDB_TslClean(fdb_tsdb_t* db); // 清空TSDB
fdb_blob_t* FlashDB_TslToBlob(fdb_tsl_t* tsl, fdb_blob_t* blob); // TSL对象转换为blob对象

#endif /* _FLASH_DB_H_ */

