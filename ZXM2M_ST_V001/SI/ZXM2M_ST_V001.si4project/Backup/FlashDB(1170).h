/*****************************************************************************
* @FileName: FlashDB.h
* @Engineer: TenYan
* @Date:     2021-1-26
* @brief     This file is part of the Flash Data Base Library.
******************************************************************************/
#ifndef _FLASH_DB_H_
#define _FLASH_DB_H_

//==包含头文件============================================
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

/******************************************************************************
 * FlashDB Config
 ******************************************************************************/
/* using KVDB feature */
#define FDB_USING_KVDB

#ifdef FDB_USING_KVDB
/* Auto update KV to latest default when current KVDB version number is changed. @see fdb_kvdb.ver_num */
/* #define FDB_KV_AUTO_UPDATE */
#endif

/* using TSDB (Time series database) feature */
#define FDB_USING_TSDB

/* Using FAL storage mode */
#define FDB_USING_FAL_MODE

#ifdef FDB_USING_FAL_MODE
/* the flash write granularity, unit: bit
 * only support 1(nor flash)/ 8(stm32f2/f4)/ 32(stm32f1) */
#define FDB_WRITE_GRAN                /* @note you must define it for a value */
#endif

/* Using file storage mode by LIBC file API, like fopen/fread/fwrte/fclose */
/* #define FDB_USING_FILE_LIBC_MODE */

/* Using file storage mode by POSIX file API, like open/read/write/close */
/* #define FDB_USING_FILE_POSIX_MODE */

/* MCU Endian Configuration, default is Little Endian Order. */
/* #define FDB_BIG_ENDIAN */ 

/* log print macro. default EF_PRINT macro is printf() */
/* #define FDB_PRINT(...)              my_printf(__VA_ARGS__) */

/* print debug information */
#define FDB_DEBUG_ENABLE

/******************************************************************************
 *   Macros
 ******************************************************************************/
/// software version number
#define FDB_SW_VERSION  "1.0.99"
#define FDB_SW_VERSION_NUM  0x10099

/// the KV max name length must less then it
#ifndef FDB_KV_NAME_MAX
#define FDB_KV_NAME_MAX  32
#endif

/// the KV cache table size, it will improve KV search speed when using cache
#ifndef FDB_KV_CACHE_TABLE_SIZE
#define FDB_KV_CACHE_TABLE_SIZE  16
#endif

/// the sector cache table size, it will improve KV save speed when using cache
#ifndef FDB_SECTOR_CACHE_TABLE_SIZE
#define FDB_SECTOR_CACHE_TABLE_SIZE  4
#endif

#if (FDB_KV_CACHE_TABLE_SIZE > 0) && (FDB_SECTOR_CACHE_TABLE_SIZE > 0)
#define FDB_KV_USING_CACHE
#endif

#if defined(FDB_USING_FILE_LIBC_MODE) || defined(FDB_USING_FILE_POSIX_MODE)
#define FDB_USING_FILE_MODE
#endif

#ifndef FDB_WRITE_GRAN
#define FDB_WRITE_GRAN 1
#endif

/// log function. default FDB_PRINT macro is printf()
#ifndef FDB_PRINT
#define FDB_PRINT(...)  printf(__VA_ARGS__)
#endif
#define FDB_LOG_PREFIX1()  FDB_PRINT("[FlashDB]"FDB_LOG_TAG)
#define FDB_LOG_PREFIX2()  FDB_PRINT(" ")
#define FDB_LOG_PREFIX()  FDB_LOG_PREFIX1();FDB_LOG_PREFIX2()
#ifdef FDB_DEBUG_ENABLE
#define FDB_DEBUG(...)  FDB_LOG_PREFIX();FDB_PRINT("(%s:%d) ", __FILE__, __LINE__);FDB_PRINT(__VA_ARGS__)
#else
#define FDB_DEBUG(...)
#endif
/// routine print function. Must be implement by user. 
#define FDB_INFO(...)  FDB_LOG_PREFIX();FDB_PRINT(__VA_ARGS__)
/// assert for developer. 
#define FDB_ASSERT(EXPR)                                                      \
if (!(EXPR))                                                                  \
{                                                                             \
    FDB_DEBUG("(%s) has assert failed at %s.\n", #EXPR, __FUNCTION__);        \
    while (1);                                                                \
}

#define FDB_KVDB_CTRL_SET_SEC_SIZE  0x0 //< set sector size control command
#define FDB_KVDB_CTRL_GET_SEC_SIZE  0x1 //< get sector size control command
#define FDB_KVDB_CTRL_SET_LOCK      0x2 //< set lock function control command
#define FDB_KVDB_CTRL_SET_UNLOCK    0x3 //< set unlock function control command
#define FDB_KVDB_CTRL_SET_FILE_MODE 0x9 //< set file mode control command
#define FDB_KVDB_CTRL_SET_MAX_SIZE  0xA //< set database max size in file mode control command

typedef time_t fdb_time_t;

#ifdef FDB_USING_TIMESTAMP_64BIT
typedef int64_t fdb_time_t;
#endif

typedef fdb_time_t (*fdb_get_time)(void);

/******************************************************************************
 *   Data Types
 ******************************************************************************/
struct fdb_default_kv_node
{
  char *key;
  void *value;
  size_t value_len;
};

struct fdb_default_kv
{
  struct fdb_default_kv_node *kvs;
  size_t num;
};

/* error code */
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

enum fdb_kv_status
{
  FDB_KV_UNUSED,
  FDB_KV_PRE_WRITE,
  FDB_KV_WRITE,
  FDB_KV_PRE_DELETE,
  FDB_KV_DELETED,
  FDB_KV_ERR_HDR,
  FDB_KV_STATUS_NUM,
};
typedef enum fdb_kv_status fdb_kv_status_t;

/* key-value node object */
struct fdb_kv
{
  fdb_kv_status_t status;                      /**< node status, @see fdb_kv_status_t */
  bool crc_is_ok;                              /**< node CRC32 check is OK */
  uint8_t name_len;                            /**< name length */
  uint32_t magic;                              /**< magic word(`K`, `V`, `4`, `0`) */
  uint32_t len;                                /**< node total length (header + name + value), must align by FDB_WRITE_GRAN */
  uint32_t value_len;                          /**< value length */
  char name[FDB_KV_NAME_MAX];                  /**< name */
  struct
  {
    uint32_t start;                          /**< node start address */
    uint32_t value;                          /**< value start address */
  } addr;
};
typedef struct fdb_kv *fdb_kv_t;

struct fdb_kv_iterator
{
  struct fdb_kv curr_kv;                       /**< Current KV we get from the iterator */
  uint32_t iterated_cnt;                       /**< How many KVs have we iterated already */
  size_t iterated_obj_bytes;                   /**< Total storage size of KVs we have iterated. */
  size_t iterated_value_bytes;                 /**< Total value size of KVs we have iterated. */
  uint32_t sector_addr;                        /**< Current sector address we're iterating. DO NOT touch it. */
};
typedef struct fdb_kv_iterator *fdb_kv_iterator_t;

typedef enum
{
  FDB_DB_TYPE_KV,
  FDB_DB_TYPE_TS,
} fdb_db_type;

/* the flash sector store status */
enum fdb_sector_store_status
{
  FDB_SECTOR_STORE_UNUSED,
  FDB_SECTOR_STORE_EMPTY,
  FDB_SECTOR_STORE_USING,
  FDB_SECTOR_STORE_FULL,
  FDB_SECTOR_STORE_STATUS_NUM,
};
typedef enum fdb_sector_store_status fdb_sector_store_status_t;

/* the flash sector dirty status */
enum fdb_sector_dirty_status
{
  FDB_SECTOR_DIRTY_UNUSED,
  FDB_SECTOR_DIRTY_FALSE,
  FDB_SECTOR_DIRTY_TRUE,
  FDB_SECTOR_DIRTY_GC,
  FDB_SECTOR_DIRTY_STATUS_NUM,
};
typedef enum fdb_sector_dirty_status fdb_sector_dirty_status_t;

/* KVDB section information */
struct kvdb_sec_info
{
  bool check_ok;                               /**< sector header check is OK */
  struct
  {
    fdb_sector_store_status_t store;         /**< sector store status @see fdb_sector_store_status_t */
    fdb_sector_dirty_status_t dirty;         /**< sector dirty status @see sector_dirty_status_t */
  } status;
  uint32_t addr;                               /**< sector start address */
  uint32_t magic;                              /**< magic word(`E`, `F`, `4`, `0`) */
  uint32_t combined;                           /**< the combined next sector number, 0xFFFFFFFF: not combined */
  size_t remain;                               /**< remain size */
  uint32_t empty_kv;                           /**< the next empty KV node start address */
};
typedef struct kvdb_sec_info *kv_sec_info_t;

struct kv_cache_node
{
  uint16_t name_crc;                           /**< KV name's CRC32 low 16bit value */
  uint16_t active;                             /**< KV node access active degree */
  uint32_t addr;                               /**< KV node address */
};
typedef struct kv_cache_node *kv_cache_node_t;

struct sector_cache_node
{
  uint32_t addr;                               /**< sector start address */
  uint32_t empty_addr;                         /**< sector empty address */
};
typedef struct sector_cache_node *sector_cache_node_t;

/* database structure */
typedef struct fdb_db *fdb_db_t;

struct fdb_db
{
  const char *name;                            /**< database name */
  fdb_db_type type;                            /**< database type */
  union
  {
#ifdef FDB_USING_FAL_MODE
    const struct fal_partition *part;        /**< flash partition for saving database */
#endif
#ifdef FDB_USING_FILE_MODE
    const char *dir;                         /**< directory path for saving database */
#endif
  } storage;
  uint32_t sec_size;                           /**< flash section size. It's a multiple of block size */
  uint32_t max_size;                           /**< database max size. It's a multiple of section size */
  bool init_ok;                                /**< initialized successfully */
  bool file_mode;                              /**< is file mode, default is false */
#ifdef FDB_USING_FILE_MODE
#if defined(FDB_USING_FILE_POSIX_MODE)
  int cur_file;                                /**< current file object */
#elif defined(FDB_USING_FILE_LIBC_MODE)
  FILE *cur_file;                              /**< current file object */
#endif
  uint32_t cur_sec;                            /**< current operate sector address  */
#endif
  void (*lock)(fdb_db_t db);                   /**< lock the database operate */
  void (*unlock)(fdb_db_t db);                 /**< unlock the database operate */

  void *user_data;
};

/* KVDB structure */
struct fdb_kvdb
{
  struct fdb_db parent;                        /**< inherit from fdb_db */
  struct fdb_default_kv default_kvs;           /**< default KV */
  bool gc_request;                             /**< request a GC check */
  bool in_recovery_check;                      /**< is in recovery check status when first reboot */

#ifdef FDB_KV_USING_CACHE
  /* KV cache table */
  struct kv_cache_node kv_cache_table[FDB_KV_CACHE_TABLE_SIZE];
  /* sector cache table, it caching the sector info which status is current using */
  struct sector_cache_node sector_cache_table[FDB_SECTOR_CACHE_TABLE_SIZE];
#endif /* FDB_KV_USING_CACHE */

#ifdef FDB_KV_AUTO_UPDATE
  uint32_t ver_num;                            /**< setting version number for update */
#endif

  void *user_data;
};
typedef struct fdb_kvdb *fdb_kvdb_t;

/* blob structure */
struct fdb_blob
{
  void *buf;                                   /**< blob data buffer */
  size_t size;                                 /**< blob data buffer size */
  struct
  {
    uint32_t meta_addr;                      /**< saved KV or TSL index address */
    uint32_t addr;                           /**< blob data saved address */
    size_t len;                              /**< blob data saved length */
  } saved;
};
typedef struct fdb_blob *fdb_blob_t;

/******************************************************************************
 *   Function prototypes
 ******************************************************************************/
/// FlashDB database API
fdb_err_t fdb_kvdb_init(fdb_kvdb_t db, const char *name, const char *part_name, struct fdb_default_kv *default_kv, void *user_data);
void fdb_kvdb_control(fdb_kvdb_t db, int cmd, void *arg);

/// blob API
fdb_blob_t fdb_blob_make(fdb_blob_t blob, const void *value_buf, size_t buf_len);
size_t fdb_blob_read(fdb_db_t db, fdb_blob_t blob);

/// Key-Value API like a KV DB
fdb_err_t fdb_kv_set(fdb_kvdb_t db, const char *key, const char *value);
char *fdb_kv_get(fdb_kvdb_t db, const char *key);
fdb_err_t fdb_kv_set_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob);
size_t fdb_kv_get_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob);
fdb_err_t fdb_kv_del(fdb_kvdb_t db, const char *key);
fdb_kv_t fdb_kv_get_obj(fdb_kvdb_t db, const char *key, fdb_kv_t kv);
fdb_blob_t fdb_kv_to_blob(fdb_kv_t   kv, fdb_blob_t blob);
fdb_err_t fdb_kv_set_default(fdb_kvdb_t db);
void fdb_kv_print(fdb_kvdb_t db);

fdb_kv_iterator_t fdb_kv_iterator_init(fdb_kv_iterator_t itr);
bool fdb_kv_iterate(fdb_kvdb_t db, fdb_kv_iterator_t itr);

uint32_t fdb_calc_crc32(uint32_t crc, const void *buf, size_t size);

#endif /* _FLASH_DB_H_ */
