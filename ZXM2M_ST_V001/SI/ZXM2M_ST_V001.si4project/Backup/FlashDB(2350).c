/*****************************************************************************
* @FileName: sfud.c
* @Engineer: TenYan
* @Date:     2020-7-6
* @brief     This file is part of the Key-Value Database Library.
******************************************************************************/
#include "sfud.h"
#include <string.h>
#include "stm32f2xx_conf.h"
#include "main.h"
#include "cmsis_os2.h"

#include <inttypes.h>
#include <string.h>
#include <FlashDB.h>

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

/******************************************************************************
 * Macros
 ******************************************************************************/
#define FDB_LOG_TAG "[kv]"
/* rewrite log prefix */
#undef  FDB_LOG_PREFIX2
#define FDB_LOG_PREFIX2()  FDB_PRINT("[%s] ", db_name(db))

#if defined(FDB_USING_KVDB)

#ifndef FDB_WRITE_GRAN
#error "Please configure flash write granularity (in fdb_cfg.h)"
#endif

#if FDB_WRITE_GRAN != 1 && FDB_WRITE_GRAN != 8 && FDB_WRITE_GRAN != 32
#error "the write gran can be only setting as 1, 8 and 32"
#endif

#define SECTOR_MAGIC_WORD  0x30424446  // magic word(`F`, `D`, `B`, `1`)
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

#define SECTOR_NOT_COMBINED  0xFFFFFFFF  // the sector is not combined value
#define FAILED_ADDR  0xFFFFFFFF  // the next address is get failed

#define KV_STATUS_TABLE_SIZE  FDB_STATUS_TABLE_SIZE(FDB_KV_STATUS_NUM)

#define SECTOR_NUM  (db_max_size(db) / db_sec_size(db))

#define SECTOR_HDR_DATA_SIZE  (FDB_WG_ALIGN(sizeof(struct sector_hdr_data)))
#define SECTOR_DIRTY_OFFSET  ((unsigned long)(&((struct sector_hdr_data *)0)->status_table.dirty))
#define KV_HDR_DATA_SIZE  (FDB_WG_ALIGN(sizeof(struct kv_hdr_data)))
#define KV_MAGIC_OFFSET  ((unsigned long)(&((struct kv_hdr_data *)0)->magic))
#define KV_LEN_OFFSET  ((unsigned long)(&((struct kv_hdr_data *)0)->len))
#define KV_NAME_LEN_OFFSET  ((unsigned long)(&((struct kv_hdr_data *)0)->name_len))

#define db_name(db)  (((fdb_db_t)db)->name)
#define db_init_ok(db)  (((fdb_db_t)db)->init_ok)
#define db_sec_size(db)  (((fdb_db_t)db)->sec_size)
#define db_max_size(db)  (((fdb_db_t)db)->max_size)
#define db_lock(db)  do {  if (((fdb_db_t)db)->lock) ((fdb_db_t)db)->lock((fdb_db_t)db);} while(0);
#define db_unlock(db)  do {   if (((fdb_db_t)db)->unlock) ((fdb_db_t)db)->unlock((fdb_db_t)db);} while(0);

#define VER_NUM_KV_NAME  "__ver_num__"

/******************************************************************************
* Data Types and Globals
******************************************************************************/
struct sector_hdr_data
{
  struct
  {
    uint8_t store[FDB_STORE_STATUS_TABLE_SIZE]; //< sector store status @see fdb_sector_store_status_t
    uint8_t dirty[FDB_DIRTY_STATUS_TABLE_SIZE]; //< sector dirty status @see fdb_sector_dirty_status_t
  } status_table;
  uint32_t magic; //< magic word(`E`, `F`, `4`, `0`)
  uint32_t combined; //< the combined next sector number, 0xFFFFFFFF: not combined
  uint32_t reserved;
};
typedef struct sector_hdr_data *sector_hdr_data_t;

struct kv_hdr_data
{
  uint8_t status_table[KV_STATUS_TABLE_SIZE]; //< KV node status, @see fdb_kv_status_t
  uint32_t magic; //< magic word(`K`, `V`, `4`, `0`)
  uint32_t len; //< KV node total length (header + name + value), must align by FDB_WRITE_GRAN
  uint32_t crc32; //< KV node crc32(name_len + data_len + name + value)
  uint8_t name_len; //< name length
  uint32_t value_len; //< value length
};
typedef struct kv_hdr_data *kv_hdr_data_t;

struct alloc_kv_cb_args
{
  fdb_kvdb_t db;
  size_t kv_size;
  uint32_t *empty_kv;
};

static void gc_collect(fdb_kvdb_t db);

//==low level API========================================
fdb_err_t _fdb_kv_load(fdb_kvdb_t db);
size_t _fdb_set_status(uint8_t status_table[], size_t status_num, size_t status_index);
size_t _fdb_get_status(uint8_t status_table[], size_t status_num);
uint32_t _fdb_continue_ff_addr(fdb_db_t db, uint32_t start, uint32_t end);
fdb_err_t _fdb_init_ex(fdb_db_t db, const char *name, const char *part_name, fdb_db_type type, void *user_data);
void _fdb_init_finish(fdb_db_t db, fdb_err_t result);
fdb_err_t _fdb_write_status(fdb_db_t db, uint32_t addr, uint8_t status_table[], size_t status_num, size_t status_index);
size_t _fdb_read_status(fdb_db_t db, uint32_t addr, uint8_t status_table[], size_t total_num);
fdb_err_t _fdb_flash_read(fdb_db_t db, uint32_t addr, void *buf, size_t size);
fdb_err_t _fdb_flash_erase(fdb_db_t db, uint32_t addr, size_t size);
fdb_err_t _fdb_flash_write(fdb_db_t db, uint32_t addr, const void *buf, size_t size);

/******************************************************************************
* function prototypes
******************************************************************************/
fdb_err_t fdb_kv_set_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob);
size_t fdb_kv_get_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob);
fdb_err_t fdb_kv_set_default(fdb_kvdb_t db);

#ifdef FDB_KV_USING_CACHE
/*
 * It's only caching the current using status sector's empty_addr
 */
static void update_sector_cache(fdb_kvdb_t db, uint32_t sec_addr, uint32_t empty_addr)
{
  size_t i, empty_index = FDB_SECTOR_CACHE_TABLE_SIZE;

  for (i = 0; i < FDB_SECTOR_CACHE_TABLE_SIZE; i++)
  {
    if ((empty_addr > sec_addr) && (empty_addr < sec_addr + db_sec_size(db)))
    {
      /* update the sector empty_addr in cache */
      if (db->sector_cache_table[i].addr == sec_addr)
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
      /* delete the sector which status is not current using */
      db->sector_cache_table[i].addr = FDB_DATA_UNUSED;
      return;
    }
  }
  /* add the sector empty_addr to cache */
  if (empty_index < FDB_SECTOR_CACHE_TABLE_SIZE)
  {
    db->sector_cache_table[empty_index].addr = sec_addr;
    db->sector_cache_table[empty_index].empty_addr = empty_addr;
  }
}

/*
 * Get sector info from cache. It's return true when cache is hit.
 */
static bool get_sector_from_cache(fdb_kvdb_t db, uint32_t sec_addr, uint32_t *empty_addr)
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

static void update_kv_cache(fdb_kvdb_t db, const char *name, size_t name_len, uint32_t addr)
{
  size_t i, empty_index = FDB_KV_CACHE_TABLE_SIZE, min_activity_index = FDB_KV_CACHE_TABLE_SIZE;
  uint16_t name_crc = (uint16_t) (FlashDB_CalcCrc32(0, name, name_len) >> 16), min_activity = 0xFFFF;

  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    if (addr != FDB_DATA_UNUSED)
    {
      /* update the KV address in cache */
      if (db->kv_cache_table[i].name_crc == name_crc)
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
      /* delete the KV */
      db->kv_cache_table[i].addr = FDB_DATA_UNUSED;
      db->kv_cache_table[i].active = 0;
      return;
    }
  }
  /* add the KV to cache, using LRU (Least Recently Used) like algorithm */
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

/*
 * Get KV info from cache. It's return true when cache is hit.
 */
static bool get_kv_from_cache(fdb_kvdb_t db, const char *name, size_t name_len, uint32_t *addr)
{
  size_t i;
  uint16_t name_crc = (uint16_t) (FlashDB_CalcCrc32(0, name, name_len) >> 16);

  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    if ((db->kv_cache_table[i].addr != FDB_DATA_UNUSED) && (db->kv_cache_table[i].name_crc == name_crc))
    {
      char saved_name[FDB_KV_NAME_MAX];
      /* read the KV name in flash */
      _fdb_flash_read((fdb_db_t)db, db->kv_cache_table[i].addr + KV_HDR_DATA_SIZE, (uint32_t *) saved_name, FDB_KV_NAME_MAX);
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

/*
 * find the next KV address by magic word on the flash
 */
static uint32_t find_next_kv_addr(fdb_kvdb_t db, uint32_t start, uint32_t end)
{
  uint8_t buf[32];
  uint32_t start_bak = start, i;
  uint32_t magic;

#ifdef FDB_KV_USING_CACHE
  uint32_t empty_kv;

  if (get_sector_from_cache(db, FDB_ALIGN_DOWN(start, db_sec_size(db)), &empty_kv) && start == empty_kv)
  {
    return FAILED_ADDR;
  }
#endif /* FDB_KV_USING_CACHE */

  for (; start < end && start + sizeof(buf) < end; start += (sizeof(buf) - sizeof(uint32_t)))
  {
    _fdb_flash_read((fdb_db_t)db, start, (uint32_t *) buf, sizeof(buf));
    for (i = 0; i < sizeof(buf) - sizeof(uint32_t) && start + i < end; i++)
    {
#ifndef FDB_BIG_ENDIAN /// Little Endian Order
      magic = buf[i] + (buf[i + 1] << 8) + (buf[i + 2] << 16) + (buf[i + 3] << 24);
#else /// Big Endian Order
      magic = buf[i + 3] + (buf[i + 2] << 8) + (buf[i + 1] << 16) + (buf[i] << 24);
#endif
      if (magic == KV_MAGIC_WORD && (start + i - KV_MAGIC_OFFSET) >= start_bak)
      {
        return start + i - KV_MAGIC_OFFSET;
      }
    }
  }

  return FAILED_ADDR;
}

static uint32_t get_next_kv_addr(fdb_kvdb_t db, kv_sec_info_t sector, fdb_kv_t pre_kv)
{
  uint32_t addr = FAILED_ADDR;

  if (sector->status.store == FDB_SECTOR_STORE_EMPTY)
  {
    return FAILED_ADDR;
  }

  if (pre_kv->addr.start == FAILED_ADDR)
  {
    /* the first KV address */
    addr = sector->addr + SECTOR_HDR_DATA_SIZE;
  }
  else
  {
    if (pre_kv->addr.start <= sector->addr + db_sec_size(db))
    {
      if (pre_kv->crc_is_ok)
      {
        addr = pre_kv->addr.start + pre_kv->len;
      }
      else
      {
        /* when pre_kv CRC check failed, maybe the flash has error data
         * find_next_kv_addr after pre_kv address */
        addr = pre_kv->addr.start + FDB_WG_ALIGN(1);
      }
      /* check and find next KV address */
      addr = find_next_kv_addr(db, addr, sector->addr + db_sec_size(db));

      if (addr > sector->addr + db_sec_size(db) || pre_kv->len == 0)
      {
        //TODO 扇区连续模式
        return FAILED_ADDR;
      }
    }
    else
    {
      /* no KV */
      return FAILED_ADDR;
    }
  }

  return addr;
}

static fdb_err_t read_kv(fdb_kvdb_t db, fdb_kv_t kv)
{
  struct kv_hdr_data kv_hdr;
  uint8_t buf[32];
  uint32_t calc_crc32 = 0, crc_data_len, kv_name_addr;
  fdb_err_t result = FDB_NO_ERR;
  size_t len, size;

  /* read KV header raw data */
  _fdb_flash_read((fdb_db_t)db, kv->addr.start, (uint32_t *)&kv_hdr, sizeof(struct kv_hdr_data));
  kv->status = (fdb_kv_status_t) _fdb_get_status(kv_hdr.status_table, FDB_KV_STATUS_NUM);
  kv->len = kv_hdr.len;

  if (kv->len == ~0UL || kv->len > db_max_size(db) || kv->len < KV_NAME_LEN_OFFSET)
  {
    /* the KV length was not write, so reserved the info for current KV */
    kv->len = KV_HDR_DATA_SIZE;
    if (kv->status != FDB_KV_ERR_HDR)
    {
      kv->status = FDB_KV_ERR_HDR;
      FDB_DEBUG("Error: The KV @0x%08" PRIX32 " length has an error.\n", kv->addr.start);
      _fdb_write_status((fdb_db_t)db, kv->addr.start, kv_hdr.status_table, FDB_KV_STATUS_NUM, FDB_KV_ERR_HDR);
    }
    kv->crc_is_ok = false;
    return FDB_READ_ERR;
  }
  else if (kv->len > db_sec_size(db) - SECTOR_HDR_DATA_SIZE && kv->len < db_max_size(db))
  {
    //TODO 扇区连续模式，或者写入长度没有写入完整
    FDB_ASSERT(0);
  }

  /* CRC32 data len(header.name_len + header.value_len + name + value) */
  crc_data_len = kv->len - KV_NAME_LEN_OFFSET;
  /* calculate the CRC32 value */
  for (len = 0, size = 0; len < crc_data_len; len += size)
  {
    if (len + sizeof(buf) < crc_data_len)
    {
      size = sizeof(buf);
    }
    else
    {
      size = crc_data_len - len;
    }

    _fdb_flash_read((fdb_db_t)db, kv->addr.start + KV_NAME_LEN_OFFSET + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
    calc_crc32 = FlashDB_CalcCrc32(calc_crc32, buf, size);
  }
  /* check CRC32 */
  if (calc_crc32 != kv_hdr.crc32)
  {
    kv->crc_is_ok = false;
    result = FDB_READ_ERR;
  }
  else
  {
    kv->crc_is_ok = true;
    /* the name is behind aligned KV header */
    kv_name_addr = kv->addr.start + KV_HDR_DATA_SIZE;
    _fdb_flash_read((fdb_db_t)db, kv_name_addr, (uint32_t *) kv->name, FDB_WG_ALIGN(kv_hdr.name_len));
    /* the value is behind aligned name */
    kv->addr.value = kv_name_addr + FDB_WG_ALIGN(kv_hdr.name_len);
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

static fdb_err_t read_sector_info(fdb_kvdb_t db, uint32_t addr, kv_sec_info_t sector, bool traversal)
{
  fdb_err_t result = FDB_NO_ERR;
  struct sector_hdr_data sec_hdr = {
    0 };

  FDB_ASSERT(addr % db_sec_size(db) == 0);
  FDB_ASSERT(sector);

  /* read sector header raw data */
  _fdb_flash_read((fdb_db_t)db, addr, (uint32_t *)&sec_hdr, sizeof(struct sector_hdr_data));

  sector->addr = addr;
  sector->magic = sec_hdr.magic;
  /* check magic word */
  if (sector->magic != SECTOR_MAGIC_WORD)
  {
    sector->check_ok = false;
    sector->combined = SECTOR_NOT_COMBINED;
    return FDB_INIT_FAILED;
  }
  sector->check_ok = true;
  /* get other sector info */
  sector->combined = sec_hdr.combined;
  sector->status.store = (fdb_sector_store_status_t) _fdb_get_status(sec_hdr.status_table.store, FDB_SECTOR_STORE_STATUS_NUM);
  sector->status.dirty = (fdb_sector_dirty_status_t) _fdb_get_status(sec_hdr.status_table.dirty, FDB_SECTOR_DIRTY_STATUS_NUM);
  /* traversal all KV and calculate the remain space size */
  if (traversal)
  {
    sector->remain = 0;
    sector->empty_kv = sector->addr + SECTOR_HDR_DATA_SIZE;
    if (sector->status.store == FDB_SECTOR_STORE_EMPTY)
    {
      sector->remain = db_sec_size(db) - SECTOR_HDR_DATA_SIZE;
    }
    else if (sector->status.store == FDB_SECTOR_STORE_USING)
    {
      struct fdb_kv kv_obj;

#ifdef FDB_KV_USING_CACHE
      if (get_sector_from_cache(db, addr, &sector->empty_kv))
      {
        sector->remain = db_sec_size(db) - (sector->empty_kv - sector->addr);
        return result;
      }
#endif /* FDB_KV_USING_CACHE */

      sector->remain = db_sec_size(db) - SECTOR_HDR_DATA_SIZE;
      kv_obj.addr.start = sector->addr + SECTOR_HDR_DATA_SIZE;
      do
      {
        read_kv(db, &kv_obj);
        if (!kv_obj.crc_is_ok)
        {
          if (kv_obj.status != FDB_KV_PRE_WRITE && kv_obj.status != FDB_KV_ERR_HDR)
          {
            FDB_INFO("Error: The KV (@0x%08" PRIX32 ") CRC32 check failed!\n", kv_obj.addr.start);
            sector->remain = 0;
            result = FDB_READ_ERR;
            break;
          }
        }
        sector->empty_kv += kv_obj.len;
        sector->remain -= kv_obj.len;
      } while ((kv_obj.addr.start = get_next_kv_addr(db, sector, &kv_obj)) != FAILED_ADDR);
      /* check the empty KV address by read continue 0xFF on flash  */
      {
        uint32_t ff_addr;

        ff_addr = _fdb_continue_ff_addr((fdb_db_t)db, sector->empty_kv, sector->addr + db_sec_size(db));
        /* check the flash data is clean */
        if (sector->empty_kv != ff_addr)
        {
          /* update the sector information */
          sector->empty_kv = ff_addr;
          sector->remain = db_sec_size(db) - (ff_addr - sector->addr);
        }
      }

#ifdef FDB_KV_USING_CACHE
      update_sector_cache(db, sector->addr, sector->empty_kv);
#endif
    }
  }

  return result;
}

static uint32_t get_next_sector_addr(fdb_kvdb_t db, kv_sec_info_t pre_sec)
{
  uint32_t next_addr;

  if (pre_sec->addr == FAILED_ADDR)
  {
    /* the next sector is on the top of the partition */
    return 0;
  }
  else
  {
    /* check KV sector combined */
    if (pre_sec->combined == SECTOR_NOT_COMBINED)
    {
      next_addr = pre_sec->addr + db_sec_size(db);
    } else {
      next_addr = pre_sec->addr + pre_sec->combined * db_sec_size(db);
    }
    /* check range */
    if (next_addr < db_max_size(db))
    {
      return next_addr;
    }
    else
    {
      /* no sector */
      return FAILED_ADDR;
    }
  }
}

static void kv_iterator(fdb_kvdb_t db, fdb_kv_t kv, void *arg1, void *arg2, bool (*callback)(fdb_kv_t kv, void *arg1, void *arg2))
{
  struct kvdb_sec_info sector;
  uint32_t sec_addr;

  sec_addr = 0;
  /* search all sectors */
  do
  {
    if (read_sector_info(db, sec_addr, &sector, false) != FDB_NO_ERR)
    {
      continue;
    }
    if (callback == NULL)
    {
      continue;
    }
    /* sector has KV */
    if (sector.status.store == FDB_SECTOR_STORE_USING || sector.status.store == FDB_SECTOR_STORE_FULL)
    {
      kv->addr.start = sector.addr + SECTOR_HDR_DATA_SIZE;
      /* search all KV */
      do
      {
        read_kv(db, kv);
        /* iterator is interrupted when callback return true */
        if (callback(kv, arg1, arg2))
        {
          return;
        }
      } while ((kv->addr.start = get_next_kv_addr(db, &sector, kv)) != FAILED_ADDR);
    }
  } while ((sec_addr = get_next_sector_addr(db, &sector)) != FAILED_ADDR);
}

static bool find_kv_cb(fdb_kv_t kv, void *arg1, void *arg2)
{
  const char *key = arg1;
  bool *find_ok = arg2;
  size_t key_len = strlen(key);

  if (key_len != kv->name_len)
  {
    return false;
  }
  /* check KV */
  if (kv->crc_is_ok && kv->status == FDB_KV_WRITE && !strncmp(kv->name, key, key_len))
  {
    *find_ok = true;
    return true;
  }
  return false;
}

static bool find_kv_no_cache(fdb_kvdb_t db, const char *key, fdb_kv_t kv)
{
  bool find_ok = false;

  kv_iterator(db, kv, (void *)key, &find_ok, find_kv_cb);

  return find_ok;
}

static bool find_kv(fdb_kvdb_t db, const char *key, fdb_kv_t kv)
{
  bool find_ok = false;

#ifdef FDB_KV_USING_CACHE
  size_t key_len = strlen(key);

  if (get_kv_from_cache(db, key, key_len, &kv->addr.start))
  {
    read_kv(db, kv);
    return true;
  }
#endif /* FDB_KV_USING_CACHE */

  find_ok = find_kv_no_cache(db, key, kv);

#ifdef FDB_KV_USING_CACHE
  if (find_ok)
  {
    update_kv_cache(db, key, key_len, kv->addr.start);
  }
#endif /* FDB_KV_USING_CACHE */

  return find_ok;
}

static bool fdb_is_str(uint8_t *value, size_t len)
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

static size_t get_kv(fdb_kvdb_t db, const char *key, void *value_buf, size_t buf_len, size_t *value_len)
{
  struct fdb_kv kv;
  size_t read_len = 0;

  if (find_kv(db, key, &kv))
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
      _fdb_flash_read((fdb_db_t)db, kv.addr.value, (uint32_t *) value_buf, read_len);
    }
  }
  else if (value_len)
  {
    *value_len = 0;
  }

  return read_len;
}

/**
 * Get a KV object by key name
 *
 * @param db database object
 * @param key KV name
 * @param kv KV object
 *
 * @return KV object when is not NULL
 */
fdb_kv_t FlashDB_KvGetObj(fdb_kvdb_t db, const char *key, fdb_kv_t kv)
{
  bool find_ok = false;

  if (!db_init_ok(db))
  {
    FDB_INFO("Error: KV (%s) isn't initialize OK.\n", db_name(db));
    return 0;
  }

  /* lock the KV cache */
  db_lock(db);

  find_ok = find_kv(db, key, kv);

  /* unlock the KV cache */
  db_unlock(db);

  return find_ok ? kv : NULL;
}

/**
 * Convert the KV object to blob object
 *
 * @param kv KV object
 * @param blob blob object
 *
 * @return new blob object
 */
fdb_blob_t FlashDB_KvToBlob(fdb_kv_t kv, fdb_blob_t blob)
{
  blob->saved.meta_addr = kv->addr.start;
  blob->saved.addr = kv->addr.value;
  blob->saved.len = kv->value_len;

  return blob;
}

/**
 * Get a blob KV value by key name.
 *
 * @param db database object
 * @param key KV name
 * @param blob blob object
 *
 * @return the actually get size on successful
 */
size_t fdb_kv_get_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob)
{
  size_t read_len = 0;

  if (!db_init_ok(db))
  {
    FDB_INFO("Error: KV (%s) isn't initialize OK.\n", db_name(db));
    return 0;
  }

  /* lock the KV cache */
  db_lock(db);

  read_len = get_kv(db, key, blob->buf, blob->size, &blob->saved.len);

  /* unlock the KV cache */
  db_unlock(db);

  return read_len;
}

/**
 * Get an KV value by key name.
 *
 * @note this function is NOT supported reentrant
 * @note this function is DEPRECATED
 *
 * @param db database object
 * @param key KV name
 *
 * @return value
 */
char* FlashDB_KvGet(fdb_kvdb_t db, const char *key)
{
  static char value[FDB_STR_KV_VALUE_MAX_SIZE + 1];
  size_t get_size;
  struct fdb_blob blob;

  if ((get_size = fdb_kv_get_blob(db, key, FlashDB_MakeBlob(&blob, value, FDB_STR_KV_VALUE_MAX_SIZE))) > 0)
  {
    /* the return value must be string */
    if (fdb_is_str((uint8_t *)value, get_size))
    {
      value[get_size] = '\0';
      return value;
    }
    else if (blob.saved.len > FDB_STR_KV_VALUE_MAX_SIZE)
    {
      FDB_INFO("Warning: The default string KV value buffer length (%d) is too less (%zu).\n", FDB_STR_KV_VALUE_MAX_SIZE,
               blob.saved.len);
    }
    else
    {
      FDB_INFO("Warning: The KV value isn't string. Could not be returned\n");
      return NULL;
    }
  }

  return NULL;
}

static fdb_err_t write_kv_hdr(fdb_kvdb_t db, uint32_t addr, kv_hdr_data_t kv_hdr)
{
  fdb_err_t result = FDB_NO_ERR;
  /* write the status will by write granularity */
  result = _fdb_write_status((fdb_db_t)db, addr, kv_hdr->status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_WRITE);
  if (result != FDB_NO_ERR)
  {
    return result;
  }
  /* write other header data */
  result = _fdb_flash_write((fdb_db_t)db, addr + KV_MAGIC_OFFSET, &kv_hdr->magic, sizeof(struct kv_hdr_data) - KV_MAGIC_OFFSET);

  return result;
}

static fdb_err_t format_sector(fdb_kvdb_t db, uint32_t addr, uint32_t combined_value)
{
  fdb_err_t result = FDB_NO_ERR;
  struct sector_hdr_data sec_hdr = {
    0 };

  FDB_ASSERT(addr % db_sec_size(db) == 0);

  result = _fdb_flash_erase((fdb_db_t)db, addr, db_sec_size(db));
  if (result == FDB_NO_ERR)
  {
    /* initialize the header data */
    memset(&sec_hdr, 0xFF, sizeof(struct sector_hdr_data));
    _fdb_set_status(sec_hdr.status_table.store, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_EMPTY);
    _fdb_set_status(sec_hdr.status_table.dirty, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_FALSE);
    sec_hdr.magic = SECTOR_MAGIC_WORD;
    sec_hdr.combined = combined_value;
    sec_hdr.reserved = 0xFFFFFFFF;
    /* save the header */
    result = _fdb_flash_write((fdb_db_t)db, addr, (uint32_t *)&sec_hdr, sizeof(struct sector_hdr_data));

#ifdef FDB_KV_USING_CACHE
    /* delete the sector cache */
    update_sector_cache(db, addr, addr + db_sec_size(db));
#endif /* FDB_KV_USING_CACHE */
  }

  return result;
}

static fdb_err_t update_sec_status(fdb_kvdb_t db, kv_sec_info_t sector, size_t new_kv_len, bool *is_full)
{
  uint8_t status_table[FDB_STORE_STATUS_TABLE_SIZE];
  fdb_err_t result = FDB_NO_ERR;
  /* change the current sector status */
  if (sector->status.store == FDB_SECTOR_STORE_EMPTY)
  {
    /* change the sector status to using */
    result = _fdb_write_status((fdb_db_t)db, sector->addr, status_table, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_USING);
  }
  else if (sector->status.store == FDB_SECTOR_STORE_USING)
  {
    /* check remain size */
    if (sector->remain < FDB_SEC_REMAIN_THRESHOLD || sector->remain - new_kv_len < FDB_SEC_REMAIN_THRESHOLD)
    {
      /* change the sector status to full */
      result = _fdb_write_status((fdb_db_t)db, sector->addr, status_table, FDB_SECTOR_STORE_STATUS_NUM, FDB_SECTOR_STORE_FULL);

#ifdef FDB_KV_USING_CACHE
      /* delete the sector cache */
      update_sector_cache(db, sector->addr, sector->addr + db_sec_size(db));
#endif /* FDB_KV_USING_CACHE */

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

static void sector_iterator(fdb_kvdb_t db, kv_sec_info_t sector, fdb_sector_store_status_t status, void *arg1, void *arg2,
                            bool (*callback)(kv_sec_info_t sector, void *arg1, void *arg2), bool traversal_kv)
{
  uint32_t sec_addr;

  /* search all sectors */
  sec_addr = 0;
  do
  {
    read_sector_info(db, sec_addr, sector, false);
    if (status == FDB_SECTOR_STORE_UNUSED || status == sector->status.store)
    {
      if (traversal_kv)
      {
        read_sector_info(db, sec_addr, sector, true);
      }
      /* iterator is interrupted when callback return true */
      if (callback && callback(sector, arg1, arg2))
      {
        return;
      }
    }
  } while ((sec_addr = get_next_sector_addr(db, sector)) != FAILED_ADDR);
}

static bool sector_statistics_cb(kv_sec_info_t sector, void *arg1, void *arg2)
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

static bool alloc_kv_cb(kv_sec_info_t sector, void *arg1, void *arg2)
{
  struct alloc_kv_cb_args *arg = arg1;

  /* 1. sector has space
   * 2. the NO dirty sector
   * 3. the dirty sector only when the gc_request is false */
  if (sector->check_ok && sector->remain > arg->kv_size && ((sector->status.dirty == FDB_SECTOR_DIRTY_FALSE)
      || (sector->status.dirty == FDB_SECTOR_DIRTY_TRUE && !arg->db->gc_request)))
  {
    *(arg->empty_kv) = sector->empty_kv;
    return true;
  }

  return false;
}

static uint32_t alloc_kv(fdb_kvdb_t db, kv_sec_info_t sector, size_t kv_size)
{
  uint32_t empty_kv = FAILED_ADDR;
  size_t empty_sector = 0, using_sector = 0;
  struct alloc_kv_cb_args arg = {
    db, kv_size, &empty_kv
  };

  /* sector status statistics */
  sector_iterator(db, sector, FDB_SECTOR_STORE_UNUSED, &empty_sector, &using_sector, sector_statistics_cb, false);
  if (using_sector > 0)
  {
    /* alloc the KV from the using status sector first */
    sector_iterator(db, sector, FDB_SECTOR_STORE_USING, &arg, NULL, alloc_kv_cb, true);
  }
  if (empty_sector > 0 && empty_kv == FAILED_ADDR)
  {
    if (empty_sector > FDB_GC_EMPTY_SEC_THRESHOLD || db->gc_request)
    {
      sector_iterator(db, sector, FDB_SECTOR_STORE_EMPTY, &arg, NULL, alloc_kv_cb, true);
    }
    else
    {
      /* no space for new KV now will GC and retry */
      FDB_DEBUG("Trigger a GC check after alloc KV failed.\n");
      db->gc_request = true;
    }
  }

  return empty_kv;
}

static fdb_err_t del_kv(fdb_kvdb_t db, const char *key, fdb_kv_t old_kv, bool complete_del)
{
  fdb_err_t result = FDB_NO_ERR;
  uint32_t dirty_status_addr;
  static bool last_is_complete_del = false;

  uint8_t status_table[KV_STATUS_TABLE_SIZE >= FDB_DIRTY_STATUS_TABLE_SIZE ? KV_STATUS_TABLE_SIZE : FDB_DIRTY_STATUS_TABLE_SIZE];

  /* need find KV */
  if (!old_kv)
  {
    struct fdb_kv kv;
    /* find KV */
    if (find_kv(db, key, &kv))
    {
      old_kv = &kv;
    }
    else
    {
      FDB_DEBUG("Not found '%s' in KV.\n", key);
      return FDB_KV_NAME_ERR;
    }
  }
  /* change and save the new status */
  if (!complete_del)
  {
    result = _fdb_write_status((fdb_db_t)db, old_kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_DELETE);
    last_is_complete_del = true;
  }
  else
  {
    result = _fdb_write_status((fdb_db_t)db, old_kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_DELETED);

    if (!last_is_complete_del && result == FDB_NO_ERR)
    {
#ifdef FDB_KV_USING_CACHE
      /* delete the KV in flash and cache */
      if (key != NULL)
      {
        /* when using del_kv(db, key, NULL, true) or del_kv(db, key, kv, true) in fdb_del_kv(db, ) and set_kv(db, ) */
        update_kv_cache(db, key, strlen(key), FDB_DATA_UNUSED);
      }
      else if (old_kv != NULL)
      {
        /* when using del_kv(db, NULL, kv, true) in move_kv(db, ) */
        update_kv_cache(db, old_kv->name, old_kv->name_len, FDB_DATA_UNUSED);
      }
#endif /* FDB_KV_USING_CACHE */
    }

    last_is_complete_del = false;
  }

  dirty_status_addr = FDB_ALIGN_DOWN(old_kv->addr.start, db_sec_size(db)) + SECTOR_DIRTY_OFFSET;
  /* read and change the sector dirty status */
  if (result == FDB_NO_ERR
      && _fdb_read_status((fdb_db_t)db, dirty_status_addr, status_table, FDB_SECTOR_DIRTY_STATUS_NUM) == FDB_SECTOR_DIRTY_FALSE)
  {
    result = _fdb_write_status((fdb_db_t)db, dirty_status_addr, status_table, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_TRUE);
  }

  return result;
}

/*
 * move the KV to new space
 */
static fdb_err_t move_kv(fdb_kvdb_t db, fdb_kv_t kv)
{
  fdb_err_t result = FDB_NO_ERR;
  uint8_t status_table[KV_STATUS_TABLE_SIZE];
  uint32_t kv_addr;
  struct kvdb_sec_info sector;

  /* prepare to delete the current KV */
  if (kv->status == FDB_KV_WRITE)
  {
    del_kv(db, NULL, kv, false);
  }

  if ((kv_addr = alloc_kv(db, &sector, kv->len)) != FAILED_ADDR)
  {
    if (db->in_recovery_check)
    {
      struct fdb_kv kv_bak;
      char name[FDB_KV_NAME_MAX + 1] = { 0 };
      strncpy(name, kv->name, kv->name_len);
      /* check the KV in flash is already create success */
      if (find_kv_no_cache(db, name, &kv_bak))
      {
        /* already create success, don't need to duplicate */
        result = FDB_NO_ERR;
        goto __exit;
      }
    }
  }
  else
  {
    return FDB_SAVED_FULL;
  }
  /* start move the KV */
  {
    uint8_t buf[32];
    size_t len, size, kv_len = kv->len;

    /* update the new KV sector status first */
    update_sec_status(db, &sector, kv->len, NULL);

    _fdb_write_status((fdb_db_t)db, kv_addr, status_table, FDB_KV_STATUS_NUM, FDB_KV_PRE_WRITE);
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
      _fdb_flash_read((fdb_db_t)db, kv->addr.start + KV_MAGIC_OFFSET + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
      result = _fdb_flash_write((fdb_db_t)db, kv_addr + KV_MAGIC_OFFSET + len, (uint32_t *) buf, size);
    }
    _fdb_write_status((fdb_db_t)db, kv_addr, status_table, FDB_KV_STATUS_NUM, FDB_KV_WRITE);

#ifdef FDB_KV_USING_CACHE
    update_sector_cache(db, FDB_ALIGN_DOWN(kv_addr, db_sec_size(db)),
                        kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv->name_len) + FDB_WG_ALIGN(kv->value_len));
    update_kv_cache(db, kv->name, kv->name_len, kv_addr);
#endif /* FDB_KV_USING_CACHE */
  }

  FDB_DEBUG("Moved the KV (%.*s) from 0x%08" PRIX32 " to 0x%08" PRIX32 ".\n", kv->name_len, kv->name, kv->addr.start, kv_addr);

__exit:
  del_kv(db, NULL, kv, true);

  return result;
}

static uint32_t new_kv(fdb_kvdb_t db, kv_sec_info_t sector, size_t kv_size)
{
  bool already_gc = false;
  uint32_t empty_kv = FAILED_ADDR;

__retry:

  if ((empty_kv = alloc_kv(db, sector, kv_size)) == FAILED_ADDR && db->gc_request && !already_gc)
  {
    FDB_DEBUG("Warning: Alloc an KV (size %zu) failed when new KV. Now will GC then retry.\n", kv_size);
    gc_collect(db);
    already_gc = true;
    goto __retry;
  }

  return empty_kv;
}

static uint32_t new_kv_ex(fdb_kvdb_t db, kv_sec_info_t sector, size_t key_len, size_t buf_len)
{
  size_t kv_len = KV_HDR_DATA_SIZE + FDB_WG_ALIGN(key_len) + FDB_WG_ALIGN(buf_len);

  return new_kv(db, sector, kv_len);
}

static bool gc_check_cb(kv_sec_info_t sector, void *arg1, void *arg2)
{
  size_t *empty_sec = arg1;

  if (sector->check_ok)
  {
    *empty_sec = *empty_sec + 1;
  }

  return false;

}

static bool do_gc(kv_sec_info_t sector, void *arg1, void *arg2)
{
  struct fdb_kv kv;
  fdb_kvdb_t db = arg1;

  if (sector->check_ok && (sector->status.dirty == FDB_SECTOR_DIRTY_TRUE || sector->status.dirty == FDB_SECTOR_DIRTY_GC))
  {
    uint8_t status_table[FDB_DIRTY_STATUS_TABLE_SIZE];
    /* change the sector status to GC */
    _fdb_write_status((fdb_db_t)db, sector->addr + SECTOR_DIRTY_OFFSET, status_table, FDB_SECTOR_DIRTY_STATUS_NUM, FDB_SECTOR_DIRTY_GC);
    /* search all KV */
    kv.addr.start = sector->addr + SECTOR_HDR_DATA_SIZE;
    do
    {
      read_kv(db, &kv);
      if (kv.crc_is_ok && (kv.status == FDB_KV_WRITE || kv.status == FDB_KV_PRE_DELETE))
      {
        /* move the KV to new space */
        if (move_kv(db, &kv) != FDB_NO_ERR)
        {
          FDB_DEBUG("Error: Moved the KV (%.*s) for GC failed.\n", kv.name_len, kv.name);
        }
      }
    } while ((kv.addr.start = get_next_kv_addr(db, sector, &kv)) != FAILED_ADDR);
    format_sector(db, sector->addr, SECTOR_NOT_COMBINED);
    FDB_DEBUG("Collect a sector @0x%08" PRIX32 "\n", sector->addr);
  }

  return false;
}

/*
 * The GC will be triggered on the following scene:
 * 1. alloc an KV when the flash not has enough space
 * 2. write an KV then the flash not has enough space
 */
static void gc_collect(fdb_kvdb_t db)
{
  struct kvdb_sec_info sector;
  size_t empty_sec = 0;

  /* GC check the empty sector number */
  sector_iterator(db, &sector, FDB_SECTOR_STORE_EMPTY, &empty_sec, NULL, gc_check_cb, false);

  /* do GC collect */
  FDB_DEBUG("The remain empty sector is %zu, GC threshold is %d.\n", empty_sec, FDB_GC_EMPTY_SEC_THRESHOLD);
  if (empty_sec <= FDB_GC_EMPTY_SEC_THRESHOLD)
  {
    sector_iterator(db, &sector, FDB_SECTOR_STORE_UNUSED, db, NULL, do_gc, false);
  }

  db->gc_request = false;
}

static fdb_err_t align_write(fdb_kvdb_t db, uint32_t addr, const uint32_t *buf, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;
  size_t align_remain;

#if (FDB_WRITE_GRAN / 8 > 0)
  uint8_t align_data[FDB_WRITE_GRAN / 8];
  size_t align_data_size = sizeof(align_data);
#else
  /* For compatibility with C89 */
  uint8_t align_data_u8, *align_data = &align_data_u8;
  size_t align_data_size = 1;
#endif

  memset(align_data, 0xFF, align_data_size);
  result = _fdb_flash_write((fdb_db_t)db, addr, buf, FDB_WG_ALIGN_DOWN(size));

  align_remain = size - FDB_WG_ALIGN_DOWN(size);
  if (result == FDB_NO_ERR && align_remain)
  {
    memcpy(align_data, (uint8_t *)buf + FDB_WG_ALIGN_DOWN(size), align_remain);
    result = _fdb_flash_write((fdb_db_t)db, addr + FDB_WG_ALIGN_DOWN(size), (uint32_t *) align_data, align_data_size);
  }

  return result;
}

static fdb_err_t create_kv_blob(fdb_kvdb_t db, kv_sec_info_t sector, const char *key, const void *value, size_t len)
{
  fdb_err_t result = FDB_NO_ERR;
  struct kv_hdr_data kv_hdr;
  bool is_full = false;
  uint32_t kv_addr = sector->empty_kv;

  if (strlen(key) > FDB_KV_NAME_MAX)
  {
    FDB_INFO("Error: The KV name length is more than %d\n", FDB_KV_NAME_MAX);
    return FDB_KV_NAME_ERR;
  }

  memset(&kv_hdr, 0xFF, sizeof(struct kv_hdr_data));
  kv_hdr.magic = KV_MAGIC_WORD;
  kv_hdr.name_len = strlen(key);
  kv_hdr.value_len = len;
  kv_hdr.len = KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len) + FDB_WG_ALIGN(kv_hdr.value_len);

  if (kv_hdr.len > db_sec_size(db) - SECTOR_HDR_DATA_SIZE)
  {
    FDB_INFO("Error: The KV size is too big\n");
    return FDB_SAVED_FULL;
  }

  if (kv_addr != FAILED_ADDR || (kv_addr = new_kv(db, sector, kv_hdr.len)) != FAILED_ADDR)
  {
    size_t align_remain;
    /* update the sector status */
    if (result == FDB_NO_ERR)
    {
      result = update_sec_status(db, sector, kv_hdr.len, &is_full);
    }
    if (result == FDB_NO_ERR)
    {
      uint8_t ff = 0xFF;
      /* start calculate CRC32 */
      kv_hdr.crc32 = FlashDB_CalcCrc32(0, &kv_hdr.name_len, KV_HDR_DATA_SIZE - KV_NAME_LEN_OFFSET);
      kv_hdr.crc32 = FlashDB_CalcCrc32(kv_hdr.crc32, key, kv_hdr.name_len);
      align_remain = FDB_WG_ALIGN(kv_hdr.name_len) - kv_hdr.name_len;
      while (align_remain--)
      {
        kv_hdr.crc32 = FlashDB_CalcCrc32(kv_hdr.crc32, &ff, 1);
      }
      kv_hdr.crc32 = FlashDB_CalcCrc32(kv_hdr.crc32, value, kv_hdr.value_len);
      align_remain = FDB_WG_ALIGN(kv_hdr.value_len) - kv_hdr.value_len;
      while (align_remain--)
      {
        kv_hdr.crc32 = FlashDB_CalcCrc32(kv_hdr.crc32, &ff, 1);
      }
      /* write KV header data */
      result = write_kv_hdr(db, kv_addr, &kv_hdr);

    }
    /* write key name */
    if (result == FDB_NO_ERR)
    {
      result = align_write(db, kv_addr + KV_HDR_DATA_SIZE, (uint32_t *) key, kv_hdr.name_len);

#ifdef FDB_KV_USING_CACHE
      if (!is_full)
      {
        update_sector_cache(db, sector->addr, kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len) + FDB_WG_ALIGN(kv_hdr.value_len));
      }
      update_kv_cache(db, key, kv_hdr.name_len, kv_addr);
#endif /* FDB_KV_USING_CACHE */
    }
    /* write value */
    if (result == FDB_NO_ERR)
    {
      result = align_write(db, kv_addr + KV_HDR_DATA_SIZE + FDB_WG_ALIGN(kv_hdr.name_len), value, kv_hdr.value_len);
    }
    /* change the KV status to KV_WRITE */
    if (result == FDB_NO_ERR)
    {
      result = _fdb_write_status((fdb_db_t)db, kv_addr, kv_hdr.status_table, FDB_KV_STATUS_NUM, FDB_KV_WRITE);
    }
    /* trigger GC collect when current sector is full */
    if (result == FDB_NO_ERR && is_full)
    {
      FDB_DEBUG("Trigger a GC check after created KV.\n");
      db->gc_request = true;
    }
  }
  else
  {
    result = FDB_SAVED_FULL;
  }

  return result;
}

/**
 * Delete an KV.
 *
 * @param db database object
 * @param key KV name
 *
 * @return result
 */
fdb_err_t FlashDB_KvDel(fdb_kvdb_t db, const char *key)
{
  fdb_err_t result = FDB_NO_ERR;

  if (!db_init_ok(db))
  {
    FDB_INFO("Error: KV (%s) isn't initialize OK.\n", db_name(db));
    return FDB_INIT_FAILED;
  }

  /* lock the KV cache */
  db_lock(db);

  result = del_kv(db, key, NULL, true);

  /* unlock the KV cache */
  db_unlock(db);

  return result;
}

static fdb_err_t set_kv(fdb_kvdb_t db, const char *key, const void *value_buf, size_t buf_len)
{
  fdb_err_t result = FDB_NO_ERR;
  static struct fdb_kv kv;
  static struct kvdb_sec_info sector;
  bool kv_is_found = false;

  if (value_buf == NULL)
  {
    result = del_kv(db, key, NULL, true);
  }
  else
  {
    /* make sure the flash has enough space */
    if (new_kv_ex(db, &sector, strlen(key), buf_len) == FAILED_ADDR)
    {
      return FDB_SAVED_FULL;
    }
    kv_is_found = find_kv(db, key, &kv);
    /* prepare to delete the old KV */
    if (kv_is_found)
    {
      result = del_kv(db, key, &kv, false);
    }
    /* create the new KV */
    if (result == FDB_NO_ERR)
    {
      result = create_kv_blob(db, &sector, key, value_buf, buf_len);
    }
    /* delete the old KV */
    if (kv_is_found && result == FDB_NO_ERR)
    {
      result = del_kv(db, key, &kv, true);
    }
    /* process the GC after set KV */
    if (db->gc_request)
    {
      gc_collect(db);
    }
  }

  return result;
}

/**
 * Set a blob KV. If it blob value is NULL, delete it.
 * If not find it in flash, then create it.
 *
 * @param db database object
 * @param key KV name
 * @param blob blob object
 *
 * @return result
 */
fdb_err_t fdb_kv_set_blob(fdb_kvdb_t db, const char *key, fdb_blob_t blob)
{
  fdb_err_t result = FDB_NO_ERR;

  if (!db_init_ok(db))
  {
    FDB_INFO("Error: KV (%s) isn't initialize OK.\n", db_name(db));
    return FDB_INIT_FAILED;
  }

  /* lock the KV cache */
  db_lock(db);

  result = set_kv(db, key, blob->buf, blob->size);

  /* unlock the KV cache */
  db_unlock(db);

  return result;
}

/**
 * Set a string KV. If it value is NULL, delete it.
 * If not find it in flash, then create it.
 *
 * @param db database object
 * @param key KV name
 * @param value KV value
 *
 * @return result
 */
fdb_err_t FlashDB_KvSet(fdb_kvdb_t db, const char *key, const char *value)
{
  struct fdb_blob blob;

  return fdb_kv_set_blob(db, key, FlashDB_MakeBlob(&blob, value, strlen(value)));
}

/**
 * recovery all KV to default.
 *
 * @param db database object
 * @return result
 */
fdb_err_t fdb_kv_set_default(fdb_kvdb_t db)
{
  fdb_err_t result = FDB_NO_ERR;
  uint32_t addr, i, value_len;
  struct kvdb_sec_info sector;

  /* lock the KV cache */
  db_lock(db);
  /* format all sectors */
  for (addr = 0; addr < db_max_size(db); addr += db_sec_size(db))
  {
    result = format_sector(db, addr, SECTOR_NOT_COMBINED);
    if (result != FDB_NO_ERR)
    {
      goto __exit;
    }
  }
  /* create default KV */
  for (i = 0; i < db->default_kvs.num; i++)
  {
    /* It seems to be a string when value length is 0.
     * This mechanism is for compatibility with older versions (less then V4.0). */
    if (db->default_kvs.kvs[i].value_len == 0)
    {
      value_len = strlen(db->default_kvs.kvs[i].value);
    }
    else
    {
      value_len = db->default_kvs.kvs[i].value_len;
    }
    sector.empty_kv = FAILED_ADDR;
    create_kv_blob(db, &sector, db->default_kvs.kvs[i].key, db->default_kvs.kvs[i].value, value_len);
    if (result != FDB_NO_ERR)
    {
      goto __exit;
    }
  }

__exit:
  /* unlock the KV cache */
  db_unlock(db);

  return result;
}

static bool print_kv_cb(fdb_kv_t kv, void *arg1, void *arg2)
{
  bool value_is_str = true, print_value = false;
  size_t *using_size = arg1;
  fdb_kvdb_t db = arg2;

  if (kv->crc_is_ok)
  {
    /* calculate the total using flash size */
    *using_size += kv->len;
    /* check KV */
    if (kv->status == FDB_KV_WRITE)
    {
      FDB_PRINT("%.*s=", kv->name_len, kv->name);

      if (kv->value_len < FDB_STR_KV_VALUE_MAX_SIZE )
      {
        uint8_t buf[32];
        size_t len, size;
__reload:
        /* check the value is string */
        for (len = 0, size = 0; len < kv->value_len; len += size)
        {
          if (len + sizeof(buf) < kv->value_len)
          {
            size = sizeof(buf);
          }
          else
          {
            size = kv->value_len - len;
          }
          _fdb_flash_read((fdb_db_t)db, kv->addr.value + len, (uint32_t *) buf, FDB_WG_ALIGN(size));
          if (print_value)
          {
            FDB_PRINT("%.*s", (int)size, buf);
          }
          else if (!fdb_is_str(buf, size))
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


/**
 * Print all KV.
 *
 * @param db database object
 */
void FlashDB_KvPrint(fdb_kvdb_t db)
{
  struct fdb_kv kv;
  size_t using_size = 0;

  if (!db_init_ok(db))
  {
    FDB_INFO("Error: KV (%s) isn't initialize OK.\n", db_name(db));
    return;
  }

  /* lock the KV cache */
  db_lock(db);

  kv_iterator(db, &kv, &using_size, db, print_kv_cb);

  FDB_PRINT("\nmode: next generation\n");
  FDB_PRINT("size: %zu/%zu bytes.\n", using_size + (size_t)((SECTOR_NUM - FDB_GC_EMPTY_SEC_THRESHOLD) * SECTOR_HDR_DATA_SIZE),
            (size_t)(db_max_size(db) - db_sec_size(db) * FDB_GC_EMPTY_SEC_THRESHOLD));

  /* unlock the KV cache */
  db_unlock(db);
}

#ifdef FDB_KV_AUTO_UPDATE
/*
 * Auto update KV to latest default when current setting version number is changed.
 */
static void kv_auto_update(fdb_kvdb_t db)
{
  size_t saved_ver_num, setting_ver_num = db->ver_num;

  if (get_kv(db, VER_NUM_KV_NAME, &saved_ver_num, sizeof(size_t), NULL) > 0)
  {
    /* check version number */
    if (saved_ver_num != setting_ver_num)
    {
      struct fdb_kv kv;
      size_t i, value_len;
      struct kvdb_sec_info sector;
      FDB_DEBUG("Update the KV from version %zu to %zu.\n", saved_ver_num, setting_ver_num);
      for (i = 0; i < db->default_kvs.num; i++)
      {
        /* add a new KV when it's not found */
        if (!find_kv(db, db->default_kvs.kvs[i].key, &kv))
        {
          /* It seems to be a string when value length is 0.
           * This mechanism is for compatibility with older versions (less then V4.0). */
          if (db->default_kvs.kvs[i].value_len == 0)
          {
            value_len = strlen(db->default_kvs.kvs[i].value);
          }
          else
          {
            value_len = db->default_kvs.kvs[i].value_len;
          }
          sector.empty_kv = FAILED_ADDR;
          create_kv_blob(db, &sector, db->default_kvs.kvs[i].key, db->default_kvs.kvs[i].value, value_len);
        }
      }
    }
    else
    {
      /* version number not changed now return */
      return;
    }
  }

  set_kv(db, VER_NUM_KV_NAME, &setting_ver_num, sizeof(size_t));
}
#endif /* FDB_KV_AUTO_UPDATE */

static bool check_sec_hdr_cb(kv_sec_info_t sector, void *arg1, void *arg2)
{
  if (!sector->check_ok)
  {
    size_t *failed_count = arg1;
    fdb_kvdb_t db = arg2;

    FDB_DEBUG("Sector header info is incorrect. Auto format this sector (0x%08" PRIX32 ").\n", sector->addr);
    (*failed_count) ++;
    format_sector(db, sector->addr, SECTOR_NOT_COMBINED);
  }

  return false;
}

static bool check_and_recovery_gc_cb(kv_sec_info_t sector, void *arg1, void *arg2)
{
  fdb_kvdb_t db = arg1;

  if (sector->check_ok && sector->status.dirty == FDB_SECTOR_DIRTY_GC)
  {
    /* make sure the GC request flag to true */
    db->gc_request = true;
    /* resume the GC operate */
    gc_collect(db);
  }

  return false;
}

static bool check_and_recovery_kv_cb(fdb_kv_t kv, void *arg1, void *arg2)
{
  fdb_kvdb_t db = arg1;

  /* recovery the prepare deleted KV */
  if (kv->crc_is_ok && kv->status == FDB_KV_PRE_DELETE)
  {
    FDB_INFO("Found an KV (%.*s) which has changed value failed. Now will recovery it.\n", kv->name_len, kv->name);
    /* recovery the old KV */
    if (move_kv(db, kv) == FDB_NO_ERR)
    {
      FDB_DEBUG("Recovery the KV successful.\n");
    }
    else
    {
      FDB_DEBUG("Warning: Moved an KV (size %" PRIu32 ") failed when recovery. Now will GC then retry.\n", kv->len);
      return true;
    }
  }
  else if (kv->status == FDB_KV_PRE_WRITE)
  {
    uint8_t status_table[KV_STATUS_TABLE_SIZE];
    /* the KV has not write finish, change the status to error */
    //TODO 绘制异常处理的状态装换图
    _fdb_write_status((fdb_db_t)db, kv->addr.start, status_table, FDB_KV_STATUS_NUM, FDB_KV_ERR_HDR);
    return true;
  }

  return false;
}

/**
 * Check and load the flash KV.
 *
 * @return result
 */
fdb_err_t _fdb_kv_load(fdb_kvdb_t db)
{
  fdb_err_t result = FDB_NO_ERR;
  struct fdb_kv kv;
  struct kvdb_sec_info sector;
  size_t check_failed_count = 0;

  db->in_recovery_check = true;
  /* check all sector header */
  sector_iterator(db, &sector, FDB_SECTOR_STORE_UNUSED, &check_failed_count, db, check_sec_hdr_cb, false);
  /* all sector header check failed */
  if (check_failed_count == SECTOR_NUM)
  {
    FDB_INFO("All sector header is incorrect. Set it to default.\n");
    fdb_kv_set_default(db);
  }

  /* lock the KV cache */
  db_lock(db);
  /* check all sector header for recovery GC */
  sector_iterator(db, &sector, FDB_SECTOR_STORE_UNUSED, db, NULL, check_and_recovery_gc_cb, false);

__retry:
  /* check all KV for recovery */
  kv_iterator(db, &kv, db, NULL, check_and_recovery_kv_cb);
  if (db->gc_request)
  {
    gc_collect(db);
    goto __retry;
  }

  db->in_recovery_check = false;

  /* unlock the KV cache */
  db_unlock(db);

  return result;
}

/**
 * This function will get or set some options of the database
 *
 * @param db database object
 * @param cmd the control command
 * @param arg the argument
 */
void FlashDB_KvControl(fdb_kvdb_t db, int cmd, void *arg)
{
  FDB_ASSERT(db);

  switch (cmd)
  {
  case FDB_KVDB_CTRL_SET_SEC_SIZE:
    /* this change MUST before database initialization */
    FDB_ASSERT(db->parent.init_ok == false);
    db->parent.sec_size = *(uint32_t *)arg;
    break;
  case FDB_KVDB_CTRL_GET_SEC_SIZE:
    *(uint32_t *)arg = db->parent.sec_size;
    break;
  case FDB_KVDB_CTRL_SET_LOCK:
    db->parent.lock = (void (*)(fdb_db_t db))arg;
    break;
  case FDB_KVDB_CTRL_SET_UNLOCK:
    db->parent.unlock = (void (*)(fdb_db_t db))arg;
    break;
  case FDB_KVDB_CTRL_SET_FILE_MODE:
#ifdef FDB_USING_FILE_MODE
    /* this change MUST before database initialization */
    FDB_ASSERT(db->parent.init_ok == false);
    db->parent.file_mode = *(bool *)arg;
#else
    FDB_INFO("Error: set file mode Failed. Please defined the FDB_USING_FILE_MODE macro.");
#endif
    break;
  case FDB_KVDB_CTRL_SET_MAX_SIZE:
#ifdef FDB_USING_FILE_MODE
    /* this change MUST before database initialization */
    FDB_ASSERT(db->parent.init_ok == false);
    db->parent.max_size = *(uint32_t *)arg;
#endif
    break;
  }
}

/**
 * The KV database initialization.
 *
 * @param db database object
 * @param name database name
 * @param part_name partition name
 * @param default_kv the default KV set @see fdb_default_kv
 * @param user_data user data
 *
 * @return result
 */
fdb_err_t FlashDB_KvInit(fdb_kvdb_t db, const char *name, const char *part_name, struct fdb_default_kv *default_kv, void *user_data)
{
  fdb_err_t result = FDB_NO_ERR;

#ifdef FDB_KV_USING_CACHE
  size_t i;
#endif

  FDB_ASSERT(default_kv);
  /* must be aligned with write granularity */
  FDB_ASSERT((FDB_STR_KV_VALUE_MAX_SIZE * 8) % FDB_WRITE_GRAN == 0);

  result = _fdb_init_ex((fdb_db_t)db, name, part_name, FDB_DB_TYPE_KV, user_data);
  if (result != FDB_NO_ERR)
  {
    goto __exit;
  }

  db->gc_request = false;
  db->in_recovery_check = false;
  db->default_kvs = *default_kv;
  /* there is at least one empty sector for GC. */
  FDB_ASSERT((FDB_GC_EMPTY_SEC_THRESHOLD > 0 && FDB_GC_EMPTY_SEC_THRESHOLD < SECTOR_NUM))

#ifdef FDB_KV_USING_CACHE
  for (i = 0; i < FDB_SECTOR_CACHE_TABLE_SIZE; i++)
  {
    db->sector_cache_table[i].addr = FDB_DATA_UNUSED;
  }
  for (i = 0; i < FDB_KV_CACHE_TABLE_SIZE; i++)
  {
    db->kv_cache_table[i].addr = FDB_DATA_UNUSED;
  }
#endif /* FDB_KV_USING_CACHE */

  FDB_DEBUG("KVDB size is %u bytes.\n", db_max_size(db));

  result = _fdb_kv_load(db);

#ifdef FDB_KV_AUTO_UPDATE
  if (result == FDB_NO_ERR)
  {
    kv_auto_update(db);
  }
#endif

__exit:

  _fdb_init_finish((fdb_db_t)db, result);

  return result;
}

/**
 * The KV database initialization.
 *
 * @param itr iterator structure to be initialized
 *
 * @return pointer to the iterator initialized.
 */
fdb_kv_iterator_t FlashDB_KvIteratorInit(fdb_kv_iterator_t itr)
{
  itr->curr_kv.addr.start = 0;

  /* If iterator statistics is needed */
  itr->iterated_cnt = 0;
  itr->iterated_obj_bytes = 0;
  itr->iterated_value_bytes = 0;
  /* Start from sector head */
  itr->sector_addr = 0;
  return itr;
}

/**
 * The KV database iterator.
 *
 * @param db database object
 * @param itr the iterator structure
 *
 * @return false if iteration is ended, true if iteration is not ended.
 */
bool FlashDB_KvIterate(fdb_kvdb_t db, fdb_kv_iterator_t itr)
{
  struct kvdb_sec_info sector;
  fdb_kv_t kv = &(itr->curr_kv);
  do
  {
    if (read_sector_info(db, itr->sector_addr, &sector, false) == FDB_NO_ERR)
    {
      if (sector.status.store == FDB_SECTOR_STORE_USING || sector.status.store == FDB_SECTOR_STORE_FULL)
      {
        if (kv->addr.start == 0)
        {
          kv->addr.start = sector.addr + SECTOR_HDR_DATA_SIZE;
        }
        else if ((kv->addr.start = get_next_kv_addr(db, &sector, kv)) == FAILED_ADDR)
        {
          kv->addr.start = 0;
          continue;
        }
        do
        {
          read_kv(db, kv);
          if (kv->status == FDB_KV_WRITE)
          {
            /* We got a valid kv here. */
            /* If iterator statistics is needed */
            itr->iterated_cnt++;
            itr->iterated_obj_bytes += kv->len;
            itr->iterated_value_bytes += kv->value_len;
            return true;
          }
        } while ((kv->addr.start = get_next_kv_addr(db, &sector, kv)) != FAILED_ADDR);
      }
    }
    /** Set kv->addr.start to 0 when we get into a new sector so that if we successfully get the next sector info,
     *  the kv->addr.start is set to the new sector.addr + SECTOR_HDR_DATA_SIZE.
    */
    kv->addr.start = 0;
  } while ((itr->sector_addr = get_next_sector_addr(db, &sector)) != FAILED_ADDR);
  /* Finally we have iterated all the KVs. */
  return false;
}

#endif /* defined(FDB_USING_KVDB) */















#include <flashdb.h>
#include <fdb_low_lvl.h>
#include <string.h>

#define FDB_LOG_TAG ""

#if !defined(FDB_USING_FAL_MODE) && !defined(FDB_USING_FILE_MODE)
#error "Please defined the FDB_USING_FAL_MODE or FDB_USING_FILE_MODE macro"
#endif

fdb_err_t _fdb_init_ex(fdb_db_t db, const char *name, const char *part_name, fdb_db_type type, void *user_data)
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
    /* must set when using file mode */
    FDB_ASSERT(db->sec_size != 0);
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

    /* FAL (Flash Abstraction Layer) initialization */
    fal_init();
    /* check the flash partition */
    if ((db->storage.part = fal_partition_find(part_name)) == NULL)
    {
      FDB_INFO("Error: Partition (%s) not found.\n", part_name);
      return FDB_PART_NOT_FOUND;
    }

    block_size = fal_flash_device_find(db->storage.part->flash_name)->blk_size;
    if (db->sec_size == 0)
    {
      db->sec_size = block_size;
    }
    else
    {
      /* must be aligned with block size */
      FDB_ASSERT(db->sec_size % block_size == 0);
    }

    db->max_size = db->storage.part->len;
#endif /* FDB_USING_FAL_MODE */
  }

  /* must align with sector size */
  FDB_ASSERT(db->max_size % db->sec_size == 0);
  /* must have more than or equal 2 sector */
  FDB_ASSERT(db->max_size / db->sec_size >= 2);

  return FDB_NO_ERR;
}

void _fdb_init_finish(fdb_db_t db, fdb_err_t result)
{
  static bool log_is_show = false;
  if (result == FDB_NO_ERR)
  {
    db->init_ok = true;
    if (!log_is_show)
    {
      FDB_INFO("FlashDB V%s is initialize success.\n", FDB_SW_VERSION);
      FDB_INFO("You can get the latest version on https://github.com/armink/FlashDB .\n");
      log_is_show = true;
    }
  }
  else
  {
    FDB_INFO("Error: %s (%s) is initialize fail (%d).\n", db->type == FDB_DB_TYPE_KV ? "KVDB" : "TSDB", db->name, (int)result);
  }
}







/*
 * Copyright (c) 2020, Armink, <armink.ztl@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief utils
 *
 * Some utils for this library.
 */

#include <stdio.h>
#include <string.h>
#include <flashdb.h>
#include <fdb_low_lvl.h>

#define FDB_LOG_TAG "[utils]"

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

/**
 * Calculate the CRC32 value of a memory buffer.
 *
 * @param crc accumulated CRC32 value, must be 0 on first call
 * @param buf buffer to calculate CRC32 value for
 * @param size bytes in buffer
 *
 * @return calculated CRC32 value
 */
uint32_t FlashDB_CalcCrc32(uint32_t crc, const void *buf, size_t size)
{
  const uint8_t *p;

  p = (const uint8_t *)buf;
  crc = crc ^ ~0U;

  while (size--) {
    crc = crc32_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
  }

  return crc ^ ~0U;
}

size_t _fdb_set_status(uint8_t status_table[], size_t status_num, size_t status_index)
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
#endif /* FDB_WRITE_GRAN == 1 */
  }

  return byte_index;
}

size_t _fdb_get_status(uint8_t status_table[], size_t status_num)
{
  size_t i = 0, status_num_bak = --status_num;

  while (status_num --)
  {
    /* get the first 0 position from end address to start address */
#if (FDB_WRITE_GRAN == 1)
    if ((status_table[status_num / 8] & (0x80 >> (status_num % 8))) == 0x00)
    {
      break;
    }
#else /*  (FDB_WRITE_GRAN == 8) ||  (FDB_WRITE_GRAN == 32) ||  (FDB_WRITE_GRAN == 64) */
    if (status_table[status_num * FDB_WRITE_GRAN / 8] == 0x00)
    {
      break;
    }
#endif /* FDB_WRITE_GRAN == 1 */
    i++;
  }

  return status_num_bak - i;
}

fdb_err_t _fdb_write_status(fdb_db_t db, uint32_t addr, uint8_t status_table[], size_t status_num, size_t status_index)
{
  fdb_err_t result = FDB_NO_ERR;
  size_t byte_index;

  FDB_ASSERT(status_index < status_num);
  FDB_ASSERT(status_table);

  /* set the status first */
  byte_index = _fdb_set_status(status_table, status_num, status_index);

  /* the first status table value is all 1, so no need to write flash */
  if (byte_index == ~0UL)
  {
    return FDB_NO_ERR;
  }
#if (FDB_WRITE_GRAN == 1)
  result = _fdb_flash_write(db, addr + byte_index, (uint32_t *)&status_table[byte_index], 1);
#else /*  (FDB_WRITE_GRAN == 8) ||  (FDB_WRITE_GRAN == 32) ||  (FDB_WRITE_GRAN == 64) */
  /* write the status by write granularity
  * some flash (like stm32 onchip) NOT supported repeated write before erase */
  result = _fdb_flash_write(db, addr + byte_index, (uint32_t *) &status_table[byte_index], FDB_WRITE_GRAN / 8);
#endif /* FDB_WRITE_GRAN == 1 */

  return result;
}

size_t _fdb_read_status(fdb_db_t db, uint32_t addr, uint8_t status_table[], size_t total_num)
{
  FDB_ASSERT(status_table);

  _fdb_flash_read(db, addr, (uint32_t *) status_table, FDB_STATUS_TABLE_SIZE(total_num));

  return _fdb_get_status(status_table, total_num);
}

/*
 * find the continue 0xFF flash address to end address
 */
uint32_t _fdb_continue_ff_addr(fdb_db_t db, uint32_t start, uint32_t end)
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
    _fdb_flash_read(db, start, (uint32_t *) buf, read_size);
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

/**
 * Make a blob object.
 *
 * @param blob blob object
 * @param value_buf value buffer
 * @param buf_len buffer length
 *
 * @return new blob object
 */
fdb_blob_t FlashDB_MakeBlob(fdb_blob_t blob, const void *value_buf, size_t buf_len)
{
  blob->buf = (void *)value_buf;
  blob->size = buf_len;

  return blob;
}

/**
 * Read the blob object in database.
 *
 * @param db database object
 * @param blob blob object
 *
 * @return read length
 */
size_t FlashDB_ReadBlob(fdb_db_t db, fdb_blob_t blob)
{
  size_t read_len = blob->size;

  if (read_len > blob->saved.len)
  {
    read_len = blob->saved.len;
  }
  _fdb_flash_read(db, blob->saved.addr, blob->buf, read_len);

  return read_len;
}

#ifdef FDB_USING_FILE_MODE
extern fdb_err_t _fdb_file_read(fdb_db_t db, uint32_t addr, void *buf, size_t size);
extern fdb_err_t _fdb_file_write(fdb_db_t db, uint32_t addr, const void *buf, size_t size);
extern fdb_err_t _fdb_file_erase(fdb_db_t db, uint32_t addr, size_t size);
#endif /* FDB_USING_FILE_LIBC */

fdb_err_t _fdb_flash_read(fdb_db_t db, uint32_t addr, void *buf, size_t size)
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
    fal_partition_read(db->storage.part, addr, (uint8_t *) buf, size);
#endif
  }

  return result;
}

fdb_err_t _fdb_flash_erase(fdb_db_t db, uint32_t addr, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    return _fdb_file_erase(db, addr, size);
#else
    return FDB_ERASE_ERR;
#endif /* FDB_USING_FILE_MODE */
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    if (fal_partition_erase(db->storage.part, addr, size) < 0)
    {
      result = FDB_ERASE_ERR;
    }
#endif
  }

  return result;
}

fdb_err_t _fdb_flash_write(fdb_db_t db, uint32_t addr, const void *buf, size_t size)
{
  fdb_err_t result = FDB_NO_ERR;

  if (db->file_mode)
  {
#ifdef FDB_USING_FILE_MODE
    return _fdb_file_write(db, addr, buf, size);
#else
    return FDB_READ_ERR;
#endif /* FDB_USING_FILE_MODE */
  }
  else
  {
#ifdef FDB_USING_FAL_MODE
    if (fal_partition_write(db->storage.part, addr, (uint8_t *)buf, size) < 0)
    {
      result = FDB_WRITE_ERR;
    }
#endif
  }

  return result;

}

