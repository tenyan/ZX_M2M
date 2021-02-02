/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: parameters.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-10-31
* @brief:
******************************************************************************/
#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "types.h"
#include "AuxCom.h"

/******************************************************************************
* MEMORY MAP
******************************************************************************/
#define FLASH_SECTOR_SIZE               0x1000UL  // 扇区大小4096=0x1000 Bytes(4K)
#define NUMBER_OF_FLASH_SECTOR          0x400UL   // 扇区总数1024

// 各个存储区大小
#define EMAP_NO_USED_1_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*4))  // 16KB
#define EMAP_M2M_ASSET_DATA1_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_LVC_INFO1_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_KVDB_SIZE                  ((uint32_t)(FLASH_SECTOR_SIZE*128UL)) // 512KB(擦除时间5210ms)
#define EMAP_HEX_FILE_SIZE              ((uint32_t)(FLASH_SECTOR_SIZE*128UL)) // 512KB(擦除时间5210ms)
#define EMAP_ECU_HEX_FILE_SIZE          ((uint32_t)(FLASH_SECTOR_SIZE*756UL)) // 3024KB
#define EMAP_LVC_INFO2_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_M2M_ASSET_DATA2_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_NO_USED_2_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*4))  // 16KB

// 各个存储区起始地址
#define EMAP_NO_USED_1_ADDRESS          0x0000UL  // = 0
#define EMAP_M2M_ASSET_DATA1_ADDRESS    (EMAP_NO_USED_1_ADDRESS + EMAP_NO_USED_1_SIZE)
#define EMAP_LVC_INFO1_ADDRESS          (EMAP_M2M_ASSET_DATA1_ADDRESS + EMAP_M2M_ASSET_DATA1_SIZE)
#define EMAP_KVDB_ADDRESS               (EMAP_LVC_INFO1_ADDRESS + EMAP_LVC_INFO1_SIZE)
#define EMAP_HEX_FILE_ADDRESS           (EMAP_KVDB_ADDRESS + EMAP_KVDB_SIZE)
#define EMAP_ECU_HEX_FILE_ADDRESS       (EMAP_HEX_FILE_ADDRESS + EMAP_HEX_FILE_SIZE)
#define EMAP_LVC_INFO2_ADDRESS          (EMAP_ECU_HEX_FILE_ADDRESS + EMAP_ECU_HEX_FILE_SIZE)
#define EMAP_M2M_ASSET_DATA2_ADDRESS    (EMAP_LVC_INFO2_ADDRESS + EMAP_LVC_INFO2_SIZE)
#define EMAP_NO_USED_2_ADDRESS          (EMAP_M2M_ASSET_DATA2_ADDRESS + EMAP_M2M_ASSET_DATA2_SIZE)
#define EMAP_END_ADDRESS                (EMAP_NO_USED_2_ADDRESS + EMAP_NO_USED_2_SIZE)

/******************************************************************************
* External variables
******************************************************************************/
typedef enum
{
  RFU_NOK = 0,
  RFU_OK  = 1,
} rfu_bool_t;

// BOOL定义
enum
{
  PARM_FALSE = 0x00,
  PARM_TRUE = 0x01
};

#define PARM_DATA_HEADER                 0x55AA5AA5

/******************************************************************************
* Function prototypes
******************************************************************************/
int32_t Parm_FlashDbInit(void);

// 作业统计类3MIN定时器
void Parm_SaveZxStsTmrInfo(void);
void Parm_ReadZxStsTmrInfo(void);

// 作业统计类
void Parm_SaveZxStsInfo(void);
void Parm_ReadZxStsInfo(void);

// 终端休眠统计
void Parm_SaveTboxStsInfo(void);
void Parm_ReadTboxStsInfo(void);

//作业油耗统计
void Parm_SaveZxEngineUpStsInfo(void);
void Parm_ReadZxEngineUpStsInfo(void);

//行驶油耗统计
void Parm_SaveZxEngineDwStsInfo(void);
void Parm_ReadZxEngineDwStsInfo(void);

//==========================================================================
void Parm_ResetM2mAssetDataToFactory(void);
void Parm_SaveM2mAssetData(void);
void Parm_ReadM2mAssetData(void);
void Parm_SaveLvcInfo(void);
void Parm_ReadLvcInfo(void);

//void Parm_SaveTotalWorkTimeInfo(void);
//void Parm_ReadTotalWorkTimeInfo(void);
//void BKP_SaveTotalWorkTimeInfo(void);
//==========================================================================
uint8_t rfu_CheckNewFirmware(rfu_context_t* pThis, uint8_t* buffer, uint16_t bufferSize);
void rfu_EraseFlashHexFile(uint8_t fileType);
void rfu_SaveFlashHexFile(uint8_t fileType, uint32_t address, uint8_t *buf, uint16_t length);
void rfu_ReadFlashHexFile(uint8_t fileType, uint32_t address, uint8_t *buf, uint16_t length);

#endif /* _PARAMETERS_H_ */

