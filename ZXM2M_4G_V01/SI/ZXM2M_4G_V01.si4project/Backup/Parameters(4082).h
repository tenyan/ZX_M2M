/*****************************************************************************
* @FileName: Parameters.h
* @Engineer: TenYan
* @version   V1.0
* @Date:     2020-1-1
* @brief
******************************************************************************/
#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "types.h"
#include "M2mProtocol.h"

/******************************************************************************
* MEMORY MAP
******************************************************************************/
#define FLASH_SECTOR_SIZE               0x1000UL   // 扇区大小4096=0x1000 Bytes(4K)
#define NUMBER_OF_FLASH_SECTOR          0x2000UL   // 扇区总数8192

// 各个存储区大小
#define EMAP_NO_USED_1_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*8))  // 32KB
#define EMAP_M2M_ASSET_DATA1_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_LVC_INFO1_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_WORK_TIME_INFO1_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_HEX_FILE_SIZE              ((uint32_t)(FLASH_SECTOR_SIZE*128UL)) // 512KB
#define EMAP_M2M_BZ_FILE_SIZE           ((uint32_t)(FLASH_SECTOR_SIZE*1280UL)) // 5MB
#define EMAP_EP_BZ_FILE_SIZE            ((uint32_t)(FLASH_SECTOR_SIZE*6556UL)) // 26MB
#define EMAP_WORK_TIME_INFO2_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_LVC_INFO2_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_M2M_ASSET_DATA2_SIZE       ((uint32_t)(FLASH_SECTOR_SIZE*1))  // 4KB
#define EMAP_NO_USED_2_SIZE             ((uint32_t)(FLASH_SECTOR_SIZE*8))  // 32KB

// 各个存储区起始地址
#define EMAP_NO_USED_1_ADDRESS          0x0000UL  // = 0
#define EMAP_M2M_ASSET_DATA1_ADDRESS    (EMAP_NO_USED_1_ADDRESS + EMAP_NO_USED_1_SIZE)
#define EMAP_LVC_INFO1_ADDRESS          (EMAP_M2M_ASSET_DATA1_ADDRESS + EMAP_M2M_ASSET_DATA1_SIZE)
#define EMAP_WORK_TIME_INFO1_ADDRESS    (EMAP_LVC_INFO1_ADDRESS + EMAP_LVC_INFO1_SIZE)
#define EMAP_HEX_FILE_ADDRESS           (EMAP_WORK_TIME_INFO1_ADDRESS + EMAP_WORK_TIME_INFO1_SIZE)
#define EMAP_M2M_BZ_FILE_ADDRESS        (EMAP_HEX_FILE_ADDRESS + EMAP_HEX_FILE_SIZE)
#define EMAP_EP_BZ_FILE_ADDRESS         (EMAP_M2M_BZ_FILE_ADDRESS + EMAP_M2M_BZ_FILE_SIZE)
#define EMAP_WORK_TIME_INFO2_ADDRESS    (EMAP_EP_BZ_FILE_ADDRESS + EMAP_EP_BZ_FILE_SIZE)
#define EMAP_LVC_INFO2_ADDRESS          (EMAP_WORK_TIME_INFO2_ADDRESS + EMAP_WORK_TIME_INFO2_SIZE)
#define EMAP_M2M_ASSET_DATA2_ADDRESS    (EMAP_LVC_INFO2_ADDRESS + EMAP_LVC_INFO2_SIZE)
#define EMAP_NO_USED_2_ADDRESS          (EMAP_M2M_ASSET_DATA2_ADDRESS + EMAP_M2M_ASSET_DATA2_SIZE)
#define EMAP_END_ADDRESS                (EMAP_NO_USED_2_ADDRESS + EMAP_NO_USED_2_SIZE)




#define FILE_SIZE                   0x100000       // 单个文件存储大小单位字节 1M 0x100000


#define EMMC_FILE_PATH           	        "/media/card/"   //can数据存储文件
#define FLASH_PAGEADDR_CAN            	    "/media/card/can2.txt"   //can数据存储文件

#define FLASH_PAGEADDR_PARAMET1            	"/media/card/paramet1.txt"                   //配置参数存储页面地址1  0~3
#define FLASH_PAGEADDR_PARAMET2            	"/data/paramet2.txt"                  //配置参数存储页面地址2  16~19

#define FLASH_PAGEADDR_ZX_PARAMET1          "/media/card/zx_paramet1.txt"                   //配置参数存储页面地址1  0~3
#define FLASH_PAGEADDR_ZX_PARAMET2          "/data/zx_paramet2.txt"                  //配置参数存储页面地址2  16~19

#define FLASH_PAGEADDR_MCU1					"/media/card/mcu1.txt" 					//MCU相关参数储页面地址1
#define FLASH_PAGEADDR_MCU2					"/data/mcu2.txt" 					//MCU相关参数储页面地址1

#define FLASH_EngineType_ADDR				"/data/engine_type.txt" 				//存储力限器发送来发动机类型地址

#define FLASH_ProtocolId_ADDR1			"/media/card/protocol_id.txt"  //存储控制器发来的车辆协议编号
#define FLASH_ProtocolId_ADDR2			"/data/protocol_id.txt" 	     //存储控制器发来的车辆协议编号

#define FLASH_VIN_ADDR1			"/media/card/vin.txt"  //存储平台下发的车辆识别码(VIN)
#define FLASH_VIN_ADDR2			"/data/vin.txt" 	     //存储平台下发的车辆识别码(VIN)

#define FLASH_GPSTimeOutTimer_ADDR1         "/media/card/gps_timeout_timer1.txt"
#define FLASH_GPSTimeOutTimer_ADDR2         "/data/gps_timeout_timer2.txt"

#define FILE_ZX_BLIND_ZONE_DATA_ADDR	"/media/card/zx_blind_zone_data.txt"  // 存储zx盲区数据
#define FILE_ZX_BLIND_ZONE_PARA_ADDR	"/media/card/zx_blind_zone_para.txt"  // 存储zx盲区参数

#define FILE_ZXEP_BLIND_ZONE_DATA_ADDR	"/media/card/zxep_blind_zone_data.txt"  // 存储zxep盲区数据
#define FILE_ZXEP_BLIND_ZONE_PARA_ADDR	"/media/card/zxep_blind_zone_para.txt"  // 存储zxep盲区参数

#define FLASH_Mileage_ADDR1                 "/media/card/mileage1.txt"
#define FLASH_Mileage_ADDR2                 "/data/mileage2.txt"

#define FLASH_PAGEADDR_WORKTIME1			"/media/card/worktime1.txt"	      //工作小时存储页地址1
#define FLASH_PAGEADDR_WORKTIME2			"/data/worktime2.txt"	      //工作小时存储页地址2
//远程升级和盲区保存暂不做处理
//#define FLASH_PAGEADDR_UPGRADE             128                 // 升级地址

//#define FLASH_Firmware_MCU                 928                 //MCU固件保持地址 最大512K

#define FLASH_FIRMWARE_Core_MCU             "/media/card/helloworld_bak"     //核心板应用升级程序地址

#define FLASH_FIRMWARE_Auxi_MCU             "/media/card/Auxi_MCU"     //辅助处理器固件保持地址(Auxiliary)
#define FLASH_FIRMWARE_Vehicle_MCU          "/media/card/Vehicle_MCU"  //车辆控制器

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

#define PARM_DATA_HEADER    0x55AA5AA5

/******************************************************************************
* Function prototypes
******************************************************************************/
// 文件操作IO
uint8_t FileIO_Write(const char *file_name, uint8_t *pdata, uint32_t size);
uint8_t FileIO_Read(const char *file_name, uint8_t *pbuf, uint32_t size);
uint8_t FileIO_RandomRead(const char *file_name, uint32_t offset_addr, uint8_t *pbuf, uint32_t size);
uint8_t FileIO_RandomWrite(const char *file_name, uint32_t offset_addr, uint8_t *pdata, uint32_t size);

// 用户接口
void Parm_ResetM2mAssetDataToFactory(void);
void Parm_SaveM2mAssetData(void);
void Parm_ReadM2mAssetData(void);
void Parm_SaveLvcInfo(void);
void Parm_ReadLvcInfo(void);
void Parm_SaveTotalWorkTimeInfo(void);
void Parm_ReadTotalWorkTimeInfo(void);
void BKP_SaveTotalWorkTimeInfo(void);

uint8_t rfu_CheckNewFirmware(rfu_context_t* pThis,uint8_t* buffer, uint16_t bufferSize);
void rfu_EraseFlashHexFile(void);
void rfu_SaveFlashHexFile(rfu_context_t* pThis, uint8_t *buf, uint16_t length);
void rfu_ReadFlashHexFile(uint32_t address, uint8_t *buf, uint32_t cnt);

#endif /* _PARAMETERS_H_ */

