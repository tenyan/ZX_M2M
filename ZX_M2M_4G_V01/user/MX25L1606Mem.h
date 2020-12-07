/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: MX25L1606Mem.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 存储器MX25L1606驱动头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-10, by  创建本文件
 *
 */

#ifndef _MX25L1606Mem_H
#define _MX25L1606Mem_H


#define FILE_SIZE                       0x100000                    // 单个文件存储大小单位字节 1M 0x100000 

#define EMMC_FILE_PATH           	    "/media/card/"              //can数据存储文件

#define FLASH_PAGEADDR_CAN            	"/media/card/can2.txt"      //can数据存储文件

#define FLASH_PAGEADDR_PARAMET1         "/media/card/paramet1.txt"  //配置参数存储页面地址1  
#define FLASH_PAGEADDR_PARAMET2         "/data/paramet2.txt"        //配置参数存储页面地址2  
#define FLASH_PAGEADDR_MCU1				"/media/card/mcu1.txt" 		//MCU相关参数储页面地址1
#define FLASH_PAGEADDR_MCU2				"/data/mcu2.txt" 			//MCU相关参数储页面地址1

#define FLASH_EngineType_ADDR			"/data/engine_type.txt" 	//存储力限器发送来发动机类型地址

#define FLASH_ProtocolId_ADDR1			"/media/card/protocol_id.txt"  //存储控制器发来的车辆协议编号
#define FLASH_ProtocolId_ADDR2			"/data/protocol_id.txt" 	    //存储控制器发来的车辆协议编号

#define FLASH_VIN_ADDR1			        "/media/card/vin.txt"       //存储平台下发的车辆识别码(VIN)
#define FLASH_VIN_ADDR2			        "/data/vin.txt" 	        //存储平台下发的车辆识别码(VIN)

#define FLASH_EngineUpEnable_ADDR1      "/data/EngineUpEnable.txt"       
#define FLASH_EngineUpEnable_ADDR2      "/media/card/EngineUpEnable.txt"       

#define FLASH_GPSTimeOutTimer_ADDR1     "/media/card/gps_timeout_timer1.txt"
#define FLASH_GPSTimeOutTimer_ADDR2     "/data/gps_timeout_timer2.txt"

#define FILE_ZX_BLIND_ZONE_DATA_ADDR	"/media/card/zx_blind_zone_data.txt"  // 存储zx盲区数据
#define FILE_ZX_BLIND_ZONE_PARA_ADDR	"/media/card/zx_blind_zone_para.txt"  // 存储zx盲区参数

#define FILE_ZXEP_BLIND_ZONE_DATA_ADDR	"/media/card/zxep_blind_zone_data.txt"  // 存储zxep盲区数据
#define FILE_ZXEP_BLIND_ZONE_PARA_ADDR	"/media/card/zxep_blind_zone_para.txt"  // 存储zxep盲区参数

#define FILE_BJEP_BLIND_ZONE_DATA_ADDR	"/media/card/bjep_blind_zone_data.txt"  // 存储bjep盲区数据
#define FILE_BJEP_BLIND_ZONE_PARA_ADDR	"/media/card/bjep_blind_zone_para.txt"  // 存储bjep盲区参数

#define FLASH_Mileage_ADDR1             "/media/card/mileage1.txt"
#define FLASH_Mileage_ADDR2             "/data/mileage2.txt"

#define FLASH_PAGEADDR_WORKTIME1		"/media/card/worktime1.txt"	      //工作小时存储页地址1
#define FLASH_PAGEADDR_WORKTIME2		"/data/worktime2.txt"	          //工作小时存储页地址2



//远程升级
#define FLASH_FIRMWARE_Core_MCU         "/data/helloworld_bak"     //核心板应用升级程序地址 

#define FLASH_FIRMWARE_Auxi_MCU         "/data/Auxi_MCU"     //辅助处理器固件保持地址(Auxiliary)
#define FLASH_FIRMWARE_Vehicle_MCU      "/media/card/Vehicle_MCU"  //车辆控制器

#define FLASH_CAN_FILE_CFG              "/data/WLRC_CANFile.txt"


uint8 WriteToFlash(char *Pathname ,uint32 Length, uint8 *p);
uint8 ReadFromFlash(char *Pathname ,uint32 Length, uint8 *Buf);
uint8 WriteCanToFlash(char *Pathname ,uint32 Length, uint8 *p);
uint8 SaveCanToFlash(uint32 Length, uint8 *p);
uint8 ReadFromWholFileFlash(char *Pathname ,uint32 Length, uint8 *Buf);
uint8 ReadFromlseekFlash(char *Pathname ,uint32 BufAddr, uint32 Length, uint8 *Buf);

#endif










