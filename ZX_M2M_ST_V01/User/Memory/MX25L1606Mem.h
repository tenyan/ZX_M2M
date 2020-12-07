/*
 * Copyright(c)2019, 硬件研发部
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
 * 2019-04-10, by lxf, 创建本文件
 *
 */

#ifndef _MX25L1606_H
#define _MX25L1606_H

#define TS25L16CAPACITY           0x3FFFFF        //FLASH存储容量
#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_CS_LOW() 		 (GPIO_ResetBits(GPIOB, GPIO_Pin_9))  // 打开Flash Memory片选
#define sFLASH_CS_HIGH() 		 (GPIO_SetBits(GPIOB, GPIO_Pin_9))    // 关闭Flash Memory片选
#define FLASHMEMCAPACITY         0x1FFFFF            // FLASH存储容量

#define FLASHMEMRDOUTTIME        25                    // 存储器读写操作超时时间
#define FLASHMEMSTOUTTIME        100                   // 读取状态寄存器超时时间


#define FLASH_PAGEADDR_PARAMET1            	1                   //配置参数存储页面地址1  0~3
#define FLASH_PAGEADDR_PARAMET2            	17                  //配置参数存储页面地址2  16~19
#define FLASH_PAGEADDR_MCU1					32					//MCU相关参数储页面地址1
#define FLASH_PAGEADDR_MCU2					48					//MCU相关参数储页面地址1

#define FLASH_PAGEADDR_WORKTIME1			64					//工作小时存储页地址1
#define FLASH_PAGEADDR_WORKTIME2			80					//工作小时存储页地址2
//远程升级和盲区保存暂不做处理
//#define FLASH_PAGEADDR_IMSI                	7                    // IMSI存储地址
#define FLASH_PAGEADDR_UPGRADE             	128                 // 升级地址 

#define FLASH_Firmware_MCU                  928                 //MCU固件保持地址 最大512K
#define FLASH_KCMCU_PARAMETER_ADDR          928+4096
void    Flash_SPI_Config(void);
void    SPISleep(void);
void    SPIWake(void);

uint8   WriteInOnePage(uint16 PageAddr, uint8 PageOffsetAddr, uint16 Length, uint8 *p);
uint8   WriteToFlash(uint16 PageAddr, uint8 PageOffsetAddr, uint32 Length, uint8 *p);
uint8   ReadFromFlash(uint16 PageAddr,uint8 PageOffsetAddr,uint32 Length, uint8 *Buf);

uint8   PageErase(uint16 Page);
uint8   SectorErase(uint16 Sector);
uint8   BlockErase(uint8 Block);
uint8   ChipErase(void);

#endif










