/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: IAP.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为System功能模块远程固件升级处理文件的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, 创建本文件
 *
 */

#ifndef _IAP_H 
#define	_IAP_H 
 
#ifdef _IN_IAP_H_ 
#define	EXT_IAP 
#else 
#define	EXT_IAP extern  
#endif 
 
#define	EMPTY					(0xff) 
 
 
//** LPC 2136 IAP Flash Address **// 
 
 
// Sector      Address 
//  0          0x0000 0000 - 0x0000 0fff   
//  1          0x0000 1000 - 0x0000 1fff 
//  2          0x0000 2000 - 0x0000 2fff 
//  3          0x0000 3000 - 0x0000 3fff 
//  4          0x0000 4000 - 0x0000 4fff 
//  5          0x0000 5000 - 0x0000 5fff 
//  6          0x0000 6000 - 0x0000 6fff 
//  7          0x0000 7000 - 0x0000 7fff 
 
// 0 - 7 Sector 4K Byte Per Sector 
 
//   8          0x0000 8000 - 0x0000 ffff 
//   9          0x0001 0000 - 0x0001 7fff 
//  10          0x0001 8000 - 0x0001 ffff 
 
 
//  11          0x0002 0000 - 0x0002 7fff 
//  12          0x0002 8000 - 0x0002 ffff 
//  13          0x0003 0000 - 0x0003 7fff 
//  14          0x0003 8000 - 0x0003 ffff 
 
 
#define IAP_ENTER_ADR           0x7FFFFFF1    // IAP入口地址定义 
 
// 定义IAP命令字  
                                    //   命令           参数 
#define     IAP_SELECTOR        50  // 选择扇区     【起始扇区号、结束扇区号】 
#define     IAP_RAMTOFLASH      51  // 拷贝数据     【FLASH目标地址、RAM源地址、写入字节数、系统时钟频率】 
#define     IAP_ERASESECTOR     52  // 擦除扇区     【起始扇区号、结束扇区号、系统时钟频率】 
#define     IAP_BLANKCHK        53  // 查空扇区     【起始扇区号、结束扇区号】 
#define     IAP_READPARTID      54  // 读器件ID     【无】 
#define     IAP_BOOTCODEID      55  // 读Boot版本号 【无】 
#define     IAP_COMPARE         56  // 比较命令     【Flash起始地址、RAM起始地址、需要比较的字节数】 
 
// 定义IAP返回状态字  
#define     CMD_SUCCESS          0 
#define     INVALID_COMMAND      1 
#define     SRC_ADDR_ERROR       2  
#define     DST_ADDR_ERROR       3 
#define     SRC_ADDR_NOT_MAPPED  4 
#define     DST_ADDR_NOT_MAPPED  5 
#define     COUNT_ERROR          6 
#define     INVALID_SECTOR       7 
#define     SECTOR_NOT_BLANK     8 
//#define     SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9 
#define     COMPARE_ERROR        10 
#define     BUSY                 11 
 
#define		BytePerPage			 256 
 
#define     UPGRADE_TIME        2700

//远程固件升级操作步骤子命令编号
#define     COMMID_GPS_GRADE_UNDERPACK_0     0   //下包
#define     COMMID_GPS_GRADE_CONFIRM_1       1   //确认GPS执行升级
#define     COMMID_GPS_GRADE_CANCEL_2        2   //取消GPS执行升级
#define     COMMID_GPS_GRADE_INQUIRY_3       3   //查询包的状态

// 定义CCLK值大小，单位为KHz  
#define  IAP_FCCLK              11059*4 
 
 
//-----结构定义----------------------------------------------------------------

/************************************************************************************************ 
**  Public Fuctions Declaration 
*************************************************************************************************/ 
//EXT_IAP uint32  SelSector(uint8 sec1, uint8 sec2); 
//EXT_IAP uint32  BlankCHK(uint8 sec1, uint8 sec2); 
//EXT_IAP uint32  BootCodeID(void); 
//EXT_IAP uint32  Compare(uint32 dst, uint32 src, uint32 no); 
//EXT_IAP uint32  EraseSector(uint8 sec1, uint8 sec2); 
EXT_IAP uint32  RamToFlash(uint32 dst, uint8 *src, uint32 no); 
//EXT_IAP uint32  ReadParID(void); 
//EXT_IAP void 	IAP_Iint(void); 
uint8  IAP_Start(void); 


#endif 



