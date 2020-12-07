/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: IAP.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为System功能模块远程固件升级处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by  , 创建本文件
 *
 */
 
   
#define _IN_IAP_H_  

//-----文件头调用--------------------------------------------------------------
#include "config.h"
#include "includes.h"
#include "stm32f2xx.h"
#include "IAP.h" 


//-----外部变量定义------------------------------------------------------------

extern uint8 Public_Buf[]; 
extern stuFirmwareUpdate FirmwareUpdate;

#define KR_KEY_Reload_iap    ((uint16_t)0xAAAA)
#define KR_KEY_Enable_iap     ((uint16_t)0xCCCC)
//-----内部变量定义------------------------------------------------------------

//-----内部变量定义------------------------------------------------------------



//-----内部函数声明------------------------------------------------------------

void IWDG_ReloadCounter_IAP(void)
{
  IWDG->KR = KR_KEY_Reload_iap ;
}
//-----外部函数定义------------------------------------------------------------


#if 0
// from http://hi.baidu.com/regenlife/blog/item/3a78a556a977474a1138c21d.html
uint32 GetCrc32(void)
{
	uint32 k, crc;
    uint16 m=0, n=0;
	uint32 nDataLen = 0;
	uint8 atempbff[258];

	nDataLen = (stu_GMS_Upgrade.usPacketNum-1)*256+stu_GMS_Upgrade.usLastPacketLen;
	crc = 0XFFFFFFFF;
   
	for(k=0; k<nDataLen; k++)
	{
	 
	    if(!(k%256))
    		ReadFromFlash(UPGRADE_ADDR+m,0,256, &atempbff[0]);  //读取数据
    		
		crc = (crc >> 8) ^ crctab[(crc & 0xFF) ^ atempbff[n]];
		n++;
		if(n==256)
		{
			n = 0;
			m++;
		}
	}

	crc ^= 0xFFFFFFFF;

	return crc;
}
#endif

uint8 IAP_Start(void)
{    
	uint32 Dest_addr = 0x08000000;
	uint16 i,temp;
	uint8 *tempbuf = Public_Buf;


    IWDG_ReloadCounter_IAP();//喂狗0418
  	FLASH_Unlock();          //解锁函数
  	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//FLASH_EraseAllPages();//删除FLASH所有页内容
	//FLASH_EraseAllSectors(VoltageRange_3);
	
	if(FirmwareUpdate.usPackets<=64)
		temp = 3;
	else if(FirmwareUpdate.usPackets<=128)
		temp = 4;
	else if(FirmwareUpdate.usPackets<=1024)
		temp = 5 + (FirmwareUpdate.usPackets - 128 - 1)/128;
	else 
		return 0;
	
	for(i=0; i<=temp; i++)
	{
		IWDG_ReloadCounter_IAP();//喂狗
		FLASH_EraseSector(FLASH_Sector_0+8*i, VoltageRange_3);
		IWDG_ReloadCounter_IAP();//喂狗
	}
	
	
	for (i=0;  i < FirmwareUpdate.usPackets; i++)
	{
		ReadFromFlash(FLASH_PAGEADDR_UPGRADE+4*i,0,FIRMWARE_PACKET_LEN,&tempbuf[0]);
		RamToFlash(Dest_addr,tempbuf,FIRMWARE_PACKET_LEN);
		Dest_addr += FIRMWARE_PACKET_LEN;
		IWDG_ReloadCounter_IAP();//喂狗
	}
	while(1)//更新完毕重启系统
    {

    }
	return 1;
}


#if 0
uint8 IAP_Start(void)
{    
	uint32 Dest_addr = 0x08000000;
	uint16 i;
	uint8 *tempbuf = Public_Buf;
	
    IWDG_ReloadCounter_IAP();//喂狗0418
  	FLASH_Unlock();          //解锁函数
  	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//FLASH_EraseAllPages();//删除FLASH所有页内容
	FLASH_EraseAllSectors(VoltageRange_3);
	
	for ( i=0;  i < FirmwareUpdate.usPackets; i++)
	{
		ReadFromFlash(FLASH_PAGEADDR_UPGRADE+4*i,0,FIRMWARE_PACKET_LEN,&tempbuf[0]);
		RamToFlash(Dest_addr,tempbuf,FIRMWARE_PACKET_LEN);
		Dest_addr += FIRMWARE_PACKET_LEN;
		IWDG_ReloadCounter_IAP();//喂狗
	}
	while(1)//更新完毕重启系统
    {

    }
	return 1;
}
#endif


/*  
*******************************************************************************************************  
** 函数名称：RamToFlash()  
** 函数功能：复制RAM的数据到FLASH。  
** 入口参数：dst        目标地址，即FLASH起始地址。  
**           src        源地址，即RAM地址。地址必须字对齐  
**           no         复制字节个数，为512/1024/4096/8192  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  RamToFlash(uint32 dst, uint8 *src, uint32 no)   
{   
    volatile FLASH_Status FLASHStatus=FLASH_COMPLETE;	
	uint32 EndAddr=dst+no;//*4;	
//	uint32 NbrOfPage = 0;	
	uint32 EraseCounter = 0x0, Address = 0x0;
	uint32 i,j;
	//int MemoryProgramStatus=1;//为一是通过
	uint32 temp = 0;	
	
	FLASH_Unlock();          //解锁函数
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//开始写数据
	i=0;
	j=0;
	Address = dst;
	while((Address<EndAddr)&&(FLASHStatus==FLASH_COMPLETE))	
	{	
	    temp=src[0+j];
	    temp|=src[1+j]<<8;
		temp|=src[2+j]<<16;
		temp|=src[3+j]<<24;
	    FLASHStatus=FLASH_ProgramWord(Address,temp);
		Address=Address+4;
		j=j+4;
		temp=0;
	}

/*	Address = dst;
	i=0;
	while((Address < EndAddr) && (MemoryProgramStatus != 0))
	{	
		if((*(uint32*) Address) != src[i++])
		{		MemoryProgramStatus = 0;
				return 1;
		}
		Address += 4;
	}*/
	return 0; 
}   
#if 0   
/*  
*******************************************************************************************************  
** 函数名称：EraseSector()  
** 函数功能：扇区擦除，命令代码52。  
** 入口参数：sec1       起始扇区  
**           sec2       终止扇区  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  EraseSector(uint8 sec1, uint8 sec2)   
{     
   /* paramin[0] = IAP_ERASESECTOR;            // 设置命令字    
    paramin[1] = sec1;                       // 设置参数    
    paramin[2] = sec2;   
    paramin[3] = IAP_FCCLK;   
    (*IAP_Entry)(paramin, paramout);         // 调用IAP服务程序    
      
    return(paramout[0]);                     // 返回状态码    */
}    
   
/*  
*******************************************************************************************************  
** 函数名称：BlankCHK()  
** 函数功能：扇区查空，命令代码53。  
** 入口参数：sec1       起始扇区  
**           sec2       终止扇区  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  BlankCHK(uint8 sec1, uint8 sec2)   
{     
   /* paramin[0] = IAP_BLANKCHK;               // 设置命令字    
    paramin[1] = sec1;                       // 设置参数    
    paramin[2] = sec2;   
    (*IAP_Entry)(paramin, paramout);         // 调用IAP服务程序    
   
    return(paramout[0]);                     // 返回状态码    */
}   
   
/*  
*******************************************************************************************************  
** 函数名称：ReadParID()  
** 函数功能：扇区查空，命令代码53。  
** 入口参数：无  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  ReadParID(void)   
{     
  /*  paramin[0] = IAP_READPARTID;             // 设置命令字    
    (*IAP_Entry)(paramin, paramout);         // 调用IAP服务程序    
   
    return(paramout[0]);                     // 返回状态码    */
}   
   
/*  
*******************************************************************************************************  
** 函数名称：BootCodeID()  
** 函数功能：读取boot代码版本号，命令代码55。  
** 入口参数：无  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  BootCodeID(void)   
{     
 /*   paramin[0] = IAP_BOOTCODEID;             // 设置命令字    
    (*IAP_Entry)(paramin, paramout);         // 调用IAP服务程序    
   
    return(paramout[0]);                     // 返回状态码    */
}   
   
/*  
*******************************************************************************************************  
** 函数名称：Compare()  
** 函数功能：校验数据，命令代码56。  
** 入口参数：dst        目标地址，即RAM/FLASH起始地址。地址必须字对齐  
**           src        源地址，即FLASH/RAM地址。地址必须字对齐  
**           no         复制字节个数，必须能被4整除  
** 出口参数：IAP操作状态码  
**           IAP返回值(paramout缓冲区)  
*******************************************************************************************************  
*/   
uint32  Compare(uint32 dst, uint32 src, uint32 no)   
{     
   /* paramin[0] = IAP_COMPARE;                // 设置命令字    
    paramin[1] = dst;                        // 设置参数    
    paramin[2] = src;   
    paramin[3] = no;   
    (*IAP_Entry)(paramin, paramout);         // 调用IAP服务程序    
   
    return(paramout[0]);                     // 返回状态码    */
}   
   
/*  
*******************************************************************************************************  
** 函数名称:IAP_Iint()  
** 函数功能：IAP函数调用，测试  
*******************************************************************************************************  
*/   
/*void IAP_Iint(void)   
{   
    IAP_Entry = (void(*)())IAP_ENTER_ADR;           // 初始化函数指针IAP_Entry    
}   
*/
#endif	


//-----文件IAP.c结束---------------------------------------------

