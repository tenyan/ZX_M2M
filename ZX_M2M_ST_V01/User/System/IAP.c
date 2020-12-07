/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: IAP.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪSystem����ģ��Զ�̹̼�����������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by  , �������ļ�
 *
 */
 
   
#define _IN_IAP_H_  

//-----�ļ�ͷ����--------------------------------------------------------------
#include "config.h"
#include "includes.h"
#include "stm32f2xx.h"
#include "IAP.h" 


//-----�ⲿ��������------------------------------------------------------------

extern uint8 Public_Buf[]; 
extern stuFirmwareUpdate FirmwareUpdate;

#define KR_KEY_Reload_iap    ((uint16_t)0xAAAA)
#define KR_KEY_Enable_iap     ((uint16_t)0xCCCC)
//-----�ڲ���������------------------------------------------------------------

//-----�ڲ���������------------------------------------------------------------



//-----�ڲ���������------------------------------------------------------------

void IWDG_ReloadCounter_IAP(void)
{
  IWDG->KR = KR_KEY_Reload_iap ;
}
//-----�ⲿ��������------------------------------------------------------------


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
    		ReadFromFlash(UPGRADE_ADDR+m,0,256, &atempbff[0]);  //��ȡ����
    		
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


    IWDG_ReloadCounter_IAP();//ι��0418
  	FLASH_Unlock();          //��������
  	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//FLASH_EraseAllPages();//ɾ��FLASH����ҳ����
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
		IWDG_ReloadCounter_IAP();//ι��
		FLASH_EraseSector(FLASH_Sector_0+8*i, VoltageRange_3);
		IWDG_ReloadCounter_IAP();//ι��
	}
	
	
	for (i=0;  i < FirmwareUpdate.usPackets; i++)
	{
		ReadFromFlash(FLASH_PAGEADDR_UPGRADE+4*i,0,FIRMWARE_PACKET_LEN,&tempbuf[0]);
		RamToFlash(Dest_addr,tempbuf,FIRMWARE_PACKET_LEN);
		Dest_addr += FIRMWARE_PACKET_LEN;
		IWDG_ReloadCounter_IAP();//ι��
	}
	while(1)//�����������ϵͳ
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
	
    IWDG_ReloadCounter_IAP();//ι��0418
  	FLASH_Unlock();          //��������
  	FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//FLASH_EraseAllPages();//ɾ��FLASH����ҳ����
	FLASH_EraseAllSectors(VoltageRange_3);
	
	for ( i=0;  i < FirmwareUpdate.usPackets; i++)
	{
		ReadFromFlash(FLASH_PAGEADDR_UPGRADE+4*i,0,FIRMWARE_PACKET_LEN,&tempbuf[0]);
		RamToFlash(Dest_addr,tempbuf,FIRMWARE_PACKET_LEN);
		Dest_addr += FIRMWARE_PACKET_LEN;
		IWDG_ReloadCounter_IAP();//ι��
	}
	while(1)//�����������ϵͳ
    {

    }
	return 1;
}
#endif


/*  
*******************************************************************************************************  
** �������ƣ�RamToFlash()  
** �������ܣ�����RAM�����ݵ�FLASH��  
** ��ڲ�����dst        Ŀ���ַ����FLASH��ʼ��ַ��  
**           src        Դ��ַ����RAM��ַ����ַ�����ֶ���  
**           no         �����ֽڸ�����Ϊ512/1024/4096/8192  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  RamToFlash(uint32 dst, uint8 *src, uint32 no)   
{   
    volatile FLASH_Status FLASHStatus=FLASH_COMPLETE;	
	uint32 EndAddr=dst+no;//*4;	
//	uint32 NbrOfPage = 0;	
	uint32 EraseCounter = 0x0, Address = 0x0;
	uint32 i,j;
	//int MemoryProgramStatus=1;//Ϊһ��ͨ��
	uint32 temp = 0;	
	
	FLASH_Unlock();          //��������
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGSERR | FLASH_FLAG_PGAERR| FLASH_FLAG_PGPERR| FLASH_FLAG_WRPERR| FLASH_FLAG_OPERR);			
	//��ʼд����
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
** �������ƣ�EraseSector()  
** �������ܣ������������������52��  
** ��ڲ�����sec1       ��ʼ����  
**           sec2       ��ֹ����  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  EraseSector(uint8 sec1, uint8 sec2)   
{     
   /* paramin[0] = IAP_ERASESECTOR;            // ����������    
    paramin[1] = sec1;                       // ���ò���    
    paramin[2] = sec2;   
    paramin[3] = IAP_FCCLK;   
    (*IAP_Entry)(paramin, paramout);         // ����IAP�������    
      
    return(paramout[0]);                     // ����״̬��    */
}    
   
/*  
*******************************************************************************************************  
** �������ƣ�BlankCHK()  
** �������ܣ�������գ��������53��  
** ��ڲ�����sec1       ��ʼ����  
**           sec2       ��ֹ����  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  BlankCHK(uint8 sec1, uint8 sec2)   
{     
   /* paramin[0] = IAP_BLANKCHK;               // ����������    
    paramin[1] = sec1;                       // ���ò���    
    paramin[2] = sec2;   
    (*IAP_Entry)(paramin, paramout);         // ����IAP�������    
   
    return(paramout[0]);                     // ����״̬��    */
}   
   
/*  
*******************************************************************************************************  
** �������ƣ�ReadParID()  
** �������ܣ�������գ��������53��  
** ��ڲ�������  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  ReadParID(void)   
{     
  /*  paramin[0] = IAP_READPARTID;             // ����������    
    (*IAP_Entry)(paramin, paramout);         // ����IAP�������    
   
    return(paramout[0]);                     // ����״̬��    */
}   
   
/*  
*******************************************************************************************************  
** �������ƣ�BootCodeID()  
** �������ܣ���ȡboot����汾�ţ��������55��  
** ��ڲ�������  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  BootCodeID(void)   
{     
 /*   paramin[0] = IAP_BOOTCODEID;             // ����������    
    (*IAP_Entry)(paramin, paramout);         // ����IAP�������    
   
    return(paramout[0]);                     // ����״̬��    */
}   
   
/*  
*******************************************************************************************************  
** �������ƣ�Compare()  
** �������ܣ�У�����ݣ��������56��  
** ��ڲ�����dst        Ŀ���ַ����RAM/FLASH��ʼ��ַ����ַ�����ֶ���  
**           src        Դ��ַ����FLASH/RAM��ַ����ַ�����ֶ���  
**           no         �����ֽڸ����������ܱ�4����  
** ���ڲ�����IAP����״̬��  
**           IAP����ֵ(paramout������)  
*******************************************************************************************************  
*/   
uint32  Compare(uint32 dst, uint32 src, uint32 no)   
{     
   /* paramin[0] = IAP_COMPARE;                // ����������    
    paramin[1] = dst;                        // ���ò���    
    paramin[2] = src;   
    paramin[3] = no;   
    (*IAP_Entry)(paramin, paramout);         // ����IAP�������    
   
    return(paramout[0]);                     // ����״̬��    */
}   
   
/*  
*******************************************************************************************************  
** ��������:IAP_Iint()  
** �������ܣ�IAP�������ã�����  
*******************************************************************************************************  
*/   
/*void IAP_Iint(void)   
{   
    IAP_Entry = (void(*)())IAP_ENTER_ADR;           // ��ʼ������ָ��IAP_Entry    
}   
*/
#endif	


//-----�ļ�IAP.c����---------------------------------------------

