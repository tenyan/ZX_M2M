/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: MX25L1606Mem.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �洢��MX25L1606����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-04-10, by lxf, �������ļ�
 *
 */

#ifndef _MX25L1606_H
#define _MX25L1606_H

#define TS25L16CAPACITY           0x3FFFFF        //FLASH�洢����
#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_CS_LOW() 		 (GPIO_ResetBits(GPIOB, GPIO_Pin_9))  // ��Flash MemoryƬѡ
#define sFLASH_CS_HIGH() 		 (GPIO_SetBits(GPIOB, GPIO_Pin_9))    // �ر�Flash MemoryƬѡ
#define FLASHMEMCAPACITY         0x1FFFFF            // FLASH�洢����

#define FLASHMEMRDOUTTIME        25                    // �洢����д������ʱʱ��
#define FLASHMEMSTOUTTIME        100                   // ��ȡ״̬�Ĵ�����ʱʱ��


#define FLASH_PAGEADDR_PARAMET1            	1                   //���ò����洢ҳ���ַ1  0~3
#define FLASH_PAGEADDR_PARAMET2            	17                  //���ò����洢ҳ���ַ2  16~19
#define FLASH_PAGEADDR_MCU1					32					//MCU��ز�����ҳ���ַ1
#define FLASH_PAGEADDR_MCU2					48					//MCU��ز�����ҳ���ַ1

#define FLASH_PAGEADDR_WORKTIME1			64					//����Сʱ�洢ҳ��ַ1
#define FLASH_PAGEADDR_WORKTIME2			80					//����Сʱ�洢ҳ��ַ2
//Զ��������ä�������ݲ�������
//#define FLASH_PAGEADDR_IMSI                	7                    // IMSI�洢��ַ
#define FLASH_PAGEADDR_UPGRADE             	128                 // ������ַ 

#define FLASH_Firmware_MCU                  928                 //MCU�̼����ֵ�ַ ���512K
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










