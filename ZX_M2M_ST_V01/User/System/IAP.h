/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: IAP.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪSystem����ģ��Զ�̹̼����������ļ���ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, �������ļ�
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
 
 
#define IAP_ENTER_ADR           0x7FFFFFF1    // IAP��ڵ�ַ���� 
 
// ����IAP������  
                                    //   ����           ���� 
#define     IAP_SELECTOR        50  // ѡ������     ����ʼ�����š����������š� 
#define     IAP_RAMTOFLASH      51  // ��������     ��FLASHĿ���ַ��RAMԴ��ַ��д���ֽ�����ϵͳʱ��Ƶ�ʡ� 
#define     IAP_ERASESECTOR     52  // ��������     ����ʼ�����š����������š�ϵͳʱ��Ƶ�ʡ� 
#define     IAP_BLANKCHK        53  // �������     ����ʼ�����š����������š� 
#define     IAP_READPARTID      54  // ������ID     ���ޡ� 
#define     IAP_BOOTCODEID      55  // ��Boot�汾�� ���ޡ� 
#define     IAP_COMPARE         56  // �Ƚ�����     ��Flash��ʼ��ַ��RAM��ʼ��ַ����Ҫ�Ƚϵ��ֽ����� 
 
// ����IAP����״̬��  
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

//Զ�̹̼���������������������
#define     COMMID_GPS_GRADE_UNDERPACK_0     0   //�°�
#define     COMMID_GPS_GRADE_CONFIRM_1       1   //ȷ��GPSִ������
#define     COMMID_GPS_GRADE_CANCEL_2        2   //ȡ��GPSִ������
#define     COMMID_GPS_GRADE_INQUIRY_3       3   //��ѯ����״̬

// ����CCLKֵ��С����λΪKHz  
#define  IAP_FCCLK              11059*4 
 
 
//-----�ṹ����----------------------------------------------------------------

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



