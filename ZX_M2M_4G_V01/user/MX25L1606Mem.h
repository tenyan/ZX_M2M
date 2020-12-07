/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
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
 * 2019-03-10, by  �������ļ�
 *
 */

#ifndef _MX25L1606Mem_H
#define _MX25L1606Mem_H


#define FILE_SIZE                       0x100000                    // �����ļ��洢��С��λ�ֽ� 1M 0x100000 

#define EMMC_FILE_PATH           	    "/media/card/"              //can���ݴ洢�ļ�

#define FLASH_PAGEADDR_CAN            	"/media/card/can2.txt"      //can���ݴ洢�ļ�

#define FLASH_PAGEADDR_PARAMET1         "/media/card/paramet1.txt"  //���ò����洢ҳ���ַ1  
#define FLASH_PAGEADDR_PARAMET2         "/data/paramet2.txt"        //���ò����洢ҳ���ַ2  
#define FLASH_PAGEADDR_MCU1				"/media/card/mcu1.txt" 		//MCU��ز�����ҳ���ַ1
#define FLASH_PAGEADDR_MCU2				"/data/mcu2.txt" 			//MCU��ز�����ҳ���ַ1

#define FLASH_EngineType_ADDR			"/data/engine_type.txt" 	//�洢���������������������͵�ַ

#define FLASH_ProtocolId_ADDR1			"/media/card/protocol_id.txt"  //�洢�����������ĳ���Э����
#define FLASH_ProtocolId_ADDR2			"/data/protocol_id.txt" 	    //�洢�����������ĳ���Э����

#define FLASH_VIN_ADDR1			        "/media/card/vin.txt"       //�洢ƽ̨�·��ĳ���ʶ����(VIN)
#define FLASH_VIN_ADDR2			        "/data/vin.txt" 	        //�洢ƽ̨�·��ĳ���ʶ����(VIN)

#define FLASH_EngineUpEnable_ADDR1      "/data/EngineUpEnable.txt"       
#define FLASH_EngineUpEnable_ADDR2      "/media/card/EngineUpEnable.txt"       

#define FLASH_GPSTimeOutTimer_ADDR1     "/media/card/gps_timeout_timer1.txt"
#define FLASH_GPSTimeOutTimer_ADDR2     "/data/gps_timeout_timer2.txt"

#define FILE_ZX_BLIND_ZONE_DATA_ADDR	"/media/card/zx_blind_zone_data.txt"  // �洢zxä������
#define FILE_ZX_BLIND_ZONE_PARA_ADDR	"/media/card/zx_blind_zone_para.txt"  // �洢zxä������

#define FILE_ZXEP_BLIND_ZONE_DATA_ADDR	"/media/card/zxep_blind_zone_data.txt"  // �洢zxepä������
#define FILE_ZXEP_BLIND_ZONE_PARA_ADDR	"/media/card/zxep_blind_zone_para.txt"  // �洢zxepä������

#define FILE_BJEP_BLIND_ZONE_DATA_ADDR	"/media/card/bjep_blind_zone_data.txt"  // �洢bjepä������
#define FILE_BJEP_BLIND_ZONE_PARA_ADDR	"/media/card/bjep_blind_zone_para.txt"  // �洢bjepä������

#define FLASH_Mileage_ADDR1             "/media/card/mileage1.txt"
#define FLASH_Mileage_ADDR2             "/data/mileage2.txt"

#define FLASH_PAGEADDR_WORKTIME1		"/media/card/worktime1.txt"	      //����Сʱ�洢ҳ��ַ1
#define FLASH_PAGEADDR_WORKTIME2		"/data/worktime2.txt"	          //����Сʱ�洢ҳ��ַ2



//Զ������
#define FLASH_FIRMWARE_Core_MCU         "/data/helloworld_bak"     //���İ�Ӧ�����������ַ 

#define FLASH_FIRMWARE_Auxi_MCU         "/data/Auxi_MCU"     //�����������̼����ֵ�ַ(Auxiliary)
#define FLASH_FIRMWARE_Vehicle_MCU      "/media/card/Vehicle_MCU"  //����������

#define FLASH_CAN_FILE_CFG              "/data/WLRC_CANFile.txt"


uint8 WriteToFlash(char *Pathname ,uint32 Length, uint8 *p);
uint8 ReadFromFlash(char *Pathname ,uint32 Length, uint8 *Buf);
uint8 WriteCanToFlash(char *Pathname ,uint32 Length, uint8 *p);
uint8 SaveCanToFlash(uint32 Length, uint8 *p);
uint8 ReadFromWholFileFlash(char *Pathname ,uint32 Length, uint8 *Buf);
uint8 ReadFromlseekFlash(char *Pathname ,uint32 BufAddr, uint32 Length, uint8 *Buf);

#endif










