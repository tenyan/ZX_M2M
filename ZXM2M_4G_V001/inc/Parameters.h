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

/******************************************************************************
* MEMORY FILE DEFINE
******************************************************************************/
#define FILE_M2M_PARA_CPY1    "/media/card/m2m_para.txt"  // m2m���ò���
#define FILE_M2M_PARA_CPY2    "/data/m2m_para.txt"        // m2m���ò���
#define FILE_USER_LVC_CPY1    "/media/card/user_lvc.txt"  // �û��·�������ָ��
#define FILE_USER_LVC_CPY2    "/data/user_lvc.txt"        // �û��·�������ָ��
#define FILE_ENGINE_TYPE_CPY1    "/media/card/engine_type.txt"  // ���̷���������
#define FILE_ENGINE_TYPE_CPY2    "/data/engine_type.txt"        // ���̷���������
#define FILE_PID_CPY1    "/media/card/protocol_id.txt"  // ����Э������
#define FILE_PID_CPY2    "/data/protocol_id.txt"        // ����Э������
#define FILE_ODO_CPY1    "/media/card/odograph.txt" // ���
#define FILE_ODO_CPY2    "/data/odograph.txt"       // ���
#define FILE_USER_VIN_CPY1    "/media/card/user_vin.txt"  // �û��·��ĳ���ʶ����(VIN)
#define FILE_USER_VIN_CPY2    "/data/user_vin.txt"        // �û��·��ĳ���ʶ����(VIN)
#define FILE_WORKTIME_CPY1    "/media/card/worktime.txt"  // �豸�ۻ�����ʱ��(ACC ON)
#define FILE_WORKTIME_CPY2    "/data/worktime.txt"        // �豸�ۻ�����ʱ��(ACC ON)
#define FILE_OFFLINE_TIME_CPY1    "/media/card/offline_time.txt"  // �ն˲������ۻ�ʱ��
#define FILE_OFFLINE_TIME_CPY2    "/data/offline_time.txt"        // �ն˲������ۻ�ʱ��

#define FILE_ZXM2M_BZ_DATA   "/media/card/zxm2m_bz_data.txt"  // zxm2mä�������ļ�
#define FILE_ZXM2M_BZ_PARA   "/media/card/zxm2m_bz_para.txt"  // zxm2mä�������ļ�
#define FILE_HJEP_BZ_DATA    "/media/card/hjep_bz_data.txt"  // hjepä�������ļ�
#define FILE_HJEP_BZ_PARA    "/media/card/hjep_bz_para.txt"  // hjepä�������ļ�

#define FILE_4G_APP_FIRMWARE    "/data/helloworld_bak" // 4Gģ��Ӧ�ù̼�
#define FILE_ST_FIRMWARE    "/data/st_firmware"    // Э������ST�̼�
#define FILE_ECU_FIRMWARE    "/media/card/ecu_firmware"   // �����������̼�

#define FILE_ZXM2M_BZ_DATA_ADDR    "/media/card/zxm2m_bz_data.txt"  // �洢ZxM2mä������
#define FILE_ZXM2M_BZ_PARA_ADDR    "/media/card/zxm2m_blind_zone_para.txt"  // �洢ZxM2mä������

#define FILE_HJEP_BZ_DATA_ADDR    "/media/card/hjep_bz_data.txt"  // �洢hjepä������
#define FILE_HJEP_BZ_PARA_ADDR    "/media/card/hjep_bz_para.txt"  // �洢hjepä������

#define FILE_GBEP_BZ_DATA_ADDR    "/media/card/gbep_bz_data.txt"  // �洢gbepä������
#define FILE_GBEP_BZ_PARA_ADDR    "/media/card/gbep_bz_para.txt"  // �洢gbepä������

/******************************************************************************
* External variables
******************************************************************************/
typedef enum
{
  RFU_NOK = 0,
  RFU_OK  = 1,
} rfu_bool_t;

// BOOL����
enum
{
  PARM_FALSE = 0x00,
  PARM_TRUE = 0x01
};

#define PARM_DATA_HEADER    0x55AA5AA5

/******************************************************************************
* Function prototypes
******************************************************************************/
// �ļ�����IO
int8_t FileIO_Write(const char *file_name, uint8_t *pdata, uint32_t size);
int8_t FileIO_Read(const char *file_name, uint8_t *pbuf, uint32_t size);
int8_t FileIO_RandomRead(const char *file_name, uint32_t offset_addr, uint8_t *pbuf, uint32_t size);
int8_t FileIO_RandomWrite(const char *file_name, uint32_t offset_addr, uint8_t *pdata, uint32_t size);

// �û��ӿ�
void Parm_ResetM2mAssetDataToFactory(void);
void Parm_SaveM2mAssetData(void);
void Parm_ReadM2mAssetData(void);

void Parm_ReadVinInfo(void);
void Parm_SaveVinInfo(void);

void Parm_SaveLvcInfo(void);
void Parm_ReadLvcInfo(void);

void Parm_SaveOfflineTimeInfo(void);
void Parm_ReadOfflineTimeInfo(void);

void Parm_SaveWorkTimeInfo(void);
void Parm_ReadWorkTimeInfo(void);

// Զ�������̼��洢�ӿ�
uint8_t rfu_CheckNewFirmware(rfu_context_t* pThis,uint8_t* buffer, uint16_t bufferSize);
void rfu_EraseFlashHexFile(rfu_context_t* pThis);
void rfu_SaveFlashHexFile(rfu_context_t* pThis, uint8_t *buf, uint16_t length);
void rfu_ReadFlashHexFile(uint8_t file_dev, uint32_t address, uint8_t *buf, uint16_t length);

#endif /* _PARAMETERS_H_ */

