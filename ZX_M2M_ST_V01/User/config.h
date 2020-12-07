/*
 * Copyright(c)2012, �칤��Ϣ�����ɷ����޹�˾��������ҵ��
 * All right reserved
 *
 * �ļ�����: App_cfg.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �û�����ͷ�ļ�����
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2012-06-04,  �������ļ�
 *
 */

#ifndef CFG_20200604_H
#define CFG_20200604_H

#define SW_VERSION_LEN	           24
#define SW_VERSION		  "AUX_MCU_WJ_G4_V03_201015"		//����汾�� 
/*
AUX_MCU--- STЭ����������
G4     --- ���ķ�����,�ն˲���2·CAN
WJ     --- �ڻ�
*/
/*******************************************************************
4G_MCU_WJ_190502:          2019-5-2 
      1)���ڻ�LRC�ն�CAN����(LRC_WJCAN_P102_190316)�����ϸ���Ϊ4G�ն�Э����������
AUX_MCUWJ_V191215:         2019-12-14
      1)�޸�4Gģ�鿪�ػ����뱣֤���ϵ�ʱ��Ҫ�����ػ�����
AUX_MCUWJ2CAN_V200402      2020-04-02
      1)�ڻ�2·CANͨѶЭ��
AUX_MCU_WJ_G4_V02_200402   2020-05-16
      1)���Ӹ�Ƶ�ɼ�CAN���ù���
AUX_MCU_WJ_G4_V02_200731
      1)���ӽ�������������֪ͨ,�����ڼ��ֹ��������
AUX_MCU_WJ_G4_V03_201015
      1)����CAN2ָʾ��״̬
      2)AddCanFrameToBuff�����滻ΪA5_AddCanFrameToBuff����
      3)�������ն�ID��ŵ�ʱ��ͬʱ����Ӳ��
********************************************************************/
#define HW_VERSION		 0x0003					//Ӳ���汾��V0.3	
/*����Ӳ���汾��������STЭ�������̼��汾
  HW_VERSION ���ֽڴ��� �̼���汾��(��ͬ���ҡ���ͬ���͵�)�����ֽڴ���汾����
  ��汾�ŷ���:0-�ڻ��д��λ,1-���,2-����
*/
#define BIT(x)	       	((uint32)1 << (x))

/*********************************************************************************************************
  Date types(Compiler specific)  �������ͣ��ͱ�������أ�                
*********************************************************************************************************/
typedef unsigned char  uint8;                   // �޷���8λ���ͱ���  
typedef signed   char  int8;                    // �з���8λ���ͱ���  
typedef unsigned short uint16;                  // �޷���16λ���ͱ��� 
typedef signed   short int16;                   // �з���16λ���ͱ��� 
typedef unsigned int   uint32;                  // �޷���32λ���ͱ��� 
typedef signed   int   int32;                   // �з���32λ���ͱ��� 
typedef float          fp32;                    // �����ȸ�������32λ���ȣ� 
typedef double         fp64;                    // ˫���ȸ�������64λ���ȣ�
typedef unsigned char  BOOL; 


#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#define GLOBAL	 extern	
#define VIRTUAL_TIMER_EN	0	//�����ʱ��ʹ��, 0:����, 1:ʹ��

#define osWaitForever   0

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>
#include  "includes.h"
#include  "BaseFun.h"
#include  "SystemModule.h"
#include  "systemcommand.h"
#include  "GsmModule.h"
#include  "CollectModule.h"
#include  "Mcu.h"
#include  "PcDebug.h"
#include  "SystemWorkMode.h"
#include  "rtc.h"
#include  "MX25L1606Mem.h"
#include  "amx8xx_v00.h"
#include  "McuUpload.h"
#include  "A5_Com1ProtocolLayer.h"
#include  "GpsModule.h"
#include  "A5_Com1HW.h"   
#include  "McuChQiUpload.h"
#include  "i2c.h"

#endif
