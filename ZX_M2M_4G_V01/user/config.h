
/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
 * All right reserved
 *
 * �ļ�����: Config.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �û�����ͷ�ļ�����
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2020-11-24,  �������ļ�
 *
 */

#ifndef CFG_20190306_H
#define CFG_20190306_H


#define SW_VERSION_LEN	           23
#define SW_VERSION		  "WLRC_ZX_M2M_201125_V001"		//����汾��
/*******************************************************************
WLRCWJ_4G_200222_V002: 2020-02-22
                       1)���ӽ��յ�PC����ƽ̨�·��Ĳ�����������ת����Э������
WLRCWJ_4G_200415_V003: 2020-04-15
                       1)����һ�汾���������ڻ�4G�µ�����ͨѶЭ��
WLRC_WJ_2CAN_200522_V003:2020-05-22
                       1)����CAN��Ƶ�������úͲ�ѯ
WLRC_WJ_2CAN_200613_V001:2020-06-13
WLRC_WJ_2CAN_200728_V002:
                       1)�����ź�ǿ�Ȳ�ѯ����ж�,�������������ִ��
                       2)����at+creg?��ѯ����ж�,�������������ִ��
                       3)�޸�AT+CGPADDR����IP��ַ���жϷ���,�յ�����һ·ip���϶�Ϊ�ɹ�
                       4)��\Э������Զ��������ַ��ֲ��data·����      
WLRC_WJ_2CAN_200804_V002:
                       1)����Զ����������֪ͨSTЭ��������ֹ�������߹���
WLRC_WJ_2CAN_200929_V006:
                       1)�޶�MAX_CAN_FRAME_NUM ��56���Ϊ90
                       2)ȥ��0x030F����TLV
WLRC_WJ_2CAN_201112_V007:
                       1)����<4G�ն�Э��B2>����Э��������CAN֡
                       2)��ȡ������������ϲɼ����������汾��
                       3)��ȡ����˹����ʮ�巢������Ϣ
                       4)��ȡ���������������Ϣ
                       5) ucOpenTimes++;�������ATָ��֮ǰ
WLRC_ZX_M2M_201125_V001  :2020-11-25
                       1)���ڻ�WLRC_WJ_2CAN_201112_V007�汾�������޸�Ϊ����M2M�汾
********************************************************************/
#define HW_VERSION		     0x0102					//Ӳ���汾��V1.2	
#define BIT(x)	         	((uint32)1 << (x))
#define OS_TICKS_PER_SEC     1000u   /* Set the number of ticks in one second                        */

/*********************************************************************************************************
  Date types(Compiler specific)  �������ͣ��ͱ�������أ�                
*********************************************************************************************************/
//typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

typedef unsigned char  BOOLEAN;

typedef unsigned char  uint8;                   // �޷���8λ���ͱ���  
typedef signed   char  int8;                    // �з���8λ���ͱ���  
typedef unsigned short uint16;                  // �޷���16λ���ͱ��� 
typedef signed   short int16;                   // �з���16λ���ͱ��� 
typedef unsigned int   uint32;                  // �޷���32λ���ͱ��� 
typedef signed   int   int32;                   // �з���32λ���ͱ��� 
typedef float          fp32;                    // �����ȸ�������32λ���ȣ� 
typedef double         fp64;                    // ˫���ȸ�������64λ���ȣ�
typedef unsigned char  BOOL; 


typedef unsigned char  uint8_t;                   // �޷���8λ���ͱ���  
typedef signed   char  int8_t;                    // �з���8λ���ͱ���  
typedef unsigned short uint16_t;                  // �޷���16λ���ͱ��� 
typedef signed   short int16_t;                   // �з���16λ���ͱ��� 
typedef unsigned int   uint32_t;                  // �޷���32λ���ͱ��� 
typedef signed   int   int32_t;                   // �з���32λ���ͱ��� 
//typedef float          fp32;                    // �����ȸ�������32λ���ȣ� 
//typedef double         fp64;                    // ˫���ȸ�������64λ���ȣ�
//typedef unsigned char  BOOL; 


#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

#define GLOBAL	 extern	
//#define VIRTUAL_TIMER_EN	0	//�����ʱ��ʹ��, 0:����, 1:ʹ��

#include <fcntl.h>
#include <termios.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>   
#include <limits.h> 
#include <asm/ioctls.h>
#include <time.h>
#include <unistd.h>
#include "pthread.h"
#include <syslog.h>
#include <stdint.h>
#include <sys/poll.h>

#include <linux/input.h>
/*-------------------------------    */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>
//#include  "includes.h"
#include  "BaseFun.h"
#include  "SystemModule.h"
#include  "SystemCommand.h"
#include  "GsmModule.h"
#include  "GpsModule.h"
#include  "CollectModule.h"
#include  "Mcu.h"
#include  "PcDebug.h"
#include  "SystemWorkMode.h"
#include  "rtc.h"
#include  "MX25L1606Mem.h"
#include  "amx8xx.h"
#include  "McuUpload.h"
#include  "Gpio_cfg.h"
#include  "Watchdog.h"
#include  "A5UART0_MCU.h"
#include  "Canconfig.h"
//#include  <tcp_client.h>

#endif




