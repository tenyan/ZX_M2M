/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: PcDebug.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪPcDebug����ģ��Э����ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by lxf, �������ļ�
 *
 */

#ifndef _PCDEBUG_H
#define _PCDEBUG_H
#include "PcHardware.h"
//-----��������----------------------------------------------------------------
#define TASK_PCDEBUG_STK_SIZE            150  
#define TASK_PCDEBUG_ID                  9   
#define TASK_PCDEBUG_PRIO                9   

#define COMM_READ_TIMEOUT                5             /*uart0 ���ڳ�ʱʱ��5��ʱ�ӽ���*/
#define UART0_RECV_BUFF_LENGTH           256           /*uart0 ����������󳤶�*/
  
#define FORMAT                           0xF9          /*���ó������ݰ�ͷ/��β*/


//�����������ģ��ѡ����  ������� PC_SendDebugData(uint8* ptr,uint16 usLen,uint8 ucModuleSelect)
#define  DEBUG_GSMMODULE                 1            //GSMģ��
#define  DEBUG_GPSMODULE                 2            //GPSģ��
#define  DEBUG_MCUMODULE                 3            //MCUģ��
#define  DEBUG_RS232MODULE               4            //MCU-RS232�ӿ�
#define  DEBUG_SYSMODULE                 5            //systemģ��
#define  DEBUG_ANYDATA                   10           //���ø�����ʱ����������������� 
#define  DEBUG_GPRS						 5			  //��ӡGPRS����
#define  DEBUG_AT                        1			  //��ӡATָ��

#define  PRINTBUFF                       100          //SMS��GPRS��ӡ���ݻ�����

#define  SMS_SEND                        1            //���ŷ��ͱ�ʶ
#define  SMS_RECV                        2            //���Ž��ձ�ʶ
#define  GPRS_SEND                       3            //GPRS���ݷ��ͱ�ʶ
#define  GPRS_RECV                       4            //GPRS���ݽ��ձ�ʶ
#define  GPRS_STO                        5            //GPRS���ݳ�ʱ���ͱ�ʶ



//-----�ṹ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
//void PC_Print(const uint8 *pucSrc, uint16 usLen, uint8 ucFlag);
void PC_SendDebugData(uint8* ptr,uint16 usLen,uint8 ucModuleSelect);   //�������������Ϣ�ĺ���,���Ա���ͬģ�����
//void PC_SendToPCData(uint8 *ptr, uint16 usLen);
void PC_SendToPCData2(uint8 *ptr, uint16 usLen);
void TaskPcDebug(void *pdata);
void A5_ExecRecvDebugData(uint8 *p, uint16 usLen);

#endif
