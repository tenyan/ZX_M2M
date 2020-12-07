/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: CollectModule.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪCollect����ģ�����ж���ӿڲ��ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-13, by , �������ļ�
 *
 */
#ifndef _CollectModule_H
#define _CollectModule_H

#include "CollectHW.h"
//-----��������----------------------------------------------------------------
#define TASK_Collect_STK_SIZE                   300
#define TASK_Collect_PRIO                       11
#define TASK_Collect_ID                         11


//P1
#define BAT_POW           (1UL << 18)
#define RING_PIN           (1UL << 6)
#define ACC_PIN           (1UL << 9)

#define  AlarmOFF               0x01                //�������عر�  
#define  AlarmON                0x02                //�������ش�
#define  KeyOFF                 0x03                //Կ�׿���ACC�ر�  
#define  KeyON                  0x04                //Կ�׿���ACC��
#define  EngworkingOFF          0x05                //Сʱ�ƿ��عر�  
#define  EngworkingON           0x06                //Сʱ�ƿ��ش�
#define  PositiveOFF            0x07                //��ӵ������ر�  
#define  PositiveON             0x08                //��ӵ�������
#define  NegativeOFF            0x09                //��ӵ縺���ر�  
#define  NegativeON             0x0a                //��ӵ縺����
#define  HulledOFF              0x0b                //�������ر�  
#define  HulledON               0x0c                //��������

#define  BOXAlarm               0x10                //�����쳣  
#define  BOXNormal              0x11                //��������  

/*�ϱ�ԭ�����Ͷ���*/

#define GET5V                    0x20     	//��繩��
#define NO5V                     0x21     	//��ع���
#define PowerLow                 0x22     	//��Դ��ѹ��
#define PowerNormal              0x23     	//��Դ��ѹ����
#define Battery_Charge           0x24     	//��س��
#define Battery_NOCharge         0x25    	//��ز����


//�ɼ�״̬
typedef struct STU_Sysstruct
{
    uint32 uiWorkingTime;			//Сʱ�ƿ���Ϊ�ߵ�ƽʱ��ʼͳ�ƣ�Ϊ�͵�ƽʱֹͣͳ��,��λ:s
    uint8 ucBAT_Voltage;			//�ڲ���ص�ѹ
    uint16 usInput_Voltage;			//���Դ��ѹ
    uint16 usStandby_Voltage; 		//����ģ�����ɼ�
    uint8 ucGPS_Temperature;		//GPS�ڲ��¶�
    uint8 ucSwitch1;				//�������ɼ�1
								    //B0����������    0=�͵�ƽ,1=�ߵ�ƽ
								    //B1��Կ�׿���ACC     0=�ر�,1=��
								    //B2��Сʱ�ƿ���  0=�ر�,1=�򿪣�
								    //B3����ӵ�����״̬  0=�Ͽ�,1=��ͨ
								    //B4����ӵ縺��״̬ 0=��ͨ,1=�Ͽ�
								    //B5��������״̬  0=��ͨ,1=�Ͽ�
								    //B6��GSM���߱���   0=�ޱ���, 1=����
								    //B7������λ
    
    uint8 ucSwitch2;				//�������ɼ�2
								    //B0���̵���1����״̬ 0=δ����;1=����
								    //B1���̵���2����״̬ 0=δ����;1=����
								    //B2������
								    //B3�����б��� 0=����;1=����
								    //B4������
								    //B5��RS232ͨѶ״̬ 0= δͨѶ��1=ͨѶ��
								    //B6��RS232ͨѶ�Ǳ���ʾ״̬ 0= δͨѶ��1=ͨѶ
								    //B7��CANͨѶ״̬ 0= δͨѶ��1=ͨѶ��
    uint8 ucSwitch3;				//�������ɼ�3
								    //B0������״̬ 0=��繩�磻1=��ع��磻
								    //B1���������״̬   0=�����磻1=����
								    //B2���ڲ���س��״̬    0=δ���,1=��磻
								    //B3��������ʻ�ٶȱ��� 0=�ٶ�������1=���ٱ�����
								    //B4������λ��Խ�籨�� 0=��Խ�磻1= Խ�籨����
								    //B5�����õ�ص�ѹ�� 0=�����磻1=����
								    //B6������
								    //B7������

    uint8  DisassmbleSwitch;		//�����࿪����
								    //B0��EGND(�����˿)       0=EGND OFF,   1=EGND ON
								    //B1�����б���1            0=����,       1=����
								    //B2�����б���2            0=����,       1=����
								    //B3��GSM���߱���,Reserved
								    //B4: GSM���Ǽ��          0=����,       1=���
								    //B5:GPS���߶�·		   0=��·,       1=����
								    //B6:GPS���߿�·		   0=����,       1=��·
								    //B7~B7:Reserved
}STU_Sysstruct,*PSTU_Sysstruct;


struct STU_ADstruct
{
    uint16 usInput_Voltage[10];
    uint16 usVBAT[10];
    uint16 usAD1[10];
    uint16 usAD2[10];
};	

//-----�ⲿ����----------------------------------------------------------------


uint16 GetInput_Voltage(void);
uint8  GetBAT_Voltage(void);
void   SetInputVol(void);
void   SetBatVol(void);
uint8  GetGpsSwitchState(void);
uint8  GetGpsAntOpenState(void);
uint8  GetGpsAntShortState(void);
uint8  GetSwitch1State(void);
uint8  GetSwitch2State(void);
uint8  GetSwitch3State(void);
uint8  GetAccState(void);
uint8  GetPwrSupplyState(void);
uint8  GetPwrLowState(void);
uint8  GetBoxOpenState(void);
uint8  GetBatChargeState(void);
uint8  GetBatVolLowState(void);
void   SaveWorktime(uint32 uiworktime);
uint32 GetWorkingTime(void);
void   Collect_TimerCount_Delay(void);
void SetWorkingTime(uint32 uiWorkTime);
uint8 BuildAD_Switch(uint8 *buf);
void   TaskCollect (void *pData);
#endif

