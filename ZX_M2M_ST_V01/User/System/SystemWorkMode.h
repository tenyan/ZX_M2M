/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: SystemWorkMode.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪSystem����ģ�鹤��ģʽ�����ļ���ͷ�ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-16, by lxf, �������ļ�
 *
 */

#ifndef _SYSTEMWORKMODE_H
#define _SYSTEMWORKMODE_H


//-----��������----------------------------------------------------------------

//�����ģʽ Ŀǰ����4��ģʽ

#define WORK_MODEL_NORMAL           0     //��������
#define WORK_MODEL_SLEEP            1     //��ʱ��CPU�������ģʽ,GPSģ���GSMģ�������
//#define WORK_MODEL_POWEROFF         2     //��⵽�ڻ����ƿ����ʱ��PIC�ж��豸�ⲿ��Դ
//#define WORK_MODEL_MOVE             3     //ACC���������عرպ����ٶ�>30KM/h

#define WEAKLAST_TIME              300    //���߻��Ѻ��������ʱ��5����
#define WORK_HEART_TIME            20     //20��
#define TRANSITMODESPEED           300    //�ж�Ϊ����ģʽ���ٶ����޵�λ0.1Km/h

//-----�ṹ����----------------------------------------------------------------

typedef struct _STU_Sleep_
{
   uint32 uiSleepWakenSlot;            //���߻��Ѽ��,��λΪ�룬ÿ��m���Ӻ������������ݣ�Ȼ��ȴ�n���Ӻ��������
   uint16 usSleepBeforSlot;            //����ǰʱ����,��λΪ�룬switch1�͵�ƽn���Ӻ�����
   uint16 usWakeLastTime;              //���߻��Ѻ��������,��λΪ�룬����ʱ��5����
//  uint8  ucBtwnGpsGsmTime;            //����GPS���ٻ���GSM֮��ļ��ʱ��
   uint8  ucFlag;                      //B0 ����;
                                      //B1=1  ���Ѻ��������5���Ӽ�ʱ��,��Ҫ���½�������
                                      //B2=1  ����ǰʱ���Ϊ0��ʼ����,
                                      
   uint8  ucSpecialFlag;               //����������ж��Ƿ�����߻��߻��ѵı�־
                                      //B0=1 ��������ʱ��������뵽����
                                      //B1=1 ��������ʱ���������ڼ���������
   uint32 usSleepTimes;				  //���ߺ��ն���Ҫ˯�ߵ�ʱ�䡣 
    uint8 ucWorkingMode;            //0=������1=���ߡ�2=����  3 = ���� 
    uint8 ucGsmSleepState;          //GSMģ������״̬:0=������;1=����
    uint8 ucGpsSleepState;          //GPSģ������״̬:0=������;1=����
    uint8 ucWork_Heart_Count;       //��ʼֵΪ20  Ϊ0 �Ա���ʱWork_heart_Flag=1,��ʾ���Խ������߻�����һģʽ 
    uint8 ucWork_heart_Flag;        //1=��ʾ��Ҫ�����������ģʽ�л��Ķ�λ����;0=��Ҫ���� 
    uint8 ucReadRTCClockFlag;       //�ն˻��Ѻ��ȡRTCʱ�ӱ�־,0-����ȡ,1-��ȡ

}STUSleep,*PSTUSleep;






//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
void SYS_GPSSleep(void);
void SYS_GPSWake(void);
void SYS_GSMSleep(void);
void SYS_GSMWake(void);
void SYS_WorkMode_Exec(void);
void SYS_WorkModeInit(void);
void SYS_WorkModeSleep_Count(void);
void SYS_VoltageLowMode_Exec(void);
void MainPowerCheck(void);
#endif















