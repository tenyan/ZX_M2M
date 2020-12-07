/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved 
 * 
 * �ļ�����: GpsModule.h
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪGPS����ģ�����ж���ӿ��ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , �������ļ�
 *
 */

#ifndef GPSMODULE_H_20110321_FF9FED25_F92A_4FB0_9B79_61E85B793F14
#define GPSMODULE_H_20110321_FF9FED25_F92A_4FB0_9B79_61E85B793F14
#include "GpsHardWareLayer.h"

//-----��������----------------------------------------------------------------
#define TASK_GPS_STK_SIZE                   250
#define TASK_GPS_PRIO                       7
#define TASK_GPS_ID                         7

#define GPS_MSG_WARN_SPEED_VOER             1    //���ٱ���
#define GPS_MSG_WARN_SPEED_OK               2    //������ٱ���
#define GPS_MSG_WARN_ANTENNA_ERROR          3    //�����쳣����
#define GPS_MSG_WARN_ANTENNA_OK             4
#define GPS_MSG_WARN_ROUND                  5    //���䱨��
#define Antenna_ok_time						10
#define Antenna_err_time					10
#define GPS_RECEIVE_BUFF_LEN        		650
#define GPS_RECEIVE_DATAOUT          		5                         //5��û���յ�GPSģ��������Ϊģ���쳣
//-----�ṹ����----------------------------------------------------------------

//����ʱ��ṹ��
typedef struct _STU_Date_
{
  uint8  ucYear;             //��
  uint8  ucMon;              //��
  uint8  ucDay;              //��
  uint8  ucHour;             //ʱ
  uint8  ucMin;              //��
  uint8  ucSec;              //��
}STU_Date, *PSTU_Date;

//GPSģ�����ݽṹ��
typedef struct _STU_Orient_
{


    int32   lLongitude;      //����,1:��,2:��,3\4:��С��
    int32   lLatitude;       //γ��,1:��,2:��,3\4:��С��
    uint16   usSpeed;         //�ٶ�,��λ:����/Сʱ
    int16    sHeight;         //�߶�,��λ��,��ƽ������Ϊ����
    uint16   usForDirect;     //0-360
    STU_Date stuDate;         //ʱ��
    uint8    ucGpsState;      //GPS״̬�ֽ�
                              // B0=0 ��������,1���߹���
                              // B1=1 GPS��λ,0 GPS����λ
                              // B5-B2=GPS �յ�������
                              // B6 =0 GPSģ������(���������)1=ģ���쳣
                              // B7=0 �ٶ�����, 1=����
    uint8    ucPostion;       //��λֵ   b0:0=��,1=��, b1:0=��,1=��
    int32 lLongitude_PPMDegree;		//����,��λ:�����֮һ��

	int32 lLatitude_PPMDegree;		    //γ��,��λ:�����֮һ��
    uint8 satellitenums;			//�յ�������
}STU_Orient, *PSTU_Orient;


//GPSģ������ṹ��
typedef struct _STU_GPS_PARA_
{
	//OS_EVENT *pMQ;         //��Ϣ�����¼�ָ��
	    
}STU_GpsPara, *PSTU_GpsPara;


struct _NMEAdata  //52
{
    uint8 Latitude[10];                                      //γ��
    uint8 NS;                                                   //N�ϱ�
    uint8 Longitude[11];                                   //����
    uint8 EW;                                                   //E����
    uint8 Height[7];                                         //�߶�
    uint8 HeightSing;                                      //��ʾ�߶���"+""-"
    uint8 UtcTime[6];                                      //
    uint8 UtcDate[6];                                       //
    uint8 Speed[8];                                          //
    uint8 ForDirect[3];                                     //
    uint8 DataStatus;                                       //GPS�Ƿ���Ч:Ϊ"A"��ʾ��Ч,Ϊ"V"��ʾ��Ч
    uint8 SatelliteNum[2];                                //��ǰ�յ�������
};


typedef struct _GPS
{ 
    struct _NMEAdata NMEAdata;                      //GPS�յ�������
    uint8 RecvBuff[GPS_RECEIVE_BUFF_LEN];
    uint16 RecvBuffLen;
    uint8 RecvTimeOut;                                    // GPSû�����������ʱ
    uint8 SleepState;                                        //=0 �����ߣ�=1����״̬
    uint8 SleepLedTime;                                   //GPS�����ڼ�����ʱ�䶨Ϊ3����,��3��֮����˸һ��
    uint8 modelFlag;                                        //�ж�GPSģ��������������û�а������߼�����
                                                                    //B0 = 0:GPS�������3�����;=1:GPS�ָ���������
}STU_Gps;


//-----�ⲿ����----------------------------------------------------------------

//-----�ⲿ����----------------------------------------------------------------
PSTU_Orient GPS_GetOrient(void);
uint32   GPS_GetLongitude(void);
uint32   GPS_GetLatitude(void);
uint8 	GPS_GetLongitudeHemisphere(void);
uint8 	GPS_GetLatitudeHemisphere(void);	
uint16 	GPS_GetForDirect(void);
int16 	GPS_GetHeight(void);
uint16   GPS_GetSpeed(void);
uint8    GPS_GetSpeedoverState(void);
uint32   GPS_GetDistance(void);
uint8    GPS_GetSatellitenums(void);
uint8    GPS_GetState(void);
void     GPS_SleepGps(void);
void     GPS_WakeGps(void);
uint8    GPS_GetGpsState(void);
uint8    GPS_GetAnteState(void);
uint8    GPS_GetOrientState(void);
void     GPS_SetDistance(uint32 uiDistance);
STU_Date GPS_GetUtcTime(void);
STU_Date GPS_GetBeiJinTime(void);
void     GPS_TimerCount_Delay(void);
void 	 GPS_TimerCount_NoDelay(void);
void     TaskGps(void *pvData);
uint8 *GPS_GetLatitude_Original(void);
uint8 *GPS_GetLongitude_Original(void);
void GPS_Function(uint8 *buff,uint16 uslen);
void* pthread_Gps_Function(void* data);
#endif 


