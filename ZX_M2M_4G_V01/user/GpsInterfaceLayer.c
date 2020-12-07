/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GpsInterfaceLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGPS����ģ��ӿڲ��ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by  �������ļ�
 *
 */
#include "config.h"
#include "GpsModule.h"
#include "GpsHardWareLayer.h"
#include "GpsIntegrateLayer.h" 
#include "GpsInterfaceLayer.h"
#include "GpsProtocolLayer.h"

//-----�ⲿ��������------------------------------------------------------------
extern  STU_Gps GPS;
//-----�ڲ���������------------------------------------------------------------
uint16  m_usGpsDly;       //GPSģ�����ʱ��
uint32  m_uiDistance;     //���ֵ

//-----�ڲ���������------------------------------------------------------------

static void InitGpsModule(void);
//static void InitDistance(void);

//-----�ⲿ��������------------------------------------------------------------


//gps��λ���³�ʼ������
void SetGps(void)
{
	uint8 gpgga1[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};
	uint8 gpgga2[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38};
	uint8 gpgga3[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46};
	uint8 gpgga4[11] = {0xB5,0x06,0x17,0x04,0x00,0x00,0x23,0x0C,0x03,0x53,0x6D};
	OSTimeDly(100);
  	//SendUart2Data(gpgga1,16);
  	WriteGpsUartData(gpgga1,16);
	OSTimeDly(4);
    //SendUart2Data(gpgga2,16);
    WriteGpsUartData(gpgga2,16);
	OSTimeDly(4);
	//SendUart2Data(gpgga3,16);
	WriteGpsUartData(gpgga3,16);
	OSTimeDly(4);
	//SendUart2Data(gpgga4,11);
	WriteGpsUartData(gpgga4,11);
	return;
	
}


/*********************************************************************************************************
** ��������: GPSRMCGGA
** ��������: GPSģ��ֻ�ͳ�GGA,RMC���� 
** �䡡     ��: ��
**
** �䡡     ��: ��
** ȫ�ֱ���: 
** ����ģ��: 
**
** ����     ��: 
** �ա�     ��: 2011��12��30��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
void GPSRMCGGA(void)
{	uint8 gpgga1[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2A};
	uint8 gpgga2[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x38};
	uint8 gpgga3[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x46};
	//uint8 gpgga4[12] = {0xB5,0x62,0x06,0x17,0x04,0x00,0x00,0x23,0x0C,0x03,0x53,0x6D};
	uint8 gpgga5[44] = {0xB5,0x62,0x06,0x24,0x24,0x00,0xFF,0xFF,0x04,0x02,0x00,
		                0x00,0x00,0x00,0x10,0x27,0x00,0x00,0x05,0x00,0x64,0x00,
		                0x64,0x00,0x32,0x00,0x32,0x00,0x64,0x00,0x00,0x00,0x00,
		                0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x20};
    OSTimeDly(OS_TICKS_PER_SEC/2);                                                                                         //ʱ, ��Ϊ0 , ���޳�ʱ����.
	WriteGpsUartData(gpgga1,16);
	 OSTimeDly(OS_TICKS_PER_SEC/10);
    WriteGpsUartData(gpgga2,16);
	 OSTimeDly(OS_TICKS_PER_SEC/10);
	WriteGpsUartData(gpgga3,16);
	 OSTimeDly(OS_TICKS_PER_SEC/10);
	WriteGpsUartData(gpgga5,44);
}

/******************************************************************************
** ��������: InitGps
** ��������: ��ʼ��GPSģ��
** 
** ��    ��: pPara,GPSģ������������ɵĽṹ��;uiBps,����2�Ĳ�����
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-03-29
**-----------------------------------------------------------------------------
*******************************************************************************/
void InitGps(uint32 uiBps)
{
	GPS_Uart_Init();
	//GPS_GPIO_Pin_Init();
	InitGpsModule();
//	InitDistance();
	GPSRMCGGA();
	OSTimeDly(OS_TICKS_PER_SEC*2);
	PC_SendDebugData((uint8 *)("GpsInit"), 7, DEBUG_ANYDATA);

}

void ReInitGps(void)	//gps���³�ʼ��
{
	static uint8 cnt = 0;
	
	if(GPS.modelFlag&BIT(0))		//GPS��λ��
	{
		if(cnt++ > 10)
		{
    		GPS.modelFlag &= ~BIT(0);
			cnt = 0;
			GPSRMCGGA();
			OSTimeDly(OS_TICKS_PER_SEC*2);
			PC_SendDebugData((uint8 *)("GpsReCfg"), 8, DEBUG_ANYDATA);
		}
		
	}
	else
		cnt = 0;
}

/******************************************************************************
** ��������: GPS_SetDistance
** ��������: ������ú���
**
** ��    ��: ���������ֵ
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: 
** ��    ��: 2011-07-29
-------------------------------------------------------------------------------
*******************************************************************************/
void GPS_SetDistance(uint32 uiDistance)
{
 //	m_uiDistance = uiDistance;
//	SaveDistanceToEEPROM(uiDistance);
//SaveDistanceToRTCRAM(uiDistance);
}

/******************************************************************************
** ��������: GPS_GetOrient
** ��������: ����GPSģ�����ݽṹ
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ�����ݽṹ
**
** ��    ��: 	
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
PSTU_Orient GPS_GetOrient(void)
{
    return &m_stuOrient;
}

/******************************************************************************
** ��������: GPS_GetLongitude
** ��������: ��ȡ��ǰ�ľ���,��λ�����֮һ��
** ��    ��:
** ��    ��: ��
** ��    ��: ��ǰ�ľ���
** ��    ��: hhm
** ��    ��: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 GPS_GetLongitude(void)
{
    return m_stuOrient.lLongitude_PPMDegree;
}

/******************************************************************************
** ��������: GPS_GetLatitude
** ��������: ��ȡ��ǰ��γ��,��λ�����֮һ��
** ��    ��:
** ��    ��: ��ǰ��γ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 GPS_GetLatitude(void)
{
    return m_stuOrient.lLatitude_PPMDegree;
}

/******************************************************************************
** ��������: GPS_GetLatitudeHemisphere
** ��������: ��ȡ��ǰ�ľ��Ȱ���,������
** ��    ��:
** ��    ��: ��ǰ�ľ��Ȱ���,1=������ 0=����
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetLongitudeHemisphere(void)
{
    return (m_stuOrient.ucPostion & BIT(0))?1:0;
}

/******************************************************************************
** ��������: GPS_GetLatitudeHemisphere
** ��������: ��ȡ��ǰ��γ�Ȱ���,�ϱ�γ
** ��    ��:
** ��    ��: ��ǰ��γ�Ȱ���,1=��γ�� 0=��γ
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetLatitudeHemisphere(void)
{
    return (m_stuOrient.ucPostion & BIT(1))?1:0;
}

/******************************************************************************
** ��������: GetLatitude_Original
** ��������: ��ȡ��ǰԭʼ��γ������
** ��    ��:
** ��    ��: ��
** ��    ��: ָ��ԭʼγ�����ݵ�ָ��
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GPS_GetLatitude_Original()
{
	return GPS.NMEAdata.Latitude;
}

/******************************************************************************
** ��������: GetLatitude_Original
** ��������: ��ȡ��ǰԭʼ�ľ�������
** ��    ��:
** ��    ��: ��
** ��    ��: ָ��ԭʼ�������ݵ�ָ��
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 *GPS_GetLongitude_Original()
{
	return GPS.NMEAdata.Longitude;
}

/******************************************************************************
** ��������: GPS_GetForDirect
** ��������: ��ȡGPS�����
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPS�����
** ��    ��: hhm
** ��    ��: 2016-7-1
*******************************************************************************/
uint16 GPS_GetForDirect(void)
{
    return m_stuOrient.usForDirect;
}

/******************************************************************************
** ��������: GPS_GetHeight
** ��������: ��ȡGPS�߶�,��λ:m
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPS�߶�
** ��    ��: hhm
** ��    ��: 2016-7-1
*******************************************************************************/
int16 GPS_GetHeight(void)
{
    return m_stuOrient.sHeight;
}
/******************************************************************************
** ��������: GPS_GetSpeed
** ��������: ��ȡ�豸�����ٶ�
** ��    ��: ��
** ��    ��: ��
** ��    ��: �豸�����ٶ�
** ��    ��: hhm
** ��    ��: 2016-7-1
*******************************************************************************/
uint16 GPS_GetSpeed(void)
{
    return m_stuOrient.usSpeed;
}

/******************************************************************************
** ��������: GPS_GetDistance
** ��������: ��ȡ�豸�������
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: �豸�������
**
** ��    ��: 
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
uint32 GPS_GetDistance(void)
{
	return m_uiDistance;
}

/******************************************************************************
** ��������: GPS_GetState
** ��������: ��ȡGPSģ�鼰����״̬
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ�鼰����״̬
**
** ��    ��: 
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetState(void)
{
    return m_stuOrient.ucGpsState;
}

/******************************************************************************
** ��������: GPS_GetGpsState
** ��������: ��ȡGPSģ��״̬
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ��״̬,1�쳣;0����
**
** ��    ��: 
** ��    ��: 2011-07-07
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetGpsState(void)
{
    return m_stuOrient.ucGpsState&BIT(6) ? 1:0;
}

/******************************************************************************
** ��������: GPS_GetAnteState
** ��������: ��ȡGPSģ������״̬
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ������״̬,0����;1�쳣
**
** ��    ��: 
** ��    ��: 2011-07-07
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetAnteState(void)
{
    return m_stuOrient.ucGpsState&BIT(0) ? 1 : 0;
}

/******************************************************************************
** ��������: GPS_GetOrientState
** ��������: ��ȡGPSģ�鶨λ״̬
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPS��λ״̬,1=��λ;0=δ��λ
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetOrientState(void)
{
    return m_stuOrient.ucGpsState&BIT(1) ? 1 : 0;
}

/******************************************************************************
** ��������: GPS_GetSatellitenums
** ��������: ��ȡGPS����
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPS����
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetSatellitenums(void)
{
    return m_stuOrient.satellitenums;
}
/******************************************************************************
** ��������: GPS_GetSpeedoverState
** ��������: ��ȡGPS����״̬
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPS����״̬,1=����;0=����
** ��    ��: hhm
** ��    ��: 2016-07-06
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GPS_GetSpeedoverState(void)
{
    return m_stuOrient.ucGpsState&BIT(7) ? 1 : 0;
}
/******************************************************************************
** ��������: GPS_GetUtcTime
** ��������: ��ȡGPSģ�������UTCʱ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ�������UTCʱ��
** ��    ��: hhm
** ��    ��: 2015-9-7
**-----------------------------------------------------------------------------
*******************************************************************************/
STU_Date GPS_GetUtcTime()
{
	return m_stuOrient.stuDate;
}


/******************************************************************************
** ��������: GPS_GetUtcTime
** ��������: ��ȡGPSģ�������UTCʱ��
** ��    ��: ��
** ��    ��: ��
** ��    ��: GPSģ�������UTCʱ��
** ��    ��: hhm
** ��    ��: 2015-9-7
**-----------------------------------------------------------------------------
*******************************************************************************/
STU_Date GPS_GetBeiJinTime()
{
	STU_Date date;
	
	UtcToBjTime(&m_stuOrient.stuDate, &date);
	return date;
}
/******************************************************************************
** ��������: GPS_SleepGps()
** ��������: GPSģ�����ߺ���
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: 
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GPS_SleepGps(void)
{
    GPS.SleepState = 1;  

	OSTimeDly(100);
	GpsPower_CLR();

}

/******************************************************************************
** ��������: GPS_WakeGps
** ��������: GPSģ�黽�Ѻ���
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: 
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
void GPS_WakeGps(void)
{
   // GPS_Uart_Init();
	//GPS_GPIO_Pin_Init();
    GpsPower_SET();

	m_usGpsDly = 200*5;         //5�����ʱ��

	GPSRMCGGA();
	GPS.SleepState = 0; 
	PC_SendDebugData((uint8 *)("GpsWk"), 5, DEBUG_ANYDATA);
}

//-----�ڲ�����ʵ��------------------------------------------------------------



/******************************************************************************
** ��������: InitGpsModule
** ��������: ��ʼ��GPSģ��,��GPSģ���ϵ�
** 
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: 
** ��    ��: 2011-03-29
**-----------------------------------------------------------------------------
*******************************************************************************/
void InitGpsModule(void)
{
	GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;  
	m_stuOrient.ucGpsState = 0x41;    //��ʼ�����߼�ģ��״̬
    GpsPower_SET();
	OSTimeDly(50);    //

	OSTimeDly(50);    //

}

/******************************************************************************
** ��������: InitDistance
** ��������: ��ʼ����̴洢��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2011-07-29
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
void InitDistance(void)
{
  //  uint8   aucTemp[6];       

}
//������̵�RTC RAM��
void SaveDistanceToRTCRAM(uint32 uiDistance)
{

}

void SaveDistanceToEEPROM(uint32 uiDistance)
{
	//uint8 aucTemp[6];

}

//��ACC OFFʱ������̵�EEPROM
//�˺���1s����һ��
void SaveDistanceToEEPROM_AccOff()
{
	static uint8 acc_pre = 0;
	uint8 acc;
	
	acc = GetAccState();
	if((acc==0) && (acc_pre==1))
	{
		SaveDistanceToEEPROM(m_uiDistance);
		SaveDistanceToRTCRAM(m_uiDistance);
	}
	acc_pre = acc;
}
#endif
//-----�ļ�GpsInterfaceLayer.c����---------------------------------------------





















