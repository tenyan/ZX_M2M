/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved 
 * 
 * �ļ�����: GpsIntergrateLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: ���ļ�ΪGPS����ģ���ۺϲ��ʵ���ļ�
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
#include "GpsProtocolLayer.h"
#include "GpsIntegrateLayer.h"
#include "GpsInterfaceLayer.h"

//-----�ⲿ��������------------------------------------------------------------
extern STU_SSData SSData;
extern uint16_t UART4Timer;
extern uint8_t UART4Buffer[];
extern uint8 GPS_Recv_Buff[GPS_RECEIVE_BUFF_LEN];
extern STUSystem g_stuSystem;
extern uint8 Speedlen ;
//-----�ڲ���������------------------------------------------------------------
STU_Orient m_stuOrient;
STU_Gps GPS;

//-----�ڲ���������------------------------------------------------------------

static void  ResetGps(void);

//-----�ⲿ����ʵ��------------------------------------------------------------
/******************************************************************************
** ��������: pthread_Gps_Function
** ��������: ��ȡ������Gpsģ�������߳�,������ʽ
** 
** ��    ��: data
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: lxf
** ��    ��: 2019-03-25
**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_Gps_Function(void* data)
{

	uint16 usLen = 0;  //���������յ������ݳ���    
	uint8 *p=NULL;
  
    InitGps(0);

	while(1)
    {       
     
        if(ReadGpsUartData(&p, &usLen))
        {     
           PC_SendDebugData(p,usLen,DEBUG_GPSMODULE);

           GPS.RecvBuffLen = usLen;
		   memcpy(GPS.RecvBuff,p,usLen);
		   usLen = 0;
        //   PrintTime();
		//   printf("read: %d , %s\n", GPS.RecvBuffLen,GPS.RecvBuff);
           m_stuOrient.ucGpsState &= ~BIT(6);           //GPSģ������
           GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
           ReadFromGpsSerialBuff(GPS.RecvBuff, GPS.RecvBuffLen);
           ParseNMEA(); 		   
        }
		else
		{
          //  usleep(20*One_MilliSecond);
		}
      //  usleep(20*One_MilliSecond);
    }
}

void GPS_Function(uint8 *buff,uint16 uslen)
{
    PC_SendDebugData(buff,uslen,DEBUG_GPSMODULE);
    m_stuOrient.ucGpsState &= ~BIT(6);           //GPSģ������
    GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
    ReadFromGpsSerialBuff(buff,uslen);
    ParseNMEA(); 
}

//-----�ڲ�����ʵ��------------------------------------------------------------

void GpsModuleErrCheck()
{
	if(0==GPS.SleepState)
	{
		if(0==GPS.RecvTimeOut--) 
		{
			GPS.RecvTimeOut = GPS_RECEIVE_DATAOUT;
		    m_stuOrient.ucGpsState |= BIT(6);    //��ģ����Ϊ�쳣
			m_stuOrient.ucGpsState &= ~BIT(1);   //GPSǿ�Ʋ���λ
	    	ResetGps();                          //����GPSģ��	    	    
	    } 
	}
}
//�����쳣����
void GpsAntenaCheck()
{
	static uint8  AntennaOkTime;	//��������״̬�ƴ�
	static uint8  AntennaErrTime;	//���߹���״̬�ƴ�
	static  uint8   ucAntennaState=1;   //���������쳣��:0����,1�쳣

	if (m_stuOrient.ucGpsState&BIT(1)) // B1=1 GPS��λ,0 GPS����λ
	{
		m_stuOrient.ucGpsState&=~BIT(0); 
	}
	else	//����λ���ж�����״̬
	{
		if (m_stuOrient.ucGpsState & BIT(0))    //���������쳣
		{
			if(!GetGpsSwitchState())			//���߼�ʱ�ָ�����
			{
				AntennaOkTime++;
				AntennaErrTime=0;
				if (AntennaOkTime>Antenna_ok_time)
				{
					m_stuOrient.ucGpsState &= ~BIT(0); 
					AntennaOkTime=0;
				}
			}
			else 		//������Ȼ����	
			{
				AntennaOkTime=0;
			}
		}
		else		//����״̬������
		{
			if(!GetGpsSwitchState())              //������Ȼ����
			{
				AntennaErrTime=0;
			}
			else     //�����쳣
			{
				AntennaErrTime++;
				AntennaOkTime=0;
				if (AntennaErrTime>Antenna_err_time)
				{
					m_stuOrient.ucGpsState|=BIT(0); 
					AntennaErrTime=0;
				}
			}
		}
	}    
	
	if(m_stuOrient.ucGpsState & BIT(0))                          
	{
	    if(0==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA WARNING\n", 16, DEBUG_GPSMODULE);
		//    f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_ERROR;
		//    SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 1;            //�����쳣��ʶ
			SSData.usTimer = 0;
    	}
	}		
	else                                       //���������쳣������
	{
	    if(1==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA OK\n", 11, DEBUG_GPSMODULE);
		//	f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_OK;
		//	SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 0;
		    SSData.usTimer = 0;
    	}
	}
}

//���ٱ���
void SpeedOverAlarm()
{
	static  uint8   ucSpeedOverTime=0;    //����ʱ��
	static  uint8   ucSpeedNormalTime=0;  //�ٶȻع�ʱ��
	static  uint8   ucSpeed = 0;        //����������:0������,1����
	
	if(m_stuOrient.ucGpsState & BIT(1))
	{
	    if(0 != SYS_GetParam()->ucSpeedOver)        //Ϊ0ʱ,�����г��ٱ���
		{
			if(m_stuOrient.usSpeed > (10*SYS_GetParam()->ucSpeedOver)) //���ٱ���
			{
				ucSpeedNormalTime = 0;
			    if(0==ucSpeed && ucSpeedOverTime++>5)
		    	{
		    	    ucSpeedOverTime = 0;
			    	ucSpeed = 1;
					m_stuOrient.ucGpsState |= BIT(7);
					PC_SendDebugData((uint8*)"OVERSPEED WARNING\n", 18, DEBUG_GPSMODULE);
				   // f_stuGpsWarning.ucKind = GPS_MSG_WARN_SPEED_VOER;
				  //  SYS_PutDataQ((void *)&f_stuGpsWarning);	
				    SSData.usTimer = 0;
		    	}
			}
			else                                                  //�������
			{
				ucSpeedOverTime = 0;
			    if(1==ucSpeed && ucSpeedNormalTime++>5) 
		    	{
		    	    ucSpeedNormalTime=0;
			    	ucSpeed = 0;
					m_stuOrient.ucGpsState &= ~BIT(7);
					//f_stuGpsWarning.ucKind = GPS_MSG_WARN_SPEED_OK;
				   // SYS_PutDataQ((void *)&f_stuGpsWarning);	
				    SSData.usTimer = 0;
		    	}
			}
		}
	}
}
/******************************************************************************
** ��������: ResetGps
** ��������: ����GPSģ��
**
** ��    ��: ��
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: 
** ��    ��: 2019-03-28
**-----------------------------------------------------------------------------
** �޸ļ�¼
**-----------------------------------------------------------------------------
** 2019-03-07  ��ʼ�汾
*******************************************************************************/
void ResetGps()
{
	GpsPower_CLR();
	OSTimeDly(200);
	GpsPower_SET();
	OSTimeDly(200);
	OSTimeDly(200);
	GPSRMCGGA();
	OSTimeDly(OS_TICKS_PER_SEC*2);
	PC_SendDebugData((uint8 *)("ResetGps"), 8, DEBUG_ANYDATA);
}
//�����ӳٵĶ�ʱִ�к���:10ms
void GPS_TimerCount_Delay()
{
	static uint8 Cnt1s=0;
	
	if(Cnt1s)
		Cnt1s--;
	else
	{
		Cnt1s = 100;
		GpsModuleErrCheck();
		ReInitGps();
	}
}
//�������ӳٵĶ�ʱִ�к���:10ms
void GPS_TimerCount_NoDelay()
{
	static uint8 Cnt1s=0;
	
	if(Cnt1s)
		Cnt1s--;
	else
	{
		Cnt1s = 100;
		GpsAntenaCheck();
		SpeedOverAlarm();
	//	SaveDistanceToEEPROM_AccOff();
	}
}
//-----�ļ�GpsIntegrateLayer.c����---------------------------------------------


