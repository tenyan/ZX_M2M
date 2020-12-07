/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: GpsProtocolLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪGPS����ģ��Э����ʵ���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , �������ļ�
 *
 */

#include "config.h"
STU_Orient m_stuOrient;

//�����쳣����
void GpsAntenaCheck(void)
{
	static uint8  AntennaOkTime;	//��������״̬�ƴ�
	static uint8  AntennaErrTime;	//���߹���״̬�ƴ�
//	static  uint8   ucAntennaState=1;   //���������쳣��:0����,1�쳣

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
//					GPS_Antenna_Ok=TRUE;			
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

	/*
	if(m_stuOrient.ucGpsState & BIT(0))                          
	{
	    if(0==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA WARNING\n", 16, DEBUG_GPSMODULE);
		    f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_ERROR;
		    SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 1;            //�����쳣��ʶ
    	}
	}		
	else                                       //���������쳣������
	{
	    if(1==ucAntennaState)
    	{
	    	PC_SendDebugData((uint8*)"ANTENNA OK\n", 11, DEBUG_GPSMODULE);
			f_stuGpsWarning.ucKind = GPS_MSG_WARN_ANTENNA_OK;
			SYS_PutDataQ((void *)&f_stuGpsWarning);
			ucAntennaState = 0;
    	}
	}
	*/
}
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

#if 0
#include "GpsModule.h"
#include "GpsIntegrateLayer.h"
#include "GpsDataCalc.h"
#include "GpsInterfaceLayer.h"
#include "GpsProtocolLayer.h"

//-----�ⲿ��������------------------------------------------------------------
extern STU_Gps GPS;
//-----�ڲ���������------------------------------------------------------------
BOOL f_bAntennaState;
extern STU_Orient m_stuOrient;



uint8 GPS_SHOT ;
uint8 GPS_OPEN;
uint8 Heightlen ;
uint8 Speedlen ;
//-----�ڲ���������------------------------------------------------------------

//-----�ⲿ����ʵ��------------------------------------------------------------

/******************************************************************************
** ��������: CheckGpsPara
** ��������: У��GPSģ�鷵�صĲ����Ƿ�Ϊ5C�������
**
** ��    ��: pucData,��У�����ݿ��ַ;usLength,��У�����ݳ���
** ��    ��: ��
** ��    ��: �����ȷ����TRUE,���󷵻�FALSE
**
** ��    ��: zwp
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL CheckGpsPara(const uint8 *pucData, uint16 usLength)
{
    uint8 *pucPtr=NULL;
	if(NULL!=(pucPtr=SearchString((uint8*)pucData, usLength, "$GPGGA", 6)))
	{
	    if(NULL!=(pucPtr=SearchString((uint8*)pucPtr, usLength-6, "$GPRMC", 6)))
    	{
    	    if(SearchString((uint8*)pucPtr, usLength-12, "$PTNLRBA", 8))
	    	{
	    	    return TRUE;
	    	}
    	}
	}
	return FALSE;
}

/******************************************************************************
** ��������: CheckGpsData
** ��������: GPSģ������У��ͺ���
** 
** ��    ��: pucData,��У�����ݿ��ַ;usLength,��У�����ݳ���
** ��    ��: ��
** ��    ��: У����ȷ����TRUE,���󷵻�FALSE
**
** ��    ��: zwp
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
BOOL CheckGpsData(const uint8 *pucData, uint16 usLength)
{
    uint16     i=0;
	uint8     *pucPtr = NULL;
	uint8      ucCheckOut = 0;
	uint8      ucCheckData = 0;
	uint16     usLen = 0;
	uint8      ucCount = 1;

    for(; ucCount<=3; ucCount++)
	{
		pucPtr = GetStringMiddle((uint8 *)pucData, usLength, '$', ucCount, '*', ucCount, &usLen);
		if(pucPtr != NULL)
		{
		    ucCheckOut = pucPtr[0];
			for(i=1; i<usLen; i++)
			{
				ucCheckOut ^= pucPtr[i];
			}
			ucCheckData = SingleASCIITOHex(pucPtr[usLen+1]);
			ucCheckData = ucCheckData << 4;
			ucCheckData += SingleASCIITOHex(pucPtr[usLen+2]);
			if(ucCheckData != ucCheckOut)
			{
			    return FALSE;
			}
		}
		else
		{
		    return FALSE;
		}
	}
	return TRUE;

}


BOOL IsNumber(uint8* str,uint8 len)
{
    uint8 i;
    for(i=0;i<len;i++)
    {
        if(str[i]>0x39||str[i]<0x30)    //'0'-'9'
            return FALSE;
    }
    return TRUE;
}

/*********************************************************************************************************
** ��������: ReadFromGpsSerialBuff
** ��������: ��GPS ���ݰ�
** �䡡     ��: ��
**
** �䡡     ��: ���ݰ���Ч������0�����򷵻�1
** ȫ�ֱ���: 
** ����ģ��: 
**
** ����     ��: zcm
** �ա�     ��: 2007��7 ��20 ��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/

void ReadFromGpsSerialBuff(uint8 *buff,uint16 len)
{
    uint8 *p1;
    uint8 *p2;	
    uint8 *p3;		
    uint16 sublen;
    uint16 i;    
    uint8 CheckOut;
    uint8 CalcCheckOut;	
    BOOL CheckOut_GPRMC_Ok;
    BOOL CheckOut_GPGGA_Ok;
    BOOL CheckOut_PTNLRBA_Ok;	
//    BOOL GPS_Antenna_Ok;	
    BOOL CheckOut_GPVTG_OK;
	
    CheckOut_GPVTG_OK=FALSE;
    CheckOut_GPRMC_Ok=FALSE;
    CheckOut_GPGGA_Ok=FALSE;
    CheckOut_PTNLRBA_Ok=FALSE;

	
    	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //ublox  $GPRMC,115845.00,A,3158.62303,N,11844.86933,E,11.607,8.35,220812,,,D*57
        //ublox  $GPRMC,082425.00,A,3158.62784,N,11844.86023,E,5.079,205.77,120912,,,A*65
        //����GPRMC�ַ���
	p1=SearchString(buff,len, "$GPRMC", 6);
	if(p1)
	{   
		p2=GetStringMiddle(p1,len,'$',1,'*',1,&sublen);
		if (p2)
		{
			//��У���		
			CalcCheckOut=p2[0];
			for (i=1;i<sublen;i++)
			{
				CalcCheckOut^=p2[i];
			}
			p3=GetStringBack(p1, len, '*',1);
			CheckOut=SingleASCIITOHex(p3[0]);
			CheckOut=CheckOut<<4;
			CheckOut+=SingleASCIITOHex(p3[1]);
			if (CalcCheckOut==CheckOut)
			{
					CheckOut_GPRMC_Ok=TRUE;
			}
			else 
			{
				CheckOut_GPRMC_Ok=FALSE;
			}
		}
		if (CheckOut_GPRMC_Ok)
		{						//GPRMC���ݰ�У����ȷ
			p2=GetStringMiddle(p1,len, ',',2,',',3, &sublen);	//��λ״̬
			if (p2)
			{
				GPS.NMEAdata.DataStatus=*p2;
				if(GPS.NMEAdata.DataStatus=='A')
				{	//����������λ״̬
					//��ȡ��������
					p2=GetStringMiddle(p1,len, ',',7,',',8, &sublen);
					if (sublen>3) 
					{   
					    if(sublen>7)   sublen = 7;
						memcpy(GPS.NMEAdata.Speed,p2,sublen);
						Speedlen = sublen;
					}
					//��ȡ���溽��
					p2=GetStringMiddle(p1,len, ',',8,',',9, &sublen);
					if (sublen>2)
					    memcpy(GPS.NMEAdata.ForDirect,p2,sublen-3);		//ublox  	
					else  //�����Ϊ0
					{
                        GPS.NMEAdata.ForDirect[0] = '0';
                        GPS.NMEAdata.ForDirect[1] = '0';
                        GPS.NMEAdata.ForDirect[2] = '0';
                    }
                    
					//��ȡUTC����
					p2=GetStringMiddle(p1,len, ',',9,',',10, &sublen);
					if (sublen>0)
					{     
						if(IsNumber(p2, 6))
						memcpy(GPS.NMEAdata.UtcDate,p2,sublen);
					}				
				}
			}
		}
	}
	else
	{
	    GPS.modelFlag |= BIT(0);
	}
    
    	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    	//�����$GPGGA,093016.00,3203.37541,N,11844.92711,E,1,06,2.07,00033,M,005,M,,*65 ���ݰ�
    	//ublox:$GPGGA,115845.00,3158.62303,N,11844.86933,E,2,11,0.99,45.0,M,2.3,M,,0000*57
    	//����$GPGGA�ַ���
    p1=SearchString(buff,len, "$GPGGA", 6);
    if(p1)
	{   
		p2=GetStringMiddle(p1,len,'$',1,'*',1,&sublen);
		if (p2)
		{
			//��У���		
			CalcCheckOut=p2[0];
			for (i=1;i<sublen;i++)
			{
				CalcCheckOut^=p2[i];
			}
			p3=GetStringBack(p1, len, '*',1);
			CheckOut=SingleASCIITOHex(p3[0]);
			CheckOut=CheckOut<<4;
			CheckOut+=SingleASCIITOHex(p3[1]);

			if (CalcCheckOut==CheckOut) 
				CheckOut_GPGGA_Ok=TRUE;
			else 
				CheckOut_GPGGA_Ok=FALSE;
		}
		if (CheckOut_GPGGA_Ok)
		{						//p1���ݰ�У����ȷ
			p2=GetStringMiddle(p1,len, ',',6,',',7, &sublen);	//��λ״̬
			if (p2)
			{
				CalcCheckOut=*p2;                      //
				//			        if((CalcCheckOut=='1')||(CalcCheckOut=='2'))                   //������  ��Ҫ��ʵ��λҪ��
				if(GPS.NMEAdata.DataStatus=='A')
				{	//����������λ״̬
					//��ȡUTCʱ��
					p2=GetStringMiddle(p1,len, ',',1,',',2, &sublen);
					if (sublen>3) memcpy(GPS.NMEAdata.UtcTime,p2,sublen-3);    //ȥ��С������
					//��ȡ����γ��
					p2=GetStringMiddle(p1,len, ',',2,',',3, &sublen);
					if (sublen>2) memcpy(GPS.NMEAdata.Latitude,p2,10);			        	
					//��ȡγ�Ȱ���
					p2=GetStringMiddle(p1,len, ',',3,',',4, &sublen);
					if (sublen==1) memcpy(&GPS.NMEAdata.NS,p2,sublen);

					//��ȡ����
					p2=GetStringMiddle(p1,len, ',',4,',',5, &sublen);
					if (sublen>0) memcpy(GPS.NMEAdata.Longitude,p2,11);//sublen);

					//��ȡ���Ȱ���
					p2=GetStringMiddle(p1,len, ',',5,',',6, &sublen);
					if (sublen==1) memcpy(&GPS.NMEAdata.EW,p2,sublen);

					//��ȡ����
					p2=GetStringMiddle(p1,len, ',',7,',',8, &sublen);
					if (sublen>0) memcpy(GPS.NMEAdata.SatelliteNum,p2,sublen);

					//��ȡ���θ߶�
					p2=GetStringMiddle(p1,len, ',',9,',',10, &sublen);
					 GPS.NMEAdata.HeightSing=*p2;

					if((GPS.NMEAdata.HeightSing!='-')&&(sublen>0)) 
					{
						memcpy(GPS.NMEAdata.Height,p2,sublen);   
						Heightlen = sublen;     //ublox 
					}
					else if ((GPS.NMEAdata.HeightSing=='-')&&(sublen>1))
					{
						memcpy(GPS.NMEAdata.Height,p2+1,sublen-1);
						Heightlen = sublen-1;   //ublox 
					}
    			}
    			else
    			{
    			     return ;                                                           //test
    			}
			}
		}
	}
	else
	{
		GPS.modelFlag |= BIT(0);
	}
}
/*********************************************************************************************************
** ��������: ParseNMEA
** ��������: ��GPS ���ݰ�
** �䡡     ��: ��
**
** �䡡     ��: 
** ȫ�ֱ���: 
** ����ģ��: 
**
** ����     ��: 
** �ա�     ��: 2012��8 ��9 ��
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
//����GPS��NMEA����
void ParseNMEA(void)
{
    uint32 value32;
    uint16 value16 ;
    uint8  satellitenums;
//	uint32 uiTemp;
	static uint8  ucFlag=0;        //��γ�ȳ�ʼ��ʶ
    static uint8  ucCounter=0;     //������
	static double oldLongitude=0;  //��̼����Ͼ���ֵ                                 
    static double oldLatitude=0;   //��̼�����γ��ֵ
    double newLongitude=0;         //��̼����¾���ֵ
	double newLatitude=0;          //��̼�����γ��ֵ

	uint32 uiDistance=0;           //�����ʱ�洢
	STU_Date date;
//	STU_Date rtc_date;
	
	if(GPS.NMEAdata.DataStatus=='A')
    {
        value32=((uint32)DecToUint16((uint8*)&GPS.NMEAdata.Latitude,2)*60+DecToUint16((uint8*)&GPS.NMEAdata.Latitude[2],2))*1000+DecToUint16((uint8*)&GPS.NMEAdata.Latitude[5],3);
		m_stuOrient.lLatitude_PPMDegree =DecToUint32((uint8*)&GPS.NMEAdata.Latitude[0],2)*1000000+
			   (DecToUint32((uint8*)&GPS.NMEAdata.Latitude[2],2)*100000+DecToUint32((uint8*)&GPS.NMEAdata.Latitude[5],5))/6;
		if(GPS.NMEAdata.NS=='N')
		{
            m_stuOrient.lLatitude=value32;    //γ��
            m_stuOrient.ucPostion |= BIT(1);
		}	
		else  
		{
            m_stuOrient.lLatitude=0-value32;
			m_stuOrient.ucPostion &= ~BIT(1);
		}
		
        value32=((uint32)DecToUint16((uint8*)&GPS.NMEAdata.Longitude,3)*60+DecToUint16((uint8*)&GPS.NMEAdata.Longitude[3],2))*1000+DecToUint16((uint8*)&GPS.NMEAdata.Longitude[6],3);
		m_stuOrient.lLongitude_PPMDegree = DecToUint32((uint8*)&GPS.NMEAdata.Longitude[0],3)*1000000+
			    (DecToUint32((uint8*)&GPS.NMEAdata.Longitude[3],2)*100000+DecToUint32((uint8*)&GPS.NMEAdata.Longitude[6],5))/6;
		if(GPS.NMEAdata.EW=='E')  
		{
            m_stuOrient.lLongitude=value32;  //����
			m_stuOrient.ucPostion |= BIT(0);
		}
		else
		{
            m_stuOrient.lLongitude=0-value32;      
			m_stuOrient.ucPostion &= ~BIT(0);
		}
		
        value16=DecToUint16((uint8*)&GPS.NMEAdata.Height[0],Heightlen-2);                    //�߶�  //ublox ���ݳ��Ȼ��б仯
		if(GPS.NMEAdata.HeightSing!='-') 
            m_stuOrient.sHeight=value16;
        else 
            m_stuOrient.sHeight=0-value16;
      
//        value16=(DecToUint16((uint8*)&GPS.NMEAdata.Speed,(Speedlen-4))*10+DecToUint8((uint8*)&GPS.NMEAdata.Speed[Speedlen-3],1));//ublox 
//		m_stuOrient.usSpeed=(value16<<1)-value16*19/128;            //speed=value16*1.852;�Ѻ���ת���ɹ���
        if(Speedlen>4)
        {
            value16=(DecToUint16((uint8*)&GPS.NMEAdata.Speed,(Speedlen-4))*10+DecToUint8((uint8*)&GPS.NMEAdata.Speed[Speedlen-3],1));//ublox 
    		m_stuOrient.usSpeed=(value16<<1)-value16*19/128;        //speed=value16*1.852;�Ѻ���ת���ɹ���
        }
        else 
        {            
        }

        m_stuOrient.usForDirect=DecToUint16((uint8*)&GPS.NMEAdata.ForDirect,3);       //�����

////////////�ٶȺͷ����У��////////////////////////////
        if (m_stuOrient.usSpeed>=2500)   m_stuOrient.usSpeed = 0;                       //test
        if (m_stuOrient.usForDirect>359)  m_stuOrient.usForDirect = 0;                    //test
/////////////////////////////////////////////////////		
        
        m_stuOrient.stuDate.ucDay=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate,2);
        m_stuOrient.stuDate.ucMon=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate[2],2);
        m_stuOrient.stuDate.ucYear=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate[4],2);
        m_stuOrient.stuDate.ucHour=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime,2);
        m_stuOrient.stuDate.ucMin=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime[2],2);
        m_stuOrient.stuDate.ucSec=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime[4],2);

      	//�����յ�������
        satellitenums= (GPS.NMEAdata.SatelliteNum[0]-'0')*10+(GPS.NMEAdata.SatelliteNum[1]-'0');	  
        if(satellitenums>15)    satellitenums = 15;
        m_stuOrient.ucGpsState=(m_stuOrient.ucGpsState&0xc3)|(satellitenums<<2);                       //�յ�������	
		m_stuOrient.satellitenums = satellitenums;
			
		if(0==ucFlag)
		{
		    oldLatitude = m_stuOrient.lLatitude/60000.0;
			oldLongitude = m_stuOrient.lLongitude/60000.0;
			ucFlag = 1;
		}

		if(++ucCounter>=10 && GetAccState())  //��̼���
		{
		    ucCounter = 0;
			newLatitude = m_stuOrient.lLatitude/60000.0;
			newLongitude = m_stuOrient.lLongitude/60000.0;

			uiDistance = (uint32)distance(oldLatitude, oldLongitude, newLatitude, newLongitude);
			if(uiDistance>=15)                 //����������
			{
			    m_uiDistance += uiDistance;
				uiDistance = m_uiDistance;
				SaveDistanceToRTCRAM(uiDistance);
			}
	
			oldLatitude = newLatitude;
			oldLongitude = newLongitude;

		}
		if(0==IsCorrectTimeOk())
		{
			if(UtcToBjTime(&m_stuOrient.stuDate, &date))
	        	AccountRTCDate(date);
		}

	}
    if(GPS.NMEAdata.DataStatus=='A')  
    {
        m_stuOrient.ucGpsState|=BIT(1);                 //'A'��ʾ��λ
    }    
    else if(GPS.NMEAdata.DataStatus=='V')
    {
		m_stuOrient.ucGpsState&=~BIT(1);                //����λ
		m_stuOrient.usSpeed = 0;
		m_stuOrient.usForDirect = 0;
		//rtc_date = GetRTCTime_UTC();
		//m_stuOrient.stuDate = rtc_date;
		m_stuOrient.ucGpsState = m_stuOrient.ucGpsState&0xc3;        //���յ���������Ϊ0   080330
        m_stuOrient.satellitenums = 0;
    }
}



/******************************************************************************
** ��������: PackGpsData
** ��������: ��GPSģ��Э���װ���ݺ���
**
** ��    ��: pucSrc,����װ���ݵ�ַ;usSrcLen,����װ���ݳ���;            
** ��    ��: pucDes,���ݷ�װ���ŵ�ַ;pusDesLen,��װ�����ݳ���
** ��    ��: �ɹ�����0,���󷵻�0x2248
**
** ��    ��: zwp
** ��    ��: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
/*
uint16 PackGpsData(const uint8 *pucSrc, uint16 usSrcLen, uint8 *pucDes, uint16 *pusDesLen)
{
    return 0;
}
*/


//-----�ڲ�����ʵ��------------------------------------------------------------

//-----�ļ�GpsProtocolLayer.c����----------------------------------------------

#endif
