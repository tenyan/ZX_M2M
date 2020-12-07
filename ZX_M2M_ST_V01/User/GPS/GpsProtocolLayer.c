/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: GpsProtocolLayer.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为GPS功能模块协议层的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-21, by , 创建本文件
 *
 */

#include "config.h"
STU_Orient m_stuOrient;

//天线异常报警
void GpsAntenaCheck(void)
{
	static uint8  AntennaOkTime;	//天线正常状态计次
	static uint8  AntennaErrTime;	//天线故障状态计次
//	static  uint8   ucAntennaState=1;   //天线正常异常锁:0正常,1异常

	if (m_stuOrient.ucGpsState&BIT(1)) // B1=1 GPS定位,0 GPS不定位
	{
		m_stuOrient.ucGpsState&=~BIT(0); 
	}
	else	//不定位则判断天线状态
	{
		if (m_stuOrient.ucGpsState & BIT(0))    //现在天线异常
		{
			if(!GetGpsSwitchState())			//天线及时恢复正常
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
			else 		//天线仍然故障	
			{
				AntennaOkTime=0;
			}
		}
		else		//现在状态是正常
		{
			if(!GetGpsSwitchState())              //天线仍然正常
			{
				AntennaErrTime=0;
			}
			else     //天线异常
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
			ucAntennaState = 1;            //天线异常标识
    	}
	}		
	else                                       //清零天线异常计数器
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
** 函数名称: GPS_GetOrientState
** 功能描述: 获取GPS模块定位状态
** 输    入: 无
** 输    出: 无
** 返    回: GPS定位状态,1=定位;0=未定位
** 作    者: hhm
** 日    期: 2016-07-06
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

//-----外部变量定义------------------------------------------------------------
extern STU_Gps GPS;
//-----内部变量定义------------------------------------------------------------
BOOL f_bAntennaState;
extern STU_Orient m_stuOrient;



uint8 GPS_SHOT ;
uint8 GPS_OPEN;
uint8 Heightlen ;
uint8 Speedlen ;
//-----内部函数声明------------------------------------------------------------

//-----外部函数实现------------------------------------------------------------

/******************************************************************************
** 函数名称: CheckGpsPara
** 功能描述: 校验GPS模块返回的参数是否为5C所需参数
**
** 输    入: pucData,待校验数据块地址;usLength,待校验数据长度
** 输    出: 无
** 返    回: 结果正确返回TRUE,错误返回FALSE
**
** 作    者: zwp
** 日    期: 2011-03-22
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
** 函数名称: CheckGpsData
** 功能描述: GPS模块数据校验和函数
** 
** 输    入: pucData,待校验数据块地址;usLength,待校验数据长度
** 输    出: 无
** 返    回: 校验正确返回TRUE,错误返回FALSE
**
** 作    者: zwp
** 日    期: 2011-03-22
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
** 函数名称: ReadFromGpsSerialBuff
** 功能描述: 读GPS 数据包
** 输　     入: 无
**
** 输　     出: 数据包有效，返回0；否则返回1
** 全局变量: 
** 调用模块: 
**
** 作　     者: zcm
** 日　     期: 2007年7 月20 日
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
        //查找GPRMC字符串
	p1=SearchString(buff,len, "$GPRMC", 6);
	if(p1)
	{   
		p2=GetStringMiddle(p1,len,'$',1,'*',1,&sublen);
		if (p2)
		{
			//求校验和		
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
		{						//GPRMC数据包校验正确
			p2=GetStringMiddle(p1,len, ',',2,',',3, &sublen);	//定位状态
			if (p2)
			{
				GPS.NMEAdata.DataStatus=*p2;
				if(GPS.NMEAdata.DataStatus=='A')
				{	//处在正常定位状态
					//读取地面速率
					p2=GetStringMiddle(p1,len, ',',7,',',8, &sublen);
					if (sublen>3) 
					{   
					    if(sublen>7)   sublen = 7;
						memcpy(GPS.NMEAdata.Speed,p2,sublen);
						Speedlen = sublen;
					}
					//读取地面航向
					p2=GetStringMiddle(p1,len, ',',8,',',9, &sublen);
					if (sublen>2)
					    memcpy(GPS.NMEAdata.ForDirect,p2,sublen-3);		//ublox  	
					else  //方向角为0
					{
                        GPS.NMEAdata.ForDirect[0] = '0';
                        GPS.NMEAdata.ForDirect[1] = '0';
                        GPS.NMEAdata.ForDirect[2] = '0';
                    }
                    
					//读取UTC日期
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
    	//哥白尼$GPGGA,093016.00,3203.37541,N,11844.92711,E,1,06,2.07,00033,M,005,M,,*65 数据包
    	//ublox:$GPGGA,115845.00,3158.62303,N,11844.86933,E,2,11,0.99,45.0,M,2.3,M,,0000*57
    	//查找$GPGGA字符串
    p1=SearchString(buff,len, "$GPGGA", 6);
    if(p1)
	{   
		p2=GetStringMiddle(p1,len,'$',1,'*',1,&sublen);
		if (p2)
		{
			//求校验和		
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
		{						//p1数据包校验正确
			p2=GetStringMiddle(p1,len, ',',6,',',7, &sublen);	//定位状态
			if (p2)
			{
				CalcCheckOut=*p2;                      //
				//			        if((CalcCheckOut=='1')||(CalcCheckOut=='2'))                   //备份用  需要核实定位要求
				if(GPS.NMEAdata.DataStatus=='A')
				{	//处在正常定位状态
					//读取UTC时间
					p2=GetStringMiddle(p1,len, ',',1,',',2, &sublen);
					if (sublen>3) memcpy(GPS.NMEAdata.UtcTime,p2,sublen-3);    //去掉小数部分
					//读取地面纬度
					p2=GetStringMiddle(p1,len, ',',2,',',3, &sublen);
					if (sublen>2) memcpy(GPS.NMEAdata.Latitude,p2,10);			        	
					//读取纬度半球
					p2=GetStringMiddle(p1,len, ',',3,',',4, &sublen);
					if (sublen==1) memcpy(&GPS.NMEAdata.NS,p2,sublen);

					//读取经度
					p2=GetStringMiddle(p1,len, ',',4,',',5, &sublen);
					if (sublen>0) memcpy(GPS.NMEAdata.Longitude,p2,11);//sublen);

					//读取经度半球
					p2=GetStringMiddle(p1,len, ',',5,',',6, &sublen);
					if (sublen==1) memcpy(&GPS.NMEAdata.EW,p2,sublen);

					//读取星数
					p2=GetStringMiddle(p1,len, ',',7,',',8, &sublen);
					if (sublen>0) memcpy(GPS.NMEAdata.SatelliteNum,p2,sublen);

					//读取海拔高度
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
** 函数名称: ParseNMEA
** 功能描述: 读GPS 数据包
** 输　     入: 无
**
** 输　     出: 
** 全局变量: 
** 调用模块: 
**
** 作　     者: 
** 日　     期: 2012年8 月9 日
**------------------------------------------------------------------------------------------------------
********************************************************************************************************/
//解析GPS的NMEA数据
void ParseNMEA(void)
{
    uint32 value32;
    uint16 value16 ;
    uint8  satellitenums;
//	uint32 uiTemp;
	static uint8  ucFlag=0;        //经纬度初始标识
    static uint8  ucCounter=0;     //计数器
	static double oldLongitude=0;  //里程计算老经度值                                 
    static double oldLatitude=0;   //里程计算老纬度值
    double newLongitude=0;         //里程计算新经度值
	double newLatitude=0;          //里程计算新纬度值

	uint32 uiDistance=0;           //里程临时存储
	STU_Date date;
//	STU_Date rtc_date;
	
	if(GPS.NMEAdata.DataStatus=='A')
    {
        value32=((uint32)DecToUint16((uint8*)&GPS.NMEAdata.Latitude,2)*60+DecToUint16((uint8*)&GPS.NMEAdata.Latitude[2],2))*1000+DecToUint16((uint8*)&GPS.NMEAdata.Latitude[5],3);
		m_stuOrient.lLatitude_PPMDegree =DecToUint32((uint8*)&GPS.NMEAdata.Latitude[0],2)*1000000+
			   (DecToUint32((uint8*)&GPS.NMEAdata.Latitude[2],2)*100000+DecToUint32((uint8*)&GPS.NMEAdata.Latitude[5],5))/6;
		if(GPS.NMEAdata.NS=='N')
		{
            m_stuOrient.lLatitude=value32;    //纬度
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
            m_stuOrient.lLongitude=value32;  //经度
			m_stuOrient.ucPostion |= BIT(0);
		}
		else
		{
            m_stuOrient.lLongitude=0-value32;      
			m_stuOrient.ucPostion &= ~BIT(0);
		}
		
        value16=DecToUint16((uint8*)&GPS.NMEAdata.Height[0],Heightlen-2);                    //高度  //ublox 数据长度会有变化
		if(GPS.NMEAdata.HeightSing!='-') 
            m_stuOrient.sHeight=value16;
        else 
            m_stuOrient.sHeight=0-value16;
      
//        value16=(DecToUint16((uint8*)&GPS.NMEAdata.Speed,(Speedlen-4))*10+DecToUint8((uint8*)&GPS.NMEAdata.Speed[Speedlen-3],1));//ublox 
//		m_stuOrient.usSpeed=(value16<<1)-value16*19/128;            //speed=value16*1.852;把海里转换成公里
        if(Speedlen>4)
        {
            value16=(DecToUint16((uint8*)&GPS.NMEAdata.Speed,(Speedlen-4))*10+DecToUint8((uint8*)&GPS.NMEAdata.Speed[Speedlen-3],1));//ublox 
    		m_stuOrient.usSpeed=(value16<<1)-value16*19/128;        //speed=value16*1.852;把海里转换成公里
        }
        else 
        {            
        }

        m_stuOrient.usForDirect=DecToUint16((uint8*)&GPS.NMEAdata.ForDirect,3);       //方向角

////////////速度和方向角校验////////////////////////////
        if (m_stuOrient.usSpeed>=2500)   m_stuOrient.usSpeed = 0;                       //test
        if (m_stuOrient.usForDirect>359)  m_stuOrient.usForDirect = 0;                    //test
/////////////////////////////////////////////////////		
        
        m_stuOrient.stuDate.ucDay=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate,2);
        m_stuOrient.stuDate.ucMon=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate[2],2);
        m_stuOrient.stuDate.ucYear=DecToUint8((uint8*)&GPS.NMEAdata.UtcDate[4],2);
        m_stuOrient.stuDate.ucHour=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime,2);
        m_stuOrient.stuDate.ucMin=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime[2],2);
        m_stuOrient.stuDate.ucSec=DecToUint8((uint8*)&GPS.NMEAdata.UtcTime[4],2);

      	//计算收到的星数
        satellitenums= (GPS.NMEAdata.SatelliteNum[0]-'0')*10+(GPS.NMEAdata.SatelliteNum[1]-'0');	  
        if(satellitenums>15)    satellitenums = 15;
        m_stuOrient.ucGpsState=(m_stuOrient.ucGpsState&0xc3)|(satellitenums<<2);                       //收到的星数	
		m_stuOrient.satellitenums = satellitenums;
			
		if(0==ucFlag)
		{
		    oldLatitude = m_stuOrient.lLatitude/60000.0;
			oldLongitude = m_stuOrient.lLongitude/60000.0;
			ucFlag = 1;
		}

		if(++ucCounter>=10 && GetAccState())  //里程计算
		{
		    ucCounter = 0;
			newLatitude = m_stuOrient.lLatitude/60000.0;
			newLongitude = m_stuOrient.lLongitude/60000.0;

			uiDistance = (uint32)distance(oldLatitude, oldLongitude, newLatitude, newLongitude);
			if(uiDistance>=15)                 //防抖动处理
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
        m_stuOrient.ucGpsState|=BIT(1);                 //'A'表示定位
    }    
    else if(GPS.NMEAdata.DataStatus=='V')
    {
		m_stuOrient.ucGpsState&=~BIT(1);                //不定位
		m_stuOrient.usSpeed = 0;
		m_stuOrient.usForDirect = 0;
		//rtc_date = GetRTCTime_UTC();
		//m_stuOrient.stuDate = rtc_date;
		m_stuOrient.ucGpsState = m_stuOrient.ucGpsState&0xc3;        //将收到的星数改为0   080330
        m_stuOrient.satellitenums = 0;
    }
}



/******************************************************************************
** 函数名称: PackGpsData
** 功能描述: 以GPS模块协议封装数据函数
**
** 输    入: pucSrc,待封装数据地址;usSrcLen,待封装数据长度;            
** 输    出: pucDes,数据封装后存放地址;pusDesLen,封装后数据长度
** 返    回: 成功返回0,错误返回0x2248
**
** 作    者: zwp
** 日    期: 2011-03-22
**-----------------------------------------------------------------------------
*******************************************************************************/
/*
uint16 PackGpsData(const uint8 *pucSrc, uint16 usSrcLen, uint8 *pucDes, uint16 *pusDesLen)
{
    return 0;
}
*/


//-----内部函数实现------------------------------------------------------------

//-----文件GpsProtocolLayer.c结束----------------------------------------------

#endif
