/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: .c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述:
   本文件为本文件为处理从A5核心板接收到的uart数据文件,对应MCU的uart3口的实现文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-05-01 by , 创建本文件
 *
 */

#include "config.h"
#include "A5_Com1HW.h"
//-----------外部变量定义-----------------
extern STU_MCUSend_GPSData stuMcuSend_Gpsdata;   //向控制器发送定位信息
extern STU_Orient m_stuOrient;
extern STU_GSM_State g_stuGsmState;
extern STUSystem g_stuSystem;
STU_SYS_LEDState g_stuSysLedState;
extern uint8 ucOpenTimes;
extern STUSleep stuSleep;
extern stuFirmwareUpdate FirmwareUpdate;
//-----------内部变量定义-----------------
STU_A5Comm stuA5Comm;
STU_A5_Date stuA5Date = {
	.ucYear = 19,
	.ucMon = 5,
	.ucDay = 8,
	.ucHour = 12,
	.ucMin = 0,
	.ucSec = 0,
	.usmSec = 0,
	.usmSectemp = 0,
};

//对协处理器发来的命令进行应答
void A5_MCU_SendReqCmdData(uint8 ucCommand, uint8 ucSN, uint8 ucResult,uint8 ucSrc)
{
    uint8 SendBuff[10] = {0};

	SendBuff[0] = 0x7E;
	SendBuff[1] = 0;
	SendBuff[2] = 4;
	SendBuff[3] = ucCommand;         //Command	
		
	SendBuff[4] = ucSN;
	SendBuff[5] = ucResult;
	SendBuff[6] = SumCalc(&SendBuff[3],3);
	
	SendBuff[7] = 0x0D;
	SendBuff[8] = 0x0A;
	if(SRCDEVICE_ID_A5_COM1==ucSrc)
		RS232_UART_Write(SendBuff, 8);
	else if(SRCDEVICE_ID_SETTOOL==ucSrc)
	    WriteGsmUartData(SendBuff, 8);
}
#if 0
//实际发送最大长度为1029Byte
void A5_MCU_SendToA5CANData(uint8 ucCommand, uint8 ucSN, uint8 ucBuffNum)
{
    uint8 SendBuff[1100] = {0};
	uint16 usSendDataLen = 0;

	usSendDataLen = (uint16)(stuA5Comm.aCANDatabuff[ucBuffNum][1]<<8) + stuA5Comm.aCANDatabuff[ucBuffNum][2] +7;

	SendBuff[0] = 0x7E;
	SendBuff[1] = (uint8)(usSendDataLen>>8); 
	SendBuff[2] = (uint8)(usSendDataLen&0xFF);
	SendBuff[3] = ucCommand;         //Command	
		
	SendBuff[4] = ucSN;
	SendBuff[5] = 1;    //TLV 个数
    memcpy(&SendBuff[6],&stuA5Comm.aCANDatabuff[ucBuffNum][0],usSendDataLen-4);
    	
	SendBuff[6+(usSendDataLen-4)] = SumCalc(&SendBuff[3],3+(usSendDataLen-4));
	
	SendBuff[usSendDataLen+3] = 0x0D;
	SendBuff[usSendDataLen+4] = 0x0A;
	
	WriteGsmUartData(SendBuff, usSendDataLen+5);
}
#endif

//实际发送最大长度为1029Byte
//ucflag =1 发送CAN数据,0-不发送CAN数据
void A5_MCU_SendToA5CANData(uint8 ucCommand, uint8 ucSN, uint8 ucBuffNum,uint8 ucflag)
{
    uint8 SendBuff[1250] = {0};
	uint16 usSendDataLen = 0;
	uint16 usTlvlen1 = 0;
	uint16 usTlvlen2 = 0;

	SendBuff[0] = 0x7E;
	SendBuff[3] = ucCommand;         //Command			
	SendBuff[4] = ucSN;
	if(ucflag==1)
	{
	
    	SendBuff[5] = 2;    //TLV 个数
    	usTlvlen1 = (uint16)(stuA5Comm.aCANDatabuff[ucBuffNum][1]<<8) + stuA5Comm.aCANDatabuff[ucBuffNum][2];
        memcpy(&SendBuff[6],&stuA5Comm.aCANDatabuff[ucBuffNum][0],usTlvlen1+3);
		SendBuff[usTlvlen1+9] = 0x04; //T- AD采集+开关量(8Byte) 
		usTlvlen2 = 10;                //L- 8
		SendBuff[usTlvlen1+10] = 0;
		SendBuff[usTlvlen1+11] = usTlvlen2;
		BuildAD_Switch(&SendBuff[usTlvlen1+12]);
		usSendDataLen = 4 + usTlvlen1+3+usTlvlen2+3;
	}
	else
	{
    	SendBuff[5] = 2;    //TLV 个数
		SendBuff[6] = 0x04; //T- AD采集+开关量(10Byte)
		usTlvlen1 = 10;     //L- 10
		SendBuff[7] = 0;
		SendBuff[8] = usTlvlen1;
		BuildAD_Switch(&SendBuff[9]); 
		SendBuff[9+usTlvlen1] = 0x03;   //T- RTC时钟(北京时间)
		usTlvlen2 = 6;
		SendBuff[10+usTlvlen1] = 0;
		SendBuff[11+usTlvlen1] = usTlvlen2;
		memcpy(&SendBuff[12+usTlvlen1],(uint8*)&stuA5Date,6);
		usSendDataLen = 4 + usTlvlen1+3 + usTlvlen2+3;		
	}
	
	SendBuff[1] = (uint8)(usSendDataLen>>8); 
	SendBuff[2] = (uint8)(usSendDataLen&0xFF);
	
	SendBuff[usSendDataLen+2] = SumCalc(&SendBuff[3],(usSendDataLen-1));
	
	SendBuff[usSendDataLen+3] = 0x0D;
	SendBuff[usSendDataLen+4] = 0x0A;	
	WriteGsmUartData(SendBuff, usSendDataLen+5);
}

//每秒运行执行一次
void A5_CommErr_Judge(void)
{
    if(++stuA5Comm.ucCommErrTime>Recv_4GMODULE_TIMEOUT)
    {
        stuA5Comm.ucCommErrTime = 0;
    	stuA5Comm.ucCommflag = 0;
        g_stuGsmState.ucRunState = 3;       //重启GSM模块  模块休眠时需要考虑
        m_stuOrient.ucGpsState &= ~BIT(1);  //接收不到核心板数据时默认为不定位
        printf("CommErrTime\n");
    }
}


void A5_Deal_RecVA5_Data(uint8 *ptr, uint16 uslen)
{

    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand;
	uint8 ucSN;
	uint16 i = 0;
	uint8 ucTlvNum = 0;
	uint8 ucTlvTag = 0;
	uint16 usTlvLen = 0;
	uint8 *ptemp;
//	uint8 *ptrTemp;  
	STU_Date stuTime;
	uint8 ucValue = 0;    //<5不需要校准时钟,>=5校准时钟
	
    if((ptr[0]!=0x7E)||(ptr[uslen-2]!=0x0D)||(ptr[uslen-1]!=0x0A))
		return;

	ucOpenTimes = 1;
	stuA5Comm.ucCommflag = 1;
	stuA5Comm.ucCommErrTime = 0;
	g_stuGsmState.ucRunState = 2;
	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
    ucCommand = ptr[3];
	ucSN = ptr[4];
	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
	if(ucSumNum!=ptr[uslen-3])
		return;
	if(ucCommand == 0x02)      //接收核心板发来的数据
	{
     //   printf("ucCommand\r");
    	ucTlvNum = ptr[5];     //TLV个数
    	if(ucTlvNum<1)
			return;
        ptemp = ptr+6;
        for(i=0;i<ucTlvNum;i++)
        {
            ucTlvTag = *ptemp++;
			if(ucTlvTag==0x07)        //指示灯状态TLV
			{
			    usTlvLen = (uint16)(*ptemp++ << 8);
				usTlvLen += (*ptemp++);
				if(usTlvLen==5)
				{
                  // printf("TLV7\r");

    			    
    			   g_stuSystem.ucOnline = *ptemp++;
				   if((*ptemp++)&BIT(0))
				       m_stuOrient.ucGpsState |= BIT(1);
				   else
				   	   m_stuOrient.ucGpsState &= ~BIT(1);
				   g_stuSysLedState.ucLed_WiFi = *ptemp++;
				   g_stuSysLedState.ucLed_ETH = *ptemp++;
				   g_stuSysLedState.ucreserve = *ptemp++;
				   if(g_stuSysLedState.ucreserve&BIT(1))
				   	  FirmwareUpdate.ucMainCPuUpgradeFlag = 1;
				   else
				   	  FirmwareUpdate.ucMainCPuUpgradeFlag = 0;
				   	
				}
				else
					break;			    
			}
			else if(ucTlvTag==0x08)   //GPS数据TLV
			{
			    usTlvLen = (uint16)(*ptemp++ << 8);
				usTlvLen += (*ptemp++);
				if(usTlvLen==16)
			    {
    			  //  printf("TLV8\r");
					
			        memcpy((uint8*)&stuMcuSend_Gpsdata,ptemp,usTlvLen);
				/*	if(*ptemp == 0)
					    m_stuOrient.ucGpsState&=~BIT(1);
					else
						m_stuOrient.ucGpsState|= BIT(1);*/					
			    }
				else
					break;
			    ptemp += usTlvLen;
			}
			else if(ucTlvTag==0x06)   //CAN命令透传
			{
			   // printf("TLV6\r");
			    usTlvLen = (uint16)(*ptemp++<<8);
				usTlvLen += *ptemp++;			
			    if(usTlvLen==15)
			    {
			        if(MCU_SendNewLockCmd(ptemp)==0)
						return;
			    /*
                    if(*ptemp <= 1)
                    {
            			ucCanChannel = *ptemp++;
                    }
					else
						return;
            		if(*ptemp <= 1)
            		{
            			ucFF = *ptemp++;
            		}
					else
						return;
					ptrTemp = ptemp;
                    uiCanID = (uint32)(ptrTemp[0]<<24) + (uint32)(ptrTemp[1]<<16) +(uint32)(ptrTemp[2]<<8) + ptrTemp[3];
                    if(ptrTemp[4]<=8)
						ucCanDatalen = ptrTemp[4];
					else 
						return;
					memcpy(aCanData,&ptrTemp[5],8);
					ptemp+=13;  
					memcpy(stuMcuSend_Gpsdata.aControlData,&aCanData[0],2);
					stuMcuSend_Gpsdata.ucCount = 2;
					
                    CanWrite(ucCanChannel, ucFF, uiCanID, ucCanDatalen, aCanData);				
			        //需要及时发送旧协议
			        */
			    }
				else
					break;
			    ptemp+=usTlvLen;
			}
			else if(ucTlvTag==0x03)   //时间TLV(北京时间)
			{
			    //判断要不要校准RTC时钟
			    usTlvLen = (uint16)(*ptemp++ << 8);
				usTlvLen += *ptemp++;				
			    if(usTlvLen==6)
			    {
    			 //   printf("TLV3\r");
			    
                    memcpy((uint8*)&stuTime,ptemp,usTlvLen);
				    if(stuTime.ucSec>stuA5Date.ucSec)
				    	ucValue = stuTime.ucSec-stuA5Date.ucSec;
					else
						ucValue = stuA5Date.ucSec-stuTime.ucSec;
					
					if(stuTime.ucMon!=0&&stuTime.ucYear>=19&&(stuTime.ucYear!=stuA5Date.ucYear||stuTime.ucMon!=stuA5Date.ucMon||stuTime.ucDay!=stuA5Date.ucDay
						||stuTime.ucHour!=stuA5Date.ucHour||ucValue>=5))
					{
                        //时钟校准
                        stuA5Date.ucYear = stuTime.ucYear;
                		stuA5Date.ucMon = stuTime.ucMon;
                		stuA5Date.ucDay = stuTime.ucDay;
                		stuA5Date.ucHour = stuTime.ucHour;
                		stuA5Date.ucMin = stuTime.ucMin;
                		stuA5Date.ucSec = stuTime.ucSec;
                        AccountRTCDate(stuTime);   //北京时间
					}					
				}
				else
					break;
			    ptemp += usTlvLen;
			}
		}		
		//应答命令
//        A5_MCU_SendReqCmdData(0x82, ucSN, ucResult,SRCDEVICE_ID_SETTOOL);
	}
}

#if 0
void A5_Deal_RecVA5_Data(uint8 *ptr, uint16 uslen)
{
    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand;
	uint8 ucSN,ucResult = 0;
	uint8 aCanData[8] = {0};
	uint8 ucCanChannel = 0;
    uint8 ucFF = 0;
	uint32 uiCanID;

    if((ptr[0]!=0x7E)||(ptr[uslen-2]!=0x0D)||(ptr[uslen-1]!=0x0A))
		return;	
	stuA5Comm.ucCommflag = 1;
	stuA5Comm.ucCommErrTime = 0;
	
	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
    ucCommand = ptr[3];
	ucSN = ptr[4];
	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
	if(ucSumNum!=ptr[uslen-3])
		return;
	
    if(ucCommand == 0x02)
    {
        if(ptr[5]==0||ptr[5]==1)
			ucCanChannel = ptr[5];
		if(ptr[6]==0||ptr[6]==1)
			ucFF = ptr[6];
        uiCanID = (uint32)(ptr[7]<<24) + (uint32)(ptr[8]<<16) +(uint32)(ptr[9]<<8) + ptr[10];
		memcpy(aCanData,&ptr[11],8);
		
        CanWrite(ucCanChannel, ucFF, uiCanID, 8, aCanData);
        A5_MCU_SendReqCmdData(0x82, ucSN, ucResult,SRCDEVICE_ID_SETTOOL);
	}
}
#endif

#if 0
//向A5定时发送CAN工况  暂定每200ms执行一次
void A5_MCUSendCANDataToA5(void)
{

    uint8 i = 0;
	uint8 ucFlag = 0;
	static uint8 m = 0;
	static uint8 ucCount = 0;
    static uint8 ucSendTimer = 0;

	if(!stuA5Comm.ucSendTimer)
		stuA5Comm.ucSendTimer = 20;
	else
	{
	//	stuA5Comm.ucSendTimer--;
		return;
	}
	
 //   if(!stuA5Comm.ucSendTimer)
 //   {
 //       stuA5Comm.ucSendTimer = 2;

		ucCount++;
        for(i=0;i<SEND_TO_A5_BUFF_NUM;i++)
        {
        
			if(m>=SEND_TO_A5_BUFF_NUM)
				m = 0;      
            if(stuA5Comm.aSenDataBuffFlag[m]==1)
            {
               
               if(stuA5Comm.ucSN>=255)
			       stuA5Comm.ucSN = 0;
			   else
			   	   stuA5Comm.ucSN++;
			   
               A5_MCU_SendToA5CANData(0x01, stuA5Comm.ucSN, m);  //发送
               stuA5Comm.aSenDataBuffFlag[m] = 0;          //标志位清零
               ucFlag = 1;
			   ucCount--;
               m++;
               break;               
			}
			else
			{
    			m++;
			}							
		}
		
		if(ucFlag == 0&&ucCount>=5)   //未有接收完成的CAN帧数据1秒钟
		{
		    ucCount = 0;
            for(i=0;i<SEND_TO_A5_BUFF_NUM;i++)
		    {
                if(m>=SEND_TO_A5_BUFF_NUM)
				    m = 0;  
                if(stuA5Comm.aSenDataBuffFlag[m]==2)
                {
                   if(stuA5Comm.ucSN>=255)
    			       stuA5Comm.ucSN = 0;
    			   else
			   	       stuA5Comm.ucSN++;
				   
                   stuA5Comm.aSenDataBuffFlag[m]=0xFF;  //抢断
                   A5_MCU_SendToA5CANData(0x01, stuA5Comm.ucSN, m);  //发送
                   stuA5Comm.aSenDataBuffFlag[m] = 0;    //标志位清零
                   m++;
                   break;
				}
    			else
    			{
        			m++;
    			}					
			}
		}	
		
//	}
//	else
//	{
 //       stuA5Comm.ucSendTimer--;
//	}
}
#endif
//向A5定时发送CAN工况  暂定每200ms发一次
void A5_MCUSendCANDataToA5(void)
{

    uint8 i = 0;
	static uint8 m = 0;
	static uint8 ucSendOverTimer = CAN_SEND_MAX_COUNTER;
//    static uint8 ucSendTimer = 0;
	uint8 ucFlag = 0;          //无CAN数时 发送其他TLV标志 

	if(!stuA5Comm.ucSendTimer)
		stuA5Comm.ucSendTimer = CAN_SEND_MIN_TIMER;
	else
	{
		return;
	}

//	if(stuA5Comm.ucCommflag==0)
//		return;
	
    for(i=0;i<SEND_TO_A5_BUFF_NUM;i++)
    {
    
		if(m>=SEND_TO_A5_BUFF_NUM)
			m = 0;      
        if(stuA5Comm.aSenDataBuffFlag[m]==1)
        {
           
           if(stuA5Comm.ucSN>=255)
		       stuA5Comm.ucSN = 0;
		   else
		   	   stuA5Comm.ucSN++;
		   
           A5_MCU_SendToA5CANData(0x01, stuA5Comm.ucSN, m, 1);  //发送
           stuA5Comm.aSenDataBuffFlag[m] = 0;          //标志位清零
		   ucSendOverTimer = CAN_SEND_MAX_COUNTER;
           m++;
           break;               
		}
		else
		{
			m++;
		}							
	}

	if(ucSendOverTimer)         //未有接收完成的CAN帧数据1秒钟
	{
		ucSendOverTimer--;
        if(!ucSendOverTimer)
        {
            ucSendOverTimer = CAN_SEND_MAX_COUNTER;
			ucFlag = 0;
            for(i=0;i<SEND_TO_A5_BUFF_NUM;i++)
    	    {
                if(m>=SEND_TO_A5_BUFF_NUM)
    			    m = 0;  
                if(stuA5Comm.aSenDataBuffFlag[m]==2)
                {
                   if(stuA5Comm.ucSN>=255)
    			       stuA5Comm.ucSN = 0;
    			   else
    		   	       stuA5Comm.ucSN++;
    			   
                   stuA5Comm.aSenDataBuffFlag[m]=0xFF;  //抢断
                   A5_MCU_SendToA5CANData(0x01, stuA5Comm.ucSN, m, 1);  //发送
                   stuA5Comm.aSenDataBuffFlag[m] = 0;    //标志位清零
                   ucFlag = 1;
                   m++;
                   break;
    			}
    			else
    			{
        			m++;
    			}					
    		}    
			//无CAN数据时 其他TLV仍然需要定时发送
			if(ucFlag==0)   //发送其LV数据
			{
                if(stuA5Comm.ucSN>=255)
                    stuA5Comm.ucSN = 0;
                else
                    stuA5Comm.ucSN++;			
                A5_MCU_SendToA5CANData(0x01, stuA5Comm.ucSN, m, 0);	
			}
    	}		
	}		
}
/******************************************************************************
** 函数名称: A5_AddCanFrameToBuff
** 功能描述: 保存CAN数据帧
** 输    入: 
** 输    出: 
** 返    回: 
** 作    者: lxf
** 日    期: 2019-5-7
******************************************************************************/
#if 0
void A5_AddCanFrameToBuff(uint32 id, uint8 ucFF,uint8 ucDataLen, uint8 *data)
{
    static uint8 m = 0;
	static uint8 n = 0;

    uint8 aCanid[4];
	uint16 usCanlen = 0;
    uint8 ucCanNum = 0;
	
	aCanid[0] = (uint8)(id>>24);
	aCanid[1] = (uint8)(id>>16);
	aCanid[2] = (uint8)(id>>8);
	aCanid[3] = (uint8)(id&0xFF);
	
	if(stuA5Comm.aSenDataBuffFlag[m]==0)
	{
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //正在保存CAN数据中
        n = 0;
		
		memcpy(&stuA5Comm.aCANDatabuff[m][3],aCanid,4);
		memcpy(&stuA5Comm.aCANDatabuff[m][7],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 12;
		stuA5Comm.aCANDatabuff[m][1] = 0;
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;
		n++;		
	}
	else if(stuA5Comm.aSenDataBuffFlag[m]==2)
	{
		memcpy(&stuA5Comm.aCANDatabuff[m][3+n*12],aCanid,4);
		memcpy(&stuA5Comm.aCANDatabuff[m][7+n*12],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = n*12;
		stuA5Comm.aCANDatabuff[m][1] = (uint8)(usCanlen<<8);
		stuA5Comm.aCANDatabuff[m][2] = (uint8)(usCanlen&0xFF);
		n++;        
		if(n>=84)        //1020=12x85
		{
            n = 0;
			stuA5Comm.aSenDataBuffFlag[m] = 1;    //接收完成
			m++;
			if(m>=SEND_TO_A5_BUFF_NUM-1)
				m = 0;
		}
	}
	else if(stuA5Comm.aSenDataBuffFlag[m]==0xFF)   //发送超时抢断
	{
	    n = 0;
        m++;
		if(m>=SEND_TO_A5_BUFF_NUM-1)
			m=0;
		
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //正在保存CAN数据中
		memcpy(&stuA5Comm.aCANDatabuff[m][3],aCanid,4);
		memcpy(&stuA5Comm.aCANDatabuff[m][7],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 12;
		stuA5Comm.aCANDatabuff[m][1] = 0;
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;
		n++;		
	}
	else  //???  所开所有缓存全满 需要覆盖
	{
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //正在保存CAN数据中
        n = 0;
		
		memcpy(&stuA5Comm.aCANDatabuff[m][3],aCanid,4);
		memcpy(&stuA5Comm.aCANDatabuff[m][7],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 12;
		stuA5Comm.aCANDatabuff[m][1] = 0;
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;
		n++;	        
	}
}
#endif

void A5_AddCanFrameToBuff(uint32 id, uint8 ucFF, uint8 ucDataLen, uint8 *data)
{
    static uint8 m = 0;       //缓存序号
	static uint8 n = 0;

    uint8 aCanid[4];
	uint16 usCanlen = 0;
    static uint8 ucCanNum = 0; //CAN帧个数
	uint16 usmSec = 0;       //时间偏移量
		
	aCanid[0] = (uint8)(id>>24);
	aCanid[1] = (uint8)(id>>16);
	aCanid[2] = (uint8)(id>>8);
	aCanid[3] = (uint8)(id&0xFF);
	
	if(stuA5Comm.aSenDataBuffFlag[m]==0)
	{
	    stuA5Comm.usmSec = 0;      //开始时清0
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //将该缓存标志为正在保存CAN数据中
        n = 0;
		ucCanNum = 1;     
		if(ucFF==0)
		    usmSec &=~BIT(15);   //标准帧
		else
			usmSec |= BIT(15);   //扩展帧
		usmSec|=((usmSec&0x87FF)|(uint16)(ucDataLen<<11));
		usmSec|=((usmSec&0xF800)|(stuA5Comm.usmSec&0x07FF));

		memcpy(&stuA5Comm.aCANDatabuff[m][4],(uint8*)&stuA5Date,6);  //时间
		stuA5Comm.aCANDatabuff[m][10] = (uint8)(stuA5Date.usmSec>>8);
		stuA5Comm.aCANDatabuff[m][11] = (uint8)(stuA5Date.usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][12],aCanid,4);
		stuA5Comm.aCANDatabuff[m][16] = (uint8)(usmSec>>8);
		stuA5Comm.aCANDatabuff[m][17] = (uint8)(usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][18],data,8);//是否需要判断帧内容长度?
		stuA5Comm.aCANDatabuff[m][0] = 0x01;          //TLV--T=0x01 
		usCanlen = 9+14;
		stuA5Comm.aCANDatabuff[m][1] = 0;             //TLV--LenLowByte
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;      //TLV--LenHighByte
		stuA5Comm.aCANDatabuff[m][3] = ucCanNum;      //CAN帧个数
		n++;		
	}
	else if(stuA5Comm.aSenDataBuffFlag[m]==2)
	{
	    ucCanNum++;
		if(ucFF==0)
		    usmSec &=~BIT(15);   //标准帧
		else
			usmSec |= BIT(15);   //扩展帧
		usmSec|=(usmSec&0x87FF)|(uint16)(ucDataLen<<11);
		usmSec|=(usmSec&0xF800)|(stuA5Comm.usmSec&0x07FF);
	
		memcpy(&stuA5Comm.aCANDatabuff[m][12+n*14],aCanid,4);
		stuA5Comm.aCANDatabuff[m][16+n*14] = (uint8)(usmSec>>8);
		stuA5Comm.aCANDatabuff[m][17+n*14] = (uint8)(usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][18+n*14],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 9+ucCanNum*14;
		stuA5Comm.aCANDatabuff[m][1] = (uint8)(usCanlen>>8);
		stuA5Comm.aCANDatabuff[m][2] = (uint8)(usCanlen&0xFF);
        stuA5Comm.aCANDatabuff[m][3] = ucCanNum;

		n++;        
		if(n>=CAN_FRAME_MAX_NUM)        //1008=14x72
		{
            n = 0;
			stuA5Comm.aSenDataBuffFlag[m] = 1;    //接收完成
			m++;
			if(m>=SEND_TO_A5_BUFF_NUM)
				m = 0;
		}
	}
	else if(stuA5Comm.aSenDataBuffFlag[m]==0xFF)   //发送超时抢断 新开缓存 存入新的缓存地址
	{
	    n = 0;
        m++;
		if(m>=SEND_TO_A5_BUFF_NUM)
			m=0;

	    stuA5Comm.usmSec = 0;                 //开始时清0
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //正在保存CAN数据中

		ucCanNum = 1;
		if(ucFF==0)
		    usmSec &=~BIT(15);   //标准帧
		else
			usmSec |= BIT(15);   //扩展帧
		usmSec|=((usmSec&0x87FF)|(uint16)(ucDataLen<<11));
		usmSec|=((usmSec&0xF800)|(stuA5Comm.usmSec&0x07FF));

		memcpy(&stuA5Comm.aCANDatabuff[m][4],(uint8*)&stuA5Date,6);  //时间
		stuA5Comm.aCANDatabuff[m][10] = (uint8)(stuA5Date.usmSec>>8);
		stuA5Comm.aCANDatabuff[m][11] = (uint8)(stuA5Date.usmSec&0xFF);		
		memcpy(&stuA5Comm.aCANDatabuff[m][12],aCanid,4);
		stuA5Comm.aCANDatabuff[m][16] = (uint8)(usmSec>>8);
		stuA5Comm.aCANDatabuff[m][17] = (uint8)(usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][18],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 9+14;
		stuA5Comm.aCANDatabuff[m][1] = 0;
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;
		stuA5Comm.aCANDatabuff[m][3] = ucCanNum;
		n++;		
	}
	else  //???  所开所有缓存全满 需要覆盖
	{
	    stuA5Comm.usmSec = 0;      //开始时清0
        stuA5Comm.aSenDataBuffFlag[m] = 2;    //正在保存CAN数据中
        n = 0;
		
		ucCanNum = 1;
		if(ucFF==0)
		    usmSec &=~BIT(15);   //标准帧
		else
			usmSec |= BIT(15);   //扩展帧
		usmSec|=((usmSec&0x87FF)|(uint16)(ucDataLen<<11));
		usmSec|=((usmSec&0xF800)|(stuA5Comm.usmSec&0x07FF));

		memcpy(&stuA5Comm.aCANDatabuff[m][4],(uint8*)&stuA5Date,6);  //时间
		stuA5Comm.aCANDatabuff[m][10] = (uint8)(stuA5Date.usmSec>>8);
		stuA5Comm.aCANDatabuff[m][11] = (uint8)(stuA5Date.usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][12],aCanid,4);
		stuA5Comm.aCANDatabuff[m][16] = (uint8)(usmSec>>8);
		stuA5Comm.aCANDatabuff[m][17] = (uint8)(usmSec&0xFF);
		memcpy(&stuA5Comm.aCANDatabuff[m][18],data,8);
		stuA5Comm.aCANDatabuff[m][0] = 0x01;
		usCanlen = 9+14;
		stuA5Comm.aCANDatabuff[m][1] = 0;
		stuA5Comm.aCANDatabuff[m][2] = usCanlen;
		stuA5Comm.aCANDatabuff[m][3] = ucCanNum;
		n++;	
	}
}
//北京时间
void AccountSysDate(void)
{
    static const uint8 DayOfMonth[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    static uint8 ucflag = 0;   //为0时从RTC读取时钟
	STU_Date stuTime;
    static uint8 ucAcc = 0;

	//防止休眠 后时间 差别 需要开机后重新读取一次
	if((!ucAcc&&1==GetAccState())||stuSleep.ucReadRTCClockFlag)
	{
        ucflag = 0;
		stuSleep.ucReadRTCClockFlag = 0;
	}
    ucAcc = GetAccState();

	//休眠唤醒后需要重新读取一次
	
	
    //系统复位重启 读取时钟
    if(ucflag==0)
    {
        PC_SendDebugData((uint8 *)("Get_RTC\n"), 8, DEBUG_ANYDATA);
		stuTime = GetRTCTime_Beijin();
		if(stuTime.ucYear>=19&&stuTime.ucMon&&stuTime.ucMon<=12
			&&stuTime.ucDay&&stuTime.ucDay<=31)
		{
    		stuA5Date.ucYear = stuTime.ucYear;
    		stuA5Date.ucMon = stuTime.ucMon;
    		stuA5Date.ucDay = stuTime.ucDay;
    		stuA5Date.ucHour = stuTime.ucHour;
    		stuA5Date.ucMin = stuTime.ucMin;
    		stuA5Date.ucSec = stuTime.ucSec;
		}
		ucflag =1;
	}


	//自定义计时器
    if(stuA5Date.usmSectemp>999)
    {
        stuA5Date.usmSectemp = stuA5Date.usmSectemp - 1000;
		stuA5Date.usmSec = stuA5Date.usmSectemp;
		
        if((++stuA5Date.ucSec)>59)
        {
            stuA5Date.ucSec=0;
            if((++stuA5Date.ucMin)>59)
            {
                stuA5Date.ucMin=0;
                if((++stuA5Date.ucHour)>23)
                {
                    stuA5Date.ucHour=0;
                    stuA5Date.ucDay++;
             		if((!(stuA5Date.ucYear%400)||!(stuA5Date.ucYear%4)&&(stuA5Date.ucYear%100))&&stuA5Date.ucMon==2)
             		{
             			if(stuA5Date.ucDay>29)
             			{
             				stuA5Date.ucDay=1;
             				stuA5Date.ucMon=3;
             			}
             		}
             		else if(stuA5Date.ucDay>DayOfMonth[stuA5Date.ucMon-1])
             		{
             			stuA5Date.ucDay=1;
             			if((++stuA5Date.ucMon)>12)
             			{
             				stuA5Date.ucMon=1;
             				stuA5Date.ucYear++;
             			}
             		}
                }
            }
        }         
    }
	else
	{
		stuA5Date.usmSec = stuA5Date.usmSectemp;
	}
}


