/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: A5_Com1ProtocolLayer.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为处理从A5核心板接收到的uart数据文件,对应MCU的uart6口
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by , 创建本文件
 *
 */
 
#include "config.h"
#include "A5_Com1HW.h"

//-----外部变量定义------------------------------------------------------------
extern STU_A5_Date stuA5Date;
extern uint8 Public_Buf[1500];
//-----内部变量定义------------------------------------------------------------


/*
*********************************************************************************************************
*Function name	:DealA5_Com1Message
*Description	:处理接收到的来自核心板A5串口数据
*Arguments  	:msg	:来自串口的报文
*Returns    	:none		
*Author			:lxf
*Date			:2019-5-10
*Modified		:                 
*********************************************************************************************************
*/
void DealA5_Com1Message(uint8 *ptr, uint16 uslen)
{
    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand;
	uint8 ucSN,ucResult = 0;
//	uint8 aCanData[8] = {0};
//	uint8 ucCanChannel = 0;
//    uint8 ucFF = 0;
//	uint32 uiCanID;

    if((ptr[0]==0x7E)&&(ptr[uslen-2]==0x0D)&&(ptr[uslen-1]==0x0A)&uslen>8)
    {
    
    	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
        ucCommand = ptr[3];
    	ucSN = ptr[4];
    	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    	if(ucSumNum!=ptr[uslen-3])
    		return;
    	
        if(ucCommand == 0x03)
        {
            if(ptr[5]<19||(ptr[6]<1||ptr[6]>12)||(ptr[7]<1||ptr[7]>31)||ptr[8]>23||ptr[9]>59||ptr[10]>59)
    			ucResult = 1;   //时钟异常
    		else
    			ucResult = 0;
			if(ucResult==0)
			{
        		stuA5Date.ucYear = ptr[5];
        		stuA5Date.ucMon = ptr[6];
        		stuA5Date.ucDay = ptr[7];
        		stuA5Date.ucHour = ptr[8];
        		stuA5Date.ucMin = ptr[9];
        		stuA5Date.ucSec = ptr[10];
			}
            A5_MCU_SendReqCmdData(0x83, ucSN, ucResult,SRCDEVICE_ID_A5_COM1);
    	}
    }
	else  //可作为透传数据处理
	{
    	if((ptr[0]==0x7B)&&(ptr[1]==0x7B)&&(ptr[uslen-1]==0x7D)&&(ptr[uslen-2]==0x7D))
    	{
    		if(uslen<17)  
            {
                return;         
            }  
    
            uslen-=4;
    		//memcpy(PCSendBuff,&ptr[2],uslen); 
    		uslen = RestoreTranslateData(&ptr[2], uslen);
            DealSerCmd(&ptr[2], uslen, SRCDEVICE_ID_A5_COM1, 0);		   
    	}

      //  DealSerCmd(ptr, uslen, SRCDEVICE_ID_A5_COM1, 0);	 //注意回复串口选择
	}

}

/******************************************************************************
** 函数名称: A5_SendToA5Data
** 功能描述: 
** 
** 输    入: ptr:指向发送数据首字节指针；usLen:需要发送数据长度,不包含包头、包尾及校验值
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2019-05-13
**-----------------------------------------------------------------------------
*******************************************************************************/
void A5_SendToA5Data(uint8 *ptr, uint16 usLen)
{
	uint16 usTemp;
    uint8* senddatabuff = Public_Buf;

 	usTemp = TranslateData(ptr, usLen);
    senddatabuff[0] = 0x7B;                 //包头
    senddatabuff[1] = 0x7B;     //GPS终端
    memcpy(&senddatabuff[2],ptr,usTemp);
    senddatabuff[usTemp+2] = 0x7D;
	senddatabuff[usTemp+3] = 0x7D;

	RS232_UART_Write(senddatabuff, usTemp+4);
}

/*
*********************************************************************************************************
*Function name	:TaskA5_Com1Function
*Description	:处理MCU与核心处理器A5之间串口通信模块的任务
*Arguments  	:pdata:传递给任务的数据
*Returns    	:none		
*Author			:lxf
*Date			:2019-5-10
*Modified		:                 
*********************************************************************************************************
*/
void TaskA5_Com1Function(void *pdata)
{

	uint16 len;
	uint8 *p;
	
	pdata = pdata;	

	RS232_Uart_Init();
	
	while(1)
	{
		if(TRUE==ReadRS232UartData(&p, &len))
		{
            if((p[0]==0x05)&&((p[15]=='P'&&p[16]=='W')||(p[15]=='R'&&p[16]=='C')))
            {
			 //   PC_ExecRecvDebugData(p, len);
			    A5_ExecRecvDebugData(p, len);
            }
			else
			{
		        Ext_McuUartMessage(p,len);
			}        
		}
	}	
}



