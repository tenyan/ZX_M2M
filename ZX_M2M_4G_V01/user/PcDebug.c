/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: PcDebug.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为PcDebug功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-11, by  lxf, 创建本文件
 *
 */
#include "config.h"
//-----头文件调用------------------------------------------------------------
#include "SystemWorkMode.h"
#include "PcHardware.h"
#define DEV_TTYS0 "/dev/ttyS0"

//-----外部变量定义------------------------------------------------------------

extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
//-----内部变量定义------------------------------------------------------------
//static uint8  f_ucGprsSmsBuff[2*PRINTBUFF+1];           //GPRS及SMS打印缓冲区
static uint8 PCSendBuff[DEBUG_DMA_UART_Rx_BSIZE];        //PC任务发送数据缓存
//static uint16 PcDebugDataLen; 
static STUSYSMsgBus f_stuPCSendmsg={SRCDEVICE_ID_SETTOOL,1,0,0,0,0,0};


//-----内部函数声明------------------
/*****************************************************************************
调试状状态  
Bit3	1：输出SYS相关调试信息，0：不输出
Bit2	1：输出MCU CAN相关调试信息，0：不输出
Bit1	1：输出GPS相关调试信息，0：不输出
Bit0	1：输出Modem相关调试信息，0：不输出
*****************************************************************************/
uint8 SYS_GetDubugStatus(void)
{
    return g_stuSystem.ucDebugPrint;
}

/******************************************************************************
** 函数名称: PC_SendDebugData
** 功能描述: 调试输出数据信息的函数,可以被不同模块调用
** 
** 输    入: ptr:指针，指向调试输出数据的指针;
             usLen:要输出数据的长度;
             ucModuleSelect:当前该调试函数被哪个模块所调用,模块的编号
             1=GSM模块;2=GPS模块;3=MCU模块;4=可以输出任意数据
** 输    出: 无
** 返    回: 命令执行完成后会返回相应的长度，长度为0表示无调试数据
**
** 作    者: Lxf
** 日    期: 2011-07-12
**-----------------------------------------------------------------------------
*******************************************************************************/
void PC_SendDebugData(uint8* ptr,uint16 usLen,uint8 ucModuleSelect)
{
	uint8 ucDebugPrint;

    ucDebugPrint = SYS_GetDubugStatus();
    if(ucDebugPrint & BIT(0))
    {
        if(ucModuleSelect==DEBUG_GSMMODULE)
        {
            DEBUG_UART_Write(ptr, usLen);
        }
    }
    else if(ucDebugPrint & BIT(1))
    {
        if(ucModuleSelect==DEBUG_GPSMODULE)
        {
            DEBUG_UART_Write(ptr, usLen);
        }
    }
    else if(ucDebugPrint & BIT(2))
    {
        if(ucModuleSelect==DEBUG_MCUMODULE)
        {
           	DEBUG_UART_Write(ptr, usLen);
        }
    }
	/*
    else if(ucDebugPrint==4)
    {
        if(ucModuleSelect==DEBUG_RS232MODULE)
        {
            DEBUG_UART_Write(ptr, usLen);  
        }
    } 
    */
    else if(ucDebugPrint & BIT(3))
    {
		 if(ucModuleSelect==DEBUG_SYSMODULE)
        {
            DEBUG_UART_Write(ptr, usLen);  
        }
    }

    if(ucModuleSelect==DEBUG_ANYDATA)             //直接输出数据
    {
        DEBUG_UART_Write(ptr, usLen);
    }
}

/******************************************************************************
** 函数名称: PC_SendToPCData2
** 功能描述: 
** 
** 输    入: ptr:指向发送数据首字节指针；usLen:需要发送数据长度,不包含包头、包尾及校验值
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-07-13
**-----------------------------------------------------------------------------
*******************************************************************************/
void PC_SendToPCData2(uint8 *ptr, uint16 usLen)
{
	uint16 usTemp;
    uint8* senddatabuff = Public_Buf;

 	usTemp = TranslateData(ptr, usLen);
    senddatabuff[0] = 0x7B;                 //包头
    senddatabuff[1] = 0x7B;     //GPS终端
    memcpy(&senddatabuff[2],ptr,usTemp);
    senddatabuff[usTemp+2] = 0x7D;
	senddatabuff[usTemp+3] = 0x7D;

    DEBUG_UART_Write(senddatabuff, usTemp+4);

}
/******************************************************************************
** 函数名称: PC_Print
** 功能描述: 将GPRS及短信数值数据转化成字符串输出
**
** 输    入: pucSrc,输入源数据; usLen,源数据长, ucFlag,输出标志
** 输    出: 无
** 返    回: 无
**
** 作    者: 
** 日    期: 2011-08-05
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
void PC_Print(const uint8 *pucSrc, uint16 usLen, uint8 ucFlag)
{
    static uint8 i;
	if(1!=SYS_GetDubugStatus())
	{
	    return;
	}
    switch(ucFlag)
	{
	    case SMS_RECV:
			PC_SendDebugData((uint8*)"\r\n\"SMSRECV: ", 12, DEBUG_GSMMODULE);
			break;
		case SMS_SEND:
			PC_SendDebugData((uint8*)"\r\n\"SMSSEND: ", 12, DEBUG_GSMMODULE);
			break;
		case GPRS_SEND:
			PC_SendDebugData((uint8*)"\r\n\"GPRSSEND: ", 13, DEBUG_GSMMODULE);
			break;
		case GPRS_RECV:
			PC_SendDebugData((uint8*)"\r\n\"GPRSRECV: ", 13, DEBUG_GSMMODULE);
			break;
		case GPRS_STO:
			PC_SendDebugData((uint8*)"\r\n\"GPRSSTO: ", 12, DEBUG_GSMMODULE);
			break;	
		default:
			break;
	}

	while(usLen>PRINTBUFF)
	{
	    i++;
		DataToHexbuff(f_ucGprsSmsBuff, (uint8*)pucSrc+(i-1)*PRINTBUFF, PRINTBUFF);
		PC_SendDebugData(f_ucGprsSmsBuff, 2*PRINTBUFF, DEBUG_GSMMODULE);
		usLen -= PRINTBUFF;
	}
	DataToHexbuff(f_ucGprsSmsBuff, (uint8*)pucSrc+i*PRINTBUFF, usLen);
	PC_SendDebugData(f_ucGprsSmsBuff, 2*usLen, DEBUG_GSMMODULE);
	PC_SendDebugData((uint8*)"\"\r\n", 3, DEBUG_GSMMODULE);
	i=0;
}

void PC_Print(const uint8 *pucSrc, uint16 usLen, uint8 ucFlag)
{
    uint8 i=0;
	
	if(DEBUG_RS232MODULE!=SYS_GetDubugStatus())
	{
	    return;
	}
  

	while(usLen>UART0_SEND_BUFFLEN)
	{
	    i++;
		PC_SendDebugData((uint8*)pucSrc+(i-1)*UART0_SEND_BUFFLEN, UART0_SEND_BUFFLEN, DEBUG_RS232MODULE);
		usLen -= UART0_SEND_BUFFLEN;
	}
	PC_SendDebugData((uint8*)pucSrc+i*UART0_SEND_BUFFLEN, usLen, DEBUG_RS232MODULE);
	i=0;
}



void PC_ExecRecvDebugData(uint8 *p, uint16 usLen)
{

	if((p[0]==0x7B)&&(p[1]==0x7B)&&(p[usLen-1]==0x7D)&&(p[usLen-2]==0x7D))
	{
		if(usLen<17)  
        {
            return;         
        }  

        usLen-=4;
		memcpy(PCSendBuff,&p[2],usLen); 
		usLen = RestoreTranslateData(PCSendBuff, usLen);
		f_stuPCSendmsg.usSize = usLen;                //数据内容长度
        f_stuPCSendmsg.pMsgPacket = PCSendBuff;     //获取数据地址
        SYS_PutDataQ((void *)&f_stuPCSendmsg);        //发送消息
	}
    else
    {    
        memcpy(&PCSendBuff[0],p,usLen);
    }
}
#endif

void* pthread_PcDebug_Function(void* data)
{
	uint16 usLen = 0;    //缓冲区接收到的数据长度 
	uint8 *p=NULL;


    DEBUG_Uart_Init();

    while(1)
    {
        if(ReadDebugUartData(&p, &usLen))
        {     

        	if((p[0]==0x7B)&&(p[1]==0x7B)&&(p[usLen-1]==0x7D)&&(p[usLen-2]==0x7D))
        	{
        		if(usLen<17)  
                {
                   // return;         
                }  
        
                usLen-=4;
        		memcpy(PCSendBuff,&p[2],usLen); 
        		usLen = RestoreTranslateData(PCSendBuff, usLen);
                if(PCSendBuff[0]==0x05&&((PCSendBuff[15]=='P'&&PCSendBuff[16]=='W')||(PCSendBuff[15]=='R'&&PCSendBuff[16]=='C')))
                    A5_UART0_Write(PCSendBuff, usLen);				
                DealSerCmd(PCSendBuff, usLen,SRCDEVICE_ID_SETTOOL,0);	
        	}
         	   
        }
		else
		{
          //  usleep(20*One_MilliSecond);
		}
    }
}

#if 0
/******************************************************************************
** 函数名称: TaskPcDebug
** 功能描述: 与PC机通信的调试任务
** 
** 输    入: 
** 输    出: 无
** 返    回: 无
**
** 作    者: Lxf
** 日    期: 2011-07-11
**-----------------------------------------------------------------------------
*******************************************************************************/
void TaskPcDebug(void *pdata)
{
	uint16 len;
	uint8 *p;
	
    pdata = pdata;
    while(1)
    {
        if(ReadDebugUartData(&p, &len))
  		{
  			PC_ExecRecvDebugData(p, len);
        }
    }
}
#endif




//-----文件PcDebug.c结束---------------------------------------------
