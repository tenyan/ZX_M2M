/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: PcDebug.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪPcDebug����ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by  lxf, �������ļ�
 *
 */
#include "config.h"
//-----ͷ�ļ�����------------------------------------------------------------
#include "SystemWorkMode.h"
#include "PcHardware.h"

//-----�ⲿ��������------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
//-----�ڲ���������------------------------------------------------------------
//static uint8  f_ucGprsSmsBuff[2*PRINTBUFF+1];           //GPRS��SMS��ӡ������
static uint8 PCSendBuff[UART0_RECV_BUFF_LENGTH];        //PC���������ݻ���
//static uint16 PcDebugDataLen; 
static STUSYSMsgBus f_stuPCSendmsg={SRCDEVICE_ID_SETTOOL,1,0,0,0,0,0};
//-----�ڲ���������------------------
/*****************************************************************************
����״״̬  
Bit3	1�����SYS��ص�����Ϣ��0�������
Bit2	1�����MCU CAN��ص�����Ϣ��0�������
Bit1	1�����GPS��ص�����Ϣ��0�������
Bit0	1�����Modem��ص�����Ϣ��0�������
*****************************************************************************/
uint8 SYS_GetDubugStatus(void)
{
    return g_stuSystem.ucDebugPrint;
}

/******************************************************************************
** ��������: PC_SendDebugData
** ��������: �������������Ϣ�ĺ���,���Ա���ͬģ�����
** 
** ��    ��: ptr:ָ�룬ָ�����������ݵ�ָ��;
             usLen:Ҫ������ݵĳ���;
             ucModuleSelect:��ǰ�õ��Ժ������ĸ�ģ��������,ģ��ı��
             1=GSMģ��;2=GPSģ��;3=MCUģ��;4=���������������
** ��    ��: ��
** ��    ��: ����ִ����ɺ�᷵����Ӧ�ĳ��ȣ�����Ϊ0��ʾ�޵�������
**
** ��    ��: Lxf
** ��    ��: 2011-07-12
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

    if(ucModuleSelect==DEBUG_ANYDATA)             //ֱ���������
    {
        DEBUG_UART_Write(ptr, usLen);
    }
}
#if 0
//-----�ⲿ��������------------------------------------------------------------
/******************************************************************************
** ��������: PC_SendToPCData
** ��������: �����ó��������ݵĺ���
** 
** ��    ��: ptr:ָ�����������ֽ�ָ�룻usLen:��Ҫ�������ݳ���,��������ͷ����β��У��ֵ
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-13
**-----------------------------------------------------------------------------
*******************************************************************************/
void PC_SendToPCData(uint8 *ptr, uint16 usLen)
{
    uint8 senddatabuff[256];
    uint16 Crc16Value;
 
    senddatabuff[0] = FORMAT;                 //��ͷ
    senddatabuff[1] = SRCDEVICE_ID_5CGPS;     //GPS�ն�
    senddatabuff[2] = SRCDEVICE_ID_SETTOOL;   //���ó���

    memcpy(&senddatabuff[3],ptr,usLen);
    Crc16Value = CRC16(ptr, usLen);           //��ȡCRC16У��ֵ  2�ֽ�
    senddatabuff[usLen+3] = (uint8)(Crc16Value&0xFF);  
    senddatabuff[usLen+4] = (uint8)(Crc16Value>>8);
    senddatabuff[usLen+5] = FORMAT;

    usLen+=6;
    DEBUG_UART_Write(senddatabuff, usLen);

}
#endif
/******************************************************************************
** ��������: PC_SendToPCData2
** ��������: 
** 
** ��    ��: ptr:ָ�����������ֽ�ָ�룻usLen:��Ҫ�������ݳ���,��������ͷ����β��У��ֵ
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-13
**-----------------------------------------------------------------------------
*******************************************************************************/
void PC_SendToPCData2(uint8 *ptr, uint16 usLen)
{
	uint16 usTemp;
    uint8* senddatabuff = Public_Buf;

 	usTemp = TranslateData(ptr, usLen);
    senddatabuff[0] = 0x7B;                 //��ͷ
    senddatabuff[1] = 0x7B;     //GPS�ն�
    memcpy(&senddatabuff[2],ptr,usTemp);
    senddatabuff[usTemp+2] = 0x7D;
	senddatabuff[usTemp+3] = 0x7D;

    DEBUG_UART_Write(senddatabuff, usTemp+4);

}
/******************************************************************************
** ��������: PC_Print
** ��������: ��GPRS��������ֵ����ת�����ַ������
**
** ��    ��: pucSrc,����Դ����; usLen,Դ���ݳ�, ucFlag,�����־
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: ����ƽ
** ��    ��: 2011-08-05
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
#endif

void A5_ExecRecvDebugData(uint8 *p, uint16 usLen)
{
	
//	if((p[0]==0x05)&&(p[15]=='P')&&(p[16]=='W'))
	if(p[0]==0x05)
	{
		if(usLen<10)  
        {
            return;         
        }  

      //  usLen-=4;
	  // memcpy(PCSendBuff,&p[2],usLen); 
	//	usLen = RestoreTranslateData(PCSendBuff, usLen);
	    memcpy(PCSendBuff,p,usLen);
		f_stuPCSendmsg.usSize = usLen;                //�������ݳ���
        f_stuPCSendmsg.pMsgPacket = PCSendBuff;      //��ȡ���ݵ�ַ
        SYS_PutDataQ((void *)&f_stuPCSendmsg);        //������Ϣ
	}
    else
    {    
    //    memcpy(&PCSendBuff[0],p,usLen);       
    }
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
		f_stuPCSendmsg.usSize = usLen;                //�������ݳ���
        f_stuPCSendmsg.pMsgPacket = PCSendBuff;     //��ȡ���ݵ�ַ
        SYS_PutDataQ((void *)&f_stuPCSendmsg);        //������Ϣ
	}
    else
    {    
        memcpy(&PCSendBuff[0],p,usLen);       
    }
}

/******************************************************************************
** ��������: TaskPcDebug
** ��������: ��PC��ͨ�ŵĵ�������
** 
** ��    ��: 
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2011-07-11
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
  		   // RS232_UART_Write(p, len);
  			PC_ExecRecvDebugData(p, len);
        }
    }
}





//-----�ļ�PcDebug.c����---------------------------------------------
