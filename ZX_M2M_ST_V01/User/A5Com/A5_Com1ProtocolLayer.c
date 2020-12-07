/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: A5_Com1ProtocolLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�Ϊ�����A5���İ���յ���uart�����ļ�,��ӦMCU��uart6��
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by , �������ļ�
 *
 */
 
#include "config.h"
#include "A5_Com1HW.h"

//-----�ⲿ��������------------------------------------------------------------
extern STU_A5_Date stuA5Date;
extern uint8 Public_Buf[1500];
//-----�ڲ���������------------------------------------------------------------


/*
*********************************************************************************************************
*Function name	:DealA5_Com1Message
*Description	:������յ������Ժ��İ�A5��������
*Arguments  	:msg	:���Դ��ڵı���
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
    			ucResult = 1;   //ʱ���쳣
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
	else  //����Ϊ͸�����ݴ���
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

      //  DealSerCmd(ptr, uslen, SRCDEVICE_ID_A5_COM1, 0);	 //ע��ظ�����ѡ��
	}

}

/******************************************************************************
** ��������: A5_SendToA5Data
** ��������: 
** 
** ��    ��: ptr:ָ�����������ֽ�ָ�룻usLen:��Ҫ�������ݳ���,��������ͷ����β��У��ֵ
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: Lxf
** ��    ��: 2019-05-13
**-----------------------------------------------------------------------------
*******************************************************************************/
void A5_SendToA5Data(uint8 *ptr, uint16 usLen)
{
	uint16 usTemp;
    uint8* senddatabuff = Public_Buf;

 	usTemp = TranslateData(ptr, usLen);
    senddatabuff[0] = 0x7B;                 //��ͷ
    senddatabuff[1] = 0x7B;     //GPS�ն�
    memcpy(&senddatabuff[2],ptr,usTemp);
    senddatabuff[usTemp+2] = 0x7D;
	senddatabuff[usTemp+3] = 0x7D;

	RS232_UART_Write(senddatabuff, usTemp+4);
}

/*
*********************************************************************************************************
*Function name	:TaskA5_Com1Function
*Description	:����MCU����Ĵ�����A5֮�䴮��ͨ��ģ�������
*Arguments  	:pdata:���ݸ����������
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



