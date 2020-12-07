//�ļ�����Mcu_Can.h

#ifndef _MCU_CAN_
#define _MCU_CAN_

#ifdef  __cplusplus
extern "C" {
#endif

#include "config.h"
#include "McuHW.h"

STU_CommState m_stuCommState = {
	.ucCan1RcvErr = 0,					//Can1�������ݴ���,0:û���յ�����,1:�յ�����
	.usCan1RcvErrTime = 0,				//Can1û���յ����ݼ�ʱ	
	.ucCan1CommState = 0,					//Can1��������״̬ͨ��״̬��1:ͨ���쳣,0:ͨ������
	.ucRcvCan1DataFlag = 2,				//Can1��������״̬ 1:�յ���CAN1���ݣ�2:û���յ�CAN1����

	.ucCan2RcvErr = 0,					//Can2�������ݴ���,0:û���յ�����,1:�յ�����
	.usCan2RcvErrTime = 0,				//Can2û���յ����ݼ�ʱ	
	.ucCan2CommState = 1,					//Can2ͨ��״̬��1:ͨ���쳣,0:ͨ������
	.ucRcvCan2DataFlag = 2,				//Can2��������״̬ 1:�յ���CAN1���ݣ�2:û���յ�CAN1����
};
STU_Date McuDataTime;					//���ݲɼ���ʱ��
uint16  McuDataTime_ms;					//���ݲɼ�ʱ���ms����
STU_CanFrame CanData[MAX_CAN_FRAME_NUM];//CAN���ݻ��� 
STU_McuCmd m_stuMcuCmd = {
	.usSpeedLimit= 3500,
    .ucCmdCheckFlag = 0,
	
};

stuXWFaultCode g_stuXWFaultCodeUp,g_stuXWFaultCodeRecv; 
faultCode m_stuActiveFaultCodeList[MAX_ACTIVE_FAULTCODE_NUM];//��ǰ��������й������б����20����ȥ����Ĺ�������0���
static uint8 f_ucNewFaultCodeFlag;		//�������µ�DTC��־,0=��,1=�� 
extern STUSYSParamSet g_stuSYSParamSet;
extern struct STU_Sysstruct STU_Systemstate;
extern STUMCUFirmware stu_McuFirmware;
extern stuFirmwareUpdate FirmwareUpdate;
extern STU_A5Comm stuA5Comm;
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;


static  const  uint16  tbl[256]={ 
0x0000,  0x1021, 0x2042,  0x3063,  0x4084,  0x50a5,  0x60c6,  0x70e7, 
0x8108,  0x9129, 0xa14a,  0xb16b,  0xc18c,  0xd1ad,  0xe1ce,  0xf1ef,  
0x1231,  0x0210, 0x3273,  0x2252,  0x52b5,  0x4294,  0x72f7,  0x62d6, 
0x9339,  0x8318, 0xb37b,  0xa35a,  0xd3bd,  0xc39c,  0xf3ff,  0xe3de, 
0x2462,  0x3443, 0x0420,  0x1401,  0x64e6,  0x74c7,  0x44a4,  0x5485, 
0xa56a,  0xb54b, 0x8528,  0x9509,  0xe5ee,  0xf5cf,  0xc5ac,  0xd58d, 
0x3653,  0x2672, 0x1611,  0x0630,  0x76d7,  0x66f6,  0x5695,  0x46b4, 
0xb75b,  0xa77a, 0x9719,  0x8738,  0xf7df,  0xe7fe,  0xd79d,  0xc7bc, 
0x48c4,  0x58e5, 0x6886,  0x78a7,  0x0840,  0x1861,  0x2802,  0x3823, 
0xc9cc,  0xd9ed, 0xe98e,  0xf9af,  0x8948,  0x9969,  0xa90a,  0xb92b, 
0x5af5,  0x4ad4, 0x7ab7,  0x6a96,  0x1a71,  0x0a50,  0x3a33,  0x2a12, 
0xdbfd,  0xcbdc, 0xfbbf,  0xeb9e,  0x9b79,  0x8b58,  0xbb3b,  0xab1a, 
0x6ca6,  0x7c87, 0x4ce4,  0x5cc5,  0x2c22,  0x3c03,  0x0c60,  0x1c41, 
0xedae,  0xfd8f, 0xcdec,  0xddcd,  0xad2a,  0xbd0b,  0x8d68,  0x9d49, 
0x7e97,  0x6eb6, 0x5ed5,  0x4ef4,  0x3e13,  0x2e32,  0x1e51,  0x0e70, 
0xff9f,  0xefbe, 0xdfdd,  0xcffc,  0xbf1b,  0xaf3a,  0x9f59,  0x8f78, 
0x9188,  0x81a9, 0xb1ca,  0xa1eb,  0xd10c,  0xc12d,  0xf14e,  0xe16f,  
0x1080,  0x00a1, 0x30c2,  0x20e3,  0x5004,  0x4025,  0x7046,  0x6067, 
0x83b9,  0x9398, 0xa3fb,  0xb3da,  0xc33d,  0xd31c,  0xe37f,  0xf35e, 
0x02b1,  0x1290, 0x22f3,  0x32d2,  0x4235,  0x5214,  0x6277,  0x7256, 
0xb5ea,  0xa5cb, 0x95a8,  0x8589,  0xf56e,  0xe54f,  0xd52c,  0xc50d, 
0x34e2,  0x24c3, 0x14a0,  0x0481,  0x7466,  0x6447,  0x5424,  0x4405, 
0xa7db,  0xb7fa, 0x8799,  0x97b8,  0xe75f,  0xf77e,  0xc71d,  0xd73c, 
0x26d3,  0x36f2, 0x0691,  0x16b0,  0x6657,  0x7676,  0x4615,  0x5634, 
0xd94c,  0xc96d, 0xf90e,  0xe92f,  0x99c8,  0x89e9,  0xb98a,  0xa9ab, 
0x5844,  0x4865, 0x7806,  0x6827,  0x18c0,  0x08e1,  0x3882,  0x28a3, 
0xcb7d,  0xdb5c, 0xeb3f,  0xfb1e,  0x8bf9,  0x9bd8,  0xabbb,  0xbb9a, 
0x4a75,  0x5a54, 0x6a37,  0x7a16,  0x0af1,  0x1ad0,  0x2ab3,  0x3a92, 
0xfd2e,  0xed0f, 0xdd6c,  0xcd4d,  0xbdaa,  0xad8b,  0x9de8,  0x8dc9, 
0x7c26,  0x6c07, 0x5c64,  0x4c45,  0x3ca2,  0x2c83,  0x1ce0,  0x0cc1, 
0xef1f,  0xff3e, 0xcf5d,  0xdf7c,  0xaf9b,  0xbfba,  0x8fd9,  0x9ff8,  
0x6e17,  0x7e36, 0x4e55,  0x5e74,  0x2e93,  0x3eb2,  0x0ed1,  0x1ef0  
};

STU_MCUSend_GPSData stuMcuSend_Gpsdata;   //����������Ͷ�λ��Ϣ
#if 0
/*
*********************************************************************************************************
*Function name	:McuSaveToMemory
*Description	:����Mcuģ����ز������洢�������Ա�����ʼ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2016-9-22
*Modified		:
*********************************************************************************************************
*/

void McuSaveToMemory()
{
	uint8 aucTemp[20];

	aucTemp[0] = 0xaa;
	aucTemp[1] =  m_stuMcuCmd.LockControlFlag;
	aucTemp[2] =  m_stuMcuCmd.ucRespSerFlag;
	aucTemp[3] =  m_stuMcuCmd.usLockOneSq & 0xff;
	aucTemp[4] = (m_stuMcuCmd.usLockOneSq>>8) & 0xff;
	aucTemp[5] =  m_stuMcuCmd.usLockSecSq & 0xff;
	aucTemp[6] = (m_stuMcuCmd.usLockSecSq>>8) & 0xff;
	aucTemp[7] =  m_stuMcuCmd.usUnLockSq & 0xff;
	aucTemp[8] = (m_stuMcuCmd.usUnLockSq>>8) & 0xff;
	aucTemp[9] =  m_stuMcuCmd.usMonitorSq & 0xff;
	aucTemp[10] =(m_stuMcuCmd.usMonitorSq>>8) & 0xff;
	
	aucTemp[19] = SumCalc(aucTemp, 19);
	//CPU_IntDis();//0413
	WriteToFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp);
	// CPU_IntEn();
	OSTimeDly(OS_TICKS_PER_SEC/100);
	WriteToFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp);
}
/*
*********************************************************************************************************
*Function name	:McuReadFromMemory
*Description	:�Ӵ洢���ж���Mcuģ����ز��������Ա�����ʼ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2016-9-22
*Modified		:
*********************************************************************************************************
*/
void McuReadFromMemory()
{
	uint8 aucTemp1[20];
	uint8 sum1 = 0;
	uint8 aucTemp2[20];
	uint8 sum2 = 0;
	uint8 m_aucDataSave[20];		//���浽�����flash�����ݻ��棬
	uint8 flag = 0;
	
	ReadFromFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp1);
	sum1 = SumCalc(aucTemp1, 19);
	
    ReadFromFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp2);
	sum2 = SumCalc(aucTemp2,19);

	if((aucTemp1[0]==0xAA) && (sum1==aucTemp1[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp1[0],20);
		if((aucTemp2[0]!=0xAA) || (sum2!=aucTemp2[19]))
		{
			PC_SendDebugData((uint8 *)("MCUSAVE 2 ERR"), 13, DEBUG_ANYDATA);
			WriteToFlash(FLASH_PAGEADDR_MCU2, 0, 20, aucTemp1);
		}
	}
	else if((aucTemp2[0]==0xAA) && (sum2==aucTemp2[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp2[0],20);
		PC_SendDebugData((uint8 *)("MCUSAVE 1 ERR"), 13, DEBUG_ANYDATA);
		WriteToFlash(FLASH_PAGEADDR_MCU1, 0, 20, aucTemp2);
	}
	else
	{
		PC_SendDebugData((uint8 *)("MCUSAVE 1_2 ERR"), 15, DEBUG_ANYDATA);
	}
	
	if(1==flag)
	{
		m_stuMcuCmd.LockControlFlag = m_aucDataSave[1];
		m_stuMcuCmd.ucRespSerFlag   = m_aucDataSave[2];
		m_stuMcuCmd.usLockOneSq		= m_aucDataSave[3] + (m_aucDataSave[4]<<8);	
		m_stuMcuCmd.usLockSecSq		= m_aucDataSave[5] + (m_aucDataSave[6]<<8);	
		m_stuMcuCmd.usUnLockSq		= m_aucDataSave[7] + (m_aucDataSave[8]<<8);
		m_stuMcuCmd.usMonitorSq 	= m_aucDataSave[9] + (m_aucDataSave[10]<<8);
	}
}

/**********************************************************************************
** ��������: void MCU_GPSStateSend(void)
** ��������  ���ڶ�ʱ��MCU����GPS����״̬���ݺ���
** ��        ��: �޲���
** �䡡      ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����     �� :
** �ա�     �� : 2016-12-25
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 MCU_CmdLockSend(void)
{
	uint8 Sendbuff[8]={0,0,0,0,0,0,0,0};
	uint8 sendbuff2[8] = {0,0,0,0,0,0,0,0};
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	uint16 password = 0;
	uint8 ucOpenModeFlag = 0;
	static uint16 seed=1;

    static uint8 lockcntflag = 0;
	
    if(m_stuMcuCmd.LockControlFlag&BIT(0))
    {
        if(m_stuMcuCmd.LockControlFlag&BIT(1)) 
        {
            Sendbuff[1] = 35;    //�������ģʽ
            ucOpenModeFlag = 1;
        }
        else
            Sendbuff[1] = 36;    //�رռ��ģʽ        
    }

    if(lockcntflag==0)
   	{
    	if(m_stuMcuCmd.LockControlFlag&BIT(2))
    	{
        	if(m_stuMcuCmd.LockControlFlag&BIT(3))  //һ������
        	{
            	Sendbuff[0] = 31;
        	}
        	else                          //һ������
        	{
            	Sendbuff[0] = 32;
        	}
      	}
		else
		{
       //     Sendbuff[0] = ucLockTemp;
		}
		lockcntflag=1;
		//ucLockTemp = Sendbuff[0];
    }
	else
	{
    	if(m_stuMcuCmd.LockControlFlag&BIT(4))
    	{
            if(m_stuMcuCmd.LockControlFlag&BIT(5))  //��������
            {
                Sendbuff[0]=33;
            }
            else                             //��������
            {
                Sendbuff[0]=34;
            }
    	}
		else
		{
       //     Sendbuff[0] = ucLockTemp;
		}
     //   ucLockTemp = Sendbuff[0];		
       	lockcntflag=0;
	}

	if(m_stuMcuCmd.LockControlFlag&0x15 )
	{	
       
		srand(seed);		//���������
	    A = rand();
	    seed = A;
		
		A_L = A & 0xff;
		A_H = (A>>8) & 0xff;
		password = tbl[A_H];
		C = ~(password ^ A);

		sendbuff2[0]= Sendbuff[0];
		sendbuff2[1] = Sendbuff[1];
		
		sendbuff2[4] = A_H;
		sendbuff2[5] = A_L;
		sendbuff2[6] = (C>>8) & 0xff;
		sendbuff2[7] = C & 0xff;
        
		
		if(ucOpenModeFlag==1)
		{
            Sendbuff[3] = (uint8)(m_stuMcuCmd.uiKey&0xFF);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiKey>>8);
    	    Sendbuff[5] = (uint8)(m_stuMcuCmd.uiKey>>16);
    	    Sendbuff[6] = (uint8)(m_stuMcuCmd.uiKey>>24);
		}
		else
		{
            Sendbuff[3] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiFs>>8);
    	    Sendbuff[5] = (uint8)(m_stuMcuCmd.uiFs>>16);
    	    Sendbuff[6] = (uint8)(m_stuMcuCmd.uiFs>>24);
		}
     	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe1af4, 8, Sendbuff);
		OSTimeDly(5);
		CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0af4, 8, sendbuff2);
		return 1;
	}
	return 0;
}
#endif

/**********************************************************************************
** ��������: void MCU_GPSStateSend(void)
** ��������  ���ڶ�ʱ��MCU����GPS����״̬���ݺ���
** ��        ��: �޲���
** �䡡      ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����     �� :
** �ա�     �� : 2016-12-25
**--------------------------------------------------------------------------------
*********************************************************************************/
#if 0
void MCU_CmdOldLockSend(void)
{
	uint8 Sendbuff[8]={0,0,0,0,0,0,0,0};
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	uint16 password = 0;
	static uint16 seed=1;
   
	srand(seed);		//���������
    A = rand();
    seed = A;
	
	A_L = A & 0xff;
	A_H = (A>>8) & 0xff;
	password = tbl[A_H];
	C = ~(password ^ A);

	Sendbuff[0] = stuMcuSend_Gpsdata.aControlData[0];
	Sendbuff[1] = stuMcuSend_Gpsdata.aControlData[1];
	Sendbuff[4] = A_H;
	Sendbuff[5] = A_L;
	Sendbuff[6] = (C>>8) & 0xff;
	Sendbuff[7] = C & 0xff;
 	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0af4, 8, Sendbuff);
}
#endif
uint8 MCU_SendNewLockCmd(uint8 *ptr)
{
    uint8 ucCanChannel;
	uint8 ucFF;
    uint32 uiCanID = 0;
	uint8 ucCanDatalen = 0;
	uint8 aCanData[8] = {0};
	
	ucCanChannel = ptr[0];
	if(ucCanChannel>1)
		return FALSE;
	
	ucFF = ptr[1];
	if(ucFF>1)
		return FALSE;	

    uiCanID = (uint32)(ptr[2]<<24) + (uint32)(ptr[3]<<16) +(uint32)(ptr[4]<<8) + ptr[5];
    ucCanDatalen = ptr[6];
    if(ucCanDatalen > 8)
		return FALSE;
	
    memcpy(aCanData,&ptr[7],8);
 //   memcpy(stuMcuSend_Gpsdata.aControlData,&aCanData[0],2);
	
	if(aCanData[0]==35)    //�������ģʽ  ������Կ
	{
        aCanData[1] = (uint8)(m_stuMcuCmd.uiKey&0xFF);
	    aCanData[2] = (uint8)(m_stuMcuCmd.uiKey>>8);
	    aCanData[3] = (uint8)(m_stuMcuCmd.uiKey>>16);
	    aCanData[4] = (uint8)(m_stuMcuCmd.uiKey>>24);
	}
	else
	{
        aCanData[1] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
	    aCanData[2] = (uint8)(m_stuMcuCmd.uiFs>>8);
	    aCanData[3] = (uint8)(m_stuMcuCmd.uiFs>>16);
	    aCanData[4] = (uint8)(m_stuMcuCmd.uiFs>>24);
	}

 //   stuMcuSend_Gpsdata.ucCount = 2;  //���;�Э������

    CanWrite(ucCanChannel, ucFF, uiCanID, ucCanDatalen, aCanData);	
	return TRUE;
}

#if 0
/**********************************************************************************
** ��������: void MCU_GPSStateSend(void)
** ��������  ���ڶ�ʱ��MCU����GPS����״̬���ݺ���
** ��        ��: �޲���
** �䡡      ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����     �� : Lxf
** �ա�     �� : 2012-8-13
**--------------------------------------------------------------------------------
*********************************************************************************/
void MCU_GPSSendWorkState(void)
{

	uint8 arr[8];
	uint16 A=0;
	uint8 A_H = 0;
	uint8 A_L = 0;
	uint16 C=0;
	static uint16 seed=1;
	uint16 password = 0;
	static uint8 heart_flag=0;
	
    if(heart_flag==0)
    {
        arr[0]=0x55;
		heart_flag=1;
    }
	else
    {
		 arr[0]=0xaa;
		 heart_flag=0;
	}	 	

    if(GetSwitch3State()&BIT(0))   //��Դ
    {
        arr[1]=13;                 //��ع���
    }
    else 
    {
        arr[1]=12;                 //���Դ����
    }
    
    if(GPS_GetAnteState())      //gps����
    {
        arr[2]=15;              //GPS���߹���  
    }
    else
    {
        arr[2]=14;              //GPS��������
    }

#if 0	
	if(0==Get_SIMCardState()||!(STU_Systemstate.DisassmbleSwitch&BIT(4)))
	{
        arr[3] = 16;        //SIM������
	}
	else
	{
        arr[3] = 17;       //SIM������
	}
#endif
    arr[3] = 16;           //SIM������

	srand(seed);	              //���������
    A = rand();
    seed = A;
	
	A_L = A & 0xff;
	A_H = (A>>8) & 0xff;
	password = tbl[A_H];
	C = ~(password ^ A);
	
	arr[4] = A_H;
	arr[5] = A_L;
	arr[6] = (C>>8) & 0xff;
	arr[7] = C & 0xff;
//	if(stuA5Comm.ucCommflag==0)
    CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe08f4, 8, arr);
}

#endif
//GPS�ն����� �յ�MCU���������У�鷢��
void MCU_GPSSendWorkState_check(uint8 *ptr)
{

	uint8 buff[8];
	static uint8 heart_flag=0;
    uint32 uis;

    uint8 n1 = 3,n2 = 2,n3 = 0,n4 = 2;

	uis = ptr[2]+(ptr[4]<<8)+(m_stuMcuCmd.uiKey&0x00FF0000)+((m_stuMcuCmd.uiKey&0xFF)<<24);
   
    m_stuMcuCmd.uiFs = (uis%17)*(uis%19)*n4+(uis%29)*(uis%57)*n3+(uis%25)*(uis%1352)*n2+(uis%5273)*n1;
   
	
    if(heart_flag==0)
    {
        buff[0]=0x55;
		heart_flag=1;
    }
	else
    {
		 buff[0]=0xaa;
		 heart_flag=0;
	}	 	

    if(GetSwitch3State()&BIT(0))   //��Դ
    {
        buff[1]=13;                 //��ع���
    }
    else 
    {
        buff[1]=12;                 //���Դ����
    }

	
    if(GPS_GetAnteState())      //gps����
    {
        buff[2]=15;              //GPS���߹���  
    }
    else
    {
        buff[2]=14;              //GPS��������
    }

#if 0	
	if(0==Get_SIMCardState()||!(STU_Systemstate.DisassmbleSwitch&BIT(4)))
	{
        arr[3] = 16;        //SIM������
	}
	else
	{
        arr[3] = 17;       //SIM������
	}
#endif
    buff[3] = 16;           //SIM������
    buff[4] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
	buff[5] = (uint8)(m_stuMcuCmd.uiFs>>8);
	buff[6] = (uint8)(m_stuMcuCmd.uiFs>>16);
	buff[7] = (uint8)(m_stuMcuCmd.uiFs>>24);
//	if(stuA5Comm.ucCommflag==0)
    CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB01B1, 8, buff);
}

#if 0
//GPS��MCU����λ��״̬ ���θ߶ȵ�
void MCU_GPSSendPostionState(void)
{
	uint8 arr[8] = {0};
	uint8 ucTemp =0;
	uint16 usHeight = 0;
	
	arr[0] = (GPS_GetState()& BIT(1))?1:0;	
    if(1==GPS_GetLatitudeHemisphere())
     	ucTemp |= BIT(0);
	else
		ucTemp&=~BIT(0);
    if(1==GPS_GetLongitudeHemisphere())
     	ucTemp |= BIT(1);
	else
		ucTemp&=~BIT(1);
    if(GPS_GetHeight())
    {
     	ucTemp |= BIT(7);
        usHeight = GPS_GetHeight();
	}
	else
	{
		ucTemp&=~BIT(7);
		usHeight = 0-GPS_GetHeight();
	}	
	
	arr[1] = ucTemp;	
	arr[2] = usHeight & 0xff;
	arr[3] = (usHeight>>8) & 0xff;
	arr[4] = GPS_GetSatellitenums();

    if(!(GPS_GetState() & BIT(1)))   //����λ
    {
		arr[1]= 0;
        arr[2]= 0;
		arr[3]= 0;
        arr[4]= 0;
		arr[5]= 0;
        arr[6]= 0;
		arr[7]= 0;
	}
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff6, 8, arr);
}

//GPS��MCU����λ����Ϣ ��γ��
void MCU_GPSSendPostionInformation(void)
{
	uint8 arr[8] = {0};
	uint8 *p;

	if(GPS_GetState() & BIT(1))      //��λ
	{
		
		p = GPS_GetLongitude_Original();
		arr[0] = 0xff & DecToUint16(p,3);
		arr[1] = 0xff & DecToUint16(p+3,2);
		arr[2] = 0xff & DecToUint16(p+6,2);
		arr[3] = 0xff & DecToUint16(p+8,2);
		p = GPS_GetLatitude_Original();
		arr[4] = 0xff & DecToUint16(p,2);
		arr[5] = 0xff & DecToUint16(p+2,2);
		arr[6] = 0xff & DecToUint16(p+5,2);
		arr[7] = 0xff & DecToUint16(p+7,2);		
	}
	else                    //����λ 0
	{
        arr[0]= 0;
		arr[1]= 0;
        arr[2]= 0;
		arr[3]= 0;
        arr[4]= 0;
		arr[5]= 0;
        arr[6]= 0;
		arr[7]= 0;
	}
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff7, 8, arr);
}
#endif
void MCU_GPSSendPostionState(void)
{
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB03B1, 8, stuMcuSend_Gpsdata.aGpsstate);
}

void MCU_GPSSendPostionInformation(void)
{
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB04B1, 8, stuMcuSend_Gpsdata.aGpsData);
}
/**********************************************************************************
** ��������: MCU_SendCan1Data
** ��������: �ú���ÿ100msִ��һ��
             ÿ���2S���һ�ε�ǰ�洢�����������������������Ǳ���
**           �����Ϊ3�� 1.���ģʽ���� 2.һ���������� 3.������������ ��������
**           ��ѭ���Ǳ��ͣ�ֱ���յ��Ǳ���Ӧ���ɾ��������ֹͣ����
** ��        ��: �޲���
** �䡡      ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����     �� : lxf
** �ա�     �� : 2016��12��25��                        
**--------------------------------------------------------------------------------
*********************************************************************************/
void MCU_SendCan1Data(void)
{
    static uint8 SendFlag=0;           //����GPS״̬���������������ѡ���־
                                       //0=����״̬����0x55;1=���Ͷ�λ����,2=���;�γ��
  
    if(GetAccState()||(1==GetCanRcvState(CAN_CHANNEL1))||MCU_GetVoltageState())         //�ղ���MCU���ݣ��򲻷�������
    {
        if(m_stuMcuCmd.usSendMCULockTimer)
			m_stuMcuCmd.usSendMCULockTimer--;
		if(m_stuMcuCmd.usSendGPSDataTimer)
			m_stuMcuCmd.usSendGPSDataTimer--;
		
		if(!m_stuMcuCmd.usSendMCULockTimer) 
		{
			m_stuMcuCmd.usSendMCULockTimer = MCU_GPSCMD_SEND_TIME;      //��ʱΪ1����
         /*
			if(1==MCU_CmdLockSend())      //�����ݷ�
			{
    			if(0==m_stuMcuCmd.usSendGPSDataTimer)
    				m_stuMcuCmd.usSendGPSDataTimer = 1;
			}
			*/
   	 	}
		if(!m_stuMcuCmd.usSendGPSDataTimer)
		{
	        if(0==SendFlag)
			{
	       // 	MCU_GPSSendWorkState();
				SendFlag = 1;
			}
			else if(1==SendFlag)
			{
				MCU_GPSSendPostionState();
				SendFlag = 2;
			}
			else if(2==SendFlag)
			{
				MCU_GPSSendPostionInformation();
				SendFlag = 0;
				m_stuMcuCmd.usSendGPSDataTimer = MCU_GPSSTATE_SEND_TIME; //��ʱ5����
			}
		}
    }
}

//���CAN֡��ַ����
void MCU_ClearCanData(void)
{
    uint8 i = 0;
	
	for(i=0; i<MAX_CAN_FRAME_NUM; i++)
	{
	    CanData[i].id = 0;
	}
    g_stuXWFaultCodeRecv.ucFaultCodeNum = 0;
	g_stuXWFaultCodeUp.ucFaultCodeNum = 0;
	g_stuXWFaultCodeRecv.ucVehicleType = 0;
	for(i=0;i<MAX_CAN_FaultCode_NUM;i++)
	{
        g_stuXWFaultCodeRecv.CanData[i].id = 0;
		g_stuXWFaultCodeUp.CanData[i].id = 0;
	}	
}
/*********************************************************************************************************
*Function name	:MCUCommStateJudge
*Description	:MCUͨ��״̬�жϣ��˺���1sִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-7-9
*Modified		:
********************************************************************************************************/
void MCUCommStateJudge(void)
{
	static uint16 usACCOnTime = TIME_COMM_UNNORMAL;          // Կ�׿���Ϊ�߳���ʱ��
	static uint8 ucACCPre;                                   // ǰһ��Կ�׿��ص�״̬
	uint8 ucACCCurrent;                                      // ��ǰԿ�׿��ص�״̬
	static uint8 ucPWRCloseTimer = 0;            //5����
	static uint8 ucCanRstflag = 0;               //0=������������CAN��1=��������������CAN
	
	ucACCCurrent =	GetAccState();    // ��ȡ��ǰԿ�׿���״̬ 
	if(ucACCCurrent != ucACCPre)
	{
		if(!ucACCCurrent)             // �ɸߵ��͵ı仯
		{
			usACCOnTime = TIME_COMM_UNNORMAL;
			ucCanRstflag = 0;
		}
	}

	if(ucACCCurrent)                  // ���Կ�׿���Ϊ��
	{
		if(usACCOnTime)
			usACCOnTime--;

		if(!usACCOnTime && m_stuCommState.ucRcvCan1DataFlag == 2&&ucCanRstflag==0)
		{
		    ucCanRstflag = 1;
			m_stuCommState.ucCan1CommState = 1;   //CANͨѶ�쳣
            PC_SendDebugData((uint8 *)("CAN OFF"), 7, DEBUG_ANYDATA);
            POWEROFF_CAN();      //�ر�CANģ���Դ
            ucPWRCloseTimer = 5;
		}
		if(ucPWRCloseTimer)
		{
		    if(!(--ucPWRCloseTimer))
            {
                McuInit();      //�ص��ʱ5�� ���¶�CANģ���ϵ粢���г�ʼ��
                PC_SendDebugData((uint8 *)("CAN REST"), 8, DEBUG_ANYDATA);
		    }
		}	
	}
	
	if(m_stuCommState.ucRcvCan1DataFlag==1)
	{
        ucCanRstflag = 0;
	}
	
    ucACCPre = ucACCCurrent;	
}

//ͳ��can�ӿ�û���յ����ݼ�ʱ
//�ú���10ms����һ��
void CanRcvDataTimer()
{
	if(++m_stuCommState.usCan1RcvErrTime > MAX_CAN1_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan1DataFlag = 2;
		MCU_ClearCanData();     //CANͨѶ�жϺ����ԭ�б���CAN����
	}
	if(++m_stuCommState.usCan2RcvErrTime > MAX_CAN2_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan2DataFlag = 2;
	}
}


/*
*********************************************************************************************************
*Function name	:GetMcuFaultCode
*Description	:��ȡ�������������
*Arguments  	:faultCodeData	:ָ�����������������ݻ����ָ��
*Returns    	:��������������0��ʾû�й������������
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetMcuFaultCode(uint8 *faultCodeData)
{
	uint8 i;
	uint8 faultCodeNum=0;

	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode)
		{
			*faultCodeData     = m_stuActiveFaultCodeList[i].faultCode & 0xff;
			*(faultCodeData+1) = (m_stuActiveFaultCodeList[i].faultCode >> 8) & 0xff;
			*(faultCodeData+2) = (m_stuActiveFaultCodeList[i].faultCode >> 16) & 0xff;
			*(faultCodeData+3) = (m_stuActiveFaultCodeList[i].faultCode >> 24) & 0xff;
		    faultCodeData += 4;
			faultCodeNum++;
		}
	}
	return faultCodeNum;
}

/*
*********************************************************************************************************
*Function name	:IsNewMcuFaultCode
*Description	:��ȡ�Ƿ����µ�DTC��Ҫ�ϱ�
*Arguments  	:
*Returns    	:0=��,1=��
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 IsNewMcuFaultCode(void)
{
	return f_ucNewFaultCodeFlag;
}
/*
*********************************************************************************************************
*Function name	:ClearNewMcuFaultCodeFlag
*Description	:������µ�DTC��Ҫ�ϱ���־,�˲����ڳɹ��ϱ���ƽ̨��ִ��
*Arguments  	:
*Returns    	:
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
void ClearNewMcuFaultCodeFlag()
{
	f_ucNewFaultCodeFlag = 0;
}
/******************************************************************************
** ��������: GetCanRcvState
** ��������: ��ȡCAN����״̬
** ��    ��:
** ��    ��: ��
** ��    ��: 1=����,2=ֹͣ
** ��    ��: hhm
** ��    ��: 2016-07-01
**-----------------------------------------------------------------------------
*******************************************************************************/
//��ȡCAN����״̬,1:�յ���CAN1���ݣ�2:û���յ�CAN1����
uint8 GetCanRcvState(uint8 channel)
{
	if(CAN_CHANNEL1==channel)
		return m_stuCommState.ucRcvCan1DataFlag;
	else if(CAN_CHANNEL2==channel)
		return m_stuCommState.ucRcvCan2DataFlag;
	else
		return 0;
}

void AddCanFrameToBuff(uint32 id, uint8 *data)
{
	uint8 i;
	uint8 j=0xff;
	uint8 flag = 0;


	for(i=0; i<MAX_CAN_FRAME_NUM; i++)    //?�˴�����Ӧ�ò����趨�����ź���
	{
		if(CanData[i].id==id)
		{
			memcpy(CanData[i].aucData, data, 8);
			flag = 1;
			break;
		}
		else
		{
			if((CanData[i].id==0) && (j==0xff))//???
			{
				j = i;
			}
		}
	}
			
	
	if(1==flag)
		return;
	else
	{
		if(j!=0xff)
		{
			CanData[j].id = id;
			memcpy(CanData[j].aucData, data, 8);  //???
		}
	}
}
/*
*********************************************************************************************************
*Function name	:AddToFaultCodeList
*Description	:���յ�����CAN1�Ĺ����������������������б��Ƿ��иù����룬���������¸ù�����ļ�ʱ,
*				 ���û�У�����Ϊ���²����Ĺ����룬��ӵ�����������б�,������;����б���������
*				 ������ֱ���б��пյ�Ԫ
*Arguments  	:faultCode	:����CAN1�Ĺ�����
				:ucSource   :���������Դ��0:������ecm��0x31:������
*Returns    	:0:û�������������������б�������1:�����˹�����		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
uint8 AddToFaultCodeList(uint32 faultCode, uint8 ucSource)
{
	uint8 i,j;
	uint8 ucSameFaultcodeFlag = 0;	//�Ƿ�����ͬ�������־��1:�в���ͬ�����룬0:�޲�ͬ������
	
	if(!faultCode)
	{
		return 0;
	}
	faultCode = (faultCode & 0x00ffffff)| ((ucSource<<24)&0xff000000);//�����Դ
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode  == faultCode)
		{
			m_stuActiveFaultCodeList[i].activeTimes = TIME_FAULTCODE_ACTIVE;//���¼�ʱ
			return 0;
		}
		else
		{
			ucSameFaultcodeFlag = 1;
		}
	}	
	if(ucSameFaultcodeFlag==1)	//����������
	{
		for(j=0; j<MAX_ACTIVE_FAULTCODE_NUM; j++)
		{
			if(m_stuActiveFaultCodeList[j].faultCode == 0)
			{
				m_stuActiveFaultCodeList[j].faultCode = faultCode;
				m_stuActiveFaultCodeList[j].activeTimes = TIME_FAULTCODE_ACTIVE;
				return 1;
			}
		}
		return 0;			//�б�����
	}
	return 0;
}

/*
*********************************************************************************************************
*Function name	:FaultcodeCheck
*Description	:�˺���ÿ1������һ�Σ��Թ������б��й�������ڵ�ʱ����е���ʱ������ʱ��0ʱ����Ϊ�ù�����
*                �Ѿ���ʧ���Ըù���������
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
void FaultcodeCheck(void)
{
	uint8 i;
	uint8 flag = 0;

	if(!GetAccState())
		return;
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode)
		{
			if(m_stuActiveFaultCodeList[i].activeTimes)
				m_stuActiveFaultCodeList[i].activeTimes--;
			else
			{
				//PC_SendDebugData((uint8 *)( "D"), 1, DEBUG_ANYDATA);
				m_stuActiveFaultCodeList[i].faultCode = 0;
				flag = 1;
			}
		}
	}
	
}

#define SP_CANID1_NUM	1
uint32 SpecialCanId1[SP_CANID1_NUM] = {0x1ADC05C1};   //��Ҫ���⴦�����ϱ�

#define SP_CANID2_NUM	5
//ֻ��Ҫ���⴦������Ҫ�ϱ�
uint32 SpecialCanId2[SP_CANID2_NUM] = {0x0591,0x0592,0x0612,0x1fffffff,0x1ffffffe};

//����ֵ:3=��ֻ֡��Ҫ�ϴ�,1=��Ҫ�ϴ��������⴦��, 2=ֻ��Ҫ���⴦��,0=ֱ�Ӷ���
uint8 CanFrameFilter(uint32 id)
{
	uint8 i;	

	for(i=0; i<SP_CANID1_NUM; i++)
	{
		if(id==SpecialCanId1[i])
		{
			return 1;
		}
	}
	
	for(i=0; i<SP_CANID2_NUM; i++)
	{
		if(id==SpecialCanId2[i])
		{
			return 2;
		}
	}
	for(i=0; i<g_stuSYSParamSet.ucCanIdNum; i++)
	{
		if(id==g_stuSYSParamSet.auiCanId[i])
		{
			return 3;
		}
	}
	
	return 0;
}

/*
*********************************************************************************************************
*Function name	:DealCan1Message
*Description	:������յ�������CAN1�ı���
*Arguments  	:msg	:����CAN1�ı���
*Returns    	:none		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
void DealCan1Message(MessageDetail msg)
{
	uint8 arr[8];
	uint8 buff[16];
	uint32 id = 0;
//	uint8 i=0;
//    uint16 usVehicleModel=0;
    uint16 usPercent = 0;
//	static uint8 ucCheckCount=0;   //У���쳣����

	memcpy(arr, msg.CANRE, 8);
	id = msg.CANID;
	
    if(msg.FF==STD_FRAME)
        stu_McuFirmware.ucCANFrameFormat = STD_FRAME;  //��׼֡
	else
		stu_McuFirmware.ucCANFrameFormat = EXT_FRAME;  //��չ֡	

	KCMCU_RecVCan_ACK(id,arr);
	
    switch(id)
    {   

	    case 0x591:

            if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev)
			{
			    switch(arr[0])
			    {
			        case 0xA4: 		//�ж�MCU�Ƿ�Ӧ����������������
						if(arr[4]==0x27)
						{
                            stu_McuFirmware.ucMcuRespflag = 1;
                            stu_McuFirmware.ucLoadStep = 2;
						}
                        else if(arr[4]==0x02)
                        {
						}						
						break;
					case 0xA2:     //�ж�MCU�Ƿ�ɹ��������ݰ�
					    if(arr[1]==0x27)
					    {					        
                            stu_McuFirmware.usReadFlashSN++;
							if(stu_McuFirmware.usReadFlashSN==stu_McuFirmware.usTotalPackets)
							{
                                stu_McuFirmware.ucLoadStep = 3;  //���ݰ�������ϣ�֪ͨMCU�����̼�
    							stu_McuFirmware.ucMcuRespflag = 1;
							}
							else
							{
    							stu_McuFirmware.ucLoadStep = 3;  //���Ϳ����ؽ�������
    							stu_McuFirmware.ucMcuRespflag = 1;
							}
						}
						else if(arr[1]==0x02)
						{
						}
						else
						{
                            stu_McuFirmware.ucMcuRespflag = 0;
						}
						break;
					case 0xA1:     //MCUӦ������ؽ������� ,��ʼ��һ�����ݿ�����
                        if(stu_McuFirmware.usReadFlashSN==stu_McuFirmware.usTotalPackets)
                        {
    					    stu_McuFirmware.ucLoadStep = 4;
                        }
						else
						{
    					    stu_McuFirmware.ucLoadStep = 1;							
						}
					//�ն�ֹͣ����
						break;
					case 0x4B:     //������Ӧ���������
					    if(arr[4]==0||arr[4]==8)
					    {
							stu_McuFirmware.ucUploadResult = 1;
					    }
						else
						{
							stu_McuFirmware.ucUploadResult = 0;
						}
					    stu_McuFirmware.ucLoadStep = 7;  //MCU�������Ӧ��
						break;
    			    default:
    			  	    break;
				}	
                
				//���ʧ�� ���·���
			}
			break;
		case 0x0612:        //�������հ�����
            if(stu_McuFirmware.ucRcvPackflag&&FirmwareUpdate.ucdev)
            {
    			if(arr[0]==0x2B&&arr[1]==0x18&&arr[2]==0xA0)
    			{
                    usPercent = arr[4] + (uint16)(arr[5]<<8);  //��usPercent=10000ʱ��ʾ�հ����
    			}
			}
			break;
        case 0x1ADC05C1:    //MCU��ǰ�������
            MCU_GPSSendWorkState_check(arr);
			break;
			
	#if 0		
		case 0x18fe0bf4:    //�˴�������������ģʽ����װģʽ������롣
		case 0x18fe1bf4:
			
           if(id==0x18fe0bf4)
		       m_stuMcuCmd.ucCmdCheckFlag = 0;
		   else
		   {
		       m_stuMcuCmd.ucCmdCheckFlag = arr[3];    //У����Ϣ
		       if(m_stuMcuCmd.ucCmdCheckFlag==52)  //0x34
		       {
		           if(ucCheckCount<3)
		           {
                       ucCheckCount++;
					   break;
		           }
                   else
				   	   ucCheckCount = 0;
			   }
			   else
			   {
                   ucCheckCount = 0;
			   }
		   }
		       
           switch(arr[0])
           {
           	  case 41:      //һ������Ӧ��
	
                if((m_stuMcuCmd.LockControlFlag&BIT(2))&&(m_stuMcuCmd.LockControlFlag&BIT(3)))
                {
                     m_stuMcuCmd.LockControlFlag &=~BIT(2);
                     m_stuMcuCmd.ucRespSerFlag |= BIT(0);
                     McuSaveToMemory();    //��������
                }
			  	break;
			  case 42:      //һ������Ӧ��

                if((m_stuMcuCmd.LockControlFlag&BIT(2))&&!(m_stuMcuCmd.LockControlFlag&BIT(3)))  //�ж���ǰ�Ƿ���һ�����������
                {
                	m_stuMcuCmd.ucRespSerFlag |= BIT(2);
                    m_stuMcuCmd.LockControlFlag &=~BIT(2);
                    McuSaveToMemory();     //��������
                }
			  	break;
			  case 43:     //��������Ӧ��

                if((m_stuMcuCmd.LockControlFlag&BIT(4))&&(m_stuMcuCmd.LockControlFlag&BIT(5)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(1);
                    m_stuMcuCmd.LockControlFlag &=~BIT(4);
                    McuSaveToMemory();     //��������     
                }
			  	break;
			  case 44:      //��������Ӧ��

                if((m_stuMcuCmd.LockControlFlag&BIT(4))&&!(m_stuMcuCmd.LockControlFlag&BIT(5)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(3);
                    m_stuMcuCmd.LockControlFlag &=~BIT(4);
                    McuSaveToMemory();     //��������   
                }
			  	break;
			  default:
			  	break;
           }
		   switch(arr[1])
           {
           	  case 45:      //�򿪼��ģʽӦ��
			  	if((m_stuMcuCmd.LockControlFlag&BIT(0))&&(m_stuMcuCmd.LockControlFlag&BIT(1)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(4);
                    m_stuMcuCmd.LockControlFlag &=~BIT(0);
                    McuSaveToMemory();     //��������                                            
                }  
			  	break;
			  case 46:      //�رռ��ģʽӦ��
			  	if((m_stuMcuCmd.LockControlFlag&BIT(0))&&!(m_stuMcuCmd.LockControlFlag&BIT(1)))
                {
                    m_stuMcuCmd.ucRespSerFlag |= BIT(5);
                    m_stuMcuCmd.LockControlFlag &=~BIT(0);                       
                    McuSaveToMemory();     //��������					  
                }   
			  	break;			  
			  default:
			  	break;
           }
		   break;

	
		case 0x18fe24f3:
            usVehicleModel = arr[3]+(uint16)(arr[4]<<8);
			if(usVehicleModel>=450)
				g_stuXWFaultCodeRecv.ucVehicleType = 2; //���λ
			else
				g_stuXWFaultCodeRecv.ucVehicleType = 1; //С��λ
			break;
		case 0x18FE27F3:    //��С��
			if(g_stuXWFaultCodeRecv.ucVehicleType!=1)
				return;
			
            if(arr[0]<=1&&arr[4]<=1)
            {
                g_stuXWFaultCodeRecv.ucFaultCodeNum = 1;
				g_stuXWFaultCodeRecv.CanData[0].id = 0x18FE27F3;
				memcpy(g_stuXWFaultCodeRecv.CanData[0].aucData,arr,8);
				g_stuXWFaultCodeUp.ucFaultCodeNum = 1;
				g_stuXWFaultCodeUp.CanData[0].id = 0x18FE27F3;
				memcpy(g_stuXWFaultCodeUp.CanData[0].aucData,arr,8);
			}
			else if(arr[0]<=15&&arr[4]<=15)
			{
                if(arr[0]>arr[4])
                {
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[1];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE27F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
					if(arr[0]==arr[1])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[1];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE27F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}
					}
				}
				else
				{
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[5];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE27F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
                    if(arr[4]==arr[5])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[5];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE27F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}					
                    }
				}
			}
			break;			
		case 0x18FE25F3: //������  ����
		    if(g_stuXWFaultCodeRecv.ucVehicleType!=2)
				return;
			
            if(arr[0]<=1&&arr[4]<=1)
            {
                g_stuXWFaultCodeRecv.ucFaultCodeNum = 1;
				g_stuXWFaultCodeRecv.CanData[0].id = 0x18FE25F3;
				memcpy(g_stuXWFaultCodeRecv.CanData[0].aucData,arr,8);
				g_stuXWFaultCodeUp.ucFaultCodeNum = 1;
				g_stuXWFaultCodeUp.CanData[0].id = 0x18FE25F3;
				memcpy(g_stuXWFaultCodeUp.CanData[0].aucData,arr,8);
			}
			else if(arr[0]<=15&&arr[4]<=15)
			{
                if(arr[0]>arr[4])
                {
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[1];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE25F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
					if(arr[0]==arr[1])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[1];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE25F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}
					}
				}
				else
				{
                    g_stuXWFaultCodeRecv.ucFaultCodeNum = arr[5];
    				g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].id = 0x18FE25F3;
                    memcpy(g_stuXWFaultCodeRecv.CanData[g_stuXWFaultCodeRecv.ucFaultCodeNum-1].aucData,arr,8);
                    if(arr[4]==arr[5])
					{
					    g_stuXWFaultCodeUp.ucFaultCodeNum = arr[5];
						for(i=0;i<g_stuXWFaultCodeUp.ucFaultCodeNum;i++)
						{
            				g_stuXWFaultCodeUp.CanData[i].id = 0x18FE25F3;
						    memcpy(g_stuXWFaultCodeUp.CanData[i].aucData,g_stuXWFaultCodeRecv.CanData[i].aucData,8);
						}					
                    }
				}
			}
			break;
    	#endif
		
		case 0x1fffffff://���� ID
			if((arr[0]==0x31)&&(arr[1]==0x61)&&(arr[2]==0x32)&&(arr[3]==0x62)&&(arr[4]==0x33)&&(arr[5]==0x63)&&(arr[6]==0x34)&&(arr[7]==0x64))
	        {
				buff[0]=101;
				buff[1]=53;
				buff[2]=102;
				buff[3]=54;
				buff[4]=103;
				buff[5]=55;
				buff[6]=104;
				buff[7]=56;
				CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1fffffff, 8, buff);
	        }
			break;

		case 0x1ffffffe://���� ID
			if((arr[0]==0x31)&&(arr[1]==0x61)&&(arr[2]==0x32)&&(arr[3]==0x62)&&(arr[4]==0x33)&&(arr[5]==0x63)&&(arr[6]==0x34)&&(arr[7]==0x64))
	        {
				buff[0]=101;
				buff[1]=53;
				buff[2]=102;
				buff[3]=54;
				buff[4]=103;
				buff[5]=55;
				buff[6]=104;
				buff[7]=56;
				CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x1ffffffe, 8, buff);
			}
			break;

		default:
			break;
    }
}



/*********************************************************************************************************
*Function name	: MCU_TimerCount_Delay
*Description	:MCUģ���ʱ�������˺���100msִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2015-8-7
*Modified		:
*********************************************************************************************************/
void  MCU_TimerCount_Delay(void)
{
	static uint8 usTime1s = 10;

	if(usTime1s)
	{
		usTime1s--;
	}
	else
	{
		usTime1s = 10;
        SYS_PowerOff_Reset();
	}
    MCU_SendCan1Data();
}


/*
*********************************************************************************************************
*Function name	:McuInit
*Description	:��Mcuģ����ر�����ʼ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-6-9
*Modified		:
*********************************************************************************************************
*/
void McuInit()
{
	m_stuCommState.ucRcvCan1DataFlag = 2;
	McuHwInit();
	Mcu_CanOpen();
	//POWEROFF_485();
	m_stuMcuCmd.uiKey = 0x18FE0AF4;
}

#if 0
/*
*********************************************************************************************************
*Function name	:MCUCommStateCheck
*Description	:���MCUͨ��״̬�Ƿ�ı䣬�˺���1sִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-7-9
*Modified		:
*********************************************************************************************************
*/
void MCUCommStateCheck()
{
	static uint8 ucCan1CommStatePre;			//ǰһ�ε�canͨ��״̬
	static uint8 ucCan2CommStatePre;			//ǰһ�ε�canͨ��״̬
	static STUSYSMsgBus sysMsg;
	static uint8 can1rcvstatepre = 1;
	static uint8 can2rcvstatepre = 1;
	
	//if(!GetAccState())
		//return;
	//can1ͨ��״̬�仯���
	if(m_stuCommState.ucCan1CommState != ucCan1CommStatePre)
	{
		if(m_stuCommState.ucCan1CommState)
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN1COMM_ERR ;
		}
		else
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN1COMM_OK;
		}
		sysMsg.ucSrcDevice = 3;
		
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	ucCan1CommStatePre = m_stuCommState.ucCan1CommState;

	//can2ͨ��״̬�仯���
	if(m_stuCommState.ucCan2CommState != ucCan2CommStatePre)
	{
		if(m_stuCommState.ucCan2CommState)
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN2COMM_ERR;
		}
		else
		{
			sysMsg.ucKind = MCU_MSG_KIND_CAN2COMM_OK ;
		}
		sysMsg.ucSrcDevice = 3;
		
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	ucCan2CommStatePre = m_stuCommState.ucCan2CommState;
	

	//can1����״̬�仯���
	if((1==m_stuCommState.ucRcvCan1DataFlag)&&(2==can1rcvstatepre))//can1����ͨ�ſ�ʼ
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_START;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	else if((2==m_stuCommState.ucRcvCan1DataFlag)&&(1==can1rcvstatepre))//can1����ͨ���ж�
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_STOP;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	can1rcvstatepre = m_stuCommState.ucRcvCan1DataFlag;

	//can2����״̬�仯���
	if((1==m_stuCommState.ucRcvCan2DataFlag)&&(2==can2rcvstatepre))//can2����ͨ�ſ�ʼ
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN2_RCV_START;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	else if((2==m_stuCommState.ucRcvCan2DataFlag)&&(1==can2rcvstatepre))//can1����ͨ���ж�
	{
		sysMsg.ucSrcDevice = 3;
		sysMsg.ucKind = MCU_MSG_KIND_CAN1_RCV_STOP;
		sysMsg.ucPrior = 0;
		sysMsg.ucResv = 0;
		sysMsg.usSize = 0;
		sysMsg.pMsgPacket=NULL;
		sysMsg.ucCheck = 0;
		SYS_PutDataQ(&sysMsg);
	}
	can2rcvstatepre = m_stuCommState.ucRcvCan2DataFlag;
}
#endif

/*********************************************************************************************************
*Function name	:MCU_TimerCount_NoDelay
*Description	:MCUģ���ʱ�������˺���10msִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2019-6-9
*Modified		:
*********************************************************************************************************/
void  MCU_TimerCount_NoDelay(void)
{
	static uint8  ucTime1s;
	
	CanRcvDataTimer();
	/*
    if(stuMcuSend_Gpsdata.ucCount)
    {
        stuMcuSend_Gpsdata.ucCount--;
		if(!stuMcuSend_Gpsdata.ucCount)
		{
            MCU_CmdOldLockSend();   //���;�ָ��
		}
	}
	*/
	if(ucTime1s)
	{
	    ucTime1s--;
	}
	else
	{
		ucTime1s = 100;	
		MCUCommStateJudge();    // �˴����1������һ�εĺ���
		//MCUCommStateCheck();
		//CanMcuCommManage();
		//FaultcodeCheck();
	}	

}

/*
*********************************************************************************************************
*Function name	:GetCanCommState
*Description	:��ȡGPS�ն���MCU��ͨ��״̬
*Arguments  	:none
*Returns    	:  	00=ͨ������
                	10=ͨѶ�쳣
*Author			:hhm
*Date			:2016-8-19
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetCanCommState(uint8 channel)
{
	if(CAN_CHANNEL1==channel)
	{
		return m_stuCommState.ucCan1CommState;
	}
	else if(CAN_CHANNEL2==channel)
	{
		return m_stuCommState.ucCan2CommState;
	}
	else
		return 0;
}

/*
*********************************************************************************************************
*Function name	:GetCanBusDeviceState
*Description	:��ȡCAN�������豸��ͨ��״̬
*Arguments  	:none
*Returns    	:B0:1=��ʾ���쳣��0=��ʾ��������
*                B1:1=�������쳣��0=������������
*                B2:1=ECM�쳣��0=ECM����

*Author			:hhm
*Date			:2011-6-9
*Modified		:                 
*********************************************************************************************************
*/
uint8 GetCanBusDeviceState()
{
	return 0;
}


void Mcu_CanOpen()
{
	m_stuCommState.ucSleepState = 0;
	POWERON_CAN();
}

void Mcu_CanClose()
{
	m_stuCommState.ucSleepState = 1;
	POWEROFF_CAN();

}

//��ȡCanģ�������״̬,1=���ߣ�����δ����
uint8 Mcu_GetCanSleepState(void)
{
	return m_stuCommState.ucSleepState;
}

//����ֵ 1=���ƿ��ѹ�ﵽ������ѹ;0=���ƿ��ѹΪδ�ﵽ������ѹ
BOOL MCU_GetVoltageState(void)
{
    if((GetInput_Voltage()>130&&GetInput_Voltage()<151)||GetInput_Voltage()>265)
        return 1;
    else
		return 0;   
}


/*
*********************************************************************************************************
*Function name	:TaskMCU
*Description	:����MCUģ�������
*Arguments  	:pdata	:���ݸ����������
*Returns    	:none		
*Author			:hhm
*Date			:2011-6-13
*Modified		:                 
*********************************************************************************************************
*/
void TaskMCU(void *pdata)
{
	extern MessageDetail MessageCAN1_0;      // ����CAN0ͨ��֡����
	extern MessageDetail MessageCAN1_1;
    extern MessageDetail MessageCAN2_0;      // ����CAN0ͨ��֡����
    extern MessageDetail MessageCAN2_1;
	
	pdata = pdata;
	
//	McuReadFromMemory();
    stu_McuFirmware.ucCANFrameFormat = STD_FRAME;

	while(1)
	{
		if(CAN1_DATA_OK==WaitForCAN1Message(2*OS_TICKS_PER_SEC))
		{
			if(MessageCAN1_0.CANID!=0)
			{
				DealCan1Message(MessageCAN1_0);
				MessageCAN1_0.CANID = 0; 
			}
			if(MessageCAN1_1.CANID!=0)
			{
				DealCan1Message(MessageCAN1_1);
				MessageCAN1_1.CANID = 0;
			}
			if(MessageCAN2_0.CANID!=0)
			{
				DealCan1Message(MessageCAN2_0);
				MessageCAN2_0.CANID = 0; 
			}
			if(MessageCAN2_1.CANID!=0)
			{
				DealCan1Message(MessageCAN2_1);
				MessageCAN2_1.CANID = 0;
			}				
		}
	}	
}



#ifdef  __cplusplus
}
#endif

#endif
