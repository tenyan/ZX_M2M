//�ļ�����Mcu.h

#ifndef _MCU_CAN_
#define _MCU_CAN_

#ifdef  __cplusplus
extern "C" {
#endif

#include "config.h"
#include "McuHW.h"

/************************�ڲ�����*********************/
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


faultCode m_stuActiveFaultCodeList[MAX_ACTIVE_FAULTCODE_NUM];//��ǰ��������й������б����20����ȥ����Ĺ�������0���
static uint8 f_ucNewFaultCodeFlag;		//�������µ�DTC��־,0=��,1=�� 

static uint8 eng_multiFaultcodeBuff[MAX_ACTIVE_FAULTCODE_NUM_ENG*4];//���������������ͬʱ����ʱ����������֡����Ĺ�������ϵ��˻���
static uint8 eng_multiFaultcodeIndex;   //���������������ͬʱ����ʱ��������֡����Ĺ���������� 
static uint8 eng_totalFaultcodeNum;	    //��������������뼤��ʱ���ܵĹ���������
static uint8 eng_faultcodeFrameNum;	    //������ͬʱ��������뼤��ʱ����֡�������֡��

static uint8 DM1FlagValidFlag_ECU = 0;
uint8 eng_LampStatus= 0;
uint8 m_ucFaultCodeNum = 0;



/************************ȫ�ֱ���*********************/
STU_STM32_MCU stu_STM32_MCU;
//stuXWFaultCode g_stuXWFaultCodeUp,g_stuXWFaultCodeRecv; 
STU_CanFrame CanData[MAX_CAN_FRAME_NUM];//CAN���ݻ��� 
STU_McuCmd m_stuMcuCmd = {
//	.usSpeedLimit= 3500,
    .ucCmdCheckFlag = 0,
	
};
STU_MultiFrame g_stuMultiFrame;
/************************��������*********************/
extern STUSYSParamSet g_stuSYSParamSet;
extern struct STU_Sysstruct STU_Systemstate;
extern STUMCUFirmware stu_McuFirmware;
extern stuFirmwareUpdate FirmwareUpdate;
extern STU_SSData SSData;
extern STUSleep stuSleep; 
extern STUSystem g_stuSystem;
extern STU_Orient m_stuOrient;
extern uint16 g_usSTMCU_Ver;
/****************************************************/


#define SP_CANID1_NUM	2	
//��Ҫ���⴦�����ϱ�*/
uint32 SpecialCanId1[SP_CANID1_NUM] = {0x1ADC05C1,0x1ADC02C1};

#define SP_CANID2_NUM	11
//ֻ��Ҫ���⴦������Ҫ�ϱ�
uint32 SpecialCanId2[SP_CANID2_NUM] = {0x1ADC06C1,0x1ADC08C1,0x1ADC09C1,0x18FECA00,0x18ECFF00,0x18EBFF00,0x1CECFF00,0x1CEBFF00,0x18FECB00,0x1fffffff,0x1ffffffe};


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

	WriteToFlash(FLASH_PAGEADDR_MCU1,20,aucTemp);
	OSTimeDly(OS_TICKS_PER_SEC/100);
	WriteToFlash(FLASH_PAGEADDR_MCU2,20, aucTemp);
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
	
	ReadFromFlash(FLASH_PAGEADDR_MCU1,20, aucTemp1);
	sum1 = SumCalc(aucTemp1, 19);
	
    ReadFromFlash(FLASH_PAGEADDR_MCU2,20, aucTemp2);
	sum2 = SumCalc(aucTemp2,19);

	if((aucTemp1[0]==0xAA) && (sum1==aucTemp1[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp1[0],20);
		if((aucTemp2[0]!=0xAA) || (sum2!=aucTemp2[19]))
		{
			PC_SendDebugData((uint8 *)("MCUSAVE 2 ERR"), 13, DEBUG_ANYDATA);
			WriteToFlash(FLASH_PAGEADDR_MCU2,20, aucTemp1);
		}
	}
	else if((aucTemp2[0]==0xAA) && (sum2==aucTemp2[19]))
	{
		flag = 1;
		memcpy(&m_aucDataSave[0],&aucTemp2[0],20);
		PC_SendDebugData((uint8 *)("MCUSAVE 1 ERR"), 13, DEBUG_ANYDATA);
		WriteToFlash(FLASH_PAGEADDR_MCU1,20, aucTemp2);
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
	uint8 ucOpenModeFlag = 0;
    static uint8 ucLockcntflag = 0;
	uint32 uiID = 0x1ADB02B1;

	if(ucLockcntflag==0)
	{
        if(m_stuMcuCmd.LockControlFlag&BIT(0))
        {
            if(m_stuMcuCmd.LockControlFlag&BIT(1)) 
            {
                Sendbuff[0] = 35;    //�������ģʽ
                ucOpenModeFlag = 1;
            }
            else
                Sendbuff[0] = 36;    //�رռ��ģʽ        
        }
		else
			ucLockcntflag = 1;
	}

    if(ucLockcntflag==1)
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
		    ucLockcntflag = 2;
    }

	if(ucLockcntflag==2)
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
		//else
       //	    ucLockcntflag=0;
	}

	if(m_stuMcuCmd.LockControlFlag&0x15 )
	{	
        if(ucLockcntflag>=2)
			ucLockcntflag = 0;
		else
			ucLockcntflag++;	
		
		if(ucOpenModeFlag==1)
		{
            Sendbuff[1] = (uint8)(m_stuMcuCmd.uiKey&0xFF);
    	    Sendbuff[2] = (uint8)(m_stuMcuCmd.uiKey>>8);
    	    Sendbuff[3] = (uint8)(m_stuMcuCmd.uiKey>>16);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiKey>>24);
		}
		else
		{
            Sendbuff[1] = (uint8)(m_stuMcuCmd.uiFs&0xFF);
    	    Sendbuff[2] = (uint8)(m_stuMcuCmd.uiFs>>8);
    	    Sendbuff[3] = (uint8)(m_stuMcuCmd.uiFs>>16);
    	    Sendbuff[4] = (uint8)(m_stuMcuCmd.uiFs>>24);
		}
		
		if(Sendbuff[0]==0)
			return 0;
		stu_STM32_MCU.ucControlFlag =1;
		stu_STM32_MCU.aControlData[0] = CAN_CHANNEL1;
		stu_STM32_MCU.aControlData[1] = EXT_FRAME;
		stu_STM32_MCU.aControlData[2] = (uint8)(uiID>>24);
		stu_STM32_MCU.aControlData[3] = (uint8)(uiID>>16);
		stu_STM32_MCU.aControlData[4] = (uint8)(uiID>>8);
		stu_STM32_MCU.aControlData[5] = (uint8)(uiID&0xFF);
		stu_STM32_MCU.aControlData[6] = 8;
		memcpy(&stu_STM32_MCU.aControlData[7],Sendbuff,8);
		
		return 1;
	}
	ucLockcntflag = 0;
	return 0;
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
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe08f4, 8, arr);
}
#endif

//GPS�ն����� �յ�MCU���������У�鷢��
void MCU_GPSSendWorkState_check(uint8 *ptr)
{

	uint8 buff[8];
	static uint8 heart_flag=0;
    uint32 uis;
    //uint32 uiKey = 0x18FE0AF4;
	//uint32 uiFs = 0;
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
  	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ADB01B1, 8, buff);
}


//GPS��MCU����λ��״̬ ���θ߶ȵ�
#if 0
void MCU_GPSSendPostionState(uint8 *arr)
{
//	uint8 arr[8] = {0};
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
 // 	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff6, 8, arr);
 //   memcpy((uint8*)&stu_STM32_MCU.stuSendGpsdata.aGpsstate,arr,8);
 
}
#endif

void MCU_GPSSendPostionState(uint8 *arr)
{
//	uint8 arr[8] = {0};
	uint8 ucTemp =0;
	uint16 usHeight = 0;
	
    arr[0] = 0;

	if(GPS_GetState()& BIT(1))
		arr[0] &= ~BIT(0);
	else
		arr[0] |= BIT(0);       //δ��λ=1
	
    if(1==GPS_GetLatitudeHemisphere())
     	arr[0] |= BIT(1);     //��γ
	else
		arr[0] &= ~BIT(1);    //��γ
    if(1==GPS_GetLongitudeHemisphere())
     	arr[0] |= BIT(2);     //����
	else
		arr[0] &= ~BIT(2);    //����
    if(GPS_GetHeight())
    {
     	arr[0] |= BIT(3);
        usHeight = GPS_GetHeight();
	}
	else
	{
		arr[0] &= ~BIT(3);
		usHeight = 0-GPS_GetHeight();
	}	

	if(GPS_GetSatellitenums()>31)
		arr[1] = 31;
	else
	    arr[1] = GPS_GetSatellitenums();	
	arr[2] = usHeight & 0xff;
	arr[3] = (usHeight>>8) & 0xff;
    arr[4] = 0;
	arr[5] = 0;
	arr[6] = 0;
	arr[7] = 0;
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
}
//GPS��MCU����λ����Ϣ ��γ��
void MCU_GPSSendPostionInformation(uint8 *arr)
{
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
 // 	CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x18fe0ff7, 8, arr);
 //   memcpy((uint8*)&stu_STM32_MCU.stuSendGpsdata.aGpsData,arr,8);
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
   // static uint8 SendFlag=0;           //����GPS״̬���������������ѡ���־
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
			if(1==MCU_CmdLockSend())      //�����ݷ�
			{
    			//if(0==m_stuMcuCmd.usSendGPSDataTimer)
    			//	m_stuMcuCmd.usSendGPSDataTimer = 1;
			}
   	 	}
		
		#if 0
		if(!m_stuMcuCmd.usSendGPSDataTimer)
		{
	        if(0==SendFlag)
			{
	        	MCU_GPSSendWorkState();
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
		#endif
    }
}

//���CAN֡��ַ����
void MCU_ClearCanData(void)
{
    uint8 i = 0;
	//����ϱ�CAN���ݻ���
	
	CAN_ClearCanDataUpbuff();
	for(i=0; i<MAX_CAN_FRAME_NUM; i++)
	{
	    CanData[i].id = 0;
	}
	g_stuMultiFrame.ucControlCanDataNum = 0;
}
/*********************************************************************************************************
*Function name	:MCUCommStateJudge
*Description	:MCUͨ��״̬�жϣ��˺���1sִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2020-7-9
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
      //      PC_SendDebugData((uint8 *)("CAN OFF"), 7, DEBUG_ANYDATA);
   //         POWEROFF_CAN();      //�ر�CANģ���Դ
            ucPWRCloseTimer = 5;
		}
		if(ucPWRCloseTimer)
		{
		    if(!(--ucPWRCloseTimer))
            {
             //   McuInit();      //�ص��ʱ5�� ���¶�CANģ���ϵ粢���г�ʼ��
            //    PC_SendDebugData((uint8 *)("CAN REST"), 8, DEBUG_ANYDATA);
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
#if 0
void CanRcvDataTimer()
{
	if(++m_stuCommState.usCan1RcvErrTime > MAX_CAN1_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan1DataFlag = 2;
	//	MCU_ClearCanData();     //CANͨѶ�жϺ����ԭ�б���CAN����
	}
	if(++m_stuCommState.usCan2RcvErrTime > MAX_CAN2_RCV_ERR_TIME)
	{
		m_stuCommState.ucRcvCan2DataFlag = 2;
	}

	if(m_stuCommState.ucRcvCan1DataFlag==2&&m_stuCommState.ucRcvCan2DataFlag==2)
	{
		MCU_ClearCanData();     //CANͨѶ�жϺ����ԭ�б���CAN����
	}	
}
#endif

void MCU_CanRcvDataTimer(void)
{
    if(m_stuCommState.ucRcvCan1DataFlag==2)
    {
        g_stuMultiFrame.ucControlCanDataNum = 0;
		g_stuMultiFrame.ucControllerHoursLen = 0;
		g_stuMultiFrame.ucControllerVerLen = 0;
	}
	if(m_stuCommState.ucRcvCan2DataFlag==2)
	{
        g_stuMultiFrame.ucCumminsEngineVerLen = 0;
		g_stuMultiFrame.ucISUZUEngineInfoLen = 0;
		g_stuMultiFrame.ucISUZUEngineVerLen = 0;
	}

	if(m_stuCommState.ucRcvCan1DataFlag==2&&m_stuCommState.ucRcvCan2DataFlag==2)
	{
		MCU_ClearCanData();     //CANͨѶ�жϺ����ԭ�б���CAN����
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
#if 0
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
#endif
uint8 GetMcuFaultCode(uint8 *faultCodeData)
{
	uint8 i;
	uint8 faultCodeNum=0;
	
	
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(m_stuActiveFaultCodeList[i].faultCode)
		{
			*faultCodeData     = m_stuActiveFaultCodeList[i].ucSrc;
			*(faultCodeData+1) = m_stuActiveFaultCodeList[i].faultCode & 0xff;
			*(faultCodeData+2) = (m_stuActiveFaultCodeList[i].faultCode >> 8) & 0xff;
			*(faultCodeData+3) = (m_stuActiveFaultCodeList[i].faultCode >> 16) & 0xff;
			*(faultCodeData+4) = (m_stuActiveFaultCodeList[i].faultCode >> 24) & 0xff;
		    faultCodeData += 5;
			faultCodeNum++;
		}
	}
	if(0==faultCodeNum)
		m_ucFaultCodeNum = 0;
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
				:ucSource   :���������Դ��0:������ecm
*Returns    	:0:û�������������������б�������1:�����˹�����		
*Author			:hhm
*Date			:2020-6-13
*Modified		:                 
*********************************************************************************************************
*/
#if 0
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
#endif
uint8 AddToFaultCodeList(uint32 faultCode, uint8 ucSource)
{
	uint8 i,j;
	uint8 ucSameFaultcodeFlag = 0;	//�Ƿ�����ͬ�������־��1:�в���ͬ�����룬0:�޲�ͬ������
	
	if(!faultCode)
	{
		return 0;
	}
		
	for(i=0; i<MAX_ACTIVE_FAULTCODE_NUM; i++)
	{
		if(((m_stuActiveFaultCodeList[i].faultCode & 0x00ffffff)==(faultCode & 0x00ffffff)) && (m_stuActiveFaultCodeList[i].ucSrc==ucSource))
		{
			m_stuActiveFaultCodeList[i].activeTimes = TIME_FAULTCODE_ACTIVE;//���¼�ʱ
			m_stuActiveFaultCodeList[i].faultCode=faultCode;//����OC CM
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
				m_stuActiveFaultCodeList[j].ucSrc = ucSource;
				m_ucFaultCodeNum++;
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

	//if(!GetAccState())
		//return;
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
				m_ucFaultCodeNum--;
				flag = 1;
			}
		}
	}
}

#if 0
//����ֵ:3=��ֻ֡��Ҫ�ϴ�,1=��Ҫ�ϴ��������⴦��, 2=ֻ��Ҫ���⴦��,0=ֱ�Ӷ���
uint8 CanFrameFilter(uint32 id)
{
	uint8 i;
	
	for(i=0; i<g_stuSYSParamSet.ucCanIdNum; i++)
	{
		if(id==g_stuSYSParamSet.auiCanId[i])
		{
			return 3;
		}
	}
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
	return 0;
}
#endif

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
*Author			:lxf
*Date			:2020-11-09
*Modified		:                 
*********************************************************************************************************
*/
void DealCan1Message(MessageDetail msg)
{
	uint8 arr[8];
	uint8 buff[16];
	uint32 id = 0;
	uint8 i=0;
  //  uint16 usVehicleModel=0;
  //  uint16 usPercent = 0;
	static uint8 ucCheckCount=0;   //У���쳣����

    static uint8 ucCount = 0;
    static uint8 ucCount1 = 0;
	
	memcpy(arr, msg.CANRE, 8);
	id = msg.CANID;
	
    if(msg.FF==STD_FRAME)
        stu_McuFirmware.ucCANFrameFormat = STD_FRAME;  //��׼֡
	else
		stu_McuFirmware.ucCANFrameFormat = EXT_FRAME;  //��չ֡	
		
    switch(id)
    {   

		case 0x1ADC06C1: 
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

        case Control_CANFrame_ID:    //���������ϴ���
            MCU_GetControllerFaultcodedata(id,arr);
            break;
			
        case 0x1ADC02C1:    //������Сʱ��
			if(arr[0]>0&&arr[0]<5)
			{
                g_stuMultiFrame.aControllerHours[(arr[0]-1)*3] = arr[1];
                g_stuMultiFrame.aControllerHours[(arr[0]-1)*3+1] = arr[2];
                g_stuMultiFrame.aControllerHours[(arr[0]-1)*3+2] = arr[3];
			}
			if(arr[4]>0&&arr[4]<10)
			{
                g_stuMultiFrame.aControllerHours[(arr[4]+3)*3] = arr[5];
                g_stuMultiFrame.aControllerHours[(arr[4]+3)*3+1] = arr[6];
                g_stuMultiFrame.aControllerHours[(arr[4]+3)*3+2] = arr[7];
				ucCount++;
				if(ucCount>=9)
				{
				    ucCount = 0; 
                    g_stuMultiFrame.ucControllerHoursLen = 39;
				}
			}				
			break;
		case 0x1ADC09C1:    //�������汾
		    if(arr[0]==2)
		    {
                if(arr[1]==1)
                {
                    memcpy(&g_stuMultiFrame.aControllerVer[0],&arr[2],6);
					ucCount1 = 1;
				}
				else if(arr[1]==2)
				{
                    memcpy(&g_stuMultiFrame.aControllerVer[6],&arr[2],6);
					if(ucCount1==1)
					{
                        ucCount1 = 0;
						g_stuMultiFrame.ucControllerVerLen = 12;
					}
				}					
			}
			break;

		case 0x1ffffffc://���� ID
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
				CanWrite(CAN_CHANNEL1, EXT_FRAME, 0x1ffffffc, 8, buff);
	        }
			break;

		case 0x1ffffffd://���� ID
			if((arr[0]==49)&&(arr[1]==97)&&(arr[2]==50)&&(arr[3]==98)&&(arr[4]==51)&&(arr[5]==99)&&(arr[6]==52)&&(arr[7]==100));
	        {
				buff[0]=101;
				buff[1]=53;
				buff[2]=102;
				buff[3]=54;
				buff[4]=103;
				buff[5]=55;
				buff[6]=104;
				buff[7]=56;
				CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x1ffffffd, 8, buff);
			}
			break;
		default:
			break;
    }
    MCU_GetEngineFaultcodedata(id,arr);	
}

void MCU_GetControllerFaultcodedata(uint32 uiID,uint8 *arr)
{
  //  uint8 i =0;
	uint8 ucPackSN = 0;
//	uint8 ucPackTotal = 0;
//	uint8 m =0,n=0;
//	uint8 ucCanPackNum = 0;
	static uint8 ucPackNum = 0;    //������

    if(arr[0]==0)  //���ϰ���Ϊ0
    {
        g_stuMultiFrame.ucControlCanDataNum = 0;
        memcpy(&g_stuMultiFrame.aControlData[0],arr,8);		
	}
	else if(arr[0]<=Control_Faultcode_MAX_NUM)
	{
	    
	    ucPackSN = arr[1];   //�����
	    ucPackNum++;
		memcpy(&g_stuMultiFrame.aControlTempData[(ucPackSN-1)*8],arr,8);		
    
		if(arr[0]==arr[1])
		{
		    if(arr[0]==ucPackNum)
		    {
    		    g_stuMultiFrame.ucControlCanDataNum = arr[0];
    			memcpy(&g_stuMultiFrame.aControlData[0],&g_stuMultiFrame.aControlTempData[0],g_stuMultiFrame.ucControlCanDataNum*8);
		    }
			else
			{
                g_stuMultiFrame.ucControlCanDataNum = 0;
			}
			ucPackNum = 0;
		}		
	}			
}

/******************************************************************************
** ��������: MCU_GetEngineFaultcodedata
** ��������: ��ȡ������J1939������
** 
** ��    ��: 
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: 
** ��    ��: 
**-----------------------------------------------------------------------------
*******************************************************************************/
void MCU_GetEngineFaultcodedata(uint32 uiID,uint8 *pdata)
{

   uint8 j = 0;
   uint8 i = 0;
   uint32 uiTemp;
   uint8 ucTemp;
   uint8 arr[8];
   static uint8 s_ucSq_Ecm = 0;     //ECM���������ʱ������֡�����ۼ���
   uint8 reportFaultcodeFlag=0;		//1:��Ҫ�ϱ������룬0:����Ҫ�ϱ�������

   memcpy(arr, pdata, 8);

    switch(uiID)
    { 

        case 0x18FECA00:   //����������������
			uiTemp = arr[2] + (arr[3]<<8) + (arr[4]<<16) + (arr[5]<<24);
			ucTemp = AddToFaultCodeList(uiTemp,CAN_ECM_ADDR);
			eng_LampStatus=arr[0];
			if(0!=ucTemp)
				f_ucNewFaultCodeFlag = 1;		
			break;
		case 0x18ECFF00:
			if(arr[5]==0xC5 && arr[6]==0xFD&&arr[7]==0) //����˹����汾��
			{
			    g_stuMultiFrame.ucFrameTotalNum = arr[3];
				g_stuMultiFrame.ucDataLen = arr[1];
				g_stuMultiFrame.ucPGNRecvFlag = 1;
				
			    g_stuMultiFrame.ucDataIndex = 0;
				g_stuMultiFrame.ucFrameSN = 1;			 
			}
			if(arr[5]==0xCA && arr[6]==0xFE&&arr[7]==0) //����˹���������ϴ���
			{
				eng_totalFaultcodeNum = (arr[1]-2)/4;   //���ϴ�������
				eng_faultcodeFrameNum = arr[3];
				g_stuMultiFrame.ucPGNRecvFlag = 3;
			}			
			break;
		case 0x18EBFF00:
			if(g_stuMultiFrame.ucPGNRecvFlag==1)  //����������汾
			{
                if(arr[0]<=g_stuMultiFrame.ucFrameTotalNum&&arr[0]==g_stuMultiFrame.ucFrameSN&&
					g_stuMultiFrame.ucDataIndex<MultiFrame_TempLEN-7)
                {
                    memcpy(&g_stuMultiFrame.aTempData[g_stuMultiFrame.ucDataIndex],&arr[1],7);
					g_stuMultiFrame.ucDataIndex += 7;
					g_stuMultiFrame.ucFrameSN++;
				}

				if(arr[0]==g_stuMultiFrame.ucFrameTotalNum&&arr[0]==(g_stuMultiFrame.ucFrameSN-1))
				{
          
                 //   if((g_stuMultiFrame.ucDataLen==g_stuMultiFrame.ucDataIndex+1))
                //    {
    				    memcpy(g_stuMultiFrame.aCumminsEngineVer,g_stuMultiFrame.aTempData,10);
    				    g_stuMultiFrame.ucCumminsEngineVerLen = 10;
				//	}
					g_stuMultiFrame.ucPGNRecvFlag = 0;
					g_stuMultiFrame.ucFrameSN = 0;
					g_stuMultiFrame.ucDataIndex = 0;
				}				
			}  
			else if(g_stuMultiFrame.ucPGNRecvFlag==3)  //���������ϴ���
			{
    			if(arr[0] == 1)
    			{
    			    memset(eng_multiFaultcodeBuff, 0xFF, MAX_ACTIVE_FAULTCODE_NUM_ENG*4);
    				memcpy(&eng_multiFaultcodeBuff[0],&arr[3],5);
    				eng_multiFaultcodeIndex = 5;
    				s_ucSq_Ecm = 2;
    				eng_LampStatus=arr[1];
    			}
    			else
    			{
    				if((eng_multiFaultcodeIndex<(MAX_ACTIVE_FAULTCODE_NUM_ENG*4-7)) && (s_ucSq_Ecm==arr[0]))
    				{
    					memcpy(&eng_multiFaultcodeBuff[eng_multiFaultcodeIndex],&arr[1],7);
    					eng_multiFaultcodeIndex += 7;
    					s_ucSq_Ecm++;
    				}
    			}
    			if(arr[0] == eng_faultcodeFrameNum)//���ͽ���
    			{
    				eng_multiFaultcodeIndex = 0;
    				for(i=0; i<eng_totalFaultcodeNum; i++)
    				{
    					uiTemp = eng_multiFaultcodeBuff[j]+(eng_multiFaultcodeBuff[j+1]<<8)
    						     +(eng_multiFaultcodeBuff[j+2]<<16) +(eng_multiFaultcodeBuff[j+3]<<24);
    					j += 4;
    					if((uiTemp==0xFFFFFFFF) || (uiTemp==0))
    						continue;
    					else
    					{
    						ucTemp = AddToFaultCodeList(uiTemp,CAN_ECM_ADDR);
    						if(ucTemp)
    							reportFaultcodeFlag = 1;
    					}
    				}
    				if(1==reportFaultcodeFlag)
    					f_ucNewFaultCodeFlag = 1;
    				s_ucSq_Ecm = 0;
    				eng_faultcodeFrameNum = 0;
    				g_stuMultiFrame.ucPGNRecvFlag = 0;
    			}
			}
			break;
		case 0x1CECFF00:
			if(arr[5]==0xCA && arr[6]==0xFE&&arr[7]==0)  //������������
			{
				eng_totalFaultcodeNum = (arr[1]-2)/4;   //���ϴ�������
				eng_faultcodeFrameNum = arr[3];
				g_stuMultiFrame.ucPGNRecvFlag = 3;
			}	
			if(arr[5]==0xDA && arr[6]==0xFE&&arr[7]==0) //��ʮ������汾��
			{
			    g_stuMultiFrame.ucFrameTotalNum = arr[3];
				g_stuMultiFrame.ucDataLen = arr[1];
				g_stuMultiFrame.ucPGNRecvFlag = 1;
			    g_stuMultiFrame.ucDataIndex = 0;
				g_stuMultiFrame.ucFrameSN = 1;
			}
			if(arr[5]==0xEB && arr[6]==0xFE&&arr[7]==0) //ISUZU��������Ϣ
			{
			    g_stuMultiFrame.ucFrameTotalNum = arr[3];
				g_stuMultiFrame.ucDataLen = arr[1];
				g_stuMultiFrame.ucPGNRecvFlag = 2;
			    g_stuMultiFrame.ucDataIndex = 0;
				g_stuMultiFrame.ucFrameSN = 1;
			}			
			break;
		case 0x1CEBFF00:
			if(g_stuMultiFrame.ucPGNRecvFlag==1)
			{
                if(arr[0]<=g_stuMultiFrame.ucFrameTotalNum&&arr[0]==g_stuMultiFrame.ucFrameSN&&
					g_stuMultiFrame.ucDataIndex<MultiFrame_TempLEN-7)
                {
                    memcpy(&g_stuMultiFrame.aTempData[g_stuMultiFrame.ucDataIndex],&arr[1],7);
					g_stuMultiFrame.ucDataIndex += 7;
					g_stuMultiFrame.ucFrameSN++;
				}

				if(arr[0]==g_stuMultiFrame.ucFrameTotalNum&&arr[0]==(g_stuMultiFrame.ucFrameSN-1))
				{
          
                    if(g_stuMultiFrame.ucDataLen<=30)
                    {
    				    memcpy(g_stuMultiFrame.aISUZUEngineVer,g_stuMultiFrame.aTempData,g_stuMultiFrame.ucDataLen);
    				    g_stuMultiFrame.ucISUZUEngineVerLen = g_stuMultiFrame.ucDataLen;
					}
					g_stuMultiFrame.ucPGNRecvFlag = 0;
					g_stuMultiFrame.ucFrameSN = 0;
					g_stuMultiFrame.ucDataIndex = 0;
				}		  
			}	
			else if(g_stuMultiFrame.ucPGNRecvFlag==2)
			{
                if(arr[0]<=g_stuMultiFrame.ucFrameTotalNum&&arr[0]==g_stuMultiFrame.ucFrameSN&&
					g_stuMultiFrame.ucDataIndex<MultiFrame_TempLEN-7)
                {
                    memcpy(&g_stuMultiFrame.aTempData[g_stuMultiFrame.ucDataIndex],&arr[1],7);
					g_stuMultiFrame.ucDataIndex += 7;
					g_stuMultiFrame.ucFrameSN++;
				}

				if(arr[0]==g_stuMultiFrame.ucFrameTotalNum&&arr[0]==(g_stuMultiFrame.ucFrameSN-1))
				{
          
                    if(g_stuMultiFrame.ucDataLen<=50)
                    {
    				    memcpy(g_stuMultiFrame.aISUZUEngineInfo,g_stuMultiFrame.aTempData,g_stuMultiFrame.ucDataLen);
    				    g_stuMultiFrame.ucISUZUEngineInfoLen = g_stuMultiFrame.ucDataLen;
					}
				    g_stuMultiFrame.ucPGNRecvFlag = 0;
					g_stuMultiFrame.ucFrameSN = 0;
					g_stuMultiFrame.ucDataIndex = 0;
				}	
			}			
			else if(g_stuMultiFrame.ucPGNRecvFlag==3)  //���������ϴ���
			{
    			if(arr[0] == 1)
    			{
    			    memset(eng_multiFaultcodeBuff, 0xFF, MAX_ACTIVE_FAULTCODE_NUM_ENG*4);
    				memcpy(&eng_multiFaultcodeBuff[0],&arr[3],5);
    				eng_multiFaultcodeIndex = 5;
    				s_ucSq_Ecm = 2;
    				eng_LampStatus=arr[1];
    			}
    			else
    			{
    				if((eng_multiFaultcodeIndex<(MAX_ACTIVE_FAULTCODE_NUM_ENG*4-7)) && (s_ucSq_Ecm==arr[0]))
    				{
    					memcpy(&eng_multiFaultcodeBuff[eng_multiFaultcodeIndex],&arr[1],7);
    					eng_multiFaultcodeIndex += 7;
    					s_ucSq_Ecm++;
    				}
    			}
    			if(arr[0] == eng_faultcodeFrameNum)//���ͽ���
    			{
    				eng_multiFaultcodeIndex = 0;
    				for(i=0; i<eng_totalFaultcodeNum; i++)
    				{
    					uiTemp = eng_multiFaultcodeBuff[j]+(eng_multiFaultcodeBuff[j+1]<<8)
    						     +(eng_multiFaultcodeBuff[j+2]<<16) +(eng_multiFaultcodeBuff[j+3]<<24);
    					j += 4;
    					if((uiTemp==0xFFFFFFFF) || (uiTemp==0))
    						continue;
    					else
    					{
    						ucTemp = AddToFaultCodeList(uiTemp,CAN_ECM_ADDR);
    						if(ucTemp)
    							reportFaultcodeFlag = 1;
    					}
    				}
    				if(1==reportFaultcodeFlag)
    					f_ucNewFaultCodeFlag = 1;
    				s_ucSq_Ecm = 0;
    				eng_faultcodeFrameNum = 0;
    				g_stuMultiFrame.ucPGNRecvFlag = 0;
    			}
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
      //  SYS_PowerOff_Reset();
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
//	CAN1_LED_ON();
//	CAN2_LED_OFF();
	Mcu_CanOpen();
	m_stuMcuCmd.uiKey = 0x18FE0AF4;
}

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
		//SYS_PutDataQ(&sysMsg);
        SSData.usTimer = 0;
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
		//SYS_PutDataQ(&sysMsg);
        SSData.usTimer = 0;
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
		//SYS_PutDataQ(&sysMsg);
        SSData.usTimer = 0;
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
		//SYS_PutDataQ(&sysMsg);
        SSData.usTimer = 0;
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
	//	SYS_PutDataQ(&sysMsg);
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
	//	SYS_PutDataQ(&sysMsg);
	}
	can2rcvstatepre = m_stuCommState.ucRcvCan2DataFlag;
}


/*********************************************************************************************************
*Function name	:MCU_TimerCount_NoDelay
*Description	:MCUģ���ʱ�������˺���10msִ��һ��
*Arguments  	:none
*Returns    	:none
*Author			:hhm
*Date			:2011-7-9
*Modified		:
*********************************************************************************************************/
void  MCU_TimerCount_NoDelay(void)
{
	static uint8  ucTime1s;
	
	//CanRcvDataTimer();
	
	if(ucTime1s)
	{
	    ucTime1s--;
	}
	else
	{
		ucTime1s = 100;	
		MCUCommStateJudge();     // �˴����1������һ�εĺ���
		MCUCommStateCheck();
		FaultcodeCheck();
		MCU_CanRcvDataTimer();
	}	
    /*	
	if(usTime15s)
	{
	    usTime15s--;
	}
	else
	{
		usTime15s = 1500;
	}
	*/
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
//	POWERON_CAN();
}

void Mcu_CanClose()
{
	m_stuCommState.ucSleepState = 1;
//	POWEROFF_CAN();

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



//��Э�������������������Ӧ��
void MCU_SendReqCmdData(uint8 ucCommand, uint8 ucSN, uint8 ucResult)
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
	
	WriteMCU_CAN_UartData(SendBuff, 8);
}

//ucflag 1=��͸������ 0=��͸������
void MCU_SendCorePlateData(uint8 ucCommand)
{
    uint8 SendBuff[200] = {0};
	uint16 usSendDataLen = 0;
	uint16 usTlvlen1 = 0;
	uint16 usTlvlen2 = 0;
    uint16 usTlvlen3 = 0;
    uint16 usTlvlen4 = 0;
    uint16 ustemplen = 0;
	static uint8 ucSN = 0;
    STU_Date date;
	
	SendBuff[0] = 0x7E;
	SendBuff[3] = ucCommand;         //Command		
	if(ucSN<255)
		ucSN++;
	else
		ucSN = 0;
	SendBuff[4] = ucSN;
	if(stu_STM32_MCU.ucControlFlag==0)
        SendBuff[5]=3;   //TLV����
    else 
        SendBuff[5]=4;   //TLV����

	SendBuff[6] = 0x07;  //ָʾ��״̬
    usTlvlen1 = 5;
    SendBuff[7] = 0;
    SendBuff[8] = usTlvlen1;
	if(g_stuSystem.ucOnline)
		SendBuff[9] = 1;     //����
	else
		SendBuff[9] = 0;     //δ����
		
	SendBuff[10] = GPS_GetOrientState();    //��λ

	SendBuff[11] = 0;
	SendBuff[12] = 0;
	SendBuff[13] = 0;
	if(FirmwareUpdate.ucStep>0)
	    SendBuff[13] |= BIT(1);
	    
    ustemplen = 9+usTlvlen1;
	SendBuff[ustemplen] = 0x03;  //GPSʱ��  ��Ҫת��Ϊ����ʱ��
    usTlvlen2 = 6;
	SendBuff[ustemplen+1] = 0;
	SendBuff[ustemplen+2] = usTlvlen2;

	//GPS_GetUtcTime();
	if(GPS_GetOrientState())
	    UtcToBjTime(&m_stuOrient.stuDate, &date);
	else
	{
		date.ucMon = 0;
	}
	memcpy(&SendBuff[ustemplen+3],(uint8*)&date,6);
	ustemplen = ustemplen+9;
	SendBuff[ustemplen] = 0x08;
	usTlvlen3 = 16;
	SendBuff[ustemplen+1] = 0;
	SendBuff[ustemplen+2] = usTlvlen3;
	MCU_GPSSendPostionState(&SendBuff[ustemplen+3]);
    MCU_GPSSendPostionInformation(&SendBuff[ustemplen+11]);
	ustemplen += 19;
	if(stu_STM32_MCU.ucControlFlag==1)
	{
	   stu_STM32_MCU.ucControlFlag = 0;
       SendBuff[ustemplen] = 0x06;
	   usTlvlen4 = 15;
	   SendBuff[ustemplen+1] = 0;
	   SendBuff[ustemplen+2] = usTlvlen4;
	   memcpy(&SendBuff[ustemplen+3],(uint8*)&stu_STM32_MCU.aControlData,usTlvlen4);
	   ustemplen += 3+usTlvlen4;
	}
	usSendDataLen = ustemplen -2;   //����=Э������ĳ���(������ŵ�У����������ݳ��ȣ���������ź�У��ͣ���)
	SendBuff[1] = (uint8)(usSendDataLen>>8); 
	SendBuff[2] = (uint8)(usSendDataLen&0xFF);
	
	SendBuff[usSendDataLen+2] = SumCalc(&SendBuff[3],(usSendDataLen-1));
	
	SendBuff[usSendDataLen+3] = 0x0D;
	SendBuff[usSendDataLen+4] = 0x0A;	

	WriteMCU_CAN_UartData(SendBuff, usSendDataLen+5);	
 
}




uint8 GetAD_SwitchState(uint8 *buf)
{
    uint16 usTemp = 0;

	usTemp += (uint16)(*buf++<<8);
	usTemp += *buf++;
	STU_Systemstate.usInput_Voltage = usTemp;
	STU_Systemstate.ucBAT_Voltage = *buf++;
	STU_Systemstate.ucSwitch1 = *buf++;
	STU_Systemstate.ucSwitch2 = *buf++;
	if(STU_Systemstate.ucSwitch2&BIT(4))
		m_stuCommState.ucRcvCan2DataFlag = 1;
	else
		m_stuCommState.ucRcvCan2DataFlag = 2;
	if(STU_Systemstate.ucSwitch2&BIT(7))
		m_stuCommState.ucRcvCan1DataFlag = 1;
	else
		m_stuCommState.ucRcvCan1DataFlag = 2;
	
	STU_Systemstate.ucSwitch3 = *buf++;

	STU_Systemstate.DisassmbleSwitch = *buf++;
	stuSleep.ucWorkingMode = *buf++;
    g_usSTMCU_Ver = *buf++;
	g_usSTMCU_Ver += (uint16)((*buf++)<<8);
	return 10;	
}


void MCU_DealCanMessage(uint8* p, uint16 uslen)
{
    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand;
	uint16 usCanNum = 0;
	uint8 ucSN,ucResult;
	MessageDetail stuMessageCAN; 
	uint16 i = 0;
	uint8 temp = 0;
	uint8 ucTlvNum = 0;
	uint8 ucTlvTag;
	uint16 usTlvLen = 0;
	uint8 m = 0;
	uint8 *ptemp;
	uint8 *ptrTemp; 
	uint8 *ptrCan;
	uint8 sbuff[30]={0};
	static uint8 ptr[1300] = {0};
	static uint16 ptrLen = 0;
	static uint8 ucNum = 0;
	STU_Date time;
    STU_DataTime stuDate;	
    PC_SendDebugData((uint8*)"Message",7 , DEBUG_GPSMODULE);


    if(p[0]==0x7E&&p[3]==0x01&&(p[uslen-2]!=0x0D||p[uslen-1]!=0x0A)&&uslen==512&&ucNum==0)
    {
        ucNum = 1;
        memcpy(&ptr[ptrLen],p,uslen);
		ptrLen = uslen;
		return;
	}
	else if((p[uslen-2]!=0x0D||p[uslen-1]!=0x0A)&&ucNum==1)
	{
        ucNum = 2;
		memcpy(&ptr[ptrLen],p,uslen);
		ptrLen += uslen;
		return;
	}
	else if((p[uslen-2]==0x0D&&p[uslen-1]==0x0A)&&ucNum)
	{
        ucNum = 0;
		memcpy(&ptr[ptrLen],p,uslen);
		uslen += ptrLen;
		ptrLen = 0;
	}
	else
	{	
	    memcpy(ptr,p,uslen);
		ucNum = 0;
	}	

	//PC_SendDebugData(ptr, uslen, DEBUG_MCUMODULE);
	sprintf(sbuff, "PTR: %d-%d-%d %d\r\n",ptr[0],ptr[uslen-2],ptr[uslen-1],uslen);
	PC_SendDebugData(sbuff, strlen(sbuff), DEBUG_GPSMODULE);
	
    if((ptr[0]!=0x7E)||(ptr[uslen-2]!=0x0D)||(ptr[uslen-1]!=0x0A))
		return;
	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
    ucCommand = ptr[3];
	ucSN = ptr[4];
	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
	if(ucSumNum!=ptr[uslen-3])
	{
        PC_SendDebugData((uint8*)"sumErr",6 , DEBUG_GPSMODULE);
		return;
	}
	
	if(ucCommand == 0x01)      //����MCU������CAN����
	{
    //    PC_SendDebugData((uint8*)"uccommand",9 , DEBUG_GPSMODULE);
		
    	ucTlvNum = ptr[5];     //TLV����
    	if(ucTlvNum<1)
			return;

		ptemp = ptr+6;
        for(m=0;m<ucTlvNum;m++)
        {
            ptrCan = ptemp;
    	    ucTlvTag = *ptemp++;     //TLV-Tag
    		if(ucTlvTag==0x01)       //CAN1����
    		{
             //   PC_SendDebugData((uint8*)"TLV1\r",5 , DEBUG_GPSMODULE);
        		m_stuCommState.usCan1RcvErrTime = 0;
        	 	m_stuCommState.ucCan1CommState = 0;
        	 	m_stuCommState.ucRcvCan1DataFlag = 1;        	 
                usTlvLen = (uint16)(*ptemp++<<8);
                usTlvLen += *ptemp++;
				if(usTlvLen>1024)
					return;
               // WriteCanToFlash(FLASH_PAGEADDR_CAN, usTlvLen+3, ptrCan);  //ȡ��
                usCanNum = *ptemp++;

			    memcpy((uint8*)&time,ptemp,6);	
				memcpy((uint8*)&stuDate,ptemp,6);
				stuDate.usmSec = (uint16)(ptemp[6]<<8) + ptemp[7];
				if(time.ucYear>=19&&time.ucMon&&time.ucDay)
				{
            	  /*  time_regs.year = time.ucYear;
            	    time_regs.month = time.ucMon;
            	    time_regs.date = time.ucDay;
            	    time_regs.hour = time.ucHour;
            	    time_regs.minute = time.ucMin;
            	    time_regs.second = time.ucSec; */
                    memcpy((uint8*)&stu_STM32_MCU.Date,ptemp,6);
				   // SaveCanToFlash(usTlvLen+3, ptrCan);		  //test 			
				}	
                ptemp+=8;
        		for(i=0;i<usCanNum;i++)
        		{
        		 
                    ptrTemp = ptemp;
                 //   CanConfig_Data_Collect(stuDate, ptrTemp);
                    stuMessageCAN.CANID = (uint32)(ptrTemp[0]<<24) + (uint32)(ptrTemp[1]<<16) +(uint32)(ptrTemp[2]<<8) + ptrTemp[3];

                    memcpy(stuMessageCAN.CANRE, &ptrTemp[6],8);
                    ptemp+=14;
            		temp = CanFrameFilter(stuMessageCAN.CANID);
                 	switch(temp)												//FFΪ1��IDΪ29λ
                   	{
                   		case 3:   // 3=��ֻ֡��Ҫ�ϴ�
            				AddCanFrameToBuff(stuMessageCAN.CANID, stuMessageCAN.CANRE);
            				break;
            			case 2:   // 2=ֻ��Ҫ���⴦��
                            DealCan1Message(stuMessageCAN);
            				break;
            			case 1:  //��Ҫ���⴦�����ϱ�
            				AddCanFrameToBuff(stuMessageCAN.CANID, stuMessageCAN.CANRE);
                            DealCan1Message(stuMessageCAN);
            				break;
            			default:
            				break;
                   	}	    			
        		}
    		}	
    		else if(ucTlvTag==0x03)   //RTCʱ��
    		{
    		  //   printf("TLV3\n");
			    //�ж�Ҫ��ҪУ׼RTCʱ��
			    usTlvLen = (uint16)(*ptemp++<<8);
				usTlvLen += *ptemp++;	
				if(usTlvLen==6)
				{
				    memcpy((uint8*)&time,ptemp,6);	
					if(time.ucYear>=19&&time.ucMon&&time.ucDay)
					{
                	   /* time_regs.year = time.ucYear;
                	    time_regs.month = time.ucMon;
                	    time_regs.date = time.ucDay;
                	    time_regs.hour = time.ucHour;
                	    time_regs.minute = time.ucMin;
                	    time_regs.second = time.ucSec;   */  
						memcpy((uint8*)&stu_STM32_MCU.Date,ptemp,6);
					}
                 //   memcpy((uint8*)&stu_STM32_MCU.Date,ptemp,6);
                    ptemp+=usTlvLen;
				}
				else
					return;

    		}
			else if(ucTlvTag==0x04)  //AD�ɼ�+������
			{
			  //  printf("TLV4\n");
			
			    usTlvLen = (uint16)(*ptemp++<<8);
				usTlvLen += *ptemp++;	
				if(usTlvLen==10)
				{
				    GetAD_SwitchState(ptemp);
                    ptemp+=usTlvLen;
				}
				else
					return;
			}			
        }
		//Ӧ������
		//MCU_SendReqCmdData(0x81, ucSN, ucResult);			
	}
	else if(ucCommand==0x82)     //MCU�Ժ��İ巢�������Ӧ��
	{
        ucResult = ptr[5];   // 0-�ɹ���1-�쳣��
	}	
}


//-----�ⲿ����ʵ��------------------------------------------------------------
/******************************************************************************
** ��������: pthread_MCU_Function
** ��������: ��ȡ������Э������MCU�����߳�,������ʽ
** 
** ��    ��: data
** ��    ��: ��
** ��    ��: ��
** 
** ��    ��: lxf
** ��    ��: 2019-04-02
**-----------------------------------------------------------------------------
*******************************************************************************/
void* pthread_MCU_Function(void* data)
{

	uint16 usLen = 0;   //���������յ������ݳ���    
	uint8 *p=NULL;
  
    MCU_CAN_Uart_Init();
    McuInit();
    McuReadFromMemory();
	CANConfig_ReadCANCFGFile_Init();//��ȡCAN��Ƶ�ɼ������ļ�
    CANConfig_extract_Param();         //����CAN��Ƶ�ɼ�������Ϣ
	CanConfig_Param_Init();         //��Ƶ�ɼ�CAN֡�ϴ�������ʼ��
    CAN_ClearCanDataUpbuff();       //�������
	while(1) 
    {       
     
        if(ReadMCU_CAN_UartData(&p, &usLen))
        { 
            PC_SendDebugData(p, usLen, DEBUG_MCUMODULE);           
            MCU_DealCanMessage(p, usLen);   
		//	PrintTime();
		//   printf("read: %d , %s\n", p, usLen);
 		    usLen = 0;
        }
      //  usleep(20*One_MilliSecond);
    }
}

#ifdef  __cplusplus
}
#endif

#endif
