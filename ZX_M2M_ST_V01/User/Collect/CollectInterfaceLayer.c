/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: CollectInterfaceLayer.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: A/D��������������⺯���ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-08, , �������ļ�
 *
 */
//-----ͷ�ļ�����------------------------------------------------------------

#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectModule.h"



//-----�ڲ���������------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
uint16 Input_Voltage = 0;
uint8 switchr[3] = {0xaa,0xbb,0xcc};

struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;
struct STU_ADstruct STU_AD;



static STUSYSMsgBus f_KeyOFF={5, KeyOFF, 0, 0, 0, 0}; //Կ�׿���ACC�ر�
static STUSYSMsgBus f_KeyON={5, KeyON, 0, 0, 0, 0}; //Կ�׿���ACC��
/*
static STUSYSMsgBus f_GSM_AntennaOFF={5, GSM_AntennaOFF, 0, 0, 0, 0}; //GSM���߹ر�
static STUSYSMsgBus f_GSM_AntennaON={5, GSM_AntennaON, 0, 0, 0, 0}; //GSM���ߴ�
*/


static STUSYSMsgBus f_BOXNormal={5, BOXNormal, 0, 0, 0, 0}; //��������
static STUSYSMsgBus f_BOXAlarm ={5, BOXAlarm,  0, 0, 0, 0}; //�����쳣


static STUSYSMsgBus f_GET5V={5, GET5V, 0, 0, 0, 0}; //�ⲿ����
static STUSYSMsgBus f_NO5V={5, NO5V, 0, 0, 0, 0}; //��ع���

//static STUSYSMsgBus f_PowerLow={5, PowerLow, 0, 0, 0, 0}; //��Դ����
//static STUSYSMsgBus f_PowerNormal={5, PowerNormal, 0, 0, 0, 0}; //��Դ�ظ�����
//static STUSYSMsgBus f_Battery_Charge={5, Battery_Charge, 0, 0, 0, 0}; //��Դ����
//static STUSYSMsgBus f_Battery_NOCharge={5, Battery_NOCharge, 0, 0, 0, 0}; //��Դ�ظ�����


//-----�ⲿ��������------------------------------------------------------------
extern STUSleep stuSleep;              //���߽ṹ�嶨��




/**********************************************************************************
** ��������: GetInput_Voltage
** ��������: �ⲿ����ģ����ȡ��ƿ��ѹ
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
uint16 GetInput_Voltage(void)
{
	return STU_Systemstate.usInput_Voltage;
}
/**********************************************************************************
** ��������: GetBAT_Voltage
** ��������: �ⲿ����ģ����ȡ��ص�ѹ
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 GetBAT_Voltage(void)
{
	return STU_Systemstate.ucBAT_Voltage;
}

/**********************************************************************************
** ��������: SaveWorktime
** ��������: ��32λ���ݷֳ�4��8λ�ֽ�,�ٱ�����������
** ��    ��: uiworktime
** �䡡  ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void SaveWorktime(uint32 uiworktime)
{
	uint8 aucTemp[7];
	 
	aucTemp[0] = 0x12;
	aucTemp[1] = 0x34;
    aucTemp[2] = uiworktime&0xff;
    aucTemp[3] = (uiworktime>>8)&0xff;
    aucTemp[4] = (uiworktime>>16)&0xff;
    aucTemp[5] = (uiworktime>>24)&0xff;
	aucTemp[6] = SumCalc(aucTemp,6);
	
	WriteToFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);//20110303��Կ�׿����ɸ߱�Ϊ��ʱ��ʼ���湤��Сʱ���洢����
	WriteToFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
	PC_SendDebugData((uint8 *)("sw2e"), 4, DEBUG_ANYDATA);
}

//���湤��Сʱ��RTC RAM
void SaveWorktimeToRTCRAM(uint32 uiworktime)
{
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0x12345678);
	RTC_WriteBackupRegister(RTC_BKP_DR1, STU_Systemstate.uiWorkingTime);
}
/**********************************************************************************
** ��������: InitWorktime
** ��������: ��EERPROM��RTC RAM�ж�ȡ�ܹ���Сʱ
** �䡡  ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : HHM
** �ա�	 �� : 2015��07 ��9 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void InitWorktime()
{
	uint8 aucTemp[7];
	uint32 uiTemp;

	uiTemp = RTC_ReadBackupRegister(RTC_BKP_DR0);
	if(0x12345678!=uiTemp)//�ϵ�
	{
		ReadFromFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);
		if(aucTemp[0]==0x12 && aucTemp[1]==0x34 && aucTemp[6]==SumCalc(aucTemp,6))
		{
			STU_Systemstate.uiWorkingTime = StrToUint32(&aucTemp[2], MEM_MOD_LITLE);
		}
		else
		{
			PC_SendDebugData((uint8 *)("rd wkt1 err"), 11, DEBUG_ANYDATA);
			ReadFromFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
			if(aucTemp[0]==0x12 && aucTemp[1]==0x34 && aucTemp[6]==SumCalc(aucTemp,6))
			{
				STU_Systemstate.uiWorkingTime = StrToUint32(&aucTemp[2], MEM_MOD_LITLE);
			}
			else
			{
				PC_SendDebugData((uint8 *)("rd wkt2 err"), 11, DEBUG_ANYDATA);
				STU_Systemstate.uiWorkingTime = 0;
			}
		}
	}
	else
	{
		uiTemp = RTC_ReadBackupRegister(RTC_BKP_DR1);
		STU_Systemstate.uiWorkingTime = uiTemp;
	}
}


/**********************************************************************************
** ��������: CountWorkTime ����Сʱͳ�Ƶ�λΪ����
** ��������: ͳ�ƹ���Сʱ:����������Ϊ�ߵ�ƽʱ���ۼ�,���ɸߵ�ƽ
**                            ��Ϊ�͵�ƽʱ����д洢����������ʱ��6���Ӵ洢һ��
** ��    ��: �޲���
** �䡡  ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void CountWorkTime(void)
{
    static uint8 ucCount60s = 60;        // һ���Ӽ�ʱ
	uint8 acc;
	static uint8 acc_pre  = 0;
    static uint8 ucAccOffCount = 0;
	
	acc = GetAccState();
	if(acc==1)
	{
		STU_Systemstate.uiWorkingTime++;   
        if(!(--ucCount60s))
        {
            ucCount60s = 60;
			SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
        }
    }
    else
	{
	 	if(acc_pre == 1)//ACC ON----->ACC OFF
 		{
 			SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
 			ucAccOffCount = 5;
 		}
	}
    if(ucAccOffCount)
    {
        if(!(--ucAccOffCount))
        {
    		SaveWorktime(STU_Systemstate.uiWorkingTime);
		}
	}	
	acc_pre = acc;
}


/**********************************************************************************
** ��������: GetWorkingTime
** ��������: �ⲿ����ģ����ȡ��ص�ѹ
** ��    ��: �޲���
** ��    ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
uint32 GetWorkingTime(void)
{
	return STU_Systemstate.uiWorkingTime;
}

//�����ܹ���ʱ��
void SetWorkingTime(uint32 uiWorkTime)
{
	STU_Systemstate.uiWorkingTime = uiWorkTime;
	SaveWorktimeToRTCRAM(STU_Systemstate.uiWorkingTime);
 	SaveWorktime(STU_Systemstate.uiWorkingTime);
}


/**********************************************************************************
** ��������: CheckSwitchState
** ��������: ��ȡ��������ǰ״̬ÿ100ms���һ��
** ��    ��: �޲���
** ��    ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void CheckSwitchState(void)
{
    static uint8 ucCountKye_Hig = 10;      	//�ܽŸ߳���1��,Key-onΪ�ߵ�ƽ
    static uint8 ucCountKye_Low = 10;        	//�ܽŵͳ���1��,Key-onΪ�͵�ƽ
    //static uint16 ucCountHulled_Hig = 50;	    //�ܽŸ߳���5��,�������ߵ�ƽ
	//static uint16 ucCountHulled_Low = 50;		//�ܽŵͳ���5��,�������͵�ƽ
    static uint16 ucCountGet5V_Hig = 50; //�ܽŸ߳���5��,��5V����ߵ�ƽ
    static uint16 ucCountGet5V_Low = 50; //�ܽŵͳ���5��,��5V���
    static uint8  GpsAntShot_Hig = 10;
	static uint8  GpsAntShot_Low = 10;
	static uint8  GpsAntOpen_Low = 10;
	static uint8  GpsAntOpen_Hig = 10;
//	static uint8  SIMCover_Hig=20;
//	static uint8  SIMCover_Low=20;
	
    if(ReadSwitchACC())  //�ж�ACC //ACC�����źŲɼ�
	{

		ucCountKye_Hig=10;
		if(!--ucCountKye_Low)
		{
			ucCountKye_Low=10;
			STU_Systemstate.ucSwitch1 &= ~BIT(1);
		}
	}
	else
	{
	
		ucCountKye_Low=10;
		if(!--ucCountKye_Hig)
		{
			ucCountKye_Hig=10;
            STU_Systemstate.ucSwitch1 |= BIT(1);
		}
	}       
	 
	 if(STU_Systemstate.usInput_Voltage>70)    //��繩����
	 {
		 ucCountGet5V_Low = 50;
		 if(ucCountGet5V_Hig) 
		 {
			 if(!(--ucCountGet5V_Hig))
			 {
				 ucCountGet5V_Hig = 50;
				 STU_Systemstate.ucSwitch3 &= ~BIT(0);	// ��繩����5V���	
			 }
		 }
	 }
	 else  
	 {
		 ucCountGet5V_Hig = 50;
		 if(ucCountGet5V_Low)
		 {
			 if(!(--ucCountGet5V_Low))
			 {
				 ucCountGet5V_Low = 50;
				 STU_Systemstate.ucSwitch3 |= BIT(0); 	// ��ع�����5V���
 
			 } 
		 }
	 }

 	//gps���߼��
	if(ReadGpsAntShot())
	{
		GpsAntShot_Low = 10;
		if(!(--GpsAntShot_Hig))
		{
			STU_Systemstate.DisassmbleSwitch |= BIT(5);
			GpsAntShot_Hig = 10;
		}
	}
	else
	{
		GpsAntShot_Hig = 10;
		if(!(--GpsAntShot_Low))
		{
			STU_Systemstate.DisassmbleSwitch &= ~BIT(5);
			GpsAntShot_Low = 10;
		}
	}
	if(ReadGpsAntOpen())
	{
		GpsAntOpen_Low = 10;
		if(!(--GpsAntOpen_Hig))
		{
			STU_Systemstate.DisassmbleSwitch |= BIT(6);
			GpsAntOpen_Hig = 10;
		}
	}
	else
	{
		GpsAntOpen_Hig = 10;
		if(!(--GpsAntOpen_Low))
		{
			STU_Systemstate.DisassmbleSwitch &= ~BIT(6);
			GpsAntOpen_Low = 10;
		}
	}

/*
	 //SIM���Ǽ��
	if(ReadSIMCardState()) 
	{
		SIMCover_Hig=20;
		if(!--SIMCover_Low)
		{
			SIMCover_Low=20;
			STU_Systemstate.DisassmbleSwitch &= ~BIT(4);   //SIM������
		}
	}
	else
	{
		SIMCover_Low=20;
		if(!--SIMCover_Hig)
		{
			SIMCover_Hig=20;
			STU_Systemstate.DisassmbleSwitch |= BIT(4);   //SIM�����
		}
	}
	*/
		
}
/******************************************************************************
** ��������: ReportSwitchState
** ��������: ����������б仯֪ͨSYSTEM
** ��            ��: �޲���
** �䡡      ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
void ReportSwitchState (void)
{
	//uint8 a[2] = {0xaa,0xaa};
	//uint8 b[2] = {0xbb,0xbb};

    /*Կ�׿��������      */
  	if((STU_Systemstate.ucSwitch1&BIT(1))^(STU_SystemstatePre.ucSwitch1&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch1&BIT(1)) 
        {
            SYS_PutDataQ((uint8*)&f_KeyOFF);        //Key���ع�
        }
        else 
        {
            SYS_PutDataQ((uint8*)&f_KeyON);         //Key���ؿ�
        }
    }


		//����ж�״̬�ϱ�
    if((STU_Systemstate.ucSwitch3&BIT(0))^(STU_SystemstatePre.ucSwitch3&BIT(0)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(0)) 
        {
            SYS_PutDataQ((uint8*)&f_NO5V);
        }
        else 
        {
           SYS_PutDataQ((uint8*)&f_GET5V); 
        }
    }

	//���Ǳ���״̬�ϱ�
	if((STU_Systemstate.ucSwitch2 & BIT(3))^(STU_SystemstatePre.ucSwitch2 & BIT(3)))
    {
        if(STU_SystemstatePre.ucSwitch2 & BIT(3)) 
        {
            SYS_PutDataQ((uint8*)&f_BOXNormal);     //֪ͨ����2�ָ�����          
        }
        else 
        {
            SYS_PutDataQ((uint8*)&f_BOXAlarm);      //֪ͨ����2��Ϊ�쳣    
        }
    }
	
	STU_SystemstatePre = STU_Systemstate;
} 
/******************************************************************************
** ��������:  GetACCState
** ��������:  �ⲿ������ȡԿ�׿��ص�ǰ״̬
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetAccState()
{
	return (STU_Systemstate.ucSwitch1 & BIT(1)) ? 1 : 0;
}

/******************************************************************************
** ��������:  GetPwrSupplyState
** ��������:  ��ȡ����Դ��ǰ����״̬
** ��    ��: �޲���
** �䡡  ��: �޲���
** ��    ��: 0=���Դ����, 1=���õ�ع���
** ����	 �� : hhm
** �ա�	 �� : 2016-7-6
*******************************************************************************/
uint8 GetPwrSupplyState()
{
	return (STU_Systemstate.ucSwitch3 & BIT(0)) ? 1 : 0;
}

/******************************************************************************
** ��������:  GetPwrLowState
** ��������:  ��ȡ����Դ����״̬
** ��    ��: �޲���
** �䡡  ��: �޲���
** ��    ��: 0=���Դ������, 1=���Դ����
** ����	 �� : hhm
** �ա�	 �� : 2016-7-6
*******************************************************************************/
uint8 GetPwrLowState()
{
	return (STU_Systemstate.ucSwitch3 & BIT(1)) ? 1 : 0;
}

/******************************************************************************
** ��������:  GetBoxOpenState
** ��������:  ��ȡ����״̬
** ��    ��: �޲���
** �䡡  ��: �޲���
** ��    ��: 0=��������, 1=���Ǳ���
** ����	 �� : hhm
** �ա�	 �� : 2016-7-6
*******************************************************************************/
uint8 GetBoxOpenState()
{
	return (STU_Systemstate.ucSwitch2 & BIT(3))?1 : 0;
}

uint8 READ_OPENBOX2()
{
	return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
}
/******************************************************************************
** ��������:  CheckBoxState
** ��������:  ��ȡ���п�������ǰ״̬ÿ100ms���һ��
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
void CheckBoxState(void)
{
    static uint8 ucCountBOX2Normal = 10;      //���п���2����
    static uint8 ucCountBOX2Alarm = 10;       //���п���2����

 
    if(READ_OPENBOX2())
    {
   		 ucCountBOX2Alarm = 20;
		 if(ucCountBOX2Normal)
		 {
			 if(!(--ucCountBOX2Normal))
			 {
				 ucCountBOX2Normal = 20;
				 STU_Systemstate.ucSwitch2 &= ~BIT(3);   //���п���2���� 
			 }
		 }
    }
    else    
    {
   		 ucCountBOX2Normal = 20;
		 if(ucCountBOX2Alarm)
		 {
			 if(!(--ucCountBOX2Alarm))
			 {
				 ucCountBOX2Alarm = 20;
				 STU_Systemstate.ucSwitch2 |=  BIT(3);  //���п���2����  
			 }
		 }
    }
 
}



/******************************************************************************
** ��������:  GetSwitch1State
** ��������:  �ⲿ������ȡ������1��ǰ״̬
**            B0����������    0=�͵�ƽ,1=�ߵ�ƽ��
**			  B1��Կ�׿���ACC    0=�ر�,1=�򿪣�
**            B2��Сʱ�ƿ��� 0=�ر�,1=�򿪣�
**            B3����ӵ�����״̬     0=�Ͽ�,1=��ͨ��
**            B4����ӵ縺��״̬     0=��ͨ,1=�Ͽ���
**            B5��������״̬     0=��ͨ,1=�Ͽ���
**            B6��GSM���߱���   0=�ޱ���, 1=����
**            B7������λ
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch1State(void)
{
	return STU_Systemstate.ucSwitch1;
}

/******************************************************************************
** ��������:  GetSwitch2State
** ��������:  �ⲿ������ȡ������2��ǰ״̬
**            B0���̵���1����״̬     0=δ����;1=����
**            B1���̵���2����״̬     0=δ����;1=����
**            B2�����б���1     0=����;1=����
**            B3�����б���2   0=����;1=����
**            B4������
**            B5��RS232ͨѶ״̬    0= δͨѶ��1=ͨѶ��
**            B6��RS232ͨѶ�Ǳ���ʾ״̬    0= δͨѶ��1=ͨѶ��
**            B7��CANͨѶ״̬    0= δͨѶ��1=ͨѶ��
** ��        ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch2State(void)
{
	return STU_Systemstate.ucSwitch2;
}

/******************************************************************************
** ��������:  GetSwitch3State
** ��������:  �������ɼ�3
**			  B0������״̬ 0=��繩�磻1=��ع��磻
**			  B1���������״̬   0=�����磻1=����
**			  B2���ڲ���س��״̬    0=δ���,1=��磻
**			  B3��������ʻ�ٶȱ��� 0=�ٶ�������1=���ٱ�����
**			  B4������λ��Խ�籨�� 0=��Խ�磻1= Խ�籨����
**			  B5������
**			  B6������
**			  B7������
** ��        ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 GetSwitch3State(void)
{
	return STU_Systemstate.ucSwitch3;
}


//��ȡGPS ���������״̬��
//����ֵ:	0=����,1=�쳣
uint8 GetGpsSwitchState()
{
	if(!(STU_Systemstate.DisassmbleSwitch&BIT(5)) ||(STU_Systemstate.DisassmbleSwitch&BIT(6)))//����
		return 1;
	else
		return 0;
}

/*****************************************************************************
 ** ��������: GetGpsAntShortState
 ** ��������: ��ȡGPS���߶�·״̬
 ** ��	  ��: ��
 ** ��	  ��: ��
 ** ��	  ��: GPS���߶�·״̬, 1=��·,0=����
 ** ����  �� : hhm
 ** �ա�  �� : 2016-7-6
 ******************************************************************************/
uint8 GetGpsAntShortState()
{
	return (!(STU_Systemstate.DisassmbleSwitch&BIT(5)))?1:0;
}

/*****************************************************************************
 ** ��������: GetGpsAntOpenState
 ** ��������: ��ȡGPS���߿�·״̬
 ** ��	  ��: ��
 ** ��	  ��: ��
 ** ��	  ��: GPS���߿�·״̬, 1=��·,0=����
 ** ����  �� : hhm
 ** �ա�  �� : 2016-7-6
 ******************************************************************************/
uint8 GetGpsAntOpenState()
{
	return (STU_Systemstate.DisassmbleSwitch&BIT(6))?1:0;
}

/*****************************************************************************
 ** ��������: GetBatChargeState
 ** ��������: ��ȡ��س��״̬
 ** ��	  ��: ��
 ** ��	  ��: ��
 ** ��	  ��: ��س��״̬, 1=���,0=δ���
 ** ����  �� : hhm
 ** �ա�  �� : 2016-8-19
 ******************************************************************************/
uint8 GetBatChargeState(void)
{
	return (STU_Systemstate.ucSwitch3&BIT(2))?1:0;
}

/*****************************************************************************
 ** ��������: GetBatVolLowState
 ** ��������: ��ȡ��ص�ѹ��״̬
 ** ��	  ��: ��
 ** ��	  ��: ��
 ** ��	  ��: ��ص�ѹ��״̬, 1=��ѹ��,0=��ѹ����
 ** ����  �� : hhm
 ** �ա�  �� : 2016-8-19
 ******************************************************************************/
uint8 GetBatVolLowState(void)
{
	return (STU_Systemstate.ucSwitch3&BIT(5))?1:0;
}


/******************************************************************************
** ��������: InputVoltageCheck
** ��������: ���ⲿ��ѹ�Ĵ���
**           ���PWR_STAΪ�͵�ƽ������Ϊ��繩�磬���PWR_STAΪ
             �ߵ�ƽ��  ���ع��磬�϶�Ϊ��ع���;����ⲿ��ѹ
**           ����9V����С�������õ͵�ѹ�򱨾�:��Դ��ѹ��

** ��    ��: �޲���
** ��    ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
void InputVoltageCheck(void)
{
	static uint16 usInputVolLowTime = 5;              // �ⲿ��ѹ�������ڱ�����ѹʱ���ݶ�Ϊ5��
	
    //if(STU_Systemstate.usInput_Voltage<=g_stuSYSParamSet.ucVoltageLowAlarm)   
    if(STU_Systemstate.usInput_Voltage<=220)
    {
        if(usInputVolLowTime)
        {
            if(!(--usInputVolLowTime))
            {
                STU_Systemstate.ucSwitch3|=BIT(1);      // �ⲿ��ѹ��
              // 	SYS_PutDataQ((uint8*)&f_PowerLow);      // ��Դ��ѹ��        
            }
        }
    }
    else
    {
        usInputVolLowTime = 5;                        // �ݶ�Ϊ30��
        if(STU_Systemstate.ucSwitch3&BIT(1))
        {
            STU_Systemstate.ucSwitch3&=~BIT(1);         // �ⲿ��ѹ����
			//SYS_PutDataQ((uint8*)&f_PowerNormal);
        }           
    }
    
}

/******************************************************************************
** ��������: BatVoltageCheck
** ��������: �ԃȲ���ص�ѹ�Ĵ���,��ѹ����3.5V����5����Ϊ��ѹ��
** ��    ��: �޲���
** ��    ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2011��07 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
void BatVoltageCheck(void)
{
	static uint16 ucBVLT = 5;              // ��ѹ��������3.5������ѹʱ���ݶ�Ϊ5s
	static uint16 ucBVHT = 5;              // ��ѹ��������3.5������ѹʱ���ݶ�Ϊ5s

	if(STU_Systemstate.ucBAT_Voltage<35)     
    {
    	ucBVHT = 5;
        if(ucBVLT>0)
        {
            if(!(--ucBVLT))
            {
                STU_Systemstate.ucSwitch3 |= BIT(5);      // �ⲿ��ѹ��
            }
        }
    }
    else
    {
    	ucBVLT = 5;
        if(ucBVHT>0)
        {
            if(!(--ucBVHT))
            {
                STU_Systemstate.ucSwitch3 &= ~BIT(5);      // �ⲿ��ѹ��
            }
        }     
    }
}


 /*****************************************************************************
 ** ��������: BatContronl
 ** ��������: ֻ��ACC���߷���������Ϊ��ʱ���ܶԵ�س��
 **
 ** ��	  ��: ��
 ** ��	  ��: ��
 ** ��	  ��: 
 **
 ** ����  �� : 
 ** �ա�  �� : 2011��07 ��19 ��

 **----------------------------------------------------------------------------
 ******************************************************************************/
 void BatContronl(void)
 {
     static uint16 usTimer=0;                      
	 
	 if(!(STU_Systemstate.ucSwitch3 & BIT(0))) 
	 {
	 //	 if(STU_Systemstate.ucSwitch1 & BIT(1))
	 //	 {
		     usTimer = 0;
			 if(STU_Systemstate.ucBAT_Voltage<41)
			 {
				 ChargeOn(); 					//����س��
				 STU_Systemstate.ucSwitch3 |= BIT(2);	  
			 }
			 if(STU_Systemstate.ucBAT_Voltage>41)
			 {
				 ChargeOff(); 				   	//�رյ�س��
				 STU_Systemstate.ucSwitch3 &= ~BIT(2);
			 }
	// 	 }
	 }
	 else										//��ع���
	 {
     	if(STU_Systemstate.ucBAT_Voltage<35) //��ر���
     	{
     	    if(usTimer++>30)
     	    {
             	usTimer = 0;		 	
    		 	g_stuSystem.ucShutDownTime = 10;
     	    }
     	}
		else
		{
            usTimer = 0;
		}
		 
		ChargeOff(); 					   //�رճ��ܽ�
		STU_Systemstate.ucSwitch3 &= ~BIT(2);
	 }
	 
 }


 void WatchDogHard(void)
 { 
/*	 if((FIO2PIN&BIT(2)))
		 WDT_LOW();
	 else
		 WDT_HIGH();
		 */
 }



uint16 ADC2ConvertedValue;
uint16 ADC3ConvertedValue;
void ADInit(void)
{
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADC3, DMA2 and GPIO clocks ****************************************/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_2;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC3->DR); //ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* DMA2 Stream0 channel3 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_1;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC2->DR); //ADC3_DR_ADDRESS;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC2ConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream3, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream3, ENABLE);
  /* Configure ADC3 Channel7 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC3, &ADC_InitStructure);
  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_112Cycles);
 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC3, ENABLE);
  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC3, ENABLE);
  /* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  ADC_SoftwareStartConv(ADC3);

   /* ADC2 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  /* ADC3 regular channel7 configuration *************************************/
  ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_112Cycles);
 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
  /* Enable ADC2 DMA */
  ADC_DMACmd(ADC2, ENABLE);
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  ADC_SoftwareStartConv(ADC2);
}


/**********************************************************************************
** ��������: ADConvert
** ��������: ADת��������ͨ��0��1
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/

void  ADConvert(void)
{
    static uint8 i=0;

	STU_AD.usVBAT[i]          = ADC2ConvertedValue;	//V_BAT
	STU_AD.usInput_Voltage[i] = ADC3ConvertedValue;	//V_POWER

	i++;
	if(i>=10)
		i = 0;
}



/**********************************************************************************
** ��������: DealVoltage
** ��������: �ⲿ��ѹ�͵�ص�ѹ�����ɼ�10 ��ƽ��ֵ
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2011��07 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/

void  DealVoltage(void)
{
    uint32 uifTemp;
   
    uifTemp = DigitFilter1(STU_AD.usInput_Voltage, 10, 2);
    STU_Systemstate.usInput_Voltage = (uint16)(uifTemp*0.09659);//�����ⲿ��ѹֵ
    if(STU_Systemstate.usInput_Voltage < 50) //��Ϊ����жϺ�ɼ���ѹ������3V���ҵ�ѹ,���Ե��ɼ������ѹС��5Vʱ��Ϊ����жϸ�ֵΪ0 
    {
		STU_Systemstate.usInput_Voltage = 0;
    }
    uifTemp = DigitFilter1(STU_AD.usVBAT, 10, 2);//���������˲�����������ȥ��������Сֵ��ʣ��ȡƽ��ֵ
    STU_Systemstate.ucBAT_Voltage = (uint8)(uifTemp*0.02578);//������ص�ѹֵ

}


uint8 BuildAD_Switch(uint8 *buf)
{
    uint16 usTemp = 0;
	
	usTemp = GetInput_Voltage();
    *buf++ = (usTemp>>8) & 0xff;
	*buf++ =  usTemp & 0xff;
	*buf++ = GetBAT_Voltage();
	*buf++ = GetSwitch1State();
	*buf++ = GetSwitch2State();
	*buf++ = GetSwitch3State();
	if(0xaa==g_stuSystem.ucWDTState)
		STU_Systemstate.DisassmbleSwitch |= BIT(0);
	else
		STU_Systemstate.DisassmbleSwitch &= ~BIT(0);
	*buf++ = STU_Systemstate.DisassmbleSwitch;
	*buf++ = stuSleep.ucWorkingMode;
	*buf++ = HW_VERSION&0xFF;      //���Э��������Ӳ���汾��
	*buf++ = (uint8)(HW_VERSION>>8)&0xFF;

	return 10;	
}
#if 0
//ͨ��4Gģ�鴫�͵Ķ�λ״̬�����ж�
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
#endif


/*-----�ļ�CollectInterfaceLayer.c����-----*/
