/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
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
 * 202-11-08, lxf�������ļ�
 *
 */
//-----ͷ�ļ�����------------------------------------------------------------

#include "config.h"
#include "CollectCompositelayer.h"
#include "CollectInterfaceLayer.h"
#include "CollectModule.h"

//-----�ⲿ��������------------------------------------------------------------
extern STUSystem g_stuSystem;
extern STUSYSParamSet g_stuSYSParamSet;
extern STU_SSData SSData;
//-----�ڲ���������------------------------------------------------------------


struct STU_Sysstruct STU_Systemstate,STU_SystemstatePre;



/**********************************************************************************
** ��������: GetInput_Voltage
** ��������: �ⲿ����ģ����ȡ��ƿ��ѹ
** ��            ��: �޲���
** �䡡      ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 �� : 
** �ա�	 �� : 2019��07 ��19 ��

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
** �ա�	 �� : 2019��03 ��19 ��

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
** �ա�	 �� : 2019��03 ��19 ��

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
	
	//WriteToFlash(FLASH_PAGEADDR_WORKTIME1, 0, 7, aucTemp);//20110303��Կ�׿����ɸ߱�Ϊ��ʱ��ʼ���湤��Сʱ���洢����
	//WriteToFlash(FLASH_PAGEADDR_WORKTIME2, 0, 7, aucTemp);
	PC_SendDebugData((uint8 *)("sw2e"), 4, DEBUG_ANYDATA);
}

//���湤��Сʱ��RTC RAM
void SaveWorktimeToRTCRAM(uint32 uiworktime)
{
//	RTC_WriteBackupRegister(RTC_BKP_DR0, 0x12345678);
//	RTC_WriteBackupRegister(RTC_BKP_DR1, STU_Systemstate.uiWorkingTime);
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
#if 0
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
	#endif
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
** �ա�	 �� : 2019��03 ��19 ��

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
** �ա�	 ��: 2019��03 ��19 ��

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


/******************************************************************************
** ��������: ReportSwitchState
** ��������: ����������б仯֪ͨSYSTEM
** ��    ��: �޲���
** �䡡  ��: 
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2019��03 ��19 ��

**-----------------------------------------------------------------------------
*******************************************************************************/
void ReportSwitchState (void)
{
    uint8 ucFlag = 0;     //1=����״̬�����ϱ�

    /*Կ�׿��������      */
  	if((STU_Systemstate.ucSwitch1&BIT(1))^(STU_SystemstatePre.ucSwitch1&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch1&BIT(1)) 
        {
           // SYS_PutDataQ((uint8*)&f_KeyOFF);        //Key���ع�
           ucFlag = 1;
        }
        else 
        {
         //   SYS_PutDataQ((uint8*)&f_KeyON);         //Key���ؿ�
           ucFlag = 1;
        }
    }


		//����ж�״̬�ϱ�
    if((STU_Systemstate.ucSwitch3&BIT(0))^(STU_SystemstatePre.ucSwitch3&BIT(0)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(0)) 
        {
         //   SYS_PutDataQ((uint8*)&f_NO5V);
             ucFlag = 1;
        }
        else 
        {
         //  SYS_PutDataQ((uint8*)&f_GET5V); 
             ucFlag = 1;
        }
    }
		//�������״̬�ϱ�
    if((STU_Systemstate.ucSwitch3&BIT(1))^(STU_SystemstatePre.ucSwitch3&BIT(1)))
    {
        if(STU_SystemstatePre.ucSwitch3&BIT(1)) 
        {
             ucFlag = 1;
        }
        else 
        {
             ucFlag = 1;
        }
    }

	/*
	//���Ǳ���״̬�ϱ�
	if((STU_Systemstate.ucSwitch2 & BIT(3))^(STU_SystemstatePre.ucSwitch2 & BIT(3)))
    {
        if(STU_SystemstatePre.ucSwitch2 & BIT(3)) 
        {
          //  SYS_PutDataQ((uint8*)&f_BOXNormal);     //֪ͨ����2�ָ�����   
            ucFlag = 1;
        }
        else 
        {
         //   SYS_PutDataQ((uint8*)&f_BOXAlarm);      //֪ͨ����2��Ϊ�쳣    
            ucFlag = 1;
        }
    }
	*/
	if(ucFlag)
	{
		SSData.usTimer = 0;
		ucFlag = 0;
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
** �ա�	 �� : 2019��03 ��19 ��

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
** �ա�	 �� : 2019��03 ��19 ��

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
** �ա�	 �� : 2019��03��19 ��

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
** �ա�	 �� : 2019��03 ��19 ��

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

#if 0

/**********************************************************************************
** ��������: GetBatDate
** ��������: ��ص�ѹ�ɼ�
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2019��03 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
int GetBatDate(void)
{
	int fd_bat,fd_scale;
	int ret;
    char rx_buf[128] = "";
    int ad_int,ad_data;
	double scale_double;
	

     fd_bat = open(DEV_TTY_BAT, O_RDWR|O_NOCTTY);
	 if (fd_bat < 0)
	 {
		//  printf("ERROR open %s ret=%d\n\r", DEV_TTY_BAT, fd_bat);
	//	 printf("ERROR open\n");
		  return 0;
	 }
	  ret =read(fd_bat, rx_buf, 127);//��ԭʼת��ֵ
	  //printf("ad=%s  fd=%d\n",rx_buf,fd_bat);
      ad_int = atoi(rx_buf);          
            
	  fd_scale = open(DEV_TTY_SCALE, O_RDWR|O_NOCTTY);
	  if (fd_scale < 0)
	  {
		//  printf("ERROR open %s ret=%d\n\r", DEV_TTY_SCALE, fd_scale);
	//	  printf("ERROR open\n");
		  return 0;
	  }
	   ret =read(fd_scale, rx_buf, 127);//��ϵ��
	   //printf("SCALE=%s  fd=%d\n",rx_buf,fd_scale);			
       scale_double = atof(rx_buf); 
            
       ad_data=(int)(ad_int*scale_double);
	   //printf("ad data=%d\n",ad_data);
			
       close(fd_bat);
       close(fd_scale);

	return ad_data;
}

/**********************************************************************************
** ��������: GetInputDate
** ��������: ��ص�ѹ�ɼ�
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2019��03 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
int GetInputDate(void)
{
	int fd_input,fd_scale;
	int ret;
    char rx_buf[128] = "";
    int ad_int,ad_data;
	double scale_double;
	

     fd_input = open(DEV_TTY_INPUT, O_RDWR|O_NOCTTY);
	 if (fd_input < 0)
	 {
		 // printf("ERROR open %s ret=%d\n\r", DEV_TTY_INPUT, fd_input);
		//  printf("ERROR open\n");
		  return 0;
	 }
	  ret =read(fd_input, rx_buf, 127);//��ԭʼת��ֵ
	  //printf("ad=%s  fd=%d\n",rx_buf,fd_bat);
      ad_int = atoi(rx_buf);          
            
	  fd_scale = open(DEV_TTY_SCALE, O_RDWR|O_NOCTTY);
	  if (fd_scale < 0)
	  {
		 // printf("ERROR open %s ret=%d\n\r", DEV_TTY_SCALE, fd_scale);
		//  printf("ERROR open\n");
		  return 0;
	  }
	   ret =read(fd_scale, rx_buf, 127);//��ϵ��
	   //printf("SCALE=%s  fd=%d\n",rx_buf,fd_scale);			
       scale_double = atof(rx_buf); 
            
       ad_data=(int)(ad_int*scale_double);
	   //printf("ad data=%d\n",ad_data);
			
       close(fd_input);
       close(fd_scale);

	return ad_data;
}
/**********************************************************************************
** ��������: ADConvert
** ��������: ADת��������ͨ��0��1
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2019��03 ��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void  ADConvert(void)
{
    static uint8 i=0;

	STU_AD.usVBAT[i]          = (uint16)GetBatDate();//ADC2ConvertedValue;	    //V_BAT
	usleep(10);
	STU_AD.usInput_Voltage[i] = (uint16)GetInputDate();//ADC3ConvertedValue;	//V_POWER

	i++;
	if(i>=10)
		i = 0;
}

/**********************************************************************************
** ��������: DealVoltage
** ��������: �ⲿ��ѹ�͵�ص�ѹ�����ɼ�10 ��ƽ��ֵ
** ��    ��: �޲���
** �䡡  ��: �޲���
** ȫ�ֱ���: 
** ����ģ��:
** ����	 ��: 
** �ա�	 ��: 2013��03��19 ��

**--------------------------------------------------------------------------------
*********************************************************************************/
void  DealVoltage(void)
{
    uint32 uifTemp;
   
    uifTemp = DigitFilter1(STU_AD.usInput_Voltage, 10, 2);
    STU_Systemstate.usInput_Voltage = (uint16)(uifTemp*0.11989);//�����ⲿ��ѹֵ
    if(STU_Systemstate.usInput_Voltage < 50) //��Ϊ����жϺ�ɼ���ѹ������3V���ҵ�ѹ,���Ե��ɼ������ѹС��5Vʱ��Ϊ����жϸ�ֵΪ0 
    {
		STU_Systemstate.usInput_Voltage = 0;
    }
    uifTemp = DigitFilter1(STU_AD.usVBAT, 10, 2);//���������˲�����������ȥ��������Сֵ��ʣ��ȡƽ��ֵ
    STU_Systemstate.ucBAT_Voltage = (uint8)(uifTemp*0.032);//������ص�ѹֵ

}
#endif
/*-----�ļ�CollectInterfaceLayer.c����-----*/
