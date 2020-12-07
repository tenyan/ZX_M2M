/*
 * Copyright(c)2020, XXXXX��˾Ӳ���з���
 * All right reserved
 *
 * �ļ�����: McuChQiUpload.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�Ϊ����������̼���������ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2020-01-17, by  lxf, �������ļ�
 *
 */
#include "config.h"
#include "IAP.h" 
//-----ͷ�ļ�����------------------------------------------------------------


//-----�ⲿ��������------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
extern uint16 f_usUpLoadCmdSn;				//�����������ˮ��
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
//-----�ڲ���������------------------------------------------------------------
STUKCMCUDownload stu_KCMCUDownload;
//uint8 ExtMCUUartSendBuff[1024] = {0};
//-----�ڲ���������------------------
/*****************************************************************************/

#if 1

void KCMCU_ProgramUpdate_Init(void)
{
    uint32 uiCounter = 0;
	
//��������ܰ���
    uiCounter = stu_KCMCUDownload.uiSoftwareWSize/38;
	if(stu_KCMCUDownload.uiSoftwareWSize%38)
		uiCounter +=1; 

    stu_KCMCUDownload.uiProgramTotPackets = uiCounter*5;
//��������ܰ���
    uiCounter = stu_KCMCUDownload.uiParamSize/8;
    if(stu_KCMCUDownload.uiParamSize%8)
		uiCounter += 1;
    stu_KCMCUDownload.uiParameterTotPackets = uiCounter;
    stu_KCMCUDownload.ucRepeatSendCount = 0;
	stu_KCMCUDownload.uiOffsetAddr = 0;
	stu_KCMCUDownload.ucLoadStep = 1;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}

//����������������
void KCMCU_Update_Start_Command(void)
{
    uint8 abuff[8]={0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//	stu_McuFirmware.usMcuUPdateTimerOut = Mcu_UPdateTimerOut;

    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
    CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x06F20000, 8, abuff);     
}

//��������ѯ������ ucType=0:�̼���������;ucType=1:��������
void KCMCU_Update_Imfomation(uint8 ucType,uint32 uiTotalPacket)
{
    uint8 abuff[8]={0,0,0,0,0xFF,0xFF,0xFF,0xFF};

   //�ܰ���
	abuff[0] = (uint8)(uiTotalPacket>>24);
	abuff[1] = (uint8)(uiTotalPacket>>16);
	abuff[2] = (uint8)(uiTotalPacket>>8);
	abuff[3] = (uint8)(uiTotalPacket&0xFF);
    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s


    if(ucType==0)
        CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x06820000, 8, abuff);  
	else
        CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x06910000, 8, abuff);  		
}

/******************************************************************************
** ��������:  1)���жϽ���ƽ̨�°��Ƿ������ͨ��У�飬2)��ȡ�����ļ��ܰ���,
              3)�Ӵ洢���ж�ȡ����
** ��������:  ÿ�ζ�ȡ380Byte��������,ȡǰ380Byte�ֳ�10�飬ÿ��38Byte
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2020-01-20
**-----------------------------------------------------------------------------
*******************************************************************************/
//��ʱδ�����������ݲ���380�Ĵ���ʽ
void KCMCU_CreateProgramUpPackage(void)
{
    uint8 abuff[400];
	uint8 aCanbuff[8];
//	uint16 usPackets = 0;    //�����ܰ���
//	uint8 ucnum,ucLastPacketlen=0;
	uint8 i=0;	
    uint8 j=0;
	uint16 usFlashAddr = 0;   //�洢����ַ
	uint16 usRemainder = 0;   //����
	static uint32 uiID = 0x06820000;
	static uint32 uiIDPre;
//
//�ж������ļ��Ƿ���� 
 //   if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
//		return;
//�ж������豸����


    usFlashAddr = (stu_KCMCUDownload.uiOffsetAddr/512)*2 + FLASH_Firmware_MCU;
    usRemainder = stu_KCMCUDownload.uiOffsetAddr%512;

	if((512-usRemainder)>=380)  //����Ҫ��ҳ��ȡ
	{
        ReadFromFlash(usFlashAddr,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[usRemainder],380);
	}
	else
	{
        ReadFromFlash(usFlashAddr,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[usRemainder],512-usRemainder);
        ReadFromFlash(usFlashAddr+2,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[512-usRemainder],&stu_KCMCUDownload.aSendbuff[0],380-(512-usRemainder));
	}

//���Է��ͱ�־    
    if(stu_KCMCUDownload.ucRepeatSendCount==0)
    {
        uiIDPre = uiID;
	}
	else
	{
        uiID = uiIDPre;
	}
	
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x068265500)
			uiID = 0x06820001;
		
		if((i+1)%5 == 0)
		{
            memcpy(&aCanbuff[0],&abuff[i*8-j*2],6);
			aCanbuff[6] = 0;
			aCanbuff[7] = 0;
			j++;
		}
		else
		{
            memcpy(&aCanbuff[0],&abuff[i*8-j*2],8);
		}
   //����CAN֡
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, aCanbuff);   
        OSTimeDly(1);   //1ms���
	}
    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}

/******************************************************************************
** ��������:  KCMCU_CreateParameterUpPackage
              1)���жϽ���ƽ̨�°��Ƿ������ͨ��У�飬2)��ȡ�����ļ��ܰ���,
              3)�Ӵ洢���ж�ȡ����
** ��������:  ÿ�ζ�ȡ400Byte��������,ȡǰ400Byte�ֳ�10�飬ÿ��40Byte
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2020-02-4
**-----------------------------------------------------------------------------
*******************************************************************************/
void KCMCU_CreateParameterUpPackage(void)
{
    uint8 abuff[400];
	uint8 aCanbuff[8];
//	uint16 usPackets = 0;    //�����ܰ���
//	uint8 ucnum,ucLastPacketlen=0;
	uint8 i=0;	
//    uint8 j=0;
	uint16 usFlashAddr = 0;   //�洢����ַ
	uint16 usRemainder = 0;   //����
	static uint32 uiID = 0x06910000;
	static uint32 uiIDPre;
//
//�ж������ļ��Ƿ���� 
 //   if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
//		return;
//�ж������豸����


    usFlashAddr = (stu_KCMCUDownload.uiOffsetAddr/512)*2 + FLASH_KCMCU_PARAMETER_ADDR;
    usRemainder = stu_KCMCUDownload.uiOffsetAddr%512;

	if((512-usRemainder)>=400)
	{
        ReadFromFlash(usFlashAddr,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[usRemainder],400);
	}
	else
	{
        ReadFromFlash(usFlashAddr,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[usRemainder],512-usRemainder);
        ReadFromFlash(usFlashAddr+2,0,512,stu_KCMCUDownload.aSendbuff);
		memcpy(&abuff[512-usRemainder],&stu_KCMCUDownload.aSendbuff[0],400-(512-usRemainder));
	}

//���Է��ͱ�־  
    if(stu_KCMCUDownload.ucRepeatSendCount==0)
    {
        uiIDPre = uiID;
	}
	else
	{
        uiID = uiIDPre;
	}
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x069165500)
			uiID = 0x06910001;
		
        memcpy(&aCanbuff[0],&abuff[i*8],8);
   //����CAN֡
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, aCanbuff);   
        OSTimeDly(1);   //1ms���
	}
    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}


#if 0
//���͹̼���������(��Ҫ�ط���ʱ�� ID��δ���)
void KCMCU_ProgramUpdateSend(void)
{
    uint8 i = 0;
    uint8 abuff[8];
	static uint32 uiID = 0x06820000;
	
//���Է��ͱ�־    
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x068265500)
			uiID = 0x06820001;
        memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[i*8],8);
   //����CAN֡
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, abuff);   //��׼֡
        OSTimeDly(1);   //1ms���
	}
}




//���Ͳ�����������(��Ҫ�ط���ʱ�� ID��δ���)
void KCMCU_ProgramUpdateSend(void)
{
    uint8 i = 0;
    uint8 abuff[8];
	static uint32 uiID = 0x06910000;
	
//���Է��ͱ�־    
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x069165500)
			uiID = 0x06910001;
        memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[i*8],8);
   //����CAN֡
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, abuff);   //��׼֡
        OSTimeDly(1);   //1ms���
	}
}
#endif



/******************************************************************************
** ��������:  KCMcuFirmwareTimerOut
** ��������:  �Կ����������Ǳ��������ʱ ��ʱ����,ÿ��ִ��1��
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2020-02-14
**-----------------------------------------------------------------------------
*******************************************************************************/
void KCMcuFirmwareTimerOut(void)
{

    if(stu_KCMCUDownload.ucLoadStep==0)
		return ;
	/*
    if(stu_KCMCUDownload.ucRepeatSendCount > 10)
    {
        KCMCU_ProgramUpdate_Init();  //��������10�δ�ͷ��ʼ
	}
	*/

//�������ʱ ʱ�� 10��   
    if(stu_KCMCUDownload.ucMcuCmdTimerOut&&stu_KCMCUDownload.ucLoadStep)
    {
		stu_KCMCUDownload.ucMcuCmdTimerOut--;
		if(!stu_KCMCUDownload.ucMcuCmdTimerOut)
		{
           stu_KCMCUDownload.ucLoadStep = 0xFF;
		   // stu_McuFirmware.ucMcuRespflag = 1;
		}
	}	

	//�����ܳ�ʱʱ��
	if(stu_KCMCUDownload.usMcuUPdateTimerOut)
	{
		stu_KCMCUDownload.usMcuUPdateTimerOut--;
		if(!stu_KCMCUDownload.usMcuUPdateTimerOut)
		{
		
			if(stu_KCMCUDownload.ucUploadResult==0&&stu_KCMCUDownload.ucLoadStep!=6)
			{
                stu_KCMCUDownload.ucLoadStep = 6;
				stu_KCMCUDownload.ucUploadResult = 0;
			}
			else
			{
                stu_KCMCUDownload.ucLoadStep = 0;
			}
		}			
	}
}

/*
void Mcu_SendUpgradeResult(void)
{

    uint8 buff[50] = {0};
	static uint8 ucSN = 0;
    uint16 uslen = 0;
	
	ucSN++;
	buff[0] = 0x03;                          //�����ʶ
	buff[1] = FirmwareUpdate.ucdev;;         //��ǰ����Ŀ���豸
	if(stu_McuFirmware.ucUploadResult==0)		
	    buff[2] = 4;   //����������ʧ��
	else if(stu_McuFirmware.ucUploadResult==1)
		buff[2] = 2;   //�����������ɹ�
    uslen = 3;
		
	Ext_McuUart_DataSend(0x04, uslen, buff, ucSN);	
}
*/

//����������̼�����������ƺ���
void KCMCU_ProgramUpdate_Function(void)
{
  //��MCU�����������ݵĲ���
/*
	if(1!=GetCanRcvState(CAN_CHANNEL1))  //�ж�CAN ͨѶ״̬
		return;
	*/
	if(!GetAccState())
		return;
  
    switch(stu_KCMCUDownload.ucLoadStep)
    {
        case 0:
			break;
        case 1:    //����������������
			KCMCU_Update_Start_Command();
			break;
		case 2:    //���Ϳ������̼���������
            KCMCU_Update_Imfomation(0, stu_KCMCUDownload.uiProgramTotPackets);
			break;
		case 3:    //���Ϳ������̼����ݰ�����
		    KCMCU_CreateProgramUpPackage();
			break;
		case 4:    //���Ϳ�����������������
            KCMCU_Update_Imfomation(1, stu_KCMCUDownload.uiParameterTotPackets);
		    break;
		case 5:    //���Ͳ������ݰ�����
		    KCMCU_CreateParameterUpPackage();
		    break;
		case 6:     //֪ͨƽ̨MCU�����������  �������ӳ�2��Ƚ��            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
	        break;
	    case 0xFF:  //��ʼ��
			KCMCU_ProgramUpdate_Init();
			break;
		default:
		    break;
	}
}


void KCMCU_RecVCan_ACK(uint32 uiID, uint8 *arr)
{
    switch(uiID)
    {
//KCMCUӦ��---����
    case 0x17F2FFFF:              //Update Start Acknowledge
        if(arr[0]==0)        //0-Permited
        {
            stu_KCMCUDownload.ucLoadStep = 2;
			stu_KCMCUDownload.ucRepeatSendCount = 0;
        }
		else if(arr[0]==1)   //1-Rejected
		{
		}
		else if(arr[0]==2)   //2-Unknown Command
		{
		}
		break;
	case 0x1782FFFF:         //Update Imfomation Acknowledge(Result)(Result 0��Success 1��Failure 2��Unknown Command)
                             //Update Data Acknowledge(ACK/NACK 0��Success 1��Failure 2��Unknown Command)
	
        if(arr[0]==0) 
        {
            if(stu_KCMCUDownload.ucLoadStep==2)
            {
                stu_KCMCUDownload.ucLoadStep = 3; 
				stu_KCMCUDownload.ucRepeatSendCount = 0;
            }
            else if(stu_KCMCUDownload.ucLoadStep==3)
            {
				stu_KCMCUDownload.uiOffsetAddr+=380;
                stu_KCMCUDownload.ucRepeatSendCount = 0; 
            }				
        }
		else if(arr[0]==1)      //Failure
		{
		}
		else if(arr[0]==2)
		{
		}
		break;               
		                         
//KCMCUӦ��---����
    case 0x1791FFFF:        //Update Imfomation Acknowledge /Update Data Acknowledge 
        if(arr[0]==0)
        {
            if(stu_KCMCUDownload.ucLoadStep==4)
            {
                stu_KCMCUDownload.ucLoadStep = 5; 
				stu_KCMCUDownload.ucRepeatSendCount = 0;
            }
            else if(stu_KCMCUDownload.ucLoadStep==5)
            {
				stu_KCMCUDownload.uiOffsetAddr+=400;
                stu_KCMCUDownload.ucRepeatSendCount = 0; 
            }
		}
		else if(arr[0]==1)   //Failure
		{
		}
		else if(arr[0]==2)
		{
		}
		break;
	case 0x17F3FFFF:       //Data Complete Acknowledge //Result 0��Success 1��Failure 2��Unknown Command
        if(arr[0]==0)
        {
            if(stu_KCMCUDownload.ucLoadStep==3)
            {
				stu_KCMCUDownload.ucLoadStep = 4;
				stu_KCMCUDownload.ucRepeatSendCount = 0; 
            }
			else if(stu_KCMCUDownload.ucLoadStep==5)
			{
				stu_KCMCUDownload.ucLoadStep = 6;   //ȫ�����
				stu_KCMCUDownload.ucRepeatSendCount = 0; 
			}
		}
		else if(arr[0]==1)
		{
		}
		else if(arr[0]==2)
		{
		}
		break;
	default:
		break;
    }
}


#endif

//-----�ļ�McuChQiUpload.c����---------------------------------------------
