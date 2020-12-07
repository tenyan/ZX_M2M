/*
 * Copyright(c)2018, XXXXX��˾Ӳ���з���
 * All right reserved
 *
 * �ļ�����: McuUpload.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪMcuUpload����ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2018-02-7, by  lxf, �������ļ�
 *
 */
#include "config.h"
#include "IAP.h" 
//-----ͷ�ļ�����------------------------------------------------------------


//-----�ⲿ��������------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
extern stuFirmwareUpdate FirmwareUpdate;
//extern uint8 aMsgSendData[1024];
extern uint16 f_usUpLoadCmdSn;				//�����������ˮ��
extern STUKCMCUDownload stu_KCMCUDownload;

//-----�ڲ���������------------------------------------------------------------
STUMCUFirmware stu_McuFirmware;
STUExtMCUUpgrade stu_ExtMCUUpgrade;
uint8 ExtMCUUartSendBuff[1024] = {0};
//-----�ڲ���������------------------
/*****************************************************************************/

/******************************************************************************
** ��������: Crc16_CCITT У���㷨
** ��������: 
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2018-02-7
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Crc16_CCITT(uint8 const *str, uint16 len)
{
    uint8 m;
	uint16 j;
	uint16 check = 0;   //���� CRC ������2 �ֽڳ��ȣ�CRC У���ʼֵ����

	for(j=0;j<len;j++)  //��ҪУ����ֽ���
	{
        check = check^(uint16)(*str++ << 8); //����ǰ�ֽ��������� 8 λ��֮���� CRC У��ֵ�������
		
		for(m=0;m<8;m++)     //����Ĳ���ѭ�� 8 ��
		{
            if(check&0x8000)
		        check = (check<<1)^0x1021;   //�ж� check �����λ�����Ϊ 1�������� 1 λ��Ȼ���� 0x1021 ���
			else
				check = check<<1;    //������λΪ 0������ 1 λ
		}		
	}
	return check;  //����У��ֵ
}


/******************************************************************************
** ��������:  1)���жϽ���ƽ̨�°��Ƿ������ͨ��У�飬2)��ȡ�����ļ��ܰ���,3)�Ӵ洢���ж�ȡ����
** ��������: 
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2018-02-7
**-----------------------------------------------------------------------------
*******************************************************************************/
void McuUpload_CreatePackage(void)
{
    uint8 abuff[256];
	uint16 usPackets = 0;    //�����ܰ���
	uint8 ucnum,ucLastPacketlen=0;
//	uint16 uscheck;
	uint8 i=0;
	////////////////test////////////////
 //   FirmwareUpdate.usLastPacketLen = 288;
 //   FirmwareUpdate.usPackets = 0x9F;
 //   FirmwareUpdate.usLastPacketLen = 200;
 //   FirmwareUpdate.usPackets = 0x9E;
 
 //---T20_Test_V_Low_build150402
 //   FirmwareUpdate.usLastPacketLen = 396;
 //   FirmwareUpdate.usPackets = 0x2A;
 //---XE150GA-V1.02-2
 //   FirmwareUpdate.usLastPacketLen = 200;
 //   FirmwareUpdate.usPackets = 0x9E;
	////////////////////////////////////
//
//�ж������ļ��Ƿ���� 
    if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
		return;
//�ж������豸����
    ucnum = FirmwareUpdate.usLastPacketLen/256;
    ucLastPacketlen = FirmwareUpdate.usLastPacketLen%256;
    if(ucLastPacketlen)
    {
		ucnum = ucnum+1;	
    }
		
    usPackets = (FirmwareUpdate.usPackets-1)*4+ucnum;   //����ÿ��256�ֽ� ���ܰ�����
    stu_McuFirmware.usTotalPackets = usPackets;
    ReadFromFlash(FLASH_Firmware_MCU+stu_McuFirmware.usReadFlashSN,0,256,abuff);
    if((stu_McuFirmware.usReadFlashSN+1<stu_McuFirmware.usTotalPackets)||(ucLastPacketlen==0))
    {
		stu_McuFirmware.aSendbuff[0] = 'S';
		stu_McuFirmware.aSendbuff[1] = 'T';
		stu_McuFirmware.aSendbuff[2] = 0x07;
		stu_McuFirmware.aSendbuff[3] = 0x01;
		stu_McuFirmware.aSendbuff[4] = 0x10;
		stu_McuFirmware.aSendbuff[5] = usPackets&0xFF;
		stu_McuFirmware.aSendbuff[6] = (uint8)(usPackets>>8);
		stu_McuFirmware.aSendbuff[7] = (uint8)(stu_McuFirmware.usReadFlashSN&0xFF);
		stu_McuFirmware.aSendbuff[8] = (uint8)(stu_McuFirmware.usReadFlashSN>>8);
        memcpy(&stu_McuFirmware.aSendbuff[9],abuff,256);	
		stu_McuFirmware.aSendbuff[256+9] = 'E';
		stu_McuFirmware.aSendbuff[256+10] = 'D';
		stu_McuFirmware.uscheck = Crc16_CCITT(stu_McuFirmware.aSendbuff,267);
		stu_McuFirmware.aSendbuff[256+11] = (uint8)(stu_McuFirmware.uscheck&0xFF);
		stu_McuFirmware.aSendbuff[256+12] = (uint8)(stu_McuFirmware.uscheck>>8);
    }
	else if((stu_McuFirmware.usReadFlashSN+1==stu_McuFirmware.usTotalPackets)&&ucLastPacketlen) 
	{
		stu_McuFirmware.aSendbuff[0] = 'S';
		stu_McuFirmware.aSendbuff[1] = 'T';
		stu_McuFirmware.aSendbuff[2] = 0x07;
		stu_McuFirmware.aSendbuff[3] = 0x01;
		stu_McuFirmware.aSendbuff[4] = 0x10;
		stu_McuFirmware.aSendbuff[5] = usPackets&0xFF;
		stu_McuFirmware.aSendbuff[6] = (uint8)(usPackets>>8);
		stu_McuFirmware.aSendbuff[7] = (uint8)(stu_McuFirmware.usReadFlashSN&0xFF);
		stu_McuFirmware.aSendbuff[8] = (uint8)(stu_McuFirmware.usReadFlashSN>>8);
		
        memcpy(&stu_McuFirmware.aSendbuff[9],abuff,256);	
		for(i=0;i<(256-ucLastPacketlen);i++)
		{
            stu_McuFirmware.aSendbuff[9+ucLastPacketlen+i] = 0;
		}
		stu_McuFirmware.aSendbuff[256+9] = 'E';
		stu_McuFirmware.aSendbuff[256+10] = 'D';
		stu_McuFirmware.uscheck = Crc16_CCITT(stu_McuFirmware.aSendbuff,267);
		stu_McuFirmware.aSendbuff[256+11] = (uint8)(stu_McuFirmware.uscheck&0xFF);
		stu_McuFirmware.aSendbuff[256+12] = (uint8)(stu_McuFirmware.uscheck>>8);
	}
}
//��������������
void Mcu_BlockDownload_Init(void)
{
    uint8 abuff[8]={0xC6,0,0,0,0x0D,0x01,0,0};

//	stu_McuFirmware.usMcuUPdateTimerOut = Mcu_UPdateTimerOut;

    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //��׼֡
}
//��ֶ���������
void McuUploadSend(void)
{
    uint8 i;
    uint8 abuff[8];
	
//���Է��ͱ�־
    
    for(i=1;i<0x28;i++)
    {
        if(i==0x27)
			abuff[0] = 0xA7;
		else
            abuff[0] = i;
        memcpy(&abuff[1],&stu_McuFirmware.aSendbuff[(i-1)*7],7);
   //����CAN֡
        CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //��׼֡
        OSTimeDly(6);   //���Կ��Ǽӿ�Ƶ��
	}
//	OSTimeDly(1000*15);
}
//�����ؽ�������
void Mcu_BlockDownload_End(void)
{

    uint8 abuff[8]={0xD1,0,0,0,0,0,0,0};

    if(stu_McuFirmware.ucMcuRespflag==1)
    {
        stu_McuFirmware.ucMcuRespflag = 0;
		stu_McuFirmware.ucMcuCmdTimerOut = 15;	
		abuff[1] = (uint8)stu_McuFirmware.uscheck&0xFF;
		abuff[2] = (uint8)(stu_McuFirmware.uscheck>>8)&0xFF;
        CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //��׼֡
        stu_McuFirmware.ucMcuRespflag = 0;
		stu_McuFirmware.ucMcuCmdTimerOut = 15;			
	}
}

#if 0
//֪ͨMCU�������� ������
void Mcu_Restart_Upgrade1(void)
{
    uint8 abuff[8]={0xc6,0,0,0,0x0d,0,0,0};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //��׼֡
  //  stu_McuFirmware.ucLoadStep = 6;
}

//֪ͨMCU�������� ����������
void Mcu_Restart_Upgrade2(void)
{
    uint8 abuff1[8]={0x01,0x53,0x54,0x07,0x00,0x11,0x55,0xaa};
    uint8 abuff2[8]={0x82,0x12,0x34,0x45,0x44,0x96,0x56,0x01};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff1);   //��׼֡
    OSTimeDly(6);
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff2);   //��׼֡
	//stu_McuFirmware.ucLoadStep = 6;
}

//֪ͨMCU�������� ���������ؽ���
void Mcu_Restart_Upgrade3(void)
{
    uint8 abuff[8]={0xc5,0x96,0x56,0,0,0,0,0};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //��׼֡

	stu_McuFirmware.ucLoadStep = 0xFF;
}
#endif
/******************************************************************************
** ��������:  McuFirmwareTimerOut
** ��������:  �Կ����������Ǳ��������ʱ ��ʱ����
** 
** ��    ��: 
** ��    ��: 
** ��    ��: 
**
** ��    ��: Lxf
** ��    ��: 2018-02-7
**-----------------------------------------------------------------------------
*******************************************************************************/
void McuFirmwareTimerOut(void)
{
    if(stu_McuFirmware.ucMcuCmdTimerOut&&stu_McuFirmware.ucLoadStep)
    {
		stu_McuFirmware.ucMcuCmdTimerOut--;
		if(!stu_McuFirmware.ucMcuCmdTimerOut)
		{
            stu_McuFirmware.ucMcuRespflag = 1;
		}
	}	
	
	if(stu_McuFirmware.usMcuUPdateTimerOut)
	{
		stu_McuFirmware.usMcuUPdateTimerOut--;
		if(!stu_McuFirmware.usMcuUPdateTimerOut)
		{
    	//	stu_McuFirmware.ucRcvPackflag = 0;
			if(stu_McuFirmware.ucUploadResult==0&&stu_McuFirmware.ucLoadStep!=7)
			{
                stu_McuFirmware.ucLoadStep = 7;
				stu_McuFirmware.ucUploadResult = 0;
			}
			else
			{
                stu_McuFirmware.ucLoadStep = 0;
			}
		}			
	}
}

#if 0
/******************************************************************************
** ��������: SYS_SendUpgradeRet
** ��������: ��ƽ̨�ϱ��������
** ��    ��: 
** ��    ��: ��
** ��    ��: 
** ��    ��: hhm
** ��    ��: 2016-9-19
******************************************************************************/
void Mcu_SendUpgradeResult(void)
{
	uint8 *p = &aMsgSendData[MSG_HEAD_LEN];
	uint16 usTemp;

	if(!g_stuSystem.ucOnline)
		return;
	
	*p++ = 0;
	*p++ = 2;
	*p++ = 'U';
	*p++ = 'R';
	if(stu_McuFirmware.ucUploadResult==0)		
	    *p++ = 4;
	else if(stu_McuFirmware.ucUploadResult==1)
		*p++ = 2;		

	usTemp = BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, 5, 0, f_usUpLoadCmdSn);
	usTemp += 5;
	aMsgSendData[usTemp] = SumCalc(aMsgSendData, usTemp);
//	GSM_SendGprs(aMsgSendData, usTemp+1, 0);
}
#endif

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


//��MCU���͹̼���������
void Mcu_FirmwareDownload_Function(void)
{
  //��MCU�����������ݵĲ���
/*
	if(1!=GetCanRcvState(CAN_CHANNEL1))  //�ж�CAN ͨѶ״̬
		return;
	*/
	if(!GetAccState())
		return;
  
    switch(stu_McuFirmware.ucLoadStep)
    {
        case 1:    //���Ϳ�������MCU
			Mcu_BlockDownload_Init();
			break;
		case 2:    //���Ϳ����ݰ�
		//    if(stu_McuFirmware.ucMcuRespflag==1)
	//	    {
        		McuUpload_CreatePackage();
        		McuUploadSend();	
			//	stu_McuFirmware.ucMcuRespflag = 0;
			//	stu_McuFirmware.ucMcuCmdTimerOut = 15;
		//	}
			break;
		case 3:    //���Ϳ����
		    Mcu_BlockDownload_End();
			break;
			#if 0
		case 4:    // �����������������
		    Mcu_Restart_Upgrade1();
			break;
		case 5:     //�����������������
		    Mcu_Restart_Upgrade2();
			break;
		case 6:     //����������������ؽ���
		    Mcu_Restart_Upgrade3();
			break; 
			#endif
		case 7:     //֪ͨƽ̨MCU�����������  �������ӳ�2��Ƚ��            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
		default:
		    break;
	}
}


/***********************************************************************
** ��������: Ext_Mcu_UpgradeNotice_RecVProcess
** ��������: ����Э������Ӧ��������������
**
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ                  
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
             
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��09��7��

************************************************************************/
void Ext_Mcu_UpgradeNotice_RecVProcess(uint8* ptr, uint16 uslen)
{
    uint8 buff[50] = {0};
	static uint8 ucSN = 0;
    uint16 usSendlen = 0;
	
	ucSN++;	
	buff[0] = 0x01;    //�����ʶ
	buff[1] = 0;       //�ɹ�
	buff[2] = 0x04;
	buff[3] = 0x00;    
	usSendlen = 4;
	if(uslen<17)
		buff[1] = 1;   //��ʽ�쳣
		
	FirmwareUpdate.uctype = *ptr++;   //��������
	FirmwareUpdate.ucdev = *ptr++;    //����Ŀ���豸
	if(FirmwareUpdate.ucdev>3)
		buff[1] = 2;   //�豸��֧��
		
	FirmwareUpdate.ucSWNameLen = *ptr++;
	memcpy(FirmwareUpdate.aucSWName, ptr, FirmwareUpdate.ucSWNameLen);
	ptr += FirmwareUpdate.ucSWNameLen;
	FirmwareUpdate.ucSWVersionLen = *ptr++;
	memcpy(FirmwareUpdate.aucSWVersion, ptr, FirmwareUpdate.ucSWVersionLen);
	ptr += FirmwareUpdate.ucSWVersionLen;
    FirmwareUpdate.uiSWSize = (*ptr<<24) + (*(ptr+1)<<16)+ (*(ptr+2)<<8)+ *(ptr+3);
    ptr += 4;
	FirmwareUpdate.usPackets = (*ptr<<8) + (*(ptr+1));
    ptr += 2;
    FirmwareUpdate.uiCrc = (*ptr<<24) + (*(ptr+1)<<16)+ (*(ptr+2)<<8)+ *(ptr+3);

	Ext_McuUart_DataSend(0x04, usSendlen, buff, ucSN);
	printf("usPackets = %d\n",FirmwareUpdate.usPackets);
	
    if(buff[1]!=0)
		stu_ExtMCUUpgrade.ucUpgradeStep = 0;//�����˳�
	else
	{
		stu_ExtMCUUpgrade.ucUpgradeStep = 2;
		stu_ExtMCUUpgrade.usTimeOut = 600;
	}
	FirmwareUpdate.usRequestPacketSn = 0;
	stu_ExtMCUUpgrade.ucTimer = 0;
    stu_ExtMCUUpgrade.ucRepeats = 0;	
}


/***********************************************************************
** ��������: Ext_Mcu_Upgradepackage_DownloadProcess
** ��������: ���յ�����������
**
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ                  
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
             
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��09��11��

************************************************************************/
#if 0
uint8 Ext_Mcu_Upgradepackage_DownloadProcess(uint8* p, uint16 uslen)
{

	uint16 usPackSn, usTotalPacks, usPackLen;
	uint32 uiTemp;

	if(uslen<6)
		stu_ExtMCUUpgrade.ucUpgradeStep = 0; //�����˳�


	usPackSn = (*p<<8) + *(p+1);
	p += 2; 
	usTotalPacks = (*p<<8) + *(p+1);
	p += 2;
	usPackLen = (*p<<8) + *(p+1);
	p += 2;
	
	if(usPackSn != FirmwareUpdate.usRequestPacketSn)
		return 0;
	if(usTotalPacks!= FirmwareUpdate.usPackets)
		return 0;
	if(usPackLen == 0)  //?
		return 0;

	if(FirmwareUpdate.ucdev==3)
	    WriteToFlash(FLASH_PAGEADDR_UPGRADE +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
	else
	    WriteToFlash(FLASH_Firmware_MCU +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
    
	FirmwareUpdate.usRequestPacketSn++;
	if(FirmwareUpdate.usRequestPacketSn==FirmwareUpdate.usPackets)//�������
	{
		FirmwareUpdate.usLastPacketLen = usPackLen;
		uiTemp = GetCrc32();
		if(uiTemp==FirmwareUpdate.uiCrc)
		{
		    stu_ExtMCUUpgrade.ucResult = 0;
		    if(FirmwareUpdate.ucdev!=3)
		    {    			
    			stu_McuFirmware.ucRcvPackflag = 1;
    			stu_McuFirmware.ucLoadStep = 1;
    			stu_McuFirmware.usReadFlashSN = 0;			
			  //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
			    stu_McuFirmware.usMcuUPdateTimerOut = 600;   //10����
			    PC_SendDebugData((uint8 *)("MCULoad OK"), 10, DEBUG_ANYDATA);
			}
			else
			    PC_SendDebugData((uint8 *)("ST Load OK"), 10, DEBUG_ANYDATA);
		}
		else
		{
			stu_ExtMCUUpgrade.ucResult = 1;
			stu_McuFirmware.ucRcvPackflag = 0;
            stu_McuFirmware.ucLoadStep = 0;
			//stu_ExtMCUUpgrade.ucUpgradeStep = 3;
			PC_SendDebugData((uint8 *)("FM CRC ERR"), 10, DEBUG_ANYDATA);
		}
		stu_ExtMCUUpgrade.ucUpgradeStep = 3;
		FirmwareUpdate.usRequestPacketSn = 0;
        stu_ExtMCUUpgrade.ucTimer = 1;
		stu_ExtMCUUpgrade.ucRepeats = 0;
		return 0;
	}
    stu_ExtMCUUpgrade.ucUpgradeStep = 2;
    stu_ExtMCUUpgrade.ucTimer = 0;
	stu_ExtMCUUpgrade.ucRepeats = 0;
	return 0;
}
#endif

uint8 Ext_Mcu_Upgradepackage_DownloadProcess(uint8* p, uint16 uslen)
{

	uint16 usPackSn, usTotalPacks, usPackLen;
	uint32 uiTemp;

	if(uslen<6)
		stu_ExtMCUUpgrade.ucUpgradeStep = 0; //�����˳�


	usPackSn = (*p<<8) + *(p+1);
	p += 2; 
	usTotalPacks = (*p<<8) + *(p+1);
	p += 2;
	usPackLen = (*p<<8) + *(p+1);
	p += 2;
	
	if(usPackSn != FirmwareUpdate.usRequestPacketSn)
		return 0;
	if(usTotalPacks!= FirmwareUpdate.usPackets)
		return 0;
	if(usPackLen == 0)  //?
		return 0;

	if(FirmwareUpdate.ucdev==3)
	    WriteToFlash(FLASH_PAGEADDR_UPGRADE +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
	else
	    WriteToFlash(FLASH_Firmware_MCU +(usPackSn*(FIRMWARE_PACKET_LEN/256)), 0, usPackLen, p);
    
	FirmwareUpdate.usRequestPacketSn++;
	if(FirmwareUpdate.usRequestPacketSn==FirmwareUpdate.usPackets)//�������
	{
		FirmwareUpdate.usLastPacketLen = usPackLen;
		uiTemp = GetCrc32();
		if(uiTemp==FirmwareUpdate.uiCrc)
		{
		    stu_ExtMCUUpgrade.ucResult = 0;
		    if(FirmwareUpdate.ucdev==1)
		    {    			
    			stu_McuFirmware.ucRcvPackflag = 1;
    			stu_McuFirmware.ucLoadStep = 1;
    			stu_McuFirmware.usReadFlashSN = 0;			
			  //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
			    stu_McuFirmware.usMcuUPdateTimerOut = 600;   //10����
			    PC_SendDebugData((uint8 *)("MCULoad OK"), 10, DEBUG_ANYDATA);
			}
			else if(FirmwareUpdate.ucdev==4)
			{
    			stu_KCMCUDownload.ucRcvPackflag = 1;
    			stu_KCMCUDownload.ucLoadStep = 0xFF;
			  //  stu_McuFirmware.usMcuUPdateTimerOut = FirmwareUpdate.usPackets*2+30;
			    stu_KCMCUDownload.usMcuUPdateTimerOut = 1200;   //20����
			    PC_SendDebugData((uint8 *)("KCMCULoad OK"), 10, DEBUG_ANYDATA);
			
			}
			else
			    PC_SendDebugData((uint8 *)("ST Load OK"), 10, DEBUG_ANYDATA);
		}
		else
		{
			stu_ExtMCUUpgrade.ucResult = 1;
			stu_McuFirmware.ucRcvPackflag = 0;
            stu_McuFirmware.ucLoadStep = 0;
			//stu_ExtMCUUpgrade.ucUpgradeStep = 3;
			PC_SendDebugData((uint8 *)("FM CRC ERR"), 10, DEBUG_ANYDATA);
		}
		stu_ExtMCUUpgrade.ucUpgradeStep = 3;
		FirmwareUpdate.usRequestPacketSn = 0;
        stu_ExtMCUUpgrade.ucTimer = 1;
		stu_ExtMCUUpgrade.ucRepeats = 0;
		return 0;
	}
    stu_ExtMCUUpgrade.ucUpgradeStep = 2;
    stu_ExtMCUUpgrade.ucTimer = 0;
	stu_ExtMCUUpgrade.ucRepeats = 0;
	return 0;
}
/***********************************************************************
** ��������: Ext_Mcu_Upgradepackage_Request
** ��������: ����Э������Ӧ��������������
**
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ                  
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
             
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��09��11��

************************************************************************/
void Ext_Mcu_UpgradeRequest_RecVProcess(uint8* ptr, uint16 uslen)
{
    
    stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //�����˳�
    if(stu_McuFirmware.ucLoadStep == 7)
    {
        stu_McuFirmware.ucLoadStep = 0;
    	stu_McuFirmware.ucRcvPackflag = 0;
    }
	if(FirmwareUpdate.ucdev!=3||(memcmp(FirmwareUpdate.aucSWName,"AUX_MCU",7)!=0))
		return;
    CPU_IntDis();
    IAP_Start();            /*Enable interrupt and reset system    */   
    CPU_IntEn();	
}

void Ext_McuUart_DataSend(uint8 ucCommand, uint16 uslen, uint8* pdata,uint8 ucSN)
{
   // uint8 SendBuff[1200] = {0};
    uint8 *SendBuff;

    SendBuff = &ExtMCUUartSendBuff[0];
	
	SendBuff[0] = 0x7E;
	SendBuff[1] = (uint8)((uslen+3)>>8);
	SendBuff[2] = (uint8)((uslen+3)&0xFF);
	SendBuff[3] = ucCommand;            //Command	
		
	SendBuff[4] = ucSN;
	memcpy(&SendBuff[5],pdata,uslen);

	SendBuff[5+uslen] = SumCalc(&SendBuff[3],uslen+2);
	
	SendBuff[6+uslen] = 0x0D;
	SendBuff[7+uslen] = 0x0A;
    RS232_UART_Write(SendBuff, 8+uslen);	
}

void Ext_McuUartMessage(uint8 *ptr, uint16 uslen)
{
    uint8 ucSumNum = 0;
	uint16 usDatalen = 0;
	uint8 ucCommand = 0, ucSubCommand = 0;
	uint8 ucSN,ucResult;

    if((ptr[0]==0x7E)&&(ptr[uslen-2]==0x0D)&&(ptr[uslen-1]==0x0A)&&uslen>8)
    {
    
    	usDatalen = (uint16)(ptr[1]<<8) + ptr[2];
        ucCommand = ptr[3];
    	ucSN = ptr[4];
		ucSubCommand = ptr[5];    //���豸��
    	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    	if(ucSumNum!=ptr[uslen-3])
    		return;
    	if(ucCommand==0x04)
    	{
            switch(ucSubCommand)
            {
                case 1:           //�յ����İ巢�͵�����֪ͨ
                    Ext_Mcu_UpgradeNotice_RecVProcess(&ptr[6], usDatalen-3);
					break;
				case 2:           //�յ����İ巢�͵�����������
				    Ext_Mcu_Upgradepackage_DownloadProcess(&ptr[6], usDatalen-3);
					break; 
				case 3:           //�յ����İ巢�͵��������Ӧ��
				    Ext_Mcu_UpgradeRequest_RecVProcess(&ptr[6], usDatalen-3);
					break;
				default:
					break;
			}
		}
      /*  else if(ucCommand==0x83)      //MCU�Ժ��İ巢�������Ӧ��
    	{
            ucResult = ptr[5];       // 0-�ɹ���1-�쳣�� ʧ�ܿ��Կ����ط�
    	}	
		*/
    }
	else   //����͸������ ���ó���
	{

	}
}

void Ext_Mcu_Upgradepackage_Request(void)
{
    uint8 buff[50] = {0};
	static uint8 ucSN = 0;
    uint16 uslen = 0;
	
	ucSN++;
	buff[0] = 0x02;    //�����ʶ
	buff[1] = (uint8)(FirmwareUpdate.usRequestPacketSn>>8);   //��ǰ���������
	buff[2] = (uint8)(FirmwareUpdate.usRequestPacketSn&0xFF); //
    uslen = 3;
	stu_ExtMCUUpgrade.ucRepeats++;	
	
	Ext_McuUart_DataSend(0x04, uslen, buff, ucSN);
}

void Mcu_SendUpgradeResult_Send(void)
{
    uint8 buff[50] = {0};
	static uint8 ucSN = 0;
    uint16 uslen = 0;
	
	ucSN++;
	buff[0] = 0x03;                    //�����ʶ
	buff[1] = FirmwareUpdate.ucdev;;   //��ǰ����Ŀ���豸
	buff[2] = stu_ExtMCUUpgrade.ucResult;    //�������
    uslen = 3;
	stu_ExtMCUUpgrade.ucRepeats++;	
	Ext_McuUart_DataSend(0x04, uslen, buff, ucSN);	
}

//10msѭ��
void Ext_Mcu_Upgrade_Function(void)
{
//	if(!GetAccState())         //���������������������Ҫ�ж�ACC
//		return;
    //ʱ��ѭ��100ms
    if(stu_ExtMCUUpgrade.ucUpgradeStep==0)
		return ;
    if(stu_ExtMCUUpgrade.ucTimer)
    {
        stu_ExtMCUUpgrade.ucTimer--;
		return ;
    }
	else
		stu_ExtMCUUpgrade.ucTimer = 10;
	
    switch(stu_ExtMCUUpgrade.ucUpgradeStep)
    {
        case 0:
			break;
        case 1:         //֪ͨ�豸��ʼ����

			break;
		case 2:         //Ӧ�����������
		    Ext_Mcu_Upgradepackage_Request();
            stu_ExtMCUUpgrade.ucTimer = 300;   //3����ط�
			break;
		case 3:         //��������ϱ�����
			Mcu_SendUpgradeResult_Send();
			stu_ExtMCUUpgrade.ucTimer = 300;
			break;
		case 0xFF:      //�м䲽��
			break;
		default:
			break;
	}
}

void Ext_Mcu_UpgradeTimer(void)
{
    if(stu_ExtMCUUpgrade.ucRepeats>3)
    {
        stu_ExtMCUUpgrade.ucRepeats = 0;
		if(stu_ExtMCUUpgrade.ucUpgradeStep < 3)	
		{
		    stu_ExtMCUUpgrade.ucUpgradeStep = 3;
			stu_ExtMCUUpgrade.ucResult = 3;
		}
		else
			stu_ExtMCUUpgrade.ucUpgradeStep = 0;
		//��Ҫ���� �������������
	}	
	
	if(stu_ExtMCUUpgrade.usTimeOut)
	{
	    stu_ExtMCUUpgrade.usTimeOut--;
        if(!stu_ExtMCUUpgrade.usTimeOut)
        {
			if(stu_ExtMCUUpgrade.ucResult!=0&&stu_ExtMCUUpgrade.ucUpgradeStep!=3)
			{
                stu_ExtMCUUpgrade.ucUpgradeStep = 3;
				stu_ExtMCUUpgrade.ucResult = 3;
			}
			else
			{
                stu_ExtMCUUpgrade.ucUpgradeStep = 0;
			}			
		}
	}
	Ext_Mcu_Upgrade_Function();
}


//-----�ļ�McuUpload.c����---------------------------------------------
