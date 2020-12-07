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
//-----ͷ�ļ�����------------------------------------------------------------


//-----�ⲿ��������------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
extern stuFirmwareUpdate FirmwareUpdate;
extern uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
extern uint16 f_usUpLoadCmdSn;				//�����������ˮ��

//-----�ڲ���������------------------------------------------------------------
STUMCUFirmware stu_McuFirmware;
STUExtMCUUpgrade stu_ExtMCUUpgrade;
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
#if 0
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
#endif
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
        OSTimeDly(6);
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
	*p++ = stu_ExtMCUUpgrade.ucResult;
/*	
	if(stu_McuFirmware.ucUploadResult==0)		
	    *p++ = 4;
	else if(stu_McuFirmware.ucUploadResult==1)
		*p++ = 2;		
*/
    f_usUpLoadCmdSn++;
	usTemp = BuildMsgHead(aMsgSendData, MSG_TYPE_UPDATE, 5, 0, f_usUpLoadCmdSn);
	usTemp += 5;
	aMsgSendData[usTemp] = SumCalc(aMsgSendData, usTemp);
	GSM_SendGprs(aMsgSendData, usTemp+1, 0);
}
#if 0
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
		case 4:    // �����������������
		    Mcu_Restart_Upgrade1();
			break;
		case 5:     //�����������������
		    Mcu_Restart_Upgrade2();
			break;
		case 6:     //����������������ؽ���
		    Mcu_Restart_Upgrade3();
			break;  
		case 7:     //֪ͨƽ̨MCU�����������  �������ӳ�2��Ƚ��            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
		default:
		    break;
	}
}
#endif

#if 0
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
#endif


/*---------------------��Э�����������ⲿ��������������------------------*/
//֪ͨЭ�����������ⲿ�������豸����

void Ext_Mcu_UpgradeNotice(void)
{
    uint8 buff[100] = {0};
	static uint8 ucSN = 0;
    uint8 uclen = 0;
	
	ucSN++;
	buff[0] = 0x01;    //�����ʶ
	buff[1] = FirmwareUpdate.uctype;   //��������
	buff[2] = FirmwareUpdate.ucdev;//����Ŀ���豸
	buff[3] = FirmwareUpdate.ucSWNameLen;
	memcpy(&buff[4],FirmwareUpdate.aucSWName,FirmwareUpdate.ucSWNameLen);
	uclen = FirmwareUpdate.ucSWNameLen;
	buff[4+uclen] = FirmwareUpdate.ucSWVersionLen;
	memcpy(&buff[5+uclen],FirmwareUpdate.aucSWVersion,FirmwareUpdate.ucSWVersionLen);
	uclen += FirmwareUpdate.ucSWVersionLen;
	buff[5+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>24);
	buff[6+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>16);
	buff[7+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>8);
	buff[8+uclen] = (uint8)(FirmwareUpdate.uiSWSize&0xFF);
	buff[9+uclen] = (uint8)(FirmwareUpdate.usPackets>>8);
	buff[10+uclen] = (uint8)(FirmwareUpdate.usPackets&0xFF);
    buff[11+uclen] = (uint8)(FirmwareUpdate.uiCrc>>24);
    buff[12+uclen] = (uint8)(FirmwareUpdate.uiCrc>>16);
    buff[13+uclen] = (uint8)(FirmwareUpdate.uiCrc>>8);
    buff[14+uclen] = (uint8)(FirmwareUpdate.uiCrc&0xFF);
	
    A5Uart0_DataSend(0x04, 15+uclen, buff, ucSN);
}

#if 0
//7E002104010101030E4155585F4D4355574A5F5630303101320001021800419CF24BB9520D0A
void Ext_Mcu_UpgradeNotice(void)
{
    uint8 buff[100] = {0x01,0x01,0x03,0x0E,0x41,0x55,0x58,0x5F,0x4D,0x43,0x55,
		0x57,0x4A,0x5F,0x56,0x30,0x30,0x31,0x01,0x32,0x00,0x01,0x02,0x18,0x00,0x41,0x9C,0xF2,0x4B,0xB9};
	static uint8 ucSN = 0;
    uint8 uclen = 0;
	
	ucSN++;
    
	FirmwareUpdate.uctype = buff[1];
	FirmwareUpdate.ucdev = buff[2];
	FirmwareUpdate.ucSWNameLen = buff[3];
	memcpy(FirmwareUpdate.aucSWName,&buff[4],FirmwareUpdate.ucSWNameLen);
	uclen = FirmwareUpdate.ucSWNameLen;
	FirmwareUpdate.ucSWVersionLen = buff[4+uclen];
	memcpy(FirmwareUpdate.aucSWVersion,&buff[5+uclen],FirmwareUpdate.ucSWVersionLen);
	uclen += FirmwareUpdate.ucSWVersionLen;
    FirmwareUpdate.uiSWSize = 0x00010218;
	
	FirmwareUpdate.usPackets = 0x0041;
	FirmwareUpdate.uiCrc = 0x9CF24BB9;
	/*
	buff[0] = 0x01;    //�����ʶ
	buff[1] = FirmwareUpdate.uctype;   //��������
	buff[2] = FirmwareUpdate.ucdev;//����Ŀ���豸
	buff[3] = FirmwareUpdate.ucSWNameLen;
	memcpy(&buff[4],FirmwareUpdate.aucSWName,FirmwareUpdate.ucSWNameLen);
	uclen = FirmwareUpdate.ucSWNameLen;
	buff[4+uclen] = FirmwareUpdate.ucSWVersionLen;
	memcpy(&buff[5+uclen],FirmwareUpdate.aucSWVersion,FirmwareUpdate.ucSWVersionLen);
	uclen += FirmwareUpdate.ucSWVersionLen;
	buff[5+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>24);
	buff[6+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>16);
	buff[7+uclen] = (uint8)(FirmwareUpdate.uiSWSize>>8);
	buff[8+uclen] = (uint8)(FirmwareUpdate.uiSWSize&0xFF);
	buff[9+uclen] = (uint8)(FirmwareUpdate.usPackets>>8);
	buff[10+uclen] = (uint8)(FirmwareUpdate.usPackets&0xFF);
    buff[11+uclen] = (uint8)(FirmwareUpdate.uiCrc>>24);
    buff[12+uclen] = (uint8)(FirmwareUpdate.uiCrc>>16);
    buff[13+uclen] = (uint8)(FirmwareUpdate.uiCrc>>8);
    buff[14+uclen] = (uint8)(FirmwareUpdate.uiCrc&0xFF);
	*/
	uclen = 30;
    A5Uart0_DataSend(0x04, uclen, buff, ucSN);
}
#endif
//��MCU�������������� (��ʼ��ȡ�ļ�����ǰȷ���ļ��Ѿ��ر�״̬) 
//�����ظ���ȡ�Ŀ���? 
void Ext_Mcu_Upgradepackage_send(void)
{
    uint8 buff[1200] = {0};
	static uint8 ucSN = 0;
    uint16 uslen = 0;
	
	ucSN++;
	buff[0] = 0x02;    //�����ʶ
	buff[1] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn>>8);   //��ǰ���������
	buff[2] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn&0xFF); //
    buff[3] = (uint8)(FirmwareUpdate.usPackets>>8);           //�����ܰ���
    buff[4] = (uint8)(FirmwareUpdate.usPackets&0xFF);
	/*
	if(FirmwareUpdate.usRequestPacketSn==0)
	{
     //�ر������ļ�
	}
	*/
	if(stu_ExtMCUUpgrade.usRequestPacketSn==FirmwareUpdate.usPackets-1)
        uslen = FirmwareUpdate.usLastPacketLen;
	else
		uslen = 1024;
	buff[5] = (uint8)(uslen>>8);
	buff[6] = (uint8)(uslen&0xFF);
  // ReadFromWholFileFlash(FLASH_FIRMWARE_Auxi_MCU, FIRMWARE_PACKET_LEN, &buff[7]);
    if(FirmwareUpdate.ucdev==3)  //Э������
	    ReadFromlseekFlash(FLASH_FIRMWARE_Auxi_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]); 
	else if(FirmwareUpdate.ucdev==1)   //����������
	    ReadFromlseekFlash(FLASH_FIRMWARE_Vehicle_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]); 
		
    A5Uart0_DataSend(0x04, 7+uslen, buff, ucSN);
}

//�����������
void Ext_Mcu_UpgradeResults_Ack(void)
{
    uint8 buff[10] = {0};
	static uint8 ucSN = 0;
	//��Ҫ��ƽ̨���ͽ������
	ucSN++;
	buff[0] = 0x03;    //�����ʶ
	buff[1] = stu_ExtMCUUpgrade.ucDev;       //��ǰ���������
	buff[2] = stu_ExtMCUUpgrade.ucResult;    //�������
	 
    A5Uart0_DataSend(0x04, 3, buff, ucSN);
}

void Ext_Mcu_Upgrade_Function(void)
{

//	if(!GetAccState())   //���������������������Ҫ�ж�ACC
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
            Ext_Mcu_UpgradeNotice();
			stu_ExtMCUUpgrade.ucTimer = 100;
			break;
		case 2:         //Ӧ�����������
		    Ext_Mcu_Upgradepackage_send();
			stu_ExtMCUUpgrade.ucUpgradeStep = 0xFF;  //���͹��������� �ȴ��ⲿMCU������һ��
			break;
		case 3:         //��������ϱ�����
		  //  Ext_Mcu_UpgradeResults_Ack();
			Mcu_SendUpgradeResult();
			stu_ExtMCUUpgrade.ucTimer = 300;
			//stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //���������ͽ���
			break;
		case 0xFF:     //�м䲽��
			break;
		default:
			break;
	}
}
/***********************************************************************
** ��������: Ext_Mcu_UpgradeNotice_Resp
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
void Ext_Mcu_UpgradeNotice_Resp(uint8* ptr, uint16 uslen)
{
    if(ptr[0]!=0)
		stu_ExtMCUUpgrade.ucUpgradeStep = 0;     //��������
	else
		stu_ExtMCUUpgrade.ucUpgradeStep = 0xFF;  //�ȴ�Э��������һ����

	//printf("step= %x",stu_ExtMCUUpgrade.ucUpgradeStep);
}

/***********************************************************************
** ��������: Ext_Mcu_Upgradepackage_Request
** ��������: ����Э��������������������
**
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ                  
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
             
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��09��7��

************************************************************************/
void Ext_Mcu_Upgradepackage_Request(uint8* ptr, uint16 uslen)
{
    uint16 usRequestPacketSn = 0;     //�������������

	usRequestPacketSn = (uint16)(ptr[0]<<8) + ptr[1];
	stu_ExtMCUUpgrade.usRequestPacketSn = usRequestPacketSn;  //�ȴ�Э��������һ����
    stu_ExtMCUUpgrade.ucUpgradeStep = 2;
	stu_ExtMCUUpgrade.ucTimer = 0;   //����Ӧ�� ��������0
	
}

/***********************************************************************
** ��������: Ext_Mcu_UpgradepackageResult_Request
** ��������: ����Э���������������������
**
** ��    ��: ptr,    ͨѶЭ���������ʶ(0x01)��һ���ֽ�ָ���ַ                  
             uslen�� Э���������峤��-1(ȥ�������ʶ1�ֽڳ���)
             
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��09��7��

************************************************************************/
void Ext_Mcu_UpgradepackageResult_Request(uint8* ptr, uint16 uslen)
{
    uint8 ucDev = 0;
	uint8 ucResult = 0;
	stu_ExtMCUUpgrade.ucDev = ptr[0]; 
	stu_ExtMCUUpgrade.ucResult = ptr[1];
	stu_ExtMCUUpgrade.ucUpgradeStep = 3;
	stu_ExtMCUUpgrade.ucTimer = 0;   //����Ӧ�� ��������0
	Ext_Mcu_UpgradeResults_Ack();
}

void Ext_Mcu_RecvUartData(uint8* ptr, uint16 uslen)
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
                case 1:           //�յ�Э����������Ӧ����
                    Ext_Mcu_UpgradeNotice_Resp(&ptr[6], usDatalen-3);
					break;
				case 2:           //�յ�Э��������������������
				    Ext_Mcu_Upgradepackage_Request(&ptr[6], usDatalen-3);
					break; 
				case 3:           //�յ�Э�����������������
				    Ext_Mcu_UpgradepackageResult_Request(&ptr[6], usDatalen-3);
					break;
				default:
					break;
			}
		}
        else if(ucCommand==0x83)      //MCU�Ժ��İ巢�������Ӧ��
    	{
            ucResult = ptr[5];   // 0-�ɹ���1-�쳣�� ʧ�ܿ��Կ����ط�
    	}	
    }
	else   //����͸������ ���ó���
	{

	}
}
//ÿ��ִ��һ��
void Ext_Mcu_Timeout_Function(void)
{
    if(stu_ExtMCUUpgrade.usTimeOut)
    {
        stu_ExtMCUUpgrade.usTimeOut--;
        if(!stu_ExtMCUUpgrade.usTimeOut)
        {
            stu_ExtMCUUpgrade.ucUpgradeStep = 3; 
            if(FirmwareUpdate.ucdev==3)         //Э������
				stu_ExtMCUUpgrade.ucResult = 1;
			else if(FirmwareUpdate.ucdev==1)    //�ⲿ������
				stu_ExtMCUUpgrade.ucResult = 4;
			else
                stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //������� �˳�����
		}
	}
}



//-----�ļ�McuUpload.c����---------------------------------------------
