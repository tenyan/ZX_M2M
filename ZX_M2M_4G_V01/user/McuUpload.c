/*
 * Copyright(c)2018, XXXXX公司硬件研发部
 * All right reserved
 *
 * 文件名称: McuUpload.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为McuUpload功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2018-02-7, by  lxf, 创建本文件
 *
 */
#include "config.h"
//-----头文件调用------------------------------------------------------------


//-----外部变量定义------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
extern stuFirmwareUpdate FirmwareUpdate;
extern uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
extern uint16 f_usUpLoadCmdSn;				//上行命令的流水号

//-----内部变量定义------------------------------------------------------------
STUMCUFirmware stu_McuFirmware;
STUExtMCUUpgrade stu_ExtMCUUpgrade;
//-----内部函数声明------------------
/*****************************************************************************/

/******************************************************************************
** 函数名称: Crc16_CCITT 校验算法
** 功能描述: 
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2018-02-7
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 Crc16_CCITT(uint8 const *str, uint16 len)
{
    uint8 m;
	uint16 j;
	uint16 check = 0;   //定义 CRC 变量，2 字节长度；CRC 校验初始值清零

	for(j=0;j<len;j++)  //需要校验的字节数
	{
        check = check^(uint16)(*str++ << 8); //将当前字节内容左移 8 位，之后与 CRC 校验值进行异或
		
		for(m=0;m<8;m++)     //下面的操作循环 8 次
		{
            if(check&0x8000)
		        check = (check<<1)^0x1021;   //判断 check 的最高位，如果为 1，先左移 1 位，然后与 0x1021 异或
			else
				check = check<<1;    //如果最高位为 0，左移 1 位
		}		
	}
	return check;  //返回校验值
}


/******************************************************************************
** 函数名称:  1)先判断接受平台下包是否结束并通过校验，2)获取升级文件总包数,3)从存储器中读取数据
** 功能描述: 
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2018-02-7
**-----------------------------------------------------------------------------
*******************************************************************************/
void McuUpload_CreatePackage(void)
{
#if 0
    uint8 abuff[256];
	uint16 usPackets = 0;    //升级总包数
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
//判断升级文件是否可用 
    if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
		return;
//判断升级设备类型
    ucnum = FirmwareUpdate.usLastPacketLen/256;
    ucLastPacketlen = FirmwareUpdate.usLastPacketLen%256;
    if(ucLastPacketlen)
    {
		ucnum = ucnum+1;	
    }
		
    usPackets = (FirmwareUpdate.usPackets-1)*4+ucnum;   //按照每包256字节 的总包数据
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
//启动块下载命令
void Mcu_BlockDownload_Init(void)
{
    uint8 abuff[8]={0xC6,0,0,0,0x0D,0x01,0,0};

//	stu_McuFirmware.usMcuUPdateTimerOut = Mcu_UPdateTimerOut;

    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //标准帧
}
//块分段下载命令
void McuUploadSend(void)
{
    uint8 i;
    uint8 abuff[8];
	
//可以发送标志
    
    for(i=1;i<0x28;i++)
    {
        if(i==0x27)
			abuff[0] = 0xA7;
		else
            abuff[0] = i;
        memcpy(&abuff[1],&stu_McuFirmware.aSendbuff[(i-1)*7],7);
   //发送CAN帧
        CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //标准帧
        OSTimeDly(6);
	}
//	OSTimeDly(1000*15);
}
//块下载结束命令
void Mcu_BlockDownload_End(void)
{

    uint8 abuff[8]={0xD1,0,0,0,0,0,0,0};

    if(stu_McuFirmware.ucMcuRespflag==1)
    {
        stu_McuFirmware.ucMcuRespflag = 0;
		stu_McuFirmware.ucMcuCmdTimerOut = 15;	
		abuff[1] = (uint8)stu_McuFirmware.uscheck&0xFF;
		abuff[2] = (uint8)(stu_McuFirmware.uscheck>>8)&0xFF;
        CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //标准帧
        stu_McuFirmware.ucMcuRespflag = 0;
		stu_McuFirmware.ucMcuCmdTimerOut = 15;			
	}
}

#if 0
//通知MCU重启命令 块启动
void Mcu_Restart_Upgrade1(void)
{
    uint8 abuff[8]={0xc6,0,0,0,0x0d,0,0,0};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //标准帧
  //  stu_McuFirmware.ucLoadStep = 6;
}

//通知MCU重启命令 块数据下载
void Mcu_Restart_Upgrade2(void)
{
    uint8 abuff1[8]={0x01,0x53,0x54,0x07,0x00,0x11,0x55,0xaa};
    uint8 abuff2[8]={0x82,0x12,0x34,0x45,0x44,0x96,0x56,0x01};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff1);   //标准帧
    OSTimeDly(6);
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff2);   //标准帧
	//stu_McuFirmware.ucLoadStep = 6;
}

//通知MCU重启命令 块数据下载结束
void Mcu_Restart_Upgrade3(void)
{
    uint8 abuff[8]={0xc5,0x96,0x56,0,0,0,0,0};
 
    CanWrite(CAN_CHANNEL1, stu_McuFirmware.ucCANFrameFormat, 0x611, 8, abuff);   //标准帧

	stu_McuFirmware.ucLoadStep = 0xFF;
}
#endif
/******************************************************************************
** 函数名称:  McuFirmwareTimerOut
** 功能描述:  对控制器或者仪表进行升级时 超时处理
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2018-02-7
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
** 函数名称: SYS_SendUpgradeRet
** 功能描述: 向平台上报升级结果
** 输    入: 
** 输    出: 无
** 返    回: 
** 作    者: hhm
** 日    期: 2016-9-19
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
//向MCU发送固件数据内容
void Mcu_FirmwareDownload_Function(void)
{
  //向MCU发送升级数据的步骤
/*
	if(1!=GetCanRcvState(CAN_CHANNEL1))  //判断CAN 通讯状态
		return;
	*/
	if(!GetAccState())
		return;
  
    switch(stu_McuFirmware.ucLoadStep)
    {
        case 1:    //发送块启动给MCU
			Mcu_BlockDownload_Init();
			break;
		case 2:    //发送块数据包
		//    if(stu_McuFirmware.ucMcuRespflag==1)
	//	    {
        		McuUpload_CreatePackage();
        		McuUploadSend();	
			//	stu_McuFirmware.ucMcuRespflag = 0;
			//	stu_McuFirmware.ucMcuCmdTimerOut = 15;
		//	}
			break;
		case 3:    //发送块结束
		    Mcu_BlockDownload_End();
			break;
		case 4:    // 发送重启命令块启动
		    Mcu_Restart_Upgrade1();
			break;
		case 5:     //发送重启命令块下载
		    Mcu_Restart_Upgrade2();
			break;
		case 6:     //发送重启命令块下载结束
		    Mcu_Restart_Upgrade3();
			break;  
		case 7:     //通知平台MCU升级结果命令  发送完延迟2秒等结果            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
		default:
		    break;
	}
}
#endif

#if 0
//向MCU发送固件数据内容
void Mcu_FirmwareDownload_Function(void)
{
  //向MCU发送升级数据的步骤
/*
	if(1!=GetCanRcvState(CAN_CHANNEL1))  //判断CAN 通讯状态
		return;
	*/
	if(!GetAccState())
		return;
  
    switch(stu_McuFirmware.ucLoadStep)
    {
        case 1:    //发送块启动给MCU
			Mcu_BlockDownload_Init();
			break;
		case 2:    //发送块数据包
		//    if(stu_McuFirmware.ucMcuRespflag==1)
	//	    {
        		McuUpload_CreatePackage();
        		McuUploadSend();	
			//	stu_McuFirmware.ucMcuRespflag = 0;
			//	stu_McuFirmware.ucMcuCmdTimerOut = 15;
		//	}
			break;
		case 3:    //发送块结束
		    Mcu_BlockDownload_End();
			break;
			#if 0
		case 4:    // 发送重启命令块启动
		    Mcu_Restart_Upgrade1();
			break;
		case 5:     //发送重启命令块下载
		    Mcu_Restart_Upgrade2();
			break;
		case 6:     //发送重启命令块下载结束
		    Mcu_Restart_Upgrade3();
			break; 
			#endif
		case 7:     //通知平台MCU升级结果命令  发送完延迟2秒等结果            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
		default:
		    break;
	}
}
#endif


/*---------------------对协处理器或者外部控制器进行升级------------------*/
//通知协处理器或者外部控制器设备升级

void Ext_Mcu_UpgradeNotice(void)
{
    uint8 buff[100] = {0};
	static uint8 ucSN = 0;
    uint8 uclen = 0;
	
	ucSN++;
	buff[0] = 0x01;    //命令标识
	buff[1] = FirmwareUpdate.uctype;   //升级类型
	buff[2] = FirmwareUpdate.ucdev;//升级目标设备
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
	buff[0] = 0x01;    //命令标识
	buff[1] = FirmwareUpdate.uctype;   //升级类型
	buff[2] = FirmwareUpdate.ucdev;//升级目标设备
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
//向MCU发送升级包数据 (开始读取文件数据前确保文件已经关闭状态) 
//考虑重复读取的可能? 
void Ext_Mcu_Upgradepackage_send(void)
{
    uint8 buff[1200] = {0};
	static uint8 ucSN = 0;
    uint16 uslen = 0;
	
	ucSN++;
	buff[0] = 0x02;    //命令标识
	buff[1] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn>>8);   //当前升级包序号
	buff[2] = (uint8)(stu_ExtMCUUpgrade.usRequestPacketSn&0xFF); //
    buff[3] = (uint8)(FirmwareUpdate.usPackets>>8);           //升级总包数
    buff[4] = (uint8)(FirmwareUpdate.usPackets&0xFF);
	/*
	if(FirmwareUpdate.usRequestPacketSn==0)
	{
     //关闭升级文件
	}
	*/
	if(stu_ExtMCUUpgrade.usRequestPacketSn==FirmwareUpdate.usPackets-1)
        uslen = FirmwareUpdate.usLastPacketLen;
	else
		uslen = 1024;
	buff[5] = (uint8)(uslen>>8);
	buff[6] = (uint8)(uslen&0xFF);
  // ReadFromWholFileFlash(FLASH_FIRMWARE_Auxi_MCU, FIRMWARE_PACKET_LEN, &buff[7]);
    if(FirmwareUpdate.ucdev==3)  //协处理器
	    ReadFromlseekFlash(FLASH_FIRMWARE_Auxi_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]); 
	else if(FirmwareUpdate.ucdev==1)   //车辆控制器
	    ReadFromlseekFlash(FLASH_FIRMWARE_Vehicle_MCU,stu_ExtMCUUpgrade.usRequestPacketSn*1024,1024,&buff[7]); 
		
    A5Uart0_DataSend(0x04, 7+uslen, buff, ucSN);
}

//升级结果反馈
void Ext_Mcu_UpgradeResults_Ack(void)
{
    uint8 buff[10] = {0};
	static uint8 ucSN = 0;
	//需要向平台发送结果命令
	ucSN++;
	buff[0] = 0x03;    //命令标识
	buff[1] = stu_ExtMCUUpgrade.ucDev;       //当前升级包序号
	buff[2] = stu_ExtMCUUpgrade.ucResult;    //升级结果
	 
    A5Uart0_DataSend(0x04, 3, buff, ucSN);
}

void Ext_Mcu_Upgrade_Function(void)
{

//	if(!GetAccState())   //如果是升级车辆控制器需要判断ACC
//		return;
    //时间循环100ms
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
        case 1:         //通知设备开始升级
            Ext_Mcu_UpgradeNotice();
			stu_ExtMCUUpgrade.ucTimer = 100;
			break;
		case 2:         //应答升级请求包
		    Ext_Mcu_Upgradepackage_send();
			stu_ExtMCUUpgrade.ucUpgradeStep = 0xFF;  //发送过升级包后 等待外部MCU请求下一包
			break;
		case 3:         //升级结果上报处理
		  //  Ext_Mcu_UpgradeResults_Ack();
			Mcu_SendUpgradeResult();
			stu_ExtMCUUpgrade.ucTimer = 300;
			//stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //升级包传送结束
			break;
		case 0xFF:     //中间步骤
			break;
		default:
			break;
	}
}
/***********************************************************************
** 函数名称: Ext_Mcu_UpgradeNotice_Resp
** 功能描述: 处理协处理器应答升级请求命令
**
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址                  
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
             
** 输　  出: 
** 返    回: 
**
** 作　  者: 
** 日　  期: 2019年09月7日

************************************************************************/
void Ext_Mcu_UpgradeNotice_Resp(uint8* ptr, uint16 uslen)
{
    if(ptr[0]!=0)
		stu_ExtMCUUpgrade.ucUpgradeStep = 0;     //结束升级
	else
		stu_ExtMCUUpgrade.ucUpgradeStep = 0xFF;  //等待协处理器下一步骤

	//printf("step= %x",stu_ExtMCUUpgrade.ucUpgradeStep);
}

/***********************************************************************
** 函数名称: Ext_Mcu_Upgradepackage_Request
** 功能描述: 处理协处理器升级包请求命令
**
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址                  
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
             
** 输　  出: 
** 返    回: 
**
** 作　  者: 
** 日　  期: 2019年09月7日

************************************************************************/
void Ext_Mcu_Upgradepackage_Request(uint8* ptr, uint16 uslen)
{
    uint16 usRequestPacketSn = 0;     //请求升级包序号

	usRequestPacketSn = (uint16)(ptr[0]<<8) + ptr[1];
	stu_ExtMCUUpgrade.usRequestPacketSn = usRequestPacketSn;  //等待协处理器下一步骤
    stu_ExtMCUUpgrade.ucUpgradeStep = 2;
	stu_ExtMCUUpgrade.ucTimer = 0;   //快速应答 计数器清0
	
}

/***********************************************************************
** 函数名称: Ext_Mcu_UpgradepackageResult_Request
** 功能描述: 处理协处理器升级结果请求命令
**
** 输    入: ptr,    通讯协议中命令标识(0x01)后一个字节指针地址                  
             uslen， 协议中数据体长度-1(去掉命令标识1字节长度)
             
** 输　  出: 
** 返    回: 
**
** 作　  者: 
** 日　  期: 2019年09月7日

************************************************************************/
void Ext_Mcu_UpgradepackageResult_Request(uint8* ptr, uint16 uslen)
{
    uint8 ucDev = 0;
	uint8 ucResult = 0;
	stu_ExtMCUUpgrade.ucDev = ptr[0]; 
	stu_ExtMCUUpgrade.ucResult = ptr[1];
	stu_ExtMCUUpgrade.ucUpgradeStep = 3;
	stu_ExtMCUUpgrade.ucTimer = 0;   //快速应答 计数器清0
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
		ucSubCommand = ptr[5];    //子设备号
    	ucSumNum = SumCalc(&ptr[3], usDatalen-1);
    	if(ucSumNum!=ptr[uslen-3])
    		return;
		
    	if(ucCommand==0x04)
    	{
            switch(ucSubCommand)
            {
                case 1:           //收到协处理器升级应答结果
                    Ext_Mcu_UpgradeNotice_Resp(&ptr[6], usDatalen-3);
					break;
				case 2:           //收到协处理器请求升级包命令
				    Ext_Mcu_Upgradepackage_Request(&ptr[6], usDatalen-3);
					break; 
				case 3:           //收到协处理器升级结果命令
				    Ext_Mcu_UpgradepackageResult_Request(&ptr[6], usDatalen-3);
					break;
				default:
					break;
			}
		}
        else if(ucCommand==0x83)      //MCU对核心板发送命令的应答
    	{
            ucResult = ptr[5];   // 0-成功；1-异常。 失败可以考虑重发
    	}	
    }
	else   //考虑透传数据 设置程序
	{

	}
}
//每秒执行一次
void Ext_Mcu_Timeout_Function(void)
{
    if(stu_ExtMCUUpgrade.usTimeOut)
    {
        stu_ExtMCUUpgrade.usTimeOut--;
        if(!stu_ExtMCUUpgrade.usTimeOut)
        {
            stu_ExtMCUUpgrade.ucUpgradeStep = 3; 
            if(FirmwareUpdate.ucdev==3)         //协处理器
				stu_ExtMCUUpgrade.ucResult = 1;
			else if(FirmwareUpdate.ucdev==1)    //外部控制器
				stu_ExtMCUUpgrade.ucResult = 4;
			else
                stu_ExtMCUUpgrade.ucUpgradeStep = 0;   //其他情况 退出升级
		}
	}
}



//-----文件McuUpload.c结束---------------------------------------------
