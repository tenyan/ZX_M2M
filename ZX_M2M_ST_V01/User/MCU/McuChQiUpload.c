/*
 * Copyright(c)2020, XXXXX公司硬件研发部
 * All right reserved
 *
 * 文件名称: McuChQiUpload.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为川崎控制器固件升级功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2020-01-17, by  lxf, 创建本文件
 *
 */
#include "config.h"
#include "IAP.h" 
//-----头文件调用------------------------------------------------------------


//-----外部变量定义------------------------------------------------------------


extern uint8 Public_Buf[];
extern STUSystem g_stuSystem;
extern uint16 f_usUpLoadCmdSn;				//上行命令的流水号
extern STUExtMCUUpgrade stu_ExtMCUUpgrade;
//-----内部变量定义------------------------------------------------------------
STUKCMCUDownload stu_KCMCUDownload;
//uint8 ExtMCUUartSendBuff[1024] = {0};
//-----内部函数声明------------------
/*****************************************************************************/

#if 1

void KCMCU_ProgramUpdate_Init(void)
{
    uint32 uiCounter = 0;
	
//计算程序总包数
    uiCounter = stu_KCMCUDownload.uiSoftwareWSize/38;
	if(stu_KCMCUDownload.uiSoftwareWSize%38)
		uiCounter +=1; 

    stu_KCMCUDownload.uiProgramTotPackets = uiCounter*5;
//计算参数总包数
    uiCounter = stu_KCMCUDownload.uiParamSize/8;
    if(stu_KCMCUDownload.uiParamSize%8)
		uiCounter += 1;
    stu_KCMCUDownload.uiParameterTotPackets = uiCounter;
    stu_KCMCUDownload.ucRepeatSendCount = 0;
	stu_KCMCUDownload.uiOffsetAddr = 0;
	stu_KCMCUDownload.ucLoadStep = 1;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}

//发送升级启动命令
void KCMCU_Update_Start_Command(void)
{
    uint8 abuff[8]={0,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//	stu_McuFirmware.usMcuUPdateTimerOut = Mcu_UPdateTimerOut;

    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
    CanWrite(CAN_CHANNEL2, EXT_FRAME, 0x06F20000, 8, abuff);     
}

//发送下载询问命令 ucType=0:固件程序数据;ucType=1:参数数据
void KCMCU_Update_Imfomation(uint8 ucType,uint32 uiTotalPacket)
{
    uint8 abuff[8]={0,0,0,0,0xFF,0xFF,0xFF,0xFF};

   //总包数
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
** 函数名称:  1)先判断接受平台下包是否结束并通过校验，2)获取升级文件总包数,
              3)从存储器中读取数据
** 功能描述:  每次读取380Byte数据内容,取前380Byte分成10组，每组38Byte
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2020-01-20
**-----------------------------------------------------------------------------
*******************************************************************************/
//暂时未考虑最后包数据不满380的处理方式
void KCMCU_CreateProgramUpPackage(void)
{
    uint8 abuff[400];
	uint8 aCanbuff[8];
//	uint16 usPackets = 0;    //升级总包数
//	uint8 ucnum,ucLastPacketlen=0;
	uint8 i=0;	
    uint8 j=0;
	uint16 usFlashAddr = 0;   //存储器地址
	uint16 usRemainder = 0;   //余数
	static uint32 uiID = 0x06820000;
	static uint32 uiIDPre;
//
//判断升级文件是否可用 
 //   if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
//		return;
//判断升级设备类型


    usFlashAddr = (stu_KCMCUDownload.uiOffsetAddr/512)*2 + FLASH_Firmware_MCU;
    usRemainder = stu_KCMCUDownload.uiOffsetAddr%512;

	if((512-usRemainder)>=380)  //不需要换页读取
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

//可以发送标志    
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
   //发送CAN帧
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, aCanbuff);   
        OSTimeDly(1);   //1ms间隔
	}
    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}

/******************************************************************************
** 函数名称:  KCMCU_CreateParameterUpPackage
              1)先判断接受平台下包是否结束并通过校验，2)获取升级文件总包数,
              3)从存储器中读取数据
** 功能描述:  每次读取400Byte数据内容,取前400Byte分成10组，每组40Byte
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2020-02-4
**-----------------------------------------------------------------------------
*******************************************************************************/
void KCMCU_CreateParameterUpPackage(void)
{
    uint8 abuff[400];
	uint8 aCanbuff[8];
//	uint16 usPackets = 0;    //升级总包数
//	uint8 ucnum,ucLastPacketlen=0;
	uint8 i=0;	
//    uint8 j=0;
	uint16 usFlashAddr = 0;   //存储器地址
	uint16 usRemainder = 0;   //余数
	static uint32 uiID = 0x06910000;
	static uint32 uiIDPre;
//
//判断升级文件是否可用 
 //   if(FirmwareUpdate.ucdev!=1||stu_McuFirmware.ucRcvPackflag!=1)
//		return;
//判断升级设备类型


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

//可以发送标志  
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
   //发送CAN帧
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, aCanbuff);   
        OSTimeDly(1);   //1ms间隔
	}
    stu_KCMCUDownload.ucRepeatSendCount++;
	stu_KCMCUDownload.ucMcuCmdTimerOut = KCMUC_SingleCMD_TIMEOUT;     //10s
}


#if 0
//发送固件升级数据(需要重发的时候 ID如何处理)
void KCMCU_ProgramUpdateSend(void)
{
    uint8 i = 0;
    uint8 abuff[8];
	static uint32 uiID = 0x06820000;
	
//可以发送标志    
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x068265500)
			uiID = 0x06820001;
        memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[i*8],8);
   //发送CAN帧
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, abuff);   //标准帧
        OSTimeDly(1);   //1ms间隔
	}
}




//发送参数升级数据(需要重发的时候 ID如何处理)
void KCMCU_ProgramUpdateSend(void)
{
    uint8 i = 0;
    uint8 abuff[8];
	static uint32 uiID = 0x06910000;
	
//可以发送标志    
    for(i=0;i<50;i++)
    {
        uiID++;
		if(uiID>0x069165500)
			uiID = 0x06910001;
        memcpy(&abuff[0],&stu_KCMCUDownload.aSendbuff[i*8],8);
   //发送CAN帧
        CanWrite(CAN_CHANNEL2, EXT_FRAME, uiID, 8, abuff);   //标准帧
        OSTimeDly(1);   //1ms间隔
	}
}
#endif



/******************************************************************************
** 函数名称:  KCMcuFirmwareTimerOut
** 功能描述:  对控制器或者仪表进行升级时 超时处理,每秒执行1次
** 
** 输    入: 
** 输    出: 
** 返    回: 
**
** 作    者: Lxf
** 日    期: 2020-02-14
**-----------------------------------------------------------------------------
*******************************************************************************/
void KCMcuFirmwareTimerOut(void)
{

    if(stu_KCMCUDownload.ucLoadStep==0)
		return ;
	/*
    if(stu_KCMCUDownload.ucRepeatSendCount > 10)
    {
        KCMCU_ProgramUpdate_Init();  //次数超过10次从头开始
	}
	*/

//单步命令超时 时间 10秒   
    if(stu_KCMCUDownload.ucMcuCmdTimerOut&&stu_KCMCUDownload.ucLoadStep)
    {
		stu_KCMCUDownload.ucMcuCmdTimerOut--;
		if(!stu_KCMCUDownload.ucMcuCmdTimerOut)
		{
           stu_KCMCUDownload.ucLoadStep = 0xFF;
		   // stu_McuFirmware.ucMcuRespflag = 1;
		}
	}	

	//升级总超时时间
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
	buff[0] = 0x03;                          //命令标识
	buff[1] = FirmwareUpdate.ucdev;;         //当前升级目标设备
	if(stu_McuFirmware.ucUploadResult==0)		
	    buff[2] = 4;   //控制器升级失败
	else if(stu_McuFirmware.ucUploadResult==1)
		buff[2] = 2;   //控制器升级成功
    uslen = 3;
		
	Ext_McuUart_DataSend(0x04, uslen, buff, ucSN);	
}
*/

//川崎控制器固件升级步骤控制函数
void KCMCU_ProgramUpdate_Function(void)
{
  //向MCU发送升级数据的步骤
/*
	if(1!=GetCanRcvState(CAN_CHANNEL1))  //判断CAN 通讯状态
		return;
	*/
	if(!GetAccState())
		return;
  
    switch(stu_KCMCUDownload.ucLoadStep)
    {
        case 0:
			break;
        case 1:    //发送升级启动命令
			KCMCU_Update_Start_Command();
			break;
		case 2:    //发送控制器固件下载命令
            KCMCU_Update_Imfomation(0, stu_KCMCUDownload.uiProgramTotPackets);
			break;
		case 3:    //发送控制器固件数据包命令
		    KCMCU_CreateProgramUpPackage();
			break;
		case 4:    //发送控制器参数下载命令
            KCMCU_Update_Imfomation(1, stu_KCMCUDownload.uiParameterTotPackets);
		    break;
		case 5:    //发送参数数据包命令
		    KCMCU_CreateParameterUpPackage();
		    break;
		case 6:     //通知平台MCU升级结果命令  发送完延迟2秒等结果            
            Mcu_SendUpgradeResult();     //send
            OSTimeDly(OS_TICKS_PER_SEC*2);
	        break;
	    case 0xFF:  //初始化
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
//KCMCU应答---程序
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
	case 0x1782FFFF:         //Update Imfomation Acknowledge(Result)(Result 0：Success 1：Failure 2：Unknown Command)
                             //Update Data Acknowledge(ACK/NACK 0：Success 1：Failure 2：Unknown Command)
	
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
		                         
//KCMCU应答---程序
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
	case 0x17F3FFFF:       //Data Complete Acknowledge //Result 0：Success 1：Failure 2：Unknown Command
        if(arr[0]==0)
        {
            if(stu_KCMCUDownload.ucLoadStep==3)
            {
				stu_KCMCUDownload.ucLoadStep = 4;
				stu_KCMCUDownload.ucRepeatSendCount = 0; 
            }
			else if(stu_KCMCUDownload.ucLoadStep==5)
			{
				stu_KCMCUDownload.ucLoadStep = 6;   //全部完成
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

//-----文件McuChQiUpload.c结束---------------------------------------------
