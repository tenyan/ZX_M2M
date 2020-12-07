/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: Canconfig.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述:
  本文件为CAN 帧按照配置文件进行数据解析分类文件
 *
 *-----------------------------------------------------------------------------
 * 修改历史
 *-----------------------------------------------------------------------------
 *
 * 2019-09-25 by lxf, 创建本文件
 *
 */
#include "config.h"

//-----外部变量定义------------------------------------------------------------
extern uint16 f_usUpLoadCmdSn;
extern STU_SSData SSData;
extern uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
//-----内部变量定义------------------------------------------------------------
STU_CanProjects g_stuCanProjects;      //CAN配置文件对应结构体
STU_CanDataUp g_stuCanDatUp[CAN_PARAMGROUP_MAX_NUM];

//单包1024为依据,如果需要5个相同CAN数组数据一起上传,该组参数最大可配置15个CAN帧(以每帧8个字节计算)
//以上报最快1S间隔计算满足15帧 200ms采集  
//时间超过5秒的采集频率 单个参数组最大 可配置75个
//目前测试 1秒最快可以发送5包数据(也可能是因为4G模块uart口收到ST发送的CAN帧时间不能再小问题)
/*
uint8 auCanParam[2048] = {
 0x7E,
 0x00,0x72,  //长度=TLV个数+数据+CheckCode
 0x03,       //TLV个数据
 0x20,0xA1,0x00,25,0x00,0x14,0x00,0x0A,0x04,0x0C,0xF0,0x03,0x00,0xFF,0x0C,0xF0,0x04,0x00,0xFF,0x18,0xFF,0xF8,0x00,0xFF,0x18,0xFF,0xF9,0x00,0xFF,//200ms
 0x20,0xA2,0x00,35,0x01,0xF4,0x00,0x0A,0x06,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,0x1A,0xDC,0x25,0xC1,0xFF,0x1A,0xDC,0x26,0xC1,0xFF,//5s
 0x20,0xA3,0x00,40,0x03,0xE8,0x00,0x0A,0x07,0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x1A,0xDC,0x01,0xC1,0xFF,0x1A,0xDC,0x06,0xC1,0xFF,0x1A,0xDC,0x08,0xC1,0xFF,//10s
 0x00,       //CheckCode
 0x0D,0x0A
};
*/


uint8 auCanParam[2048] = {
/*
0x7E,
0x00,0xB3,    //数据长度(TLV参数个数1byte+TLV数据长度)不包含长度自身和校验码
0x02,         //TLV个数
0x20,0xA1,0x00,0x23,0x00,0x0A,0x00,0x0A,0x06,0x0C,0xF0,0x03,0x00,0xFF,0x0C,0xF0,0x04,0x00,0xFF,0x18,0xFF,0xF8,0x00,0xFF,0x18,0xFE,0xD9,0x00,0xFF,0x18,0xFF,0xE2,0x00,0xFF,0x18,0xFF,0xF9,0x00,0xFF,
0x20,0xA2,0x00,0x87,0x01,0xF4,0x00,0x0A,0x1A,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,0x1A,0xDC,0x25,0xC1,0xFF,0x1A,0xDC,0x26,0xC1,0xFF,0x1A,0xDC,0x27,0xC1,0xFF,0x1A,0xDC,0x28,0xC1,0xFF,0x1A,0xDC,0x29,0xC1,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xF7,0x00,0xFF,0x18,0xFE,0xDB,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,
                                             0x18,0xFE,0xF6,0x00,0xFF,0x18,0xFF,0x03,0x00,0xFF,0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xF5,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x18,0xFE,0xD0,0x00,0xFF,0x18,0xFE,0xD2,0x00,0xFF,0x18,0xFF,0xDC,0x00,0xFF,0x18,0xFE,0xEB,0x00,0xFF,0x18,0xFF,0x7A,0x00,0xFF,0x18,0xFE,0xB1,0x00,0xFF,0x18,0xE8,0xE4,0x00,0xFF,0x18,0xFE,0xE5,0x00,0xFF,
0x6D     //累加和校验码(从0x7E开始到校验和之前所有数据相加)
*/
};
/*
uint8 auCanParam[2048] = {
 0x7E,
 0x00,0x72,  //长度=TLV个数+数据+CheckCode
 0x03,       //TLV个数据
 0x20,0xA1,0x00,80,0x00,0x14,0x00,0x0A,0x0F,0x0C,0xF0,0x03,0x00,0xFF,0x0C,0xF0,0x04,0x00,0xFF,0x18,0xFF,0xF8,0x00,0xFF,0x18,0xFF,0xF9,0x00,0xFF,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,
                                            0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x1A,0xDC,0x01,0xC1,0xFF,0x1A,0xDC,0x06,0xC1,0xFF,0x1A,0xDC,0x08,0xC1,0xFF, //200ms
 0x20,0xA2,0x00,35,0x01,0xF4,0x00,0x0A,0x06,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,0x1A,0xDC,0x25,0xC1,0xFF,0x1A,0xDC,0x26,0xC1,0xFF,//5s
 0x20,0xA3,0x00,40,0x03,0xE8,0x00,0x0A,0x07,0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x1A,0xDC,0x01,0xC1,0xFF,0x1A,0xDC,0x06,0xC1,0xFF,0x1A,0xDC,0x08,0xC1,0xFF,//10s
 0x00,       //CheckCode
 0x0D,0x0A
};
*/
#if 0   //通过读取ASCII然后转换为hex格式
void CANConfig_ReadCANCFGFile_Init(void)
{
    uint8 *p1,*p2;
	uint8 i = 0;
    uint8 buff[CAN_CFG_FILE_MAX_LEN] = {0};
	uint8 CanParambuff[CAN_CFG_FILE_MAX_LEN/2];
	uint16 sublen = 0;
	uint16 addr = 0,usPacklen = 0;
	uint16 usdatalen = 0;     
    ReadFromFlash(FLASH_CAN_FILE_CFG,CAN_CFG_FILE_MAX_LEN,buff);    //读取CAN配置文件数据，让后放入auCanParam
	p1=SearchString(buff,CAN_CFG_FILE_MAX_LEN, "$", 1);
	if(p1)
	{   
		p2 = GetStringMiddle(p1,CAN_CFG_FILE_MAX_LEN,'$',1,'$',2,&sublen);
		if(p2)
		{
            HexToDatabuff(CanParambuff, p2, sublen);
			if(CanParambuff[0]!=0x7E||CanParambuff[3]==0||CanParambuff[3]>5)
				return;
			p2 += sublen+2;
		}
		for(i=0;i<CanParambuff[3];i++)
		{   
		    sublen = 0;
		    p1 = GetStringMiddle(p2,CAN_CFG_FILE_MAX_LEN,'&',1,'&',2,&sublen);
			HexToDatabuff(&CanParambuff[4+addr], p1, sublen);
			p2 = p1 + sublen+2;
			addr += sublen/2;
		//	printf("sublen= %d,addr= %d \n",sublen,addr);
		}
		p1 = GetStringMiddle(p2,CAN_CFG_FILE_MAX_LEN,'$',1,'$',2,&sublen);
		if(p1)
		{
            HexToDatabuff(&CanParambuff[4+addr], p1, sublen);
			addr += sublen/2;
		}
		usPacklen = 4+addr;     //配置文件数据总长度
		//包尾校验
		if(CanParambuff[usPacklen-2]!=0x0D||CanParambuff[usPacklen-1]!=0x0A)
			return;
		
	/*	//长度校验
		usdatalen = (uint16)(CanParambuff[1]<<8) + CanParambuff[2];
		if(usdatalen != usPacklen-6)   //长度不包含校验字节
		    return;
		
		//校验和判断,
	*/		
		memcpy(auCanParam,CanParambuff,CAN_CFG_FILE_MAX_LEN/2);
	}
}
#endif
/**********************************************************************************
** 函数名称: CANConfig_ReadCANCFGFile_Init
** 功能描述: 从存储文件读取CAN高频参数配置数据(数据为hex格式)
** 输    入: 无
** 输　  出: 无
** 全局变量: 
** 调用模块:
** 作　  者: lxf
** 日　  期: 2020-05-22
**--------------------------------------------------------------------------------
*********************************************************************************/
void CANConfig_ReadCANCFGFile_Init(void)
{
    uint8 *p1,*p2;
	uint8 i = 0;
    uint8 buff[CAN_CFG_FILE_MAX_LEN] = {0};
	uint8 CanParambuff[CAN_CFG_FILE_MAX_LEN/2];
	uint16 sublen = 0;
	uint16 addr = 0,usPacklen = 0;
	uint16 usdatalen = 0;  
	uint8 ucFlag = 0;
	uint8 ucSumValue = 0;
    ucFlag = ReadFromFlash(FLASH_CAN_FILE_CFG,CAN_CFG_FILE_MAX_LEN,buff);    //读取CAN配置文件数据，让后放入auCanParam
    if(ucFlag==1)
    {
        PC_SendDebugData((uint8 *)("CANFile_ERR1"), 12, DEBUG_ANYDATA);
		return ;
    }
	usdatalen = (uint16)(buff[1]<<8) + buff[2];
    if(buff[0]!=0x7E||usdatalen>1460)
    {
        PC_SendDebugData((uint8 *)("CANFile_ERR2"), 12, DEBUG_ANYDATA);
		return ;
    }
	ucSumValue = SumCalc(&buff[0], usdatalen+3);
	if(ucSumValue!=buff[usdatalen+3])
	{
        PC_SendDebugData((uint8 *)("CANFile_ERR3"), 12, DEBUG_ANYDATA);
		return;
	}
	memcpy(auCanParam,buff,usdatalen+3);
    PC_SendDebugData((uint8 *)("CANFile_OK"), 10, DEBUG_ANYDATA);
}


//-----内部函数声明------------------------------------------------------------
//CAN配置文件初始化
#if 0
void CANConfig_Read_Param(void)
{
    uint8 i = 0,j = 0;
	uint8 *ptemp;

    CANConfig_ReadCANCFGFile_Init();     //读取CAN配置文件

	//判断长度校验
	if(auCanParam[3]<=CAN_PARAMGROUP_MAX_NUM)  //此处可以从文件读取,作为校验条件
        g_stuCanProjects.ucParamGroupNum = auCanParam[3];
	else
		g_stuCanProjects.ucParamGroupNum = CAN_PARAMGROUP_MAX_NUM;
	ptemp = &auCanParam[4];
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
	{
    	g_stuCanProjects.stuParamGroup[i].usTlvName = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usTlvLen = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usSamplingFreq = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usUpFreq = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].ucCanNum = *ptemp++;
		for(j=0;j<g_stuCanProjects.stuParamGroup[i].ucCanNum;j++)
		{
			g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID = (uint32)((*ptemp++)<<24)+(uint32)((*ptemp++)<<16)+(uint32)((*ptemp++)<<8)+*ptemp++;
			g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].ucOptionByte = *ptemp++;
			//printf("Name = %x, uiID = %x, byte= %x \n",g_stuCanProjects.stuParamGroup[i].usTlvName,g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID,g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].ucOptionByte);
		}
	}
	CanConfig_Param_Init();
	CAN_ClearCanDataUpbuff();
}
#endif
/**********************************************************************************
** 函数名称: CANConfig_extract_Param
** 功能描述: 对收到CAN配制数据进行解析和判断准确性
** 输    入: 无
** 输　  出: 1=返回异常,0-返回正常
** 全局变量: 
** 调用模块:
** 作　  者: lxf
** 日　  期: 2020-05-22
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 CANConfig_extract_Param(void)
{
    uint8 i = 0,j = 0;
	uint8 *ptemp;


	//判断长度校验
	if(auCanParam[3]<=CAN_PARAMGROUP_MAX_NUM)  //此处可以从文件读取,作为校验条件
    {
        g_stuCanProjects.ucParamGroupNum = auCanParam[3];
		if(g_stuCanProjects.ucParamGroupNum==0)
			return 0;   //为0无需向下执行
	}
	else
		return 1;   //异常
	ptemp = &auCanParam[4];
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
	{
	    
    	g_stuCanProjects.stuParamGroup[i].usTlvName = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usTlvLen = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usSamplingFreq = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].usUpFreq = (uint16)((*ptemp++)<<8) + *ptemp++;
		g_stuCanProjects.stuParamGroup[i].ucCanNum = *ptemp++;
        if(g_stuCanProjects.stuParamGroup[i].ucCanNum>CAN_FRAME_MAX_MUN)
			return 1;
		if(g_stuCanProjects.stuParamGroup[i].usTlvLen!=5+g_stuCanProjects.stuParamGroup[i].ucCanNum*5)
			return 1;   //TLV长度和内容不符
		
		for(j=0;j<g_stuCanProjects.stuParamGroup[i].ucCanNum;j++)
		{
			g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID = (uint32)((*ptemp++)<<24)+(uint32)((*ptemp++)<<16)+(uint32)((*ptemp++)<<8)+*ptemp++;
			g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].ucOptionByte = *ptemp++;
			//printf("Name = %x, uiID = %x, byte= %x \n",g_stuCanProjects.stuParamGroup[i].usTlvName,g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID,g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].ucOptionByte);
		}
	}
	return 0;
//	CanConfig_Param_Init();
//	CAN_ClearCanDataUpbuff();
}

/**********************************************************************************
** 函数名称: CANConfig_Read_CanParam
** 功能描述: 读取CAN配置参数并进行打包
** 输    入: 无
** 输　  出: 参数长度 长度=1 表示当前无参数配置
** 全局变量: 
** 调用模块:
** 作　  者: lxf
** 日　  期: 2020-05-22
**--------------------------------------------------------------------------------
*********************************************************************************/
uint16 CANConfig_Read_CanParam(uint8* ptr)
{
    uint8 i = 0,j = 0;
    uint16 usdatalen = 0;
	
    *ptr++ = g_stuCanProjects.ucParamGroupNum;
	usdatalen = 1;
    for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
    {
    	*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usTlvName>>8);
    	*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usTlvName&0xFF);
		
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usTlvLen>>8);
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usTlvLen&0xFF);
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usSamplingFreq>>8);
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usSamplingFreq&0xFF);
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usUpFreq>>8);
		*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].usUpFreq&0xFF);
		*ptr++ = g_stuCanProjects.stuParamGroup[i].ucCanNum;
		for(j=0;j<g_stuCanProjects.stuParamGroup[i].ucCanNum;j++)
		{
			*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID>>24);
			*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID>>16);
			*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID>>8);
			*ptr++ = (uint8)(g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].uiID&0xFF);
			*ptr++ = g_stuCanProjects.stuParamGroup[i].stuCanFrame[j].ucOptionByte;
		}		
		usdatalen += (4+g_stuCanProjects.stuParamGroup[i].usTlvLen);
	}
	if(g_stuCanProjects.ucParamGroupNum==0)   //无参数组
		return 1;
	else
	{
        return usdatalen;
	}
}

//CAN配置初始化函数
void CanConfig_Param_Init(void)
{
    uint8 i = 0,j = 0;
	
//需要将所有的配置CAN获取到，作为协处理器过滤依据
    for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
    {
        if(g_stuCanProjects.stuParamGroup[i].usSamplingFreq<10)
			g_stuCanProjects.stuParamGroup[i].usSamplingFreq = 10;
		if(g_stuCanProjects.stuParamGroup[i].usSamplingFreq<500)    //??500还是50(单位是10ms)
			g_stuCanProjects.agrouptype[i] = 1;   //需要采集够5次上报
		else
			g_stuCanProjects.agrouptype[i] = 2;   //按照正常采集频率上报
		for(j=0;j<CAN_PARAMGROUP_MAX_NUM;j++)
            g_stuCanDatUp[i].aCount[j] = 0;
		
		g_stuCanDatUp[i].usTlvName = g_stuCanProjects.stuParamGroup[i].usTlvName+0x1000;
		g_stuCanDatUp[i].usSamplingFreq = g_stuCanProjects.stuParamGroup[i].usSamplingFreq;
		g_stuCanDatUp[i].usDataLen = CAN_GetCacheLocation_Function(i, g_stuCanProjects.stuParamGroup[i].ucCanNum);
        if(g_stuCanProjects.agrouptype[i]==1)
        {
			g_stuCanDatUp[i].usTlvLen = 7+(5*(2+g_stuCanDatUp[i].usDataLen));
			g_stuCanDatUp[i].ucTlvNum = 5;
        }
		else
		{
			g_stuCanDatUp[i].usTlvLen = 7+(2+g_stuCanDatUp[i].usDataLen);
			g_stuCanDatUp[i].ucTlvNum = 1;
		}
        printf("Name = %x, len = %x, TlvNum= %x \n",g_stuCanDatUp[i].usTlvName,g_stuCanDatUp[i].usDataLen,g_stuCanDatUp[i].ucTlvNum);
	}
}

void CAN_ClearCanDataUpbuff(void)
{
    uint8 i = 0,j = 0;
	uint16 m = 0;

	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
	{
        for(j=0;j<CAN_PARAMGROUP_MAX_NUM;j++)
            g_stuCanDatUp[i].aCount[j] = 0;
		
		g_stuCanDatUp[i].ustimer = 0;
		for(m=0;m<CAN_DATA_MAX_LEN;m++)
		    g_stuCanDatUp[i].aData[m] = 0xFF;		
	}
}

/******************************************************************************
** 函数名称: CAN_GetFrameSave_Flag
** 功能描述: 根据上次CAN帧保存时间计算当前时间接收到的CAN帧是否在规定采集时间内
** 
** 输    入: uiOldtime:CAN帧上次保存时间,uiNewtime:本次接收到CAN帧时间  单位ms
             usCollectTime:CAN帧采集频率单位10ms
** 输    出: 无
** 返    回: 0-不需要保存,1-需要保存
**
** 作    者: lxf
** 日    期: 2019-09-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 CAN_GetFrameSave_Flag(uint32 uiOldtime,uint32 uiNewtime,uint16 usCollectTime)
{
    uint32 uitimer = 0;
    uint32 uitimerTemp = 0;
	
	uitimer = usCollectTime*10;
	
    if(uiNewtime>=uiOldtime)
    {
        if((uiNewtime-uiOldtime)>=uitimer)
			return 1;
	}
	else
	{   
        uitimerTemp = (3600000 - uiOldtime) + uiNewtime;
		if(uitimerTemp >= uitimer)
			return 1;
	}
	return 0;		
}

#if 0
//两个相邻CAN帧时间,如果时间差值超过5秒 则重新计时为0
uint16 CAN_GetBetweenTwoFrametimer(uint32 uiOldtime,uint32 uiNewtime)
{
    uint32 uitimer = 0;

    if(uiNewtime>=uiOldtime)
    {
        uitimer = uiNewtime-uiOldtime;
        if(uitimer>5000)
			return 0;
		else
			return uitimer;
	}
	else
	{   
        uitimer = (3601000 - uiOldtime) + uiNewtime;
        if(uitimer>5000)
			return 0;	
		else 
			return uitimer;
	}
	return 0;		
}
#endif
//两个相邻CAN帧时间,如果时间差值超过300秒 则重返回0xFFFFFFFF
uint32 CAN_GetBetweenTwoFrametimer(uint32 uiOldtime,uint32 uiNewtime)
{
    uint32 uitimer = 0;

    if(uiNewtime>=uiOldtime)
    {
        uitimer = uiNewtime-uiOldtime;
        if(uitimer>300000)
			return 0xFFFFFFFF;
		else
			return uitimer;
	}
	else
	{   
        uitimer = (3601000 - uiOldtime) + uiNewtime;
        if(uitimer>300000)
			return 0xFFFFFFFF;	
		else 
			return uitimer;
	}
	return 0;		
}
/******************************************************************************
** 函数名称: CAN_ucOptionByte_Function
** 功能描述: 根据上次CAN配置选项字 选取CNA帧上报数据
** 
** 输    入: STU_Can_Message:输入CAN帧数据,pOutdata:输出CAN帧数据, ucOptionbyte:CAN帧内容选项字
** 输    出: 无
** 返    回: 需要采集的数据长度
**
** 作    者: lxf
** 日    期: 2019-09-26
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 CAN_OptionByte_Function(STU_Can_Message stuCanMessage, uint8 *pOutdata,uint8 ucOptionbyte)
{
    uint8 i = 0;
	uint8 n = 0;

	*pOutdata++ = (uint8)(stuCanMessage.CanID>>24);
	*pOutdata++ = (uint8)(stuCanMessage.CanID>>16);
	*pOutdata++ = (uint8)(stuCanMessage.CanID>>8);
	*pOutdata++ = (uint8)(stuCanMessage.CanID&0xFF);
	*pOutdata++ = ucOptionbyte;
	for(i=0;i<8;i++)
	{
        if(ucOptionbyte&BIT(i))
        {
			*pOutdata++ = stuCanMessage.data[i];
			n++;
        }
	}
    return 5+n;
}

#if 0
uint16 CAN_GetOneCANParamLen(uint8 ucGroupIndex)
{
    uint16 usvalue = 0;
    uint8 i = 0;
	uint8 j = 0;    //计算单个CAN帧需要采集的数据长度
    uint8 uccanlen = 0; //单个CAN帧需要获取的长度
    uint8 ucCanNum = 0;
	ucCanNum = g_stuCanProjects.stuParamGroup[ucGroupIndex].ucCanNum;
	for(i=0;i<ucCanNum;i++)
	{
	    j = 0;
		uccanlen = 0;
    	for(j=0;j<8;j++)
    	{
            if(g_stuCanProjects.stuParamGroup[ucGroupIndex].stuCanFrame[i].ucOptionByte&BIT(j))
            {
    			uccanlen++;
            }
    	}	
        usvalue += (5+uccanlen);		
	}	
    return 2+usvalue;
}
#endif

/******************************************************************************
** 函数名称: CAN_GetCacheLocation_Function
** 功能描述: 根据CAN帧所在参数组内位置+配置选项字 计算出CAN组上报缓存起始位置
** 
** 输    入: ucGroupIndex:参数组序号 从0开始,   ucCanNumIndex:参数组中当前CAN帧序号 从0开始
** 输    出: 无
** 返    回: 需要保存当前CAN帧缓存位置 从0开始
**
** 作    者: lxf
** 日    期: 2019-12-7
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 CAN_GetCacheLocation_Function(uint8 ucGroupIndex,uint8 ucCanNumIndex)
{
    uint16 usvalue = 0;
    uint8 i = 0;
	uint8 j = 0;    //计算单个CAN帧需要采集的数据长度
    uint8 uccanlen = 0; //单个CAN帧需要获取的长度
	for(i=0;i<ucCanNumIndex;i++)
	{
	    j = 0;
		uccanlen = 0;
    	for(j=0;j<8;j++)
    	{
            if(g_stuCanProjects.stuParamGroup[ucGroupIndex].stuCanFrame[i].ucOptionByte&BIT(j))
            {
    			uccanlen++;
            }
    	}	
        usvalue += (5+uccanlen);		
	}
	
    return usvalue;
}

#if 0
uint16 CAN_GetCacheLocation_Function(uint8 ucGroupIndex,uint8 ucCanNumIndex)
{
    uint16 usvalue = 0;
    uint8 i = 0;

	for(i=0;i<ucCanNumIndex;i++)
	{
        usvalue += (5+g_stuCanProjects.stuParamGroup[ucGroupIndex].stuCanFrame[i].ucOptionByte);
	}
	
    return usvalue;
}


void CanConfig_Data_Collect(STU_DataTime stuDate, uint8 *ptr)
{
    uint8 i = 0;
	uint8 n = 0,m = 0;
	uint8 ucSaveFlag = 0;
	uint32 uiNewtime = 0;
	uint8 ucdatalen = 0;
    STU_Can_Message stuCanMessage;
	uint8 abuff[8] = {0};

    //计算当前时间 转换为ms   是否需要判断时间有效性?
    uiNewtime = (uint32)(stuDate.ucMin*60 +stuDate.ucSec)*1000 + stuDate.usmSec;
	stuCanMessage.usmSec = (uint16)(ptr[0]&0x07) + ptr[1];
	stuCanMessage.CanID = (uint32)(ptr[2]<<24) + (uint32)(ptr[3]<<16) +(uint32)(ptr[4]<<8) + ptr[5];
    memcpy(&stuCanMessage.data[0],&ptr[6],8);  
	//参数组1
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)    //确认在哪参数组
	{
        for(n=0;n<g_stuCanProjects.stuParamGroup[i].ucCanNum;n++)    //确认参数组具体哪一个CAN帧
        {
                 //处理CAN帧数据
            if(stuCanMessage.CanID==g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].uiID)  //确认参数组具体哪一个CAN帧
            {
                //判断时间
                ucSaveFlag = CAN_GetFrameSave_Flag(g_stuCanProjects.stuParamGroup.stuCanFrame[n].uiTime, uiNewtime);
                if(ucSaveFlag==0)
					return;
				//数据拷贝                //计算数组保存指针位置
				//搜索判断保存到那个上报CAN数组缓存
				for(m=0;m<CAN_PARAMGROUP_MAX_NUM;m++)
				{
			    	if((g_stuCanProjects.stuParamGroup[i].usTlvName+0x1000)==g_stuCanDatUp[m].usTlvName)
			    	{
			    	
			    	 //判断缓存长度是否溢出
			    	    ucdatalen = CAN_ucOptionByte_Function(stuCanMessage.data, &abuff[0]], g_stuCanProjects.stuParamGroup.stuCanFrame[n].ucOptionByte);
                        if(ucdatalen<=(CAN_DATA_MAX_LEN - g_stuCanDatUp[m].stuCanData[0].usDataLen))
                        {
                            memcpy(g_stuCanDatUp[m].stuCanData[0].aData[g_stuCanDatUp[m].stuCanData[0].usDataLen],&abuff[0],ucdatalen);
    					//	g_stuCanDatUp[m].stuCanData[0].usDataptr += ucdatalen;
    						g_stuCanDatUp[m].stuCanData[0].usDataLen += ucdatalen;
						//保存时间
						    g_stuCanProjects.stuParamGroup.stuCanFrame[n].uiTime = uiNewtime;
						    if(g_stuCanProjects.agrouptype[m]==1)    //需要立即触发的参数组
								g_stuCanProjects.agroupSendFlag[m] = 1;
						}	
						else   //当前存储缓存已满,是否更换下一个缓存
						{
                            g_stuCanProjects.agroupSendFlag[m] = 1;   //要考虑采集频率慢的帧情况
						}
						break;
					}
				}
				return;               
			}
		}
	}
}
#endif
/******************************************************************************
** 函数名称: CanConfig_Data_Collect
** 功能描述: 对接收到的每个CAN帧进行判断是否是CAN配置参数组中的需要采集的
** 
** 输    入: stuDate-当前CAN帧的时间; ptr-采集到的CAN帧数据: 2byte时间+8byteCAN数据
** 输    出: 无
** 返    回: 
**
** 作    者: lxf
** 日    期: 2019-09-26
**-----------------------------------------------------------------------------
*******************************************************************************/
//CAN数据采集打包 采集频率计算,CAN帧参数组分类, CAN数据格式打包
void CanConfig_Data_Collect(STU_DataTime stuDate, uint8 *ptr)
{
    uint8 i = 0;    //哪个参数组 、哪个上报参数组缓存
	uint8 n = 0;    //参数组的第几个CAN帧
	uint8 ucSaveFlag = 0;
	uint32 uiNewtime = 0;
	uint8 ucdatalen = 0;
    STU_Can_Message stuCanMessage;
	uint8 abuff[15] = {0};
	uint16 usPosition;
//	uint16 ustimer = 0;
//	static uint32 uiOldtime = 0;
    STU_Date stutime = {0,0,0,0,0,0};
	//uint16 ustimer[CAN_PARAMGROUP_MAX_NUM] = {0};
    static uint32 uiOldtime[CAN_PARAMGROUP_MAX_NUM] = {0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF};
	
    //CAN帧ID
	stuCanMessage.CanID = (uint32)(ptr[0]<<24) + (uint32)(ptr[1]<<16) +(uint32)(ptr[2]<<8) + ptr[3];
    //计算当前时间 转换为ms   是否需要判断时间有效性?
	stuCanMessage.usmSec = (uint16)((ptr[4]&0x07)<<8) + ptr[5];
    memcpy(&stuCanMessage.data[0],&ptr[6],8);  
    uiNewtime = (uint32)(stuDate.ucMin*60 +stuDate.ucSec)*1000 + stuDate.usmSec + stuCanMessage.usmSec;
    

	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++) 
	{
	    if(uiOldtime[i]==0xFFFFFFFF)
			uiOldtime[i] = uiNewtime;
        g_stuCanDatUp[i].ustimer = CAN_GetBetweenTwoFrametimer(uiOldtime[i], uiNewtime);
	//	printf("min = %d,sec = %d, msec1 = %d,msec2 = %d \n",stuDate.ucMin,stuDate.ucSec,stuDate.usmSec,stuCanMessage.usmSec);
	//	printf("ustimer = %d,old = %d, new = %d \n",g_stuCanDatUp[i].ustimer,uiOldtime[i],uiNewtime);

        if(g_stuCanDatUp[i].ustimer==0xFFFFFFFF)
        {
            uiOldtime[i] = uiNewtime;
			continue;
		}
      //  g_stuCanDatUp[i].ustimer += ustimer;
	//判断是否需要采集多组数据			
        if(g_stuCanDatUp[i].ustimer >= g_stuCanProjects.stuParamGroup[i].usSamplingFreq*10)
        {
    	    if(g_stuCanProjects.agrouptype[i]==1)	
    	    {
    	        if(stutime.ucYear==0||stutime.ucMon==0)
					memcpy((uint8*)&g_stuCanDatUp[i].Date,(uint8*)&stuDate,6);
				else
					memcpy((uint8*)&g_stuCanDatUp[i].Date,(uint8*)&stutime,6);
                g_stuCanDatUp[i].aCount[i]++;
				if(g_stuCanDatUp[i].aCount[i]>=5)
				{
				    g_stuCanDatUp[i].aCount[i] = 0;
					if(g_stuCanProjects.aucCanRcvFlag[i]==1)
			            g_stuCanProjects.agroupSendFlag[i] = 1;   //触发发送	
			        g_stuCanDatUp[i].ustimer = 0;
					//uiOldtime[i] = uiNewtime;
				    memcpy((uint8*)&stutime,(uint8*)&stuDate,6);    //用来保存下一次上报的采集起始时间				  
				}
    	    }
			else
			{
			    memcpy((uint8*)&g_stuCanDatUp[i].Date,(uint8*)&stuDate,6);
				if(g_stuCanProjects.aucCanRcvFlag[i]==1)
				    g_stuCanProjects.agroupSendFlag[i] = 1;       //触发发送	
				g_stuCanDatUp[i].aCount[i] = 0;
				g_stuCanDatUp[i].ustimer = 0;
				//uiOldtime[i] = uiNewtime;
			}
			uiOldtime[i] = uiNewtime;
        } 		
	}
		
	//uiOldtime = uiNewtime; //保存
	//参数组1
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)    //确认在哪个参数组
	{
        for(n=0;n<g_stuCanProjects.stuParamGroup[i].ucCanNum;n++)    //确认参数组具体哪一个CAN帧
        {
            //处理CAN帧数据
            if(stuCanMessage.CanID==g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].uiID)  //确认参数组具体哪一个CAN帧
            {
                //计算当前CAN帧 应该保持缓存的位置
				usPosition = CAN_GetCacheLocation_Function(i, n);
				//计算当前CAN帧 需要保存的长度(4byteID+1byte选项位+数据内容)
			    ucdatalen = CAN_OptionByte_Function(stuCanMessage, &abuff[0], g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].ucOptionByte);

                //判断 是否需要保存几组数据
                //CAN帧保存按照参数组顺序对应保存
                memcpy(&g_stuCanDatUp[i].aData[usPosition + g_stuCanDatUp[i].aCount[i]*g_stuCanDatUp[i].usDataLen],&abuff[0],ucdatalen);	
                g_stuCanProjects.aucCanRcvFlag[i] = 1;   //有收到配置CAN帧
		//		printf("usPosition = %d, ucdatalen = %d Count = %d\n",usPosition,ucdatalen,g_stuCanDatUp[i].aCount[i]);
				break;              
			}
		}
	}	
}


uint16 CanConfig_BuildPT(uint8 ucCanParamGroupNum, uint8 *buf)
{
	uint16 usTemp;
	uint16 usMsgBodyLen;
	uint8 ucTlvNmr = 0;
	uint8 *p = buf;
	uint8 i = 0;
	uint16 usTemplen = 0;
	
	*buf++ = 0x00;				//数据类型长度
	*buf++ = 0x02;
	*buf++ = 'P';
	*buf++ = 'T';
	buf++;						//数据内容长度
	buf++;
	buf++;						//状态同步TLV个数 

	usMsgBodyLen = 7;	
    
    *buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvName>>8);   //TLV1-T高字节
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvName&0xFF); //TLV1-T低字节
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvLen>>8);    //TLV1-L高字节
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvLen&0xFF);  //TLV1-L低字节
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usSamplingFreq>>8); //采集频率-高字节
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usSamplingFreq&0xFF);//采集频率-低字节
	memcpy(buf,(uint8*)&g_stuCanDatUp[ucCanParamGroupNum].Date,6); 
	buf += 6;
	*buf++ = g_stuCanDatUp[ucCanParamGroupNum].ucTlvNum;
	for(i=0;i<g_stuCanDatUp[ucCanParamGroupNum].ucTlvNum;i++)
	{
    	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usDataLen>>8);
    	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usDataLen&0xFF);
    	memcpy(buf, &g_stuCanDatUp[ucCanParamGroupNum].aData[i*g_stuCanDatUp[ucCanParamGroupNum].usDataLen],
			        g_stuCanDatUp[ucCanParamGroupNum].usDataLen);
		buf += g_stuCanDatUp[ucCanParamGroupNum].usDataLen;
		usTemplen += (2+g_stuCanDatUp[ucCanParamGroupNum].usDataLen);
	}
	//是否判断长度 是否为0?
    usMsgBodyLen += 13+usTemplen;  //数据长度
    ucTlvNmr++;                    //TLV  个数     
    
	p += 4;
	usTemp = usMsgBodyLen - 6;     //去掉前6个字节
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;

    return usMsgBodyLen;
}


//根据上报频率处理 500ms执行一次
uint8 CanConfig_Data_Send(void)
{
	uint16 usMsgBodyLen = 0, usSendLen = 0;
	static uint8 ucGroupNum = 0;
	static uint8 ucTimer = 0;
	uint8 i = 0;
/*
	if(ucTimer)
	{
		ucTimer--;
		return FALSE;
    }
	else
		ucTimer = 2;
	*/
	
    //先判断发送标识如果为1  在查看具体数据哪一个参数组
    //发送过清0标志和内容
	ucGroupNum++;
	if(ucGroupNum>g_stuCanProjects.ucParamGroupNum)
		ucGroupNum = 1;	
    if(g_stuCanProjects.agroupSendFlag[ucGroupNum-1]!=1)
    {    
        return FALSE;
    }

    //打包
    //1.参数组1
    f_usUpLoadCmdSn++;
	SSData.usRepSn = f_usUpLoadCmdSn;
	usMsgBodyLen = CanConfig_BuildPT(ucGroupNum-1,&aMsgSendData[MSG_HEAD_LEN]);
	usSendLen = BuildMsgHead(aMsgSendData, MSG_TYPE_PUSH_DATA, usMsgBodyLen, 0, f_usUpLoadCmdSn);
	usSendLen += usMsgBodyLen;
	aMsgSendData[usSendLen] = SumCalc(aMsgSendData, usSendLen);
	GSM_SendGprs(aMsgSendData, usSendLen+1, 0);  
/*	for(i=0;i<1;i++)
	{
    	if(GSM_SendGprs(aMsgSendData, usSendLen+1, 0)<0)
			OSTimeDly(20);
		else
			break;
	}	*/
    //清除发送标志 
	g_stuCanProjects.agroupSendFlag[ucGroupNum-1] = 0;
    g_stuCanProjects.aucCanRcvFlag[ucGroupNum-1] = 0;
	return TRUE;
}



