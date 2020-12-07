/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: Canconfig.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����:
  ���ļ�ΪCAN ֡���������ļ��������ݽ��������ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸���ʷ
 *-----------------------------------------------------------------------------
 *
 * 2019-09-25 by lxf, �������ļ�
 *
 */
#include "config.h"

//-----�ⲿ��������------------------------------------------------------------
extern uint16 f_usUpLoadCmdSn;
extern STU_SSData SSData;
extern uint8 aMsgSendData[GSM_SEND_BUFF_MAX_SIZE];
//-----�ڲ���������------------------------------------------------------------
STU_CanProjects g_stuCanProjects;      //CAN�����ļ���Ӧ�ṹ��
STU_CanDataUp g_stuCanDatUp[CAN_PARAMGROUP_MAX_NUM];

//����1024Ϊ����,�����Ҫ5����ͬCAN��������һ���ϴ�,���������������15��CAN֡(��ÿ֡8���ֽڼ���)
//���ϱ����1S�����������15֡ 200ms�ɼ�  
//ʱ�䳬��5��Ĳɼ�Ƶ�� ������������� ������75��
//Ŀǰ���� 1�������Է���5������(Ҳ��������Ϊ4Gģ��uart���յ�ST���͵�CAN֡ʱ�䲻����С����)
/*
uint8 auCanParam[2048] = {
 0x7E,
 0x00,0x72,  //����=TLV����+����+CheckCode
 0x03,       //TLV������
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
0x00,0xB3,    //���ݳ���(TLV��������1byte+TLV���ݳ���)���������������У����
0x02,         //TLV����
0x20,0xA1,0x00,0x23,0x00,0x0A,0x00,0x0A,0x06,0x0C,0xF0,0x03,0x00,0xFF,0x0C,0xF0,0x04,0x00,0xFF,0x18,0xFF,0xF8,0x00,0xFF,0x18,0xFE,0xD9,0x00,0xFF,0x18,0xFF,0xE2,0x00,0xFF,0x18,0xFF,0xF9,0x00,0xFF,
0x20,0xA2,0x00,0x87,0x01,0xF4,0x00,0x0A,0x1A,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,0x1A,0xDC,0x25,0xC1,0xFF,0x1A,0xDC,0x26,0xC1,0xFF,0x1A,0xDC,0x27,0xC1,0xFF,0x1A,0xDC,0x28,0xC1,0xFF,0x1A,0xDC,0x29,0xC1,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xF7,0x00,0xFF,0x18,0xFE,0xDB,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,
                                             0x18,0xFE,0xF6,0x00,0xFF,0x18,0xFF,0x03,0x00,0xFF,0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xF5,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x18,0xFE,0xD0,0x00,0xFF,0x18,0xFE,0xD2,0x00,0xFF,0x18,0xFF,0xDC,0x00,0xFF,0x18,0xFE,0xEB,0x00,0xFF,0x18,0xFF,0x7A,0x00,0xFF,0x18,0xFE,0xB1,0x00,0xFF,0x18,0xE8,0xE4,0x00,0xFF,0x18,0xFE,0xE5,0x00,0xFF,
0x6D     //�ۼӺ�У����(��0x7E��ʼ��У���֮ǰ�����������)
*/
};
/*
uint8 auCanParam[2048] = {
 0x7E,
 0x00,0x72,  //����=TLV����+����+CheckCode
 0x03,       //TLV������
 0x20,0xA1,0x00,80,0x00,0x14,0x00,0x0A,0x0F,0x0C,0xF0,0x03,0x00,0xFF,0x0C,0xF0,0x04,0x00,0xFF,0x18,0xFF,0xF8,0x00,0xFF,0x18,0xFF,0xF9,0x00,0xFF,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,
                                            0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x1A,0xDC,0x01,0xC1,0xFF,0x1A,0xDC,0x06,0xC1,0xFF,0x1A,0xDC,0x08,0xC1,0xFF, //200ms
 0x20,0xA2,0x00,35,0x01,0xF4,0x00,0x0A,0x06,0x1A,0xDC,0x21,0xC1,0xFF,0x1A,0xDC,0x22,0xC1,0xFF,0x1A,0xDC,0x23,0xC1,0xFF,0x1A,0xDC,0x24,0xC1,0xFF,0x1A,0xDC,0x25,0xC1,0xFF,0x1A,0xDC,0x26,0xC1,0xFF,//5s
 0x20,0xA3,0x00,40,0x03,0xE8,0x00,0x0A,0x07,0x18,0xFE,0xEE,0x00,0xFF,0x18,0xFE,0xEF,0x00,0xFF,0x18,0xFE,0xF2,0x00,0xFF,0x18,0xFE,0xE9,0x00,0xFF,0x1A,0xDC,0x01,0xC1,0xFF,0x1A,0xDC,0x06,0xC1,0xFF,0x1A,0xDC,0x08,0xC1,0xFF,//10s
 0x00,       //CheckCode
 0x0D,0x0A
};
*/
#if 0   //ͨ����ȡASCIIȻ��ת��Ϊhex��ʽ
void CANConfig_ReadCANCFGFile_Init(void)
{
    uint8 *p1,*p2;
	uint8 i = 0;
    uint8 buff[CAN_CFG_FILE_MAX_LEN] = {0};
	uint8 CanParambuff[CAN_CFG_FILE_MAX_LEN/2];
	uint16 sublen = 0;
	uint16 addr = 0,usPacklen = 0;
	uint16 usdatalen = 0;     
    ReadFromFlash(FLASH_CAN_FILE_CFG,CAN_CFG_FILE_MAX_LEN,buff);    //��ȡCAN�����ļ����ݣ��ú����auCanParam
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
		usPacklen = 4+addr;     //�����ļ������ܳ���
		//��βУ��
		if(CanParambuff[usPacklen-2]!=0x0D||CanParambuff[usPacklen-1]!=0x0A)
			return;
		
	/*	//����У��
		usdatalen = (uint16)(CanParambuff[1]<<8) + CanParambuff[2];
		if(usdatalen != usPacklen-6)   //���Ȳ�����У���ֽ�
		    return;
		
		//У����ж�,
	*/		
		memcpy(auCanParam,CanParambuff,CAN_CFG_FILE_MAX_LEN/2);
	}
}
#endif
/**********************************************************************************
** ��������: CANConfig_ReadCANCFGFile_Init
** ��������: �Ӵ洢�ļ���ȡCAN��Ƶ������������(����Ϊhex��ʽ)
** ��    ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: lxf
** �ա�  ��: 2020-05-22
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
    ucFlag = ReadFromFlash(FLASH_CAN_FILE_CFG,CAN_CFG_FILE_MAX_LEN,buff);    //��ȡCAN�����ļ����ݣ��ú����auCanParam
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


//-----�ڲ���������------------------------------------------------------------
//CAN�����ļ���ʼ��
#if 0
void CANConfig_Read_Param(void)
{
    uint8 i = 0,j = 0;
	uint8 *ptemp;

    CANConfig_ReadCANCFGFile_Init();     //��ȡCAN�����ļ�

	//�жϳ���У��
	if(auCanParam[3]<=CAN_PARAMGROUP_MAX_NUM)  //�˴����Դ��ļ���ȡ,��ΪУ������
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
** ��������: CANConfig_extract_Param
** ��������: ���յ�CAN�������ݽ��н������ж�׼ȷ��
** ��    ��: ��
** �䡡  ��: 1=�����쳣,0-��������
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: lxf
** �ա�  ��: 2020-05-22
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8 CANConfig_extract_Param(void)
{
    uint8 i = 0,j = 0;
	uint8 *ptemp;


	//�жϳ���У��
	if(auCanParam[3]<=CAN_PARAMGROUP_MAX_NUM)  //�˴����Դ��ļ���ȡ,��ΪУ������
    {
        g_stuCanProjects.ucParamGroupNum = auCanParam[3];
		if(g_stuCanProjects.ucParamGroupNum==0)
			return 0;   //Ϊ0��������ִ��
	}
	else
		return 1;   //�쳣
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
			return 1;   //TLV���Ⱥ����ݲ���
		
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
** ��������: CANConfig_Read_CanParam
** ��������: ��ȡCAN���ò��������д��
** ��    ��: ��
** �䡡  ��: �������� ����=1 ��ʾ��ǰ�޲�������
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: lxf
** �ա�  ��: 2020-05-22
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
	if(g_stuCanProjects.ucParamGroupNum==0)   //�޲�����
		return 1;
	else
	{
        return usdatalen;
	}
}

//CAN���ó�ʼ������
void CanConfig_Param_Init(void)
{
    uint8 i = 0,j = 0;
	
//��Ҫ�����е�����CAN��ȡ������ΪЭ��������������
    for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)
    {
        if(g_stuCanProjects.stuParamGroup[i].usSamplingFreq<10)
			g_stuCanProjects.stuParamGroup[i].usSamplingFreq = 10;
		if(g_stuCanProjects.stuParamGroup[i].usSamplingFreq<500)    //??500����50(��λ��10ms)
			g_stuCanProjects.agrouptype[i] = 1;   //��Ҫ�ɼ���5���ϱ�
		else
			g_stuCanProjects.agrouptype[i] = 2;   //���������ɼ�Ƶ���ϱ�
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
** ��������: CAN_GetFrameSave_Flag
** ��������: �����ϴ�CAN֡����ʱ����㵱ǰʱ����յ���CAN֡�Ƿ��ڹ涨�ɼ�ʱ����
** 
** ��    ��: uiOldtime:CAN֡�ϴα���ʱ��,uiNewtime:���ν��յ�CAN֡ʱ��  ��λms
             usCollectTime:CAN֡�ɼ�Ƶ�ʵ�λ10ms
** ��    ��: ��
** ��    ��: 0-����Ҫ����,1-��Ҫ����
**
** ��    ��: lxf
** ��    ��: 2019-09-26
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
//��������CAN֡ʱ��,���ʱ���ֵ����5�� �����¼�ʱΪ0
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
//��������CAN֡ʱ��,���ʱ���ֵ����300�� ���ط���0xFFFFFFFF
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
** ��������: CAN_ucOptionByte_Function
** ��������: �����ϴ�CAN����ѡ���� ѡȡCNA֡�ϱ�����
** 
** ��    ��: STU_Can_Message:����CAN֡����,pOutdata:���CAN֡����, ucOptionbyte:CAN֡����ѡ����
** ��    ��: ��
** ��    ��: ��Ҫ�ɼ������ݳ���
**
** ��    ��: lxf
** ��    ��: 2019-09-26
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
	uint8 j = 0;    //���㵥��CAN֡��Ҫ�ɼ������ݳ���
    uint8 uccanlen = 0; //����CAN֡��Ҫ��ȡ�ĳ���
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
** ��������: CAN_GetCacheLocation_Function
** ��������: ����CAN֡���ڲ�������λ��+����ѡ���� �����CAN���ϱ�������ʼλ��
** 
** ��    ��: ucGroupIndex:��������� ��0��ʼ,   ucCanNumIndex:�������е�ǰCAN֡��� ��0��ʼ
** ��    ��: ��
** ��    ��: ��Ҫ���浱ǰCAN֡����λ�� ��0��ʼ
**
** ��    ��: lxf
** ��    ��: 2019-12-7
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 CAN_GetCacheLocation_Function(uint8 ucGroupIndex,uint8 ucCanNumIndex)
{
    uint16 usvalue = 0;
    uint8 i = 0;
	uint8 j = 0;    //���㵥��CAN֡��Ҫ�ɼ������ݳ���
    uint8 uccanlen = 0; //����CAN֡��Ҫ��ȡ�ĳ���
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

    //���㵱ǰʱ�� ת��Ϊms   �Ƿ���Ҫ�ж�ʱ����Ч��?
    uiNewtime = (uint32)(stuDate.ucMin*60 +stuDate.ucSec)*1000 + stuDate.usmSec;
	stuCanMessage.usmSec = (uint16)(ptr[0]&0x07) + ptr[1];
	stuCanMessage.CanID = (uint32)(ptr[2]<<24) + (uint32)(ptr[3]<<16) +(uint32)(ptr[4]<<8) + ptr[5];
    memcpy(&stuCanMessage.data[0],&ptr[6],8);  
	//������1
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)    //ȷ�����Ĳ�����
	{
        for(n=0;n<g_stuCanProjects.stuParamGroup[i].ucCanNum;n++)    //ȷ�ϲ����������һ��CAN֡
        {
                 //����CAN֡����
            if(stuCanMessage.CanID==g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].uiID)  //ȷ�ϲ����������һ��CAN֡
            {
                //�ж�ʱ��
                ucSaveFlag = CAN_GetFrameSave_Flag(g_stuCanProjects.stuParamGroup.stuCanFrame[n].uiTime, uiNewtime);
                if(ucSaveFlag==0)
					return;
				//���ݿ���                //�������鱣��ָ��λ��
				//�����жϱ��浽�Ǹ��ϱ�CAN���黺��
				for(m=0;m<CAN_PARAMGROUP_MAX_NUM;m++)
				{
			    	if((g_stuCanProjects.stuParamGroup[i].usTlvName+0x1000)==g_stuCanDatUp[m].usTlvName)
			    	{
			    	
			    	 //�жϻ��泤���Ƿ����
			    	    ucdatalen = CAN_ucOptionByte_Function(stuCanMessage.data, &abuff[0]], g_stuCanProjects.stuParamGroup.stuCanFrame[n].ucOptionByte);
                        if(ucdatalen<=(CAN_DATA_MAX_LEN - g_stuCanDatUp[m].stuCanData[0].usDataLen))
                        {
                            memcpy(g_stuCanDatUp[m].stuCanData[0].aData[g_stuCanDatUp[m].stuCanData[0].usDataLen],&abuff[0],ucdatalen);
    					//	g_stuCanDatUp[m].stuCanData[0].usDataptr += ucdatalen;
    						g_stuCanDatUp[m].stuCanData[0].usDataLen += ucdatalen;
						//����ʱ��
						    g_stuCanProjects.stuParamGroup.stuCanFrame[n].uiTime = uiNewtime;
						    if(g_stuCanProjects.agrouptype[m]==1)    //��Ҫ���������Ĳ�����
								g_stuCanProjects.agroupSendFlag[m] = 1;
						}	
						else   //��ǰ�洢��������,�Ƿ������һ������
						{
                            g_stuCanProjects.agroupSendFlag[m] = 1;   //Ҫ���ǲɼ�Ƶ������֡���
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
** ��������: CanConfig_Data_Collect
** ��������: �Խ��յ���ÿ��CAN֡�����ж��Ƿ���CAN���ò������е���Ҫ�ɼ���
** 
** ��    ��: stuDate-��ǰCAN֡��ʱ��; ptr-�ɼ�����CAN֡����: 2byteʱ��+8byteCAN����
** ��    ��: ��
** ��    ��: 
**
** ��    ��: lxf
** ��    ��: 2019-09-26
**-----------------------------------------------------------------------------
*******************************************************************************/
//CAN���ݲɼ���� �ɼ�Ƶ�ʼ���,CAN֡���������, CAN���ݸ�ʽ���
void CanConfig_Data_Collect(STU_DataTime stuDate, uint8 *ptr)
{
    uint8 i = 0;    //�ĸ������� ���ĸ��ϱ������黺��
	uint8 n = 0;    //������ĵڼ���CAN֡
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
	
    //CAN֡ID
	stuCanMessage.CanID = (uint32)(ptr[0]<<24) + (uint32)(ptr[1]<<16) +(uint32)(ptr[2]<<8) + ptr[3];
    //���㵱ǰʱ�� ת��Ϊms   �Ƿ���Ҫ�ж�ʱ����Ч��?
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
	//�ж��Ƿ���Ҫ�ɼ���������			
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
			            g_stuCanProjects.agroupSendFlag[i] = 1;   //��������	
			        g_stuCanDatUp[i].ustimer = 0;
					//uiOldtime[i] = uiNewtime;
				    memcpy((uint8*)&stutime,(uint8*)&stuDate,6);    //����������һ���ϱ��Ĳɼ���ʼʱ��				  �m
				}
    	    }
			else
			{
			    memcpy((uint8*)&g_stuCanDatUp[i].Date,(uint8*)&stuDate,6);
				if(g_stuCanProjects.aucCanRcvFlag[i]==1)
				    g_stuCanProjects.agroupSendFlag[i] = 1;       //��������	
				g_stuCanDatUp[i].aCount[i] = 0;
				g_stuCanDatUp[i].ustimer = 0;
				//uiOldtime[i] = uiNewtime;
			}
			uiOldtime[i] = uiNewtime;
        } 		
	}
		
	//uiOldtime = uiNewtime; //����
	//������1
	for(i=0;i<g_stuCanProjects.ucParamGroupNum;i++)    //ȷ�����ĸ�������
	{
        for(n=0;n<g_stuCanProjects.stuParamGroup[i].ucCanNum;n++)    //ȷ�ϲ����������һ��CAN֡
        {
            //����CAN֡����
            if(stuCanMessage.CanID==g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].uiID)  //ȷ�ϲ����������һ��CAN֡
            {
                //���㵱ǰCAN֡ Ӧ�ñ��ֻ����λ��
				usPosition = CAN_GetCacheLocation_Function(i, n);
				//���㵱ǰCAN֡ ��Ҫ����ĳ���(4byteID+1byteѡ��λ+��������)
			    ucdatalen = CAN_OptionByte_Function(stuCanMessage, &abuff[0], g_stuCanProjects.stuParamGroup[i].stuCanFrame[n].ucOptionByte);

                //�ж� �Ƿ���Ҫ���漸������
                //CAN֡���水�ղ�����˳���Ӧ����
                memcpy(&g_stuCanDatUp[i].aData[usPosition + g_stuCanDatUp[i].aCount[i]*g_stuCanDatUp[i].usDataLen],&abuff[0],ucdatalen);	
                g_stuCanProjects.aucCanRcvFlag[i] = 1;   //���յ�����CAN֡
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
	
	*buf++ = 0x00;				//�������ͳ���
	*buf++ = 0x02;
	*buf++ = 'P';
	*buf++ = 'T';
	buf++;						//�������ݳ���
	buf++;
	buf++;						//״̬ͬ��TLV���� 

	usMsgBodyLen = 7;	
    
    *buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvName>>8);   //TLV1-T���ֽ�
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvName&0xFF); //TLV1-T���ֽ�
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvLen>>8);    //TLV1-L���ֽ�
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usTlvLen&0xFF);  //TLV1-L���ֽ�
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usSamplingFreq>>8); //�ɼ�Ƶ��-���ֽ�
	*buf++ = (uint8)(g_stuCanDatUp[ucCanParamGroupNum].usSamplingFreq&0xFF);//�ɼ�Ƶ��-���ֽ�
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
	//�Ƿ��жϳ��� �Ƿ�Ϊ0?
    usMsgBodyLen += 13+usTemplen;  //���ݳ���
    ucTlvNmr++;                    //TLV  ����     
    
	p += 4;
	usTemp = usMsgBodyLen - 6;     //ȥ��ǰ6���ֽ�
	*p++ = (usTemp >> 8) & 0xff;
	*p++ =  usTemp       & 0xff;
	*p = ucTlvNmr;

    return usMsgBodyLen;
}


//�����ϱ�Ƶ�ʴ��� 500msִ��һ��
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
	
    //���жϷ��ͱ�ʶ���Ϊ1  �ڲ鿴����������һ��������
    //���͹���0��־������
	ucGroupNum++;
	if(ucGroupNum>g_stuCanProjects.ucParamGroupNum)
		ucGroupNum = 1;	
    if(g_stuCanProjects.agroupSendFlag[ucGroupNum-1]!=1)
    {    
        return FALSE;
    }

    //���
    //1.������1
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
    //������ͱ�־ 
	g_stuCanProjects.agroupSendFlag[ucGroupNum-1] = 0;
    g_stuCanProjects.aucCanRcvFlag[ucGroupNum-1] = 0;
	return TRUE;
}



