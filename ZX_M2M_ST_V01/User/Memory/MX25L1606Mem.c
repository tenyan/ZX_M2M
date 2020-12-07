/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: MX25L1606.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 存储器MX25L1606驱动文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-04-10, by , 创建本文件
 *
 */

#include "config.h"
#include "MX25L1606Mem.h"


uint8 m_aucFlashBuf[4096];
OS_EVENT *g_SpiSem;

/******************************************************************************
** 函数名称: Flash_SPI_Config
** 功能描述: 初始化SPI外设
**
** 输    入: N/A
** 输　  出: N/A
** 返    回:
**
** 作　  者: hhm
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
void Flash_SPI_Config(void)
{
   	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	/* Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* Enable the GPIOB clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable the GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Connect PB10 to SPI2_SCK */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
	/* Connect PC2 to SPI2_MISO */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);
	/* Connect PC3 to SPI2_MOSI */
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* PB9 as CS PIN */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	sFLASH_CS_HIGH();
	
	/* SPI configuration *****************************************/
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);
	SPI_Cmd(SPI2, ENABLE);

	
    if(NULL==g_SpiSem)
    	g_SpiSem=OSSemCreate(1);   //创建存储器之间的互斥信号量 
}

void SPISleep(void)
{
	#if 0
	FIO1DIR &= ~BIT(4);
	//PINSEL0 &= ~(BIT(30)+BIT(31));
	PINSEL0 &= 0x3fffffff;
	PINSEL1 &= ~(BIT(2)+BIT(3)+BIT(4)+BIT(5));
	FIO0DIR &= ~(BIT(15)+BIT(17)+BIT(18));
	#endif
}

void SPIWake(void)
{
	#if 0
	FIO1DIR |= BIT(4);
	//PINSEL0 |= (BIT(30)+BIT(31));
	PINSEL0 |= 0xc0000000;
	PINSEL1 |= (BIT(2)+BIT(3)+BIT(4)+BIT(5));
	#endif
}
#if 0
uint8_t sFLASH_SendByte(uint8_t byte)
{
    /*!< Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2, byte);

    /*!< Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI2);
}
#endif
#define MAX_RETRY		5000
uint8_t sFLASH_SendByte(uint8_t byte)
{
	uint16 i;
    /*!< Loop while DR register in not emplty */
	for(i=0;i<MAX_RETRY; i++)
	{
		if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) != RESET)
			break;
	}

    /*!< Send byte through the SPI1 peripheral */
    SPI_I2S_SendData(SPI2, byte);

    /*!< Wait to receive a byte */
   	for(i=0;i<MAX_RETRY; i++)
	{
		if(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) != RESET)
			break;
   	}
    /*!< Return the byte read from the SPI bus */
    return SPI_I2S_ReceiveData(SPI2);
}


/******************************************************************************
** 函数名称: ByteWrite
** 功能描述: 单字节写函数
**
** 输    入: data 所写字节
** 输　  出: N/A
** 返    回: 正确返回TRUE; 异常返回FALSE
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
static BOOL ByteWrite(uint8 data)
{
    sFLASH_SendByte(data);
    return TRUE;

}

/******************************************************************************
** 函数名称: ByteRead
** 功能描述: 单字节读函数
**
** 输    入: N/A
** 输　  出: *pucOut,所读到的字节
** 返    回: 正确返回0; 空指针返回1; 读异常返回2
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
static uint8 ByteRead(uint8 *pucOut)
{
    *pucOut=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
    return TRUE;
}

/******************************************************************************
** 函数名称: WriteEnable
** 功能描述: 写使能函数
**
** 输    入: N/A
** 输　  出: N/A
** 返    回:
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
static BOOL  WriteEnable()
{ 
	sFLASH_CS_LOW();
    ByteWrite(0x06);
	sFLASH_CS_HIGH();
	return TRUE;
}

/******************************************************************************
** 函数名称: ReadStatusRegister
** 功能描述: 读A25L080状态寄存器
**
** 输    入: N/A
** 输　  出: 状态寄存器值
** 返    回:
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
BOOL ReadStatusRegister(uint8 *pucOut)
{
    uint8  data=0;
	
    while(1)
    {	
	    sFLASH_CS_LOW();
        ByteWrite(0x05);   	//读状态命令字
        ByteRead(&data);
		sFLASH_CS_HIGH();
        if(0==(data&0x01)) 
    	{
    	    *pucOut = data;
	        break;
    	}		
    }  
	return TRUE;
}
#endif

BOOL ReadStatusRegister(uint8 *pucOut)
{
	extern uint32  g_uiSysTime;
    uint8  data=0;

	uint32 uiTemp;
	uint32 uiTemp1;
	
	uiTemp = g_uiSysTime;
    while(1)
	{	
	    sFLASH_CS_LOW();
        ByteWrite(0x05);   	//读状态命令字
        ByteRead(&data);
		sFLASH_CS_HIGH();
        if(0==(data&0x01)) 
    	{
    	    *pucOut = data;
	        break;
    	}

		if(g_uiSysTime >= uiTemp)
			uiTemp1 = g_uiSysTime - uiTemp;
		else
			uiTemp1 = 0xffffffff - uiTemp + g_uiSysTime;
		if(uiTemp1 > 100)
			break;
    }  
	return TRUE;
}

/******************************************************************************
** 函数名称: WriteCmdAndAddr
** 功能描述: 封装命令及地址
**
** 输    入: Cmd， 命令编号
             PageAddr，页地址值
             BufAddr，页内地址值
** 输　  出: N/A
** 返    回:
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
static BOOL WriteCmdAndAddr(uint8 Cmd,uint16 PageAddr,uint8 BufAddr)
{
	ByteWrite(Cmd);
	ByteWrite((uint8)(PageAddr>>8) & 0x1F);
	ByteWrite((uint8)((PageAddr<<8)>>8));
	ByteWrite(BufAddr);
	return TRUE;
}

/******************************************************************************
** 函数名称: WriteInOnePage
** 功能描述: 页内写数据，函数调用时请确保BufAddr+Length<256
**
** 输    入: PageAddr， 页地址
             BufAddr，页内地址
             Length，数据长度
             *p ，数据指针
** 输　  出: N/A
** 返    回: 成功返回0,失败返回相应错误码
** 作　  者:   yzjlg
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteInOnePage(uint16 PageAddr,uint8 BufAddr,uint16 Length, uint8 *p)
{
	uint8  dummy1;
	//uint16 i=0;
	//uint8  aBuff[256]={0};
	
	
	ReadStatusRegister(&dummy1);    //写回
	WriteEnable();
	sFLASH_CS_LOW();
    WriteCmdAndAddr(0x02,PageAddr,0);
    while(Length--)   //页编程
	{
	    ByteWrite(*p++);
    	
	}
	sFLASH_CS_HIGH();
	return 0;
	

}

/******************************************************************************
** 函数名称: WriteInManyPage
** 功能描述: 连续写多页数据
**
** 输    入: PageAddr，起始 页地址
             BufAddr，起始页内偏移地址
             Length，数据长度
             *p ，数据指针
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者:   yzjlg
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteToFlash(uint16 PageAddr,uint8 PageOffsetAddr,uint32 Length, uint8 *p)
{ 
   /* uint32 len=0,temp=0;
    uint8  ucResult = 0;
	
    temp=BufAddr+Length<=256 ? Length : 256-BufAddr;
    ucResult=WriteInOnePage(PageAddr, BufAddr, temp, p);
	if(ucResult != 0)
	{
	    return ucResult;
	}
	
    p += temp;
    len = Length-temp;
	
    while(len>256)
    {   
        ucResult=WriteInOnePage(++PageAddr, 0, 256, p);
		if(ucResult != 0)
    	{
    	    return ucResult;
    	}
	    len -= 256;
	    p += 256;
    }
    if(len > 0) //防止写空数据
    {
        ucResult=WriteInOnePage(++PageAddr, 0, len, p);
		if(ucResult != 0)
    	{
    	    return ucResult;
    	}
    }
	return 0;*/


    uint32 len=0;
    uint16  SectorOffset;
    uint16  i;
	uint8  dummy;

	IWdtFeed();	
    while(len<Length)
    {
        //读缓冲区
        ReadFromFlash(PageAddr&0xfff0,0,4096,(unsigned char *)m_aucFlashBuf);
        //更新缓冲区
        SectorOffset=(PageAddr&15)*256+PageOffsetAddr;
        PageOffsetAddr=0;
        PageAddr&=0xfff0;
        for(i=SectorOffset;i<4096;i++)
        {
            m_aucFlashBuf[i]=*p++;
            if(++len>=Length)
            {
                break;
            }
        }
        //区擦除
        //SectorErase(PageAddr/16);

        ReadStatusRegister(&dummy);   
	    WriteEnable();//写使能  
	    sFLASH_CS_LOW();
	    WriteCmdAndAddr(0x20, PageAddr, 0); //擦除
	    sFLASH_CS_HIGH();
		
        //写入
        for(i=0;i<4096;i+=256)
        {
            WriteInOnePage(PageAddr++,0,256,&m_aucFlashBuf[i]);
        }
    }
	return 0;
}

/******************************************************************************
** 函数名称: WriteFlash
** 功能描述: 写FLASH
**
** 输    入: uiAddress,FLASH绝对地址,从0开始; pucData,数据指针, uiLength,数据长度
** 输    出: 无
** 返    回: 成功返回0,失败返回相应错误码
             0成功,1信号量问题,2存储器操作失败,3存储器越界
**
** 作    者: 周卫平
** 日    期: 2011-09-27
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteFlash(uint32 uiAddress, const uint8 *pucData, uint32 uiLength)
{
    uint16 usPageAddr = 0;
	uint8  ucOffsetAddr = 0;
	
    if(uiAddress+uiLength-1>TS25L16CAPACITY)  //存储越界
	{
	    return 3;
	}

	usPageAddr = uiAddress>>8;                //计算得到页地址
	ucOffsetAddr = uiAddress%8;               //计算得到页内偏移地址

	return WriteToFlash(usPageAddr, ucOffsetAddr, uiLength, (uint8*)pucData);
}


/******************************************************************************
** 函数名称: ReadFromFlash
** 功能描述: 连续读多页数据
**
** 输    入: PageAddr， 起始页地址
             BufAddr，起始页内偏移地址
             Length，数据长度
             *buf ，缓冲区指针
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFromFlash(uint16 PageAddr,uint8 BufAddr,uint32 Length, uint8 *Buf)
{
	uint8 dummy;

	ReadStatusRegister(&dummy);
	sFLASH_CS_LOW();
	WriteCmdAndAddr(0x03,PageAddr,BufAddr);

	while(Length--)  
	{
		ByteRead(Buf++);
	}  
	sFLASH_CS_HIGH();
	return 0;
	
}

/******************************************************************************
** 函数名称: ReadFlash
** 功能描述: 读FLASH数据
**
** 输    入: uiAddress,FLASH绝对地址, uiLength,待读出数据长度
** 输    出: *pucBuff,数据存放缓冲区
** 返    回:
**
** 作    者: 周卫平
** 日    期: 2011-09-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFlash(uint32 uiAddress, uint8 *pucBuff, uint32 uiLength)
{
    uint16 usPageAddr = 0;
	uint8  ucOffsetAddr = 0;

    if(uiAddress+uiLength-1>TS25L16CAPACITY)  //存储越界
	{
	    return 3;
	}

	usPageAddr = uiAddress>>8;                //计算得到页地址
	ucOffsetAddr = uiAddress%8;               //计算得到页内偏移地址

	return ReadFromFlash(usPageAddr, ucOffsetAddr, uiLength, pucBuff);
}

#if 0
/******************************************************************************
** 函数名称: PageErase
** 功能描述: 页擦除函数
**
** 输    入: Page,页地址;最大值为2的13次方
** 输    出: 无
** 返    回: 0成功擦除,1信号量问题,2存储器操作失败
**
** 作    者: 周卫平
** 日    期: 2011-08-22
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 PageErase(uint16 Page)
{
/*
    uint8 err;
	uint8 dummy;
	OSSemPend(g_SpiSem, 0, &err);
	if(err != OS_NO_ERR)
	{
	    return 1;
	}

    FM25L16CS_SET();
    if(!ReadStatusRegister(&dummy))
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
	if(!WriteEnable())
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}

	MEMCS_CLR();
	if(!WriteCmdAndAddr(0xDB, Page, 0))
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
	MEMCS_SET();
	OSSemPost(g_SpiSem);
	return 0;
	*/
}

/******************************************************************************
** 函数名称: SectorErase
** 功能描述: 段擦除工作
**
** 输    入: Sector
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: 
** 日　  期: 2011年02月18日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 SectorErase(uint16 Sector)
{
/*
    uint8 err;
	uint8 dummy;
	OSSemPend(g_SpiSem, 0, &err);
	if(err != OS_NO_ERR)
	{
	    return 1;
	}

    FM25L16CS_SET();
    if(!ReadStatusRegister(&dummy))
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
	if(!WriteEnable())
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
    MEMCS_CLR();
    if(!ByteWrite(0x20))
	{
	    OSSemPost(g_SpiSem);
		return 2;
	}
	if(!ByteWrite(Sector>>4))
	{
	    OSSemPost(g_SpiSem);
		return 2;
	}
	
	if(!ByteWrite(Sector<<4 | 0x0F))
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
	if(ByteWrite(0xFF))
	{
	    OSSemPost(g_SpiSem);
		return 2;
	}
    MEMCS_SET();
	
	OSSemPost(g_SpiSem);
	return 0;
	*/
}

/******************************************************************************
** 函数名称: BlockErase
** 功能描述: 芯片擦除工作
**
** 输    入: Block
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: 
** 日　  期: 2011年02月18日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 BlockErase(uint8 Block)
{
/*
    uint8 err;
	uint8 dummy;
	OSSemPend(g_SpiSem, 0, &err);
	if(err != OS_NO_ERR)
	{
	    return 1;
	}

    FM25L16CS_SET();
    if(!ReadStatusRegister(&dummy))
	{
	    OSSemPost(g_SpiSem);
	    return 2;
	}
	if(!WriteEnable())
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
    MEMCS_CLR();
    if(!ByteWrite(0xD8))
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
	if(!ByteWrite(Block))
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
	if(!ByteWrite(0x00))
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
	if(!ByteWrite(0x00))
	{
		OSSemPost(g_SpiSem);
		return 2;
	}
    MEMCS_SET();
	
	OSSemPost(g_SpiSem);
	return 0;
	*/
}

/******************************************************************************
** 函数名称: ChipErase
** 功能描述: 芯片擦除工作
**
** 输    入: N/A
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: 
** 日　  期: 2011年02月18日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ChipErase(void)
{
/*
    uint8 err;
	uint8 dummy;
	OSSemPend(g_SpiSem, 0, &err);
	if(err != OS_NO_ERR)
	{
	    return 1;
	}

    FM25L16CS_SET();
    if(!ReadStatusRegister(&dummy))
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
	if(!WriteEnable())
	{
		OSSemPost(g_SpiSem);
	    return 2;
	}
    MEMCS_CLR();
	if(!ByteWrite(0xc7))
	{
	    OSSemPost(g_SpiSem);
		return 2;
	}
    MEMCS_SET();

	OSSemPost(g_SpiSem);
	return 0;
	*/
}
#endif
//-----文件TS25L16Mem.c结束----------------------------------------------------



















