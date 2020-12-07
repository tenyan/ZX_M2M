/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: MX25L1606.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �洢��MX25L1606�����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-04-10, by , �������ļ�
 *
 */

#include "config.h"
#include "MX25L1606Mem.h"


uint8 m_aucFlashBuf[4096];
OS_EVENT *g_SpiSem;

/******************************************************************************
** ��������: Flash_SPI_Config
** ��������: ��ʼ��SPI����
**
** ��    ��: N/A
** �䡡  ��: N/A
** ��    ��:
**
** ����  ��: hhm
** �ա�  ��: 2011��02��17��
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
    	g_SpiSem=OSSemCreate(1);   //�����洢��֮��Ļ����ź��� 
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
** ��������: ByteWrite
** ��������: ���ֽ�д����
**
** ��    ��: data ��д�ֽ�
** �䡡  ��: N/A
** ��    ��: ��ȷ����TRUE; �쳣����FALSE
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
**-----------------------------------------------------------------------------
*******************************************************************************/
static BOOL ByteWrite(uint8 data)
{
    sFLASH_SendByte(data);
    return TRUE;

}

/******************************************************************************
** ��������: ByteRead
** ��������: ���ֽڶ�����
**
** ��    ��: N/A
** �䡡  ��: *pucOut,���������ֽ�
** ��    ��: ��ȷ����0; ��ָ�뷵��1; ���쳣����2
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
**-----------------------------------------------------------------------------
*******************************************************************************/
static uint8 ByteRead(uint8 *pucOut)
{
    *pucOut=sFLASH_SendByte(sFLASH_DUMMY_BYTE);
    return TRUE;
}

/******************************************************************************
** ��������: WriteEnable
** ��������: дʹ�ܺ���
**
** ��    ��: N/A
** �䡡  ��: N/A
** ��    ��:
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
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
** ��������: ReadStatusRegister
** ��������: ��A25L080״̬�Ĵ���
**
** ��    ��: N/A
** �䡡  ��: ״̬�Ĵ���ֵ
** ��    ��:
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
**-----------------------------------------------------------------------------
*******************************************************************************/
#if 0
BOOL ReadStatusRegister(uint8 *pucOut)
{
    uint8  data=0;
	
    while(1)
    {	
	    sFLASH_CS_LOW();
        ByteWrite(0x05);   	//��״̬������
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
        ByteWrite(0x05);   	//��״̬������
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
** ��������: WriteCmdAndAddr
** ��������: ��װ�����ַ
**
** ��    ��: Cmd�� ������
             PageAddr��ҳ��ֵַ
             BufAddr��ҳ�ڵ�ֵַ
** �䡡  ��: N/A
** ��    ��:
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
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
** ��������: WriteInOnePage
** ��������: ҳ��д���ݣ���������ʱ��ȷ��BufAddr+Length<256
**
** ��    ��: PageAddr�� ҳ��ַ
             BufAddr��ҳ�ڵ�ַ
             Length�����ݳ���
             *p ������ָ��
** �䡡  ��: N/A
** ��    ��: �ɹ�����0,ʧ�ܷ�����Ӧ������
** ����  ��:   yzjlg
** �ա�  ��: 2011��02��17��
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteInOnePage(uint16 PageAddr,uint8 BufAddr,uint16 Length, uint8 *p)
{
	uint8  dummy1;
	//uint16 i=0;
	//uint8  aBuff[256]={0};
	
	
	ReadStatusRegister(&dummy1);    //д��
	WriteEnable();
	sFLASH_CS_LOW();
    WriteCmdAndAddr(0x02,PageAddr,0);
    while(Length--)   //ҳ���
	{
	    ByteWrite(*p++);
    	
	}
	sFLASH_CS_HIGH();
	return 0;
	

}

/******************************************************************************
** ��������: WriteInManyPage
** ��������: ����д��ҳ����
**
** ��    ��: PageAddr����ʼ ҳ��ַ
             BufAddr����ʼҳ��ƫ�Ƶ�ַ
             Length�����ݳ���
             *p ������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��:   yzjlg
** �ա�  ��: 2011��02��17��
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
    if(len > 0) //��ֹд������
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
        //��������
        ReadFromFlash(PageAddr&0xfff0,0,4096,(unsigned char *)m_aucFlashBuf);
        //���»�����
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
        //������
        //SectorErase(PageAddr/16);

        ReadStatusRegister(&dummy);   
	    WriteEnable();//дʹ��  
	    sFLASH_CS_LOW();
	    WriteCmdAndAddr(0x20, PageAddr, 0); //����
	    sFLASH_CS_HIGH();
		
        //д��
        for(i=0;i<4096;i+=256)
        {
            WriteInOnePage(PageAddr++,0,256,&m_aucFlashBuf[i]);
        }
    }
	return 0;
}

/******************************************************************************
** ��������: WriteFlash
** ��������: дFLASH
**
** ��    ��: uiAddress,FLASH���Ե�ַ,��0��ʼ; pucData,����ָ��, uiLength,���ݳ���
** ��    ��: ��
** ��    ��: �ɹ�����0,ʧ�ܷ�����Ӧ������
             0�ɹ�,1�ź�������,2�洢������ʧ��,3�洢��Խ��
**
** ��    ��: ����ƽ
** ��    ��: 2011-09-27
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteFlash(uint32 uiAddress, const uint8 *pucData, uint32 uiLength)
{
    uint16 usPageAddr = 0;
	uint8  ucOffsetAddr = 0;
	
    if(uiAddress+uiLength-1>TS25L16CAPACITY)  //�洢Խ��
	{
	    return 3;
	}

	usPageAddr = uiAddress>>8;                //����õ�ҳ��ַ
	ucOffsetAddr = uiAddress%8;               //����õ�ҳ��ƫ�Ƶ�ַ

	return WriteToFlash(usPageAddr, ucOffsetAddr, uiLength, (uint8*)pucData);
}


/******************************************************************************
** ��������: ReadFromFlash
** ��������: ��������ҳ����
**
** ��    ��: PageAddr�� ��ʼҳ��ַ
             BufAddr����ʼҳ��ƫ�Ƶ�ַ
             Length�����ݳ���
             *buf ��������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
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
** ��������: ReadFlash
** ��������: ��FLASH����
**
** ��    ��: uiAddress,FLASH���Ե�ַ, uiLength,���������ݳ���
** ��    ��: *pucBuff,���ݴ�Ż�����
** ��    ��:
**
** ��    ��: ����ƽ
** ��    ��: 2011-09-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFlash(uint32 uiAddress, uint8 *pucBuff, uint32 uiLength)
{
    uint16 usPageAddr = 0;
	uint8  ucOffsetAddr = 0;

    if(uiAddress+uiLength-1>TS25L16CAPACITY)  //�洢Խ��
	{
	    return 3;
	}

	usPageAddr = uiAddress>>8;                //����õ�ҳ��ַ
	ucOffsetAddr = uiAddress%8;               //����õ�ҳ��ƫ�Ƶ�ַ

	return ReadFromFlash(usPageAddr, ucOffsetAddr, uiLength, pucBuff);
}

#if 0
/******************************************************************************
** ��������: PageErase
** ��������: ҳ��������
**
** ��    ��: Page,ҳ��ַ;���ֵΪ2��13�η�
** ��    ��: ��
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ��    ��: ����ƽ
** ��    ��: 2011-08-22
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
** ��������: SectorErase
** ��������: �β�������
**
** ��    ��: Sector
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: 
** �ա�  ��: 2011��02��18��
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
** ��������: BlockErase
** ��������: оƬ��������
**
** ��    ��: Block
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: 
** �ա�  ��: 2011��02��18��
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
** ��������: ChipErase
** ��������: оƬ��������
**
** ��    ��: N/A
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: 
** �ա�  ��: 2011��02��18��
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
//-----�ļ�TS25L16Mem.c����----------------------------------------------------



















