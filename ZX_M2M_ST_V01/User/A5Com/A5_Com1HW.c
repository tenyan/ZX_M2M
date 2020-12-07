/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: A5_Com1HW.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为uart6
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-3-22, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"


//-----外部变量定义------------------------------------------------------------
#define RS232_BRT		115200
#define RS485_BRT		RS232_BRT

#define RS232_DMA_UART_Tx_BSIZE        1300
uint8 RS232_DMA_UART_Tx_Buff[RS232_DMA_UART_Tx_BSIZE];

#define RS232_DMA_UART_Rx_BSIZE        1300
uint8 RS232_DMA_UART_Rx_Buff[RS232_DMA_UART_Rx_BSIZE];

uint8 RS232_UART_Rx_Buff[RS232_DMA_UART_Rx_BSIZE];
uint16 RS232_UART_Rx_Cnt;


OS_EVENT   *UartRS232ReadSem;    //串口接收信号量
OS_EVENT   *UartRS232WriteSem;   //串口发送信号量


BOOL ReadRS232UartData(uint8 **data, uint16* len)
{
	uint8 err = 0;
	
	OSSemPend(UartRS232ReadSem, 0, &err);
	if(0==err)
	{
		*data = RS232_UART_Rx_Buff;
		*len = RS232_UART_Rx_Cnt;
		return TRUE;
	}
	else
		return FALSE;
}

void RS232_UART_Write(uint8 *Data, uint16 len)
{
    uint8 err = 0;
    if((0==len) || (len>RS232_DMA_UART_Tx_BSIZE))
		return;
    OSSemPend(UartRS232WriteSem, 0, &err);
    memcpy(RS232_DMA_UART_Tx_Buff, Data, len);
   	DMA_SetCurrDataCounter(DMA2_Stream6, len); 	// 设置要发送的字节数目
	DMA_Cmd(DMA2_Stream6, ENABLE);        		//开始DMA发送
}
/*
void RS485_UART_Write(uint8 *Data, uint16 len)
{
	RS485_CON_SEND();
	OSTimeDly(OS_TICKS_PER_SEC/1000);
	RS232_UART_Write(Data, len);
	OSTimeDly((OS_TICKS_PER_SEC/1000) * (2+len*10*1000/RS485_BRT));
	RS485_CON_RCV();
}
*/
void USART6_DMA_Config(void)  
{  
    DMA_InitTypeDef DMA_InitStructure;  
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//开启DMA时钟 

	//接收DMA
	DMA_DeInit(DMA2_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;//通道5   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RS232_DMA_UART_Rx_Buff; //内存地址  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存  
    DMA_InitStructure.DMA_BufferSize = RS232_DMA_UART_Rx_BSIZE; //缓冲大小  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址增  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据大小1byte  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//内存数据大小1byte  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//优先级高  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;// 不开fifo  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//  
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化dma2流1  
    DMA_Cmd(DMA2_Stream1, ENABLE);//开启dma2流1  

	//发送DMA配置
	DMA_InitStructure.DMA_Channel =DMA_Channel_5;         
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /* Specifies whether the Peripheral address register should be incremented or not */
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; /* Specifies whether the memory address register should be incremented or not */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; 
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; 
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable; 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; 
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single; 
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; 
	        
	DMA_DeInit(DMA2_Stream6);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RS232_DMA_UART_Tx_Buff;;   
	DMA_InitStructure.DMA_BufferSize = RS232_DMA_UART_Tx_BSIZE;         
	DMA_Init(DMA2_Stream6,&DMA_InitStructure);
	//DMA_Cmd(UART3_TX_DMAy_Streamx, ENABLE);/* move to rt_serial_enable_dma() by RTT */
	        
	DMA_ITConfig(DMA2_Stream6, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA2_Stream6, DMA_FLAG_TCIF6| DMA_FLAG_TEIF6);
}  

//串口1相关配置及空闲中断配置：
void UART6_Config(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef   NVIC_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
    /* Enable GPIO clock */  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
    /* Enable UART clock */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);  
    /* Connect PXx to USARTx_Tx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);  
    /* Connect PXx to USARTx_Rx*/  
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);  

	/*USART_DeInit(USART6); */
	
    /* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
    /* Configure USART Rx as alternate function  */  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
    USART_InitStructure.USART_BaudRate = RS232_BRT;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    /* USART configuration */  
    USART_Init(USART6, &USART_InitStructure);  
	
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 13;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 14;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
	
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);  
    /* Enable USART */  
    USART_Cmd(USART6, ENABLE);  
}  
//串口相关配置总函数：
void RS232_Uart_Init(void)  
{ 

	if(NULL==UartRS232ReadSem)
		UartRS232ReadSem = OSSemCreate(0);
	if(NULL==UartRS232WriteSem)
		UartRS232WriteSem = OSSemCreate(1);

  	UART6_Config();  
  	USART6_DMA_Config();//主要是配置外设地址和内存地址以及缓冲区大小，配置好后DMA就会自动的把串口数据存到相应的内存地址。  
  	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);	//配置串口6DMA接收  
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);  //开启串口DMA发送
}  
//串口1空闲中断服务函数：
void USART6_IRQHandler(void)  
{  
    u16 i;  
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)//如果为空闲总线中断  
    {  
        i = USART6->SR;  
        i = USART6->DR;  
        USART_ClearITPendingBit(USART6, USART_IT_IDLE);  
        DMA_Cmd(DMA2_Stream1, DISABLE);//关闭DMA,防止处理其间有数据  
        //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);	//配置串口1DMA接收  
        RS232_UART_Rx_Cnt = RS232_DMA_UART_Rx_BSIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
		memcpy(RS232_UART_Rx_Buff, RS232_DMA_UART_Rx_Buff, RS232_UART_Rx_Cnt);
		DMA_SetCurrDataCounter(DMA2_Stream1, RS232_DMA_UART_Rx_BSIZE);
		DMA_Cmd(DMA2_Stream1, ENABLE);//开启DMA
        //USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//配置串口1DMA接收  
        OSSemPost(UartRS232ReadSem);
    }  
}  
/*********************DMA中断服务函数****************************/
void DMA2_Stream6_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream6,DMA_IT_TCIF6))
    {
     	OSSemPost(UartRS232WriteSem);
    }
    /* clear DMA flag */
    DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6);
}

#if 0
void McuIOPinInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//PD12-- PWRC_485  PD15--485_DE
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Configure PD12 PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	POWERON_485();
	RS485_CON_RCV();

	//PE4--CAN_BUS1_LED PE5--CAN_BUS2_LED
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Configure PE4 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure); 
	CAN1_LED_ON();
	CAN2_LED_ON();
	BLU_LED_ON();	
}

void McuHwInit(void)
{
	//McuIOPinInit();
	RS232_Uart_Init();
}

// UART 配置为GPIO输入,关UART时钟，主要为在休眠时降低功耗
void Mcu_Uart_Deinit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;  
	
	/*USART_DeInit(USART6);*/
	/* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC, &GPIO_InitStructure);  
	
    /* Configure USART Rx as alternate function  */  
  //  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  
  //  GPIO_Init(GPIOD, &GPIO_InitStructure);  
}
#endif



