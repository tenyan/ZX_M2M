/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: PcDebug.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 本文件为PcDebug功能模块协议层处理的文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by  lxf, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
#include "PcHardware.h"

//-----外部变量定义------------------------------------------------------------


#define DEBUG_DMA_UART_Tx_BSIZE        1500
uint8 DEBUG_DMA_UART_Tx_Buff[DEBUG_DMA_UART_Tx_BSIZE];

#define DEBUG_DMA_UART_Rx_BSIZE        1500
uint8 DEBUG_DMA_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];

uint8 DEBUG_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];
uint16 DEBUG_UART_Rx_Cnt;


OS_EVENT   *UartDEBUGReadSem;   //gps串口接收信号量
OS_EVENT   *UartDEBUGWriteSem;   //gps串口发送信号量



int fputc(int ch, FILE *f){

	/* 将Printf内容发往串口 */
	USART_GetFlagStatus(USART1,USART_FLAG_TC);
	USART_SendData(USART1, (unsigned char) ch);

//	while (!(USART1->SR & USART_FLAG_TXE));

	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);	
	return (ch);

}

BOOL ReadDebugUartData(uint8 **data, uint16* len)
{
	uint8 err = 0;
	
	OSSemPend(UartDEBUGReadSem, 0, &err);
	if(0==err)
	{
		*data = DEBUG_UART_Rx_Buff;
		*len = DEBUG_UART_Rx_Cnt;
		return TRUE;
	}
	else
		return FALSE;
}

void DEBUG_UART_Write(uint8 *Data, uint16 len)
{
    uint8 err = 0;

	if((0==len) || (len>DEBUG_DMA_UART_Tx_BSIZE))
		return;
    OSSemPend(UartDEBUGWriteSem, 0, &err);
    memcpy(DEBUG_DMA_UART_Tx_Buff, Data, len);
   	DMA_SetCurrDataCounter(DMA2_Stream7, len); 	// 设置要发送的字节数目
	DMA_Cmd(DMA2_Stream7, ENABLE);        		//开始DMA发送
}



void USART1_DMA_Config(void)  
{  
    DMA_InitTypeDef DMA_InitStructure;  
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//开启DMA时钟 

	//接收DMA
    DMA_DeInit(DMA2_Stream5);

    DMA_InitStructure.DMA_Channel = DMA_Channel_4;//通道4   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DEBUG_DMA_UART_Rx_Buff; //内存地址  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存  
    DMA_InitStructure.DMA_BufferSize = DEBUG_DMA_UART_Rx_BSIZE; //缓冲大小  
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
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);//初始化dma2流5  
    DMA_Cmd(DMA2_Stream5, ENABLE);//开启dma2流5  

	//发送DMA配置
	DMA_InitStructure.DMA_Channel =DMA_Channel_4;         
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
	        
	DMA_DeInit(DMA2_Stream7);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR); 
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DEBUG_DMA_UART_Tx_Buff;
	DMA_InitStructure.DMA_BufferSize = DEBUG_DMA_UART_Tx_BSIZE;         
	DMA_Init(DMA2_Stream7,&DMA_InitStructure);
	//DMA_Cmd(UART3_TX_DMAy_Streamx, ENABLE);/* move to rt_serial_enable_dma() by RTT */
	        
	DMA_ITConfig(DMA2_Stream7, DMA_IT_TC | DMA_IT_TE, ENABLE);
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7| DMA_FLAG_TEIF7);
}  

//串口1相关配置及空闲中断配置：
void UART1_Config(void)  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    NVIC_InitTypeDef   NVIC_InitStructure;  
    USART_InitTypeDef USART_InitStructure;   
    /* Enable GPIO clock PA9 PA10*/  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  
    /* Enable UART clock */  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  
    /* Connect PXx to USARTx_Tx PA9*/  
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);  
    /* Connect PXx to USARTx_Rx PA10*/  
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  
    /* Configure USART Tx as alternate function  */  
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;  
    GPIO_Init(GPIOA, &GPIO_InitStructure);  
    /* Configure USART Rx as alternate function  */  
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
    USART_InitStructure.USART_BaudRate = 115200;  
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
    USART_InitStructure.USART_StopBits = USART_StopBits_1;  
    USART_InitStructure.USART_Parity = USART_Parity_No;  
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
    /* USART configuration */  
    USART_Init(USART1, &USART_InitStructure);  
	
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 11;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 12;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
    NVIC_Init(&NVIC_InitStructure); 
	
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);  
    /* Enable USART */  
    USART_Cmd(USART1, ENABLE);  
}  
//串口相关配置总函数：
void DEBUG_Uart_Init(void)  
{  
	if(NULL==UartDEBUGReadSem)
		UartDEBUGReadSem = OSSemCreate(0);
	if(NULL==UartDEBUGWriteSem)
		UartDEBUGWriteSem = OSSemCreate(1);
  	UART1_Config();  
  	USART1_DMA_Config();//主要是配置外设地址和内存地址以及缓冲区大小，配置好后DMA就会自动的把串口数据存到相应的内存地址。  
  	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//配置串口1DMA接收  
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //开启串口DMA发送
}  
//串口1空闲中断服务函数：
void USART1_IRQHandler(void)  
{  
    u16 i;  
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//如果为空闲总线中断  
    {  
        i = USART1->SR;  
        i = USART1->DR;  
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);  
        DMA_Cmd(DMA2_Stream5, DISABLE);//关闭DMA,防止处理其间有数据  
        //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);	//配置串口1DMA接收  
        DEBUG_UART_Rx_Cnt = DEBUG_DMA_UART_Rx_BSIZE - DMA_GetCurrDataCounter(DMA2_Stream5);
		memcpy(DEBUG_UART_Rx_Buff, DEBUG_DMA_UART_Rx_Buff, DEBUG_UART_Rx_Cnt);
		DMA_SetCurrDataCounter(DMA2_Stream5, DEBUG_DMA_UART_Rx_BSIZE);
		DMA_Cmd(DMA2_Stream5, ENABLE);//开启DMA
        //USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//配置串口1DMA接收  
        OSSemPost(UartDEBUGReadSem);
    }  
}  
/*********************DMA中断服务函数****************************/
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
    {
     	OSSemPost(UartDEBUGWriteSem);
    }
    /* clear DMA flag */
    DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7 | DMA_FLAG_TEIF7);
}

//-----文件PcDebug.c结束---------------------------------------------
