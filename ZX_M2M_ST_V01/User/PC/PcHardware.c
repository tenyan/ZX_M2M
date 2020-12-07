/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: PcDebug.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�ΪPcDebug����ģ��Э��㴦����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-05-11, by  lxf, �������ļ�
 *
 */

//-----ͷ�ļ�����------------------------------------------------------------
#include "config.h"
#include "PcHardware.h"

//-----�ⲿ��������------------------------------------------------------------


#define DEBUG_DMA_UART_Tx_BSIZE        1500
uint8 DEBUG_DMA_UART_Tx_Buff[DEBUG_DMA_UART_Tx_BSIZE];

#define DEBUG_DMA_UART_Rx_BSIZE        1500
uint8 DEBUG_DMA_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];

uint8 DEBUG_UART_Rx_Buff[DEBUG_DMA_UART_Rx_BSIZE];
uint16 DEBUG_UART_Rx_Cnt;


OS_EVENT   *UartDEBUGReadSem;   //gps���ڽ����ź���
OS_EVENT   *UartDEBUGWriteSem;   //gps���ڷ����ź���



int fputc(int ch, FILE *f){

	/* ��Printf���ݷ������� */
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
   	DMA_SetCurrDataCounter(DMA2_Stream7, len); 	// ����Ҫ���͵��ֽ���Ŀ
	DMA_Cmd(DMA2_Stream7, ENABLE);        		//��ʼDMA����
}



void USART1_DMA_Config(void)  
{  
    DMA_InitTypeDef DMA_InitStructure;  
  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);//����DMAʱ�� 

	//����DMA
    DMA_DeInit(DMA2_Stream5);

    DMA_InitStructure.DMA_Channel = DMA_Channel_4;//ͨ��4   
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)DEBUG_DMA_UART_Rx_Buff; //�ڴ��ַ  
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�  
    DMA_InitStructure.DMA_BufferSize = DEBUG_DMA_UART_Rx_BSIZE; //�����С  
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ����  
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ��  
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݴ�С1byte  
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�ڴ����ݴ�С1byte  
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ  
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//���ȼ���  
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;// ����fifo  
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; //  
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//  
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//  
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);//��ʼ��dma2��5  
    DMA_Cmd(DMA2_Stream5, ENABLE);//����dma2��5  

	//����DMA����
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

//����1������ü������ж����ã�
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
//������������ܺ�����
void DEBUG_Uart_Init(void)  
{  
	if(NULL==UartDEBUGReadSem)
		UartDEBUGReadSem = OSSemCreate(0);
	if(NULL==UartDEBUGWriteSem)
		UartDEBUGWriteSem = OSSemCreate(1);
  	UART1_Config();  
  	USART1_DMA_Config();//��Ҫ�����������ַ���ڴ��ַ�Լ���������С�����úú�DMA�ͻ��Զ��İѴ������ݴ浽��Ӧ���ڴ��ַ��  
  	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//���ô���1DMA����  
	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);  //��������DMA����
}  
//����1�����жϷ�������
void USART1_IRQHandler(void)  
{  
    u16 i;  
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//���Ϊ���������ж�  
    {  
        i = USART1->SR;  
        i = USART1->DR;  
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);  
        DMA_Cmd(DMA2_Stream5, DISABLE);//�ر�DMA,��ֹ�������������  
        //USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);	//���ô���1DMA����  
        DEBUG_UART_Rx_Cnt = DEBUG_DMA_UART_Rx_BSIZE - DMA_GetCurrDataCounter(DMA2_Stream5);
		memcpy(DEBUG_UART_Rx_Buff, DEBUG_DMA_UART_Rx_Buff, DEBUG_UART_Rx_Cnt);
		DMA_SetCurrDataCounter(DMA2_Stream5, DEBUG_DMA_UART_Rx_BSIZE);
		DMA_Cmd(DMA2_Stream5, ENABLE);//����DMA
        //USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);	//���ô���1DMA����  
        OSSemPost(UartDEBUGReadSem);
    }  
}  
/*********************DMA�жϷ�����****************************/
void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7))
    {
     	OSSemPost(UartDEBUGWriteSem);
    }
    /* clear DMA flag */
    DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7 | DMA_FLAG_TEIF7);
}

//-----�ļ�PcDebug.c����---------------------------------------------
