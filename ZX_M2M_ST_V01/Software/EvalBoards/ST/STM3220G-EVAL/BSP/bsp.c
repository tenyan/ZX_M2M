/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                             (c) Copyright 2012; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                           STM3220G-EVAL
*                                         Evaluation Board
*
* Filename      : bsp.c
* Version       : V1.00
* Programmer(s) : SL
*                 DC
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define   BSP_MODULE

#include  <bsp.h>
#include "config.h"
#if 0 
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

#define LEDn                        4


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

GPIO_TypeDef*     GPIO_PORT[LEDn] = {GPIOG,
                                     GPIOG,
                                     GPIOI,
                                     GPIOC};

const CPU_INT16U  GPIO_PIN[LEDn]  = {GPIO_Pin_6,
                                     GPIO_Pin_8,
                                     GPIO_Pin_9,
                                     GPIO_Pin_7};

const CPU_INT32U  GPIO_CLK[LEDn]  = {RCC_AHB1Periph_GPIOG,
                                     RCC_AHB1Periph_GPIOG,
                                     RCC_AHB1Periph_GPIOI,
                                     RCC_AHB1Periph_GPIOC};

#endif
/*
*********************************************************************************************************
*                                             REGISTERS
*********************************************************************************************************
*/

#define  DWT_CR      *(CPU_REG32 *)0xE0001000
#define  DWT_CYCCNT  *(CPU_REG32 *)0xE0001004
#define  DEM_CR      *(CPU_REG32 *)0xE000EDFC


/*
*********************************************************************************************************
*                                            REGISTER BITS
*********************************************************************************************************
*/

#define  DEM_CR_TRCENA                   (1 << 24)

#define  DWT_CR_CYCCNTENA                (1 <<  0)


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               BSP_Init()
*
* Description : Initialize the Board Support Package (BSP).
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function SHOULD be called before any other BSP function is called.
*
*               (2) CPU instruction / data tracing requires the use of the following pins :
*                   (a) (1) Aysynchronous     :  PB[3]
*                       (2) Synchronous 1-bit :  PE[3:2]
*                       (3) Synchronous 2-bit :  PE[4:2]
*                       (4) Synchronous 4-bit :  PE[6:2]
*
*                   (c) The application may wish to adjust the trace bus width depending on I/O
*                       requirements.
*********************************************************************************************************
*/

void  BSP_Init (void)
{
    /* At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f2xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f2xx.c file
     */

#if 0
#ifdef VECT_TAB_FLASH
  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#elif VECT_TAB_RAM
  NVIC_SetVectorTable(NVIC_VectTab_RAM,   0x0);
#endif
#endif
}


/*
*********************************************************************************************************
*                                               BSP_RNG_Read()
*
* Description : Read the value of the RNG.
*
* Argument(s) : none.
*
* Return(s)   : ????.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/
#if 0
CPU_INT32U  BSP_RNG_Read (void)
{
    CPU_INT32U  rng_val;


    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, ENABLE);         /* Enable RNG clock.                                    */

    RNG_REG->CR |= RNG_CR_IE;                                   /* Enable RNG ISR.                                      */
    RNG_REG->CR |= RNG_CR_RNGEN;                                /* Enable LFSR & err detector.                          */

    while ((RNG_REG->SR & RNG_SR_DRDY) == 0) {
            ;
    }
    rng_val = (CPU_INT32U)RNG_REG->DR;                          /* Save RNG data reg.                                   */

    RNG_REG->CR &= ~RNG_CR_IE;                                  /* Disable RNG ISR.                                     */
    RNG_REG->CR &= ~RNG_CR_RNGEN;                               /* Disable LFSR & err detector.                         */
    RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_RNG, DISABLE);        /* Disable RNG clock.                                   */

    return (rng_val);                                           /* Return rand value.                                   */
}
#endif

/*
*********************************************************************************************************
*                                            BSP_CPU_ClkFreq()
*
* Description : Read CPU registers to determine the CPU clock frequency of the chip.
*
* Argument(s) : none.
*
* Return(s)   : The CPU clock frequency, in Hz.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

CPU_INT32U  BSP_CPU_ClkFreq (void)
{
    RCC_ClocksTypeDef  rcc_clocks;


    RCC_GetClocksFreq(&rcc_clocks);
    return ((CPU_INT32U)rcc_clocks.HCLK_Frequency);
}


/*
*********************************************************************************************************
*                                       BSP_CPU_TickInit()
*
* Description : This function reads CPU registers to determine the CPU clock frequency of the chip in KHz.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_CPU_TickInit  (void)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;


    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
#if (OS_VERSION >= 30000u)
    cnts         = (cpu_clk_freq / OSCfg_TickRate_Hz);          /* Determine nbr SysTick increments                     */
#else
    cnts         = (cpu_clk_freq / OS_TICKS_PER_SEC);
#endif
    OS_CPU_SysTickInit(cnts);                                   /* Initialize the SysTick.                            */
}
/*
*********************************************************************************************************
*                                          BSP_PSRAM_Init()
*
* Description : Initialize PSRAM.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/
#if 0
void  BSP_PSRAM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE |
                           RCC_AHB1Periph_GPIOF |
                           RCC_AHB1Periph_GPIOG, ENABLE);


    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  |
                                  GPIO_Pin_3  | GPIO_Pin_4  |
                                  GPIO_Pin_5  | GPIO_Pin_6  |
                                  GPIO_Pin_7  | GPIO_Pin_8  |
                                  GPIO_Pin_9  | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 |
                                  GPIO_Pin_13 | GPIO_Pin_14 |
                                  GPIO_Pin_15;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  |
                                  GPIO_Pin_3  | GPIO_Pin_4  |
                                  GPIO_Pin_5  | GPIO_Pin_6  |
                                  GPIO_Pin_7  | GPIO_Pin_8  |
                                  GPIO_Pin_9  | GPIO_Pin_10 |
                                  GPIO_Pin_11 | GPIO_Pin_12 |
                                  GPIO_Pin_13 | GPIO_Pin_14 |
                                  GPIO_Pin_15;

    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  |
                                  GPIO_Pin_2  | GPIO_Pin_3  |
                                  GPIO_Pin_4  | GPIO_Pin_5  |
                                  GPIO_Pin_12 | GPIO_Pin_13 |
                                  GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |
                                  GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5;

    GPIO_Init(GPIOG, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FSMC);


    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOF, GPIO_Pin_6);

    FSMC_Bank1->BTCR[0] = FSMC_DataAddressMux_Disable | FSMC_MemoryType_SRAM       |
                          FSMC_MemoryDataWidth_16b    | FSMC_WriteOperation_Enable |
                          FSMC_ExtendedMode_Disable   | FSMC_AsynchronousWait_Disable;

    FSMC_Bank1->BTCR[0] = FSMC_DataAddressMux_Disable | FSMC_MemoryType_SRAM       |
                          FSMC_MemoryDataWidth_16b    | FSMC_WriteOperation_Enable |
                          FSMC_ExtendedMode_Enable    | FSMC_AsynchronousWait_Disable;

    FSMC_Bank1->BTCR[1]  = 0x00000404;
    FSMC_Bank1E->BWTR[0] = 0x00000404;

    FSMC_Bank1->BTCR[0] |= 0x0001;
}
#endif

/*
*********************************************************************************************************
*                                          BSP_SRAM_Init()
*
* Description : Initialize SRAM.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Init().
*
* Note(s)     : none.
*********************************************************************************************************
*/
#if 0
void  BSP_SRAM_Init(void)
{
    FSMC_NORSRAMInitTypeDef        FSMC_SRAM_Init;
    FSMC_NORSRAMTimingInitTypeDef  FSMC_SRAM_TimingInit;
    GPIO_InitTypeDef               GPIO_InitStructure;


    RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE |
                           RCC_AHB1Periph_GPIOF |
                           RCC_AHB1Periph_GPIOG,
                           ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  |
                                  GPIO_Pin_4  | GPIO_Pin_5  |
                                  GPIO_Pin_8  | GPIO_Pin_9  |
                                  GPIO_Pin_10 | GPIO_Pin_11 |
                                  GPIO_Pin_12 | GPIO_Pin_13 |
                                  GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource4,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1   |
                                  GPIO_Pin_3  | GPIO_Pin_4   |
                                  GPIO_Pin_5  | GPIO_Pin_6   |
                                  GPIO_Pin_7  | GPIO_Pin_8   |
                                  GPIO_Pin_9  | GPIO_Pin_10  |
                                  GPIO_Pin_11 | GPIO_Pin_12  |
                                  GPIO_Pin_13 | GPIO_Pin_14  |
                                  GPIO_Pin_15;

    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource0,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource1,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource3,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource4,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FSMC);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0  | GPIO_Pin_1  |
                                  GPIO_Pin_2  | GPIO_Pin_3  |
                                  GPIO_Pin_4  | GPIO_Pin_5  |
                                  GPIO_Pin_12 | GPIO_Pin_13 |
                                  GPIO_Pin_14 | GPIO_Pin_15;

    GPIO_Init(GPIOF, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource2,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource3,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource4,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource5,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FSMC);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |
                                  GPIO_Pin_2 | GPIO_Pin_3 |
                                  GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_9;

    GPIO_Init(GPIOG, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOG, GPIO_PinSource0,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource1,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource2,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource3,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource4,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource5,  GPIO_AF_FSMC);
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9,  GPIO_AF_FSMC);

                                                                /*------------------- SRAM BANK 2 ----------------------*/
    FSMC_SRAM_TimingInit.FSMC_AddressSetupTime      = 0;
    FSMC_SRAM_TimingInit.FSMC_AddressHoldTime       = 0;
    FSMC_SRAM_TimingInit.FSMC_DataSetupTime         = 4;
    FSMC_SRAM_TimingInit.FSMC_BusTurnAroundDuration = 1;
    FSMC_SRAM_TimingInit.FSMC_CLKDivision           = 0;
    FSMC_SRAM_TimingInit.FSMC_DataLatency           = 0;
    FSMC_SRAM_TimingInit.FSMC_AccessMode            = FSMC_AccessMode_A;

    FSMC_SRAM_Init.FSMC_ReadWriteTimingStruct = &FSMC_SRAM_TimingInit;
    FSMC_SRAM_Init.FSMC_WriteTimingStruct     = &FSMC_SRAM_TimingInit;

    FSMC_SRAM_Init.FSMC_Bank                  = FSMC_Bank1_NORSRAM2;

    FSMC_SRAM_Init.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
    FSMC_SRAM_Init.FSMC_MemoryType            = FSMC_MemoryType_SRAM;
    FSMC_SRAM_Init.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
    FSMC_SRAM_Init.FSMC_AsynchronousWait      = FSMC_AsynchronousWait_Disable;
    FSMC_SRAM_Init.FSMC_ExtendedMode          = FSMC_ExtendedMode_Enable;
    FSMC_SRAM_Init.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;

    FSMC_SRAM_Init.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
    FSMC_SRAM_Init.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
    FSMC_SRAM_Init.FSMC_WrapMode              = FSMC_WrapMode_Disable;
    FSMC_SRAM_Init.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_SRAM_Init.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
    FSMC_SRAM_Init.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;

    FSMC_NORSRAMInit(&FSMC_SRAM_Init);

    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
}
#endif










/*$PAGE*/
/*
*********************************************************************************************************
*                                          CPU_TS_TmrInit()
*
* Description : Initialize & start CPU timestamp timer.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : CPU_TS_Init().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but MUST NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrInit() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                   (c) When applicable, timer period SHOULD be less than the typical measured time
*                       but MUST be less than the maximum measured time; otherwise, timer resolution
*                       inadequate to measure desired times.
*
*                   See also 'CPU_TS_TmrRd()  Note #2'.
*********************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
void  CPU_TS_TmrInit (void)
{
    CPU_INT32U  cpu_clk_freq_hz;


    DEM_CR         |= (CPU_INT32U)DEM_CR_TRCENA;                /* Enable Cortex-M3's DWT CYCCNT reg.                   */
    DWT_CYCCNT      = (CPU_INT32U)0u;
    DWT_CR         |= (CPU_INT32U)DWT_CR_CYCCNTENA;

    cpu_clk_freq_hz = BSP_CPU_ClkFreq();
    CPU_TS_TmrFreqSet(cpu_clk_freq_hz);
}
#endif


/*$PAGE*/
/*
*********************************************************************************************************
*                                           CPU_TS_TmrRd()
*
* Description : Get current CPU timestamp timer count value.
*
* Argument(s) : none.
*
* Return(s)   : Timestamp timer count (see Notes #2a & #2b).
*
* Caller(s)   : CPU_TS_Init(),
*               CPU_TS_Get32(),
*               CPU_TS_Get64(),
*               CPU_IntDisMeasStart(),
*               CPU_IntDisMeasStop().
*
*               This function is an INTERNAL CPU module function & MUST be implemented by application/
*               BSP function(s) [see Note #1] but SHOULD NOT be called by application function(s).
*
* Note(s)     : (1) CPU_TS_TmrRd() is an application/BSP function that MUST be defined by the developer
*                   if either of the following CPU features is enabled :
*
*                   (a) CPU timestamps
*                   (b) CPU interrupts disabled time measurements
*
*                   See 'cpu_cfg.h  CPU TIMESTAMP CONFIGURATION  Note #1'
*                     & 'cpu_cfg.h  CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION  Note #1a'.
*
*               (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR'
*                       data type.
*
*                       (1) If timer has more bits, truncate timer values' higher-order bits greater
*                           than the configured 'CPU_TS_TMR' timestamp timer data type word size.
*
*                       (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR'
*                           timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be
*                           configured so that ALL bits in 'CPU_TS_TMR' data type are significant.
*
*                           In other words, if timer size is not a binary-multiple of 8-bit octets
*                           (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple
*                           octet word size SHOULD be configured (e.g. to 16-bits).  However, the
*                           minimum supported word size for CPU timestamp timers is 8-bits.
*
*                       See also 'cpu_cfg.h   CPU TIMESTAMP CONFIGURATION  Note #2'
*                              & 'cpu_core.h  CPU TIMESTAMP DATA TYPES     Note #1'.
*
*                   (b) Timer SHOULD be an 'up'  counter whose values increase with each time count.
*
*                       (1) If timer is a 'down' counter whose values decrease with each time count,
*                           then the returned timer value MUST be ones-complemented.
*
*                   (c) (1) When applicable, the amount of time measured by CPU timestamps is
*                           calculated by either of the following equations :
*
*                           (A) Time measured  =  Number timer counts  *  Timer period
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer period            Timer's period in some units of
*                                                                   (fractional) seconds
*                                       Time measured           Amount of time measured, in same
*                                                                   units of (fractional) seconds
*                                                                   as the Timer period
*
*                                                  Number timer counts
*                           (B) Time measured  =  ---------------------
*                                                    Timer frequency
*
*                                   where
*
*                                       Number timer counts     Number of timer counts measured
*                                       Timer frequency         Timer's frequency in some units
*                                                                   of counts per second
*                                       Time measured           Amount of time measured, in seconds
*
*                       (2) Timer period SHOULD be less than the typical measured time but MUST be less
*                           than the maximum measured time; otherwise, timer resolution inadequate to
*                           measure desired times.
*********************************************************************************************************
*/

#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED)
CPU_TS_TMR  CPU_TS_TmrRd (void)
{
    return ((CPU_TS_TMR)DWT_CYCCNT);
}
#endif

