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
 * 2019-03-11, by  lxf, 创建本文件
 *
 */

//-----头文件调用------------------------------------------------------------
#include "config.h"
//#define DEV_TTY "/dev/ttyUSB2"
#define DEV_TTY "/dev/smd8"

//-----外部变量定义------------------------------------------------------------

#define GSM_DMA_UART_Tx_BSIZE        1500
uint8 GSM_DMA_UART_Tx_Buff[GSM_DMA_UART_Tx_BSIZE];

#define GSM_DMA_UART_Rx_BSIZE        1500
uint8 GSM_DMA_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];

uint8 GSM_UART_Rx_Buff[GSM_DMA_UART_Rx_BSIZE];
uint16 GSM_UART_Rx_Cnt;
int32 gGsmAT_fd = -1;


int32 OpenGsmAT_DEV_TTY(void)
{

	struct termios options;
	
//打开失败，如何处理?	   
	gGsmAT_fd = open(DEV_TTY, O_RDWR|O_NOCTTY);
	if (gGsmAT_fd < 0) {
		printf("ERROR open %s ret=%d\n\r", DEV_TTY, gGsmAT_fd);
		//return -1;
		return gGsmAT_fd;
	}
    printf("gGsmAT_fd: %d \n", gGsmAT_fd);

	tcgetattr(gGsmAT_fd, &options);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cc[VTIME] = 10; // read timeout 10*100ms
	options.c_cc[VMIN]  = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(ICRNL | IXON);
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	options.c_cflag |= (CLOCAL | CREAD);
	tcflush(gGsmAT_fd, TCIFLUSH);
	tcsetattr(gGsmAT_fd, TCSANOW, &options);
	return gGsmAT_fd;
}



//-----文件GsmHardWareLayer.c结束---------------------------------------------
