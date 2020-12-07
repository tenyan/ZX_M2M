#include "config.h"
#include "GpsHardWareLayer.h"

//#define GPS_UART_Tx_BSIZE        150
//uint8 GPS_UART_Tx_Buff[GPS_UART_Tx_BSIZE];

#define DEV_TTY_GPS "/dev/ttyS1"
#define DEV_TTYUSB1_GPS "/dev/ttyUSB1"

uint8 GPS_Recv_Buff[GPS_RECEIVE_BUFF_LEN]={0};

int fd_gps = -1;

void GPS_Uart_Init(void)  
{ 
    
	struct termios options;

	/* open uart */
	fd_gps = open(DEV_TTYUSB1_GPS, O_RDWR|O_NOCTTY);
	if (fd_gps < 0)
	{
		printf("ERROR open %s ret=%d\n\r", DEV_TTYUSB1_GPS, fd_gps);
        close(fd_gps);
		fd_gps = -1;
		return ;
	}
	else
	{
    	
	}
    printf("fd_gps: %d \n", fd_gps);

	/* configure uart */
	tcgetattr(fd_gps, &options);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_cc[VTIME] = 5; // read timeout 单位*100ms
	options.c_cc[VMIN]  = 0;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST;
	options.c_iflag &= ~(ICRNL | IXON);
	cfsetispeed(&options, B115200);//波特率设置
	cfsetospeed(&options, B115200);//波特率设置
	options.c_cflag |= (CLOCAL | CREAD);
	tcflush(fd_gps, TCIFLUSH);
	tcsetattr(fd_gps, TCSANOW, &options);
}  
BOOL ReadGpsUartData(uint8 **data, uint16* len)
{
	int ret;
    memset(GPS_Recv_Buff,0,GPS_RECEIVE_BUFF_LEN);
	
	if(fd_gps<0)
		return FALSE;
		
	if((ret = read(fd_gps, GPS_Recv_Buff, GPS_RECEIVE_BUFF_LEN-1)) > 0)
	{
		*data = GPS_Recv_Buff;
		*len = (uint16)ret;
		return TRUE;
	}
	else
		return FALSE;
}
uint16 WriteGpsUartData(uint8 *data, uint16 Len)
{
    int ret;

	if((0==Len) || (NULL==data)||(fd_gps<0))
		return 0;
	/* write uart */
	ret = write(fd_gps, data, Len);
	if (ret != Len)
	{
		    printf("ERROR GPS write ret=%d\n", ret);
			return 0;
	}
	else
		return Len;
}


