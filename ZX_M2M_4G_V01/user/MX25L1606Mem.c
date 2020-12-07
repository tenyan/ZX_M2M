/*
 * Copyright(c)2020, 江苏徐工信息技术股份有限公司-智能硬件事业部
 * All right reserved
 *
 * 文件名称: MX25L1606Mem.c
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 存储器MX25L1606Mem驱动文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2019-03-10, by  创建本文件
 *
 */

#include "config.h"
#include "MX25L1606Mem.h"
#include <sys/statfs.h>
#include <sys/vfs.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>


extern STU_STM32_MCU stu_STM32_MCU;
//char save[6]={"save1"};//当前存储目录
//uint8 flagsave=0;//当前存储目录标志 1 :目录1    2:目录2
//char Emmcfilename[1000][256];
//char Deletfilename[50];
//char Emmcfilename_pro[50];
//int filelen = 0;

/******************************************************************************
** 函数名称: DateCompare
** 功能描述: 比较两个日期的大小
**
** 输    入: 
** 输　  出: 
** 返    回: 返回值：0(相等) 1(大于) -1(小于)
**
** 作　  者: 
** 日　  期: 2019年07月03日
**-----------------------------------------------------------------------------
*******************************************************************************/
int DateCompare(STU_Date date1, STU_Date date2)
{
    int result = 0;
 
    if (date1.ucYear < date2.ucYear)
        result = -1;
    else if (date1.ucYear > date2.ucYear)
        result = 1;
    else
    {
        if (date1.ucMon < date2.ucMon)
            result = -1;
        else if (date1.ucMon > date2.ucMon)
            result = 1;
        else
        {
            if (date1.ucDay < date2.ucDay)
                result = -1;
            else if (date1.ucDay > date2.ucDay)
                result = 1;
            else
            {
                if (date1.ucHour < date2.ucHour)
                    result = -1;
                else if (date1.ucHour > date2.ucHour)
                    result = 1;
                else
                {
                    if (date1.ucMin < date2.ucMin)
                        result = -1;
                    else if (date1.ucMin > date2.ucMin)
                        result = 1;
                    else
                    {
                        if (date1.ucSec < date2.ucSec)
                            result = -1;
                        else if (date1.ucSec > date2.ucSec)
                            result = 1;
                        else
                        {
                            result = 0;
                        }
                    }
                }
            }
        }
    }
 
    return result;
}
/******************************************************************************
** 函数名称: trave_dir
** 功能描述: 删除最早数据文件
**
** 输    入: 
** 输　  出: 
** 返    回: 
**
** 作　  者: 
** 日　  期: 2019年07月03日
**-----------------------------------------------------------------------------
*******************************************************************************/
int trave_dir(char* path, int depth)
{
    DIR *d; //声明一个句柄
    struct dirent *file; //readdir函数的返回值就存放在这个结构体中
    struct stat sb; 
	char Emmcfilename[50]= {"\0"};
    char Emmcfilename_pro[50]= {"\0"};
	char strTmpPath[50] = {"\0"};//删除的路径
    int n;
	STU_Date Date,Date_Pro;

	Date_Pro.ucYear=0xFF;
    
    if(!(d = opendir(path)))
    {
        printf("error opendir %s!!!/n",path);
        return -1;
    }
    while((file = readdir(d)) != NULL)
    {
        //把当前目录.，上一级目录..及隐藏文件都去掉，避免死循环遍历目录
        if(strncmp(file->d_name, ".", 1) == 0)
            continue;
        strcpy(&Emmcfilename[0], file->d_name); //保存遍历到的文件名
       // printf("seach file= %s ok\n",Emmcfilename);		
       //19_06_27_14_13_51
        Date.ucYear=  DecToUint16(&Emmcfilename[0],2);//********测试用 正式版本把FileNum 去掉*********
		Date.ucMon =  DecToUint16(&Emmcfilename[3],2);
		Date.ucDay =  DecToUint16(&Emmcfilename[6],2);
		Date.ucHour=  DecToUint16(&Emmcfilename[9],2);
		Date.ucMin =  DecToUint16(&Emmcfilename[12],2);
		Date.ucSec =  DecToUint16(&Emmcfilename[15],2);
        //Date.ucYear=  DecToUint16(&Emmcfilename[0+4],2);//********测试用 正式版本把FileNum 去掉*********
		//Date.ucMon =  DecToUint16(&Emmcfilename[3+4],2);//001_19_06_27_14_13_51
		//Date.ucDay =  DecToUint16(&Emmcfilename[6+4],2);
		//Date.ucHour=  DecToUint16(&Emmcfilename[9+4],2);
		//Date.ucMin =  DecToUint16(&Emmcfilename[12+4],2);
		//Date.ucSec =  DecToUint16(&Emmcfilename[15+4],2);
        if(1==DateCompare(Date_Pro,Date))
		{
		    Date_Pro=Date;
			strcpy(Emmcfilename_pro,Emmcfilename);
        }
		 
        //判断该文件是否是目录，及是否已搜索了三层，这里我定义只搜索了三层目录，太深就不搜了，省得搜出太多文件
       //if(stat(file->d_name, &sb) >= 0 && S_ISDIR(sb.st_mode) && depth <= 3)
       // {
        //    trave_dir(file->d_name, depth + 1);
        //}
    }

	   //sprintf(strTmpPath, "%s/%s", dir_name, dp->d_name);
	   if(Emmcfilename_pro[0]!='\0')
	   {
		    sprintf(strTmpPath, "/media/card/%s",Emmcfilename_pro);
		    n = remove(strTmpPath);

           if(n==0)
	   	        printf("Full to Delete %s ok\n",strTmpPath);
	       else
	        	printf("Error Full to Delete %s fail\n",strTmpPath);
	   	}
	
    closedir(d);
    return 0;
}
/******************************************************************************
** 函数名称: FullProcess
** 功能描述: 存储卡存满处理
**
** 输    入: 
** 输　  出: 
** 返    回: 
**
** 作　  者: 
** 日　  期: 2019年07月03日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 FullProcess(void)
{ 
    struct statfs sfs;
	
    int i = statfs(EMMC_FILE_PATH, &sfs);
    int percent = (sfs.f_blocks - sfs.f_bfree ) * 100 / (sfs.f_blocks -sfs.f_bfree + sfs.f_bavail) + 1;
 /*   printf("/media/card/            %ld    %ld  %ld   %d%% \n",
           4*sfs.f_blocks, 4*(sfs.f_blocks - sfs.f_bfree), 4*sfs.f_bavail, percent);
*/	
	if(percent>=95)
	{
		trave_dir(EMMC_FILE_PATH, 1);
		//printf("save full delete \n");
	
	}

	    //printf(" sfs.f_type=%d sfs.f_bsize=%d  sfs.f_blocks=%d sfs.f_bfree=%d sfs.f_bavail=%d sfs.f_files=%d sfs.f_ffree=%d  sfs.f_fsid=%d sfs.f_namelen=%d  \n",
                // sfs.f_type ,  sfs.f_bsize ,   sfs.f_blocks   ,  sfs.f_bfree ,sfs.f_bavail, sfs.f_files, sfs.f_ffree,     sfs.f_fsid ,sfs.f_namelen);
	
    
}

/******************************************************************************
** 函数名称: WriteToFlash
** 功能描述: 连续数据存储
**
** 输    入: Pathname,存储文件路径             
             Length，数据长度
             *p ，数据指针
** 输　  出: N/A
** 返    回: 0成功返回,
**
** 作　  者: yzj
** 日　  期: 2019年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 WriteToFlash(char *Pathname ,uint32 Length, uint8 *p)
{ 
    int fd_write;

	fd_write = open(Pathname, O_RDWR|O_CREAT);
	if (fd_write < 0)
	{
		printf("ERROR open %s fd=%d\n\r", Pathname, fd_write);
        close(fd_write);
		return 1;
	}
	
	 write(fd_write,p, Length);
	 close(fd_write);
     return 0;
}


/******************************************************************************
** 函数名称: WriteCanToFlash
** 功能描述: 存储can数据  追加数据存储
**
** 输    入: Pathname,存储文件路径             
             Length，数据长度
             *p ，数据指针
** 输　  出: N/A
** 返    回: 0成功返回,
**
** 作　  者: yzj
** 日　  期: 2019年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
//uint8 WriteToFlash(uint16 PageAddr,uint8 PageOffsetAddr,uint32 Length, uint8 *p)
uint8 WriteCanToFlash(char *Pathname ,uint32 Length, uint8 *p)
{ 
    int fd_write;

	fd_write = open(Pathname, O_RDWR|O_CREAT|O_APPEND);
	if (fd_write < 0)
	{
		printf("ERROR open %s fd=%d\n\r", Pathname, fd_write);
        close(fd_write);
		return 1;
	}
	
	 write(fd_write,p, Length);
	 close(fd_write);
     return 0;
}


/******************************************************************************
** 函数名称: WriteCanToFlash
** 功能描述: 存储can数据
**
** 输    入: Pathname,存储文件路径
             
             Length，数据长度
             *p ，数据指针
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: zwp  yzjlg
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 SaveCanToFlash(uint32 Length, uint8 *p)
{ 
    static uint32 SaveSize=0;//当前文件存储大小
	//static uint32 FileNum=0;
	static char FilePath[50]={"\0"};//存储路径
	uint8 RTime[6];//当前时间 年月日时分秒0-5
	char i=0,j=0;

	//无效时间返回 不存储
	memcpy(&RTime[0],(uint8*)&stu_STM32_MCU.Date,6);
	if(RTime[0]<19||RTime[0]>39||RTime[1]>12||RTime[1]==0||RTime[2]>31||RTime[2]==0||RTime[3]>59||RTime[4]>59||RTime[5]>59)
	{
	    printf("error time %d_%d_%d_%d_%d_%d\n",RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
	    return 0;
	}

	FullProcess();//存储空间满处理
		
    if(SaveSize==0||SaveSize>FILE_SIZE)//首次上电或文件过大超过1M 新建文件进行存储
    {  
	    //sprintf(FilePath, "/media/card/%03d_%02d_%02d_%02d_%02d_%02d_%02d.txt", FileNum,RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
	    sprintf(FilePath, "/media/card/%02d_%02d_%02d_%02d_%02d_%02d.txt",RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
		//FileNum++;
		SaveSize=0;//当前文件存储大小清零
		printf("open new file\n");
    } 
    WriteCanToFlash(FilePath, Length, p);

	SaveSize+=Length;

    return 0;
}

/******************************************************************************
** 函数名称: ReadFromFlash
** 功能描述: 连续读多页数据
**
** 输    入: Pathname,存储文件路径  PageAddr起始页地址BufAddr起始页内偏移地址
                 
             Length，数据长度
             *buf ，缓冲区指针
** 输　  出: N/A
** 返    回: 0成功返回,1信号量问题,2存储器操作失败
**
** 作　  者: 
** 日　  期: 2011年02月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFromFlash(char *Pathname ,uint32 Length, uint8 *Buf)
{
	int fd_read;

	fd_read = open(Pathname, O_RDWR);//|O_NOCTTY|O_CREAT);
	if (fd_read < 0)
	{
		printf("ERROR open %s fd=%d\n\r", Pathname, fd_read);
        close(fd_read);
		return 1;
	}

	 read(fd_read,Buf,Length);
	 close(fd_read);
	
	return 0;	
}


/******************************************************************************
** 函数名称: ReadFromFlash
** 功能描述: 连续读数据
**
** 输    入: Pathname,存储文件路径                   
             Length，需要读取的数据长度
             *buf ，缓冲区指针
             ucflag, 0-正常读取,1-读取最后一包数据，读取后关闭文件
             
** 输　  出: 
** 返    回: 0成功返回,1操作失败
**
** 作　  者: 
** 日　  期: 2019年07月17日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFromWholFileFlash(char *Pathname ,uint32 Length, uint8 *Buf)
{
	static int fd_read = -1;
    int usLen = 0;
	
    if(fd_read<0)
        fd_read = open(Pathname, O_RDWR);
        
	if (fd_read < 0)
	{
		printf("ERROR open %s fd=%d\n\r", Pathname, fd_read);
        close(fd_read);
		return 1;
	}
 
	usLen = read(fd_read,Buf,Length);
	if(usLen < Length)
	    close(fd_read);
	
	return 0;	
}

/******************************************************************************
** 函数名称: ReadFromlseekFlash
** 功能描述: 任意地址读取文件数据
**
** 输    入: Pathname,存储文件路径  BufAddr文件内偏移地址                 
             Length，需要读取的数据长度
             *buf ，缓冲区指针
** 输　  出: N/A
** 返    回: 0成功返回,1文件打开失败,
**
** 作　  者: lxf
** 日　  期: 2019年09月12日
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 ReadFromlseekFlash(char *Pathname ,uint32 BufAddr, uint32 Length, uint8 *Buf)
{
	int fd_read;
    int off_set;
	fd_read = open(Pathname, O_RDWR);//|O_NOCTTY|O_CREAT);
	if (fd_read < 0)
	{
		printf("ERROR open %s fd=%d\n\r", Pathname, fd_read);
        close(fd_read);
		return 1;
	}
	
  //  off_set = lseek(Pathname,BufAddr,SEEK_SET); 
    off_set = lseek(fd_read,BufAddr,SEEK_SET); 
    if(off_set==-1)
    {
		printf("ERROR lseek %s fd=%d\n\r", Pathname, off_set);
        close(fd_read);
		return 1;
	}
	read(fd_read,Buf,Length);
	close(fd_read);
	
	return 0;	
}

//-----文件TS25L16Mem.c结束----------------------------------------------------


