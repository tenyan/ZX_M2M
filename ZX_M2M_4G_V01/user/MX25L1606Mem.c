/*
 * Copyright(c)2020, �����칤��Ϣ�����ɷ����޹�˾-����Ӳ����ҵ��
 * All right reserved
 *
 * �ļ�����: MX25L1606Mem.c
 * �汾��  : V1.0
 * �ļ���ʶ: 
 * �ļ�����: �洢��MX25L1606Mem�����ļ�
 *
 *-----------------------------------------------------------------------------
 * �޸ļ�¼
 *-----------------------------------------------------------------------------
 *
 * 2019-03-10, by  �������ļ�
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
//char save[6]={"save1"};//��ǰ�洢Ŀ¼
//uint8 flagsave=0;//��ǰ�洢Ŀ¼��־ 1 :Ŀ¼1    2:Ŀ¼2
//char Emmcfilename[1000][256];
//char Deletfilename[50];
//char Emmcfilename_pro[50];
//int filelen = 0;

/******************************************************************************
** ��������: DateCompare
** ��������: �Ƚ��������ڵĴ�С
**
** ��    ��: 
** �䡡  ��: 
** ��    ��: ����ֵ��0(���) 1(����) -1(С��)
**
** ����  ��: 
** �ա�  ��: 2019��07��03��
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
** ��������: trave_dir
** ��������: ɾ�����������ļ�
**
** ��    ��: 
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��07��03��
**-----------------------------------------------------------------------------
*******************************************************************************/
int trave_dir(char* path, int depth)
{
    DIR *d; //����һ�����
    struct dirent *file; //readdir�����ķ���ֵ�ʹ��������ṹ����
    struct stat sb; 
	char Emmcfilename[50]= {"\0"};
    char Emmcfilename_pro[50]= {"\0"};
	char strTmpPath[50] = {"\0"};//ɾ����·��
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
        //�ѵ�ǰĿ¼.����һ��Ŀ¼..�������ļ���ȥ����������ѭ������Ŀ¼
        if(strncmp(file->d_name, ".", 1) == 0)
            continue;
        strcpy(&Emmcfilename[0], file->d_name); //������������ļ���
       // printf("seach file= %s ok\n",Emmcfilename);		
       //19_06_27_14_13_51
        Date.ucYear=  DecToUint16(&Emmcfilename[0],2);//********������ ��ʽ�汾��FileNum ȥ��*********
		Date.ucMon =  DecToUint16(&Emmcfilename[3],2);
		Date.ucDay =  DecToUint16(&Emmcfilename[6],2);
		Date.ucHour=  DecToUint16(&Emmcfilename[9],2);
		Date.ucMin =  DecToUint16(&Emmcfilename[12],2);
		Date.ucSec =  DecToUint16(&Emmcfilename[15],2);
        //Date.ucYear=  DecToUint16(&Emmcfilename[0+4],2);//********������ ��ʽ�汾��FileNum ȥ��*********
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
		 
        //�жϸ��ļ��Ƿ���Ŀ¼�����Ƿ������������㣬�����Ҷ���ֻ����������Ŀ¼��̫��Ͳ����ˣ�ʡ���ѳ�̫���ļ�
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
** ��������: FullProcess
** ��������: �洢����������
**
** ��    ��: 
** �䡡  ��: 
** ��    ��: 
**
** ����  ��: 
** �ա�  ��: 2019��07��03��
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
** ��������: WriteToFlash
** ��������: �������ݴ洢
**
** ��    ��: Pathname,�洢�ļ�·��             
             Length�����ݳ���
             *p ������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,
**
** ����  ��: yzj
** �ա�  ��: 2019��02��17��
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
** ��������: WriteCanToFlash
** ��������: �洢can����  ׷�����ݴ洢
**
** ��    ��: Pathname,�洢�ļ�·��             
             Length�����ݳ���
             *p ������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,
**
** ����  ��: yzj
** �ա�  ��: 2019��02��17��
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
** ��������: WriteCanToFlash
** ��������: �洢can����
**
** ��    ��: Pathname,�洢�ļ�·��
             
             Length�����ݳ���
             *p ������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: zwp  yzjlg
** �ա�  ��: 2011��02��17��
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8 SaveCanToFlash(uint32 Length, uint8 *p)
{ 
    static uint32 SaveSize=0;//��ǰ�ļ��洢��С
	//static uint32 FileNum=0;
	static char FilePath[50]={"\0"};//�洢·��
	uint8 RTime[6];//��ǰʱ�� ������ʱ����0-5
	char i=0,j=0;

	//��Чʱ�䷵�� ���洢
	memcpy(&RTime[0],(uint8*)&stu_STM32_MCU.Date,6);
	if(RTime[0]<19||RTime[0]>39||RTime[1]>12||RTime[1]==0||RTime[2]>31||RTime[2]==0||RTime[3]>59||RTime[4]>59||RTime[5]>59)
	{
	    printf("error time %d_%d_%d_%d_%d_%d\n",RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
	    return 0;
	}

	FullProcess();//�洢�ռ�������
		
    if(SaveSize==0||SaveSize>FILE_SIZE)//�״��ϵ���ļ����󳬹�1M �½��ļ����д洢
    {  
	    //sprintf(FilePath, "/media/card/%03d_%02d_%02d_%02d_%02d_%02d_%02d.txt", FileNum,RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
	    sprintf(FilePath, "/media/card/%02d_%02d_%02d_%02d_%02d_%02d.txt",RTime[0],RTime[1],RTime[2],RTime[3],RTime[4],RTime[5]);
		//FileNum++;
		SaveSize=0;//��ǰ�ļ��洢��С����
		printf("open new file\n");
    } 
    WriteCanToFlash(FilePath, Length, p);

	SaveSize+=Length;

    return 0;
}

/******************************************************************************
** ��������: ReadFromFlash
** ��������: ��������ҳ����
**
** ��    ��: Pathname,�洢�ļ�·��  PageAddr��ʼҳ��ַBufAddr��ʼҳ��ƫ�Ƶ�ַ
                 
             Length�����ݳ���
             *buf ��������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ź�������,2�洢������ʧ��
**
** ����  ��: 
** �ա�  ��: 2011��02��17��
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
** ��������: ReadFromFlash
** ��������: ����������
**
** ��    ��: Pathname,�洢�ļ�·��                   
             Length����Ҫ��ȡ�����ݳ���
             *buf ��������ָ��
             ucflag, 0-������ȡ,1-��ȡ���һ�����ݣ���ȡ��ر��ļ�
             
** �䡡  ��: 
** ��    ��: 0�ɹ�����,1����ʧ��
**
** ����  ��: 
** �ա�  ��: 2019��07��17��
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
** ��������: ReadFromlseekFlash
** ��������: �����ַ��ȡ�ļ�����
**
** ��    ��: Pathname,�洢�ļ�·��  BufAddr�ļ���ƫ�Ƶ�ַ                 
             Length����Ҫ��ȡ�����ݳ���
             *buf ��������ָ��
** �䡡  ��: N/A
** ��    ��: 0�ɹ�����,1�ļ���ʧ��,
**
** ����  ��: lxf
** �ա�  ��: 2019��09��12��
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

//-----�ļ�TS25L16Mem.c����----------------------------------------------------


