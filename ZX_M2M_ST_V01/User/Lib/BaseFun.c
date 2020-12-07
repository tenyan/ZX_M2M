/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: BaseFun.c
 * 版本号  : V1.0
 * 文件标识:
 * 文件描述: 此文件为基本函数库的实现文件
 *
 *-------------------------------------------------------
 * 修改记录
 *-------------------------------------------------------
 *
 * 2011-03-24, by , 创建本文件
 *
 */
#include "config.h"


void ClearMem(uint8_t* p,uint16_t len)
{
    while(len--) *p++=0;
}


uint16_t CRC16(uint8_t *message, uint16_t len)
{
    uint16_t i, j;
    uint16_t temp;

    temp = 0xFFFF;
    for (i = 0; i < len; i++) 
    {
        temp = temp ^ message[i];
        for(j = 0; j < 8; j++)
        {
            if(temp & 0x0001)
            {
                temp = temp >> 1;
                temp = temp ^ 0x1021;
            }
            else
                temp = temp >> 1;
        }
    }
    return temp;
}  

/**************************************************************************
** 函数名称: DecToUint8
** 功能描述: 将字符串转化为无符号字符
** 
** 输    入: str,待转化字符串; len,待转化串长度
** 输    出:
** 返    回:
**
** 作    者:
** 日    期:2011-03-24
**------------------------------------------------------------------------
**************************************************************************/
uint8_t DecToUint8(uint8_t * str,uint8_t len)
{
    uint8_t i,sum=0;

    for(i=0;i<len;i++)
    {
        if(!isdigit(str[i]))    
	    return 0;
        sum=sum*10+str[i]-'0';
  }
    return sum;
}
uint16_t  hex2uint(uint8_t * str)
{
    uint16_t RetValue=0;
    while(1)
    {
        if(*str>='0'&&*str<='9')
        {
            RetValue*=16;
            RetValue+=(*str++)-'0';
        }
        else if(*str>='A'&&*str<='F')
        {
            RetValue*=16;
            RetValue+=(*str++)-'A'+10;
        }
        else
        {
            return RetValue;
        }
    }
}
uint32_t  dec2uint(uint8_t * str)
{
    uint32_t RetValue=0;
    while(*str>='0'&&*str<='9')
    {
        RetValue*=10;
        RetValue+=(*str++)-'0';
    }
    return RetValue;
}
/**************************************************************************
** 函数名称: DecToUint16
** 功能描述: 将字符串转化为短整数
**
** 输    入: str,待转化字符串; len,待转化串长度
** 输    出: 无
** 返    回: 转化后数值
**
** 作    者: 
** 日    期: 2011-03-24
**------------------------------------------------------------------------
**************************************************************************/
uint16_t DecToUint16(uint8_t * str,uint8_t len)
{
    uint8_t i;
    uint16_t sum=0;
	
    for(i=0;i<len;i++)
    sum=sum*10+str[i]-'0';
    return sum;
}

/**************************************************************************
** 函数名称: DecToUint32
** 功能描述: 将字符串转化为短整数
**
** 输    入: str,待转化字符串; len,待转化串长度
** 输    出: 无
** 返    回: 转化后数值
**
** 作    者: 
** 日    期: 2011-03-24
**------------------------------------------------------------------------
**************************************************************************/
uint32_t DecToUint32(uint8_t * str,uint8_t len)
{
    uint8_t i;
    uint32_t sum=0;
	
    for(i=0;i<len;i++)
    sum=sum*10+str[i]-'0';
    return sum;
}


void Uint8ToDec2(uint8_t value,uint8_t* destbuff)
{
    destbuff[0]=value/10+'0';
    destbuff[1]=value%10+'0';
}

uint8_t Uint8ToDec(uint8_t value,uint8_t* destbuff)
{
    if(value>99)
    {
        destbuff[2]=value%10+'0';
        value/=10;
        destbuff[1]=value%10+'0';
        value/=10;
        destbuff[0]=value+'0';
        return 3;
    }
     else if(value>9)
    {
        destbuff[1]=value%10+'0';
        value/=10;
        destbuff[0]=value+'0';
        return 2;
    }
     destbuff[0]=value+'0';
     return 1;
}

uint8_t Uint16ToDec(uint16_t value,uint8_t* dest)
{
     uint8_t i=5,len=0;
  
     while(i--)
     {
        dest[i]=value%10+'0';
        len++;
        value/=10;
        if(!value)  break;
     }

     for(i=0;i<len;i++)  dest[i]=dest[i+5-len];
    
     return len;
}


/**********************************************************************************
** 函数名称: SearchString
** 功能描述: 查询相等的字符串,如果有相等的,
**           返回的应该是相同字符串的第一个字符的指针
** 输    入: uint8 *str:被搜索的字符串,uint16 strlen:被搜索字符串的长度
**           char *substr:需要寻找的字符串,uint8 substrlen:需要寻找的字符串的长度
** 输　  出: 如果没有搜索到返回0,如果搜索到返回相同字符的第一个字符指针地址
** 全局变量: 
** 调用模块:
** 作　  者: 
** 日　  期: 2011-03-24
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t *SearchString(uint8_t *str,uint16_t strlen,char *substr,uint8_t substrlen)
{
    if(!strlen||!substrlen)  return 0;
    if(strlen<substrlen)  return 0;
    
    strlen-=substrlen;
    do
    {
        if(strncmp(str,substr, substrlen)==0)  return str;
        str++;
		
    }while(strlen--);
    
    return 0;
}

/**********************************************************************************
** 函数名称: Uint32IPDatatoDecWithDot
** 功能描述: 将32位整形变量变化为IP格式(如:211.139.111.40)
** 输    入: uint32 IPData:32位整形变量，
** 输　  出: uint8* destbuff:转换后IP起始地址指针
**           len :转换后IP地址长度,包含"."
** 全局变量: 
** 调用模块:
** 作　  者: 
** 日　  期: 2011-03-24
**--------------------------------------------------------------------------------
*********************************************************************************/

//将32位整形变量变化为IP格式
uint8_t Uint32IPDatatoDecWithDot(uint32_t IPData,uint8_t* destbuff)
{
    uint8_t* Tmp=(uint8_t*)&IPData;
    uint8_t len,len1;

    len=Uint8ToDec(Tmp[0],&destbuff[0]);
    destbuff[len]='.';
    len1=Uint8ToDec(Tmp[1],&destbuff[len+1]);
    len=len+len1+1;
    destbuff[len]='.';
    len1=Uint8ToDec(Tmp[2],&destbuff[len+1]);
    len=len+len1+1;
    destbuff[len]='.';
    len1=Uint8ToDec(Tmp[3],&destbuff[len+1]);
    len=len+len1+1;

    return len;	
}

/**********************************************************************************
** 函数名称: HexToASCII
** 功能描述: 将Hex格式转换为ASCII
** 输    入: data: 转换数据的入口指针 , len : 需要转换的长度 
** 输　  出: buffer: 转换后数据入口指针 
**           pos  转换后的长度   
** 全局变量: 
** 调用模块:
** 作　  者: 
** 日　  期: 2011-03-24
**--------------------------------------------------------------------------------
*********************************************************************************/
uint16_t HexToASCII(uint8_t* data, uint8_t* buffer, uint16_t len)
{ 
    const static uint8_t ascTable[17] = {"0123456789ABCDEF"}; 
    uint8_t *tmp_p = buffer; 
    uint16_t i, pos; 
	
    pos = 0; 
    for(i = 0; i < len; i++) 
	{ 
		tmp_p[pos++] = ascTable[data[i] >> 4]; 
		tmp_p[pos++] = ascTable[data[i] & 0x0f]; 
	} 
    tmp_p[pos] = '\0'; 
    return pos; 
 } 

/**********************************************************************************
** 函数名称: ASCIITOHex
** 功能描述: 将ASCII格式转换为Hex
**                            注意：OData[]数组中的数据在转换过程中会被修改。
** 输    入: OData: 转换数据的入口指针，  len : 需要转换的长度 
** 输　  出: NData: 转换后新数据的入口指针 
**           tmp_len  转换后的长度   
** 全局变量: 
** 调用模块:
** 作　  者: 
** 日　  期: 2011-03-24
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t ASCIITOHex(uint8_t* OData,uint8_t* NData,uint8_t len)
{ 
	int i,j,tmp_len; 
	uint8_t tmpData; 
	uint8_t *O_buf = OData; 
	uint8_t *N_buf = NData; 
	
	for(i = 0; i < len; i++) 
	{ 
		if ((O_buf[i] >= '0') && (O_buf[i] <= '9')) 
		{ 
			tmpData = O_buf[i] - '0'; 
		} 
		else if ((O_buf[i] >= 'A') && (O_buf[i] <= 'F'))    /*A....F*/ 
		{ 
			tmpData = O_buf[i] - 0x37; 
		} 
		else if((O_buf[i] >= 'a') && (O_buf[i] <= 'f'))      /*a....f */
		{ 
			tmpData = O_buf[i] - 0x57; 
		} 
		else 
		{ 
			return 0; 
		} 
		O_buf[i] = tmpData; 
	} 
	
	for(tmp_len = 0,j = 0; j < i; j+=2) 
	{ 
		N_buf[tmp_len++] = (O_buf[j]<<4) | O_buf[j+1]; 
	} 
	
	return tmp_len; 
	
} 

/******************************************************************************
** 函数名称: StrToUint32
** 功能描述: 将小端或大端模式存储的4字节内存数据转换成32位无符号长整型
**
** 输    入: p=数据指针;  mem_mod=存储模式,  取值为MEM_MOD_LITLE   MEM_MOD_BIG    
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/

uint32 StrToUint32(uint8 *ptr, uint8 mem_mod)
{
	uint32 temp = 0;
	
	if(MEM_MOD_LITLE == mem_mod)
	{
		temp =  *ptr++;
	    temp += (uint32)((*ptr++)<<8);
	    temp += (uint32)((*ptr++)<<16);
	    temp += (uint32)((*ptr++)<<24);
	}
	else if(MEM_MOD_BIG== mem_mod)
	{
		temp += (uint32)((*ptr++)<<24);
	 	temp += (uint32)((*ptr++)<<16);
	 	temp += (uint32)((*ptr++)<<8);
		temp +=  *ptr;
	}
	else
		temp = 0;
	
	return temp;
}


/******************************************************************************
** 函数名称: StrToUint16
** 功能描述: 将小端或大端模式存储的2字节内存数据转换成16位无符号长整型
**
** 输    入: p=数据指针;  mem_mod=存储模式,  取值为MEM_MOD_LITLE   MEM_MOD_BIG    
** 输    出: 无
** 返    回: 无
**
** 作    者: hhm
** 日    期: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/

uint16 StrToUint16(uint8 *ptr, uint8 mem_mod)
{
	uint16 temp = 0;
	
	if(MEM_MOD_LITLE == mem_mod)
	{
		temp =  *ptr++;
	    temp += (uint16)(*ptr<<8);
	}
	else if(MEM_MOD_BIG== mem_mod)
	{
		temp = (uint16)((*ptr++)<<8);
	 	temp += *ptr;
	}
	else
		temp = 0;
	
	return temp;
}


/*计算校验和*/
uint8_t SumCalc(uint8* p,uint16 len)
{
	uint16 i;
	uint8 sum = 0;
	for(i = 0; i < len; i++)
	{
		sum += p[i];
	}
	return sum;
}

/**********************************************************************************
** 函数名称: GetStringMiddle
** 功能描述: 在一个字符串中寻找两个指定字符中间的字符串
**                            返回的应该是相同字符串的第一个字符的指针和长度
** 输    入: uint8 *str:被搜索的字符串,uint16 strlen:被搜索字符串的长度
**                           char *substr:需要寻找的字符串,uint8 substrlen:需要寻找的字符串的长度
** 输　  出: 如果没有搜索到返回0,如果搜索到返回相同字符的第一个字符指针地址
** 全局变量: 
** 调用模块:
** 作　  者: zcm
** 日  　期: 2008-09-04
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t *GetStringMiddle(uint8_t *str,uint16_t strlen, char str1,uint16_t Nostr1, char str2,uint16_t Nostr2,uint16_t *substrlen)
{
    uint16_t No1p;		//字符1 位置计数器
    uint16_t No2p;		//字符2 位置计数器
    uint16_t i;
    uint8_t *Firststr=NULL;	//首字符位置
    uint8_t *Secstr=NULL;
    uint8_t Str1Ok;
    uint8_t Str2Ok;	
	
//被搜索字符长度为0，则返回0值
    if(strlen==0)  return 0;
//位置累计清零
    No1p=0;
    No2p=0;
//是否找到标记置零	
    Str1Ok=FALSE;
    Str2Ok=FALSE;	
//查找字符位置	
    for (i=0;i<strlen;i++)
    {
        if (*(str+i)==str1)
        {
            No1p++;
            if (No1p==Nostr1) 
            {
                Firststr=str+i;
                Str1Ok=TRUE;
            }
        }

        if (*(str+i)==str2) 
        {
            No2p++;
            if (No2p==Nostr2) 
            {
                Secstr=str+i;	
                Str2Ok=TRUE;
            }
        }
        if (Str1Ok && Str2Ok) 
		{
			break;  //首尾字符都找到
		}
    }
	
    if (Str1Ok&&Str2Ok&&Secstr!=Firststr)
    {
        if (Secstr>Firststr)
        {
            *substrlen=Secstr-Firststr-1;				
            return Firststr+1;
        }
        *substrlen=Firststr-Secstr-1;				
        return Secstr+1;			
    }
	return 0;
}

/**********************************************************************************
** 函数名称: SingleASCIITOHex
** 功能描述: 将ASCII格式转换为Hex
**           注意：OData[]数组中的数据在转换过程中会被修改。
** 输    入: OData: 转换数据的入口指针，  len : 需要转换的长度 
** 输　  出: NData: 转换后新数据的入口指针 
**           tmp_len  转换后的长度   
** 全局变量: 
** 调用模块:
** 作　  者: zcm
** 日　  期: 2008-09-04
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t SingleASCIITOHex(uint8_t NData)
{ 
    if ((NData >= '0') && (NData <= '9'))   
		return  (NData - '0');
    else if ((NData >= 'A') && (NData <= 'F'))   
		return  (NData - 0x37);
    else if ((NData >= 'a') && (NData <= 'f'))   
		return  (NData - 0x57);	
    else 
		return 0xff;

} 

/**********************************************************************************
** 函数名称: GetStringBack
** 功能描述: 在一个字符串中寻找两个指定字符中间的字符串
**           返回的应该是相同字符串的第一个字符的指针和长度
** 输    入: uint8 *str:被搜索的字符串,uint16 strlen:被搜索字符串的长度
**           char *substr:需要寻找的字符串,uint8 substrlen:需要寻找的字符串的长度
** 输　  出: 如果没有搜索到返回0,如果搜索到返回相同字符的第一个字符指针地址
** 全局变量: 
** 调用模块:
** 作　  者: zcm
** 日　  期: 2008-09-04
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t * GetStringBack(uint8_t *str,uint16_t strlen,char str1,uint16_t Nostr1)
{

	uint16_t No1p;		//字符1 位置计数器
	uint16_t i;		    //字符1 位置计数器	
		
	uint8_t *Firststr=NULL;	//首字符位置
	uint8_t Str1Ok=FALSE;

    //被搜索字符长度为0，则返回0值
    if(strlen==0)  return 0;
    //位置累计清零
	No1p=0;

    //查找字符位置	
    for (i=0;i< strlen; i++)
	{
		if (*(str+i)==str1) 
		{
			No1p++;
			if (No1p==Nostr1) 
			{
				Firststr=str+i;
				Str1Ok=TRUE;
				break;
			}
		}
	}
	
	if (Str1Ok)	
		return Firststr+1;
			
	else 	
		return 0;

}

uint8_t Hex2ToUint8(uint8_t *ahex)
{
    uint8_t sum,Ch;
   
    if ((Ch=*ahex++)<='9')
        sum=Ch-'0';
    else
        sum=Ch-'A'+10;
    if ((Ch=*ahex)<='9')
        sum=sum*16+Ch-'0';
    else
        sum=sum*16+Ch-'A'+10;
   
    return sum;
}

uint8_t Dec3ToUint8(uint8_t * ahex,uint8_t len)
{
    uint8_t i,sum=0;
   
    for(i=0;i<len;i++)
    {
        sum=sum*10+*ahex-'0';
        ahex++;
    }
    return sum;
}

uint8_t Dec2ToUint8(uint8_t *ahex)
{
    uint8_t i,sum=0;
   
    for(i=0;i<2;i++)
    {
        sum=sum*10+*ahex-'0';
        ahex++;
    }
    return sum;
}

void HexToDatabuff(uint8_t *buff,uint8_t *hex,uint16_t hexlen)
{
    hexlen>>=1;
    while(hexlen--)
    {
       *buff++=Hex2ToUint8(hex);
       hex+=2;
    }
}

void ConvertBCDB(uint8_t *Abuff,uint8_t len)
{
    uint8_t *p,temp;
     
    p=Abuff++;
    len>>=1;
   
    while (len--)
    {
        temp=*p;
        *p=*Abuff;
        *Abuff=temp;
        p+=2;
        Abuff+=2;
    }
}

void Uint8ToHex2(uint8_t value,uint8_t* destbuff)
{
    uint8_t lowByte;
   
    lowByte=value&0x0F;
    value/=16;
    if (value>9)
        destbuff[0]=value+'A'-10; 
    else
        destbuff[0]=value+'0';
   
    if (lowByte>9)
        destbuff[1]=lowByte+'A'-10; 
    else
        destbuff[1]=lowByte+'0';
}

void DataToHexbuff(uint8_t* dest,uint8_t* src,uint8_t srclen)
{
    while(srclen--)
    {
        Uint8ToHex2(*src,dest);
        src++;
        dest+=2;
    }
}

double deg2rad(double deg)
{
    return (deg*M_PI)/180.0;
}

//快速求地球两点距离算法，传入单位角度，传出单位米
//lat1为起点纬度，lo1为起点经度
//lat2为终点纬度，lo2为终点经度
double distance(double lat1,double lon1,double lat2,double lon2)
{
    double test=sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon1 - lon2));
    return NUMBER*acos(test);
} 

/******************************************************************************
** 函数名称: Lrc
** 功能描述: 异或和校验函数
**
** 输    入: p,数据指针;len,数据长度
** 输    出: 无
** 返    回: 无
**
** 作    者: zhu
** 日    期: 2013-06-28
**-----------------------------------------------------------------------------
*******************************************************************************/
uint8_t Lrc(uint8_t *p, uint16_t len)
{
	uint8_t sum = 0;
	uint16_t i;

	for(i = 0; i < len; i++)
	{
		sum ^= p[i];
	}
	return sum;
}

/******************************************************************************
** 函数名称: DigitFilter1
** 功能描述: 将存储在p所指的缓存的len个字节，去掉最大的offsize个字节
             去掉最小的offsize个字节,剩下的取平均
** 输    入: p=数据指针;  len=数据长度, offsize=去掉的数据长度    
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 DigitFilter1(uint16*p, uint16 len, uint16 offsize)
{
    uint16 i,ii;
    uint16 temp,result;
    uint32 sum=0;

	if(len <= offsize*2)
		return 0;
    for(ii=len-1; ii>0; ii--)//排序:从小到大
	{
		for(i=0; i<ii; i++)
		{
			if(p[i] > p[i+1] )
			{
				temp = p[i];
				p[i] = p[i+1];
				p[i + 1] = temp;
			}
		}	
	}
    for(i=offsize; i<len-offsize; i++)
        sum += p[i];
    result = (sum/(len-offsize*2)) & 0xffff;
    return result;
}

/******************************************************************************
** 函数名称: Delay
** 功能描述: 延时若干指令周期
** 输    入: uiTime--指令周期数    
** 输    出: 无
** 返    回: 无
** 作    者: hhm
** 日    期: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
void Delay(uint32 uiTime)
{
	uint32 i;
	
	for(i=0; i<uiTime; i++)
		;
}

