/*
 * Copyright(c)2019, Ӳ���з���
 * All right reserved
 *
 * �ļ�����: BaseFun.c
 * �汾��  : V1.0
 * �ļ���ʶ:
 * �ļ�����: ���ļ�Ϊ�����������ʵ���ļ�
 *
 *-------------------------------------------------------
 * �޸ļ�¼
 *-------------------------------------------------------
 *
 * 2011-03-24, by , �������ļ�
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
** ��������: DecToUint8
** ��������: ���ַ���ת��Ϊ�޷����ַ�
** 
** ��    ��: str,��ת���ַ���; len,��ת��������
** ��    ��:
** ��    ��:
**
** ��    ��:
** ��    ��:2011-03-24
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
** ��������: DecToUint16
** ��������: ���ַ���ת��Ϊ������
**
** ��    ��: str,��ת���ַ���; len,��ת��������
** ��    ��: ��
** ��    ��: ת������ֵ
**
** ��    ��: 
** ��    ��: 2011-03-24
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
** ��������: DecToUint32
** ��������: ���ַ���ת��Ϊ������
**
** ��    ��: str,��ת���ַ���; len,��ת��������
** ��    ��: ��
** ��    ��: ת������ֵ
**
** ��    ��: 
** ��    ��: 2011-03-24
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
** ��������: SearchString
** ��������: ��ѯ��ȵ��ַ���,�������ȵ�,
**           ���ص�Ӧ������ͬ�ַ����ĵ�һ���ַ���ָ��
** ��    ��: uint8 *str:���������ַ���,uint16 strlen:�������ַ����ĳ���
**           char *substr:��ҪѰ�ҵ��ַ���,uint8 substrlen:��ҪѰ�ҵ��ַ����ĳ���
** �䡡  ��: ���û������������0,���������������ͬ�ַ��ĵ�һ���ַ�ָ���ַ
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: 
** �ա�  ��: 2011-03-24
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
** ��������: Uint32IPDatatoDecWithDot
** ��������: ��32λ���α����仯ΪIP��ʽ(��:211.139.111.40)
** ��    ��: uint32 IPData:32λ���α�����
** �䡡  ��: uint8* destbuff:ת����IP��ʼ��ַָ��
**           len :ת����IP��ַ����,����"."
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: 
** �ա�  ��: 2011-03-24
**--------------------------------------------------------------------------------
*********************************************************************************/

//��32λ���α����仯ΪIP��ʽ
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
** ��������: HexToASCII
** ��������: ��Hex��ʽת��ΪASCII
** ��    ��: data: ת�����ݵ����ָ�� , len : ��Ҫת���ĳ��� 
** �䡡  ��: buffer: ת�����������ָ�� 
**           pos  ת����ĳ���   
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: 
** �ա�  ��: 2011-03-24
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
** ��������: ASCIITOHex
** ��������: ��ASCII��ʽת��ΪHex
**                            ע�⣺OData[]�����е�������ת�������лᱻ�޸ġ�
** ��    ��: OData: ת�����ݵ����ָ�룬  len : ��Ҫת���ĳ��� 
** �䡡  ��: NData: ת���������ݵ����ָ�� 
**           tmp_len  ת����ĳ���   
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: 
** �ա�  ��: 2011-03-24
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
** ��������: StrToUint32
** ��������: ��С�˻���ģʽ�洢��4�ֽ��ڴ�����ת����32λ�޷��ų�����
**
** ��    ��: p=����ָ��;  mem_mod=�洢ģʽ,  ȡֵΪMEM_MOD_LITLE   MEM_MOD_BIG    
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-12-1
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
** ��������: StrToUint16
** ��������: ��С�˻���ģʽ�洢��2�ֽ��ڴ�����ת����16λ�޷��ų�����
**
** ��    ��: p=����ָ��;  mem_mod=�洢ģʽ,  ȡֵΪMEM_MOD_LITLE   MEM_MOD_BIG    
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: hhm
** ��    ��: 2014-12-1
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


/*����У���*/
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
** ��������: GetStringMiddle
** ��������: ��һ���ַ�����Ѱ������ָ���ַ��м���ַ���
**                            ���ص�Ӧ������ͬ�ַ����ĵ�һ���ַ���ָ��ͳ���
** ��    ��: uint8 *str:���������ַ���,uint16 strlen:�������ַ����ĳ���
**                           char *substr:��ҪѰ�ҵ��ַ���,uint8 substrlen:��ҪѰ�ҵ��ַ����ĳ���
** �䡡  ��: ���û������������0,���������������ͬ�ַ��ĵ�һ���ַ�ָ���ַ
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: zcm
** ��  ����: 2008-09-04
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t *GetStringMiddle(uint8_t *str,uint16_t strlen, char str1,uint16_t Nostr1, char str2,uint16_t Nostr2,uint16_t *substrlen)
{
    uint16_t No1p;		//�ַ�1 λ�ü�����
    uint16_t No2p;		//�ַ�2 λ�ü�����
    uint16_t i;
    uint8_t *Firststr=NULL;	//���ַ�λ��
    uint8_t *Secstr=NULL;
    uint8_t Str1Ok;
    uint8_t Str2Ok;	
	
//�������ַ�����Ϊ0���򷵻�0ֵ
    if(strlen==0)  return 0;
//λ���ۼ�����
    No1p=0;
    No2p=0;
//�Ƿ��ҵ��������	
    Str1Ok=FALSE;
    Str2Ok=FALSE;	
//�����ַ�λ��	
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
			break;  //��β�ַ����ҵ�
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
** ��������: SingleASCIITOHex
** ��������: ��ASCII��ʽת��ΪHex
**           ע�⣺OData[]�����е�������ת�������лᱻ�޸ġ�
** ��    ��: OData: ת�����ݵ����ָ�룬  len : ��Ҫת���ĳ��� 
** �䡡  ��: NData: ת���������ݵ����ָ�� 
**           tmp_len  ת����ĳ���   
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: zcm
** �ա�  ��: 2008-09-04
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
** ��������: GetStringBack
** ��������: ��һ���ַ�����Ѱ������ָ���ַ��м���ַ���
**           ���ص�Ӧ������ͬ�ַ����ĵ�һ���ַ���ָ��ͳ���
** ��    ��: uint8 *str:���������ַ���,uint16 strlen:�������ַ����ĳ���
**           char *substr:��ҪѰ�ҵ��ַ���,uint8 substrlen:��ҪѰ�ҵ��ַ����ĳ���
** �䡡  ��: ���û������������0,���������������ͬ�ַ��ĵ�һ���ַ�ָ���ַ
** ȫ�ֱ���: 
** ����ģ��:
** ����  ��: zcm
** �ա�  ��: 2008-09-04
**--------------------------------------------------------------------------------
*********************************************************************************/
uint8_t * GetStringBack(uint8_t *str,uint16_t strlen,char str1,uint16_t Nostr1)
{

	uint16_t No1p;		//�ַ�1 λ�ü�����
	uint16_t i;		    //�ַ�1 λ�ü�����	
		
	uint8_t *Firststr=NULL;	//���ַ�λ��
	uint8_t Str1Ok=FALSE;

    //�������ַ�����Ϊ0���򷵻�0ֵ
    if(strlen==0)  return 0;
    //λ���ۼ�����
	No1p=0;

    //�����ַ�λ��	
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

//�����������������㷨�����뵥λ�Ƕȣ�������λ��
//lat1Ϊ���γ�ȣ�lo1Ϊ��㾭��
//lat2Ϊ�յ�γ�ȣ�lo2Ϊ�յ㾭��
double distance(double lat1,double lon1,double lat2,double lon2)
{
    double test=sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon1 - lon2));
    return NUMBER*acos(test);
} 

/******************************************************************************
** ��������: Lrc
** ��������: ����У�麯��
**
** ��    ��: p,����ָ��;len,���ݳ���
** ��    ��: ��
** ��    ��: ��
**
** ��    ��: zhu
** ��    ��: 2013-06-28
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
** ��������: DigitFilter1
** ��������: ���洢��p��ָ�Ļ����len���ֽڣ�ȥ������offsize���ֽ�
             ȥ����С��offsize���ֽ�,ʣ�µ�ȡƽ��
** ��    ��: p=����ָ��;  len=���ݳ���, offsize=ȥ�������ݳ���    
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
uint16 DigitFilter1(uint16*p, uint16 len, uint16 offsize)
{
    uint16 i,ii;
    uint16 temp,result;
    uint32 sum=0;

	if(len <= offsize*2)
		return 0;
    for(ii=len-1; ii>0; ii--)//����:��С����
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
** ��������: Delay
** ��������: ��ʱ����ָ������
** ��    ��: uiTime--ָ��������    
** ��    ��: ��
** ��    ��: ��
** ��    ��: hhm
** ��    ��: 2014-12-1
**-----------------------------------------------------------------------------
*******************************************************************************/
void Delay(uint32 uiTime)
{
	uint32 i;
	
	for(i=0; i<uiTime; i++)
		;
}

