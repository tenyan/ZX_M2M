//文件名：Canconfig.h
//功能： 
#ifndef _CANCONFIG_H_
#define _CANCONFIG_H_

#define CAN_FRAME_MAX_MUN       100
#define CAN_DATA_MAX_LEN        1024     //
#define CAN_PARAMGROUP_MAX_NUM   5      //参数组最大个数
#define CAN_CFG_FILE_MAX_LEN    2048
typedef struct _STU_CanParamFrame
{
	uint32 uiID;            //帧ID    
	uint8 ucOptionByte;     //选项字节
	//uint32 uiTime;          //上次CAN数据保存时间 单位ms  由分-秒-毫秒换算得到
}STU_CanParamFrame;

typedef struct _STU_CanParam_Group{
    uint16 usTlvName;
	uint16 usTlvLen;
	uint16 usSamplingFreq;  //采集频率  当该值<=200ms 以200ms为准
	uint16 usUpFreq;        //参数组上传频率
	uint8  ucCanNum;        //该组参数CAN个数
	STU_CanParamFrame stuCanFrame[CAN_FRAME_MAX_MUN];
	
}STU_CanParam_Group;

typedef struct _STU_CanProjects
{
    uint8 ucParamGroupNum;           //配置的所有的参数组总个数
    STU_CanParam_Group stuParamGroup[CAN_PARAMGROUP_MAX_NUM];
	uint8 agroupSendFlag[CAN_PARAMGROUP_MAX_NUM];   //参数组发送标识  1-采集完成触发发送 ,2-需要再采集,0-发送完毕\不发送
	uint8 agrouptype[CAN_PARAMGROUP_MAX_NUM];       //判断 1-采集够5次上报,2-按照采集频率上报
    uint8 aucCanRcvFlag[CAN_PARAMGROUP_MAX_NUM];    //计时发送时间到了需要判断该参数组是否收到了CAN帧 1-收到,0-未收到
	//
}STU_CanProjects;

typedef struct _STU_CanData
{
    uint16 usDataLen;
	uint8 aData[CAN_DATA_MAX_LEN];
	//uint16 usDataptr;
}STU_CanData;

typedef struct _STU_CanDataUp{
    uint16 usTlvName;        //CAN数据组初始化的时候需要赋初值
	uint16 usTlvLen;         //整个参数组的数据(可能有多个)
	uint16 usSamplingFreq;   //采集频率  对应设置的采集TLV中采集频率 by lxf 20200316
	STU_Date Date;
	uint8 ucTlvNum;         //参数组个数
	uint8 aCount[CAN_PARAMGROUP_MAX_NUM];  //记录次数  一次上报最多包含5组
	uint16 usDataLen;        // 一组数据的长度  不包含自身2byte
	
	uint8 aData[CAN_DATA_MAX_LEN];
	//STU_CanData stuCanData[3];	
	uint32 ustimer;         //通过接收到的CAN针时间计时
	//uint8 ucUpTimer;
}STU_CanDataUp;

typedef struct _STU_DateTime
{
  uint8  ucYear;             //年
  uint8  ucMon;              //月
  uint8  ucDay;              //日
  uint8  ucHour;             //时
  uint8  ucMin;              //分
  uint8  ucSec;              //秒
  uint16 usmSec;             //毫秒
}STU_DataTime;

typedef struct _STU_Can_Message
{
    uint16 usmSec;
	uint32 CanID;
	uint8 ucLen;
	uint8 ucFF;           	// 是否标准帧
	uint8 data[8];
}STU_Can_Message;

void CanConfig_Data_Collect(STU_DataTime stuDate, uint8 *ptr);
uint16 CAN_GetCacheLocation_Function(uint8 ucGroupIndex,uint8 ucCanNumIndex);
uint8 CanConfig_Data_Send(void);
void CanConfig_Param_Init(void);
uint8 CANConfig_extract_Param(void);
void CAN_ClearCanDataUpbuff(void);
void CANConfig_ReadCANCFGFile_Init(void);
uint16 CANConfig_Read_CanParam(uint8* ptr);

#endif
