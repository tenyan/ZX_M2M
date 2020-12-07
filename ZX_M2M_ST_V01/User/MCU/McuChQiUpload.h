/*
 * Copyright(c)2020, XXXXX公司硬件研发部
 * All right reserved
 *
 * 文件名称: McuChQiUpload.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为川崎控制器估计升级功能模块协议层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2020-01-17, by lxf, 创建本文件
 *
 */

#ifndef _MCUCHQIUPLOAD_H
#define _MCUCHQIUPLOAD_H

//-----常量定义----------------------------------------------------------------

#define KCMCU_RePeat_MAXCOUNT              10
#define KCMUC_SingleCMD_TIMEOUT            10          //单条CAN命令超时 10s

typedef struct _STU_KCMCUDownload
{
    uint8 ucLoadStep;        /*0-默认状态;
                               1-发送升级启动命令-Update Start Command ;
                               2-发送固件下载命令-Update Imfomation ;
                               3-发送固件数据包命令-Update Data(1-50);
                               4-发送参数下载命令-Update Imfomation;
                               5-发送参数数据包命令-Update Data(1-50);
                               6-升级结果-Data Complete Acknowledge */
    uint8 ucRcvPackflag;     //B0:0-川崎固件文件收包未成功或者校验错误,1-收包成功并且校验通过
                             //B1:0-川崎参数文件收包未成功或者校验错误,1-收包成功并且校验通过
  //  uint8 ucMcuRespflag;     //控制器收块下载应答标志0-未应答,1-应答失败(不应答超时15秒会重发)
    uint8 ucUpDeviceType;    //需要升级的设备类型 0-终端，1-威卡控制器，2-仪表,4-川崎控制器
    uint8 ucMcuCmdTimerOut;  //终端与MUC通讯是超时计数器,最大超时10S
    uint16 usMcuUPdateTimerOut;//从终端开始向MCU发送升级包开始计时器 200k=150S
	uint8 aSendbuff[512];    //向MCU发送升级包原始报文组包
	uint16 usReadFlashSN;    //读取存储器页序号(逻辑序号)从0开始
	uint16 usTotalPackets;   //向mcu发送的升级文件总包数
	uint8 ucUploadResult;    //控制器升级结果0-失败 1-成功
	uint16 uscheck;
	uint32 uiProgramTotPackets;   //程序总包数
	uint32 uiParameterTotPackets; //参数总包数
	uint32 uiOffsetAddr;      //升级数据取数据指向的偏移量
	uint8 ucRepeatSendCount;     //重发计数器
    uint32 uiSoftwareWSize;       //川崎程序文件大小
	uint32 uiParamSize;           //川崎参数文件大小
	
}STUKCMCUDownload;


void KCMCU_RecVCan_ACK(uint32 uiID, uint8 *arr);
void KCMcuFirmwareTimerOut(void);

#endif
