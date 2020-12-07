/*
 * Copyright(c)2018, XXXXX公司硬件研发部
 * All right reserved
 *
 * 文件名称: McuUpload.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 本文件为McuUpload功能模块协议层的头文件
 *
 *-----------------------------------------------------------------------------
 * 修改记录
 *-----------------------------------------------------------------------------
 *
 * 2018-02-07, by lxf, 创建本文件
 *
 */

#ifndef _MCUUPLOAD_H
#define _MCUUPLOAD_H

//-----常量定义----------------------------------------------------------------

#define Mcu_UPdateTimerOut   240

typedef struct _MCUFirmware
{
    uint8 ucLoadStep;        //0-默认状态;1-发送块启动;2-发送块数据;3-启动发送块结束命令;
                               //4-收到MCU对启动块下载结束的应答;5-升级结果
    uint8 ucRcvPackflag;     //0-收包未成功或者校验错误,1-收包成功并且校验通过
    uint8 ucMcuRespflag;     //控制器收块下载应答标志0-未应答,1-应答失败(不应答超时15秒会重发)
    uint8 ucUpDeviceType;    //需要升级的设备类型 0-终端，1-控制器，2-仪表
    uint8 ucMcuCmdTimerOut;  //终端与MUC通讯是超时计数器,最大超时15S
    uint16 usMcuUPdateTimerOut;//从终端开始向MCU发送升级包开始计时器 200k=150S
	uint8 aSendbuff[300];    //向MCU发送SDO组包
	uint16 usReadFlashSN;    //读取存储器页序号(逻辑序号)从0开始
	uint16 usTotalPackets;   //向mcu发送的升级文件总包数
	uint8 ucUploadResult;    //控制器升级结果0-失败 1-成功
	uint16 uscheck;
	uint8 ucCANFrameFormat;  //接收到MCU数据帧格式: 0-标准帧,1-扩展帧
	
}STUMCUFirmware;

typedef struct _ExTMCUUpgrade
{
    uint8 ucUpgradeStep;     //升级文件下载步骤  0-无升级进程;1-通知ExtMcu升级;2-发送升级包数据
                             //3-升级结果应答,0xFF-中间步骤(不希望重发?)
    uint16 ucTimer;           //运行定时器
    uint16 usRequestPacketSn; //请求升级包序号
    uint8 ucDev;             //升级设备编号
    uint8 ucResult;          //设备升级结果
    uint16 usTimeOut;        //升级文件下载总超时时间

}STUExtMCUUpgrade;



void McuUploadSend(void);
void McuUpload_CreatePackage(void);
void McuFirmwareTimerOut(void);
void Mcu_FirmwareDownload_Function(void);
void Ext_Mcu_Upgrade_Function(void);
void Ext_Mcu_Timeout_Function(void);
void Ext_Mcu_RecvUartData(uint8* ptr, uint16 uslen);
#endif
