/*****************************************************************************
* Copyright (c) 2020-2040 XGIT Limited. All rights reserved.
* @FileName: tcw.h
* @Engineer: TenYan
* @Company:  徐工信息智能硬件部
* @version   V1.0
* @Date:     2020-11-20
* @brief     徐工重型起重机can通信协议定义
******************************************************************************/
#ifndef TCW_H_
#define TCW_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "types.h"

/******************************************************************************
* Macros
******************************************************************************/
#define TCW_BUILD_WORD(pbuf) (pbuf[1] + (pbuf[0]<<8))
#define TCW_BUILD_DWORD(pbuf) (pbuf[3] + (pbuf[2]<<8) + (pbuf[1]<<16) + (pbuf[0]<<24))

#define TCW_SPLIT_WORD(pbuf, word) do{ pbuf[0]=(uint8_t)(word>>8); pbuf[1]=(uint8_t)(word);}while(0)
#define TCW_SPLIT_DWORD(pbuf, dword) do{ pbuf[0]=(uint8_t)(word>>24); pbuf[1]=(uint8_t)(word>>16); pbuf[2]=(uint8_t)(word>>8); pbuf[3]=(uint8_t)(word);}while(0)

/******************************************************************************
* Macros(车型配置信息): 序号和地址定义
******************************************************************************/
//==TAG-A501通用状态字2(重型专用)===============================================
#define ZXINFO_A501_ADDR         0  // 起始地址
#define zxinfo_buffer_a501       (zxinfo_buffer+ZXINFO_A501_ADDR)  // 起始地址定义
#define ZXINFO_A501_POS1_ADDR    0                          // 工作状态位
#define ZXINFO_A501_POS2_ADDR    (ZXINFO_A501_POS1_ADDR+1)  // 环保状态位
#define ZXINFO_A501_POS3_ADDR    (ZXINFO_A501_POS2_ADDR+1)  // 绑定和锁车状态位
#define ZXINFO_A501_POS4_ADDR    (ZXINFO_A501_POS3_ADDR+1)  // 锁车原因和获取信息状态位
#define SIZE_OF_ZXINFO_A501      (ZXINFO_A501_POS4_ADDR+1)  // 总字节数(4B)

//==TAG-A504采集协议信息=========================================================
#define ZXINFO_A504_ADDR         (ZXINFO_A501_ADDR+SIZE_OF_ZXINFO_A501)  // 起始地址
#define zxinfo_buffer_a504       (zxinfo_buffer+ZXINFO_A504_ADDR)  // 起始地址定义
#define ZXINFO_A504_POS1_ADDR    0                          // 车型配置
#define ZXINFO_A504_POS2_ADDR    (ZXINFO_A504_POS1_ADDR+1)  // 上车CAN协议
#define ZXINFO_A504_POS3_ADDR    (ZXINFO_A504_POS2_ADDR+1)  // 下车CAN协议
#define SIZE_OF_ZXINFO_A504      (ZXINFO_A504_POS3_ADDR+1)  // 总字节数(3B)

//==TAG-A5FF终端状态信息==========================================================
#define ZXINFO_A5FF_ADDR         (ZXINFO_A504_ADDR+SIZE_OF_ZXINFO_A504)  // 起始地址
#define zxinfo_buffer_a5ff       (zxinfo_buffer+ZXINFO_A5FF_ADDR)  // 起始地址定义
#define ZXINFO_A5FF_POS1_ADDR    0                          // 外部电源电压
#define ZXINFO_A5FF_POS2_ADDR    (ZXINFO_A5FF_POS1_ADDR+2)  // 内置电池电压
#define ZXINFO_A5FF_POS3_ADDR    (ZXINFO_A5FF_POS2_ADDR+2)  // 开关量采集1
#define ZXINFO_A5FF_POS4_ADDR    (ZXINFO_A5FF_POS3_ADDR+1)  // 开关量采集2
#define ZXINFO_A5FF_POS5_ADDR    (ZXINFO_A5FF_POS4_ADDR+1)  // 开关量采集3
#define ZXINFO_A5FF_POS6_ADDR    (ZXINFO_A5FF_POS5_ADDR+1)  // 防拆类开关量
#define ZXINFO_A5FF_POS7_ADDR    (ZXINFO_A5FF_POS6_ADDR+1)  // CAN通信状态
#define ZXINFO_A5FF_POS8_ADDR    (ZXINFO_A5FF_POS7_ADDR+1)  // RTC时间
#define ZXINFO_A5FF_POS9_ADDR    (ZXINFO_A5FF_POS8_ADDR+6)  // 工作模式
#define ZXINFO_A5FF_POS10_ADDR    (ZXINFO_A5FF_POS9_ADDR+1)  // ST固件版本信息
#define SIZE_OF_ZXINFO_A5FF      (ZXINFO_A5FF_POS10_ADDR+2)  // 总字节数(11B)

// 车型配置信息数据缓存大小
#define SIZE_OF_ZXINFO_BUFFER  (ZXINFO_A5FF_ADDR+SIZE_OF_ZXINFO_A5FF)

/******************************************************************************
* Macros(上车通信): 起重机上车数据序号和地址定义
******************************************************************************/
//==TAG-A5A0力限器工况信息(单缸)=================================================
#define ZXUP_A5A0_ADDR    0  // 起始地址
#define zxup_buffer_a5a0  (zxup_buffer+ZXUP_A5A0_ADDR)  // 起始地址定义
#define ZXUP_A5A0_POS1    0  // 力限器配置1-2
#define ZXUP_A5A0_POS2    (ZXUP_A5A0_POS1+2)  // 工况代码(ABCDEFGH)
#define ZXUP_A5A0_POS3    (ZXUP_A5A0_POS2+4)  // 倍率
#define ZXUP_A5A0_POS4    (ZXUP_A5A0_POS3+1)  // 主臂头部角度
#define ZXUP_A5A0_POS5    (ZXUP_A5A0_POS4+2)  // 主臂根部角度
#define ZXUP_A5A0_POS6    (ZXUP_A5A0_POS5+2)  // 主臂长度
#define ZXUP_A5A0_POS7    (ZXUP_A5A0_POS6+2)  // 二节臂长度
#define ZXUP_A5A0_POS8    (ZXUP_A5A0_POS7+1)  // 节臂百分比
#define ZXUP_A5A0_POS9    (ZXUP_A5A0_POS8+8)  // 缸臂销锁死
#define ZXUP_A5A0_POS10   (ZXUP_A5A0_POS9+1)  // 副臂长度
#define ZXUP_A5A0_POS11   (ZXUP_A5A0_POS10+2)  // 副臂角度
#define ZXUP_A5A0_POS12   (ZXUP_A5A0_POS11+2)  // 副臂/塔臂根部角度
#define ZXUP_A5A0_POS13   (ZXUP_A5A0_POS12+2)  // 副臂/塔臂头部角度
#define ZXUP_A5A0_POS14   (ZXUP_A5A0_POS13+2)  // 腔压力
#define ZXUP_A5A0_POS15   (ZXUP_A5A0_POS14+8)  // 臂头高度
#define ZXUP_A5A0_POS16   (ZXUP_A5A0_POS15+2)  // 额定重量
#define ZXUP_A5A0_POS17   (ZXUP_A5A0_POS16+2)  // 实际重量
#define ZXUP_A5A0_POS18   (ZXUP_A5A0_POS17+2)  // 力矩百分比
#define ZXUP_A5A0_POS19   (ZXUP_A5A0_POS18+2)  // 工作幅度
#define ZXUP_A5A0_POS20   (ZXUP_A5A0_POS19+2)  // LMI故障代码1-4
#define ZXUP_A5A0_POS21   (ZXUP_A5A0_POS20+8)  // LMI故障代码5-8
#define ZXUP_A5A0_POS22   (ZXUP_A5A0_POS21+8)  // 非对称故障代码
#define ZXUP_A5A0_POS23   (ZXUP_A5A0_POS22+4)  // LMI运行时间
#define ZXUP_A5A0_POS24   (ZXUP_A5A0_POS23+4)  // 强制位
#define ZXUP_A5A0_POS25   (ZXUP_A5A0_POS24+1)  // 水平仪X
#define ZXUP_A5A0_POS26   (ZXUP_A5A0_POS25+2)  // 水平仪Y
#define ZXUP_A5A0_POS27   (ZXUP_A5A0_POS26+2)  // 风速
#define ZXUP_A5A0_POS28   (ZXUP_A5A0_POS27+2)  // 回转角度
#define ZXUP_A5A0_POS29   (ZXUP_A5A0_POS28+2)  // 强制和故障救援状态位
#define SIZE_OF_ZXUP_A5A0 (ZXUP_A5A0_POS29+1) // 总字节数(83B)

//==TAG-A5A1超起工况信息==========================================================
#define ZXUP_A5A1_ADDR    (ZXUP_A5A0_ADDR+SIZE_OF_ZXUP_A5A0)  // 起始地址81
#define zxup_buffer_a5a1  (zxup_buffer+ZXUP_A5A1_ADDR) // 起始地址定义
#define ZXUP_A5A1_POS1    0                   // 左超起卷扬角度
#define ZXUP_A5A1_POS2    (ZXUP_A5A1_POS1+2)  // 左超起拉力
#define ZXUP_A5A1_POS3    (ZXUP_A5A1_POS2+2)  // 左超起展开角度
#define ZXUP_A5A1_POS4    (ZXUP_A5A1_POS3+2)  // 左超起变幅角度
#define ZXUP_A5A1_POS5    (ZXUP_A5A1_POS4+2)  // 右超起卷扬角度
#define ZXUP_A5A1_POS6    (ZXUP_A5A1_POS5+2)  // 右超起拉力
#define ZXUP_A5A1_POS7    (ZXUP_A5A1_POS6+2)  // 右超起展开角度
#define ZXUP_A5A1_POS8    (ZXUP_A5A1_POS7+2)  // 右超起变幅角度
#define ZXUP_A5A1_POS9    (ZXUP_A5A1_POS8+2)  // 左超起-锁止和解锁
#define ZXUP_A5A1_POS10   (ZXUP_A5A1_POS9+1)  // 右超起-锁止和解锁
#define ZXUP_A5A1_POS11   (ZXUP_A5A1_POS10+1)  // 左超起齿数
#define ZXUP_A5A1_POS12   (ZXUP_A5A1_POS11+1)  // 右超起齿数
#define ZXUP_A5A1_POS13   (ZXUP_A5A1_POS12+1)  // 左超起张紧缸长度
#define ZXUP_A5A1_POS14   (ZXUP_A5A1_POS13+2)  // 右超起张紧缸长度
#define SIZE_OF_ZXUP_A5A1 (ZXUP_A5A1_POS14+2)  // 总字节数(24B)

//==TAG-A5A2塔臂工况信息==========================================================
#define ZXUP_A5A2_ADDR    (ZXUP_A5A1_ADDR+SIZE_OF_ZXUP_A5A1)  // 起始地址105
#define zxup_buffer_a5a2  (zxup_buffer+ZXUP_A5A2_ADDR)// 起始地址定义
#define ZXUP_A5A2_POS1    0                   // 塔臂拉力左
#define ZXUP_A5A2_POS2    (ZXUP_A5A2_POS1+2)  // 塔臂拉力右
#define ZXUP_A5A2_POS3    (ZXUP_A5A2_POS2+2)  // 防后倾压力
#define ZXUP_A5A2_POS4    (ZXUP_A5A2_POS3+2)  // 前支架角度
#define ZXUP_A5A2_POS5    (ZXUP_A5A2_POS4+2)  // 第三支架检测
#define SIZE_OF_ZXUP_A5A2 (ZXUP_A5A2_POS5+1)  // 总字节数(9B)

//==TAG-A5A5上车发动机信息========================================================
#define ZXUP_A5A5_ADDR    (ZXUP_A5A2_ADDR+SIZE_OF_ZXUP_A5A2)  // 起始地址114
#define zxup_buffer_a5a5  (zxup_buffer+ZXUP_A5A5_ADDR)// 起始地址定义
#define ZXUP_A5A5_POS1    0                   // 发动机转速
#define ZXUP_A5A5_POS2    (ZXUP_A5A5_POS1+2)  // 实际扭矩百分比
#define ZXUP_A5A5_POS3    (ZXUP_A5A5_POS2+1)  // 摩擦扭矩百分比
#define ZXUP_A5A5_POS4    (ZXUP_A5A5_POS3+1)  // 进气歧管压力
#define ZXUP_A5A5_POS5    (ZXUP_A5A5_POS4+1)  // 进气歧管温度
#define ZXUP_A5A5_POS6    (ZXUP_A5A5_POS5+1)  // 冷却液温度
#define ZXUP_A5A5_POS7    (ZXUP_A5A5_POS6+1)  // 机油温度
#define ZXUP_A5A5_POS8    (ZXUP_A5A5_POS7+2)  // 机油液位
#define ZXUP_A5A5_POS9    (ZXUP_A5A5_POS8+1)  // 机油压力
#define ZXUP_A5A5_POS10   (ZXUP_A5A5_POS9+1)  // 发动机总运行时间
#define ZXUP_A5A5_POS11   (ZXUP_A5A5_POS10+4)  // 油中有水指示灯
#define ZXUP_A5A5_POS12   (ZXUP_A5A5_POS11+1)  // 油门踏板百分比
#define ZXUP_A5A5_POS13   (ZXUP_A5A5_POS12+1)  // 发动机燃油消耗率
#define ZXUP_A5A5_POS14   (ZXUP_A5A5_POS13+2)  // 发动机平均燃油消耗率
#define ZXUP_A5A5_POS15   (ZXUP_A5A5_POS14+2)  // 燃油液位
#define ZXUP_A5A5_POS16   (ZXUP_A5A5_POS15+1)  // 发动机扭矩模式(调速器类型)
#define SIZE_OF_ZXUP_A5A5 (ZXUP_A5A5_POS16+1)  // 总字节数(23B)

//==TAG-A5A6手柄信息==============================================================
#define ZXUP_A5A6_ADDR    (ZXUP_A5A5_ADDR+SIZE_OF_ZXUP_A5A5)  // 起始地址137
#define zxup_buffer_a5a6  (zxup_buffer+ZXUP_A5A6_ADDR)// 起始地址定义
#define ZXUP_A5A6_POS1    0                   // 左手柄状态位
#define ZXUP_A5A6_POS2    (ZXUP_A5A6_POS1+1)  // 左手柄X输出
#define ZXUP_A5A6_POS3    (ZXUP_A5A6_POS2+2)  // 左手柄Y输出
#define ZXUP_A5A6_POS4    (ZXUP_A5A6_POS3+2)  // 右手柄状态位
#define ZXUP_A5A6_POS5    (ZXUP_A5A6_POS4+1)  // 右手柄X输出
#define ZXUP_A5A6_POS6    (ZXUP_A5A6_POS5+2)  // 右手柄Y输出
#define SIZE_OF_ZXUP_A5A6 (ZXUP_A5A6_POS6+2)  // 总字节数(10B)

//==TAG-A5A7显示器1信息===========================================================
#define ZXUP_A5A7_ADDR    (ZXUP_A5A6_ADDR+SIZE_OF_ZXUP_A5A6)  // 起始地址147
#define zxup_buffer_a5a7  (zxup_buffer+ZXUP_A5A7_ADDR)// 起始地址定义
#define ZXUP_A5A7_POS1    0                   // 产品配置
#define ZXUP_A5A7_POS2    (ZXUP_A5A7_POS1+4)  // 工况代码
#define ZXUP_A5A7_POS3    (ZXUP_A5A7_POS2+4)  // 主卷起速度
#define ZXUP_A5A7_POS4    (ZXUP_A5A7_POS3+1)  // 主卷落速度
#define ZXUP_A5A7_POS5    (ZXUP_A5A7_POS4+1)  // 副卷起速度
#define ZXUP_A5A7_POS6    (ZXUP_A5A7_POS5+1)  // 副卷落速度
#define ZXUP_A5A7_POS7    (ZXUP_A5A7_POS6+1)  // 变幅起速度
#define ZXUP_A5A7_POS8    (ZXUP_A5A7_POS7+1)  // 变幅落速度
#define ZXUP_A5A7_POS9    (ZXUP_A5A7_POS8+1)  // 左回转速度
#define ZXUP_A5A7_POS10   (ZXUP_A5A7_POS9+1)  // 右回转速度
#define ZXUP_A5A7_POS11   (ZXUP_A5A7_POS10+1)  // 目标组合
#define ZXUP_A5A7_POS12   (ZXUP_A5A7_POS11+1)  // 状态位1
#define ZXUP_A5A7_POS13   (ZXUP_A5A7_POS12+1)  // 状态位2
#define ZXUP_A5A7_POS14   (ZXUP_A5A7_POS13+1)  // 伸缩模式
#define SIZE_OF_ZXUP_A5A7 (ZXUP_A5A7_POS14+1)  // 总字节数(20B)

//==TAG-A5A8显示器2信息===========================================================
#define ZXUP_A5A8_ADDR    (ZXUP_A5A7_ADDR+SIZE_OF_ZXUP_A5A7)  // 起始地址167
#define zxup_buffer_a5a8  (zxup_buffer+ZXUP_A5A8_ADDR)  // 起始地址定义
#define ZXUP_A5A8_POS1    0                   // 超起操控
#define ZXUP_A5A8_POS2    (ZXUP_A5A8_POS1+8)  // 超起维修
#define SIZE_OF_ZXUP_A5A8 (ZXUP_A5A8_POS2+2)  // 总字节数(10B)

//==TAG-A5A9总线面板信息==========================================================
#define ZXUP_A5A9_ADDR    (ZXUP_A5A8_ADDR+SIZE_OF_ZXUP_A5A8)  // 起始地址177
#define zxup_buffer_a5a9  (zxup_buffer+ZXUP_A5A9_ADDR)// 起始地址定义
#define ZXUP_A5A9_POS1    0                   // 面板1
#define ZXUP_A5A9_POS2    (ZXUP_A5A9_POS1+2)  // 面板2
#define ZXUP_A5A9_POS3    (ZXUP_A5A9_POS2+2)  // 面板3
#define SIZE_OF_ZXUP_A5A9 (ZXUP_A5A9_POS3+2)  // 总字节数(6B)

//==TAG-A5AA无线操控信息==========================================================
#define ZXUP_A5AA_ADDR    (ZXUP_A5A9_ADDR+SIZE_OF_ZXUP_A5A9)  // 起始地址183
#define zxup_buffer_a5aa  (zxup_buffer+ZXUP_A5AA_ADDR)// 起始地址定义
#define ZXUP_A5AA_POS1    0                   // Msg1
#define ZXUP_A5AA_POS2    (ZXUP_A5AA_POS1+8)  // Msg2
#define ZXUP_A5AA_POS3    (ZXUP_A5AA_POS2+8)  // Msg3
#define ZXUP_A5AA_POS4    (ZXUP_A5AA_POS3+8)  // Msg4
#define SIZE_OF_ZXUP_A5AA (ZXUP_A5AA_POS4+8)  // 总字节数(32B)

//==TAG-A5AB网络拓扑信息==========================================================
#define ZXUP_A5AB_ADDR    (ZXUP_A5AA_ADDR+SIZE_OF_ZXUP_A5AA)  // 起始地址215
#define zxup_buffer_a5ab  (zxup_buffer+ZXUP_A5AB_ADDR)// 起始地址定义
#define ZXUP_A5AB_POS1    0                   // 节点状态
#define SIZE_OF_ZXUP_A5AB (ZXUP_A5AB_POS1+8)  // 总字节数(8B)

//==TAG-A5AC伸缩逻辑信息==========================================================
#define ZXUP_A5AC_ADDR    (ZXUP_A5AB_ADDR+SIZE_OF_ZXUP_A5AB)  // 起始地址223
#define zxup_buffer_a5ac  (zxup_buffer+ZXUP_A5AC_ADDR)// 起始地址定义
#define ZXUP_A5AC_POS1    0                   // 伸缩限制和解除
#define SIZE_OF_ZXUP_A5AC (ZXUP_A5AC_POS1+6)  // 总字节数(6B)

//==TAG-A5AD变幅逻辑信息==========================================================
#define ZXUP_A5AD_ADDR    (ZXUP_A5AC_ADDR+SIZE_OF_ZXUP_A5AC)  // 起始地址229
#define zxup_buffer_a5ad  (zxup_buffer+ZXUP_A5AD_ADDR)// 起始地址定义
#define ZXUP_A5AD_POS1    0                   // 起落限制和解除
#define SIZE_OF_ZXUP_A5AD (ZXUP_A5AD_POS1+8)  // 总字节数(8B)

//==TAG-A5AE回转逻辑信息==========================================================
#define ZXUP_A5AE_ADDR    (ZXUP_A5AD_ADDR+SIZE_OF_ZXUP_A5AD)  // 起始地址237
#define zxup_buffer_a5ae  (zxup_buffer+ZXUP_A5AE_ADDR)// 起始地址定义
#define ZXUP_A5AE_POS1    0                   // 左回限制1
#define ZXUP_A5AE_POS2    (ZXUP_A5AE_POS1+1)  // 左回限制2
#define ZXUP_A5AE_POS3    (ZXUP_A5AE_POS2+1)  // 左回解除
#define ZXUP_A5AE_POS4    (ZXUP_A5AE_POS3+1)  // 右回限制1
#define ZXUP_A5AE_POS5    (ZXUP_A5AE_POS4+1)  // 右回限制2
#define ZXUP_A5AE_POS6    (ZXUP_A5AE_POS5+1)  // 右回解除
#define SIZE_OF_ZXUP_A5AE (ZXUP_A5AE_POS6+1)  // 总字节数(6B)

//==TAG-A5AF主卷扬逻辑信息========================================================
#define ZXUP_A5AF_ADDR    (ZXUP_A5AE_ADDR+SIZE_OF_ZXUP_A5AE)  // 起始地址243
#define zxup_buffer_a5af  (zxup_buffer+ZXUP_A5AF_ADDR)// 起始地址定义
#define ZXUP_A5AF_POS1    0                   // 主卷扬起落限制和解除
#define SIZE_OF_ZXUP_A5AF (ZXUP_A5AF_POS1+5)  // 总字节数(5B)

//==TAG-A5B0副卷扬逻辑信息========================================================
#define ZXUP_A5B0_ADDR    (ZXUP_A5AF_ADDR+SIZE_OF_ZXUP_A5AF)  // 起始地址248
#define zxup_buffer_a5b0  (zxup_buffer+ZXUP_A5B0_ADDR)// 起始地址定义
#define ZXUP_A5B0_POS1    0                   // 副卷扬起落限制和解除
#define SIZE_OF_ZXUP_A5B0 (ZXUP_A5B0_POS1+4)  // 总字节数(4B)

//==TAG-A5B1超起逻辑信息==========================================================
#define ZXUP_A5B1_ADDR    (ZXUP_A5B0_ADDR+SIZE_OF_ZXUP_A5B0)  // 起始地址252
#define zxup_buffer_a5b1  (zxup_buffer+ZXUP_A5B1_ADDR)// 起始地址定义
#define ZXUP_A5B1_POS1    0                   // 超起目标展开角度
#define ZXUP_A5B1_POS2    (ZXUP_A5B1_POS1+1)  // 超起目标张紧角度
#define ZXUP_A5B1_POS3    (ZXUP_A5B1_POS2+2)  // 超起一键张紧限制逻辑
#define ZXUP_A5B1_POS4    (ZXUP_A5B1_POS3+2)  // 张紧拉力状态位
#define SIZE_OF_ZXUP_A5B1 (ZXUP_A5B1_POS4+1)  // 总字节数(6B)

//==TAG-A5B2塔臂逻辑信息==========================================================
#define ZXUP_A5B2_ADDR    (ZXUP_A5B1_ADDR+SIZE_OF_ZXUP_A5B1)  // 起始地址258
#define zxup_buffer_a5b2  (zxup_buffer+ZXUP_A5B2_ADDR)// 起始地址定义
#define ZXUP_A5B2_POS1    0                        // 塔卷起限制
#define ZXUP_A5B2_POS2    (ZXUP_A5B2_POS1+2)  // 塔卷落限制
#define ZXUP_A5B2_POS3    (ZXUP_A5B2_POS2+2)  // 塔臂限制伸臂
#define ZXUP_A5B2_POS4    (ZXUP_A5B2_POS3+1)  // 塔臂限制缩臂
#define ZXUP_A5B2_POS5    (ZXUP_A5B2_POS4+1)  // 塔臂限制变幅起
#define ZXUP_A5B2_POS6    (ZXUP_A5B2_POS5+1)  // 塔臂限制变幅落
#define ZXUP_A5B2_POS7    (ZXUP_A5B2_POS6+1)  // 塔臂限制主卷起
#define ZXUP_A5B2_POS8    (ZXUP_A5B2_POS7+1)  // 塔臂限制主卷落
#define SIZE_OF_ZXUP_A5B2 (ZXUP_A5B2_POS8+1)  // 总字节数(10B)

//==TAG-A5B3液压油温度============================================================
#define ZXUP_A5B3_ADDR    (ZXUP_A5B2_ADDR+SIZE_OF_ZXUP_A5B2)  // 起始地址268
#define zxup_buffer_a5b3  (zxup_buffer+ZXUP_A5B3_ADDR)// 起始地址定义
#define ZXUP_A5B3_POS1    0                        // 液压油温度
#define ZXUP_A5B3_POS2    (ZXUP_A5B3_POS1+2)  // 液压油位
#define ZXUP_A5B3_POS3    (ZXUP_A5B3_POS2+1)  // 回油阻塞报警
#define SIZE_OF_ZXUP_A5B3 (ZXUP_A5B3_POS3+1)  // 总字节数(4B)

//==TAG-A5B4主泵信息==============================================================
#define ZXUP_A5B4_ADDR    (ZXUP_A5B3_ADDR+SIZE_OF_ZXUP_A5B3)  // 起始地址272
#define zxup_buffer_a5b4  (zxup_buffer+ZXUP_A5B4_ADDR)// 起始地址定义
#define ZXUP_A5B4_POS1    0                   // 1#主泵电磁阀
#define ZXUP_A5B4_POS2    (ZXUP_A5B4_POS1+2)  // 2#主泵电磁阀
#define ZXUP_A5B4_POS3    (ZXUP_A5B4_POS2+2)  // 主泵压力
#define SIZE_OF_ZXUP_A5B4 (ZXUP_A5B4_POS3+2)  // 总字节数(6B)

//==TAG-A5B5主阀1 XHVME4400P1=====================================================
#define ZXUP_A5B5_ADDR    (ZXUP_A5B4_ADDR+SIZE_OF_ZXUP_A5B4)  // 起始地址278
#define zxup_buffer_a5b5  (zxup_buffer+ZXUP_A5B5_ADDR)// 起始地址定义
#define ZXUP_A5B5_POS1    0                   // 伸电磁阀
#define ZXUP_A5B5_POS2    (ZXUP_A5B5_POS1+2)  // 缩电磁阀
#define ZXUP_A5B5_POS3    (ZXUP_A5B5_POS2+2)  // 变幅起电磁阀
#define ZXUP_A5B5_POS4    (ZXUP_A5B5_POS3+2)  // 变幅落电磁阀
#define ZXUP_A5B5_POS5    (ZXUP_A5B5_POS4+2)  // 主卷起电磁阀
#define ZXUP_A5B5_POS6    (ZXUP_A5B5_POS5+2)  // 主卷落电磁阀
#define ZXUP_A5B5_POS7    (ZXUP_A5B5_POS6+2)  // 副卷落电磁阀
#define ZXUP_A5B5_POS8    (ZXUP_A5B5_POS7+2)  // 副卷落电磁阀
#define ZXUP_A5B5_POS9    (ZXUP_A5B5_POS8+2)  // MP1压力
#define ZXUP_A5B5_POS10   (ZXUP_A5B5_POS9+2)  // LS1压力
#define ZXUP_A5B5_POS11   (ZXUP_A5B5_POS10+2)  // MP2压力
#define ZXUP_A5B5_POS12   (ZXUP_A5B5_POS11+2)  // LS2压力
#define ZXUP_A5B5_POS13   (ZXUP_A5B5_POS12+2)  // 合流电磁阀
#define SIZE_OF_ZXUP_A5B5 (ZXUP_A5B5_POS13+1)  // 总字节数(25B)

//==TAG-A5B6主阀3 大吨位==========================================================
#define ZXUP_A5B6_ADDR    (ZXUP_A5B5_ADDR+SIZE_OF_ZXUP_A5B5)  // 起始地址303
#define zxup_buffer_a5b6  (zxup_buffer+ZXUP_A5B6_ADDR)// 起始地址定义
#define ZXUP_A5B6_POS1    0                   // 状态位
#define SIZE_OF_ZXUP_A5B6 (ZXUP_A5B6_POS1+1)  // 总字节数(1B)

//==TAG-A5B7主泵信息==============================================================
#define ZXUP_A5B7_ADDR    (ZXUP_A5B6_ADDR+SIZE_OF_ZXUP_A5B6)  // 起始地址304
#define zxup_buffer_a5b7  (zxup_buffer+ZXUP_A5B7_ADDR)// 起始地址定义
#define ZXUP_A5B7_POS1    0                   // 伸缩缸压力
#define ZXUP_A5B7_POS2    (ZXUP_A5B7_POS1+2)  // 伸缩缸长度
#define ZXUP_A5B7_POS3    (ZXUP_A5B7_POS2+2)  // 电控平衡阀
#define SIZE_OF_ZXUP_A5B7 (ZXUP_A5B7_POS3+2)  // 总字节数(6B)

//==TAG-A5B8塔臂逻辑信息==========================================================
#define ZXUP_A5B8_ADDR    (ZXUP_A5B7_ADDR+SIZE_OF_ZXUP_A5B7)  // 起始地址310
#define zxup_buffer_a5b8  (zxup_buffer+ZXUP_A5B8_ADDR)// 起始地址定义
#define ZXUP_A5B8_POS1    0                   // 销锁死和解锁状态位
#define ZXUP_A5B8_POS2    (ZXUP_A5B8_POS1+1)  // 头体标志
#define ZXUP_A5B8_POS3    (ZXUP_A5B8_POS2+1)  // 前缸头体检测开关8
#define ZXUP_A5B8_POS4    (ZXUP_A5B8_POS3+1)  // 后缸头体检测开关8
#define ZXUP_A5B8_POS5    (ZXUP_A5B8_POS4+1)  // 臂位
#define SIZE_OF_ZXUP_A5B8 (ZXUP_A5B8_POS5+1)  // 总字节数(5B)

//==TAG-A5B9缸臂销控制阀==========================================================
#define ZXUP_A5B9_ADDR    (ZXUP_A5B8_ADDR+SIZE_OF_ZXUP_A5B8)  // 起始地址315
#define zxup_buffer_a5b9  (zxup_buffer+ZXUP_A5B9_ADDR)// 起始地址定义
#define ZXUP_A5B9_POS1    0                   // 状态字节
#define ZXUP_A5B9_POS2    (ZXUP_A5B9_POS1+1)  // 蓄能器压力
#define SIZE_OF_ZXUP_A5B9 (ZXUP_A5B9_POS2+2)  // 总字节数(3B)

//==TAG-A5BA变幅平衡阀============================================================
#define ZXUP_A5BA_ADDR    (ZXUP_A5B9_ADDR+SIZE_OF_ZXUP_A5B9)  // 起始地址318
#define zxup_buffer_a5ba  (zxup_buffer+ZXUP_A5BA_ADDR)// 起始地址定义
#define ZXUP_A5BA_POS1    0                   // 左变幅平衡阀电流
#define ZXUP_A5BA_POS2    (ZXUP_A5BA_POS1+2)  // 右变幅平衡阀电流
#define SIZE_OF_ZXUP_A5BA (ZXUP_A5BA_POS2+2)  // 总字节数(4B)

//==TAG-A5BB主卷泵================================================================
#define ZXUP_A5BB_ADDR    (ZXUP_A5BA_ADDR+SIZE_OF_ZXUP_A5BA)  // 起始地址322
#define zxup_buffer_a5bb  (zxup_buffer+ZXUP_A5BB_ADDR)// 起始地址定义
#define ZXUP_A5BB_POS1    0                   // 起升电磁阀
#define ZXUP_A5BB_POS2    (ZXUP_A5BB_POS1+2)  // 下落电磁阀
#define ZXUP_A5BB_POS3    (ZXUP_A5BB_POS2+2)  // 油泵压力
#define SIZE_OF_ZXUP_A5BB (ZXUP_A5BB_POS3+2)  // 总字节数(6B)

//==TAG-A5BC主卷马达==============================================================
#define ZXUP_A5BC_ADDR    (ZXUP_A5BB_ADDR+SIZE_OF_ZXUP_A5BB)  // 起始地址328
#define zxup_buffer_a5bc  (zxup_buffer+ZXUP_A5BC_ADDR)// 起始地址定义
#define ZXUP_A5BC_POS1    0                   // 马达电流
#define ZXUP_A5BC_POS2    (ZXUP_A5BC_POS1+2)  // 卷筒转速
#define ZXUP_A5BC_POS3    (ZXUP_A5BC_POS2+2)  // 状态位
#define SIZE_OF_ZXUP_A5BC (ZXUP_A5BC_POS3+1)  // 总字节数(5B)

//==TAG-A5BD塔卷泵================================================================
#define ZXUP_A5BD_ADDR    (ZXUP_A5BC_ADDR+SIZE_OF_ZXUP_A5BC)  // 起始地址333
#define zxup_buffer_a5bd  (zxup_buffer+ZXUP_A5BD_ADDR)// 起始地址定义
#define ZXUP_A5BD_POS1    0                   // 起升电磁阀
#define ZXUP_A5BD_POS2    (ZXUP_A5BD_POS1+2)  // 下落电磁阀
#define ZXUP_A5BD_POS3    (ZXUP_A5BD_POS2+2)  // 油泵压力
#define SIZE_OF_ZXUP_A5BD (ZXUP_A5BD_POS3+2)  // 总字节数(6B)

//==TAG-A5BE塔卷马达==============================================================
#define ZXUP_A5BE_ADDR    (ZXUP_A5BD_ADDR+SIZE_OF_ZXUP_A5BD)  // 起始地址339
#define zxup_buffer_a5be  (zxup_buffer+ZXUP_A5BE_ADDR)// 起始地址定义
#define ZXUP_A5BE_POS1    0                   // 马达电流
#define ZXUP_A5BE_POS2    (ZXUP_A5BE_POS1+2)  // 卷筒转速
#define ZXUP_A5BE_POS3    (ZXUP_A5BE_POS2+2)  // 状态位
#define SIZE_OF_ZXUP_A5BE (ZXUP_A5BE_POS3+1)  // 总字节数(5B)

//==TAG-A5BF回转泵================================================================
#define ZXUP_A5BF_ADDR    (ZXUP_A5BE_ADDR+SIZE_OF_ZXUP_A5BE)  // 起始地址344
#define zxup_buffer_a5bf  (zxup_buffer+ZXUP_A5BF_ADDR)// 起始地址定义
#define ZXUP_A5BF_POS1    0                   // 左回转电磁阀
#define ZXUP_A5BF_POS2    (ZXUP_A5BF_POS1+2)  // 右回转电磁阀
#define ZXUP_A5BF_POS3    (ZXUP_A5BF_POS2+2)  // 回转缓冲阀
#define ZXUP_A5BF_POS4    (ZXUP_A5BF_POS3+2)  // 油泵压力（回转压力检测）
#define SIZE_OF_ZXUP_A5BF (ZXUP_A5BF_POS4+2)  // 总字节数(8B)

//==TAG-A5C0回转制动阀============================================================
#define ZXUP_A5C0_ADDR    (ZXUP_A5BF_ADDR+SIZE_OF_ZXUP_A5BF)  // 起始地址352
#define zxup_buffer_a5c0  (zxup_buffer+ZXUP_A5C0_ADDR)// 起始地址定义
#define ZXUP_A5C0_POS1    0                   // 字节状态位
#define ZXUP_A5C0_POS2    (ZXUP_A5C0_POS1+1)  // 卷筒转速
#define SIZE_OF_ZXUP_A5C0 (ZXUP_A5C0_POS2+2)  // 总字节数(3B)

//==TAG-A5C1辅助阀1===============================================================
#define ZXUP_A5C1_ADDR    (ZXUP_A5C0_ADDR+SIZE_OF_ZXUP_A5C0)  // 起始地址355
#define zxup_buffer_a5c1  (zxup_buffer+ZXUP_A5C1_ADDR)// 起始地址定义
#define ZXUP_A5C1_POS1    0                   // 字节状态位1
#define ZXUP_A5C1_POS2    (ZXUP_A5C1_POS1+1)  // 字节状态位2
#define SIZE_OF_ZXUP_A5C1 (ZXUP_A5C1_POS2+1)  // 总字节数(2B)

//==TAG-A5C2辅助阀2(超大吨位)=====================================================
#define ZXUP_A5C2_ADDR    (ZXUP_A5C1_ADDR+SIZE_OF_ZXUP_A5C1)  // 起始地址357
#define zxup_buffer_a5c2  (zxup_buffer+ZXUP_A5C2_ADDR)// 起始地址定义
#define ZXUP_A5C2_POS1    0                   // 压力选择
#define ZXUP_A5C2_POS2    (ZXUP_A5C2_POS1+2)  // 字节状态位
#define SIZE_OF_ZXUP_A5C2 (ZXUP_A5C2_POS2+1)  // 总字节数(3B)

//==TAG-A5C3左超起阀组============================================================
#define ZXUP_A5C3_ADDR    (ZXUP_A5C2_ADDR+SIZE_OF_ZXUP_A5C2)  // 起始地址360
#define zxup_buffer_a5c3  (zxup_buffer+ZXUP_A5C3_ADDR)// 起始地址定义
#define ZXUP_A5C3_POS1    0                   // 卷扬起
#define ZXUP_A5C3_POS2    (ZXUP_A5C3_POS1+2)  // 卷扬落
#define ZXUP_A5C3_POS3    (ZXUP_A5C3_POS2+2)  // 超起马达变量
#define ZXUP_A5C3_POS4    (ZXUP_A5C3_POS3+2)  // 字节状态位1
#define ZXUP_A5C3_POS5    (ZXUP_A5C3_POS4+1)  // 字节状态位2
#define ZXUP_A5C3_POS6    (ZXUP_A5C3_POS5+1)  // 张紧缸伸
#define ZXUP_A5C3_POS7    (ZXUP_A5C3_POS6+2)  // 张紧缸缩
#define SIZE_OF_ZXUP_A5C3 (ZXUP_A5C3_POS7+2)  // 总字节数(12B)

//==TAG-A5C4右超起阀组============================================================
#define ZXUP_A5C4_ADDR    (ZXUP_A5C3_ADDR+SIZE_OF_ZXUP_A5C3)  // 起始地址372
#define zxup_buffer_a5c4  (zxup_buffer+ZXUP_A5C4_ADDR)// 起始地址定义
#define ZXUP_A5C4_POS1    0                   // 卷扬起
#define ZXUP_A5C4_POS2    (ZXUP_A5C4_POS1+2)  // 卷扬落
#define ZXUP_A5C4_POS3    (ZXUP_A5C4_POS2+2)  // 超起马达变量
#define ZXUP_A5C4_POS4    (ZXUP_A5C4_POS3+2)  // 字节状态位1
#define ZXUP_A5C4_POS5    (ZXUP_A5C4_POS4+1)  // 字节状态位2
#define ZXUP_A5C4_POS6    (ZXUP_A5C4_POS5+1)  // 张紧缸伸
#define ZXUP_A5C4_POS7    (ZXUP_A5C4_POS6+2)  // 张紧缸缩
#define SIZE_OF_ZXUP_A5C4 (ZXUP_A5C4_POS7+2)  // 总字节数(12B)

//==TAG-A5C8作业油耗信息==========================================================
#define ZXUP_A5C8_ADDR    (ZXUP_A5C4_ADDR+SIZE_OF_ZXUP_A5C4)  // 起始地址384
#define zxup_buffer_a5c8  (zxup_buffer+ZXUP_A5C8_ADDR)// 起始地址定义
#define ZXUP_A5C8_POS1    0                   // 作业总运行时间
#define ZXUP_A5C8_POS2    (ZXUP_A5C8_POS1+4)  // 作业燃油总消耗量
#define ZXUP_A5C8_POS3    (ZXUP_A5C8_POS2+4)  // 作业平均油耗
#define SIZE_OF_ZXUP_A5C8 (ZXUP_A5C8_POS3+2)  // 总字节数(10B)

//==TAG-A5C9 ECU响应锁车CAN帧=====================================================
#define ZXUP_A5C9_ADDR    (ZXUP_A5C8_ADDR+SIZE_OF_ZXUP_A5C8)  // 起始地址394
#define zxup_buffer_a5c9  (zxup_buffer+ZXUP_A5C9_ADDR)// 起始地址定义
#define ZXUP_A5C9_POS1    0                   // 作业总运行时间
#define ZXUP_A5C9_POS2    (ZXUP_A5C9_POS1+1)  // 作业燃油总消耗量
#define ZXUP_A5C9_POS3    (ZXUP_A5C9_POS2+4)  // 作业平均油耗
#define SIZE_OF_ZXUP_A5C9 (ZXUP_A5C9_POS3+8)  // 总字节数(13B)

// 上车数据缓存大小
#define SIZE_OF_ZXUP_BUFFER  (ZXUP_A5C9_ADDR+SIZE_OF_ZXUP_A5C9) // 407

/******************************************************************************
* Macros(下车通信): 起重机下车数据序号和地址定义
******************************************************************************/
//==TAG-A5E0节点状态==============================================================
#define ZXDOWN_A5E0_ADDR    0  // 起始地址
#define zxdown_buffer_a5e0  (zxdown_buffer+ZXDOWN_A5E0_ADDR)  // 起始地址定义
#define ZXDOWN_A5E0_POS1    0                     // 字节状态位1
#define ZXDOWN_A5E0_POS2    (ZXDOWN_A5E0_POS1+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E0 (ZXDOWN_A5E0_POS2+1)  // 总字节数(2B)

//==TAG-A5E1传动系统==============================================================
#define ZXDOWN_A5E1_ADDR    (ZXDOWN_A5E0_ADDR+SIZE_OF_ZXDOWN_A5E0)  // 起始地址
#define zxdown_buffer_a5e1  (zxdown_buffer+ZXDOWN_A5E1_ADDR) // 起始地址定义
#define ZXDOWN_A5E1_POS1    0                     // 变速箱档位
#define ZXDOWN_A5E1_POS2    (ZXDOWN_A5E1_POS1+1)  // 字节状态位
#define ZXDOWN_A5E1_POS3    (ZXDOWN_A5E1_POS2+1)  // 变速箱油温
#define ZXDOWN_A5E1_POS4    (ZXDOWN_A5E1_POS3+2)  // 变速箱进气气压
#define ZXDOWN_A5E1_POS5    (ZXDOWN_A5E1_POS4+1)  // 离合器位置
#define SIZE_OF_ZXDOWN_A5E1 (ZXDOWN_A5E1_POS5+1)  // 总字节数(6B)

//==TAG-A5E2支腿相关信息==========================================================
#define ZXDOWN_A5E2_ADDR    (ZXDOWN_A5E1_ADDR+SIZE_OF_ZXDOWN_A5E1)  // 起始地址
#define zxdown_buffer_a5e2  (zxdown_buffer+ZXDOWN_A5E2_ADDR)// 起始地址定义
#define ZXDOWN_A5E2_POS1    0                     // 字节状态位1
#define ZXDOWN_A5E2_POS2    (ZXDOWN_A5E2_POS1+1)  // 字节状态位2
#define ZXDOWN_A5E2_POS3    (ZXDOWN_A5E2_POS2+1)  // 字节状态位3
#define SIZE_OF_ZXDOWN_A5E2 (ZXDOWN_A5E2_POS3+1)  // 总字节数(3B)

//==TAG-A5E3悬挂系统==============================================================
#define ZXDOWN_A5E3_ADDR    (ZXDOWN_A5E2_ADDR+SIZE_OF_ZXDOWN_A5E2)  // 起始地址
#define zxdown_buffer_a5e3  (zxdown_buffer+ZXDOWN_A5E3_ADDR)// 起始地址定义
#define ZXDOWN_A5E3_POS1    0                     // 悬挂压力
#define ZXDOWN_A5E3_POS2    (ZXDOWN_A5E3_POS1+8)  // 悬挂行程
#define ZXDOWN_A5E3_POS3    (ZXDOWN_A5E3_POS2+8)  // 整车重量
#define ZXDOWN_A5E3_POS4    (ZXDOWN_A5E3_POS3+4)  // 字节状态位1
#define ZXDOWN_A5E3_POS5    (ZXDOWN_A5E3_POS4+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E3 (ZXDOWN_A5E3_POS5+1)  // 总字节数(22B)

//==TAG-A5E4转向系统（全地面）====================================================
#define ZXDOWN_A5E4_ADDR    (ZXDOWN_A5E3_ADDR+SIZE_OF_ZXDOWN_A5E3)  // 起始地址
#define zxdown_buffer_a5e4  (zxdown_buffer+ZXDOWN_A5E4_ADDR)// 起始地址定义
#define ZXDOWN_A5E4_POS1    0                     // 一二轴转向角度
#define ZXDOWN_A5E4_POS2    (ZXDOWN_A5E4_POS1+8)  // 三四五六轴转向角度
#define ZXDOWN_A5E4_POS3    (ZXDOWN_A5E4_POS2+8)  // 一二轴传感器电流
#define ZXDOWN_A5E4_POS4    (ZXDOWN_A5E4_POS3+8)  // 三四五六轴传感器电流
#define ZXDOWN_A5E4_POS5    (ZXDOWN_A5E4_POS4+8)  // 当前转向模式
#define ZXDOWN_A5E4_POS6    (ZXDOWN_A5E4_POS5+1)  // 目标转向模式
#define ZXDOWN_A5E4_POS7    (ZXDOWN_A5E4_POS6+1)  // 转向系统压力
#define ZXDOWN_A5E4_POS8    (ZXDOWN_A5E4_POS7+2)  // 比例调压阀电流
#define ZXDOWN_A5E4_POS9    (ZXDOWN_A5E4_POS8+2)  // 123桥左右转阀占空比
#define ZXDOWN_A5E4_POS10   (ZXDOWN_A5E4_POS9+6)  // 456桥左右转阀占空比
#define ZXDOWN_A5E4_POS11   (ZXDOWN_A5E4_POS10+6)  // 桥锁止阀
#define SIZE_OF_ZXDOWN_A5E4 (ZXDOWN_A5E4_POS11+1)  // 总字节数(51B)

//==TAG-A5E5转向系统（汽车）======================================================
#define ZXDOWN_A5E5_ADDR    (ZXDOWN_A5E4_ADDR+SIZE_OF_ZXDOWN_A5E4)  // 起始地址
#define zxdown_buffer_a5e5  (zxdown_buffer+ZXDOWN_A5E5_ADDR)// 起始地址定义
#define ZXDOWN_A5E5_POS1    0                     // 当前一轴转角
#define ZXDOWN_A5E5_POS2    (ZXDOWN_A5E5_POS1+2)  // 转向锁死压力(bar)
#define ZXDOWN_A5E5_POS3    (ZXDOWN_A5E5_POS2+2)  // 目标转向模式+当前转向模式
#define ZXDOWN_A5E5_POS4    (ZXDOWN_A5E5_POS3+2)  // 程序标记位
#define ZXDOWN_A5E5_POS5    (ZXDOWN_A5E5_POS4+1)  // 字节状态位1
#define ZXDOWN_A5E5_POS6    (ZXDOWN_A5E5_POS5+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E5 (ZXDOWN_A5E5_POS6+1)  // 总字节数(9B)

//==TAG-A5E6制动系统==============================================================
#define ZXDOWN_A5E6_ADDR    (ZXDOWN_A5E5_ADDR+SIZE_OF_ZXDOWN_A5E5)  // 起始地址
#define zxdown_buffer_a5e6  (zxdown_buffer+ZXDOWN_A5E6_ADDR)// 起始地址定义
#define ZXDOWN_A5E6_POS1    0                     // 回路一气压
#define ZXDOWN_A5E6_POS2    (ZXDOWN_A5E6_POS1+2)  // 回路二气压
#define ZXDOWN_A5E6_POS3    (ZXDOWN_A5E6_POS2+2)  // 制动压力
#define ZXDOWN_A5E6_POS4    (ZXDOWN_A5E6_POS3+2)  // 电压检测
#define ZXDOWN_A5E6_POS5    (ZXDOWN_A5E6_POS4+2)  // 缓速器油温
#define ZXDOWN_A5E6_POS6    (ZXDOWN_A5E6_POS5+2)  // 缓速力矩百分比
#define ZXDOWN_A5E6_POS7    (ZXDOWN_A5E6_POS6+1)  // 变矩器油温
#define ZXDOWN_A5E6_POS8    (ZXDOWN_A5E6_POS7+2)  // 字节状态位1
#define ZXDOWN_A5E6_POS9    (ZXDOWN_A5E6_POS8+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E6 (ZXDOWN_A5E6_POS9+1)  // 总字节数(15B)

//==TAG-A5E7动力系统==============================================================
#define ZXDOWN_A5E7_ADDR    (ZXDOWN_A5E6_ADDR+SIZE_OF_ZXDOWN_A5E6)  // 起始地址
#define zxdown_buffer_a5e7  (zxdown_buffer+ZXDOWN_A5E7_ADDR)// 起始地址定义
#define ZXDOWN_A5E7_POS1    0                     // 字节状态位
#define SIZE_OF_ZXDOWN_A5E7 (ZXDOWN_A5E7_POS1+1)  // 总字节数(1B)

//==TAG-A5E8单发取力系统==========================================================
#define ZXDOWN_A5E8_ADDR    (ZXDOWN_A5E7_ADDR+SIZE_OF_ZXDOWN_A5E7)  // 起始地址
#define zxdown_buffer_a5e8  (zxdown_buffer+ZXDOWN_A5E8_ADDR)// 起始地址定义
#define ZXDOWN_A5E8_POS1    0                     // 字节状态位1
#define ZXDOWN_A5E8_POS2    (ZXDOWN_A5E8_POS1+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E8 (ZXDOWN_A5E8_POS2+1)  // 总字节数(2B)

//==TAG-A5E9液压系统==============================================================
#define ZXDOWN_A5E9_ADDR    (ZXDOWN_A5E8_ADDR+SIZE_OF_ZXDOWN_A5E8)  // 起始地址
#define zxdown_buffer_a5e9  (zxdown_buffer+ZXDOWN_A5E9_ADDR)// 起始地址定义
#define ZXDOWN_A5E9_POS1    0                     // 液压油温度
#define ZXDOWN_A5E9_POS2    (ZXDOWN_A5E9_POS1+1)  // 液压系统压力
#define SIZE_OF_ZXDOWN_A5E9 (ZXDOWN_A5E9_POS2+2)  // 总字节数(3B)

//==TAG-A5EA双动力驱动系统========================================================
#define ZXDOWN_A5EA_ADDR    (ZXDOWN_A5E9_ADDR+SIZE_OF_ZXDOWN_A5E9)  // 起始地址
#define zxdown_buffer_a5ea  (zxdown_buffer+ZXDOWN_A5EA_ADDR)// 起始地址定义
#define ZXDOWN_A5EA_POS1    0                     // 字节状态位1
#define ZXDOWN_A5EA_POS2    (ZXDOWN_A5EA_POS1+1)  // 字节状态位2
#define ZXDOWN_A5EA_POS3    (ZXDOWN_A5EA_POS2+1)  // 上车发动机水温
#define ZXDOWN_A5EA_POS4    (ZXDOWN_A5EA_POS3+1)  // 上车发动机机油压力
#define ZXDOWN_A5EA_POS5    (ZXDOWN_A5EA_POS4+2)  // 上车发动机转速
#define ZXDOWN_A5EA_POS6    (ZXDOWN_A5EA_POS5+2)  // 上车泵出口压力
#define ZXDOWN_A5EA_POS7    (ZXDOWN_A5EA_POS6+2)  // 马达控制电流
#define ZXDOWN_A5EA_POS8    (ZXDOWN_A5EA_POS7+2)  // 散热器液压驱动泵控制电流
#define SIZE_OF_ZXDOWN_A5EA (ZXDOWN_A5EA_POS8+2)  // 总字节数(13B)

//==TAG-A5EB轮胎胎压==============================================================
#define ZXDOWN_A5EB_ADDR    (ZXDOWN_A5EA_ADDR+SIZE_OF_ZXDOWN_A5EA)  // 起始地址
#define zxdown_buffer_a5eb  (zxdown_buffer+ZXDOWN_A5EB_ADDR)// 起始地址定义
#define ZXDOWN_A5EB_POS1    0         // 胎压#1-18
#define SIZE_OF_ZXDOWN_A5EB (ZXDOWN_A5EB_POS1+18)  // 总字节数(18B)

//==TAG-A5EC左支腿面板============================================================
#define ZXDOWN_A5EC_ADDR    (ZXDOWN_A5EB_ADDR+SIZE_OF_ZXDOWN_A5EB)  // 起始地址
#define zxdown_buffer_a5ec  (zxdown_buffer+ZXDOWN_A5EC_ADDR)// 起始地址定义
#define ZXDOWN_A5EC_POS1    0                     // 字节状态位1
#define ZXDOWN_A5EC_POS2    (ZXDOWN_A5EC_POS1+1)  // 字节状态位2
#define ZXDOWN_A5EC_POS3    (ZXDOWN_A5EC_POS2+1)  // 字节状态位3
#define ZXDOWN_A5EC_POS4    (ZXDOWN_A5EC_POS3+1)  // 字节状态位4
#define ZXDOWN_A5EC_POS5    (ZXDOWN_A5EC_POS4+1)  // 字节状态位5
#define ZXDOWN_A5EC_POS6    (ZXDOWN_A5EC_POS5+1)  // 预留
#define ZXDOWN_A5EC_POS7    (ZXDOWN_A5EC_POS6+1)  // DIR
#define ZXDOWN_A5EC_POS8    (ZXDOWN_A5EC_POS7+1)  // 调平盒心跳
#define SIZE_OF_ZXDOWN_A5EC (ZXDOWN_A5EC_POS8+1)  // 总字节数(8B)

//==TAG-A5ED右支腿面板============================================================
#define ZXDOWN_A5ED_ADDR    (ZXDOWN_A5EC_ADDR+SIZE_OF_ZXDOWN_A5EC)  // 起始地址
#define zxdown_buffer_a5ed  (zxdown_buffer+ZXDOWN_A5ED_ADDR)// 起始地址定义
#define ZXDOWN_A5ED_POS1    0                     // 字节状态位1
#define ZXDOWN_A5ED_POS2    (ZXDOWN_A5ED_POS1+1)  // 字节状态位2
#define ZXDOWN_A5ED_POS3    (ZXDOWN_A5ED_POS2+1)  // 字节状态位3
#define ZXDOWN_A5ED_POS4    (ZXDOWN_A5ED_POS3+1)  // 字节状态位4
#define ZXDOWN_A5ED_POS5    (ZXDOWN_A5ED_POS4+1)  // 字节状态位5
#define ZXDOWN_A5ED_POS6    (ZXDOWN_A5ED_POS5+1)  // 预留
#define ZXDOWN_A5ED_POS7    (ZXDOWN_A5ED_POS6+1)  // DIR
#define ZXDOWN_A5ED_POS8    (ZXDOWN_A5ED_POS7+1)  // 调平盒心跳
#define SIZE_OF_ZXDOWN_A5ED (ZXDOWN_A5ED_POS8+1)  // 总字节数(8B)

//==TAG-A5EE中控台输入信息========================================================
#define ZXDOWN_A5EE_ADDR    (ZXDOWN_A5ED_ADDR+SIZE_OF_ZXDOWN_A5ED)  // 起始地址
#define zxdown_buffer_a5ee  (zxdown_buffer+ZXDOWN_A5EE_ADDR)// 起始地址定义
#define ZXDOWN_A5EE_POS1    0                     // 字节状态位1
#define ZXDOWN_A5EE_POS2    (ZXDOWN_A5EE_POS1+1)  // 字节状态位2
#define ZXDOWN_A5EE_POS3    (ZXDOWN_A5EE_POS2+1)  // 字节状态位3
#define ZXDOWN_A5EE_POS4    (ZXDOWN_A5EE_POS3+1)  // 字节状态位4
#define ZXDOWN_A5EE_POS5    (ZXDOWN_A5EE_POS4+1)  // 字节状态位5
#define ZXDOWN_A5EE_POS6    (ZXDOWN_A5EE_POS5+1)  // 字节状态位6
#define ZXDOWN_A5EE_POS7    (ZXDOWN_A5EE_POS6+1)  // 字节状态位7
#define ZXDOWN_A5EE_POS8    (ZXDOWN_A5EE_POS7+1)  // 后桥转向角度
#define ZXDOWN_A5EE_POS9    (ZXDOWN_A5EE_POS8+1)  // 字节状态位8
#define SIZE_OF_ZXDOWN_A5EE (ZXDOWN_A5EE_POS9+1)  // 总字节数(9B)

//==TAG-A5A3支腿作业信息==========================================================
#define ZXDOWN_A5A3_ADDR    (ZXDOWN_A5EE_ADDR+SIZE_OF_ZXDOWN_A5EE)  // 起始地址
#define zxdown_buffer_a5a3  (zxdown_buffer+ZXDOWN_A5A3_ADDR)// 起始地址定义
#define ZXDOWN_A5A3_POS1    0                     // 水平支腿长度
#define ZXDOWN_A5A3_POS2    (ZXDOWN_A5A3_POS1+8)  // 支腿压力
#define ZXDOWN_A5A3_POS3    (ZXDOWN_A5A3_POS2+8)  // 摆缸长度
#define SIZE_OF_ZXDOWN_A5A3 (ZXDOWN_A5A3_POS3+8)  // 总字节数(24B)

//==TAG-A5A4辅助支腿作业信息=======================================================
#define ZXDOWN_A5A4_ADDR    (ZXDOWN_A5A3_ADDR+SIZE_OF_ZXDOWN_A5A3)  // 起始地址
#define zxdown_buffer_a5a4  (zxdown_buffer+ZXDOWN_A5A4_ADDR)// 起始地址定义
#define ZXDOWN_A5A4_POS1    0                     // 辅助支腿状态
#define ZXDOWN_A5A4_POS2    (ZXDOWN_A5A4_POS1+1)  // 左前辅助支腿压力
#define ZXDOWN_A5A4_POS3    (ZXDOWN_A5A4_POS2+2)  // 右前辅助支腿压力
#define SIZE_OF_ZXDOWN_A5A4 (ZXDOWN_A5A4_POS3+2)  // 总字节数(5B)

// 下车底盘数据缓存大小
#define SIZE_OF_ZXDOWN_BUFFER   (ZXDOWN_A5A4_ADDR+SIZE_OF_ZXDOWN_A5A4)

/******************************************************************************
* Macros(底盘发动机): 起重机底盘发动机数据序号和地址定义
******************************************************************************/
//==TAG-A502环境信息(发动机的数据)================================================
#define ZXENGINE_A502_ADDR      (0)  // 起始地址
#define zxengine_buffer_a502    (zxengine_buffer+ZXENGINE_A502_ADDR)  // 起始地址定义
#define ZXENGINE_A502_POS1_ADDR 0                            // 大气压力
#define ZXENGINE_A502_POS2_ADDR (ZXENGINE_A502_POS1_ADDR+1)  // 环境温度
#define SIZE_OF_ZXENGINE_A502   (ZXENGINE_A502_POS2_ADDR+2)  // 总字节数(3B)

//==TAG-A5EF发动机运行参数(国四、国五、国六)======================================
#define ZXENGINE_A5EF_ADDR    (ZXENGINE_A502_ADDR+SIZE_OF_ZXENGINE_A502)  // 起始地址
#define zxengine_buffer_a5ef  (zxengine_buffer+ZXENGINE_A5EF_ADDR)  // 起始地址定义
#define ZXENGINE_A5EF_POS1    0                       // 发动机转速
#define ZXENGINE_A5EF_POS2    (ZXENGINE_A5EF_POS1+2)  // 实际扭矩百分比
#define ZXENGINE_A5EF_POS3    (ZXENGINE_A5EF_POS2+1)  // 摩擦扭矩百分比
#define ZXENGINE_A5EF_POS4    (ZXENGINE_A5EF_POS3+1)  // 进气歧管压力
#define ZXENGINE_A5EF_POS5    (ZXENGINE_A5EF_POS4+1)  // 进气歧管温度
#define ZXENGINE_A5EF_POS6    (ZXENGINE_A5EF_POS5+1)  // 冷却液温度
#define ZXENGINE_A5EF_POS7    (ZXENGINE_A5EF_POS6+1)  // 机油温度
#define ZXENGINE_A5EF_POS8    (ZXENGINE_A5EF_POS7+2)  // 机油液位
#define ZXENGINE_A5EF_POS9    (ZXENGINE_A5EF_POS8+1)  // 机油压力
#define ZXENGINE_A5EF_POS10   (ZXENGINE_A5EF_POS9+1)  // 发动机总运行时间
#define ZXENGINE_A5EF_POS11   (ZXENGINE_A5EF_POS10+4)  // 油门踏板百分比
#define ZXENGINE_A5EF_POS12   (ZXENGINE_A5EF_POS11+1)  // 车速
#define ZXENGINE_A5EF_POS13   (ZXENGINE_A5EF_POS12+2)  // 字节状态位1
#define ZXENGINE_A5EF_POS14   (ZXENGINE_A5EF_POS13+1)  // 字节状态位2
#define ZXENGINE_A5EF_POS15   (ZXENGINE_A5EF_POS14+1)  // 巡航设定速度
#define ZXENGINE_A5EF_POS16   (ZXENGINE_A5EF_POS15+1)  // 发动机燃油消耗率
#define ZXENGINE_A5EF_POS17   (ZXENGINE_A5EF_POS16+2)  // 发动机平均燃油消耗率
#define ZXENGINE_A5EF_POS18   (ZXENGINE_A5EF_POS17+2)  // 燃油总油耗量
#define ZXENGINE_A5EF_POS19   (ZXENGINE_A5EF_POS18+4)  // 总行驶里程
#define ZXENGINE_A5EF_POS20   (ZXENGINE_A5EF_POS19+4)  // 燃油液位1
#define ZXENGINE_A5EF_POS21   (ZXENGINE_A5EF_POS20+1)  // 燃油液位2
#define ZXENGINE_A5EF_POS22   (ZXENGINE_A5EF_POS21+1)  // 字节状态位3
#define ZXENGINE_A5EF_POS23   (ZXENGINE_A5EF_POS22+1)  // 字节状态位4
#define ZXENGINE_A5EF_POS24   (ZXENGINE_A5EF_POS23+1)  // 进气流量
#define SIZE_OF_ZXENGINE_A5EF (ZXENGINE_A5EF_POS24+2)  // 总字节数(39B)

//==TAG-A5F0行驶油耗==============================================================
#define ZXENGINE_A5F0_ADDR    (ZXENGINE_A5EF_ADDR+SIZE_OF_ZXENGINE_A5EF) // 起始地址
#define zxengine_buffer_a5f0  (zxengine_buffer+ZXENGINE_A5F0_ADDR)// 起始地址定义
#define ZXENGINE_A5F0_POS1    0                       // 行驶总运行时间
#define ZXENGINE_A5F0_POS2    (ZXENGINE_A5F0_POS1+4)  // 行驶燃油总油耗量
#define ZXENGINE_A5F0_POS3    (ZXENGINE_A5F0_POS2+4)  // 百公里油耗
#define SIZE_OF_ZXENGINE_A5F0 (ZXENGINE_A5F0_POS3+4)  // 总字节数(12B)

//==TAG-A5F1 SCR参数（国五）======================================================
#define ZXENGINE_A5F1_ADDR    (ZXENGINE_A5F0_ADDR+SIZE_OF_ZXENGINE_A5F0)                             // 起始地址
#define zxengine_buffer_a5f1  (zxengine_buffer+ZXENGINE_A5F1_ADDR)// 起始地址定义
#define ZXENGINE_A5F1_POS1    0                       // 尿素喷射状态
#define ZXENGINE_A5F1_POS2    (ZXENGINE_A5F1_POS1+1)  // T15_DCU
#define ZXENGINE_A5F1_POS3    (ZXENGINE_A5F1_POS2+1)  // 尿素泵压力
#define ZXENGINE_A5F1_POS4    (ZXENGINE_A5F1_POS3+2)  // 尿素箱液位
#define ZXENGINE_A5F1_POS5    (ZXENGINE_A5F1_POS4+1)  // 尿素箱温度
#define ZXENGINE_A5F1_POS6    (ZXENGINE_A5F1_POS5+1)  // 尿素喷射量
#define ZXENGINE_A5F1_POS7    (ZXENGINE_A5F1_POS6+2)  // SCR上游NOx浓度
#define ZXENGINE_A5F1_POS8    (ZXENGINE_A5F1_POS7+2)  // SCR下游NOx浓度
#define ZXENGINE_A5F1_POS9    (ZXENGINE_A5F1_POS8+2)  // SCR上游排气温度(T6温度)
#define ZXENGINE_A5F1_POS10   (ZXENGINE_A5F1_POS9+2)  // SCR下游排气温度(T7温度)
#define ZXENGINE_A5F1_POS11   (ZXENGINE_A5F1_POS10+2)  // 尿素浓度(SPN 3516)
#define ZXENGINE_A5F1_POS12   (ZXENGINE_A5F1_POS11+1)  // 累计尿素消耗量
#define ZXENGINE_A5F1_POS13   (ZXENGINE_A5F1_POS12+4)  // 尿素品质传感器温度(SPN 3515)
#define ZXENGINE_A5F1_POS14   (ZXENGINE_A5F1_POS13+1)  // 品质温度传感器FMI (SPN 3519)
#define ZXENGINE_A5F1_POS15   (ZXENGINE_A5F1_POS14+1)  // 品质传感器FMI (SPN3520)
#define ZXENGINE_A5F1_POS16   (ZXENGINE_A5F1_POS15+1)  // 催化剂试剂类型(SPN3521)
#define ZXENGINE_A5F1_POS17   (ZXENGINE_A5F1_POS16+1)  // 尿素箱液位传感器失效模式FMI
#define ZXENGINE_A5F1_POS18   (ZXENGINE_A5F1_POS17+1)  // 尿素箱温度传感器失效模式FMI
#define ZXENGINE_A5F1_POS19   (ZXENGINE_A5F1_POS18+1)  // Nox传感器露点状态
#define SIZE_OF_ZXENGINE_A5F1 (ZXENGINE_A5F1_POS19+1)  // 总字节数(28B)

//==TAG-A5F2 DPF参数(国六）=======================================================
#define ZXENGINE_A5F2_ADDR    (ZXENGINE_A5F1_ADDR+SIZE_OF_ZXENGINE_A5F1)                             // 起始地址
#define zxengine_buffer_a5f2  (zxengine_buffer+ZXENGINE_A5F2_ADDR)// 起始地址定义
#define ZXENGINE_A5F2_POS1    0                       // DOC上游排气温度
#define ZXENGINE_A5F2_POS2    (ZXENGINE_A5F2_POS1+2)  // DPF上游排气温度
#define ZXENGINE_A5F2_POS3    (ZXENGINE_A5F2_POS2+2)  // DPF碳载量负荷率
#define ZXENGINE_A5F2_POS4    (ZXENGINE_A5F2_POS3+1)  // DPF压差
#define ZXENGINE_A5F2_POS5    (ZXENGINE_A5F2_POS4+2)  // 字节状态位1
#define ZXENGINE_A5F2_POS6    (ZXENGINE_A5F2_POS5+1)  // 字节状态位2
#define ZXENGINE_A5F2_POS7    (ZXENGINE_A5F2_POS6+1)  // 字节状态位3
#define ZXENGINE_A5F2_POS8    (ZXENGINE_A5F2_POS7+1)  // 字节状态位4
#define SIZE_OF_ZXENGINE_A5F2 (ZXENGINE_A5F2_POS8+1)  // 总字节数(11B)

// 下车发动机数据缓存大小
#define SIZE_OF_ZXENGINE_BUFFER  (ZXENGINE_A5F2_ADDR+SIZE_OF_ZXENGINE_A5F2)

/******************************************************************************
* Macros(统计类信息): 数据序号和地址定义
******************************************************************************/
//==TAG-A5C5 动作频次统计1========================================================
#define ZXSTATISTICS_A5C5_ADDR    0  // 起始地址
#define zxstatistics_buffer_a5c5  (zxstatistics_buffer+ZXSTATISTICS_A5C5_ADDR)
#define ZXSTATISTICS_A5C5_POS1    0                           // 二节伸
#define ZXSTATISTICS_A5C5_POS2    (ZXSTATISTICS_A5C5_POS1+8)  // 二节缩
#define ZXSTATISTICS_A5C5_POS3    (ZXSTATISTICS_A5C5_POS2+8)  // 多节伸
#define ZXSTATISTICS_A5C5_POS4    (ZXSTATISTICS_A5C5_POS3+8)  // 多节缩
#define ZXSTATISTICS_A5C5_POS5    (ZXSTATISTICS_A5C5_POS4+8)  // 变幅起
#define ZXSTATISTICS_A5C5_POS6    (ZXSTATISTICS_A5C5_POS5+8)  // 变幅下落
#define ZXSTATISTICS_A5C5_POS7    (ZXSTATISTICS_A5C5_POS6+8)  // 主卷起升
#define ZXSTATISTICS_A5C5_POS8    (ZXSTATISTICS_A5C5_POS7+8)  // 主卷下落
#define ZXSTATISTICS_A5C5_POS9    (ZXSTATISTICS_A5C5_POS8+8)  // 副卷起升
#define ZXSTATISTICS_A5C5_POS10   (ZXSTATISTICS_A5C5_POS9+8)  // 副卷下落
#define ZXSTATISTICS_A5C5_POS11   (ZXSTATISTICS_A5C5_POS10+8)  // 左回转
#define ZXSTATISTICS_A5C5_POS12   (ZXSTATISTICS_A5C5_POS11+8)  // 右回转
#define SIZE_OF_ZXSTATISTICS_A5C5 (ZXSTATISTICS_A5C5_POS12+8)  // 总字节数(96B)

//==TAG-A5C6 动作频次统计2========================================================
#define ZXSTATISTICS_A5C6_ADDR    (ZXSTATISTICS_A5C5_ADDR+SIZE_OF_ZXSTATISTICS_A5C5)  // 起始地址
#define zxstatistics_buffer_a5c6  (zxstatistics_buffer+ZXSTATISTICS_A5C6_ADDR)
#define ZXSTATISTICS_A5C6_POS1    0                           // 空缸伸
#define ZXSTATISTICS_A5C6_POS2    (ZXSTATISTICS_A5C6_POS1+8)  // 空缸缩
#define ZXSTATISTICS_A5C6_POS3    (ZXSTATISTICS_A5C6_POS2+8)  // 带臂伸
#define ZXSTATISTICS_A5C6_POS4    (ZXSTATISTICS_A5C6_POS3+8)  // 带臂缩
#define ZXSTATISTICS_A5C6_POS5    (ZXSTATISTICS_A5C6_POS4+8)  // 变幅起
#define ZXSTATISTICS_A5C6_POS6    (ZXSTATISTICS_A5C6_POS5+8)  // 变幅下落
#define ZXSTATISTICS_A5C6_POS7    (ZXSTATISTICS_A5C6_POS6+8)  // 主卷起升
#define ZXSTATISTICS_A5C6_POS8    (ZXSTATISTICS_A5C6_POS7+8)  // 主卷下落
#define ZXSTATISTICS_A5C6_POS9    (ZXSTATISTICS_A5C6_POS8+8)  // 副卷起升
#define ZXSTATISTICS_A5C6_POS10   (ZXSTATISTICS_A5C6_POS9+8)  // 副卷下落
#define ZXSTATISTICS_A5C6_POS11   (ZXSTATISTICS_A5C6_POS10+8)  // 左回转
#define ZXSTATISTICS_A5C6_POS12   (ZXSTATISTICS_A5C6_POS11+8)  // 右回转
#define SIZE_OF_ZXSTATISTICS_A5C6 (ZXSTATISTICS_A5C6_POS12+8)  // 总字节数(96B)

//==TAG-A5C7 安全统计=============================================================
#define ZXSTATISTICS_A5C7_ADDR    (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)  // 起始地址
#define zxstatistics_buffer_a5c7  (zxstatistics_buffer+ZXSTATISTICS_A5C7_ADDR)
#define ZXSTATISTICS_A5C7_POS1    0                           // 超载
#define ZXSTATISTICS_A5C7_POS2    (ZXSTATISTICS_A5C7_POS1+8)  // 三圈
#define ZXSTATISTICS_A5C7_POS3    (ZXSTATISTICS_A5C7_POS2+8)  // 高限
#define ZXSTATISTICS_A5C7_POS4    (ZXSTATISTICS_A5C7_POS3+8)  // 总强制
#define ZXSTATISTICS_A5C7_POS5    (ZXSTATISTICS_A5C7_POS4+8)  // 拆装开关
#define ZXSTATISTICS_A5C7_POS6    (ZXSTATISTICS_A5C7_POS5+8)  // 变幅起强制
#define ZXSTATISTICS_A5C7_POS7    (ZXSTATISTICS_A5C7_POS6+8)  // 高限强制
#define ZXSTATISTICS_A5C7_POS8    (ZXSTATISTICS_A5C7_POS7+8)  // 三圈强制
#define ZXSTATISTICS_A5C7_POS9    (ZXSTATISTICS_A5C7_POS8+8)  // 风速超限
#define SIZE_OF_ZXSTATISTICS_A5C7 (ZXSTATISTICS_A5C7_POS9+8)  // 总字节数(72B)

//==TAG-301E 休眠时间统计数据======================================================
#define ZXSTATISTICS_301E_ADDR       (ZXSTATISTICS_A5C6_ADDR+SIZE_OF_ZXSTATISTICS_A5C6)  // 起始地址
#define zxstatistics_buffer_301e     (zxstatistics_buffer+ZXSTATISTICS_301E_ADDR)
#define ZXSTATISTICS_301E_POS1_ADDR  0                                // 总休眠时间
#define ZXSTATISTICS_301E_POS2_ADDR  (ZXSTATISTICS_301E_POS1_ADDR+4)  // 本次休眠时间
#define SIZE_OF_ZXSTATISTICS_301E    (ZXSTATISTICS_301E_POS2_ADDR+4)  // 总字节数(8B)

// 统计数据缓存大小
#define SIZE_OF_ZXSTATISTICS_BUFFER  (ZXSTATISTICS_301E_ADDR+SIZE_OF_ZXSTATISTICS_301E)

/******************************************************************************
* Macros(上车、下车版本信息): 数据序号和地址定义
******************************************************************************/
//==TAG-A505 上车系统版本=========================================================
#define ZXVERSION_A505_ADDR    0  // 起始地址
#define zxversion_buffer_a505  (zxversion_buffer+ZXVERSION_A505_ADDR)
#define ZXVERSION_A505_POS1    0                        // 力矩限制器
#define ZXVERSION_A505_POS2    (ZXVERSION_A505_POS1+3)  // 显示器1
#define ZXVERSION_A505_POS3    (ZXVERSION_A505_POS2+3)  // 显示器底层版本
#define ZXVERSION_A505_POS4    (ZXVERSION_A505_POS3+3)  // GPS终端
#define ZXVERSION_A505_POS5    (ZXVERSION_A505_POS4+3)  // 控制器
#define ZXVERSION_A505_POS6    (ZXVERSION_A505_POS5+3)  // 显示器2
#define ZXVERSION_A505_POS7    (ZXVERSION_A505_POS6+3)  // 显示器2底层
#define SIZE_OF_ZXVERSION_A505 (ZXVERSION_A505_POS7+3)  // 总字节数(18B)

//==TAG-A506 下车系统版本=========================================================
#define ZXVERSION_A506_ADDR   (ZXVERSION_A505_ADDR+SIZE_OF_ZXVERSION_A505)  // 起始地址
#define zxversion_buffer_a506 (zxversion_buffer+ZXVERSION_A506_ADDR)
#define ZXVERSION_A506_POS1   0                          // 显示器应用层
#define ZXVERSION_A506_POS2   (ZXVERSION_A506_POS1+3)    // 显示器底层
#define ZXVERSION_A506_POS3   (ZXVERSION_A506_POS2+3)    // P1应用层
#define ZXVERSION_A506_POS4   (ZXVERSION_A506_POS3+3)    // P1底层
#define ZXVERSION_A506_POS5   (ZXVERSION_A506_POS4+3)    // P2应用层
#define ZXVERSION_A506_POS6   (ZXVERSION_A506_POS5+3)    // P2底层
#define ZXVERSION_A506_POS7   (ZXVERSION_A506_POS6+3)    // P3应用层
#define ZXVERSION_A506_POS8   (ZXVERSION_A506_POS7+3)    // P3底层
#define ZXVERSION_A506_POS9   (ZXVERSION_A506_POS8+3)    // P4应用层
#define ZXVERSION_A506_POS10   (ZXVERSION_A506_POS9+3)   // P4底层
#define ZXVERSION_A506_POS11   (ZXVERSION_A506_POS10+3)  // P5应用层
#define ZXVERSION_A506_POS12   (ZXVERSION_A506_POS11+3)  // P5底层
#define ZXVERSION_A506_POS13   (ZXVERSION_A506_POS12+3)  // P6应用层
#define ZXVERSION_A506_POS14   (ZXVERSION_A506_POS13+3)  // P6底层
#define ZXVERSION_A506_POS15   (ZXVERSION_A506_POS14+3)  // P7应用层
#define ZXVERSION_A506_POS16   (ZXVERSION_A506_POS15+3)  // P7底层
#define ZXVERSION_A506_POS17   (ZXVERSION_A506_POS16+3)  // P8应用层
#define ZXVERSION_A506_POS18   (ZXVERSION_A506_POS17+3)  // P8底层
#define SIZE_OF_ZXVERSION_A506 (ZXVERSION_A506_POS18+3)  // 总字节数(54B)

// 版本信息数据缓存大小
#define SIZE_OF_ZXVERSION_BUFFER   (ZXVERSION_A506_ADDR+SIZE_OF_ZXVERSION_A506)

/******************************************************************************
 * Data Types
 ******************************************************************************/
// TLV有效标志位
extern bittype2 zxinfo_tlv_flag;
#define tlv_a501_valid_flag    zxinfo_tlv_flag.w.bit0  //==TAG-A501通用状态字2(重型专用)
#define tlv_a504_valid_flag    zxinfo_tlv_flag.w.bit1  //==TAG-A504采集协议信息
#define tlv_a5ff_valid_flag    zxinfo_tlv_flag.w.bit2  //==TAG-A5FF终端采集信息
//#define X    zxinfo_tlv_flag.w.bit3
//#define X    zxinfo_tlv_flag.w.bit4
//#define X    zxinfo_tlv_flag.w.bit5
//#define X    zxinfo_tlv_flag.w.bit6
//#define X    zxinfo_tlv_flag.w.bit7
//#define X    zxinfo_tlv_flag.w.bit8
//#define X    zxinfo_tlv_flag.w.bit9
//#define X    zxinfo_tlv_flag.w.bit10
//#define X    zxinfo_tlv_flag.w.bit11
//#define X    zxinfo_tlv_flag.w.bit12
//#define X    zxinfo_tlv_flag.w.bit13
//#define X    zxinfo_tlv_flag.w.bit14
//#define X    zxinfo_tlv_flag.w.bit15


// TLV有效标志位
extern bittype2 zxup_tlv_flag1;
#define tlv_a5a0_valid_flag    zxup_tlv_flag1.w.bit0  //==TAG-A5A0力限器工况信息(单缸)
#define tlv_a5a1_valid_flag    zxup_tlv_flag1.w.bit1  //==TAG-A5A1超起工况信息
#define tlv_a5a2_valid_flag    zxup_tlv_flag1.w.bit2  //==TAG-A5A2塔臂工况信息
//#define X    zxup_tlv_flag1.w.bit3
//#define X    zxup_tlv_flag1.w.bit4
#define tlv_a5a5_valid_flag    zxup_tlv_flag1.w.bit5  //==TAG-A5A5上车发动机信息
#define tlv_a5a6_valid_flag    zxup_tlv_flag1.w.bit6  //==TAG-A5A6手柄信息
#define tlv_a5a7_valid_flag    zxup_tlv_flag1.w.bit7  //==TAG-A5A7显示器1信息
#define tlv_a5a8_valid_flag    zxup_tlv_flag1.w.bit8  //==TAG-A5A8显示器2信息
#define tlv_a5a9_valid_flag    zxup_tlv_flag1.w.bit9  //==TAG-A5A9总线面板信息
#define tlv_a5aa_valid_flag    zxup_tlv_flag1.w.bit10 //==TAG-A5AA无线操控信息
#define tlv_a5ab_valid_flag    zxup_tlv_flag1.w.bit11 //==TAG-A5AB网络拓扑信息
#define tlv_a5ac_valid_flag    zxup_tlv_flag1.w.bit12 //==TAG-A5AC伸缩逻辑信息
#define tlv_a5ad_valid_flag    zxup_tlv_flag1.w.bit13 //==TAG-A5AD变幅逻辑信息
#define tlv_a5ae_valid_flag    zxup_tlv_flag1.w.bit14 //==TAG-A5AE回转逻辑信息
#define tlv_a5af_valid_flag    zxup_tlv_flag1.w.bit15 //==TAG-A5AF主卷扬逻辑信息

// TLV有效标志位
extern bittype2 zxup_tlv_flag2;
#define tlv_a5b0_valid_flag    zxup_tlv_flag2.w.bit0  //==TAG-A5B0副卷扬逻辑信息
#define tlv_a5b1_valid_flag    zxup_tlv_flag2.w.bit1  //==TAG-A5B1超起逻辑信息
#define tlv_a5b2_valid_flag    zxup_tlv_flag2.w.bit2  //==TAG-A5B2塔臂逻辑信息
#define tlv_a5b3_valid_flag    zxup_tlv_flag2.w.bit3  //==TAG-A5B3液压油温度
#define tlv_a5b4_valid_flag    zxup_tlv_flag2.w.bit4  //==TAG-A5B4主泵信息
#define tlv_a5b5_valid_flag    zxup_tlv_flag2.w.bit5  //==TAG-A5B5主阀1 XHVME4400P1
#define tlv_a5b6_valid_flag    zxup_tlv_flag2.w.bit6  //==TAG-A5B6主阀3 大吨位
#define tlv_a5b7_valid_flag    zxup_tlv_flag2.w.bit7  //==TAG-A5B7主泵信息
#define tlv_a5b8_valid_flag    zxup_tlv_flag2.w.bit8  //==TAG-A5B8塔臂逻辑信息
#define tlv_a5b9_valid_flag    zxup_tlv_flag2.w.bit9  //==TAG-A5B9缸臂销控制阀
#define tlv_a5ba_valid_flag    zxup_tlv_flag2.w.bit10 //==TAG-A5BA变幅平衡阀
#define tlv_a5bb_valid_flag    zxup_tlv_flag2.w.bit11 //==TAG-A5BB主卷泵
#define tlv_a5bc_valid_flag    zxup_tlv_flag2.w.bit12 //==TAG-A5BC主卷马达
#define tlv_a5bd_valid_flag    zxup_tlv_flag2.w.bit13 //==TAG-A5BD塔卷泵
#define tlv_a5be_valid_flag    zxup_tlv_flag2.w.bit14 //==TAG-A5BE塔卷马达
#define tlv_a5bf_valid_flag    zxup_tlv_flag2.w.bit15 //==TAG-A5BF回转泵

// TLV有效标志位
extern bittype2 zxup_tlv_flag3;
#define tlv_a5c0_valid_flag    zxup_tlv_flag3.w.bit0  //==TAG-A5C0回转制动阀
#define tlv_a5c1_valid_flag    zxup_tlv_flag3.w.bit1  //==TAG-A5C1辅助阀1
#define tlv_a5c2_valid_flag    zxup_tlv_flag3.w.bit2  //==TAG-A5C2辅助阀2(超大吨位)
#define tlv_a5c3_valid_flag    zxup_tlv_flag3.w.bit3  //==TAG-A5C3左超起阀组
#define tlv_a5c4_valid_flag    zxup_tlv_flag3.w.bit4  //==TAG-A5C4右超起阀组
//#define X    zxup_tlv_flag3.w.bit5
//#define X    zxup_tlv_flag3.w.bit6
//#define X    zxup_tlv_flag3.w.bit7
#define tlv_a5c8_valid_flag    zxup_tlv_flag3.w.bit8  //==TAG-A5C8作业油耗信息
#define tlv_a5c9_valid_flag    zxup_tlv_flag3.w.bit9  //==TAG-A5C9 ECU响应锁车CAN帧
//#define X    zxup_tlv_flag3.w.bit10
//#define X    zxup_tlv_flag3.w.bit11
//#define X    zxup_tlv_flag3.w.bit12
//#define X    zxup_tlv_flag3.w.bit13
//#define X    zxup_tlv_flag3.w.bit14
//#define X    zxup_tlv_flag3.w.bit15

// TLV有效标志位
extern bittype2 zxdown_tlv_flag1;
#define tlv_a5e0_valid_flag    zxdown_tlv_flag1.w.bit0  //==TAG-A5E0节点状态
#define tlv_a5e1_valid_flag    zxdown_tlv_flag1.w.bit1  //==TAG-A5E1传动系统
#define tlv_a5e2_valid_flag    zxdown_tlv_flag1.w.bit2  //==TAG-A5E2支腿相关信息
#define tlv_a5e3_valid_flag    zxdown_tlv_flag1.w.bit3  //==TAG-A5E3悬挂系统
#define tlv_a5e4_valid_flag    zxdown_tlv_flag1.w.bit4  //==TAG-A5E4转向系统（全地面）
#define tlv_a5e5_valid_flag    zxdown_tlv_flag1.w.bit5  //==TAG-A5E5转向系统（汽车）
#define tlv_a5e6_valid_flag    zxdown_tlv_flag1.w.bit6  //==TAG-A5E6制动系统
#define tlv_a5e7_valid_flag    zxdown_tlv_flag1.w.bit7  //==TAG-A5E7动力系统
#define tlv_a5e8_valid_flag    zxdown_tlv_flag1.w.bit8  //==TAG-A5E8单发取力系统
#define tlv_a5e9_valid_flag    zxdown_tlv_flag1.w.bit9  //==TAG-A5E9液压系统
#define tlv_a5ea_valid_flag    zxdown_tlv_flag1.w.bit10 //==TAG-A5EA双动力驱动系统
#define tlv_a5eb_valid_flag    zxdown_tlv_flag1.w.bit11 //==TAG-A5EB轮胎胎压
#define tlv_a5ec_valid_flag    zxdown_tlv_flag1.w.bit12 //==TAG-A5EC左支腿面板
#define tlv_a5ed_valid_flag    zxdown_tlv_flag1.w.bit13 //==TAG-A5ED右支腿面板
#define tlv_a5ee_valid_flag    zxdown_tlv_flag1.w.bit14 //==TAG-A5EE中控台输入信息
//#define x    zxdown_tlv_flag1.w.bit15

// TLV有效标志位
extern bittype2 zxdown_tlv_flag2;
#define tlv_a5a3_valid_flag    zxdown_tlv_flag2.w.bit0  //==TAG-A5A3支腿作业信息
#define tlv_a5a4_valid_flag    zxdown_tlv_flag2.w.bit1  //==TAG-A5A4辅助支腿作业信息
//#define X    zxdown_tlv_flag2.w.bit2
//#define X    zxdown_tlv_flag2.w.bit3
//#define X    zxdown_tlv_flag2.w.bit4
//#define X    zxdown_tlv_flag2.w.bit5
//#define X    zxdown_tlv_flag2.w.bit6
//#define X    zxdown_tlv_flag2.w.bit7
//#define X    zxdown_tlv_flag2.w.bit8
//#define X    zxdown_tlv_flag2.w.bit9
//#define X    zxdown_tlv_flag2.w.bit10
//#define X    zxdown_tlv_flag2.w.bit11
//#define X    zxdown_tlv_flag2.w.bit12
//#define X    zxdown_tlv_flag2.w.bit13
//#define X    zxdown_tlv_flag2.w.bit14
//#define X    zxdown_tlv_flag2.w.bit15

// TLV有效标志位
extern bittype2 zxengine_tlv_flag;
#define tlv_a502_valid_flag    zxengine_tlv_flag.w.bit0  //==TAG-A502环境信息(发动机的数据)
#define tlv_a5ef_valid_flag    zxengine_tlv_flag.w.bit1  //==TAG-A5EF发动机运行参数(国四、国五、国六)
#define tlv_a5f0_valid_flag    zxengine_tlv_flag.w.bit2  //==TAG-A5F0行驶油耗
#define tlv_a5f1_valid_flag    zxengine_tlv_flag.w.bit3  //==TAG-A5F1 SCR参数（国五）
#define tlv_a5f2_valid_flag    zxengine_tlv_flag.w.bit4  //==TAG-A5F2 DPF参数(国六）
//#define X    zxengine_tlv_flag.w.bit5
//#define X    zxengine_tlv_flag.w.bit6
//#define X    zxengine_tlv_flag.w.bit7
//#define X    zxengine_tlv_flag.w.bit8
//#define X    zxengine_tlv_flag.w.bit9
//#define X    zxengine_tlv_flag.w.bit10
//#define X    zxengine_tlv_flag.w.bit11
//#define X    zxengine_tlv_flag.w.bit12
//#define X    zxengine_tlv_flag.w.bit13
//#define X    zxengine_tlv_flag.w.bit14
//#define X    zxengine_tlv_flag.w.bit15

// TLV有效标志位
extern bittype2 zxversion_tlv_flag;
#define tlv_a505_valid_flag    zxversion_tlv_flag.w.bit0  //==TAG-A505 上车系统版本
#define tlv_a506_valid_flag    zxversion_tlv_flag.w.bit1  //==TAG-A506 下车系统版本
//#define X    zxversion_tlv_flag.w.bit2
//#define X    zxversion_tlv_flag.w.bit3
//#define X    zxversion_tlv_flag.w.bit4
//#define X    zxversion_tlv_flag.w.bit5
//#define X    zxversion_tlv_flag.w.bit6
//#define X    zxversion_tlv_flag.w.bit7
//#define X    zxversion_tlv_flag.w.bit8
//#define X    zxversion_tlv_flag.w.bit9
//#define X    zxversion_tlv_flag.w.bit10
//#define X    zxversion_tlv_flag.w.bit11
//#define X    zxversion_tlv_flag.w.bit12
//#define X    zxversion_tlv_flag.w.bit13
//#define X    zxversion_tlv_flag.w.bit14
//#define X    zxversion_tlv_flag.w.bit15

// TLV有效标志位
extern bittype2 zxstatistics_tlv_flag;
#define tlv_a5c5_valid_flag    zxstatistics_tlv_flag.w.bit0  //==TAG-A5C5 动作频次统计1
#define tlv_a5c6_valid_flag    zxstatistics_tlv_flag.w.bit1  //==TAG-A5C6 动作频次统计2
#define tlv_a5c7_valid_flag    zxstatistics_tlv_flag.w.bit2  //==TAG-A5C7 安全统计
#define tlv_301e_valid_flag    zxstatistics_tlv_flag.w.bit3  //==TAG-301E 休眠时间统计数据
//#define X    zxstatistics_tlv_flag.w.bit4
//#define X    zxstatistics_tlv_flag.w.bit5
//#define X    zxstatistics_tlv_flag.w.bit6
//#define X    zxstatistics_tlv_flag.w.bit7
//#define X    zxstatistics_tlv_flag.w.bit8
//#define X    zxstatistics_tlv_flag.w.bit9
//#define X    zxstatistics_tlv_flag.w.bit10
//#define X    zxstatistics_tlv_flag.w.bit11
//#define X    zxstatistics_tlv_flag.w.bit12
//#define X    zxstatistics_tlv_flag.w.bit13
//#define X    zxstatistics_tlv_flag.w.bit14
//#define X    zxstatistics_tlv_flag.w.bit15

extern uint8_t zxinfo_buffer[SIZE_OF_ZXINFO_BUFFER]; /// 基本信息
extern uint8_t zxup_buffer[SIZE_OF_ZXUP_BUFFER]; /// 上车数据缓存
extern uint8_t zxdown_buffer[SIZE_OF_ZXDOWN_BUFFER]; /// 下车底盘数据缓存
extern uint8_t zxengine_buffer[SIZE_OF_ZXENGINE_BUFFER]; /// 下车发动机数据缓存
extern uint8_t zxstatistics_buffer[SIZE_OF_ZXSTATISTICS_BUFFER]; ///统计数据缓存
extern uint8_t zxversion_buffer[SIZE_OF_ZXVERSION_BUFFER]; /// 版本信息缓存

/******************************************************************************
 * 作业时间和频次统计
 ******************************************************************************/
 #define ZXSTS_STEP_SP   1800  // 梯级:3分钟,基于100ms
 #define ZXSTS_DEBOUNCE_TIME_SP  30 // 3秒,基于100ms

 // BOOL定义
enum
{
  ZXSTS_FALSE = 0x00,
  ZXSTS_TRUE = 0x01
};

// 频次统计类型
enum
{
  ZXSTS_TYPE_EJEX = 0,  // 二节伸
  ZXSTS_TYPE_EJS,  // 二节缩
  ZXSTS_TYPE_DJEX,  // 多节伸
  ZXSTS_TYPE_DJS,  // 多节缩

  ZXSTS_TYPE_KGEX,  // 空缸伸
  ZXSTS_TYPE_KGS,  // 空缸缩
  ZXSTS_TYPE_DBEX,  // 带臂伸
  ZXSTS_TYPE_DBS,  // 带臂缩

  ZXSTS_TYPE_BFUP,  // 变幅起
  ZXSTS_TYPE_BFDW,  // 变幅落
  ZXSTS_TYPE_ZJUP,  // 主卷起升
  ZXSTS_TYPE_ZJDW,  // 主卷下落
  ZXSTS_TYPE_FJUP,  // 副卷起升
  ZXSTS_TYPE_FJDW,  // 副卷下落
  ZXSTS_TYPE_ZHR,  // 左回转
  ZXSTS_TYPE_YHR,  // 右回转
  
  ZXSTS_TYPE_LMI,  // 超载 16
  ZXSTS_TYPE_ZJSQ,  // 主卷三圈保护
  ZXSTS_TYPE_FJSQ,  // 副卷三圈 18
  ZXSTS_TYPE_ZBGX,  // 主臂高限触发
  ZXSTS_TYPE_FBGX,  // 副臂高限(备用) 20
  ZXSTS_TYPE_ZQZ,  // 总强制 21
  ZXSTS_TYPE_CZKG,  // 拆装开关 22
  ZXSTS_TYPE_BFQQZ,  // 变幅起强制 23
  ZXSTS_TYPE_GXQZ,  // 高限强制 24
  ZXSTS_TYPE_SQQZ,  // 三圈强制 25
  ZXSTS_TYPE_FSCX,  // 风速超限
  NUMBER_OF_ZXSTS_TYPES
};

// 频次统计结构体
typedef struct
{
  uint8_t previous_state;  // 上次状态
  uint8_t current_state;  // 当前状态
  uint8_t number_flag;  // 计数标志
  uint8_t work_flag;  // 工作标志: 0x00=停止, 0x01=开始
  
  uint32_t total_work_number;  // 总工作次数
  uint32_t total_work_time;  // 单位0.05H(3min)
  
  uint16_t timer_100ms;  // 基于100mS计时器
  uint8_t debounce_timer;  // CAN消息去藕时间
}zxsts_context_t;
extern zxsts_context_t zxsts_context[NUMBER_OF_ZXSTS_TYPES];
extern uint8_t zxsts_tmr[NUMBER_OF_ZXSTS_TYPES*2]; /// 频次统计100ms定时器

// TLV有效标志位
extern bittype2 zxsts_flag1;
#define zxsts_empty_cylinder_flag    zxsts_flag1.w.bit0  //空缸标志
#define zxsts_cylinder_extend_flag    zxsts_flag1.w.bit1  //伸缩缸伸操作标志
#define zxsts_cylinder_shrink_flag    zxsts_flag1.w.bit2  //伸缩缸缩操作标志
#define zxsts_arm_work_flag    zxsts_flag1.w.bit3  // 带臂标志
#define zxsts_luff_up_flag    zxsts_flag1.w.bit4  // 变幅起操作标志
#define zxsts_luff_down_flag    zxsts_flag1.w.bit5  // 变幅缩操作标志
#define zxsts_main_hoist_up_flag    zxsts_flag1.w.bit6  // 主卷起操作标志
#define zxsts_main_hoist_down_flag    zxsts_flag1.w.bit7  // 主卷落操作标志
#define zxsts_deputy_hoist_up_flag    zxsts_flag1.w.bit8  // 副卷起操作标志
#define zxsts_deputy_hoist_down_flag    zxsts_flag1.w.bit9  // 副卷落操作标志
#define zxsts_slew_left_flag    zxsts_flag1.w.bit10  // 左回转操作
#define zxsts_slew_right_flag    zxsts_flag1.w.bit11  // 右回转操作
//#define X    zxsts_flag1.w.bit12
//#define X    zxsts_flag1.w.bit13
//#define X    zxsts_flag1.w.bit14
//#define X    zxsts_flag1.w.bit15

extern bittype2 zxsts_flag2;
#define zxsts_overload_flag    zxsts_flag2.w.bit0  // 超载标志
#define zxsts_od_main_hoist_flag    zxsts_flag2.w.bit1  // 主卷三圈保护标志
#define zxsts_od_deputy_hoist_flag    zxsts_flag2.w.bit2  // 副卷三圈保护标志
#define zxsts_a2b_main_arm_flag    zxsts_flag2.w.bit3  // 主臂高限触发
#define zxsts_a2b_deputy_arm_flag    zxsts_flag2.w.bit4  // 副臂高限触发
#define zxsts_lmi_force_flag    zxsts_flag2.w.bit5  // 总强制
#define zxsts_setup_flag    zxsts_flag2.w.bit6  // 拆装开关
#define zxsts_luff_up_force_flag    zxsts_flag2.w.bit7  // 变幅起强制
#define zxsts_a2b_force_flag    zxsts_flag2.w.bit8  // 高限强制
#define zxsts_od_force_flag    zxsts_flag2.w.bit9  // 三圈强制
#define zxsts_lmi_wind_speed_flag   zxsts_flag2.w.bit10  // 风速超限
//#define X    zxsts_flag2.w.bit11
//#define X    zxsts_flag2.w.bit12
//#define X    zxsts_flag2.w.bit13
//#define X    zxsts_flag2.w.bit14
//#define X    zxsts_flag2.w.bit15

/******************************************************************************
 * 发动机工作统计
 * twt = total work time
 * tfc = total fuel consumption
 * afc = average fuel consumption
 ******************************************************************************/
// 频次统计结构体
typedef struct
{
  uint8_t engine_type;  // 发动机类型: 0x00=无效, 单发=0x01, 双发=0x02
  uint8_t pto_status;  // PTO状态: 0x00=停止, 0x01=作业, 0x02=行驶
  uint8_t work_flag;  // 工作标志: 0x00=停止, 0x01=开始

  uint8_t up_work_flag;  // 上车工作标志: 0x00=停止, 0x01=开始
  uint8_t down_work_flag;  // 下车工作标志: 0x00=停止, 0x01=开始

  //==工时统计=========================================
  uint32_t current_twt_up; // 当前上车总工作时间
  uint32_t previous_twt_up; // 上次上车总工作时间
  uint16_t tdv_up; // 上车时间差值
  uint32_t twt_up;  // 作业(上车)总工作时间(单位0.05H(3min))

  uint32_t current_twt_down; // 当前下车总工作时间
  uint32_t previous_twt_down; // 上次下车总工作时间
  uint16_t tdv_down; // 下车时间差值
  uint32_t twt_down;  // 行驶(下车)总工作时间(单位0.05H(3min))

  //==油耗统计=========================================
  uint32_t current_tfc_up; // 当前上车总油耗
  uint32_t previous_tfc_up; // 上次上车总油耗
  uint16_t fdv_up; // 上车油耗差值
  uint32_t tfc_up;  // 作业(上车)总油耗量(单位0.5L)

  uint32_t current_tfc_down; // 当前下车总油耗
  uint32_t previous_tfc_down; // 上次下车总油耗
  uint16_t fdv_down; // 下车油耗差值
  uint32_t tfc_down;  // 行驶(下车)总油耗量(单位0.5L)

  //==平均油耗=========================================
  uint16_t afc_up;  // 作业(上车)平均油耗(单位0.1L/100Km)
  uint32_t afc_down;  // 行驶(下车)平均油耗(单位0.1L/100Km)

  //uint16_t timer_100ms;  // 基于100mS计时器
  //uint8_t debounce_timer;  // CAN消息去藕时间
}zxsts_engine_context_t;
extern zxsts_engine_context_t zxsts_engine_context;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
uint8_t CAN_ProcessRecvUpMsg(uint32_t canId, uint8_t *pdata, uint8_t size); // 上车
uint8_t CAN_ProcessRecvDownMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size); // 全地面底盘
uint8_t CAN_ProcessRecvEngineMsg_AG(uint32_t canId, uint8_t *pdata, uint8_t size); // 全地面发动机
uint8_t CAN_ProcessRecvDownMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size); // 汽车式底盘
uint8_t CAN_ProcessRecvEngineMsg_AC(uint32_t canId, uint8_t *pdata, uint8_t size); // 汽车式发动机

void ZxSts_StateMachine(void);
void ZxSts_Initialize(void);

void ZxStsEngine_StateMachine(zxsts_engine_context_t* pThis);
void ZxStsEngine_Initialize(zxsts_engine_context_t* pThis);


#endif /* TCW_H_ */

