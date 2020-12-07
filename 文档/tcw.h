/*****************************************************************************
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

/******************************************************************************
* Macros(车型配置信息): 序号和地址定义
******************************************************************************/
//==TAG-A501通用状态字2(重型专用)=================================================

//==TAG-A502环境信息(发动机的数据)================================================

//==TAG-A504采集协议信息==========================================================

/******************************************************************************
* Macros(上车通信): 起重机上车数据序号和地址定义
******************************************************************************/
//==TAG-A5A0力限器工况信息(单缸)==================================================
#define zxup_buffer_a5a0              (zxup_buffer)    // 起始地址定义
#define ZXUP_A5A0_POS1_ADDR    0                                       // 力限器配置1-2
#define ZXUP_A5A0_POS2_ADDR    (ZXUP_A5A0_POS1_ADDR+2)  // 工况代码(ABCDEFGH)
#define ZXUP_A5A0_POS3_ADDR    (ZXUP_A5A0_POS2_ADDR+4)  // 倍率
#define ZXUP_A5A0_POS4_ADDR    (ZXUP_A5A0_POS3_ADDR+1)  // 主臂头部角度
#define ZXUP_A5A0_POS5_ADDR    (ZXUP_A5A0_POS4_ADDR+2)  // 主臂根部角度
#define ZXUP_A5A0_POS6_ADDR    (ZXUP_A5A0_POS5_ADDR+2)  // 主臂长度
#define ZXUP_A5A0_POS7_ADDR    (ZXUP_A5A0_POS6_ADDR+2)  // 二节臂长度
#define ZXUP_A5A0_POS8_ADDR    (ZXUP_A5A0_POS7_ADDR+1)  // 节臂百分比
#define ZXUP_A5A0_POS9_ADDR    (ZXUP_A5A0_POS8_ADDR+8)  // 缸臂销锁死
#define ZXUP_A5A0_POS10_ADDR   (ZXUP_A5A0_POS9_ADDR+1)  // 副臂长度
#define ZXUP_A5A0_POS11_ADDR   (ZXUP_A5A0_POS10_ADDR+2)  // 副臂角度
#define ZXUP_A5A0_POS12_ADDR   (ZXUP_A5A0_POS11_ADDR+2)  // 副臂/塔臂根部角度
#define ZXUP_A5A0_POS13_ADDR   (ZXUP_A5A0_POS12_ADDR+2)  // 副臂/塔臂头部角度
#define ZXUP_A5A0_POS14_ADDR   (ZXUP_A5A0_POS13_ADDR+2)  // 腔压力
#define ZXUP_A5A0_POS15_ADDR   (ZXUP_A5A0_POS14_ADDR+8)  // 臂头高度
#define ZXUP_A5A0_POS16_ADDR   (ZXUP_A5A0_POS15_ADDR+2)  // 额定重量
#define ZXUP_A5A0_POS17_ADDR   (ZXUP_A5A0_POS16_ADDR+2)  // 实际重量
#define ZXUP_A5A0_POS18_ADDR   (ZXUP_A5A0_POS17_ADDR+2)  // 力矩百分比
#define ZXUP_A5A0_POS19_ADDR   (ZXUP_A5A0_POS18_ADDR+2)  // 工作幅度
#define ZXUP_A5A0_POS20_ADDR   (ZXUP_A5A0_POS19_ADDR+2)  // LMI故障代码1-4
#define ZXUP_A5A0_POS21_ADDR   (ZXUP_A5A0_POS20_ADDR+8)  // LMI故障代码5-8
#define ZXUP_A5A0_POS22_ADDR   (ZXUP_A5A0_POS21_ADDR+8)  // 非对称故障代码
#define ZXUP_A5A0_POS23_ADDR   (ZXUP_A5A0_POS22_ADDR+4)  // LMI运行时间
#define ZXUP_A5A0_POS24_ADDR   (ZXUP_A5A0_POS23_ADDR+4)  // 强制位
#define ZXUP_A5A0_POS25_ADDR   (ZXUP_A5A0_POS24_ADDR+1)  // 水平仪X
#define ZXUP_A5A0_POS26_ADDR   (ZXUP_A5A0_POS25_ADDR+2)  // 水平仪Y
#define ZXUP_A5A0_POS27_ADDR   (ZXUP_A5A0_POS26_ADDR+2)  // 风速
#define ZXUP_A5A0_POS28_ADDR   (ZXUP_A5A0_POS27_ADDR+2)  // 回转角度
#define SIZE_OF_ZXUP_A5A0         (ZXUP_A5A0_POS28_ADDR+2) // 总字节数(82B)

//==TAG-A5A1超起工况信息==========================================================
#define zxup_buffer_a5a1       (zxup_buffer_a5a0+SIZE_OF_ZXUP_A5A0) // 起始地址定义
#define ZXUP_A5A1_POS1_ADDR    0                        // 左超起卷扬角度
#define ZXUP_A5A1_POS2_ADDR    (ZXUP_A5A1_POS1_ADDR+2)  // 左超起拉力
#define ZXUP_A5A1_POS3_ADDR    (ZXUP_A5A1_POS2_ADDR+2)  // 左超起展开角度
#define ZXUP_A5A1_POS4_ADDR    (ZXUP_A5A1_POS3_ADDR+2)  // 左超起变幅角度
#define ZXUP_A5A1_POS5_ADDR    (ZXUP_A5A1_POS4_ADDR+2)  // 右超起卷扬角度
#define ZXUP_A5A1_POS6_ADDR    (ZXUP_A5A1_POS5_ADDR+2)  // 右超起拉力
#define ZXUP_A5A1_POS7_ADDR    (ZXUP_A5A1_POS6_ADDR+2)  // 右超起展开角度
#define ZXUP_A5A1_POS8_ADDR    (ZXUP_A5A1_POS7_ADDR+2)  // 右超起变幅角度
#define ZXUP_A5A1_POS9_ADDR    (ZXUP_A5A1_POS8_ADDR+2)  // 左超起-锁止和解锁
#define ZXUP_A5A1_POS10_ADDR   (ZXUP_A5A1_POS9_ADDR+1)  // 右超起-锁止和解锁
#define ZXUP_A5A1_POS11_ADDR   (ZXUP_A5A1_POS10_ADDR+1)  // 左超起齿数
#define ZXUP_A5A1_POS12_ADDR   (ZXUP_A5A1_POS11_ADDR+1)  // 右超起齿数
#define ZXUP_A5A1_POS13_ADDR   (ZXUP_A5A1_POS12_ADDR+1)  // 左超起张紧缸长度
#define ZXUP_A5A1_POS14_ADDR   (ZXUP_A5A1_POS13_ADDR+2)  // 右超起张紧缸长度
#define SIZE_OF_ZXUP_A5A1         (ZXUP_A5A1_POS14_ADDR+2)  // 总字节数(24B)

//==TAG-A5A2塔臂工况信息==========================================================
#define zxup_buffer_a5a2       (zxup_buffer_a5a1+SIZE_OF_ZXUP_A5A1)// 起始地址定义
#define ZXUP_A5A2_POS1_ADDR    0                        // 塔臂拉力左
#define ZXUP_A5A2_POS2_ADDR    (ZXUP_A5A2_POS1_ADDR+2)  // 塔臂拉力右
#define ZXUP_A5A2_POS3_ADDR    (ZXUP_A5A2_POS2_ADDR+2)  // 防后倾压力
#define ZXUP_A5A2_POS4_ADDR    (ZXUP_A5A2_POS3_ADDR+2)  // 前支架角度
#define ZXUP_A5A2_POS5_ADDR    (ZXUP_A5A2_POS4_ADDR+2)  // 第三支架检测
#define SIZE_OF_ZXUP_A5A2      (ZXUP_A5A2_POS5_ADDR+1)  // 总字节数(9B)

//==TAG-A5A5上车发动机信息========================================================
#define zxup_buffer_a5a5       (zxup_buffer_a5a2+SIZE_OF_ZXUP_A5A2)// 起始地址定义
#define ZXUP_A5A5_POS1_ADDR    0                        // 发动机转速
#define ZXUP_A5A5_POS2_ADDR    (ZXUP_A5A5_POS1_ADDR+2)  // 实际扭矩百分比
#define ZXUP_A5A5_POS3_ADDR    (ZXUP_A5A5_POS2_ADDR+1)  // 摩擦扭矩百分比
#define ZXUP_A5A5_POS4_ADDR    (ZXUP_A5A5_POS3_ADDR+1)  // 进气歧管压力
#define ZXUP_A5A5_POS5_ADDR    (ZXUP_A5A5_POS4_ADDR+1)  // 进气歧管温度
#define ZXUP_A5A5_POS6_ADDR    (ZXUP_A5A5_POS5_ADDR+1)  // 冷却液温度
#define ZXUP_A5A5_POS7_ADDR    (ZXUP_A5A5_POS6_ADDR+1)  // 机油温度
#define ZXUP_A5A5_POS8_ADDR    (ZXUP_A5A5_POS7_ADDR+2)  // 机油液位
#define ZXUP_A5A5_POS9_ADDR    (ZXUP_A5A5_POS8_ADDR+1)  // 机油压力
#define ZXUP_A5A5_POS10_ADDR   (ZXUP_A5A5_POS9_ADDR+1)  // 发动机总运行时间
#define ZXUP_A5A5_POS11_ADDR   (ZXUP_A5A5_POS10_ADDR+4)  // 油中有水指示灯
#define ZXUP_A5A5_POS12_ADDR   (ZXUP_A5A5_POS11_ADDR+1)  // 油门踏板百分比
#define ZXUP_A5A5_POS13_ADDR   (ZXUP_A5A5_POS12_ADDR+1)  // 发动机燃油消耗率
#define ZXUP_A5A5_POS14_ADDR   (ZXUP_A5A5_POS13_ADDR+2)  // 发动机平均燃油消耗率
#define ZXUP_A5A5_POS15_ADDR   (ZXUP_A5A5_POS14_ADDR+2)  // 燃油液位
#define ZXUP_A5A5_POS16_ADDR   (ZXUP_A5A5_POS15_ADDR+1)  // 发动机扭矩模式(调速器类型)
#define SIZE_OF_ZXUP_A5A5      (ZXUP_A5A5_POS16_ADDR+1)  // 总字节数(23B)

//==TAG-A5A6手柄信息==============================================================
#define zxup_buffer_a5a6       (zxup_buffer_a5a5+SIZE_OF_ZXUP_A5A5)// 起始地址定义
#define ZXUP_A5A6_POS1_ADDR    0                        // 左手柄状态位
#define ZXUP_A5A6_POS2_ADDR    (ZXUP_A5A6_POS1_ADDR+1)  // 左手柄X输出
#define ZXUP_A5A6_POS3_ADDR    (ZXUP_A5A6_POS2_ADDR+2)  // 左手柄Y输出
#define ZXUP_A5A6_POS4_ADDR    (ZXUP_A5A6_POS3_ADDR+2)  // 右手柄状态位
#define ZXUP_A5A6_POS5_ADDR    (ZXUP_A5A6_POS4_ADDR+1)  // 右手柄X输出
#define ZXUP_A5A6_POS6_ADDR    (ZXUP_A5A6_POS5_ADDR+2)  // 右手柄Y输出
#define SIZE_OF_ZXUP_A5A6      (ZXUP_A5A6_POS6_ADDR+2)  // 总字节数(10B)

//==TAG-A5A7显示器1信息===========================================================
#define zxup_buffer_a5a7       (zxup_buffer_a5a6+SIZE_OF_ZXUP_A5A6)// 起始地址定义
#define ZXUP_A5A7_POS1_ADDR    0                        // 产品配置
#define ZXUP_A5A7_POS2_ADDR    (ZXUP_A5A7_POS1_ADDR+4)  // 工况代码
#define ZXUP_A5A7_POS3_ADDR    (ZXUP_A5A7_POS2_ADDR+4)  // 主卷起速度
#define ZXUP_A5A7_POS4_ADDR    (ZXUP_A5A7_POS3_ADDR+1)  // 主卷落速度
#define ZXUP_A5A7_POS5_ADDR    (ZXUP_A5A7_POS4_ADDR+1)  // 副卷起速度
#define ZXUP_A5A7_POS6_ADDR    (ZXUP_A5A7_POS5_ADDR+1)  // 副卷落速度
#define ZXUP_A5A7_POS7_ADDR    (ZXUP_A5A7_POS6_ADDR+1)  // 变幅起速度
#define ZXUP_A5A7_POS8_ADDR    (ZXUP_A5A7_POS7_ADDR+1)  // 变幅落速度
#define ZXUP_A5A7_POS9_ADDR    (ZXUP_A5A7_POS8_ADDR+1)  // 左回转速度
#define ZXUP_A5A7_POS10_ADDR   (ZXUP_A5A7_POS9_ADDR+1)  // 右回转速度
#define ZXUP_A5A7_POS11_ADDR   (ZXUP_A5A7_POS10_ADDR+1)  // 目标组合
#define ZXUP_A5A7_POS12_ADDR   (ZXUP_A5A7_POS11_ADDR+1)  // 状态位1
#define ZXUP_A5A7_POS13_ADDR   (ZXUP_A5A7_POS12_ADDR+1)  // 状态位2
#define ZXUP_A5A7_POS14_ADDR   (ZXUP_A5A7_POS13_ADDR+1)  // 伸缩模式
#define SIZE_OF_ZXUP_A5A7      (ZXUP_A5A7_POS14_ADDR+1)  // 总字节数(20B)

//==TAG-A5A8显示器2信息===========================================================
#define zxup_buffer_a5a8       (zxup_buffer_a5a7+SIZE_OF_ZXUP_A5A7)// 起始地址定义
#define ZXUP_A5A8_POS1_ADDR    0                        // 超起操控
#define ZXUP_A5A8_POS2_ADDR    (ZXUP_A5A8_POS1_ADDR+8)  // 超起维修
#define SIZE_OF_ZXUP_A5A8      (ZXUP_A5A8_POS2_ADDR+2)  // 总字节数(10B)

//==TAG-A5A9总线面板信息==========================================================
#define zxup_buffer_a5a9       (zxup_buffer_a5a8+SIZE_OF_ZXUP_A5A8)// 起始地址定义
#define ZXUP_A5A9_POS1_ADDR    0                        // 面板1
#define ZXUP_A5A9_POS2_ADDR    (ZXUP_A5A9_POS1_ADDR+2)  // 面板2
#define ZXUP_A5A9_POS3_ADDR    (ZXUP_A5A9_POS2_ADDR+2)  // 面板3
#define SIZE_OF_ZXUP_A5A9      (ZXUP_A5A9_POS3_ADDR+2)  // 总字节数(6B)

//==TAG-A5AA无线操控信息==========================================================
#define zxup_buffer_a5aa       (zxup_buffer_a5a9+SIZE_OF_ZXUP_A5A9)// 起始地址定义
#define ZXUP_A5AA_POS1_ADDR    0                        // Msg1
#define ZXUP_A5AA_POS2_ADDR    (ZXUP_A5AA_POS1_ADDR+8)  // Msg2
#define ZXUP_A5AA_POS3_ADDR    (ZXUP_A5AA_POS2_ADDR+8)  // Msg3
#define ZXUP_A5AA_POS4_ADDR    (ZXUP_A5AA_POS2_ADDR+8)  // Msg4
#define SIZE_OF_ZXUP_A5AA      (ZXUP_A5AA_POS4_ADDR+8)  // 总字节数(32B)

//==TAG-A5AB网络拓扑信息==========================================================
#define zxup_buffer_a5ab       (zxup_buffer_a5aa+SIZE_OF_ZXUP_A5AA)// 起始地址定义
#define ZXUP_A5AB_POS1_ADDR    0                        // 节点状态
#define SIZE_OF_ZXUP_A5AB      (ZXUP_A5AB_POS1_ADDR+8)  // 总字节数(8B)

//==TAG-A5AC伸缩逻辑信息==========================================================
#define zxup_buffer_a5ac       (zxup_buffer_a5ab+SIZE_OF_ZXUP_A5AB)// 起始地址定义
#define ZXUP_A5AC_POS1_ADDR    0                        // 伸缩限制和解除
#define SIZE_OF_ZXUP_A5AC      (ZXUP_A5AC_POS1_ADDR+6)  // 总字节数(6B)

//==TAG-A5AD变幅逻辑信息==========================================================
#define zxup_buffer_a5ad       (zxup_buffer_a5ac+SIZE_OF_ZXUP_A5AC)// 起始地址定义
#define ZXUP_A5AD_POS1_ADDR    0                        // 起落限制和解除
#define SIZE_OF_ZXUP_A5AD      (ZXUP_A5AD_POS1_ADDR+8)  // 总字节数(8B)

//==TAG-A5AE回转逻辑信息==========================================================
#define zxup_buffer_a5ae       (zxup_buffer_a5ad+SIZE_OF_ZXUP_A5AD)// 起始地址定义
#define ZXUP_A5AE_POS1_ADDR    0                        // 左回限制1
#define ZXUP_A5AE_POS2_ADDR    (ZXUP_A5AE_POS1_ADDR+1)  // 左回限制2
#define ZXUP_A5AE_POS3_ADDR    (ZXUP_A5AE_POS2_ADDR+1)  // 左回解除
#define ZXUP_A5AE_POS4_ADDR    (ZXUP_A5AE_POS3_ADDR+1)  // 右回限制1
#define ZXUP_A5AE_POS5_ADDR    (ZXUP_A5AE_POS4_ADDR+1)  // 右回限制2
#define ZXUP_A5AE_POS6_ADDR    (ZXUP_A5AE_POS5_ADDR+1)  // 右回解除
#define SIZE_OF_ZXUP_A5AE      (ZXUP_A5AE_POS6_ADDR+1)  // 总字节数(6B)

//==TAG-A5AF主卷扬逻辑信息========================================================
#define zxup_buffer_a5af       (zxup_buffer_a5ae+SIZE_OF_ZXUP_A5AE)// 起始地址定义
#define ZXUP_A5AF_POS1_ADDR    0                        // 主卷扬起落限制和解除
#define SIZE_OF_ZXUP_A5AF      (ZXUP_A5AF_POS1_ADDR+5)  // 总字节数(5B)

//==TAG-A5B0副卷扬逻辑信息========================================================
#define zxup_buffer_a5b0       (zxup_buffer_a5af+SIZE_OF_ZXUP_A5AF)// 起始地址定义
#define ZXUP_A5B0_POS1_ADDR    0                        // 副卷扬起落限制和解除
#define SIZE_OF_ZXUP_A5B0      (ZXUP_A5B0_POS1_ADDR+4)  // 总字节数(4B)

//==TAG-A5B1超起逻辑信息==========================================================
#define zxup_buffer_a5b1       (zxup_buffer_a5b0+SIZE_OF_ZXUP_A5B0)// 起始地址定义
#define ZXUP_A5B1_POS1_ADDR    0                        // 超起目标展开角度
#define ZXUP_A5B1_POS2_ADDR    (ZXUP_A5B1_POS1_ADDR+1)  // 超起目标张紧角度
#define ZXUP_A5B1_POS3_ADDR    (ZXUP_A5B1_POS2_ADDR+2)  // 超起一键张紧限制逻辑
#define ZXUP_A5B1_POS4_ADDR    (ZXUP_A5B1_POS2_ADDR+2)  // 张紧拉力状态位
#define SIZE_OF_ZXUP_A5B1      (ZXUP_A5B1_POS4_ADDR+1)  // 总字节数(6B)

//==TAG-A5B2塔臂逻辑信息==========================================================
#define zxup_buffer_a5b2       (zxup_buffer_a5b1+SIZE_OF_ZXUP_A5B1)// 起始地址定义
#define ZXUP_A5B2_POS1_ADDR    0                        // 塔卷起限制
#define ZXUP_A5B2_POS2_ADDR    (ZXUP_A5B2_POS1_ADDR+2)  // 塔卷落限制
#define ZXUP_A5B2_POS3_ADDR    (ZXUP_A5B2_POS2_ADDR+2)  // 塔臂限制伸臂
#define ZXUP_A5B2_POS4_ADDR    (ZXUP_A5B2_POS3_ADDR+1)  // 塔臂限制缩臂
#define ZXUP_A5B2_POS5_ADDR    (ZXUP_A5B2_POS4_ADDR+1)  // 塔臂限制变幅起
#define ZXUP_A5B2_POS6_ADDR    (ZXUP_A5B2_POS5_ADDR+1)  // 塔臂限制变幅落
#define ZXUP_A5B2_POS7_ADDR    (ZXUP_A5B2_POS6_ADDR+1)  // 塔臂限制主卷起
#define ZXUP_A5B2_POS8_ADDR    (ZXUP_A5B2_POS7_ADDR+1)  // 塔臂限制主卷落
#define SIZE_OF_ZXUP_A5B2      (ZXUP_A5B2_POS8_ADDR+1)  // 总字节数(10B)

//==TAG-A5B3液压油温度============================================================
#define zxup_buffer_a5b3       (zxup_buffer_a5b2+SIZE_OF_ZXUP_A5B2)// 起始地址定义
#define ZXUP_A5B3_POS1_ADDR    0                        // 液压油温度
#define ZXUP_A5B3_POS2_ADDR    (ZXUP_A5B3_POS1_ADDR+2)  // 液压油位
#define ZXUP_A5B3_POS3_ADDR    (ZXUP_A5B3_POS2_ADDR+1)  // 回油阻塞报警
#define SIZE_OF_ZXUP_A5B3      (ZXUP_A5B3_POS3_ADDR+1)  // 总字节数(4B)

//==TAG-A5B4主泵信息==============================================================
#define zxup_buffer_a5b4       (zxup_buffer_a5b3+SIZE_OF_ZXUP_A5B3)// 起始地址定义
#define ZXUP_A5B4_POS1_ADDR    0                        // 1#主泵电磁阀
#define ZXUP_A5B4_POS2_ADDR    (ZXUP_A5B4_POS1_ADDR+2)  // 2#主泵电磁阀
#define ZXUP_A5B4_POS3_ADDR    (ZXUP_A5B4_POS2_ADDR+2)  // 主泵压力
#define SIZE_OF_ZXUP_A5B4      (ZXUP_A5B4_POS3_ADDR+2)  // 总字节数(6B)

//==TAG-A5B5主阀1 XHVME4400P1=====================================================
#define zxup_buffer_a5b5       (zxup_buffer_a5b4+SIZE_OF_ZXUP_A5B4)// 起始地址定义
#define ZXUP_A5B5_POS1_ADDR    0                        // 伸电磁阀
#define ZXUP_A5B5_POS2_ADDR    (ZXUP_A5B5_POS1_ADDR+2)  // 缩电磁阀
#define ZXUP_A5B5_POS3_ADDR    (ZXUP_A5B5_POS2_ADDR+2)  // 变幅起电磁阀
#define ZXUP_A5B5_POS4_ADDR    (ZXUP_A5B5_POS3_ADDR+2)  // 变幅落电磁阀
#define ZXUP_A5B5_POS5_ADDR    (ZXUP_A5B5_POS4_ADDR+2)  // 主卷起电磁阀
#define ZXUP_A5B5_POS6_ADDR    (ZXUP_A5B5_POS5_ADDR+2)  // 主卷落电磁阀
#define ZXUP_A5B5_POS7_ADDR    (ZXUP_A5B5_POS6_ADDR+2)  // 副卷落电磁阀
#define ZXUP_A5B5_POS8_ADDR    (ZXUP_A5B5_POS7_ADDR+2)  //副卷落电磁阀
#define ZXUP_A5B5_POS9_ADDR    (ZXUP_A5B5_POS8_ADDR+2)  // MP1压力
#define ZXUP_A5B5_POS10_ADDR   (ZXUP_A5B5_POS9_ADDR+2)  // LS1压力
#define ZXUP_A5B5_POS11_ADDR   (ZXUP_A5B5_POS10_ADDR+2)  // MP2压力
#define ZXUP_A5B5_POS12_ADDR   (ZXUP_A5B5_POS11_ADDR+2)  // LS2压力
#define ZXUP_A5B5_POS13_ADDR   (ZXUP_A5B5_POS12_ADDR+2)  // 合流电磁阀
#define SIZE_OF_ZXUP_A5B5      (ZXUP_A5B5_POS13_ADDR+1)  // 总字节数(23B)

//==TAG-A5B6主阀3 大吨位==========================================================
#define zxup_buffer_a5b6       (zxup_buffer_a5b5+SIZE_OF_ZXUP_A5B5)// 起始地址定义
#define ZXUP_A5B6_POS1_ADDR    0                        // 状态位
#define SIZE_OF_ZXUP_A5B6      (ZXUP_A5B6_POS1_ADDR+1)  // 总字节数(1B)

//==TAG-A5B7主泵信息==============================================================
#define zxup_buffer_a5b7       (zxup_buffer_a5b6+SIZE_OF_ZXUP_A5B6)// 起始地址定义
#define ZXUP_A5B7_POS1_ADDR    0                        // 伸缩缸压力
#define ZXUP_A5B7_POS2_ADDR    (ZXUP_A5B7_POS1_ADDR+2)  // 伸缩缸长度
#define ZXUP_A5B7_POS3_ADDR    (ZXUP_A5B7_POS2_ADDR+2)  // 电控平衡阀
#define SIZE_OF_ZXUP_A5B7      (ZXUP_A5B7_POS3_ADDR+2)  // 总字节数(6B)

//==TAG-A5B8塔臂逻辑信息==========================================================
#define zxup_buffer_a5b8       (zxup_buffer_a5b7+SIZE_OF_ZXUP_A5B7)// 起始地址定义
#define ZXUP_A5B8_POS1_ADDR    0                        // 销锁死和解锁状态位
#define ZXUP_A5B8_POS2_ADDR    (ZXUP_A5B8_POS1_ADDR+1)  // 头体标志
#define ZXUP_A5B8_POS3_ADDR    (ZXUP_A5B8_POS2_ADDR+1)  // 前缸头体检测开关8
#define ZXUP_A5B8_POS4_ADDR    (ZXUP_A5B8_POS3_ADDR+1)  // 后缸头体检测开关8
#define ZXUP_A5B8_POS5_ADDR    (ZXUP_A5B8_POS4_ADDR+1)  // 臂位
#define SIZE_OF_ZXUP_A5B8      (ZXUP_A5B8_POS5_ADDR+1)  // 总字节数(5B)

//==TAG-A5B9缸臂销控制阀==========================================================
#define zxup_buffer_a5b9       (zxup_buffer_a5b8+SIZE_OF_ZXUP_A5B8)// 起始地址定义
#define ZXUP_A5B9_POS1_ADDR    0                        // 状态字节
#define ZXUP_A5B9_POS2_ADDR    (ZXUP_A5B9_POS1_ADDR+1)  // 蓄能器压力
#define SIZE_OF_ZXUP_A5B9      (ZXUP_A5B9_POS2_ADDR+2)  // 总字节数(3B)

//==TAG-A5BA变幅平衡阀============================================================
#define zxup_buffer_a5ba       (zxup_buffer_a5b9+SIZE_OF_ZXUP_A5B9)// 起始地址定义
#define ZXUP_A5BA_POS1_ADDR    0                        // 左变幅平衡阀电流
#define ZXUP_A5BA_POS2_ADDR    (ZXUP_A5BA_POS1_ADDR+1)  // 右变幅平衡阀电流
#define SIZE_OF_ZXUP_A5BA      (ZXUP_A5BA_POS2_ADDR+2)  // 总字节数(4B)

//==TAG-A5BB主卷泵================================================================
#define zxup_buffer_a5bb       (zxup_buffer_a5ba+SIZE_OF_ZXUP_A5BA)// 起始地址定义
#define ZXUP_A5BB_POS1_ADDR    0                        // 起升电磁阀
#define ZXUP_A5BB_POS2_ADDR    (ZXUP_A5BB_POS1_ADDR+2)  // 下落电磁阀
#define ZXUP_A5BB_POS3_ADDR    (ZXUP_A5BB_POS2_ADDR+2)  // 油泵压力
#define SIZE_OF_ZXUP_A5BB      (ZXUP_A5BB_POS3_ADDR+2)  // 总字节数(6B)

//==TAG-A5BC主卷马达==============================================================
#define zxup_buffer_a5bc       (zxup_buffer_a5bb+SIZE_OF_ZXUP_A5BB)// 起始地址定义
#define ZXUP_A5BC_POS1_ADDR    0                        // 马达电流
#define ZXUP_A5BC_POS2_ADDR    (ZXUP_A5BC_POS1_ADDR+2)  // 卷筒转速
#define ZXUP_A5BC_POS3_ADDR    (ZXUP_A5BC_POS2_ADDR+2)  // 状态位
#define SIZE_OF_ZXUP_A5BC      (ZXUP_A5BC_POS3_ADDR+1)  // 总字节数(5B)

//==TAG-A5BD塔卷泵================================================================
#define zxup_buffer_a5bd       (zxup_buffer_a5bc+SIZE_OF_ZXUP_A5BC)// 起始地址定义
#define ZXUP_A5BD_POS1_ADDR    0                        // 起升电磁阀
#define ZXUP_A5BD_POS2_ADDR    (ZXUP_A5BD_POS1_ADDR+2)  // 下落电磁阀
#define ZXUP_A5BD_POS3_ADDR    (ZXUP_A5BD_POS2_ADDR+2)  // 油泵压力
#define SIZE_OF_ZXUP_A5BD      (ZXUP_A5BD_POS3_ADDR+2)  // 总字节数(6B)

//==TAG-A5BE塔卷马达==============================================================
#define zxup_buffer_a5be       (zxup_buffer_a5bd+SIZE_OF_ZXUP_A5BD)// 起始地址定义
#define ZXUP_A5BE_POS1_ADDR    0                        // 马达电流
#define ZXUP_A5BE_POS2_ADDR    (ZXUP_A5BE_POS1_ADDR+2)  // 卷筒转速
#define ZXUP_A5BE_POS3_ADDR    (ZXUP_A5BE_POS2_ADDR+2)  // 状态位
#define SIZE_OF_ZXUP_A5BE      (ZXUP_A5BE_POS3_ADDR+1)  // 总字节数(5B)

//==TAG-A5BF回转泵================================================================
#define zxup_buffer_a5bf       (zxup_buffer_a5be+SIZE_OF_ZXUP_A5BE)// 起始地址定义
#define ZXUP_A5BF_POS1_ADDR    0                        // 左回转电磁阀
#define ZXUP_A5BF_POS2_ADDR    (ZXUP_A5BF_POS1_ADDR+2)  // 右回转电磁阀
#define ZXUP_A5BF_POS3_ADDR    (ZXUP_A5BF_POS2_ADDR+2)  // 回转缓冲阀
#define ZXUP_A5BF_POS4_ADDR    (ZXUP_A5BF_POS3_ADDR+2)  // 油泵压力（回转压力检测）
#define SIZE_OF_ZXUP_A5BF      (ZXUP_A5BF_POS4_ADDR+2)  // 总字节数(8B)

//==TAG-A5C0回转制动阀============================================================
#define zxup_buffer_a5c0       (zxup_buffer_a5bf+SIZE_OF_ZXUP_A5BF)// 起始地址定义
#define ZXUP_A5C0_POS1_ADDR    0                        // 字节状态位
#define ZXUP_A5C0_POS2_ADDR    (ZXUP_A5C0_POS1_ADDR+1)  // 卷筒转速
#define SIZE_OF_ZXUP_A5C0      (ZXUP_A5C0_POS2_ADDR+2)  // 总字节数(3B)

//==TAG-A5C1辅助阀1===============================================================
#define zxup_buffer_a5c1       (zxup_buffer_a5c0+SIZE_OF_ZXUP_A5C0)// 起始地址定义
#define ZXUP_A5C1_POS1_ADDR    0                        // 字节状态位1
#define ZXUP_A5C1_POS2_ADDR    (ZXUP_A5C1_POS1_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXUP_A5C1      (ZXUP_A5C1_POS2_ADDR+1)  // 总字节数(2B)

//==TAG-A5C2辅助阀2(超大吨位)=====================================================
#define zxup_buffer_a5c2       (zxup_buffer_a5c1+SIZE_OF_ZXUP_A5C1)// 起始地址定义
#define ZXUP_A5C2_POS1_ADDR    0                        // 压力选择
#define ZXUP_A5C2_POS2_ADDR    (ZXUP_A5C2_POS1_ADDR+2)  // 字节状态位
#define SIZE_OF_ZXUP_A5C2      (ZXUP_A5C2_POS2_ADDR+1)  // 总字节数(3B)

//==TAG-A5C3左超起阀组============================================================
#define zxup_buffer_a5c3       (zxup_buffer_a5c2+SIZE_OF_ZXUP_A5C2)// 起始地址定义
#define ZXUP_A5C3_POS1_ADDR    0                        // 卷扬起
#define ZXUP_A5C3_POS2_ADDR    (ZXUP_A5C3_POS1_ADDR+2)  // 卷扬落
#define ZXUP_A5C3_POS3_ADDR    (ZXUP_A5C3_POS2_ADDR+2)  // 超起马达变量
#define ZXUP_A5C3_POS4_ADDR    (ZXUP_A5C3_POS3_ADDR+2)  // 字节状态位1
#define ZXUP_A5C3_POS5_ADDR    (ZXUP_A5C3_POS4_ADDR+1)  // 字节状态位2
#define ZXUP_A5C3_POS6_ADDR    (ZXUP_A5C3_POS5_ADDR+1)  // 张紧缸伸
#define ZXUP_A5C3_POS7_ADDR    (ZXUP_A5C3_POS6_ADDR+2)  // 张紧缸缩
#define SIZE_OF_ZXUP_A5C3      (ZXUP_A5C3_POS7_ADDR+2)  // 总字节数(12B)

//==TAG-A5C4右超起阀组============================================================
#define zxup_buffer_a5c4       (zxup_buffer_a5c3+SIZE_OF_ZXUP_A5C3)// 起始地址定义
#define ZXUP_A5C4_POS1_ADDR    0                        // 卷扬起
#define ZXUP_A5C4_POS2_ADDR    (ZXUP_A5C4_POS1_ADDR+2)  // 卷扬落
#define ZXUP_A5C4_POS3_ADDR    (ZXUP_A5C4_POS2_ADDR+2)  // 超起马达变量
#define ZXUP_A5C4_POS4_ADDR    (ZXUP_A5C4_POS3_ADDR+2)  // 字节状态位1
#define ZXUP_A5C4_POS5_ADDR    (ZXUP_A5C4_POS4_ADDR+1)  // 字节状态位2
#define ZXUP_A5C4_POS6_ADDR    (ZXUP_A5C4_POS5_ADDR+1)  // 张紧缸伸
#define ZXUP_A5C4_POS7_ADDR    (ZXUP_A5C4_POS6_ADDR+2)  // 张紧缸缩
#define SIZE_OF_ZXUP_A5C4      (ZXUP_A5C4_POS7_ADDR+2)  // 总字节数(12B)

//==TAG-A5C8作业油耗信息==========================================================
#define zxup_buffer_a5c8              (zxup_buffer_a5c4+SIZE_OF_ZXUP_A5C4)// 起始地址定义
#define ZXUP_A5C8_POS1_ADDR    0                        // 作业总运行时间
#define ZXUP_A5C8_POS2_ADDR    (ZXUP_A5C8_POS1_ADDR+4)  // 作业燃油总消耗量
#define ZXUP_A5C8_POS3_ADDR    (ZXUP_A5C8_POS2_ADDR+4)  // 作业平均油耗
#define SIZE_OF_ZXUP_A5C8      (ZXUP_A5C8_POS3_ADDR+2)  // 总字节数(10B)

//==TAG-A5C9 ECU响应锁车CAN帧=====================================================
#define zxup_buffer_a5c9       (zxup_buffer_a5c8+SIZE_OF_ZXUP_A5C8)// 起始地址定义
#define ZXUP_A5C9_POS1_ADDR    0                        // 作业总运行时间
#define ZXUP_A5C9_POS2_ADDR    (ZXUP_A5C9_POS1_ADDR+1)  // 作业燃油总消耗量
#define ZXUP_A5C9_POS3_ADDR    (ZXUP_A5C9_POS2_ADDR+4)  // 作业平均油耗
#define SIZE_OF_ZXUP_A5C9      (ZXUP_A5C9_POS3_ADDR+8)  // 总字节数(13B)

// 上车数据缓存大小
#define SIZE_OF_ZXUP_BUFFER   (zxup_buffer_a5c9+SIZE_OF_ZXUP_A5C9-zxup_buffer_a5a0)

/******************************************************************************
* Macros(下车通信): 起重机下车数据序号和地址定义
******************************************************************************/
//==TAG-A5E0节点状态==============================================================
#define zxdown_buffer_a5e0          (zxdown_buffer)   // 起始地址定义
#define ZXDOWN_A5E0_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5E0_POS2_ADDR    (ZXDOWN_A5E0_POS1_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E0      (ZXDOWN_A5E0_POS2_ADDR+1)  // 总字节数(2B)

//==TAG-A5E1传动系统==============================================================
#define zxdown_buffer_a5e1       (zxdown_buffer_a5e0+SIZE_OF_ZXDOWN_A5E0) // 起始地址定义
#define ZXDOWN_A5E1_POS1_ADDR    0                          // 变速箱档位
#define ZXDOWN_A5E1_POS2_ADDR    (ZXDOWN_A5E1_POS1_ADDR+1)  // 字节状态位
#define ZXDOWN_A5E1_POS3_ADDR    (ZXDOWN_A5E1_POS2_ADDR+1)  // 变速箱油温
#define ZXDOWN_A5E1_POS4_ADDR    (ZXDOWN_A5E1_POS3_ADDR+2)  // 变速箱进气气压
#define ZXDOWN_A5E1_POS5_ADDR    (ZXDOWN_A5E1_POS4_ADDR+1)  // 离合器位置
#define SIZE_OF_ZXDOWN_A5E1      (ZXDOWN_A5E1_POS5_ADDR+1)  // 总字节数(6B)

//==TAG-A5E2支腿相关信息==========================================================
#define zxdown_buffer_a5e2       (zxdown_buffer_a5e1+SIZE_OF_ZXDOWN_A5E1)// 起始地址定义
#define ZXDOWN_A5E2_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5E2_POS2_ADDR    (ZXDOWN_A5E2_POS1_ADDR+1)  // 字节状态位2
#define ZXDOWN_A5E2_POS3_ADDR    (ZXDOWN_A5E2_POS2_ADDR+1)  // 字节状态位3
#define SIZE_OF_ZXDOWN_A5E2      (ZXDOWN_A5E2_POS3_ADDR+1)  // 总字节数(3B)

//==TAG-A5E3悬挂系统==============================================================
#define zxdown_buffer_a5e3       (zxdown_buffer_a5e2+SIZE_OF_ZXDOWN_A5E2)// 起始地址定义
#define ZXDOWN_A5E3_POS1_ADDR    0                          // 悬挂压力
#define ZXDOWN_A5E3_POS2_ADDR    (ZXDOWN_A5E3_POS1_ADDR+8)  // 悬挂行程
#define ZXDOWN_A5E3_POS3_ADDR    (ZXDOWN_A5E3_POS2_ADDR+8)  // 整车重量
#define ZXDOWN_A5E3_POS4_ADDR    (ZXDOWN_A5E3_POS3_ADDR+4)  // 字节状态位1
#define ZXDOWN_A5E3_POS5_ADDR    (ZXDOWN_A5E3_POS4_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E3      (ZXDOWN_A5E3_POS5_ADDR+1)  // 总字节数(22B)

//==TAG-A5E4转向系统（全地面）====================================================
#define zxdown_buffer_a5e4       (zxdown_buffer_a5e3+SIZE_OF_ZXDOWN_A5E3)// 起始地址定义
#define ZXDOWN_A5E4_POS1_ADDR    0                          // 一二轴转向角度
#define ZXDOWN_A5E4_POS2_ADDR    (ZXDOWN_A5E4_POS1_ADDR+8)  // 三四五六轴转向角度
#define ZXDOWN_A5E4_POS3_ADDR    (ZXDOWN_A5E4_POS2_ADDR+8)  // 一二轴传感器电流
#define ZXDOWN_A5E4_POS4_ADDR    (ZXDOWN_A5E4_POS3_ADDR+8)  // 三四五六轴传感器电流
#define ZXDOWN_A5E4_POS5_ADDR    (ZXDOWN_A5E4_POS4_ADDR+8)  // 当前转向模式
#define ZXDOWN_A5E4_POS6_ADDR    (ZXDOWN_A5E4_POS5_ADDR+1)  // 目标转向模式
#define ZXDOWN_A5E4_POS7_ADDR    (ZXDOWN_A5E4_POS6_ADDR+1)  // 转向系统压力
#define ZXDOWN_A5E4_POS8_ADDR    (ZXDOWN_A5E4_POS7_ADDR+2)  // 比例调压阀电流
#define ZXDOWN_A5E4_POS9_ADDR    (ZXDOWN_A5E4_POS8_ADDR+2)  // 123桥左右转阀占空比
#define ZXDOWN_A5E4_POS10_ADDR   (ZXDOWN_A5E4_POS9_ADDR+6)  // 456桥左右转阀占空比
#define ZXDOWN_A5E4_POS11_ADDR   (ZXDOWN_A5E4_POS10_ADDR+6)  // 桥锁止阀
#define SIZE_OF_ZXDOWN_A5E4      (ZXDOWN_A5E4_POS11_ADDR+1)  // 总字节数(51B)

//==TAG-A5E5转向系统（汽车）======================================================
#define zxdown_buffer_a5e5       (zxdown_buffer_a5e4+SIZE_OF_ZXDOWN_A5E4)// 起始地址定义
#define ZXDOWN_A5E5_POS1_ADDR    0                          // 当前一轴转角
#define ZXDOWN_A5E5_POS2_ADDR    (ZXDOWN_A5E5_POS1_ADDR+2)  // 转向锁死压力(bar)
#define ZXDOWN_A5E5_POS3_ADDR    (ZXDOWN_A5E5_POS2_ADDR+2)  // 目标转向模式+当前转向模式
#define ZXDOWN_A5E5_POS4_ADDR    (ZXDOWN_A5E5_POS3_ADDR+2)  // 程序标记位
#define ZXDOWN_A5E5_POS5_ADDR    (ZXDOWN_A5E5_POS4_ADDR+1)  // 字节状态位1
#define ZXDOWN_A5E5_POS6_ADDR    (ZXDOWN_A5E5_POS5_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E5      (ZXDOWN_A5E5_POS6_ADDR+1)  // 总字节数(9B)

//==TAG-A5E6制动系统==============================================================
#define zxdown_buffer_a5e6       (zxdown_buffer_a5e5+SIZE_OF_ZXDOWN_A5E5)// 起始地址定义
#define ZXDOWN_A5E6_POS1_ADDR    0                          // 回路一气压
#define ZXDOWN_A5E6_POS2_ADDR    (ZXDOWN_A5E6_POS1_ADDR+2)  // 回路二气压
#define ZXDOWN_A5E6_POS3_ADDR    (ZXDOWN_A5E6_POS2_ADDR+2)  // 制动压力
#define ZXDOWN_A5E6_POS4_ADDR    (ZXDOWN_A5E6_POS3_ADDR+2)  // 电压检测
#define ZXDOWN_A5E6_POS5_ADDR    (ZXDOWN_A5E6_POS4_ADDR+2)  // 缓速器油温
#define ZXDOWN_A5E6_POS6_ADDR    (ZXDOWN_A5E6_POS5_ADDR+2)  // 缓速力矩百分比
#define ZXDOWN_A5E6_POS7_ADDR    (ZXDOWN_A5E6_POS6_ADDR+1)  // 变矩器油温
#define ZXDOWN_A5E6_POS8_ADDR    (ZXDOWN_A5E6_POS7_ADDR+2)  // 字节状态位1
#define ZXDOWN_A5E6_POS9_ADDR    (ZXDOWN_A5E6_POS8_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E6      (ZXDOWN_A5E6_POS9_ADDR+1)  // 总字节数(15B)

//==TAG-A5E7动力系统==============================================================
#define zxdown_buffer_a5e7       (zxdown_buffer_a5e6+SIZE_OF_ZXDOWN_A5E6)// 起始地址定义
#define ZXDOWN_A5E7_POS1_ADDR    0                          // 字节状态位
#define SIZE_OF_ZXDOWN_A5E7      (ZXDOWN_A5E7_POS1_ADDR+1)  // 总字节数(1B)

//==TAG-A5E8单发取力系统==========================================================
#define zxdown_buffer_a5e8       (zxdown_buffer_a5e7+SIZE_OF_ZXDOWN_A5E7)// 起始地址定义
#define ZXDOWN_A5E8_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5E8_POS2_ADDR    (ZXDOWN_A5E8_POS1_ADDR+1)  // 字节状态位2
#define SIZE_OF_ZXDOWN_A5E8      (ZXDOWN_A5E8_POS2_ADDR+1)  // 总字节数(2B)

//==TAG-A5E9液压系统==============================================================
#define zxdown_buffer_a5e9       (zxdown_buffer_a5e8+SIZE_OF_ZXDOWN_A5E8)// 起始地址定义
#define ZXDOWN_A5E9_POS1_ADDR    0                                       // 液压油温度
#define ZXDOWN_A5E9_POS2_ADDR    (ZXDOWN_A5E9_POS1_ADDR+1)  // 液压系统压力
#define SIZE_OF_ZXDOWN_A5E9      (ZXDOWN_A5E9_POS2_ADDR+2)  // 总字节数(3B)

//==TAG-A5EA双动力驱动系统========================================================
#define zxdown_buffer_a5ea       (zxdown_buffer_a5e9+SIZE_OF_ZXDOWN_A5E9)// 起始地址定义
#define ZXDOWN_A5EA_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5EA_POS2_ADDR    (ZXDOWN_A5EA_POS1_ADDR+1)  // 字节状态位2
#define ZXDOWN_A5EA_POS3_ADDR    (ZXDOWN_A5EA_POS2_ADDR+1)  // 上车发动机水温
#define ZXDOWN_A5EA_POS4_ADDR    (ZXDOWN_A5EA_POS3_ADDR+1)  // 上车发动机机油压力
#define ZXDOWN_A5EA_POS5_ADDR    (ZXDOWN_A5EA_POS4_ADDR+2)  // 上车发动机转速
#define ZXDOWN_A5EA_POS6_ADDR    (ZXDOWN_A5EA_POS5_ADDR+2)  // 上车泵出口压力
#define ZXDOWN_A5EA_POS7_ADDR    (ZXDOWN_A5EA_POS6_ADDR+2)  // 马达控制电流
#define ZXDOWN_A5EA_POS8_ADDR    (ZXDOWN_A5EA_POS7_ADDR+2)  // 散热器液压驱动泵控制电流
#define SIZE_OF_ZXDOWN_A5EA      (ZXDOWN_A5EA_POS8_ADDR+2)  // 总字节数(13B)

//==TAG-A5EB轮胎胎压==============================================================
#define zxdown_buffer_a5eb       (zxdown_buffer_a5ea+SIZE_OF_ZXDOWN_A5EA)// 起始地址定义
#define ZXDOWN_A5EB_POS1_ADDR    0                          // 胎压#1-6
#define ZXDOWN_A5EB_POS2_ADDR    (ZXDOWN_A5EB_POS1_ADDR+6)  // 胎压#7-12
#define ZXDOWN_A5EB_POS3_ADDR    (ZXDOWN_A5EB_POS2_ADDR+6)  // 胎压#13-18
#define SIZE_OF_ZXDOWN_A5EB      (ZXDOWN_A5EB_POS3_ADDR+6)  // 总字节数(18B)

//==TAG-A5EC左支腿面板============================================================
#define zxdown_buffer_a5ec       (zxdown_buffer_a5eb+SIZE_OF_ZXDOWN_A5EB)// 起始地址定义
#define ZXDOWN_A5EC_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5EC_POS2_ADDR    (ZXDOWN_A5EC_POS1_ADDR+1)  // 字节状态位2
#define ZXDOWN_A5EC_POS3_ADDR    (ZXDOWN_A5EC_POS2_ADDR+1)  // 字节状态位3
#define ZXDOWN_A5EC_POS4_ADDR    (ZXDOWN_A5EC_POS3_ADDR+1)  // 字节状态位4
#define ZXDOWN_A5EC_POS5_ADDR    (ZXDOWN_A5EC_POS4_ADDR+1)  // 字节状态位5
#define ZXDOWN_A5EC_POS6_ADDR    (ZXDOWN_A5EC_POS5_ADDR+1)  // 预留
#define ZXDOWN_A5EC_POS7_ADDR    (ZXDOWN_A5EC_POS6_ADDR+1)  // DIR
#define ZXDOWN_A5EC_POS8_ADDR    (ZXDOWN_A5EC_POS7_ADDR+1)  // 调平盒心跳
#define SIZE_OF_ZXDOWN_A5EC      (ZXDOWN_A5EC_POS8_ADDR+1)  // 总字节数(8B)

//==TAG-A5ED右支腿面板============================================================
#define zxdown_buffer_a5ed       (zxdown_buffer_a5ec+SIZE_OF_ZXDOWN_A5EC)// 起始地址定义
#define ZXDOWN_A5ED_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5ED_POS2_ADDR    (ZXDOWN_A5ED_POS1_ADDR+1)  // 字节状态位2
#define ZXDOWN_A5ED_POS3_ADDR    (ZXDOWN_A5ED_POS2_ADDR+1)  // 字节状态位3
#define ZXDOWN_A5ED_POS4_ADDR    (ZXDOWN_A5ED_POS3_ADDR+1)  // 字节状态位4
#define ZXDOWN_A5ED_POS5_ADDR    (ZXDOWN_A5ED_POS4_ADDR+1)  // 字节状态位5
#define ZXDOWN_A5ED_POS6_ADDR    (ZXDOWN_A5ED_POS5_ADDR+1)  // 预留
#define ZXDOWN_A5ED_POS7_ADDR    (ZXDOWN_A5ED_POS6_ADDR+1)  // DIR
#define ZXDOWN_A5ED_POS8_ADDR    (ZXDOWN_A5ED_POS7_ADDR+1)  // 调平盒心跳
#define SIZE_OF_ZXDOWN_A5ED      (ZXDOWN_A5ED_POS8_ADDR+1)  // 总字节数(8B)

//==TAG-A5EE中控台输入信息========================================================
#define zxdown_buffer_a5ee       (zxdown_buffer_a5ed+SIZE_OF_ZXDOWN_A5ED)// 起始地址定义
#define ZXDOWN_A5EE_POS1_ADDR    0                          // 字节状态位1
#define ZXDOWN_A5EE_POS2_ADDR    (ZXDOWN_A5EE_POS1_ADDR+1)  // 字节状态位2
#define ZXDOWN_A5EE_POS3_ADDR    (ZXDOWN_A5EE_POS2_ADDR+1)  // 字节状态位3
#define ZXDOWN_A5EE_POS4_ADDR    (ZXDOWN_A5EE_POS3_ADDR+1)  // 字节状态位4
#define ZXDOWN_A5EE_POS5_ADDR    (ZXDOWN_A5EE_POS4_ADDR+1)  // 字节状态位5
#define ZXDOWN_A5EE_POS6_ADDR    (ZXDOWN_A5EE_POS5_ADDR+1)  // 字节状态位6
#define ZXDOWN_A5EE_POS7_ADDR    (ZXDOWN_A5EE_POS6_ADDR+1)  // 字节状态位7
#define ZXDOWN_A5EE_POS8_ADDR    (ZXDOWN_A5EE_POS7_ADDR+1)  // 后桥转向角度
#define ZXDOWN_A5EE_POS9_ADDR    (ZXDOWN_A5EE_POS8_ADDR+1)  // 字节状态位8
#define SIZE_OF_ZXDOWN_A5EE      (ZXDOWN_A5EE_POS9_ADDR+1)  // 总字节数(9B)

//==TAG-A5A3支腿作业信息==========================================================
#define zxdown_buffer_a5a3       (zxdown_buffer_a5ee+SIZE_OF_ZXDOWN_A5EE)// 起始地址定义
#define ZXDOWN_A5A3_POS1_ADDR    0                        // 水平支腿长度
#define ZXDOWN_A5A3_POS2_ADDR    (ZXDOWN_A5A3_POS1_ADDR+8)  // 支腿压力
#define ZXDOWN_A5A3_POS3_ADDR    (ZXDOWN_A5A3_POS2_ADDR+8)  // 摆缸长度
#define SIZE_OF_ZXDOWN_A5A3        (ZXDOWN_A5A3_POS3_ADDR+8)  // 总字节数(24B)

//==TAG-A5A4辅助支腿作业信息=======================================================
#define zxdown_buffer_a5a4       (zxdown_buffer_a5a3+SIZE_OF_ZXDOWN_A5A3)// 起始地址定义
#define ZXDOWN_A5A4_POS1_ADDR    0                        // 辅助支腿状态
#define ZXDOWN_A5A4_POS2_ADDR    (ZXDOWN_A5A4_POS1_ADDR+1)  // 左前辅助支腿压力
#define ZXDOWN_A5A4_POS3_ADDR    (ZXDOWN_A5A4_POS2_ADDR+2)  // 右前辅助支腿压力
#define SIZE_OF_ZXDOWN_A5A4      (ZXDOWN_A5A4_POS3_ADDR+2)  // 总字节数(5B)

// 下车底盘数据缓存大小
#define SIZE_OF_ZXDOWN_BUFFER   (zxdown_buffer_a5a4+SIZE_OF_ZXDOWN_A5A4-zxdown_buffer_a5e0) 

/******************************************************************************
* Macros(底盘发动机): 起重机底盘发动机数据序号和地址定义
******************************************************************************/
//==TAG-A5EF发动机运行参数(国四、国五、国六)======================================
#define zxengine_buffer_a5ef              zxengine_buffer  // 起始地址定义
#define ZXENGINE_A5EF_POS1_ADDR    0                            // 发动机转速
#define ZXENGINE_A5EF_POS2_ADDR    (ZXENGINE_A5EF_POS1_ADDR+2)  // 实际扭矩百分比
#define ZXENGINE_A5EF_POS3_ADDR    (ZXENGINE_A5EF_POS2_ADDR+1)  // 摩擦扭矩百分比
#define ZXENGINE_A5EF_POS4_ADDR    (ZXENGINE_A5EF_POS3_ADDR+1)  // 进气歧管压力
#define ZXENGINE_A5EF_POS5_ADDR    (ZXENGINE_A5EF_POS4_ADDR+1)  // 进气歧管温度
#define ZXENGINE_A5EF_POS6_ADDR    (ZXENGINE_A5EF_POS5_ADDR+1)  // 冷却液温度
#define ZXENGINE_A5EF_POS7_ADDR    (ZXENGINE_A5EF_POS6_ADDR+1)  // 机油温度
#define ZXENGINE_A5EF_POS8_ADDR    (ZXENGINE_A5EF_POS7_ADDR+2)  // 机油液位
#define ZXENGINE_A5EF_POS9_ADDR    (ZXENGINE_A5EF_POS8_ADDR+1)  // 机油压力
#define ZXENGINE_A5EF_POS10_ADDR   (ZXENGINE_A5EF_POS9_ADDR+1)  // 发动机总运行时间
#define ZXENGINE_A5EF_POS11_ADDR   (ZXENGINE_A5EF_POS10_ADDR+4)  // 油门踏板百分比
#define ZXENGINE_A5EF_POS12_ADDR   (ZXENGINE_A5EF_POS11_ADDR+1)  // 车速
#define ZXENGINE_A5EF_POS13_ADDR   (ZXENGINE_A5EF_POS12_ADDR+2)  // 字节状态位1
#define ZXENGINE_A5EF_POS14_ADDR   (ZXENGINE_A5EF_POS13_ADDR+1)  // 字节状态位2
#define ZXENGINE_A5EF_POS15_ADDR   (ZXENGINE_A5EF_POS14_ADDR+1)  // 巡航设定速度
#define ZXENGINE_A5EF_POS16_ADDR   (ZXENGINE_A5EF_POS15_ADDR+1)  // 发动机燃油消耗率
#define ZXENGINE_A5EF_POS17_ADDR   (ZXENGINE_A5EF_POS16_ADDR+2)  // 发动机平均燃油消耗率
#define ZXENGINE_A5EF_POS18_ADDR   (ZXENGINE_A5EF_POS17_ADDR+2)  // 燃油总油耗量
#define ZXENGINE_A5EF_POS19_ADDR   (ZXENGINE_A5EF_POS18_ADDR+4)  // 总行驶里程
#define ZXENGINE_A5EF_POS20_ADDR   (ZXENGINE_A5EF_POS19_ADDR+4)  // 燃油液位1
#define ZXENGINE_A5EF_POS21_ADDR   (ZXENGINE_A5EF_POS20_ADDR+1)  // 燃油液位2
#define ZXENGINE_A5EF_POS22_ADDR   (ZXENGINE_A5EF_POS21_ADDR+1)  // 字节状态位3
#define ZXENGINE_A5EF_POS23_ADDR   (ZXENGINE_A5EF_POS22_ADDR+1)  // 字节状态位4
#define SIZE_OF_ZXENGINE_A5EF      (ZXENGINE_A5EF_POS23_ADDR+1)  // 总字节数(37B)

//==TAG-A5F0行驶油耗==============================================================
#define zxengine_buffer_a5f0              (zxengine_buffer_a5ef+SIZE_OF_ZXENGINE_A5EF)// 起始地址定义
#define ZXENGINE_A5F0_POS1_ADDR    0                            // 行驶总运行时间
#define ZXENGINE_A5F0_POS2_ADDR    (ZXENGINE_A5F0_POS1_ADDR+4)  // 行驶燃油总油耗量
#define ZXENGINE_A5F0_POS3_ADDR    (ZXENGINE_A5F0_POS2_ADDR+4)  // 百公里油耗
#define SIZE_OF_ZXENGINE_A5F0      (ZXENGINE_A5F0_POS3_ADDR+4)  // 总字节数(12B)

//==TAG-A5F1 SCR参数（国五）======================================================
#define zxengine_buffer_a5f1               (zxengine_buffer_a5f0+SIZE_OF_ZXENGINE_A5F0)// 起始地址定义
#define ZXENGINE_A5F1_POS1_ADDR    0                            // 尿素喷射状态
#define ZXENGINE_A5F1_POS2_ADDR    (ZXENGINE_A5F1_POS1_ADDR+1)  // T15_DCU
#define ZXENGINE_A5F1_POS3_ADDR    (ZXENGINE_A5F1_POS2_ADDR+1)  // 尿素泵压力
#define ZXENGINE_A5F1_POS4_ADDR    (ZXENGINE_A5F1_POS3_ADDR+2)  // 尿素箱液位
#define ZXENGINE_A5F1_POS5_ADDR    (ZXENGINE_A5F1_POS4_ADDR+1)  // 尿素箱温度
#define ZXENGINE_A5F1_POS6_ADDR    (ZXENGINE_A5F1_POS5_ADDR+1)  // 尿素喷射量
#define ZXENGINE_A5F1_POS7_ADDR    (ZXENGINE_A5F1_POS6_ADDR+2)  // SCR上游NOx浓度
#define ZXENGINE_A5F1_POS8_ADDR    (ZXENGINE_A5F1_POS7_ADDR+2)  // SCR下游NOx浓度
#define ZXENGINE_A5F1_POS9_ADDR    (ZXENGINE_A5F1_POS8_ADDR+2)  // SCR上游排气温度(T6温度)
#define ZXENGINE_A5F1_POS10_ADDR   (ZXENGINE_A5F1_POS9_ADDR+2)  // SCR下游排气温度(T7温度)
#define ZXENGINE_A5F1_POS11_ADDR   (ZXENGINE_A5F1_POS10_ADDR+2)  // 尿素浓度(SPN 3516)
#define ZXENGINE_A5F1_POS12_ADDR   (ZXENGINE_A5F1_POS11_ADDR+1)  // 累计尿素消耗量
#define ZXENGINE_A5F1_POS13_ADDR   (ZXENGINE_A5F1_POS12_ADDR+4)  // 尿素品质传感器温度(SPN 3515)
#define ZXENGINE_A5F1_POS14_ADDR   (ZXENGINE_A5F1_POS13_ADDR+1)  // 品质温度传感器FMI (SPN 3519)
#define ZXENGINE_A5F1_POS15_ADDR   (ZXENGINE_A5F1_POS14_ADDR+1)  // 品质传感器FMI (SPN3520)
#define ZXENGINE_A5F1_POS16_ADDR   (ZXENGINE_A5F1_POS15_ADDR+1)  // 催化剂试剂类型(SPN3521)
#define ZXENGINE_A5F1_POS17_ADDR   (ZXENGINE_A5F1_POS16_ADDR+1)  // 尿素箱液位传感器失效模式FMI
#define ZXENGINE_A5F1_POS18_ADDR   (ZXENGINE_A5F1_POS17_ADDR+1)  // 尿素箱温度传感器失效模式FMI
#define ZXENGINE_A5F1_POS19_ADDR   (ZXENGINE_A5F1_POS18_ADDR+1)  // Nox传感器露点状态
#define SIZE_OF_ZXENGINE_A5F1      (ZXENGINE_A5F1_POS19_ADDR+1)  // 总字节数(28B)

//==TAG-A5F2 DPF参数(国六）=======================================================
#define zxengine_buffer_a5f2              (zxengine_buffer_a5f1+SIZE_OF_ZXENGINE_A5F1)// 起始地址定义
#define ZXENGINE_A5F2_POS1_ADDR    0                            // DOC上游排气温度
#define ZXENGINE_A5F2_POS2_ADDR    (ZXENGINE_A5F2_POS1_ADDR+2)  // DPF上游排气温度
#define ZXENGINE_A5F2_POS3_ADDR    (ZXENGINE_A5F2_POS2_ADDR+2)  // DPF碳载量负荷率
#define ZXENGINE_A5F2_POS4_ADDR    (ZXENGINE_A5F2_POS3_ADDR+1)  // DPF压差
#define ZXENGINE_A5F2_POS5_ADDR    (ZXENGINE_A5F2_POS4_ADDR+2)  // 字节状态位1
#define SIZE_OF_ZXENGINE_A5F2      (ZXENGINE_A5F2_POS5_ADDR+1)  // 总字节数(8B)

// 下车发动机数据缓存大小
#define SIZE_OF_ZXENGINE_BUFFER   (zxengine_buffer_a5f2+SIZE_OF_ZXENGINE_A5F2-zxengine_buffer_a5ef) 

/******************************************************************************
* Macros(统计类信息): 数据序号和地址定义
******************************************************************************/
//==TAG-A5C5 动作频次统计1========================================================
#define zxstatistics_buffer_a5c5        zxstatistics_buffer
#deinfe ZXSTATISTICS_A5C5_POS1_ADDR   0  // 二节伸
#deinfe ZXSTATISTICS_A5C5_POS2_ADDR   (ZXSTATISTICS_A5C5_POS1_ADDR+8)  // 二节缩
#deinfe ZXSTATISTICS_A5C5_POS3_ADDR   (ZXSTATISTICS_A5C5_POS2_ADDR+8)  // 多节伸
#deinfe ZXSTATISTICS_A5C5_POS4_ADDR   (ZXSTATISTICS_A5C5_POS3_ADDR+8)  // 多节缩
#deinfe ZXSTATISTICS_A5C5_POS5_ADDR   (ZXSTATISTICS_A5C5_POS4_ADDR+8)  // 变幅起
#deinfe ZXSTATISTICS_A5C5_POS6_ADDR   (ZXSTATISTICS_A5C5_POS5_ADDR+8)  // 变幅下落
#deinfe ZXSTATISTICS_A5C5_POS7_ADDR   (ZXSTATISTICS_A5C5_POS6_ADDR+8)  // 主卷起升
#deinfe ZXSTATISTICS_A5C5_POS8_ADDR   (ZXSTATISTICS_A5C5_POS7_ADDR+8)  // 主卷下落
#deinfe ZXSTATISTICS_A5C5_POS9_ADDR   (ZXSTATISTICS_A5C5_POS8_ADDR+8)  // 副卷起升
#deinfe ZXSTATISTICS_A5C5_POS10_ADDR  (ZXSTATISTICS_A5C5_POS9_ADDR+8)  // 副卷下落
#deinfe ZXSTATISTICS_A5C5_POS11_ADDR   (ZXSTATISTICS_A5C5_POS10_ADDR+8)  // 左回转
#deinfe ZXSTATISTICS_A5C5_POS12_ADDR   (ZXSTATISTICS_A5C5_POS11_ADDR+8)  // 右回转
#define SIZE_OF_ZXSTATISTICS_A5C5        (ZXSTATISTICS_A5C5_POS12_ADDR+8)  // 总字节数(96B)

//==TAG-A5C6 动作频次统计2========================================================
#define zxstatistics_buffer_a5c6        (zxstatistics_buffer_a5c5+SIZE_OF_ZXSTATISTICS_A5C5)
#deinfe ZXSTATISTICS_A5C6_POS1_ADDR   0  // 空缸伸
#deinfe ZXSTATISTICS_A5C6_POS2_ADDR   (ZXSTATISTICS_A5C6_POS1_ADDR+8)  // 空缸缩
#deinfe ZXSTATISTICS_A5C6_POS3_ADDR   (ZXSTATISTICS_A5C6_POS2_ADDR+8)  // 带臂伸
#deinfe ZXSTATISTICS_A5C6_POS4_ADDR   (ZXSTATISTICS_A5C6_POS3_ADDR+8)  // 带臂缩
#deinfe ZXSTATISTICS_A5C6_POS5_ADDR   (ZXSTATISTICS_A5C6_POS4_ADDR+8)  // 变幅起
#deinfe ZXSTATISTICS_A5C6_POS6_ADDR   (ZXSTATISTICS_A5C6_POS5_ADDR+8)  // 变幅下落
#deinfe ZXSTATISTICS_A5C6_POS7_ADDR   (ZXSTATISTICS_A5C6_POS6_ADDR+8)  // 主卷起升
#deinfe ZXSTATISTICS_A5C6_POS8_ADDR   (ZXSTATISTICS_A5C6_POS7_ADDR+8)  // 主卷下落
#deinfe ZXSTATISTICS_A5C6_POS9_ADDR   (ZXSTATISTICS_A5C6_POS8_ADDR+8)  // 副卷起升
#deinfe ZXSTATISTICS_A5C6_POS10_ADDR   (ZXSTATISTICS_A5C6_POS9_ADDR+8)  // 副卷下落
#deinfe ZXSTATISTICS_A5C6_POS11_ADDR   (ZXSTATISTICS_A5C6_POS10_ADDR+8)  // 左回转
#deinfe ZXSTATISTICS_A5C6_POS12_ADDR   (ZXSTATISTICS_A5C6_POS11_ADDR+8)  // 右回转
#define SIZE_OF_ZXSTATISTICS_A5C6        (ZXSTATISTICS_A5C6_POS12_ADDR+8)  // 总字节数(96B)

//==TAG-A5C7 安全统计=============================================================
#define zxstatistics_buffer_a5c7        (zxstatistics_buffer_a5c6+SIZE_OF_ZXSTATISTICS_A5C6)
#deinfe ZXSTATISTICS_A5C7_POS1_ADDR   0  // 超载
#deinfe ZXSTATISTICS_A5C7_POS2_ADDR   (ZXSTATISTICS_A5C7_POS1_ADDR+8)  // 三圈
#deinfe ZXSTATISTICS_A5C7_POS3_ADDR   (ZXSTATISTICS_A5C7_POS2_ADDR+8)  // 高限
#deinfe ZXSTATISTICS_A5C7_POS4_ADDR   (ZXSTATISTICS_A5C7_POS3_ADDR+8)  // 总强制
#deinfe ZXSTATISTICS_A5C7_POS5_ADDR   (ZXSTATISTICS_A5C7_POS4_ADDR+8)  // 拆装开关
#deinfe ZXSTATISTICS_A5C7_POS6_ADDR   (ZXSTATISTICS_A5C7_POS5_ADDR+8)  // 变幅起强制
#deinfe ZXSTATISTICS_A5C7_POS7_ADDR   (ZXSTATISTICS_A5C7_POS6_ADDR+8)  // 高限强制
#deinfe ZXSTATISTICS_A5C7_POS8_ADDR   (ZXSTATISTICS_A5C7_POS7_ADDR+8)  // 三圈强制
#deinfe ZXSTATISTICS_A5C7_POS9_ADDR   (ZXSTATISTICS_A5C7_POS8_ADDR+8)  // 风速超限
#define SIZE_OF_ZXSTATISTICS_A5C7       (ZXSTATISTICS_A5C7_POS9_ADDR+8)  // 总字节数(72B)

/******************************************************************************
* Macros(上车、下车版本信息): 数据序号和地址定义
******************************************************************************/
//==TAG-A505 上车系统版本=========================================================
#define zxversion_buffer_a505        zxversion_buffer
#deinfe ZXVERSION_A505_POS1_ADDR   0  // 力矩限制器
#deinfe ZXVERSION_A505_POS2_ADDR   (ZXVERSION_A505_POS1_ADDR+3)  // 显示器1
#deinfe ZXVERSION_A505_POS3_ADDR   (ZXVERSION_A505_POS2_ADDR+3)  // 显示器底层版本
#deinfe ZXVERSION_A505_POS4_ADDR   (ZXVERSION_A505_POS3_ADDR+3)  // GPS终端
#deinfe ZXVERSION_A505_POS5_ADDR   (ZXVERSION_A505_POS4_ADDR+3)  // 控制器
#deinfe ZXVERSION_A505_POS6_ADDR   (ZXVERSION_A505_POS5_ADDR+3)  // 显示器2
#define SIZE_OF_ZXVERSION_A505       (ZXVERSION_A505_POS6_ADDR+3)  // 总字节数(18B)

//==TAG-A506 下车系统版本=========================================================
#define zxversion_buffer_a506        (zxversion_buffer_a505+SIZE_OF_ZXVERSION_A505)
#deinfe ZXVERSION_A506_POS1_ADDR   0   // 显示器应用层
#deinfe ZXVERSION_A506_POS2_ADDR   (ZXVERSION_A506_POS1_ADDR+3)    // 显示器底层
#deinfe ZXVERSION_A506_POS3_ADDR   (ZXVERSION_A506_POS2_ADDR+3)    // P1应用层
#deinfe ZXVERSION_A506_POS4_ADDR   (ZXVERSION_A506_POS3_ADDR+3)    // P1底层
#deinfe ZXVERSION_A506_POS5_ADDR   (ZXVERSION_A506_POS4_ADDR+3)    // P2应用层
#deinfe ZXVERSION_A506_POS6_ADDR   (ZXVERSION_A506_POS5_ADDR+3)    // P2底层
#deinfe ZXVERSION_A506_POS7_ADDR   (ZXVERSION_A506_POS6_ADDR+3)    // P3应用层
#deinfe ZXVERSION_A506_POS8_ADDR   (ZXVERSION_A506_POS7_ADDR+3)    // P3底层
#deinfe ZXVERSION_A506_POS9_ADDR   (ZXVERSION_A506_POS8_ADDR+3)    // P4应用层
#deinfe ZXVERSION_A506_POS10_ADDR   (ZXVERSION_A506_POS9_ADDR+3)   // P4底层
#deinfe ZXVERSION_A506_POS11_ADDR   (ZXVERSION_A506_POS10_ADDR+3)  // P5应用层
#deinfe ZXVERSION_A506_POS12_ADDR   (ZXVERSION_A506_POS11_ADDR+3)  // P5底层
#deinfe ZXVERSION_A506_POS13_ADDR   (ZXVERSION_A506_POS12_ADDR+3)  // P6应用层
#deinfe ZXVERSION_A506_POS14_ADDR   (ZXVERSION_A506_POS13_ADDR+3)  // P6底层
#deinfe ZXVERSION_A506_POS15_ADDR   (ZXVERSION_A506_POS14_ADDR+3)  // P7应用层
#deinfe ZXVERSION_A506_POS16_ADDR   (ZXVERSION_A506_POS15_ADDR+3)  // P7底层
#deinfe ZXVERSION_A506_POS17_ADDR   (ZXVERSION_A506_POS16_ADDR+3)  // P8应用层
#deinfe ZXVERSION_A506_POS18_ADDR   (ZXVERSION_A506_POS17_ADDR+3)  // P8底层
#define SIZE_OF_ZXVERSION_A506       (ZXVERSION_A506_POS18_ADDR+3)  // 总字节数(54B)

/******************************************************************************
 * Data Types
 ******************************************************************************/
typedef union
{
	unsigned short word;
	struct{
		unsigned bit0 : 1;
		unsigned bit1 : 1;
		unsigned bit2 : 1;
		unsigned bit3 : 1;
		unsigned bit4 : 1;
		unsigned bit5 : 1;
		unsigned bit6 : 1;
		unsigned bit7 : 1;
		unsigned bit8 : 1;
		unsigned bit9 : 1;
		unsigned bit10 : 1;
		unsigned bit11 : 1;
		unsigned bit12 : 1;
		unsigned bit13 : 1;
		unsigned bit14 : 1;
		unsigned bit15 : 1;
	} w;
}bittype2;
// TLV有效标志位
extern bittype2 zxup_tlv_flag1;
#define tlv_a5a0_valid_flag    zxup_tlv_flag1.w.bit0
#define tlv_a5a1_valid_flag    zxup_tlv_flag1.w.bit1
#define tlv_a5a2_valid_flag    zxup_tlv_flag1.w.bit2
//#define X    zxup_tlv_flag1.w.bit3
//#define X    zxup_tlv_flag1.w.bit4
#define tlv_a5a5_valid_flag    zxup_tlv_flag1.w.bit5
#define tlv_a5a6_valid_flag    zxup_tlv_flag1.w.bit6
#define tlv_a5a7_valid_flag    zxup_tlv_flag1.w.bit7
#define tlv_a5a8_valid_flag    zxup_tlv_flag1.w.bit8
#define tlv_a5a9_valid_flag    zxup_tlv_flag1.w.bit9
#define tlv_a5aa_valid_flag    zxup_tlv_flag1.w.bit10
#define tlv_a5ab_valid_flag    zxup_tlv_flag1.w.bit11
#define tlv_a5ac_valid_flag    zxup_tlv_flag1.w.bit12
#define tlv_a5ad_valid_flag    zxup_tlv_flag1.w.bit13
#define tlv_a5ae_valid_flag    zxup_tlv_flag1.w.bit14
#define tlv_a5af_valid_flag    zxup_tlv_flag1.w.bit15

// TLV有效标志位
extern bittype2 zxup_tlv_flag2;
#define tlv_a5b0_valid_flag    zxup_tlv_flag2.w.bit0
#define tlv_a5b1_valid_flag    zxup_tlv_flag2.w.bit1
#define tlv_a5b2_valid_flag    zxup_tlv_flag2.w.bit2
#define tlv_a5b3_valid_flag    zxup_tlv_flag2.w.bit3
#define tlv_a5b4_valid_flag    zxup_tlv_flag2.w.bit4
#define tlv_a5b5_valid_flag    zxup_tlv_flag2.w.bit5
#define tlv_a5b6_valid_flag    zxup_tlv_flag2.w.bit6
#define tlv_a5b7_valid_flag    zxup_tlv_flag2.w.bit7
#define tlv_a5b8_valid_flag    zxup_tlv_flag2.w.bit8
#define tlv_a5b9_valid_flag    zxup_tlv_flag2.w.bit9
#define tlv_a5ba_valid_flag    zxup_tlv_flag2.w.bit10
#define tlv_a5bb_valid_flag    zxup_tlv_flag2.w.bit11
#define tlv_a5bc_valid_flag    zxup_tlv_flag2.w.bit12
#define tlv_a5bd_valid_flag    zxup_tlv_flag2.w.bit13
#define tlv_a5be_valid_flag    zxup_tlv_flag2.w.bit14
#define tlv_a5bf_valid_flag    zxup_tlv_flag2.w.bit15

// TLV有效标志位
extern bittype2 zxup_tlv_flag3;
#define tlv_a5c0_valid_flag    zxup_tlv_flag3.w.bit0
#define tlv_a5c1_valid_flag    zxup_tlv_flag3.w.bit1
#define tlv_a5c2_valid_flag    zxup_tlv_flag3.w.bit2
#define tlv_a5c3_valid_flag    zxup_tlv_flag3.w.bit3
#define tlv_a5c4_valid_flag    zxup_tlv_flag3.w.bit4
#define tlv_a5c5_valid_flag    zxup_tlv_flag3.w.bit5
#define tlv_a5c6_valid_flag    zxup_tlv_flag3.w.bit6
#define tlv_a5c7_valid_flag    zxup_tlv_flag3.w.bit7
#define tlv_a5c8_valid_flag    zxup_tlv_flag3.w.bit8
#define tlv_a5c9_valid_flag    zxup_tlv_flag3.w.bit9
#define tlv_a5ca_valid_flag    zxup_tlv_flag3.w.bit10
#define tlv_a5cb_valid_flag    zxup_tlv_flag3.w.bit11
#define tlv_a5cc_valid_flag    zxup_tlv_flag3.w.bit12
#define tlv_a5cd_valid_flag    zxup_tlv_flag3.w.bit13
#define tlv_a5ce_valid_flag    zxup_tlv_flag3.w.bit14
#define tlv_a5cf_valid_flag    zxup_tlv_flag3.w.bit15

// TLV有效标志位
extern bittype2 zxdown_tlv_flag1;
#define tlv_a5e0_valid_flag    zxdown_tlv_flag1.w.bit0
#define tlv_a5e1_valid_flag    zxdown_tlv_flag1.w.bit1
#define tlv_a5e2_valid_flag    zxdown_tlv_flag1.w.bit2
#define tlv_a5e3_valid_flag    zxdown_tlv_flag1.w.bit3
#define tlv_a5e4_valid_flag    zxdown_tlv_flag1.w.bit4
#define tlv_a5e5_valid_flag    zxdown_tlv_flag1.w.bit5
#define tlv_a5e6_valid_flag    zxdown_tlv_flag1.w.bit6
#define tlv_a5e7_valid_flag    zxdown_tlv_flag1.w.bit7
#define tlv_a5e8_valid_flag    zxdown_tlv_flag1.w.bit8
#define tlv_a5e9_valid_flag    zxdown_tlv_flag1.w.bit9
#define tlv_a5ea_valid_flag    zxdown_tlv_flag1.w.bit10
#define tlv_a5eb_valid_flag    zxdown_tlv_flag1.w.bit11
#define tlv_a5ec_valid_flag    zxdown_tlv_flag1.w.bit12
#define tlv_a5ed_valid_flag    zxdown_tlv_flag1.w.bit13
#define tlv_a5ee_valid_flag    zxdown_tlv_flag1.w.bit14
//#define x    zxdown_tlv_flag1.w.bit15

// TLV有效标志位
extern bittype2 zxdown_tlv_flag2;
#define tlv_a5a3_valid_flag    zxdown_tlv_flag2.w.bit0
#define tlv_a5a4_valid_flag    zxdown_tlv_flag2.w.bit1
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
#define tlv_a5f0_valid_flag    zxengine_tlv_flag.w.bit0
#define tlv_a5f1_valid_flag    zxengine_tlv_flag.w.bit1
#define tlv_a5f2_valid_flag    zxengine_tlv_flag.w.bit2
#define tlv_a5f3_valid_flag    zxengine_tlv_flag.w.bit3
#define tlv_a5f4_valid_flag    zxengine_tlv_flag.w.bit4
#define tlv_a5f5_valid_flag    zxengine_tlv_flag.w.bit5
#define tlv_a5f6_valid_flag    zxengine_tlv_flag.w.bit6
#define tlv_a5f7_valid_flag    zxengine_tlv_flag.w.bit7
#define tlv_a5f8_valid_flag    zxengine_tlv_flag.w.bit8
#define tlv_a5f9_valid_flag    zxengine_tlv_flag.w.bit9
#define tlv_a5fa_valid_flag    zxengine_tlv_flag.w.bit10
#define tlv_a5fb_valid_flag    zxengine_tlv_flag.w.bit11
#define tlv_a5fc_valid_flag    zxengine_tlv_flag.w.bit12
#define tlv_a5fd_valid_flag    zxengine_tlv_flag.w.bit13
#define tlv_a5fe_valid_flag    zxengine_tlv_flag.w.bit14
#define tlv_a5ef_valid_flag    zxengine_tlv_flag.w.bit15  // 注意这个位

/******************************************************************************
 * Function prototypes
 ******************************************************************************/



#endif /* TCW_H_ */
