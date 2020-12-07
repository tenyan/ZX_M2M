/*
 * Copyright(c)2019, 硬件研发部
 * All right reserved
 *
 * 文件名称: Watchdog.h
 * 版本号  : V1.0
 * 文件标识: 
 * 文件描述: 此文件为喂狗声明文件
 * 
 *--------------------------------------------------------------
 * 修改记录
 *--------------------------------------------------------------
 *
 * 2019-03-24, by lxf, 创建本文件
 *
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H


int32 Watchdog_Init(void);
void Watchdog_Feed(void);
void Watchdog_Close(void);

#endif







