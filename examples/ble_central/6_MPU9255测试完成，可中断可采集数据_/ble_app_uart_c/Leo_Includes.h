/*
*********************************************************************************************************
*
*    模块名称 : 头文件及宏定义汇总
*    文件名称 : Leo_Includes.h
*    版    本 : V1.0
*    说    明 : 各类头文件 及 宏定义
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-14     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef  LEO_INCLUDES_H
#define  LEO_INCLUDES_H

/*
*********************************************************************************************************
*                                       头文件引用 区
*********************************************************************************************************
*/

/*----------------------------------------通用头文件----------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------nRF专用头文件--------------------------------------------------*/
/* nRF通用 */
#include "nordic_common.h"
#include "bsp.h"                        //QF开发板

/* nRF驱动_延迟 */
#include "nrf_delay.h"

/* nRF驱动_nrf_sdh SoftDevice Handler */
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

/* nRF驱动_计时器 管脚 */
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

/* nRF驱动_电源管理相关 */
#include "nrf_pwr_mgmt.h"

/* nRF驱动_BLE服务相关 */
#include "nrf_ble_gatt.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "ble_db_discovery.h"

/* nRF应用_module模块 */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"

/* nRF应用_日志输出相关 */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/*-------------------------------------FreeRTOS 系统相关------------------------------------------------*/                                  
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"





/*
*********************************************************************************************************
*                                       宏定义_参数设置
*********************************************************************************************************
*/
/*--------------------------------------- 测试相关 Debug -----------------------------------------------*/
#define Leo_Debug                                   1               //1:运行一些debug代码; 0:不运行




/*---------------------------------------TIMER计时器相关-------------------------------------------------
 * Your application cannot use RTC0 and TIMER0 if you are using BLE
 * Your application cannot use RTC1 if you are using FreeRTOS.
 * 系统默认设定的 TIMER 中断权限为 7
 */

/* TIMER0_参数设定 */


/* TIMER1_参数设定 */


/* TIMER2_参数设定 */


/* TIMER3_参数设定 
 * 用于1ms的计时，提供标准的时间基准，并通过gps 1pps 对齐
 * 注意：TICK的单位是ms                                   */
#define  configTIMER3_ENABLE                        1 
#define  configTIMER3_INSTANCE                      3
#define  configTIMER3_TICK                          1                //ms

/* TIMER4_参数设定 
 * 用于FreeRTOS任务分析统计使用                           */
#define  configTIMER4_ENABLE                        1 
#define  configTIMER4_INSTANCE                      4
#define  configTIMER4_TICK                          50              //us


/*--------------------------------------- 寄存器等常值 -------------------------------------------------*/
#define configRegister_MPU_SPIReadBit 		        0x80		        //SPI读取数据时，对地址添加 (目前在MPU9255中使用)




/*
*********************************************************************************************************
*                                       宏定义_管脚配置GPIO
*********************************************************************************************************
*/
/*--------------------------------------- GPIO管脚 选型 -------------------------------------------------*/

#define configGPIO_Set_QFBoard                      1                  //青风开发板
#define configGPIO_Set_DVPFirst                     0                  //自己第一版研制的 使用的 UWB1000
#define configGPIO_Set_DVPSecond                    0                  //自己第二版研制的 使用的 UWB1001(包含蓝牙)
#define configGPIO_Set_DWM1001Dev                   0                  //DWB1001 Dev 开发板

#if ((configGPIO_Set_QFBoard + configGPIO_Set_DVPFirst + configGPIO_Set_DVPSecond + configGPIO_Set_DWM1001Dev) != 1)
    #error "The configGPIO_Set is Wrong!"
#endif


/*-------------------------------- QFBoard 青风开发板 ---------------------------------------------------*/
#if configGPIO_Set_QFBoard

/* INT___外部中断引脚设定 */
#define configGPIO_INT_SDCard                       25                  /*用于触发SDCard 停止存储 按钮
                                                                         *以及 FreeRTOS 任务分析 中断输出  */
#define configGPIO_INT_MPU9255                      26                  /*MPU9255 数据采集中断

/* LED___设备LED显示管脚设定 */
#define configGPIO_LED_Red                          LED_1               /* 红色LED灯  */
#define configGPIO_LED_Green                        LED_1               /* 绿色LED灯  */




/* SPI_1    MPU9255  */
#define configGPIO_SPI_MPU_INSTANCE                 1                   //使用 SPI 的实例1
#define configGPIO_SPI_MPU_CS				        13				    //连接MPU9255的——>NCS
#define configGPIO_SPI_MPU_SCK				        14				    //连接MPU9255的——>SCL
#define configGPIO_SPI_MPU_MOSI				        15				    //连接MPU9255的——>SDA
#define configGPIO_SPI_MPU_MISO 				    16				    //连接MPU9255的——>ADO  


#endif

/*-------------------------------- DVPFirst 自己第一版 ---------------------------------------------------*/
#if configGPIO_Set_DVPFirst




#endif

/*-------------------------------- DVPSecond 自己第二版 --------------------------------------------------*/
#if configGPIO_Set_DVPSecond




#endif

/*---------------------------------- DWM1001Dev  开发板 --------------------------------------------------*/
#if configGPIO_Set_DWM1001Dev

 
/* INT___外部中断引脚设定                                    */
#define configGPIO_INT_SDCard               2               /*用于触发SDCard 停止存储 按钮
                                                             *以及 FreeRTOS 任务分析 中断输出  */
#define configGPIO_INT_MPU9255              0               /*MPU9255 数据采集中断             */



#endif












#endif 


