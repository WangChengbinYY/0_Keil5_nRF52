/*
*********************************************************************************************************
*
*    模块名称 : 头文件及宏定义汇总
*    文件名称 :Leo_Includes.h
*    版    本 : V1.0
*    说    明 : 各类头文件 及 宏定义
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-14     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef  LEO_INCLUDES_H
#define  LEO_INCLUDES_H

/*======================== 头文件引用 ================================*/
/**
 * 通用头文件
*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * nRF驱动相关
*/
#include "nordic_common.h"
//#include "bsp.h"                      
#include "nrf_delay.h"

/* nRF驱动_SoftDevice Handler*/
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

/* nRF驱动_计时器 管脚 */
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

/* nRF驱动_SAADC */
#include "nrfx_saadc.h"

/* nRF驱动_Uart */
//#include "nrfx_uart.h"

/* nRF驱动_电源管理相关 */
//#include "nrf_pwr_mgmt.h"

/* nRF驱动_BLE服务相关 */
//#include "nrf_ble_gatt.h"
//#include "ble.h"
//#include "ble_gap.h"
//#include "ble_hci.h"
//#include "ble_advdata.h"
//#include "ble_nus_c.h"
//#include "ble_db_discovery.h"

/* nRF驱动_SPI相关 */
#include "nrf_drv_spi.h"

/**
 * nRF应用_module模块 */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"

/**
 * nRF应用_日志输出相关 */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * FreeRTOS 系统相关 */                             
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/**
 * FatFs 文件操作相关 */
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

/**
 * GPS解码相关 */
#include "minmea.h"

/**
 * UWB 测距相关 */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"






//待整理
/*===================================== 宏定义_参数设置 =================================================*/

/*------------------------- 寄存器相关 -------------------------*/
#define configRegister_MPU_SPIReadBit 		        0x80		        //SPI读取数据时，对地址添加 (目前在MPU9255中使用)
/* SDCard 存储buffer相关 */
#define configBuffer_SDCard_Max                     1024//8500
#define configBuffer_SDCard_Save                    512//8192



/*=================================== 宏定义_管脚配置GPIO ===============================================*/
/**
 * GPIO管脚 选型
*/
#define configGPIO_Set_QFBoard                      0                  //青风开发板
#define configGPIO_Set_DWM1001Dev                   0                  //DWB1001 Dev 开发板

#define configGPIO_Set_nRF52                        0                  //自己第一版研制的 使用的 UWB1000
#define configGPIO_Set_UWB1001                      1                  //自己第二版研制的 使用的 UWB1001(包含蓝牙)

#if ((configGPIO_Set_QFBoard + configGPIO_Set_nRF52 + configGPIO_Set_UWB1001 + configGPIO_Set_DWM1001Dev) != 1)
    #error "The configGPIO_Set is Wrong!"
#endif

/**
 * QFBoard 青风开发板
*/
#if configGPIO_Set_QFBoard

/* INT___外部中断引脚设定 -----------------------------*/
#define configGPIO_INT_SDCard                       25                  /*用于触发SDCard 停止存储 按钮
                                                                         *以及 FreeRTOS 任务分析 中断输出  */
#define configGPIO_INT_MPU9255                      26                  /*MPU9255 数据采集中断     */
#define configGPIO_INT_GPSPPS                       24                  /*GPS 1PPS中断     */

/* LED___设备LED显示管脚设定 ---------------------------*/
#define configGPIO_LED_R                            17                  /* 红色LED灯  */

/* SPI_0    SDCard  -----------------------------------*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE
                                                                        /* 对应 SPI 的 0号 Instance
                                                                         * 如果要修改SDCard使用的实例，需要修改"sdk_config"里面的APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_CS					29				    //连接SDCard的——> SD_CS
#define configGPIO_SPI_SDCard_SCK				    3				    //连接SDCard的——> 5管脚
#define configGPIO_SPI_SDCard_MOSI				    2				    //连接SDCard的——> DIN
#define configGPIO_SPI_SDCard_MISO 				    28				    //连接SDCard的——> DC


/* SPI_1    MPU9255  ---------------------------------*/
#define configGPIO_SPI_CollectData_INSTANCE         2                   //使用 SPI 的实例1
#define configGPIO_SPI_CollectData_SCK				14				    //连接MPU9255的——>SCL
#define configGPIO_SPI_CollectData_MOSI				15				    //连接MPU9255的——>SDA
#define configGPIO_SPI_CollectData_MISO 			16				    //连接MPU9255的——>ADO  

#define configGPIO_SPI_IMUA_nCS				        13				    //连接第一个MPU9255的——>NCS
#define configGPIO_SPI_IMUB_nCS 				        18				    //连接第二个MPU9255的——>ADO 

/* Uart  串口 ----------------------------------------*/
#define configGPIO_UART_GPS_RXD                      22              //接GPS的TXD
#define configGPIO_UART_GPS_TXD                      23              //接GPS的RXD
#define configGPIO_UART_GPS_CTS                      5               //接GPS的CTS
#define configGPIO_UART_GPS_RTS                      7               //接GPS的RTS


#endif


/**
 * DWM1001Dev  开发板
*/
#if configGPIO_Set_DWM1001Dev

 
/* INT___外部中断引脚设定                                    */
#define configGPIO_INT_SDCard               2               /*用于触发SDCard 停止存储 按钮
                                                             *以及 FreeRTOS 任务分析 中断输出  */
#define configGPIO_INT_MPU9255              0               /*MPU9255 数据采集中断             */



#endif




/**
 * 自己第一版 nRF52
*/
#if configGPIO_Set_nRF52

/* INT___外部中断引脚设定 -----------------------------*/
#define configGPIO_INT_SDCard                       8                   /*用于触发SDCard 停止存储 按钮
                                                                         *以及 FreeRTOS 任务分析 中断输出  */
#define configGPIO_INT_MPU9255                      14                  /*MPU9255 数据采集中断     */
#define configGPIO_INT_GPSPPS                       17                  /*GPS 1PPS中断     */

/* LED___设备LED显示管脚设定 ---------------------------*/
#define configGPIO_LED_R                            4                  /* 红色LED灯  */
#define configGPIO_LED_G                            6                  /* 绿色LED灯  */

/* SPI_0    SDCard  -----------------------------------*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE
                                                                        /* 对应 SPI 的 0号 Instance
                                                                         * 如果要修改SDCard使用的实例，需要修改"sdk_config"里面的APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_CS					1				    //连接SDCard的——> SD_CS
#define configGPIO_SPI_SDCard_SCK				    5				    //连接SDCard的——> 5管脚
#define configGPIO_SPI_SDCard_MOSI				    3				    //连接SDCard的——> DIN
#define configGPIO_SPI_SDCard_MISO 				    7				    //连接SDCard的——> DC


/* SPI_1    MPU9255  ---------------------------------*/
#define configGPIO_SPI_CollectData_INSTANCE                 1                   //使用 SPI 的实例1
#define configGPIO_SPI_MPU_CS				        15				    //连接MPU9255的——>NCS
#define configGPIO_SPI_CollectData_SCK				        12				    //连接MPU9255的——>SCL
#define configGPIO_SPI_CollectData_MOSI				        11				    //连接MPU9255的——>SDA
#define configGPIO_SPI_CollectData_MISO 				    13				    //连接MPU9255的——>ADO  


/* Uart  串口 ----------------------------------------*/
#define configGPIO_UART_GPS_RXD                      19              //接GPS的TXD
#define configGPIO_UART_GPS_TXD                      23              //接GPS的RXD
#define configGPIO_UART_GPS_CTS                      4               //接GPS的CTS
#define configGPIO_UART_GPS_RTS                      6               //接GPS的RTS


#endif


/**
 * 自己第二版 UWB1001
*/
#if configGPIO_Set_UWB1001

/* INT___外部中断引脚设定 */
#define configGPIO_INT_SDCard                       2                   /*Button_SDCard 用于触发SDCard 停止存储 按钮,以及 FreeRTOS 任务分析 中断输出  */
//#define configGPIO_INT_MPU9255                    26                  /*MPU9255 数据采集中断     */
#define configGPIO_INT_GPSPPS                       26                  /*GPS 1PPS中断     */
#define configGPIO_INT_UWB                          19                  //UWB 中断

/* LED___设备LED显示管脚设定 */
#define configGPIO_LED_R                            23                  /* 红色LED灯  */

/* SPI_1    SDCard  */
/*      0 给UWB1001使用，这里用 1 */
/*      如果要修改SDCard使用的实例，需要修改"sdk_config"里面的 APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE                                                                         
#define configGPIO_SPI_SDCard_CS					12				    //连接SDCard的——> SD_CS
#define configGPIO_SPI_SDCard_SCK				    22				    //连接SDCard的——> 5管脚
#define configGPIO_SPI_SDCard_MOSI				    14				    //连接SDCard的——> DIN
#define configGPIO_SPI_SDCard_MISO 				    27				    //连接SDCard的——> DC

/* SPI_2    IMU  */
#define configGPIO_SPI_CollectData_INSTANCE         2                   //使用 SPI 的实例2
#define configGPIO_SPI_CollectData_SCK				8				    //连接MPU9255的——>SCL
#define configGPIO_SPI_CollectData_MOSI				7				    //连接MPU9255的——>SDA
#define configGPIO_SPI_CollectData_MISO 			6				    //连接MPU9255的——>ADO  

#define configGPIO_SPI_IMUA_nCS				        15				    //IMU_A(U4) 的片选
#define configGPIO_SPI_IMUB_nCS 				    13				    //IMU_B(U5) 的片选
#define configGPIO_SPI_UWB_nCS 				        17				    //UWB SPI 的片选

/* Uart  串口  */
#define configGPIO_UART_GPS_RXD                      11                 //接GPS的TXD
#define configGPIO_UART_GPS_TXD                      5                  //接GPS的RXD
//#define configGPIO_UART_GPS_CTS                    5                //接GPS的CTS
//#define configGPIO_UART_GPS_RTS                    7                //接GPS的RTS


/* UWB 配置  1：发起者；0：响应者*/
#define configUWB_INIT                               1
/* UWB 配置  发起者接收错误 Mask 标识*/
#define configUWB_INIT_SYSMASK_ALL_RX_ERR   (DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RFTO | DWT_INT_RXPTO) 
/* UWB 配置  发起者接收错误 Status 标识 用于重置中断标志*/
#define configUWB_INIT_SYSSTATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO )


/* UWB 配置  响应者接收错误 Mask 标识*/
#define configUWB_RESP_SYSMASK_ALL_RX_ERR   (DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT )
/* UWB 配置  响应者接收错误 Status 标识 用于重置中断标志*/
#define configUWB_RESP_SYSSTATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO )






#endif























#endif 


