	
/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_nRF52_config     
 * 平台    ：nRF52832
 * 描述    ：针对项目应用的所有管脚定义 和 相应的全局宏定义  
 * 作者    ：王成宾
**********************************************************************/

#ifndef LEO_NRF52_CONFIG_H
#define LEO_NRF52_CONFIG_H


#include <stdint.h>
#include <stdio.h>

#include "sdk_config.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h" 


#ifdef __cplusplus
extern "C" {
#endif
	
/********************************管脚定义区*******************************************************/	

#define LEO_nRF52_BOARDSETTING          //发板的管教定义     否则  使用自定义管脚
    
    
    
#ifdef LEO_nRF52_BOARDSETTING
    
/**@brief nRF52<—SPI—>MPU9255 管脚定义
<*--------------------------------------------------------------------------*>
<*目前考虑无法和SDCard的SPI共用，两个分别定义，后面可以试验改进
<*试验阶段，考虑开发板上LED灯使用的管脚有4个,
<*#define BUTTON_1       13
<*#define BUTTON_2       14
<*#define BUTTON_3       15
<*#define BUTTON_4       16			
<*切记需要去更改“nrf_drv_config.h”中的管脚定义，否则无效  如：SPI1_CONFIG_SCK_PIN	
<*--------------------------------------------------------------------------*>*/
#define Leo_nRF52_MPU9255_SPI						1				//使用 SPI 的实例1
#define Leo_nRF52_MPU9255_SPI_CS_PIN				13				//连接MPU9255的——>NCS
#define Leo_nRF52_MPU9255_SPI_SCK_PIN				14				//连接MPU9255的——>SCL
#define Leo_nRF52_MPU9255_SPI_MOSI_PIN				15				//连接MPU9255的——>SDA
#define Leo_nRF52_MPU9255_SPI_MISO_PIN 				16				//连接MPU9255的——>ADO  
	
#define Leo_INT_MPU9255						26				//连接MPU9255的——>INT  中断

//@brief nRF52<—SPI—>SDCard  管脚定义
/*--------------------------------------------------------------------------*/
//<*	直接采用实验板上默认的管脚
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_SDCard_SPI						0				//使用 SPI 的实例0
#define Leo_nRF52_SDCard_SPI_CS_PIN					29				//连接SDCard的——> SD_CS
#define Leo_nRF52_SDCard_SPI_SCK_PIN				3				//连接SDCard的——> 5管脚
#define Leo_nRF52_SDCard_SPI_MOSI_PIN				2				//连接SDCard的——> DIN
#define Leo_nRF52_SDCard_SPI_MISO_PIN 				28				//连接SDCard的——> DC

#define Leo_INT_SDCard						        25				//关闭存储文件的外部——>INT  中断

/**@brief nRF52<—UART—>GPS模块 串口数据接收管脚定义
<*--------------------------------------------------------------------------*>
<*采用开发板目前已设计好的串口管脚	
<*--------------------------------------------------------------------------*>*/
//#define Leo_nRF52_GPS_UART_TXD                    6               //接GPS的RXD   //板载原来的管脚，实验的时候，如法接收，可能是和后面的芯片有影响
//#define Leo_nRF52_GPS_UART_RXD                    8               //接GPS的TXD   //暂时改成下面的
#define Leo_nRF52_GPS_UART_RXD                      22              //接GPS的TXD
#define Leo_nRF52_GPS_UART_TXD                      23              //接GPS的RXD
#define Leo_nRF52_GPS_UART_CTS                      5               //接GPS的CTS
#define Leo_nRF52_GPS_UART_RTS                      7               //接GPS的RTS
#define Leo_nRF52_GPS_UART_PPS                      24              //接GPS的RTS

#define LEO_GPS_BUFFER_MAXLENGTH                    512             //GPS数据存放缓存区字节大小

//@brief nRF52 设计两个LED等，一个红色 一个绿色    修改"nrf_gpio.h"里的 #define LEDS_NUMBER    4
/*--------------------------------------------------------------------------*/
//<*	板子刚上电                                   红(灭)	[有电没电可以看GPS模块的灯是否亮]
//<*	传感器初始化完成,正常接收数据                 红(亮)	
//<*	开始存储采集数据                             红(闪)
//<*	停止存储采集数据,只正常接收                   红(亮)
//<*	GPS定位有效,能正常接收解析                    绿(亮)
//<*	GPS开机没有定位 或 间隔5秒没有接收到数据       绿(灭)
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_LED_RED							17				//红灯管脚
#define Leo_nRF52_LED_GREEN							18				//绿灯管脚

/********************************宏定义区*******************************************************/	
#define Leo_nRF52_MPU9255_SPI_READ_BIT 		        0x80		//SPI读取数据时，对地址添加 (目前在MPU9255中使用)


#else
    
/**@brief nRF52<—SPI—>MPU9255 管脚定义*/
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_MPU9255_SPI						1				//使用 SPI 的实例1
#define Leo_nRF52_MPU9255_SPI_CS_PIN				15				//连接MPU9255的——>NCS
#define Leo_nRF52_MPU9255_SPI_SCK_PIN				12    //9				//连接MPU9255的——>SCL
#define Leo_nRF52_MPU9255_SPI_MOSI_PIN				11				//连接MPU9255的——>SDA
#define Leo_nRF52_MPU9255_SPI_MISO_PIN 				13				//连接MPU9255的——>ADO  
	
#define Leo_INT_MPU9255						        14				//连接MPU9255的——>INT  中断


//@brief nRF52<—SPI—>SDCard  管脚定义
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_SDCard_SPI						0				//使用 SPI 的实例0
#define Leo_nRF52_SDCard_SPI_CS_PIN					1				//连接SDCard的——> SD_CS
#define Leo_nRF52_SDCard_SPI_SCK_PIN				5				//连接SDCard的——> 5管脚
#define Leo_nRF52_SDCard_SPI_MOSI_PIN				3				//连接SDCard的——> DIN
#define Leo_nRF52_SDCard_SPI_MISO_PIN 				7				//连接SDCard的——> DC

#define Leo_INT_SDCard						        8				//关闭存储文件的外部——>INT  中断

/**@brief nRF52<—UART—>GPS模块 串口数据接收管脚定义*/
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_GPS_UART_RXD                      19              //接GPS的TXD
#define Leo_nRF52_GPS_UART_TXD                      23              //接GPS的RXD
#define Leo_nRF52_GPS_UART_CTS                      4               //接GPS的CTS      //没用的管脚
#define Leo_nRF52_GPS_UART_RTS                      6               //接GPS的RTS      //没用的管脚
#define Leo_nRF52_GPS_UART_PPS                      17              //接GPS的RTS

#define LEO_GPS_BUFFER_MAXLENGTH                    512             //GPS数据存放缓存区字节大小

//@brief nRF52 设计两个LED等，一个红色 一个绿色    修改"nrf_gpio.h"里的 #define LEDS_NUMBER    4
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_LED_RED							4				//红灯管脚
#define Leo_nRF52_LED_GREEN							6				//绿灯管脚

/********************************宏定义区*******************************************************/	
#define Leo_nRF52_MPU9255_SPI_READ_BIT 		        0x80		//SPI读取数据时，对地址添加 (目前在MPU9255中使用)


#endif


		
#ifdef __cplusplus
}
#endif


#endif  /* Leo_nRF52_config.h */		
