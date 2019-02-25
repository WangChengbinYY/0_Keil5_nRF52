
/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_uart     
 * 平台    ：nRF52832
 * 描述    ：串口定义及实现  
 * 作者    ：王成宾
**********************************************************************/

#include <stdint.h>
#include "app_uart.h"

#ifndef LEO_UART_H
#define LEO_UART_H

#ifdef __cplusplus
extern "C" {
#endif
	
	
//---------------------------------正式------------------------------------------------------
#define Leo_UART_RX_PIN_NUMBER 15
#define Leo_UART_TX_PIN_NUMBER 16

	
/**********************************************************************************************
* 描  述 : 	Leo_UART_GPS_Init函数
*          	初始化和 GPS 模块连接的串口
* 入  参 :	获取到数据字节的个数
* 返回值 : 
* 注意	:		禁止流控，波特率：9600，因此只设置了 2个管脚 RX 15 TX16 注意模块上 输入 输出的方向
***********************************************************************************************/ 
void Leo_UART_GPS_Init(void);		
	
	

	
	
	
	
	
	
	
	
	
//---------------------------------测试------------------------------------------------------	
	
	

	
//(用于测试)板载串口初始化。禁止流控，波特率：115200
void Leo_UART_BSP_Init(void);
	

	
	
	

/**********************************************************************************************
* 描  述 : 	Leo_UART_GetData函数
*          	获取接收到的串口数据
* 入  参 :	pBuffer_Get 			存放数据数组的指针
						mBuffer_L	数组的大小
						mGetNumber					获取到数据字节的个数
* 返回值 : * 返回值 : 0 表示没有收到数据，1 表示收到数据
* 注意	:	这里需要触发外部中断，来调用此函数，否则 串口缓存区会溢出！！！
***********************************************************************************************/ 
uint32_t Leo_UART_GetData(uint8_t * pBuffer_Get,uint16_t mBuffer_L,uint16_t * mGetNumber);	
	

/**********************************************************************************************
* 描  述 : 	Leo_UART_SendData函数
*          	往串口发送数据
* 入  参 :	pBuffer_Send 			所要发送数据的数组指针
						mBuffer_L	数组的大小
* 返回值 : err_code 0 表示发送成功！
***********************************************************************************************/ 
uint32_t Leo_UART_SendData(uint8_t * pBuffer_Send,uint16_t mBuffer_L);	



	
#ifdef __cplusplus
}
#endif


#endif  /* Leo_uart.h */

