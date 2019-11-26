/*
*********************************************************************************************************
*
*    模块名称 : 外部串口配置
*    文件名称 : Leo_UART
*    版    本 : V1.0
*    说    明 : 外部串口设置相关
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-09     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef LEO_UART_H
#define LEO_UART_H

#include "Leo_Includes.h"

#ifdef __cplusplus
extern "C" {
#endif

///* GPS串口初始化
// *    返回 0 成功；返回 1 失败  */    
//uint8_t ucUART_GPS_Initial(void);


///* GPS串口接收字符 */   
//uint8_t ucUART_GPS_RX(void);
    
/* GPS串口初始化
 *    返回 0 成功；返回 1 失败  */    
uint8_t ucUARTInital_GPS(void);
    
/**
 * GPS UTC时间转 周内秒  */
void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek);    
    
#ifdef __cplusplus
}
#endif


#endif 
