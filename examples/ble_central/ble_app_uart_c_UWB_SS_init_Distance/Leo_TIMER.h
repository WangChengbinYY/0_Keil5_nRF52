/*
*********************************************************************************************************
*
*    模块名称 : 计时器
*    文件名称 : Leo_TIMER
*    版    本 : V1.0
*    说    明 : 计时器实现相关
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-17     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef LEO_TIMER_H
#define LEO_TIMER_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif
    


#if configTIMER3_ENABLE    
    /* TIMER3 计数器初始化*/ 
    uint8_t ucTimerInitial_3(void);
    /* TIMER3 计数器启动*/ 
    uint8_t ucTimerStart_3(void);
#endif 


#if configTIMER4_ENABLE    
    /* TIMER4 计数器初始化*/ 
    uint8_t ucTimerInitial_4(void);
    /* TIMER4 计数器启动*/ 
    uint8_t ucTimerStart_4(void);
#endif    
    
    
#ifdef __cplusplus
}
#endif


#endif     
    
    
    
    
    
    