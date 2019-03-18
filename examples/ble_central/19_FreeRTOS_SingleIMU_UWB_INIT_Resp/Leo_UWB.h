/*
*********************************************************************************************************
*
*    模块名称 : UWB 测距
*    文件名称 : Leo_UWB
*    版    本 : V1.0
*    说    明 : UWB 测距实现
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-12     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/



#ifndef LEO_UWB_H
#define LEO_UWB_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * UWB    初始化   */
uint8_t ucSS_INIT_Initial(void);
    
/**
 * 发起者  启动测距   */
void vSS_INIT_Start(void);  
    
/**
 * 接收反馈 并处理   */
uint8_t ucSS_INIT_Handler(uint16* pDistance,uint8_t* pNumber);    
    
//    
///**
// * 发起者  启动测距   */
//uint8_t ucSS_INIT_RUN(uint16* pDistance,uint8_t* pNumber);
//    
///**
// * 响应者  反馈测距   */
//uint8_t ucSS_RESP_RUN(void);

    
    

#ifdef __cplusplus
}
#endif


#endif 