/*
*********************************************************************************************************
*
*    模块名称 : 外部 AD 数据的采集
*    文件名称 : Leo_SAADC
*    版    本 : V1.0
*    说    明 : 外部 AD采集
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-10     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef LEO_SAADC_H
#define LEO_SAADC_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif

    
    
/**
 * ADC 初始化
 *   0 成功； 1 失败
*/
uint8_t ucSAADCInitial(void);    
    
    
    


#ifdef __cplusplus
}
#endif


#endif 
