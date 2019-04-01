/*
*********************************************************************************************************
*
*    模块名称 : FreeRTOS多任务实现
*    文件名称 : Leo_FreeRTOS_TASK
*    版    本 : V1.0
*    说    明 : 项目中所有任务的建立
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-19     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/



#ifndef LEO_FREERTOS_FOOTBLE_H
#define LEO_FREERTOS_FOOTBLE_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * 全局变量初始化函数   待完善  
*/
void vINIT_Variable(void);    
/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*     成功 返回0；失败 返回1.                                            */    
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void);    
    
    
    
#ifdef __cplusplus
}
#endif


#endif 