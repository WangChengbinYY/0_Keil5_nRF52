/*
*********************************************************************************************************
*
*    ģ������ : ��ʱ��
*    �ļ����� : Leo_TIMER
*    ��    �� : V1.0
*    ˵    �� : ��ʱ��ʵ�����
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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
    





#if configTIMER4_ENABLE    

/* TIMER4 ��������ʼ��*/ 
void vTimeStart_FreeRTOSTaskTest(void);

/* TIMER4 ����������*/ 
void vTimerStart_4(void);    
    
    
#endif    
    
    
#ifdef __cplusplus
}
#endif


#endif     
    
    
    
    
    
    