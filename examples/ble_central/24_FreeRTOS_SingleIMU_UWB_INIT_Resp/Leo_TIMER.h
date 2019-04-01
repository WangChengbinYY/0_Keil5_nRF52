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
  
/* TIMER3 ��������ʼ��*/ 
uint8_t ucTimerInitial_3(void);
/* TIMER3 ����������*/ 
uint8_t ucTimerStart_3(void);

/* TIMER4 ��������ʼ��*/ 
uint8_t ucTimerInitial_4(void);
/* TIMER4 ����������*/ 
uint8_t ucTimerStart_4(void);
     
    
#ifdef __cplusplus
}
#endif


#endif     
    
    
    
    
    
    