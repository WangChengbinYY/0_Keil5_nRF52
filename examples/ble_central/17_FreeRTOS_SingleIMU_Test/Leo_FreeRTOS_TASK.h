/*
*********************************************************************************************************
*
*    ģ������ : FreeRTOS������ʵ��
*    �ļ����� : Leo_FreeRTOS_TASK
*    ��    �� : V1.0
*    ˵    �� : ��Ŀ����������Ľ���
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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
 * ȫ�ֱ�����ʼ������   ������  
*/
void vINIT_Variable(void);    
/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*     �ɹ� ����0��ʧ�� ����1.                                            */    
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void);    
    
    
    
#ifdef __cplusplus
}
#endif


#endif 