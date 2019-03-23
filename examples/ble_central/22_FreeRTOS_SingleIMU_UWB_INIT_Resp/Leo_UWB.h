/*
*********************************************************************************************************
*
*    ģ������ : UWB ���
*    �ļ����� : Leo_UWB
*    ��    �� : V1.0
*    ˵    �� : UWB ���ʵ��
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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
 * UWB    ��ʼ��   */
uint8_t ucSS_INIT_Initial(void);
    
/**
 * ������  �������   */
void vSS_INIT_Start(void);  
    
/**
 * ���շ��� ������   */
uint8_t ucSS_INIT_Handler(uint16* pDistance,uint8_t* pNumber);    
    
//    
///**
// * ������  �������   */
//uint8_t ucSS_INIT_RUN(uint16* pDistance,uint8_t* pNumber);
//    
///**
// * ��Ӧ��  �������   */
//uint8_t ucSS_RESP_RUN(void);

    
    

#ifdef __cplusplus
}
#endif


#endif 