/*
*********************************************************************************************************
*
*    ģ������ : �ⲿӲ���ж�����
*    �ļ����� : Leo_INT
*    ��    �� : V1.0
*    ˵    �� : �ⲿӲ���ж��������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-14     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef LEO_INT_H
#define LEO_INT_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif

/* SDCard �洢��ͣ�ж� ��ʼ��  */  
uint8_t ucINTInital_SDCard(void); 
/* SDCard �洢��ͣ�ж� ����  */
uint8_t ucINTStart_SDCard(void);  
    

/* 1pps �жϳ�ʼ�� */
uint8_t ucINTInital_PPS(void);
/* 1pps �ж� ���� */
uint8_t ucINTStart_PPS(void);    
    
    
/* UWB�жϳ�ʼ�� */
uint8_t ucINTInital_UWB(void);
/* UWB�ж� ����  */
uint8_t ucINTStart_UWB(void);
    
    
/* IMUA�жϳ�ʼ�� */
uint8_t ucINTInital_IMUA(void);
/* IMUA�ж� ����  */
uint8_t ucINTStart_IMUA(void);    
    
    
/* IMUB�жϳ�ʼ�� */
uint8_t ucINTInital_IMUB(void);
/* IMUB�ж� ����  */
uint8_t ucINTStart_IMUB(void);    

#ifdef __cplusplus
}
#endif


#endif 







