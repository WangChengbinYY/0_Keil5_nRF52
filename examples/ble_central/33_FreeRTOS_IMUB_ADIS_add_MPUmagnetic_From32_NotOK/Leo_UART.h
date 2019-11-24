/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ��������
*    �ļ����� : Leo_UART
*    ��    �� : V1.0
*    ˵    �� : �ⲿ�����������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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

///* GPS���ڳ�ʼ��
// *    ���� 0 �ɹ������� 1 ʧ��  */    
//uint8_t ucUART_GPS_Initial(void);


///* GPS���ڽ����ַ� */   
//uint8_t ucUART_GPS_RX(void);
    
/* GPS���ڳ�ʼ��
 *    ���� 0 �ɹ������� 1 ʧ��  */    
uint8_t ucUARTInital_GPS(void);
    
/**
 * GPS UTCʱ��ת ������  */
void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek);    
    
#ifdef __cplusplus
}
#endif


#endif 
