/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ������ IMU(MPU9255)
*    �ļ����� : Leo_IMU_MPU92
*    ��    �� : V1.0
*    ˵    �� : �ⲿ������ IMU(MPU9255)
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-10     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/



#ifndef LEO_IMU_MPU92_H
#define LEO_IMU_MPU92_H

#include "Leo_Includes.h"

#ifdef __cplusplus
extern "C" {
#endif
    


/*=========================================== IMU��ʼ�� ============================================*/

/**
 * IMU ��ʼ��
 *   ���� �ܽŹ���ʽ����ʼ�� IMU_A �� IMU_B
*/
uint8_t ucIMU_INIT_MPU_ADIS(void);

///**
// * IMU ��ʼ�� ADIS
// *   ���� �ܽŹ���ʽ����ʼ�� IMU_A �� IMU_B
//*/
//uint8_t ucIMU_ADIS_Initial(void);

uint8_t Leo_ADIS_Read_ALLData(uint8_t * Data,uint8_t Length);


	
#ifdef __cplusplus
}
#endif


#endif  /* LEO_IMU_H.h */    