/*
*********************************************************************************************************
*
*    模块名称 : 外部传感器 IMU(MPU9255)
*    文件名称 : Leo_IMU_MPU92
*    版    本 : V1.0
*    说    明 : 外部传感器 IMU(MPU9255)
*
*    修改记录 :
*        版本号    日期          作者     
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
    


/*=========================================== IMU初始化 ============================================*/

/**
 * IMU 初始化
 *   采用 管脚共享方式，初始化 IMU_A 和 IMU_B
*/
uint8_t ucIMU_INIT_MPU_ADIS(void);

///**
// * IMU 初始化 ADIS
// *   采用 管脚共享方式，初始化 IMU_A 和 IMU_B
//*/
//uint8_t ucIMU_ADIS_Initial(void);

uint8_t Leo_ADIS_Read_ALLData(uint8_t * Data,uint8_t Length);


	
#ifdef __cplusplus
}
#endif


#endif  /* LEO_IMU_H.h */    