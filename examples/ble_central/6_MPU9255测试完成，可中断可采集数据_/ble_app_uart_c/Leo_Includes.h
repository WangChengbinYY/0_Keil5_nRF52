/*
*********************************************************************************************************
*
*    ģ������ : ͷ�ļ����궨�����
*    �ļ����� : Leo_Includes.h
*    ��    �� : V1.0
*    ˵    �� : ����ͷ�ļ� �� �궨��
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-14     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef  LEO_INCLUDES_H
#define  LEO_INCLUDES_H

/*
*********************************************************************************************************
*                                       ͷ�ļ����� ��
*********************************************************************************************************
*/

/*----------------------------------------ͨ��ͷ�ļ�----------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------nRFר��ͷ�ļ�--------------------------------------------------*/
/* nRFͨ�� */
#include "nordic_common.h"
#include "bsp.h"                        //QF������

/* nRF����_�ӳ� */
#include "nrf_delay.h"

/* nRF����_nrf_sdh SoftDevice Handler */
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

/* nRF����_��ʱ�� �ܽ� */
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

/* nRF����_��Դ������� */
#include "nrf_pwr_mgmt.h"

/* nRF����_BLE������� */
#include "nrf_ble_gatt.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "ble_db_discovery.h"

/* nRFӦ��_moduleģ�� */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"

/* nRFӦ��_��־������ */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/*-------------------------------------FreeRTOS ϵͳ���------------------------------------------------*/                                  
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"





/*
*********************************************************************************************************
*                                       �궨��_��������
*********************************************************************************************************
*/
/*--------------------------------------- ������� Debug -----------------------------------------------*/
#define Leo_Debug                                   1               //1:����һЩdebug����; 0:������




/*---------------------------------------TIMER��ʱ�����-------------------------------------------------
 * Your application cannot use RTC0 and TIMER0 if you are using BLE
 * Your application cannot use RTC1 if you are using FreeRTOS.
 * ϵͳĬ���趨�� TIMER �ж�Ȩ��Ϊ 7
 */

/* TIMER0_�����趨 */


/* TIMER1_�����趨 */


/* TIMER2_�����趨 */


/* TIMER3_�����趨 
 * ����1ms�ļ�ʱ���ṩ��׼��ʱ���׼����ͨ��gps 1pps ����
 * ע�⣺TICK�ĵ�λ��ms                                   */
#define  configTIMER3_ENABLE                        1 
#define  configTIMER3_INSTANCE                      3
#define  configTIMER3_TICK                          1                //ms

/* TIMER4_�����趨 
 * ����FreeRTOS�������ͳ��ʹ��                           */
#define  configTIMER4_ENABLE                        1 
#define  configTIMER4_INSTANCE                      4
#define  configTIMER4_TICK                          50              //us


/*--------------------------------------- �Ĵ����ȳ�ֵ -------------------------------------------------*/
#define configRegister_MPU_SPIReadBit 		        0x80		        //SPI��ȡ����ʱ���Ե�ַ��� (Ŀǰ��MPU9255��ʹ��)




/*
*********************************************************************************************************
*                                       �궨��_�ܽ�����GPIO
*********************************************************************************************************
*/
/*--------------------------------------- GPIO�ܽ� ѡ�� -------------------------------------------------*/

#define configGPIO_Set_QFBoard                      1                  //��翪����
#define configGPIO_Set_DVPFirst                     0                  //�Լ���һ�����Ƶ� ʹ�õ� UWB1000
#define configGPIO_Set_DVPSecond                    0                  //�Լ��ڶ������Ƶ� ʹ�õ� UWB1001(��������)
#define configGPIO_Set_DWM1001Dev                   0                  //DWB1001 Dev ������

#if ((configGPIO_Set_QFBoard + configGPIO_Set_DVPFirst + configGPIO_Set_DVPSecond + configGPIO_Set_DWM1001Dev) != 1)
    #error "The configGPIO_Set is Wrong!"
#endif


/*-------------------------------- QFBoard ��翪���� ---------------------------------------------------*/
#if configGPIO_Set_QFBoard

/* INT___�ⲿ�ж������趨 */
#define configGPIO_INT_SDCard                       25                  /*���ڴ���SDCard ֹͣ�洢 ��ť
                                                                         *�Լ� FreeRTOS ������� �ж����  */
#define configGPIO_INT_MPU9255                      26                  /*MPU9255 ���ݲɼ��ж�

/* LED___�豸LED��ʾ�ܽ��趨 */
#define configGPIO_LED_Red                          LED_1               /* ��ɫLED��  */
#define configGPIO_LED_Green                        LED_1               /* ��ɫLED��  */




/* SPI_1    MPU9255  */
#define configGPIO_SPI_MPU_INSTANCE                 1                   //ʹ�� SPI ��ʵ��1
#define configGPIO_SPI_MPU_CS				        13				    //����MPU9255�ġ���>NCS
#define configGPIO_SPI_MPU_SCK				        14				    //����MPU9255�ġ���>SCL
#define configGPIO_SPI_MPU_MOSI				        15				    //����MPU9255�ġ���>SDA
#define configGPIO_SPI_MPU_MISO 				    16				    //����MPU9255�ġ���>ADO  


#endif

/*-------------------------------- DVPFirst �Լ���һ�� ---------------------------------------------------*/
#if configGPIO_Set_DVPFirst




#endif

/*-------------------------------- DVPSecond �Լ��ڶ��� --------------------------------------------------*/
#if configGPIO_Set_DVPSecond




#endif

/*---------------------------------- DWM1001Dev  ������ --------------------------------------------------*/
#if configGPIO_Set_DWM1001Dev

 
/* INT___�ⲿ�ж������趨                                    */
#define configGPIO_INT_SDCard               2               /*���ڴ���SDCard ֹͣ�洢 ��ť
                                                             *�Լ� FreeRTOS ������� �ж����  */
#define configGPIO_INT_MPU9255              0               /*MPU9255 ���ݲɼ��ж�             */



#endif












#endif 


