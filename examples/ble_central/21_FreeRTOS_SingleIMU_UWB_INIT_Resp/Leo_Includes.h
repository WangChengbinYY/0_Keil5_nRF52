/*
*********************************************************************************************************
*
*    ģ������ : ͷ�ļ����궨�����
*    �ļ����� :Leo_Includes.h
*    ��    �� : V1.0
*    ˵    �� : ����ͷ�ļ� �� �궨��
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-14     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef  LEO_INCLUDES_H
#define  LEO_INCLUDES_H

/*======================== ͷ�ļ����� ================================*/
/**
 * ͨ��ͷ�ļ�
*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * nRF�������
*/
#include "nordic_common.h"
//#include "bsp.h"                      
#include "nrf_delay.h"

/* nRF����_SoftDevice Handler*/
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

/* nRF����_��ʱ�� �ܽ� */
#include "nrfx_timer.h"
#include "nrfx_gpiote.h"

/* nRF����_SAADC */
#include "nrfx_saadc.h"

/* nRF����_Uart */
//#include "nrfx_uart.h"

/* nRF����_��Դ������� */
//#include "nrf_pwr_mgmt.h"

/* nRF����_BLE������� */
//#include "nrf_ble_gatt.h"
//#include "ble.h"
//#include "ble_gap.h"
//#include "ble_hci.h"
//#include "ble_advdata.h"
//#include "ble_nus_c.h"
//#include "ble_db_discovery.h"

/* nRF����_SPI��� */
#include "nrf_drv_spi.h"

/**
 * nRFӦ��_moduleģ�� */
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"

/**
 * nRFӦ��_��־������ */
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/**
 * FreeRTOS ϵͳ��� */                             
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/**
 * FatFs �ļ�������� */
#include "ff.h"
#include "diskio_blkdev.h"
#include "nrf_block_dev_sdc.h"

/**
 * GPS������� */
#include "minmea.h"

/**
 * UWB ������ */
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"






//������
/*===================================== �궨��_�������� =================================================*/

/*------------------------- �Ĵ������ -------------------------*/
#define configRegister_MPU_SPIReadBit 		        0x80		        //SPI��ȡ����ʱ���Ե�ַ��� (Ŀǰ��MPU9255��ʹ��)
/* SDCard �洢buffer��� */
#define configBuffer_SDCard_Max                     1024//8500
#define configBuffer_SDCard_Save                    512//8192



/*=================================== �궨��_�ܽ�����GPIO ===============================================*/
/**
 * GPIO�ܽ� ѡ��
*/
#define configGPIO_Set_QFBoard                      0                  //��翪����
#define configGPIO_Set_DWM1001Dev                   0                  //DWB1001 Dev ������

#define configGPIO_Set_nRF52                        0                  //�Լ���һ�����Ƶ� ʹ�õ� UWB1000
#define configGPIO_Set_UWB1001                      1                  //�Լ��ڶ������Ƶ� ʹ�õ� UWB1001(��������)

#if ((configGPIO_Set_QFBoard + configGPIO_Set_nRF52 + configGPIO_Set_UWB1001 + configGPIO_Set_DWM1001Dev) != 1)
    #error "The configGPIO_Set is Wrong!"
#endif

/**
 * QFBoard ��翪����
*/
#if configGPIO_Set_QFBoard

/* INT___�ⲿ�ж������趨 -----------------------------*/
#define configGPIO_INT_SDCard                       25                  /*���ڴ���SDCard ֹͣ�洢 ��ť
                                                                         *�Լ� FreeRTOS ������� �ж����  */
#define configGPIO_INT_MPU9255                      26                  /*MPU9255 ���ݲɼ��ж�     */
#define configGPIO_INT_GPSPPS                       24                  /*GPS 1PPS�ж�     */

/* LED___�豸LED��ʾ�ܽ��趨 ---------------------------*/
#define configGPIO_LED_R                            17                  /* ��ɫLED��  */

/* SPI_0    SDCard  -----------------------------------*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE
                                                                        /* ��Ӧ SPI �� 0�� Instance
                                                                         * ���Ҫ�޸�SDCardʹ�õ�ʵ������Ҫ�޸�"sdk_config"�����APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_CS					29				    //����SDCard�ġ���> SD_CS
#define configGPIO_SPI_SDCard_SCK				    3				    //����SDCard�ġ���> 5�ܽ�
#define configGPIO_SPI_SDCard_MOSI				    2				    //����SDCard�ġ���> DIN
#define configGPIO_SPI_SDCard_MISO 				    28				    //����SDCard�ġ���> DC


/* SPI_1    MPU9255  ---------------------------------*/
#define configGPIO_SPI_CollectData_INSTANCE         2                   //ʹ�� SPI ��ʵ��1
#define configGPIO_SPI_CollectData_SCK				14				    //����MPU9255�ġ���>SCL
#define configGPIO_SPI_CollectData_MOSI				15				    //����MPU9255�ġ���>SDA
#define configGPIO_SPI_CollectData_MISO 			16				    //����MPU9255�ġ���>ADO  

#define configGPIO_SPI_IMUA_nCS				        13				    //���ӵ�һ��MPU9255�ġ���>NCS
#define configGPIO_SPI_IMUB_nCS 				        18				    //���ӵڶ���MPU9255�ġ���>ADO 

/* Uart  ���� ----------------------------------------*/
#define configGPIO_UART_GPS_RXD                      22              //��GPS��TXD
#define configGPIO_UART_GPS_TXD                      23              //��GPS��RXD
#define configGPIO_UART_GPS_CTS                      5               //��GPS��CTS
#define configGPIO_UART_GPS_RTS                      7               //��GPS��RTS


#endif


/**
 * DWM1001Dev  ������
*/
#if configGPIO_Set_DWM1001Dev

 
/* INT___�ⲿ�ж������趨                                    */
#define configGPIO_INT_SDCard               2               /*���ڴ���SDCard ֹͣ�洢 ��ť
                                                             *�Լ� FreeRTOS ������� �ж����  */
#define configGPIO_INT_MPU9255              0               /*MPU9255 ���ݲɼ��ж�             */



#endif




/**
 * �Լ���һ�� nRF52
*/
#if configGPIO_Set_nRF52

/* INT___�ⲿ�ж������趨 -----------------------------*/
#define configGPIO_INT_SDCard                       8                   /*���ڴ���SDCard ֹͣ�洢 ��ť
                                                                         *�Լ� FreeRTOS ������� �ж����  */
#define configGPIO_INT_MPU9255                      14                  /*MPU9255 ���ݲɼ��ж�     */
#define configGPIO_INT_GPSPPS                       17                  /*GPS 1PPS�ж�     */

/* LED___�豸LED��ʾ�ܽ��趨 ---------------------------*/
#define configGPIO_LED_R                            4                  /* ��ɫLED��  */
#define configGPIO_LED_G                            6                  /* ��ɫLED��  */

/* SPI_0    SDCard  -----------------------------------*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE
                                                                        /* ��Ӧ SPI �� 0�� Instance
                                                                         * ���Ҫ�޸�SDCardʹ�õ�ʵ������Ҫ�޸�"sdk_config"�����APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_CS					1				    //����SDCard�ġ���> SD_CS
#define configGPIO_SPI_SDCard_SCK				    5				    //����SDCard�ġ���> 5�ܽ�
#define configGPIO_SPI_SDCard_MOSI				    3				    //����SDCard�ġ���> DIN
#define configGPIO_SPI_SDCard_MISO 				    7				    //����SDCard�ġ���> DC


/* SPI_1    MPU9255  ---------------------------------*/
#define configGPIO_SPI_CollectData_INSTANCE                 1                   //ʹ�� SPI ��ʵ��1
#define configGPIO_SPI_MPU_CS				        15				    //����MPU9255�ġ���>NCS
#define configGPIO_SPI_CollectData_SCK				        12				    //����MPU9255�ġ���>SCL
#define configGPIO_SPI_CollectData_MOSI				        11				    //����MPU9255�ġ���>SDA
#define configGPIO_SPI_CollectData_MISO 				    13				    //����MPU9255�ġ���>ADO  


/* Uart  ���� ----------------------------------------*/
#define configGPIO_UART_GPS_RXD                      19              //��GPS��TXD
#define configGPIO_UART_GPS_TXD                      23              //��GPS��RXD
#define configGPIO_UART_GPS_CTS                      4               //��GPS��CTS
#define configGPIO_UART_GPS_RTS                      6               //��GPS��RTS


#endif


/**
 * �Լ��ڶ��� UWB1001
*/
#if configGPIO_Set_UWB1001

/* INT___�ⲿ�ж������趨 */
#define configGPIO_INT_SDCard                       2                   /*Button_SDCard ���ڴ���SDCard ֹͣ�洢 ��ť,�Լ� FreeRTOS ������� �ж����  */
//#define configGPIO_INT_MPU9255                    26                  /*MPU9255 ���ݲɼ��ж�     */
#define configGPIO_INT_GPSPPS                       26                  /*GPS 1PPS�ж�     */
#define configGPIO_INT_UWB                          19                  //UWB �ж�

/* LED___�豸LED��ʾ�ܽ��趨 */
#define configGPIO_LED_R                            23                  /* ��ɫLED��  */

/* SPI_1    SDCard  */
/*      0 ��UWB1001ʹ�ã������� 1 */
/*      ���Ҫ�޸�SDCardʹ�õ�ʵ������Ҫ�޸�"sdk_config"����� APP_SDCARD_SPI_INSTANCE*/
#define configGPIO_SPI_SDCard_INSTANCE		        APP_SDCARD_SPI_INSTANCE                                                                         
#define configGPIO_SPI_SDCard_CS					12				    //����SDCard�ġ���> SD_CS
#define configGPIO_SPI_SDCard_SCK				    22				    //����SDCard�ġ���> 5�ܽ�
#define configGPIO_SPI_SDCard_MOSI				    14				    //����SDCard�ġ���> DIN
#define configGPIO_SPI_SDCard_MISO 				    27				    //����SDCard�ġ���> DC

/* SPI_2    IMU  */
#define configGPIO_SPI_CollectData_INSTANCE         2                   //ʹ�� SPI ��ʵ��2
#define configGPIO_SPI_CollectData_SCK				8				    //����MPU9255�ġ���>SCL
#define configGPIO_SPI_CollectData_MOSI				7				    //����MPU9255�ġ���>SDA
#define configGPIO_SPI_CollectData_MISO 			6				    //����MPU9255�ġ���>ADO  

#define configGPIO_SPI_IMUA_nCS				        15				    //IMU_A(U4) ��Ƭѡ
#define configGPIO_SPI_IMUB_nCS 				    13				    //IMU_B(U5) ��Ƭѡ
#define configGPIO_SPI_UWB_nCS 				        17				    //UWB SPI ��Ƭѡ

/* Uart  ����  */
#define configGPIO_UART_GPS_RXD                      11                 //��GPS��TXD
#define configGPIO_UART_GPS_TXD                      5                  //��GPS��RXD
//#define configGPIO_UART_GPS_CTS                    5                //��GPS��CTS
//#define configGPIO_UART_GPS_RTS                    7                //��GPS��RTS


/* UWB ����  1�������ߣ�0����Ӧ��*/
#define configUWB_INIT                               1
/* UWB ����  �����߽��մ��� Mask ��ʶ*/
#define configUWB_INIT_SYSMASK_ALL_RX_ERR   (DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RFTO | DWT_INT_RXPTO) 
/* UWB ����  �����߽��մ��� Status ��ʶ ���������жϱ�־*/
#define configUWB_INIT_SYSSTATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO )


/* UWB ����  ��Ӧ�߽��մ��� Mask ��ʶ*/
#define configUWB_RESP_SYSMASK_ALL_RX_ERR   (DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT )
/* UWB ����  ��Ӧ�߽��մ��� Status ��ʶ ���������жϱ�־*/
#define configUWB_RESP_SYSSTATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_RXRFTO | SYS_STATUS_RXPTO )






#endif























#endif 


