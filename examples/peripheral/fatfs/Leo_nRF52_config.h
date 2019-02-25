	
/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_nRF52_config     
 * ƽ̨    ��nRF52832
 * ����    �������ĿӦ�õ����йܽŶ��� �� ��Ӧ��ȫ�ֺ궨��  
 * ����    �����ɱ�
**********************************************************************/

#ifndef LEO_NRF52_CONFIG_H
#define LEO_NRF52_CONFIG_H


#include <stdint.h>
#include <stdio.h>

#include "sdk_config.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h" 


#ifdef __cplusplus
extern "C" {
#endif
	
/********************************�ܽŶ�����*******************************************************/	

#define LEO_nRF52_BOARDSETTING          //����Ĺ̶ܽ���     ����  ʹ���Զ���ܽ�
    
    
    
#ifdef LEO_nRF52_BOARDSETTING
    
/**@brief nRF52<��SPI��>MPU9255 �ܽŶ���
<*--------------------------------------------------------------------------*>
<*Ŀǰ�����޷���SDCard��SPI���ã������ֱ��壬�����������Ľ�
<*����׶Σ����ǿ�������LED��ʹ�õĹܽ���4��,
<*#define BUTTON_1       13
<*#define BUTTON_2       14
<*#define BUTTON_3       15
<*#define BUTTON_4       16			
<*�м���Ҫȥ���ġ�nrf_drv_config.h���еĹܽŶ��壬������Ч  �磺SPI1_CONFIG_SCK_PIN	
<*--------------------------------------------------------------------------*>*/
#define Leo_nRF52_MPU9255_SPI						1				//ʹ�� SPI ��ʵ��1
#define Leo_nRF52_MPU9255_SPI_CS_PIN				13				//����MPU9255�ġ���>NCS
#define Leo_nRF52_MPU9255_SPI_SCK_PIN				14				//����MPU9255�ġ���>SCL
#define Leo_nRF52_MPU9255_SPI_MOSI_PIN				15				//����MPU9255�ġ���>SDA
#define Leo_nRF52_MPU9255_SPI_MISO_PIN 				16				//����MPU9255�ġ���>ADO  
	
#define Leo_INT_MPU9255						26				//����MPU9255�ġ���>INT  �ж�

//@brief nRF52<��SPI��>SDCard  �ܽŶ���
/*--------------------------------------------------------------------------*/
//<*	ֱ�Ӳ���ʵ�����Ĭ�ϵĹܽ�
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_SDCard_SPI						0				//ʹ�� SPI ��ʵ��0
#define Leo_nRF52_SDCard_SPI_CS_PIN					29				//����SDCard�ġ���> SD_CS
#define Leo_nRF52_SDCard_SPI_SCK_PIN				3				//����SDCard�ġ���> 5�ܽ�
#define Leo_nRF52_SDCard_SPI_MOSI_PIN				2				//����SDCard�ġ���> DIN
#define Leo_nRF52_SDCard_SPI_MISO_PIN 				28				//����SDCard�ġ���> DC

#define Leo_INT_SDCard						        25				//�رմ洢�ļ����ⲿ����>INT  �ж�

/**@brief nRF52<��UART��>GPSģ�� �������ݽ��չܽŶ���
<*--------------------------------------------------------------------------*>
<*���ÿ�����Ŀǰ����ƺõĴ��ڹܽ�	
<*--------------------------------------------------------------------------*>*/
//#define Leo_nRF52_GPS_UART_TXD                    6               //��GPS��RXD   //����ԭ���Ĺܽţ�ʵ���ʱ���編���գ������Ǻͺ����оƬ��Ӱ��
//#define Leo_nRF52_GPS_UART_RXD                    8               //��GPS��TXD   //��ʱ�ĳ������
#define Leo_nRF52_GPS_UART_RXD                      22              //��GPS��TXD
#define Leo_nRF52_GPS_UART_TXD                      23              //��GPS��RXD
#define Leo_nRF52_GPS_UART_CTS                      5               //��GPS��CTS
#define Leo_nRF52_GPS_UART_RTS                      7               //��GPS��RTS
#define Leo_nRF52_GPS_UART_PPS                      24              //��GPS��RTS

#define LEO_GPS_BUFFER_MAXLENGTH                    512             //GPS���ݴ�Ż������ֽڴ�С

//@brief nRF52 �������LED�ȣ�һ����ɫ һ����ɫ    �޸�"nrf_gpio.h"��� #define LEDS_NUMBER    4
/*--------------------------------------------------------------------------*/
//<*	���Ӹ��ϵ�                                   ��(��)	[�е�û����Կ�GPSģ��ĵ��Ƿ���]
//<*	��������ʼ�����,������������                 ��(��)	
//<*	��ʼ�洢�ɼ�����                             ��(��)
//<*	ֹͣ�洢�ɼ�����,ֻ��������                   ��(��)
//<*	GPS��λ��Ч,���������ս���                    ��(��)
//<*	GPS����û�ж�λ �� ���5��û�н��յ�����       ��(��)
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_LED_RED							17				//��ƹܽ�
#define Leo_nRF52_LED_GREEN							18				//�̵ƹܽ�

/********************************�궨����*******************************************************/	
#define Leo_nRF52_MPU9255_SPI_READ_BIT 		        0x80		//SPI��ȡ����ʱ���Ե�ַ��� (Ŀǰ��MPU9255��ʹ��)


#else
    
/**@brief nRF52<��SPI��>MPU9255 �ܽŶ���*/
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_MPU9255_SPI						1				//ʹ�� SPI ��ʵ��1
#define Leo_nRF52_MPU9255_SPI_CS_PIN				15				//����MPU9255�ġ���>NCS
#define Leo_nRF52_MPU9255_SPI_SCK_PIN				12    //9				//����MPU9255�ġ���>SCL
#define Leo_nRF52_MPU9255_SPI_MOSI_PIN				11				//����MPU9255�ġ���>SDA
#define Leo_nRF52_MPU9255_SPI_MISO_PIN 				13				//����MPU9255�ġ���>ADO  
	
#define Leo_INT_MPU9255						        14				//����MPU9255�ġ���>INT  �ж�


//@brief nRF52<��SPI��>SDCard  �ܽŶ���
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_SDCard_SPI						0				//ʹ�� SPI ��ʵ��0
#define Leo_nRF52_SDCard_SPI_CS_PIN					1				//����SDCard�ġ���> SD_CS
#define Leo_nRF52_SDCard_SPI_SCK_PIN				5				//����SDCard�ġ���> 5�ܽ�
#define Leo_nRF52_SDCard_SPI_MOSI_PIN				3				//����SDCard�ġ���> DIN
#define Leo_nRF52_SDCard_SPI_MISO_PIN 				7				//����SDCard�ġ���> DC

#define Leo_INT_SDCard						        8				//�رմ洢�ļ����ⲿ����>INT  �ж�

/**@brief nRF52<��UART��>GPSģ�� �������ݽ��չܽŶ���*/
//<*--------------------------------------------------------------------------*>
#define Leo_nRF52_GPS_UART_RXD                      19              //��GPS��TXD
#define Leo_nRF52_GPS_UART_TXD                      23              //��GPS��RXD
#define Leo_nRF52_GPS_UART_CTS                      4               //��GPS��CTS      //û�õĹܽ�
#define Leo_nRF52_GPS_UART_RTS                      6               //��GPS��RTS      //û�õĹܽ�
#define Leo_nRF52_GPS_UART_PPS                      17              //��GPS��RTS

#define LEO_GPS_BUFFER_MAXLENGTH                    512             //GPS���ݴ�Ż������ֽڴ�С

//@brief nRF52 �������LED�ȣ�һ����ɫ һ����ɫ    �޸�"nrf_gpio.h"��� #define LEDS_NUMBER    4
/*--------------------------------------------------------------------------*/
#define Leo_nRF52_LED_RED							4				//��ƹܽ�
#define Leo_nRF52_LED_GREEN							6				//�̵ƹܽ�

/********************************�궨����*******************************************************/	
#define Leo_nRF52_MPU9255_SPI_READ_BIT 		        0x80		//SPI��ȡ����ʱ���Ե�ַ��� (Ŀǰ��MPU9255��ʹ��)


#endif


		
#ifdef __cplusplus
}
#endif


#endif  /* Leo_nRF52_config.h */		
