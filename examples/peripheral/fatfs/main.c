/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

/**@brief ͷ�ļ�����
<*--------------------------------------------------------------------------*>*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h" 

//==========���йܽ� �� ȫ�����ò��� �궨��  ���й�ͬ��Ҫ��ͷ�ļ���
#include "Leo_nRF52_config.h"			

//==========������趨��
#include "Leo_BatteryCheck.h"       //ADC �ɼ����
#include "Leo_nRF52_GPS.h"	        //GPSģ�� ����
#include "Leo_int.h"	            //�ж�    GPIOTE
#include "Leo_nRF52_SDCard.h"	    //SDCard�洢ʵ��
#include "Leo_mpu9255.h"	        //MPU9255���������				
#include "Leo_nRF52_Time.h"		    //�Զ��� Timer��ʱ�����	

//BLE �������
//#include "ble.h"
//#include "ble_gap.h"
//#include "ble_hci.h"
//#include "nrf_sdh_soc.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_ble.h"
//#include "ble_advdata.h"
//#include "nrf_ble_gatt.h"

#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"



/**@brief ȫ�ֱ���������
<*--------------------------------------------------------------------------*>*/

//===========�������ݴ洢����=========================
//AK8963��ǿ�ƶ����������������������� 2~4
uint8_t		G_MPU9255_MAG_ASAXYZ[7];
uint8_t		G_MPU9255_MAG_ASAXYZ_IsValid;
//MPU9255��������ŵ�����
uint8_t		G_MPU9255_Data[32];
uint8_t		G_MPU9255_Data_IsValid;
uint32_t	G_MPU9255_Counter;  //MPU9255�жϴ����ļ����������ں���ͳ���Ƿ񶪰� ��Ӧ��G_MPU9255_Data[32] ��8~11λ
//GPS��������
uint8_t     G_GPS_Data[35];
uint8_t     G_GPS_Data_RMCIsValid;
uint8_t     G_GPS_Data_GGAIsValid;
//�㲿ѹ������������
uint8_t     G_FOOTPressure_Data[25];
uint8_t     G_FOOTPressure_Data_IsValid;


//===========ʱ����ش洢����=========================
//GPS����������
uint32_t    G_GPSWeekSecond;
//nRF52ʱ����������Ƶ� 1s��1000����ֵ���� �ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
uint16_t    G_MicroSecond;


//===========ϵͳ������ز���=========================

//SDCard �洢�ļ��Ƿ��Ѵ򿪣��Ƿ������洢  0 û��  1 ����
uint8_t     G_SDCard_IsSaved;
//�ɼ��������ݳ���ļ�¼
uint8_t     G_WRONG_Record[11];



/**@brief ���幦��ʵ����
<*--------------------------------------------------------------------------*>*/


static void Leo_nRF52_Globale_Initial(void)
{
//===========�������ݴ洢������ʼ��=========================    
    //AK8963��ǿ�ƶ������������������� 2~4
    //uint8_t		G_MPU9255_MAG_ASAXYZ[7] = {0};
    memset(G_MPU9255_MAG_ASAXYZ,0,7);
	G_MPU9255_MAG_ASAXYZ[0] = 0xC1;
	G_MPU9255_MAG_ASAXYZ[1] = 0xC2;
    G_MPU9255_MAG_ASAXYZ[5] = 0xF1;
    G_MPU9255_MAG_ASAXYZ[6] = 0xF2;    
    G_MPU9255_MAG_ASAXYZ_IsValid = 0;
    //MPU9255��������ŵ�����
    //uint8_t		G_MPU9255_Data[32] = {0};
    memset(G_MPU9255_Data,0,32);
    G_MPU9255_Data[0] = 0xA1;
	G_MPU9255_Data[1] = 0xA2;
    G_MPU9255_Data[30] = 0xF1;
    G_MPU9255_Data[31] = 0xF2;
    G_MPU9255_Data_IsValid = 0;
    G_MPU9255_Counter = 0;  //MPU9255�жϴ����ļ����������ں���ͳ���Ƿ񶪰� ��Ӧ��G_MPU9255_Data[32] ��8~11λ
    //GPS��������
    //uint8_t     G_GPS_Data[35] = {0};
    memset(G_GPS_Data,0,35);
    G_GPS_Data[0] = 0xA3;
    G_GPS_Data[1] = 0xA4;
    G_GPS_Data[33] = 0xF1;
    G_GPS_Data[34] = 0xF2;
    G_GPS_Data_RMCIsValid = 0;
    G_GPS_Data_GGAIsValid = 0;
    //�㲿ѹ������������
    //uint8_t     G_FOOTPressure_Data[25] = {0};
    memset(G_FOOTPressure_Data,0,25);
    G_FOOTPressure_Data[0] = 0xA5;
	G_FOOTPressure_Data[1] = 0xA6;
    G_FOOTPressure_Data[23] = 0xF1;
    G_FOOTPressure_Data[24] = 0xF2;
    G_FOOTPressure_Data_IsValid = 0;    
    
    
    
//===========ʱ����ش洢������ʼ��=========================   
    G_GPSWeekSecond = 0;
    G_MicroSecond = 0;



//===========ϵͳ������ز�����ʼ��=========================
    G_SDCard_IsSaved = 0;
 
    //�ɼ��������ݳ���ļ�¼ 
    memset(G_WRONG_Record,0,11);
    G_WRONG_Record[0] = 0xC3;
	G_WRONG_Record[1] = 0xC4;
    G_WRONG_Record[9] = 0xF1;
    G_WRONG_Record[10] = 0xF2;
    
}




uint32_t    gtNum = 0;


/*-------------------------------------------------    
 *                  test SDCardд���ٶ�   
 *--------------------------------------------------*/  
static void Leo_TIME2_Event_Handler(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE2)
    {
        gtNum++;
    }

}

const nrfx_timer_t Leo_TIMER2 = NRFX_TIMER_INSTANCE(2); 





/**
 * @brief Function for main application entry.
 */
int main(void)
{
    
    
/*-------------------------------------------------    
 *                  test SDCardд���ٶ�   
 *--------------------------------------------------*/   
    uint8_t error_code = 0;
    bsp_board_init(BSP_INIT_LEDS);
//�� log �� RTT���ģʽ        
    error_code |= NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();    
	NRF_LOG_INFO("nRF52 LOG Initialization is DONE!");     
    NRF_LOG_FLUSH();    
    
    //��ʼ�� GPIOTE �ܽŹ���
//	error_code |= nrfx_gpiote_init();
//    APP_ERROR_CHECK(error_code);
//	NRF_LOG_INFO("nRF52 GPIOTE Initialization is DONE!");
//    NRF_LOG_FLUSH();
    
    
    NRF_LOG_INFO("TIME0 Initialization is Begin....!");
    error_code |= Leo_TIME1_Initial();
    Leo_TIME1_Begin();      //������ʱ��
    NRF_LOG_INFO("TIME0 Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);
    
     while (true)
    {
        __WFE();
    }
    
    
    
    //��ʼ�� SDCard 
//	NRF_LOG_INFO("nRF52 SDCard Initialization is Begin....!");
//	error_code |= Leo_nRF52_SDCard_Initial();	
//	NRF_LOG_INFO("nRF52 SDCard Initialization is DONE(err_code is 0x%x)",error_code);
//	nrf_delay_ms(300);		
//    //���� SDCard��д��
//    uint8_t MyName[] = "Leo nRF52 is begining!.";    
//	error_code |= Leo_nRF52_SDCard_SaveData(MyName,sizeof(MyName));	
//    if(error_code == 0)
//    {
//        //д�������Ч
//        NRF_LOG_INFO("nRF52 SDCard Writen is OK!");
//        NRF_LOG_FLUSH();
//    }else
//    {
//        NRF_LOG_INFO("nRF52 SDCard Writen is Wrong!");
//        NRF_LOG_FLUSH();
//        return 0;
//    }
    
    //����ʱ�������
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    error_code = nrfx_timer_init(&Leo_TIMER2, &timer_cfg, Leo_TIME2_Event_Handler);
    APP_ERROR_CHECK(error_code);

    nrfx_timer_extended_compare(&Leo_TIMER2,
                                NRF_TIMER_CC_CHANNEL2,
                                nrfx_timer_us_to_ticks(&Leo_TIMER2, 50),
                                NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK,
                                true);
    nrfx_timer_enable(&Leo_TIMER2);
    
    uint8_t test10[] = "123456789"; 

    
    uint16_t i = 0;
    uint16_t j = 0;
    uint32_t tulStart   = 0;
    uint32_t tulEnd     = 0;
    uint32_t tulUsed    = 0;
    
    
    float tTry = 0.0;
    tulStart = gtNum;
    for(i=0;i<9000;i++)
    {
        tTry = 1.0/999.99;     
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Calculate_1000 is OK,Used time is  %d us!",tulUsed*50);
    NRF_LOG_FLUSH(); 
 
 
 
/*

    //Test 6
    uint8_t test6[1024] = {0}; 
    tulStart = gtNum;
    for(i=0;i<1000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test6,sizeof(test6));	     
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_1024 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_delay_ms(3000);

    uint8_t test7[32] = {0}; 

    tulStart = gtNum;
    for(i=0;i<1000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test7,sizeof(test7));	     
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_512 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_delay_ms(3000);

    uint8_t test8[35] = {0}; 
    tulStart = gtNum;
    for(i=0;i<1000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test8,sizeof(test8));	     
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_64 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_delay_ms(3000);
    
    uint8_t test9[25] = {0}; 
    tulStart = gtNum;
    for(i=0;i<1000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test9,sizeof(test9));	     
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_16 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_delay_ms(3000);
*/    
/*

    
    //Test 5
    uint8_t test5[400] = {0}; 
    for(j=0;j<40;j++)
    {    
        memcpy(test5+j*10,test10,sizeof(test10));
    }

    tulStart = gtNum;
    for(i=0;i<4000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test5,sizeof(test5));	 
        nrf_delay_us(1);        
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart -4;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_400 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_delay_ms(3000);
    
     //Test 4
    uint8_t test4[200] = {0}; 
    for(j=0;j<20;j++)
    {    
        memcpy(test4+j*10,test10,sizeof(test10));
    }

    tulStart = gtNum;
    for(i=0;i<8000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test4,sizeof(test4));
        nrf_delay_us(1);
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart -8;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_200 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_gpio_pin_toggle(LED_3);
    nrf_gpio_pin_toggle(LED_4);
    nrf_delay_ms(3000);

        
    //Test 3
    uint8_t test3[100] = {0}; 
    for(j=0;j<10;j++)
    {    
        memcpy(test3+j*10,test10,sizeof(test10));
    }

    tulStart = gtNum;
    for(i=0;i<16000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test3,sizeof(test3));
        nrf_delay_us(1);
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart -16;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_100 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH(); 
    nrf_gpio_pin_toggle(LED_2);
    nrf_gpio_pin_toggle(LED_3);
    nrf_delay_ms(3000);
    

    //Test 2
    uint8_t test2[50] = {0}; 
    for(j=0;j<5;j++)
    {    
        memcpy(test2+j*10,test10,sizeof(test10));
    }

    tulStart = gtNum;
    for(i=0;i<32000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test2,sizeof(test2));
        nrf_delay_us(1);    
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart - 32;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_50 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH();    
    nrf_gpio_pin_toggle(LED_1);
    nrf_gpio_pin_toggle(LED_2);
    nrf_delay_ms(3000);

    //Test 1
    uint8_t test1[10] = {0}; 
    for(j=0;j<1;j++)
    {    
        memcpy(test1+j*10,test10,sizeof(test10));
    }

    tulStart = gtNum;
    for(i=0;i<160000;i++)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(test1,sizeof(test1));	 
        nrf_delay_us(1);
    }
    tulEnd = gtNum;     
    tulUsed = tulEnd - tulStart - 160;
    APP_ERROR_CHECK(error_code);
    NRF_LOG_INFO("The Writen_10 is OK,Used time is  %d!",tulUsed);
    NRF_LOG_FLUSH();
    nrf_gpio_pin_toggle(LED_1);
    nrf_delay_ms(3000);
    
*/




    
    Leo_nRF52_SDCard_FileClose();
    
    
    
     while (true)
    {
        __WFE();
    }
    
   
    
    
//    uint8_t error_code = 0;	
 
//�� log �� RTT���ģʽ    
//    error_code |= NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(error_code);
//    NRF_LOG_DEFAULT_BACKENDS_INIT();    
//	NRF_LOG_INFO("nRF52 LOG Initialization is DONE!");
//	nrf_delay_ms(30);      
//    NRF_LOG_FLUSH();
        
    
//���� LED������
   	//��ʼ�� GPIOTE �ܽŹ���
//	NRF_LOG_INFO("nRF52 GPIOTE Initialization is Begin....!");
//	error_code |= nrfx_gpiote_init();
//    APP_ERROR_CHECK(error_code);
//	NRF_LOG_INFO("nRF52 GPIOTE Initialization is DONE(err_code is 0x%x)",error_code);
//	nrf_delay_ms(30);	
//    NRF_LOG_FLUSH();

//    nrf_gpio_cfg_output(Leo_nRF52_LED_RED);
//    nrf_gpio_cfg_output(Leo_nRF52_LED_GREEN);
//    nrf_gpio_pin_write(Leo_nRF52_LED_RED,0);
//    nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,0);    
    
//������Ϣ ADC �ɼ� ʵ��	
/*    
    error_code |= Leo_BatteryCheck_Initial();    

    float val = 0.0;
    uint16_t    time = 1;    
    while(1)
    {        
        error_code |= Leo_BatteryCheck(&val);
        NRF_LOG_INFO("The Time is %d ;The Volt is." NRF_LOG_FLOAT_MARKER ,time, NRF_LOG_FLOAT(val));
        NRF_LOG_FLUSH();        
        nrf_delay_ms(60000);
        time++;
    }
*/



       
   


//BLE ʵ��
//----------Э��ջ��ʼ��
//    err_code = nrf_sdh_enable_request();
//    APP_ERROR_CHECK(err_code);


    
/*  
  
    

//=============================��Χ���������ݲɼ����洢=======================================   

//-----------------------------------��һ����ʼ�� ����-------------------------------------
//��ʵ״̬��ʶ--->��ʼ״̬ ���̵ƶ���
    uint8_t MyName[] = "Leo nRF52 is begining!.";
    
	
    //ȫ�ֱ�����ʼ��
	Leo_nRF52_Globale_Initial();
	
	//��ʼ�� LOG ��־����
    error_code |= NRF_LOG_INIT(NULL); 
    NRF_LOG_DEFAULT_BACKENDS_INIT();    
	NRF_LOG_INFO("nRF52 LOG Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);      
    NRF_LOG_FLUSH();
   
    //=============�ܽ��������========================== 
	//��ʼ�� GPIOTE �ܽŹ���
	NRF_LOG_INFO("nRF52 GPIOTE Initialization is Begin....!");
	error_code |= nrfx_gpiote_init();
    APP_ERROR_CHECK(error_code);
	NRF_LOG_INFO("nRF52 GPIOTE Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();
	
    //��ʼ�� LED �� ��� 17 �̵� 18
    nrf_gpio_cfg_output(Leo_nRF52_LED_RED);
    nrf_gpio_cfg_output(Leo_nRF52_LED_GREEN);
    nrf_gpio_pin_write(Leo_nRF52_LED_RED,1);
    nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,1);    
    
	//��ʼ�� ����MPU9255�ж� �ĹܽŹ���
	NRF_LOG_INFO("INT_MPU9255 Initialization is Begin....!");
	error_code |= Leo_INT_MPU9255_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("nRF52 INT_MPU9255 Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();	

	//��ʼ�� ����SDCard�洢�����ж� �ĹܽŹ���
	NRF_LOG_INFO("INT_SDCard Initialization is Begin....!");
	error_code |= Leo_INT_SDCard_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("INT_SDCard Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();    
    

	//��ʼ�� GPS 1PPS �����ж� �ĹܽŹ���
	NRF_LOG_INFO("INT_GPS_1PPS Initialization is Begin....!");
	error_code |= Leo_INT_GPSPPS_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("INT_GPS_1PPS Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();
    
    
    //=============�����������===========================
	//��ʼ�� SDCard 
	NRF_LOG_INFO("nRF52 SDCard Initialization is Begin....!");
	error_code |= Leo_nRF52_SDCard_Initial();	
	NRF_LOG_INFO("nRF52 SDCard Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(300);		
    //���� SDCard��д��
    NRF_LOG_INFO("nRF52 SDCard Writen is Begin....!");
	error_code |= Leo_nRF52_SDCard_SaveData(MyName,sizeof(MyName));	
	NRF_LOG_INFO("nRF52 SDCard Writen is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    if(error_code == 0)
    {
        //д�������Ч
        G_SDCard_IsSaved = 1;
    }



	//��ʼ�� MPU9255��������
	NRF_LOG_INFO("MPU9255 Initialization is Begin....!");
	error_code |= Leo_MPU9255_SPI_Initial();
	NRF_LOG_INFO("MPU9255 Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);	
    NRF_LOG_FLUSH();
    //MPU9255��ʼ����ɣ���һ������ ��ǿ�������ȱ�У����
    if(G_MPU9255_MAG_ASAXYZ_IsValid == 1)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(G_MPU9255_MAG_ASAXYZ,sizeof(G_MPU9255_MAG_ASAXYZ));	
        nrf_delay_ms(30);
    }
    
    
    //��ʼ�� GPS����ͨ��
    NRF_LOG_INFO("GPS Uart Initialization is Begin....!");
    error_code |= Leo_nRF52_GPS_Initial();
    NRF_LOG_INFO("GPS Uart Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);	
    NRF_LOG_FLUSH();
      
    //��ʼ�� ��ʱ������
    NRF_LOG_INFO("TIME0 Initialization is Begin....!");
    error_code |= Leo_TIME1_Initial();
    Leo_TIME1_Begin();      //������ʱ��
    NRF_LOG_INFO("TIME0 Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);	
    NRF_LOG_FLUSH();    
    
    //=============���������ж�===========================    
    //���� MPU9255�жϴ���  Ҳ�������� nrf_drv_gpiote_in_event_disable �رչܽŵ��жϴ���
    nrf_drv_gpiote_in_event_enable(Leo_INT_MPU9255, true);
	NRF_LOG_INFO("The INT of MPU9255 is Begining!");
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();	
	
 	//���� SDCard�ⲿ�����ж� ���ڹر�file�ļ����Է����ݶ�ʧ
    nrf_drv_gpiote_in_event_enable(Leo_INT_SDCard, true);
	NRF_LOG_INFO("The INT of SDCard Close is Begining!");
	nrf_delay_ms(30); 
    NRF_LOG_FLUSH();    
    
    //���� GPS 1PPS�������ж� ����ʱ��У׼
    nrf_drv_gpiote_in_event_enable(Leo_nRF52_GPS_UART_PPS, true);
	NRF_LOG_INFO("The INT of GPS_UART_PPS is Begining!");
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();    

    //��ʼ��״̬�ж�
    if(error_code == 0)
    {
        nrf_gpio_pin_write(Leo_nRF52_LED_RED,0);
    }else
    {
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(Leo_nRF52_LED_RED);            
        }
    }
//��һ��״̬��ʶ--->����ʼ����� �ж�error_code �������⣬����(17)������������,������ѭ���������˸>
    

    nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,0);
//----------------------������ ��ʼ��һ����ɣ�����ѭ�����ݲɼ� �洢---------------------------
    while(1)
    {
        if(G_SDCard_IsSaved == 0)
        {
            //�ⲿ���� ֹͣ�ɼ��жϣ��ر��ļ������˳�ѭ�� ����� �̵���
            Leo_nRF52_SDCard_FileClose();
            nrf_delay_ms(30);
            nrf_gpio_pin_write(Leo_nRF52_LED_RED,0);
            nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,1); 
            break;
        }
        
        //MPU9255���ݲɼ�
        if(G_MPU9255_Data_IsValid == 1)
        {
            //��д�뵱ǰ ms��������ʱ����׼ȷ��
            memcpy(G_MPU9255_Data+6,&G_MicroSecond,2);
            memcpy(G_MPU9255_Data+2,&G_GPSWeekSecond,4);
            memcpy(G_MPU9255_Data+8,&G_MPU9255_Counter,4);   
            
            uint8_t error_temp = 0;
            if(Leo_MPU9255_Read_ACC())
                error_temp |= (1);
            if(Leo_MPU9255_Read_Gyro())
                error_temp |= (1 << 1);
            if(Leo_MPU9255_Read_Magnetic())
                error_temp |= (1 << 2);
            //MPU9255���ݲɼ��д�
            if(error_temp > 0)
            {                
                memcpy(G_WRONG_Record+2,G_MPU9255_Data+2,6);
                G_WRONG_Record[8] = error_temp;
                error_code |= Leo_nRF52_SDCard_SaveData(G_WRONG_Record,sizeof(G_WRONG_Record));	       
            }
            //��Ϊ��ȷ�����ĸ��������������⣬�ȶ������ݴ������������� �����¼�����Է���
            
            error_code |= Leo_nRF52_SDCard_SaveData(G_MPU9255_Data,sizeof(G_MPU9255_Data));
            G_MPU9255_Data_IsValid = 0;
        }
        
        //GPS���� ���Խ��� �ɹ�
        if(!Leo_GPS_Decode())
        {
            //������������Ч
            //if((G_GPS_Data_RMCIsValid == 1) && (G_GPS_Data_GGAIsValid == 1))
            if(G_GPS_Data_RMCIsValid == 1)   //RMC��Ч�ʹ洢
            {
                error_code |= Leo_nRF52_SDCard_SaveData(G_GPS_Data,sizeof(G_GPS_Data));
                G_GPS_Data_RMCIsValid = 0;
                //G_GPS_Data_GGAIsValid = 0;
            }
            
        }

        
        //д��ʧ�ܣ�ֹͣ�ɼ���������ѭ���������˸���̵���
        if(error_code != 0)
        {
            nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,1);            
            while(1)
            {
                nrf_delay_ms(250);
                nrf_gpio_pin_toggle(Leo_nRF52_LED_RED);            
            }
        }                
    }       

//������״̬��ʶ---> ����ѭ���ɼ����洢������  �����     �̵���
//������״̬��ʶ---> ���ݴ洢����           �����˸   �̵���    
//------------------------------------Over������------------------------------------------    

   
    

   while (true)
    {
        __WFE();
    }

*/




}

/** @} */
