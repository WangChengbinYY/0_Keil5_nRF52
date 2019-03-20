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

/**@brief 头文件声明
<*--------------------------------------------------------------------------*>*/
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h" 

//==========所有管脚 及 全局配置参数 宏定义  还有共同需要的头文件等
#include "Leo_nRF52_config.h"			

//==========相关外设定义
#include "Leo_BatteryCheck.h"       //ADC 采集相关
#include "Leo_nRF52_GPS.h"	        //GPS模块 串口
#include "Leo_int.h"	            //中断    GPIOTE
#include "Leo_nRF52_SDCard.h"	    //SDCard存储实现
#include "Leo_mpu9255.h"	        //MPU9255传感器相关				
#include "Leo_nRF52_Time.h"		    //自定义 Timer计时器相关	

//BLE 蓝牙相关
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



/**@brief 全局变量定义区
<*--------------------------------------------------------------------------*>*/

//===========各类数据存储变量=========================
//AK8963磁强计读出的灵敏度修正参数数据 2~4
uint8_t		G_MPU9255_MAG_ASAXYZ[7];
uint8_t		G_MPU9255_MAG_ASAXYZ_IsValid;
//MPU9255传感器存放的数据
uint8_t		G_MPU9255_Data[32];
uint8_t		G_MPU9255_Data_IsValid;
uint32_t	G_MPU9255_Counter;  //MPU9255中断触发的计数器，用于后期统计是否丢包 对应到G_MPU9255_Data[32] 的8~11位
//GPS接收数据
uint8_t     G_GPS_Data[35];
uint8_t     G_GPS_Data_RMCIsValid;
uint8_t     G_GPS_Data_GGAIsValid;
//足部压力传感器数据
uint8_t     G_FOOTPressure_Data[25];
uint8_t     G_FOOTPressure_Data_IsValid;


//===========时间相关存储变量=========================
//GPS周内秒数据
uint32_t    G_GPSWeekSecond;
//nRF52时间计数器控制的 1s的1000计数值，由 外部GPS的1PPS校准 1PPS触发时 将其置0
uint16_t    G_MicroSecond;


//===========系统控制相关参数=========================

//SDCard 存储文件是否已打开，是否正常存储  0 没有  1 正常
uint8_t     G_SDCard_IsSaved;
//采集过程数据出错的记录
uint8_t     G_WRONG_Record[11];



/**@brief 主体功能实现区
<*--------------------------------------------------------------------------*>*/


static void Leo_nRF52_Globale_Initial(void)
{
//===========各类数据存储变量初始化=========================    
    //AK8963磁强计读出的灵敏度修正参数 2~4
    //uint8_t		G_MPU9255_MAG_ASAXYZ[7] = {0};
    memset(G_MPU9255_MAG_ASAXYZ,0,7);
	G_MPU9255_MAG_ASAXYZ[0] = 0xC1;
	G_MPU9255_MAG_ASAXYZ[1] = 0xC2;
    G_MPU9255_MAG_ASAXYZ[5] = 0xF1;
    G_MPU9255_MAG_ASAXYZ[6] = 0xF2;    
    G_MPU9255_MAG_ASAXYZ_IsValid = 0;
    //MPU9255传感器存放的数据
    //uint8_t		G_MPU9255_Data[32] = {0};
    memset(G_MPU9255_Data,0,32);
    G_MPU9255_Data[0] = 0xA1;
	G_MPU9255_Data[1] = 0xA2;
    G_MPU9255_Data[30] = 0xF1;
    G_MPU9255_Data[31] = 0xF2;
    G_MPU9255_Data_IsValid = 0;
    G_MPU9255_Counter = 0;  //MPU9255中断触发的计数器，用于后期统计是否丢包 对应到G_MPU9255_Data[32] 的8~11位
    //GPS接收数据
    //uint8_t     G_GPS_Data[35] = {0};
    memset(G_GPS_Data,0,35);
    G_GPS_Data[0] = 0xA3;
    G_GPS_Data[1] = 0xA4;
    G_GPS_Data[33] = 0xF1;
    G_GPS_Data[34] = 0xF2;
    G_GPS_Data_RMCIsValid = 0;
    G_GPS_Data_GGAIsValid = 0;
    //足部压力传感器数据
    //uint8_t     G_FOOTPressure_Data[25] = {0};
    memset(G_FOOTPressure_Data,0,25);
    G_FOOTPressure_Data[0] = 0xA5;
	G_FOOTPressure_Data[1] = 0xA6;
    G_FOOTPressure_Data[23] = 0xF1;
    G_FOOTPressure_Data[24] = 0xF2;
    G_FOOTPressure_Data_IsValid = 0;    
    
    
    
//===========时间相关存储变量初始化=========================   
    G_GPSWeekSecond = 0;
    G_MicroSecond = 0;



//===========系统控制相关参数初始化=========================
    G_SDCard_IsSaved = 0;
 
    //采集过程数据出错的记录 
    memset(G_WRONG_Record,0,11);
    G_WRONG_Record[0] = 0xC3;
	G_WRONG_Record[1] = 0xC4;
    G_WRONG_Record[9] = 0xF1;
    G_WRONG_Record[10] = 0xF2;
    
}




uint32_t    gtNum = 0;


/*-------------------------------------------------    
 *                  test SDCard写入速度   
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
 *                  test SDCard写入速度   
 *--------------------------------------------------*/   
    uint8_t error_code = 0;
    bsp_board_init(BSP_INIT_LEDS);
//打开 log 的 RTT输出模式        
    error_code |= NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(error_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();    
	NRF_LOG_INFO("nRF52 LOG Initialization is DONE!");     
    NRF_LOG_FLUSH();    
    
    //初始化 GPIOTE 管脚功能
//	error_code |= nrfx_gpiote_init();
//    APP_ERROR_CHECK(error_code);
//	NRF_LOG_INFO("nRF52 GPIOTE Initialization is DONE!");
//    NRF_LOG_FLUSH();
    
    
    NRF_LOG_INFO("TIME0 Initialization is Begin....!");
    error_code |= Leo_TIME1_Initial();
    Leo_TIME1_Begin();      //启动定时器
    NRF_LOG_INFO("TIME0 Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);
    
     while (true)
    {
        __WFE();
    }
    
    
    
    //初始化 SDCard 
//	NRF_LOG_INFO("nRF52 SDCard Initialization is Begin....!");
//	error_code |= Leo_nRF52_SDCard_Initial();	
//	NRF_LOG_INFO("nRF52 SDCard Initialization is DONE(err_code is 0x%x)",error_code);
//	nrf_delay_ms(300);		
//    //测试 SDCard的写入
//    uint8_t MyName[] = "Leo nRF52 is begining!.";    
//	error_code |= Leo_nRF52_SDCard_SaveData(MyName,sizeof(MyName));	
//    if(error_code == 0)
//    {
//        //写入测试有效
//        NRF_LOG_INFO("nRF52 SDCard Writen is OK!");
//        NRF_LOG_FLUSH();
//    }else
//    {
//        NRF_LOG_INFO("nRF52 SDCard Writen is Wrong!");
//        NRF_LOG_FLUSH();
//        return 0;
//    }
    
    //设置时间计数器
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
 
//打开 log 的 RTT输出模式    
//    error_code |= NRF_LOG_INIT(NULL);
//    APP_ERROR_CHECK(error_code);
//    NRF_LOG_DEFAULT_BACKENDS_INIT();    
//	NRF_LOG_INFO("nRF52 LOG Initialization is DONE!");
//	nrf_delay_ms(30);      
//    NRF_LOG_FLUSH();
        
    
//试验 LED灯亮灭
   	//初始化 GPIOTE 管脚功能
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
    
//电量信息 ADC 采集 实验	
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



       
   


//BLE 实验
//----------协议栈初始化
//    err_code = nrf_sdh_enable_request();
//    APP_ERROR_CHECK(err_code);


    
/*  
  
    

//=============================外围传感器数据采集及存储=======================================   

//-----------------------------------第一步初始化 外设-------------------------------------
//其实状态标识--->起始状态 红绿灯都灭
    uint8_t MyName[] = "Leo nRF52 is begining!.";
    
	
    //全局变量初始化
	Leo_nRF52_Globale_Initial();
	
	//初始化 LOG 日志功能
    error_code |= NRF_LOG_INIT(NULL); 
    NRF_LOG_DEFAULT_BACKENDS_INIT();    
	NRF_LOG_INFO("nRF52 LOG Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);      
    NRF_LOG_FLUSH();
   
    //=============管脚相关设置========================== 
	//初始化 GPIOTE 管脚功能
	NRF_LOG_INFO("nRF52 GPIOTE Initialization is Begin....!");
	error_code |= nrfx_gpiote_init();
    APP_ERROR_CHECK(error_code);
	NRF_LOG_INFO("nRF52 GPIOTE Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();
	
    //初始化 LED 灯 红灯 17 绿灯 18
    nrf_gpio_cfg_output(Leo_nRF52_LED_RED);
    nrf_gpio_cfg_output(Leo_nRF52_LED_GREEN);
    nrf_gpio_pin_write(Leo_nRF52_LED_RED,1);
    nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,1);    
    
	//初始化 接收MPU9255中断 的管脚功能
	NRF_LOG_INFO("INT_MPU9255 Initialization is Begin....!");
	error_code |= Leo_INT_MPU9255_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("nRF52 INT_MPU9255 Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();	

	//初始化 接收SDCard存储申请中断 的管脚功能
	NRF_LOG_INFO("INT_SDCard Initialization is Begin....!");
	error_code |= Leo_INT_SDCard_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("INT_SDCard Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();    
    

	//初始化 GPS 1PPS 申请中断 的管脚功能
	NRF_LOG_INFO("INT_GPS_1PPS Initialization is Begin....!");
	error_code |= Leo_INT_GPSPPS_Initial();
    APP_ERROR_CHECK(error_code);	
	NRF_LOG_INFO("INT_GPS_1PPS Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();
    
    
    //=============外设相关设置===========================
	//初始化 SDCard 
	NRF_LOG_INFO("nRF52 SDCard Initialization is Begin....!");
	error_code |= Leo_nRF52_SDCard_Initial();	
	NRF_LOG_INFO("nRF52 SDCard Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(300);		
    //测试 SDCard的写入
    NRF_LOG_INFO("nRF52 SDCard Writen is Begin....!");
	error_code |= Leo_nRF52_SDCard_SaveData(MyName,sizeof(MyName));	
	NRF_LOG_INFO("nRF52 SDCard Writen is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);
    if(error_code == 0)
    {
        //写入测试有效
        G_SDCard_IsSaved = 1;
    }



	//初始化 MPU9255器件参数
	NRF_LOG_INFO("MPU9255 Initialization is Begin....!");
	error_code |= Leo_MPU9255_SPI_Initial();
	NRF_LOG_INFO("MPU9255 Initialization is DONE(err_code is 0x%x)",error_code);
	nrf_delay_ms(30);	
    NRF_LOG_FLUSH();
    //MPU9255初始化完成，第一步存入 磁强计灵敏度标校参数
    if(G_MPU9255_MAG_ASAXYZ_IsValid == 1)
    {
        error_code |= Leo_nRF52_SDCard_SaveData(G_MPU9255_MAG_ASAXYZ,sizeof(G_MPU9255_MAG_ASAXYZ));	
        nrf_delay_ms(30);
    }
    
    
    //初始化 GPS串口通信
    NRF_LOG_INFO("GPS Uart Initialization is Begin....!");
    error_code |= Leo_nRF52_GPS_Initial();
    NRF_LOG_INFO("GPS Uart Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);	
    NRF_LOG_FLUSH();
      
    //初始化 定时器设置
    NRF_LOG_INFO("TIME0 Initialization is Begin....!");
    error_code |= Leo_TIME1_Initial();
    Leo_TIME1_Begin();      //启动定时器
    NRF_LOG_INFO("TIME0 Initialization is DONE(err_code is 0x%x)",error_code);
    nrf_delay_ms(30);	
    NRF_LOG_FLUSH();    
    
    //=============启动各类中断===========================    
    //启动 MPU9255中断触发  也可以利用 nrf_drv_gpiote_in_event_disable 关闭管脚的中断触发
    nrf_drv_gpiote_in_event_enable(Leo_INT_MPU9255, true);
	NRF_LOG_INFO("The INT of MPU9255 is Begining!");
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();	
	
 	//启动 SDCard外部控制中断 用于关闭file文件，以防数据丢失
    nrf_drv_gpiote_in_event_enable(Leo_INT_SDCard, true);
	NRF_LOG_INFO("The INT of SDCard Close is Begining!");
	nrf_delay_ms(30); 
    NRF_LOG_FLUSH();    
    
    //启动 GPS 1PPS秒脉冲中断 用于时间校准
    nrf_drv_gpiote_in_event_enable(Leo_nRF52_GPS_UART_PPS, true);
	NRF_LOG_INFO("The INT of GPS_UART_PPS is Begining!");
	nrf_delay_ms(30);
    NRF_LOG_FLUSH();    

    //初始化状态判断
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
//第一步状态标识--->若初始化完成 判断error_code 若无问题，则红灯(17)亮；若有问题,进入死循环，红灯闪烁>
    

    nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,0);
//----------------------第三步 初始化一切完成，进入循环数据采集 存储---------------------------
    while(1)
    {
        if(G_SDCard_IsSaved == 0)
        {
            //外部触发 停止采集中断，关闭文件，并退出循环 红灯亮 绿灯灭
            Leo_nRF52_SDCard_FileClose();
            nrf_delay_ms(30);
            nrf_gpio_pin_write(Leo_nRF52_LED_RED,0);
            nrf_gpio_pin_write(Leo_nRF52_LED_GREEN,1); 
            break;
        }
        
        //MPU9255数据采集
        if(G_MPU9255_Data_IsValid == 1)
        {
            //先写入当前 ms数，这样时间能准确点
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
            //MPU9255数据采集有错
            if(error_temp > 0)
            {                
                memcpy(G_WRONG_Record+2,G_MPU9255_Data+2,6);
                G_WRONG_Record[8] = error_temp;
                error_code |= Leo_nRF52_SDCard_SaveData(G_WRONG_Record,sizeof(G_WRONG_Record));	       
            }
            //因为不确定是哪个传感器出了问题，先都把数据存下来，反正有 出错记录，可以反查
            
            error_code |= Leo_nRF52_SDCard_SaveData(G_MPU9255_Data,sizeof(G_MPU9255_Data));
            G_MPU9255_Data_IsValid = 0;
        }
        
        //GPS数据 尝试解析 成功
        if(!Leo_GPS_Decode())
        {
            //解析的数据有效
            //if((G_GPS_Data_RMCIsValid == 1) && (G_GPS_Data_GGAIsValid == 1))
            if(G_GPS_Data_RMCIsValid == 1)   //RMC有效就存储
            {
                error_code |= Leo_nRF52_SDCard_SaveData(G_GPS_Data,sizeof(G_GPS_Data));
                G_GPS_Data_RMCIsValid = 0;
                //G_GPS_Data_GGAIsValid = 0;
            }
            
        }

        
        //写入失败，停止采集，进入死循环，红灯闪烁，绿灯亮
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

//第三步状态标识---> 数据循环采集，存储正常：  红灯亮     绿灯亮
//第三步状态标识---> 数据存储出错：           红灯闪烁   绿灯亮    
//------------------------------------Over！！！------------------------------------------    

   
    

   while (true)
    {
        __WFE();
    }

*/




}

/** @} */
