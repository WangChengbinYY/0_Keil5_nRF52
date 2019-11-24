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

#include "Leo_Includes.h"
#include "Leo_TIMER.h"
#include "Leo_INT.h"
#include "Leo_FreeRTOS.h"





/*
*********************************************************************************************************
*                                       全局变量定义区
*********************************************************************************************************
*/


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






















//---------------------------------BLE  Uart-------------------------------------





//---------------------------------FreeRTOS-------------------------------------
#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}














/******************************************************
 * 全局变量初始化
 *****************************************************/
void vInitial_Variable(void)
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



int main(void)
{
    // Initialize.
    bool erase_bonds;
    ret_code_t err_code;
    
/*
 1. 初始化_LOG日志输出 */    
    err_code = NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),err_code);
/*----------------------------------------------------------------------------*/  
    
/*
 2. 初始化_全局变量     */       
    vInitial_Variable();    
    NRF_LOG_INFO(("||Initialize||-->Variable------>error  0x%x"),err_code);
/*----------------------------------------------------------------------------*/   
    
/*
 3. 初始化_时钟 及 设定的定时器
    @brief Function for initializing the nrf_drv_clock module.
    After initialization, the module is in power off state (clocks are not requested).  */  
    err_code = nrf_drv_clock_init();
    
    /* 初始化 计时器TIMER4 */
    #if configTIMER4_ENABLE    
        vTimerInitial_4();
    #endif
    
    
    NRF_LOG_INFO(("||Initialize||-->TIMER--------->error  0x%x"),err_code);
/*----------------------------------------------------------------------------*/     

/*
 5. 初始化_LED灯管脚设定    */
    err_code |= nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code); 
    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);
/*----------------------------------------------------------------------------*/

/*
 6. 初始化_外部传感器设定    */
    /* MPU9255初始化 */
//    err_code |= Leo_MPU9255_SPI_Initial();
//    NRF_LOG_INFO(("||Initialize||-->MPU9255------->error  0x%x"),err_code);

//    //test
//    NRF_LOG_HEXDUMP_INFO(G_MPU9255_MAG_ASAXYZ,sizeof(G_MPU9255_MAG_ASAXYZ));
//    nrf_delay_ms(1000);
//    Leo_INT_MPU9255_Initial();
//    vINTStart_MPU9255();
//    uint8_t tM = 0;
//    for(;;)
//    {
//        if(G_MPU9255_Data_IsValid == 1)
//        {
//           
//            uint8_t error_temp = 0;
//            if(Leo_MPU9255_Read_ACC())
//                error_temp |= (1);
//            if(Leo_MPU9255_Read_Gyro())
//                error_temp |= (1 << 1);
//            if(Leo_MPU9255_Read_Magnetic())
//                error_temp |= (1 << 2);
//            //MPU9255数据采集有错
//            if(error_temp > 0)
//            {                
////                memcpy(G_WRONG_Record+2,G_MPU9255_Data+2,6);
////                G_WRONG_Record[8] = error_temp;
////                error_code |= Leo_nRF52_SDCard_SaveData(G_WRONG_Record,sizeof(G_WRONG_Record));	      
//                NRF_LOG_INFO("The MPU9255 Data is Wrong!!!!!");                
//            }
//            //因为不确定是哪个传感器出了问题，先都把数据存下来，反正有 出错记录，可以反查
//            
//            //err_code |= Leo_nRF52_SDCard_SaveData(G_MPU9255_Data,sizeof(G_MPU9255_Data));
//            G_MPU9255_Data_IsValid = 0;
//            
//            tM ++;
//            if(tM == 200)
//            {
//                NRF_LOG_INFO("%d ",G_MPU9255_Counter);
//                NRF_LOG_HEXDUMP_INFO(G_MPU9255_Data+8,8);
//                tM = 0;
//            }          
//            
//            
//        }
//    }
    
    
    
    /* SDCard初始化  */
    
    /* GPS初始化     */
    
    
    NRF_LOG_INFO("||Initialize||-->SENSOR-------->completed!");
/*----------------------------------------------------------------------------*/

    
/*任务建立_LOG日志 
注意：此处必须放在这里，如果放在后面则不显示！！！！！ */    
//#if NRF_LOG_ENABLED
//    // Start execution.
//    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", configMINIMAL_STACK_SIZE, NULL, 1, &m_logger_thread))
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }    
//#endif
         
    
/*
7.  初始化_BLE服务 压力传感器串口客户端     
    Create a FreeRTOS task for the BLE stack.
    The task will run advertising_start() before entering its loop.*/    
//    vBLEFootScan_Prepare();    
//    nrf_sdh_freertos_init(vBLEFootScan_Start, &erase_bonds);
  
 
/*
8.  创建任务_FreeRTOS xTask 
    Do not start any interrupt that uses system functions before system initialisation.
    The best solution is to start the OS before any other initalisation. */ 
//    vTask_Create();

/*
9.  启动_计时器    */
    /* 启动 计时器TIMER4 */
    #if configTIMER4_ENABLE    
        vTimerStart_4();
    #endif    

/*
10. 启动_外部中断INT  ____后面放到任务中执行  */       
//    vINTInital_SDCard();
//    NRF_LOG_INFO("||Initialize||---->>INT SDCard---->>completed!");
//    /* 启动 SDCard 中断响应 */
//    vINTStart_SDCard(); 
 
 
/*
11. 启动_FreeRTOS 任务循环执行    
    Start FreeRTOS scheduler.*/ 
//    NRF_LOG_INFO("||Start     ||---->>xTaskCreate---->>completed!");
//    NRF_LOG_FLUSH();
//    vTaskStartScheduler();
    
/*
12. 任务循环出错  
    // Enter main loop.*/ 
    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
    for (;;)
    {
        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        __WFE();
    }
}
