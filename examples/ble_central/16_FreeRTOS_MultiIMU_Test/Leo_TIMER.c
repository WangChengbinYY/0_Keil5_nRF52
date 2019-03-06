/*
*********************************************************************************************************
*
*    模块名称 : 计时器
*    文件名称 : Leo_TIMER
*    版    本 : V1.0
*    说    明 : 计时器实现相关
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-17     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_TIMER.h"




extern uint16_t    G_MicroSecond;
extern uint32_t    G_GPSWeekSecond;
extern uint8_t	   G_IMUData_Counter;
extern uint8_t     G_SDCard_FileIsOpen;
extern TaskHandle_t    xTaskHandle_CollectData;         /*5ms触发的采集任务    句柄 */





/* TIMER计时器相关 
 * Your application cannot use RTC0 and TIMER0 if you are using BLE
 * Your application cannot use RTC1 if you are using FreeRTOS.
 * 系统默认设定的 TIMER 中断权限为 7
 */


/*========================= TIMER2 的实现 ============================*/
/* TIMER2_参数设定 
 * 用于5ms的计时，数据5ms采集一次
 * 注意：TICK的单位是ms                                   */
#define  configTIMER2_INSTANCE                      2
#define  configTIMER2_TICK                          5                //ms

const nrfx_timer_t  xTimerInstance_2 = NRFX_TIMER_INSTANCE(configTIMER2_INSTANCE); 

/*-----------------------------------------------------------------------
 * TIMER2 触发回调函数                                                    
 * 使用的是    NRF_TIMER_EVENT_COMPARE2 通道比较器                        
*-----------------------------------------------------------------------*/
static void vTimerHandler_2(nrf_timer_event_t event_type, void* p_context)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //5ms 触发采集任务
    if(event_type == NRF_TIMER_EVENT_COMPARE2)
    {
        //这做判断 是否采集
        if(G_SDCard_FileIsOpen == 1)
        {
            xTaskNotifyFromISR(xTaskHandle_CollectData,    
                                0,           
                                eNoAction,
                                &xHigherPriorityTaskWoken);            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  
        }
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER2 计数器初始化                                                   */
/*----------------------------------------------------------------------*/
uint8_t ucTimerInitial_2(void)
{
    uint8_t error_code = 0;	
    nrfx_timer_config_t txTIMEConfig_2 = NRFX_TIMER_DEFAULT_CONFIG;   
    error_code = nrfx_timer_init(&xTimerInstance_2, &txTIMEConfig_2, vTimerHandler_2);    
    nrfx_timer_extended_compare(
                                &xTimerInstance_2, 
                                NRF_TIMER_CC_CHANNEL2, 
                                nrfx_timer_ms_to_ticks(&xTimerInstance_2,configTIMER2_TICK), 
                                NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, 
                                true);    
 
    return error_code;
}

/*-----------------------------------------------------------------------*/
/* TIMER3 计数器启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_2(void)
{  
    nrfx_timer_enable(&xTimerInstance_2);  
    return 0;
}












/*========================= TIMER3 的实现 ============================*/
/* TIMER3_参数设定 
 * 用于1ms的计时，提供标准的时间基准，并通过gps 1pps 对齐
 * 注意：TICK的单位是ms                                   */
#define  configTIMER3_INSTANCE                      3
#define  configTIMER3_TICK                          1                //ms

const nrfx_timer_t  xTimerInstance_3 = NRFX_TIMER_INSTANCE(configTIMER3_INSTANCE); 

/*-----------------------------------------------------------------------
 * TIMER3 触发回调函数                                                    
 * 使用的是    NRF_TIMER_EVENT_COMPARE3 通道比较器                        
*-----------------------------------------------------------------------*/
static void vTimerHandler_3(nrf_timer_event_t event_type, void* p_context)
{
    //待完善 此处后面添加 5ms 的判断
    if(event_type == NRF_TIMER_EVENT_COMPARE3)
    {
        //(1)时间 + 1ms
        if(G_MicroSecond < 999)
            G_MicroSecond++;
        else
        {
            G_MicroSecond = 0;
            G_GPSWeekSecond++;
        }
/*        
        //(2)判断是否为 整5ms
        if((G_MicroSecond % 5) == 0)
        {
            G_IMUData_Counter++;
            //将时间信息和采样计数值 赋值到 数据结构体中 此处先不实现，看看实际处理情况
            //是否有超过1ms的延迟，如果数据的ms时间都是按照5来递增的，就不考虑
            
            //通知 数据采集 和 测距 未完成
            xTaskNotifyFromISR(xTaskHandle_CollectData,    
                                0,           
                                eNoAction,
                                &xHigherPriorityTaskWoken);            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);              
        }            
*/
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER3 计数器初始化                                                   */
/*----------------------------------------------------------------------*/
uint8_t ucTimerInitial_3(void)
{
    uint8_t error_code = 0;	
    nrfx_timer_config_t txTIMEConfig_3 = NRFX_TIMER_DEFAULT_CONFIG;   
    error_code = nrfx_timer_init(&xTimerInstance_3, &txTIMEConfig_3, vTimerHandler_3);    
    nrfx_timer_extended_compare(
                                &xTimerInstance_3, 
                                NRF_TIMER_CC_CHANNEL3, 
                                nrfx_timer_ms_to_ticks(&xTimerInstance_3,configTIMER3_TICK), 
                                NRF_TIMER_SHORT_COMPARE3_CLEAR_MASK, 
                                true);    
 
    return error_code;
}

/*-----------------------------------------------------------------------*/
/* TIMER3 计数器启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_3(void)
{  
    nrfx_timer_enable(&xTimerInstance_3);  
    return 0;
}


/*========================= TIMER4 的实现 ============================*/
/* TIMER4_参数设定 
 *      用于FreeRTOS任务分析统计使用                           */
#define  configTIMER4_INSTANCE                      4
#define  configTIMER4_TICK                          50              //us

/* TIMER4 对象声明  */  
const nrfx_timer_t  xTimerInstance_4 = NRFX_TIMER_INSTANCE(configTIMER4_INSTANCE); 
/* TIMER4 使用的计数器 Ticks  */ 
volatile uint32_t   ulTimerTicks_4   = 0UL;

/*-----------------------------------------------------------------------*/
/* TIMER4 触发回调函数                                                    */
/* 使用的是    NRF_TIMER_EVENT_COMPARE4 通道比较器                        */
/*-----------------------------------------------------------------------*/
static void vTimerHandler_4(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE4)
    {
        ulTimerTicks_4++;
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER4 计数器初始化                                                   */
/*----------------------------------------------------------------------*/
uint8_t ucTimerInitial_4(void)
{
    uint8_t error_code = 0;	
    nrfx_timer_config_t txTIMEConfig_4 = NRFX_TIMER_DEFAULT_CONFIG;   
    error_code = nrfx_timer_init(&xTimerInstance_4, &txTIMEConfig_4, vTimerHandler_4);    
    nrfx_timer_extended_compare(
                                &xTimerInstance_4, 
                                NRF_TIMER_CC_CHANNEL4, 
                                nrfx_timer_us_to_ticks(&xTimerInstance_4,configTIMER4_TICK), 
                                NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, 
                                true);    
    return error_code;
}

/*-----------------------------------------------------------------------*/
/* TIMER4 计数器启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_4(void)
{  
    nrfx_timer_enable(&xTimerInstance_4);  
    return 0;
}
