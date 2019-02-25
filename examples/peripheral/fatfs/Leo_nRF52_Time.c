/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_nRF52_Time     
 * 平台    ：nRF52832 
 * 描述    ：通过计时器实现的时钟，用于所有传感器的时间同步  
 * 作者    ：王成宾
**********************************************************************/

#include "Leo_nRF52_Time.h"


const nrfx_timer_t Leo_TIMER1 = NRFX_TIMER_INSTANCE(1);   

//计时器计数值，1ms一个数 从0开始，1000一个循环
extern uint16_t    G_MicroSecond;



//@brief TIME0计时器触发的事件
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*返回值说明:
/*--------------------------------------------------------------------------*/
static void Leo_TIME1_Event_Handler(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE1)
    {
        if(G_MicroSecond < 999)
            G_MicroSecond++;
        else
            G_MicroSecond = 0;
    }
}

//@brief TIME0计时器 初始化
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*返回值说明:
/*--------------------------------------------------------------------------*/
uint8_t Leo_TIME1_Initial(void)
{
    uint8_t error_code = 0;
    uint32_t time_ms = 1; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks = 0;
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    error_code = nrfx_timer_init(&Leo_TIMER1, &timer_cfg, Leo_TIME1_Event_Handler);
    
    time_ticks = nrfx_timer_ms_to_ticks(&Leo_TIMER1, time_ms);

    nrfx_timer_extended_compare(
         &Leo_TIMER1, NRF_TIMER_CC_CHANNEL1, time_ticks, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
    
    return error_code;
}


//@brief TIME0计时器 开始工作
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*返回值说明:
/*--------------------------------------------------------------------------*/
void Leo_TIME1_Begin(void)
{
    nrfx_timer_enable(&Leo_TIMER1);
}


