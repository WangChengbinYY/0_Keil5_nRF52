/*
*********************************************************************************************************
*
*    ģ������ : ��ʱ��
*    �ļ����� : Leo_TIMER
*    ��    �� : V1.0
*    ˵    �� : ��ʱ��ʵ�����
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-17     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_TIMER.h"






















/*
*********************************************************************************************************
*                                       TIMER4 ʵ��
*********************************************************************************************************
*/
#if configTIMER4_ENABLE


/* TIMER4 ��������  */  
const nrfx_timer_t  xTimerInstance_4                = NRFX_TIMER_INSTANCE(configTIMER4_INSTANCE); 
/* TIMER4 ʹ�õļ����� Ticks  */ 
volatile uint32_t   ulTimerTicks_4                  = 0UL;

/*-----------------------------------------------------------------------*/
/* TIMER4 �����ص�����                                                    */
/* ʹ�õ���    NRF_TIMER_EVENT_COMPARE4 ͨ���Ƚ���                        */
/*-----------------------------------------------------------------------*/
static void vTimerHandler_4(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE4)
    {
        ulTimerTicks_4++;
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER4 ��������ʼ��                                                   */
/*----------------------------------------------------------------------*/
void vTimerInitial_4(void)
{
    nrfx_err_t error_code = 0;	
    nrfx_timer_config_t txTIMEConfig_4 = NRFX_TIMER_DEFAULT_CONFIG;   
    error_code = nrfx_timer_init(&xTimerInstance_4, &txTIMEConfig_4, vTimerHandler_4);
    APP_ERROR_CHECK(error_code);
    nrfx_timer_extended_compare(
                                &xTimerInstance_4, 
                                NRF_TIMER_CC_CHANNEL4, 
                                nrfx_timer_us_to_ticks(&xTimerInstance_4,configTIMER4_TICK), 
                                NRF_TIMER_SHORT_COMPARE4_CLEAR_MASK, 
                                true);    
    nrfx_timer_enable(&xTimerInstance_4);      
}


/*-----------------------------------------------------------------------*/
/* TIMER4 ����������                                                     */
/*----------------------------------------------------------------------*/
void vTimerStart_4(void)
{  
    nrfx_timer_enable(&xTimerInstance_4);      
}


#endif


