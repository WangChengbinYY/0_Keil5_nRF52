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




/* 1ms��ʱ������ֵ��1msһ���� ��0��ʼ��1000һ��ѭ�� */
extern uint16_t    G_MicroSecond;
extern uint32_t    G_GPSWeekSecond;






/*
*********************************************************************************************************
*                                       TIMER3 ʵ��
*********************************************************************************************************
*/

#if configTIMER3_ENABLE

/* TIMER4 ��������  */  
const nrfx_timer_t  xTimerInstance_3                = NRFX_TIMER_INSTANCE(configTIMER3_INSTANCE); 

/*-----------------------------------------------------------------------*/
/* TIMER3 �����ص�����                                                    */
/* ʹ�õ���    NRF_TIMER_EVENT_COMPARE3 ͨ���Ƚ���                        */
/*-----------------------------------------------------------------------*/
static void vTimerHandler_3(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE3)
    {
        if(G_MicroSecond < 999)
            G_MicroSecond++;
        else
        {
            G_MicroSecond = 0;
            G_GPSWeekSecond++;
        }
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER3 ��������ʼ��                                                   */
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
/* TIMER3 ����������                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_3(void)
{  
    nrfx_timer_enable(&xTimerInstance_3);  
    return 0;
}


#endif

/*******************************************************************************************************/



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
/* TIMER4 ����������                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_4(void)
{  
    nrfx_timer_enable(&xTimerInstance_4);  
    return 0;
}

#endif

/*******************************************************************************************************/
