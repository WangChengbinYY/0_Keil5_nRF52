/*
*********************************************************************************************************
*
*    ģ������ : ��ʱ��
*    �ļ����� :Leo_TIMER
*    ��    �� : V1.0
*    ˵    �� : ��ʱ��ʵ�����
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-17     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_TIMER.h"


 

extern uint16_t    G_MicroSecond;
extern uint32_t    G_GPSWeekSecond;

extern uint8_t	    G_A0A0_Time_Seconds[7]; 
extern uint8_t	    G_A0A0_Time_Seconds_IsReady;

extern uint8_t     G_SDCard_FileIsOpen;
extern TaskHandle_t    xTaskHandle_UWB_Start;         /*5ms�����Ĳɼ�����    ��� */

//extern SemaphoreHandle_t   xMutex_SDCard_CirBuffer;
//extern TaskHandle_t    xTaskHandle_GPS_RxData;

/* TIMER��ʱ����� 
 * Your application cannot use RTC0 and TIMER0 if you are using BLE
 * Your application cannot use RTC1 if you are using FreeRTOS.
 * ϵͳĬ���趨�� TIMER �ж�Ȩ��Ϊ 7
 */


/*========================= TIMER2 ��ʵ�� ============================*/
/* TIMER2_�����趨 
 * ����10ms�ļ�ʱ������10ms�ɼ�һ��
 * ע�⣺TICK�ĵ�λ��ms                                   */
#define  configTIMER2_INSTANCE                      2
#define  configTIMER2_TICK                          13                //ms

const nrfx_timer_t  xTimerInstance_2 = NRFX_TIMER_INSTANCE(configTIMER2_INSTANCE); 

/*-----------------------------------------------------------------------
 * TIMER2 �����ص�����                                                    
 * ʹ�õ���    NRF_TIMER_EVENT_COMPARE2 ͨ���Ƚ���                        
*-----------------------------------------------------------------------*/
static void vTimerHandler_2(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE2)
    {
        if(G_SDCard_FileIsOpen == 1)
        {
#if configUWB_INIT   //����UWB���            
//            BaseType_t xHigherPriorityTaskWoken = pdFALSE;  
//            BaseType_t xReturn = pdPASS;
//            xReturn = xTaskNotifyFromISR(xTaskHandle_UWB_Start,0,eSetValueWithoutOverwrite,&xHigherPriorityTaskWoken);            
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);  
//            if(xReturn == pdFAIL)
//            {
//                NRF_LOG_INFO("     MessageOverFlow_____vTask_UWB_Start");
//                NRF_LOG_FLUSH(); 
//            }            
#endif
        }
    }
}

/*-----------------------------------------------------------------------*/
/* TIMER2 ��������ʼ��                                                   */
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
/* TIMER3 ����������                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucTimerStart_2(void)
{  
    nrfx_timer_enable(&xTimerInstance_2);  
    return 0;
}


/*========================= TIMER3 ��ʵ�� ============================*/
/* TIMER3_�����趨 
 * ����1ms�ļ�ʱ���ṩ��׼��ʱ���׼����ͨ��gps 1pps ����
 * ע�⣺TICK�ĵ�λ��ms                                   */
#define  configTIMER3_INSTANCE                      3
#define  configTIMER3_TICK                          1                    //ms

const nrfx_timer_t  xTimerInstance_3 = NRFX_TIMER_INSTANCE(configTIMER3_INSTANCE); 

/*-----------------------------------------------------------------------
 * TIMER3 �����ص�����                                                    
 * ʹ�õ���    NRF_TIMER_EVENT_COMPARE3 ͨ���Ƚ���                        
*-----------------------------------------------------------------------*/
static void vTimerHandler_3(nrf_timer_event_t event_type, void* p_context)
{
    if(event_type == NRF_TIMER_EVENT_COMPARE3)
    { 
        //(1)ʱ�� + 1ms
        if(G_MicroSecond < 999)
            G_MicroSecond = G_MicroSecond + 1;
        else
        {
            G_MicroSecond = 0;
            G_GPSWeekSecond = G_GPSWeekSecond + 1;     

            G_A0A0_Time_Seconds_IsReady = 1;
            G_A0A0_Time_Seconds[6] = 0;     //���������������            
            memcpy(G_A0A0_Time_Seconds+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));            
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


/*========================= TIMER4 ��ʵ�� ============================*/
/* TIMER4_�����趨 
 *      ����FreeRTOS�������ͳ��ʹ��                           */
#define  configTIMER4_INSTANCE                      4
#define  configTIMER4_TICK                          50              //us

/* TIMER4 ��������  */  
const nrfx_timer_t  xTimerInstance_4 = NRFX_TIMER_INSTANCE(configTIMER4_INSTANCE); 
/* TIMER4 ʹ�õļ����� Ticks  */ 
volatile uint32_t   ulTimerTicks_4   = 0UL;

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
