/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_nRF52_Time     
 * ƽ̨    ��nRF52832 
 * ����    ��ͨ����ʱ��ʵ�ֵ�ʱ�ӣ��������д�������ʱ��ͬ��  
 * ����    �����ɱ�
**********************************************************************/

#include "Leo_nRF52_Time.h"


const nrfx_timer_t Leo_TIMER1 = NRFX_TIMER_INSTANCE(1);   

//��ʱ������ֵ��1msһ���� ��0��ʼ��1000һ��ѭ��
extern uint16_t    G_MicroSecond;



//@brief TIME0��ʱ���������¼�
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*����ֵ˵��:
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

//@brief TIME0��ʱ�� ��ʼ��
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*����ֵ˵��:
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


//@brief TIME0��ʱ�� ��ʼ����
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*����ֵ˵��:
/*--------------------------------------------------------------------------*/
void Leo_TIME1_Begin(void)
{
    nrfx_timer_enable(&Leo_TIMER1);
}


