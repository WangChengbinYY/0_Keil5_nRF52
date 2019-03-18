/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ AD ���ݵĲɼ�
*    �ļ����� :Leo_SAADC
*    ��    �� : V1.0
*    ˵    �� : �ⲿ AD�ɼ�
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-10     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_SAADC.h"




/*------------------------------------------------------------
 *SAADC �¼��ص�����                                                                       
 *------------------------------------------------------------*/
static void vSAADC_Event_Handler(nrfx_saadc_evt_t const * p_event)
{
    
}



/**
 * ADC ��ʼ��
 *   0 �ɹ��� 1 ʧ��
*/
uint8_t ucSAADCInitial(void)
{
    uint8_t error_code = 0;  
    
//ͨ����������
    /* ����ͨ��0 ��Ӧ ��Ÿ���6���ܽ�Ϊ P0.03  AIN1 */
    nrf_saadc_channel_config_t txSAADC_Channel_0_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);    
    /* ����ͨ��1 ��Ӧ ��Ÿ���7���ܽ�Ϊ P0.04  AIN2 */
    nrf_saadc_channel_config_t txSAADC_Channel_1_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);  
    /* ����ͨ��2 ��Ӧ ǰ���Ƶ�5���ܽ�Ϊ P0.31  AIN7 */
    nrf_saadc_channel_config_t txSAADC_Channel_2_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);      
    /* ����ͨ��3 ��Ӧ ǰ���Ƶ�2���ܽ�Ϊ P0.30  AIN6 */
    nrf_saadc_channel_config_t txSAADC_Channel_3_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);       
    
    /* ADC��ʼ�� */
    nrfx_saadc_config_t txSAADC_Config = NRFX_SAADC_DEFAULT_CONFIG;    
    error_code |= nrfx_saadc_init(&txSAADC_Config,
                                vSAADC_Event_Handler);
    
    /* ͨ����ʼ�� */
    error_code |= nrfx_saadc_channel_init(0,&txSAADC_Channel_0_Config);
    error_code |= nrfx_saadc_channel_init(1,&txSAADC_Channel_1_Config);
    error_code |= nrfx_saadc_channel_init(2,&txSAADC_Channel_2_Config);    
    error_code |= nrfx_saadc_channel_init(3,&txSAADC_Channel_3_Config);      
    
    
    return error_code;
}