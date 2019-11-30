/*
*********************************************************************************************************
*
*    模块名称 : 外部 AD 数据的采集
*    文件名称 :Leo_SAADC
*    版    本 : V1.0
*    说    明 : 外部 AD采集
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-10     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_SAADC.h"




/*------------------------------------------------------------
 *SAADC 事件回调函数                                                                       
 *------------------------------------------------------------*/
static void vSAADC_Event_Handler(nrfx_saadc_evt_t const * p_event)
{
    
}



/**
 * ADC 初始化
 *   0 成功； 1 失败
*/
uint8_t ucSAADCInitial(void)
{
    uint8_t error_code = 0;  
    
//通道参数配置
    /* 配置通道0 对应 后脚跟点6，管脚为 P0.03  AIN1 */
    nrf_saadc_channel_config_t txSAADC_Channel_0_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);    
    /* 配置通道1 对应 后脚跟点7，管脚为 P0.04  AIN2 */
    nrf_saadc_channel_config_t txSAADC_Channel_1_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);  
    /* 配置通道2 对应 前脚掌点5，管脚为 P0.31  AIN7 */
    nrf_saadc_channel_config_t txSAADC_Channel_2_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);      
    /* 配置通道3 对应 前脚掌点2，管脚为 P0.30  AIN6 */
    nrf_saadc_channel_config_t txSAADC_Channel_3_Config = 
        NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6);       
    
    /* ADC初始化 */
    nrfx_saadc_config_t txSAADC_Config = NRFX_SAADC_DEFAULT_CONFIG;    
    error_code |= nrfx_saadc_init(&txSAADC_Config,
                                vSAADC_Event_Handler);
    
    /* 通道初始化 */
    error_code |= nrfx_saadc_channel_init(0,&txSAADC_Channel_0_Config);
    error_code |= nrfx_saadc_channel_init(1,&txSAADC_Channel_1_Config);
    error_code |= nrfx_saadc_channel_init(2,&txSAADC_Channel_2_Config);    
    error_code |= nrfx_saadc_channel_init(3,&txSAADC_Channel_3_Config);      
    
    
    return error_code;
}