
#include "Leo_BatteryCheck.h"

#define SAMPLES_IN_BUFFER 1
static nrf_saadc_value_t    m_buffer_pool[SAMPLES_IN_BUFFER];
static uint32_t             m_adc_evt_counter;


static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
}




//@brief ��������ʼ��
/*--------------------------------------------------------------------------*/
//<*����˵��:    ��
//<*����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_BatteryCheck_Initial(void)
{
    uint8_t error_code = 0;	
    nrfx_saadc_config_t saadc_config = NRFX_SAADC_DEFAULT_CONFIG;   
    error_code = nrfx_saadc_init(&saadc_config, saadc_event_handler);
    APP_ERROR_CHECK(error_code);    
    
     nrf_saadc_channel_config_t saadc_channel_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);    
    error_code |= nrfx_saadc_channel_init(0, &saadc_channel_config);
    APP_ERROR_CHECK(error_code);   
    return error_code;
}


//@brief ������� 
/*--------------------------------------------------------------------------*/
//<*����˵��:    ��
//<*����ֵ˵��: 
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_BatteryCheck(float* mVolt)
{
    uint8_t error_code = 0;	
    nrf_saadc_value_t saadc_val;       
    error_code = nrfx_saadc_sample_convert(0,&saadc_val);

    *mVolt = saadc_val*3.6/1024.0;
    
    return error_code;
}