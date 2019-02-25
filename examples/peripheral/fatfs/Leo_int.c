/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_int     
 * ƽ̨    ��nRF52832
 * ����    ���ж϶��弰ʵ��  
 * ����    �����ɱ�
**********************************************************************/



#include "Leo_int.h"
		

extern uint8_t		G_MPU9255_Data_IsValid;
extern uint32_t	    G_MPU9255_Counter; 

extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;

extern uint8_t		G_SDCard_IsSaved;

/**********************************************************************************************
* ��  ��: MPU9255 �ж� ��Ӧ�¼�
* ��  ��: ��	
* ����ֵ: ��
***********************************************************************************************/ 
static void Leo_INT_MPU9255_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{

	G_MPU9255_Data_IsValid = 1;
    G_MPU9255_Counter++;
}



/**********************************************************************************************
* ��  ��: �ⲿ�������ƴ洢 ��Ӧ�¼�
* ��  ��: ��	
* ����ֵ: ��
***********************************************************************************************/ 
static void Leo_INT_SDCard_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    
    //if(nrf_gpio_pin_read(Leo_INT_SDCard)== 0)
    if(nrfx_gpiote_in_is_set(Leo_INT_SDCard)== 0)
    {   
        if(G_SDCard_IsSaved == 1)
        {
            //���ݴ洢ֹͣ�����ر��ļ�(�����߳���ʵʩ)       
            G_SDCard_IsSaved = 0;            
        }        
    }
}

/**********************************************************************************************
* ��  ��: GPS 1PPS������ �ж� ��Ӧ�¼�
* ��  ��: ��	
* ����ֵ: ��
***********************************************************************************************/ 
static void Leo_INT_GPSPPS_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    //�յ�GPS 1PPS������
    G_GPSWeekSecond++;
    G_MicroSecond = 0;
		if(G_SDCard_IsSaved == 1)
		{
			nrf_gpio_pin_toggle(Leo_nRF52_LED_GREEN);
		}
}

/**********************************************************************************************
* ��  �� : 	MPU9255 �жϳ�ʼ��
* ��  �� :	��	
* ����ֵ : 	��
***********************************************************************************************/ 
uint8_t Leo_INT_MPU9255_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//��������Ч
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;																						//���� ��̬�͵�ƽ
	
	err_code = nrf_drv_gpiote_in_init(Leo_INT_MPU9255, &in_config, Leo_INT_MPU9255_Handle);
  APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}


/**********************************************************************************************
* ��  �� : 	�ⲿ�������ƴ洢 �жϳ�ʼ��
* ��  �� :	��	
* ����ֵ : 	��
***********************************************************************************************/ 
uint8_t Leo_INT_SDCard_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);    	//�½�����Ч
	in_config.pull = NRF_GPIO_PIN_PULLUP;																						// ����  ��̬�ߵ�ƽ
	
	err_code = nrfx_gpiote_in_init(Leo_INT_SDCard, &in_config, Leo_INT_SDCard_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}


/**********************************************************************************************
* ��  �� : 	GPS 1PPS������ �жϳ�ʼ�� �����ش���
* ��  �� :	��	
* ����ֵ : 	��
***********************************************************************************************/ 
uint8_t Leo_INT_GPSPPS_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//��������Ч
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;																						// ���� ��̬�͵�ƽ
    
	err_code = nrfx_gpiote_in_init(Leo_nRF52_GPS_UART_PPS, &in_config, Leo_INT_GPSPPS_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}
