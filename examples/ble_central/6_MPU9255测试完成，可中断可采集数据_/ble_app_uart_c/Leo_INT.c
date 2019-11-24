/*
*********************************************************************************************************
*
*    ģ������ : �ⲿӲ���ж�����
*    �ļ����� : Leo_INT
*    ��    �� : V1.0
*    ˵    �� : �ⲿӲ���ж��������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-14     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_INT.h"


/*
*********************************************************************************************************
*                                       �ж�ʹ�õ� ȫ�ֱ���
*********************************************************************************************************
*/
extern uint8_t		G_MPU9255_Data_IsValid;
extern uint32_t	    G_MPU9255_Counter; 

extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;

extern uint8_t		G_SDCard_IsSaved;


/*
*********************************************************************************************************
*                                       SDCard �洢��ͣ �ⲿ�ж�
*********************************************************************************************************
*/


/*-----------------------------------------------------------------------*/
/* SDCard �洢��ͣ�ж� ��Ӧ����                                           */
/*----------------------------------------------------------------------*/
static void vINTHandler_SDCard(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

/*    
    if(nrfx_gpiote_in_is_set(Leo_INT_SDCard)== 0)
    {   
        if(G_SDCard_IsSaved == 1)
        {
            //���ݴ洢ֹͣ�����ر��ļ�(�����߳���ʵʩ)       
            G_SDCard_IsSaved = 0;            
        }        
    }
*/
    
/* 
 * Debug_Test       */
#if Leo_Debug
    NRF_LOG_INFO("TEST:   SDCard INT is ok!");
    NRF_LOG_FLUSH();
    
    uint8_t pcWriteBuffer[300];
    NRF_LOG_INFO("=================================================");
    NRF_LOG_INFO("name      namestate  priority   rest   number");
    vTaskList((char *)&pcWriteBuffer);
    NRF_LOG_INFO("%s",pcWriteBuffer);
    NRF_LOG_FLUSH();
    
    NRF_LOG_INFO("=================================================");
    NRF_LOG_INFO("name       counter         reate");
    vTaskGetRunTimeStats((char *)&pcWriteBuffer);
    NRF_LOG_RAW_INFO("%s",pcWriteBuffer);
    NRF_LOG_FLUSH();    
#endif



}


/*-----------------------------------------------------------------------*/
/* SDCard �洢��ͣ�ж� ��ʼ��                                             */
/*----------------------------------------------------------------------*/
void vINTInital_SDCard(void)
{	
	nrfx_err_t err_code;	
    nrfx_gpiote_in_config_t txINTConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);
    txINTConfig.pull = NRF_GPIO_PIN_PULLUP;		
    err_code = nrfx_gpiote_in_init(configGPIO_INT_SDCard, &txINTConfig, vINTHandler_SDCard);
    APP_ERROR_CHECK(err_code);
}


/*-----------------------------------------------------------------------*/
/* SDCard �洢��ͣ�ж� ����                                               */
/*-----------------------------------------------------------------------*/
void vINTStart_SDCard(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_SDCard, true);
}



/*
*********************************************************************************************************
*                                       MPU9255 ���ݽ��� �ж�
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* MPU9255 �ж� ��Ӧ�¼�                                                 */
/*----------------------------------------------------------------------*/
static void Leo_INT_MPU9255_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{

	G_MPU9255_Data_IsValid = 1;
    G_MPU9255_Counter++;
}

/*-----------------------------------------------------------------------*/
/* MPU9255 �жϳ�ʼ��                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t Leo_INT_MPU9255_Initial(void)
{	
	nrfx_err_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);    	//��������Ч
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											//���� ��̬�͵�ƽ
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_MPU9255, &in_config, Leo_INT_MPU9255_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}

/*-----------------------------------------------------------------------*/
/* MPU9255 �ж� ����                                             */
/*----------------------------------------------------------------------*/
void vINTStart_MPU9255(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_MPU9255, true);
}



