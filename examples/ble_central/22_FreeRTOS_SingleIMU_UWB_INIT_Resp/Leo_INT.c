/*
*********************************************************************************************************
*
*    ģ������ : �ⲿӲ���ж�����
*    �ļ����� :Leo_INT
*    ��    �� : V1.0
*    ˵    �� : �ⲿӲ���ж��������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-14     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_INT.h"
#include "Leo_SDCard.h"

/*
*********************************************************************************************************
*                                       �ж�ʹ�õ� ȫ�ֱ���
*********************************************************************************************************
*/

extern uint8_t      G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ�
extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;
//extern uint8_t      G_UWBData_IsComing;

extern TaskHandle_t xTaskHandle_SDCard_Close;         /*SDCard �ر��ļ�����  ��� */
extern TaskHandle_t xTaskHandle_UWB_EventHandler;    



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

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(nrf_gpio_pin_read(configGPIO_INT_SDCard) == 0)
    {
        nrf_delay_ms(100);
        if(nrf_gpio_pin_read(configGPIO_INT_SDCard) == 0)
        {
            if(G_SDCard_FileIsOpen == 1)
            {
                //��־λ ����
                G_SDCard_FileIsOpen = 0;
                
                //֪ͨ �ر��ļ���������
                xTaskNotifyFromISR(xTaskHandle_SDCard_Close,0,eNoAction,&xHigherPriorityTaskWoken);            
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            

            //�������
            
            uint8_t pcWriteBuffer[300];
            NRF_LOG_INFO("=================================================");
            NRF_LOG_INFO("\nname      namestate  priority   rest   number");
            vTaskList((char *)&pcWriteBuffer);
            NRF_LOG_INFO("\n%s",pcWriteBuffer);
            NRF_LOG_FLUSH();
            
            NRF_LOG_INFO("=================================================");
            NRF_LOG_INFO("\nname       counter         reate");
            vTaskGetRunTimeStats((char *)&pcWriteBuffer);
            NRF_LOG_RAW_INFO("\n%s",pcWriteBuffer);
            NRF_LOG_FLUSH();             
            
        }        
    }   

    
}


/*-----------------------------------------------------------------------*/
/* SDCard �洢��ͣ�ж� ��ʼ��                                             */
/*----------------------------------------------------------------------*/
uint8_t ucINTInital_SDCard(void)
{	
	uint8_t err_code = 0;	
    nrfx_gpiote_in_config_t txINTConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);            //�½�����Ч
    txINTConfig.pull = NRF_GPIO_PIN_PULLUP;		                                                // ����  ��̬�ߵ�ƽ
    err_code = nrfx_gpiote_in_init(configGPIO_INT_SDCard, &txINTConfig, vINTHandler_SDCard);
    return err_code;
}


/*-----------------------------------------------------------------------*/
/* SDCard �洢��ͣ�ж� ����                                               */
/*-----------------------------------------------------------------------*/
uint8_t ucINTStart_SDCard(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_SDCard, true);
    return 0;
}



///*
//*********************************************************************************************************
//*                                       MPU9255 ���ݽ��� �ж�
//*********************************************************************************************************
//*/

///*-----------------------------------------------------------------------*/
///* MPU9255 �ж� ��Ӧ�¼�                                                 */
///*----------------------------------------------------------------------*/
//static void vINTHandler_MPU9255(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
//{
//    //������+1
//    if( G_MPU9255_Counter == 255)
//    {
//        G_MPU9255_Counter = 0;
//    }else
//    {
//        G_MPU9255_Counter++;
//    }

//    //֪ͨ����������ݲɼ�
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xTaskNotifyFromISR(xTaskHandle_MPU9255_RxData,      /* Ŀ������ */
//                       0,                               /* �������� */
//                       eSetValueWithOverwrite,           /* ���Ŀ�������ϴε����ݻ�û�д����ϴε����ݻᱻ���� */
//                        &xHigherPriorityTaskWoken);
//     /* ���xHigherPriorityTaskWoken = pdTRUE����ô�˳��жϺ��е���ǰ������ȼ�����ִ�� */
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

///*-----------------------------------------------------------------------*/
///* MPU9255 �жϳ�ʼ��                                                    */
///*----------------------------------------------------------------------*/ 
//uint8_t ucINTInital_MPU9255(void)
//{	
//	uint8_t err_code;	
//	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//��������Ч
//	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //���� ��̬�͵�ƽ
//	
//	err_code = nrfx_gpiote_in_init(configGPIO_INT_MPU9255, &in_config, vINTHandler_MPU9255);	
//	return err_code;
//}

///*-----------------------------------------------------------------------*/
///* MPU9255 �ж� ����                                                     */
///*----------------------------------------------------------------------*/
//uint8_t ucINTStart_MPU9255(void)
//{	
//    nrfx_gpiote_in_event_enable(configGPIO_INT_MPU9255, true);
//    return 0;
//}



/*
*********************************************************************************************************
*                                       GPS 1pps �ж�
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* 1pps �ж� ��Ӧ�¼�                                                    */
/*----------------------------------------------------------------------*/

static void vINTHandler_PPS(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //�յ�GPS 1PPS������
//    G_GPSWeekSecond++;
//    G_MicroSecond = 0;
    if(G_SDCard_FileIsOpen == 1)
    {
        nrf_gpio_pin_toggle(configGPIO_LED_R);
    }
}


/*-----------------------------------------------------------------------*/
/* 1pps �жϳ�ʼ��                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_PPS(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);    	//��������Ч
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //���� ��̬�͵�ƽ
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_GPSPPS, &in_config, vINTHandler_PPS);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* 1pps �ж� ����                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_PPS(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_GPSPPS, true);
    return 0;
}


/*******************************************************************************************************/





/*
*********************************************************************************************************
*                                       UWB �ж�
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* UWB �ж� ��Ӧ�¼�                                                    */
/*----------------------------------------------------------------------*/

static void vINTHandler_UWB(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{ 
    if(G_SDCard_FileIsOpen == 1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(xTaskHandle_UWB_EventHandler,0,eNoAction,&xHigherPriorityTaskWoken);            
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        G_UWBData_IsComing = 1;
    }
}


/*-----------------------------------------------------------------------*/
/* UWB�жϳ�ʼ��                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_UWB(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//��������Ч
	in_config.pull = NRF_GPIO_PIN_NOPULL;											    //���� ��̬�͵�ƽ
    
	err_code = nrfx_gpiote_in_init(configGPIO_INT_UWB, &in_config, vINTHandler_UWB);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* UWB�ж� ����                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_UWB(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_UWB, true);
    return 0;
}


/*******************************************************************************************************/












