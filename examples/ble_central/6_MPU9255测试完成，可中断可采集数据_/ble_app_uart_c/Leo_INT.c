/*
*********************************************************************************************************
*
*    模块名称 : 外部硬件中断配置
*    文件名称 : Leo_INT
*    版    本 : V1.0
*    说    明 : 外部硬件中断设置相关
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-14     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_INT.h"


/*
*********************************************************************************************************
*                                       中断使用的 全局变量
*********************************************************************************************************
*/
extern uint8_t		G_MPU9255_Data_IsValid;
extern uint32_t	    G_MPU9255_Counter; 

extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;

extern uint8_t		G_SDCard_IsSaved;


/*
*********************************************************************************************************
*                                       SDCard 存储暂停 外部中断
*********************************************************************************************************
*/


/*-----------------------------------------------------------------------*/
/* SDCard 存储暂停中断 响应函数                                           */
/*----------------------------------------------------------------------*/
static void vINTHandler_SDCard(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

/*    
    if(nrfx_gpiote_in_is_set(Leo_INT_SDCard)== 0)
    {   
        if(G_SDCard_IsSaved == 1)
        {
            //数据存储停止，并关闭文件(在主线程中实施)       
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
/* SDCard 存储暂停中断 初始化                                             */
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
/* SDCard 存储暂停中断 启动                                               */
/*-----------------------------------------------------------------------*/
void vINTStart_SDCard(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_SDCard, true);
}



/*
*********************************************************************************************************
*                                       MPU9255 数据接收 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* MPU9255 中断 响应事件                                                 */
/*----------------------------------------------------------------------*/
static void Leo_INT_MPU9255_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{

	G_MPU9255_Data_IsValid = 1;
    G_MPU9255_Counter++;
}

/*-----------------------------------------------------------------------*/
/* MPU9255 中断初始化                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t Leo_INT_MPU9255_Initial(void)
{	
	nrfx_err_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											//下拉 常态低电平
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_MPU9255, &in_config, Leo_INT_MPU9255_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}

/*-----------------------------------------------------------------------*/
/* MPU9255 中断 启动                                             */
/*----------------------------------------------------------------------*/
void vINTStart_MPU9255(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_MPU9255, true);
}



