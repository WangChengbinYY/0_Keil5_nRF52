/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_int     
 * 平台    ：nRF52832
 * 描述    ：中断定义及实现  
 * 作者    ：王成宾
**********************************************************************/



#include "Leo_int.h"
		

extern uint8_t		G_MPU9255_Data_IsValid;
extern uint32_t	    G_MPU9255_Counter; 

extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;

extern uint8_t		G_SDCard_IsSaved;

/**********************************************************************************************
* 描  述: MPU9255 中断 响应事件
* 入  参: 无	
* 返回值: 无
***********************************************************************************************/ 
static void Leo_INT_MPU9255_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{

	G_MPU9255_Data_IsValid = 1;
    G_MPU9255_Counter++;
}



/**********************************************************************************************
* 描  述: 外部按键控制存储 响应事件
* 入  参: 无	
* 返回值: 无
***********************************************************************************************/ 
static void Leo_INT_SDCard_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    
    //if(nrf_gpio_pin_read(Leo_INT_SDCard)== 0)
    if(nrfx_gpiote_in_is_set(Leo_INT_SDCard)== 0)
    {   
        if(G_SDCard_IsSaved == 1)
        {
            //数据存储停止，并关闭文件(在主线程中实施)       
            G_SDCard_IsSaved = 0;            
        }        
    }
}

/**********************************************************************************************
* 描  述: GPS 1PPS秒脉冲 中断 响应事件
* 入  参: 无	
* 返回值: 无
***********************************************************************************************/ 
static void Leo_INT_GPSPPS_Handle(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    //收到GPS 1PPS秒脉冲
    G_GPSWeekSecond++;
    G_MicroSecond = 0;
		if(G_SDCard_IsSaved == 1)
		{
			nrf_gpio_pin_toggle(Leo_nRF52_LED_GREEN);
		}
}

/**********************************************************************************************
* 描  述 : 	MPU9255 中断初始化
* 入  参 :	无	
* 返回值 : 	无
***********************************************************************************************/ 
uint8_t Leo_INT_MPU9255_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;																						//下拉 常态低电平
	
	err_code = nrf_drv_gpiote_in_init(Leo_INT_MPU9255, &in_config, Leo_INT_MPU9255_Handle);
  APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}


/**********************************************************************************************
* 描  述 : 	外部按键控制存储 中断初始化
* 入  参 :	无	
* 返回值 : 	无
***********************************************************************************************/ 
uint8_t Leo_INT_SDCard_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);    	//下降沿有效
	in_config.pull = NRF_GPIO_PIN_PULLUP;																						// 上拉  常态高电平
	
	err_code = nrfx_gpiote_in_init(Leo_INT_SDCard, &in_config, Leo_INT_SDCard_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}


/**********************************************************************************************
* 描  述 : 	GPS 1PPS秒脉冲 中断初始化 上升沿触发
* 入  参 :	无	
* 返回值 : 	无
***********************************************************************************************/ 
uint8_t Leo_INT_GPSPPS_Initial(void)
{	
	nrfx_err_t err_code;	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;																						// 下拉 常态低电平
    
	err_code = nrfx_gpiote_in_init(Leo_nRF52_GPS_UART_PPS, &in_config, Leo_INT_GPSPPS_Handle);
    APP_ERROR_CHECK(err_code);
	
	return (uint8_t)err_code;
}
