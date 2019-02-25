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
#include "Leo_SDCard.h"

/*
*********************************************************************************************************
*                                       中断使用的 全局变量
*********************************************************************************************************
*/

extern uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件
extern uint32_t         G_GPSWeekSecond;
extern uint16_t         G_MicroSecond;

extern TaskHandle_t    xTaskHandle_SDCard_Open;         /*SDCard 新建文件任务  句柄 */
extern TaskHandle_t    xTaskHandle_SDCard_Close;         /*SDCard 关闭文件任务  句柄 */









//extern uint32_t	        G_MPU9255_Counter; 



//extern TaskHandle_t     xTaskHandle_MPU9255_RxData;
//extern TaskHandle_t     xTaskHandle_DataSave_Start;         /*开始存储数据           句柄 */
//extern TaskHandle_t     xTaskHandle_DataSave_End;





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
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(nrf_gpio_pin_read(configGPIO_INT_SDCard) == 0)
    {
        if(G_SDCard_FileIsOpen == 1)
        {
            //通知 关闭文件操作任务
            xTaskNotifyFromISR(xTaskHandle_SDCard_Close,    
                                0,           
                                eNoAction,
                                &xHigherPriorityTaskWoken);            
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        
        nrfx_gpiote_out_toggle(configGPIO_LED_R);
        //任务分析
//        NRF_LOG_INFO("TEST:   SDCard INT is ok!");
//        NRF_LOG_FLUSH();
//        
//        uint8_t pcWriteBuffer[300];
//        NRF_LOG_INFO("=================================================");
//        NRF_LOG_INFO("\nname      namestate  priority   rest   number");
//        vTaskList((char *)&pcWriteBuffer);
//        NRF_LOG_INFO("\n%s",pcWriteBuffer);
//        NRF_LOG_FLUSH();
//        
//        NRF_LOG_INFO("=================================================");
//        NRF_LOG_INFO("\nname       counter         reate");
//        vTaskGetRunTimeStats((char *)&pcWriteBuffer);
//        NRF_LOG_RAW_INFO("\n%s",pcWriteBuffer);
//        NRF_LOG_FLUSH();  
        
        
    }   
//        
//        
//        
//        nrf_delay_ms(2);
//        if(nrf_gpio_pin_read(configGPIO_INT_SDCard) == 0)
//        {
//            NRF_LOG_INFO("Fuck");
//            NRF_LOG_FLUSH(); 
//            nrfx_gpiote_out_toggle(configGPIO_LED_R);
//            
//            if(G_SDCard_FileIsOpen == 0)
//            {
//                //通知 打开文件操作任务        
//                xTaskNotifyFromISR(xTaskHandle_SDCard_Open,    
//                                    0,           
//                                    eNoAction,
//                                    &xHigherPriorityTaskWoken);            
//                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//                
//            }else
//            {
//                //通知 关闭文件操作任务
//                xTaskNotifyFromISR(xTaskHandle_SDCard_Close,    
//                                    0,           
//                                    eNoAction,
//                                    &xHigherPriorityTaskWoken);            
//                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//            }
//        }

//        
//    }
//    
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if(G_Ctrl_DataSave == 0)
//    {
//        xTaskNotifyFromISR(xTaskHandle_DataSave_Start,      /* 目标任务 */
//                           0,                               /* 发送数据 */
//                           eSetValueWithOverwrite,          /* 如果目标任务上次的数据还没有处理，上次的数据会被覆盖 */
//                            &xHigherPriorityTaskWoken);
//         /* 如果xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        nrfx_gpiote_out_clear(configGPIO_LED_G);
//    }else
//    {
//        xTaskNotifyFromISR(xTaskHandle_DataSave_End,        /* 目标任务 */
//                           0,                               /* 发送数据 */
//                           eSetValueWithOverwrite,          /* 如果目标任务上次的数据还没有处理，上次的数据会被覆盖 */
//                            &xHigherPriorityTaskWoken);
//         /* 如果xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        nrfx_gpiote_out_set(configGPIO_LED_G);
//    }

//    

  

    
}


/*-----------------------------------------------------------------------*/
/* SDCard 存储暂停中断 初始化                                             */
/*----------------------------------------------------------------------*/
uint8_t ucINTInital_SDCard(void)
{	
	uint8_t err_code = 0;	
    nrfx_gpiote_in_config_t txINTConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false);            //下降沿有效
    txINTConfig.pull = NRF_GPIO_PIN_PULLUP;		                                                // 上拉  常态高电平
    err_code = nrfx_gpiote_in_init(configGPIO_INT_SDCard, &txINTConfig, vINTHandler_SDCard);
    return err_code;
}


/*-----------------------------------------------------------------------*/
/* SDCard 存储暂停中断 启动                                               */
/*-----------------------------------------------------------------------*/
uint8_t ucINTStart_SDCard(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_SDCard, true);
    return 0;
}



///*
//*********************************************************************************************************
//*                                       MPU9255 数据接收 中断
//*********************************************************************************************************
//*/

///*-----------------------------------------------------------------------*/
///* MPU9255 中断 响应事件                                                 */
///*----------------------------------------------------------------------*/
//static void vINTHandler_MPU9255(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
//{
//    //计数器+1
//    if( G_MPU9255_Counter == 255)
//    {
//        G_MPU9255_Counter = 0;
//    }else
//    {
//        G_MPU9255_Counter++;
//    }

//    //通知任务进行数据采集
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xTaskNotifyFromISR(xTaskHandle_MPU9255_RxData,      /* 目标任务 */
//                       0,                               /* 发送数据 */
//                       eSetValueWithOverwrite,           /* 如果目标任务上次的数据还没有处理，上次的数据会被覆盖 */
//                        &xHigherPriorityTaskWoken);
//     /* 如果xHigherPriorityTaskWoken = pdTRUE，那么退出中断后切到当前最高优先级任务执行 */
//     portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}

///*-----------------------------------------------------------------------*/
///* MPU9255 中断初始化                                                    */
///*----------------------------------------------------------------------*/ 
//uint8_t ucINTInital_MPU9255(void)
//{	
//	uint8_t err_code;	
//	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
//	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //下拉 常态低电平
//	
//	err_code = nrfx_gpiote_in_init(configGPIO_INT_MPU9255, &in_config, vINTHandler_MPU9255);	
//	return err_code;
//}

///*-----------------------------------------------------------------------*/
///* MPU9255 中断 启动                                                     */
///*----------------------------------------------------------------------*/
//uint8_t ucINTStart_MPU9255(void)
//{	
//    nrfx_gpiote_in_event_enable(configGPIO_INT_MPU9255, true);
//    return 0;
//}



/*
*********************************************************************************************************
*                                       GPS 1pps 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* 1pps 中断 响应事件                                                    */
/*----------------------------------------------------------------------*/

static void vINTHandler_PPS(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    //收到GPS 1PPS秒脉冲
    G_GPSWeekSecond++;
    G_MicroSecond = 0;
//		if(G_SDCard_IsSaved == 1)
//		{
//			nrf_gpio_pin_toggle(Leo_nRF52_LED_GREEN);
//		}
}


/*-----------------------------------------------------------------------*/
/* 1pps 中断初始化                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_PPS(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //下拉 常态低电平
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_GPSPPS, &in_config, vINTHandler_PPS);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* 1pps 中断 启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_PPS(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_GPSPPS, true);
    return 0;
}


/*******************************************************************************************************/



















