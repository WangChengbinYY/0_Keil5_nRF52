/*
*********************************************************************************************************
*
*    模块名称 : 外部硬件中断配置
*    文件名称 :Leo_INT
*    版    本 : V1.0
*    说    明 : 外部硬件中断设置相关
*
*    修改记录 :
*        版本号    日期          作者     
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
*                                       中断使用的 全局变量
*********************************************************************************************************
*/

extern uint8_t      G_SDCard_FileIsOpen;               //标记是否已经打开文件
extern uint32_t     G_GPSWeekSecond;
extern uint16_t     G_MicroSecond;

extern uint8_t	    G_IMU_Data_A[27];                   //第一组IMU_A(U4)存放的数据
extern uint8_t	    G_IMU_Data_B[27];                   //第二组IMU_A(U5)存放的数据

extern TaskHandle_t xTaskHandle_SDCard_Close;         /*SDCard 关闭文件任务  句柄 */
extern TaskHandle_t xTaskHandle_UWB_EventHandler;    
extern TaskHandle_t xTaskHandle_CollectData_IMUA;    
extern TaskHandle_t xTaskHandle_CollectData_IMUB;    

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
        nrf_delay_ms(100);
        if(nrf_gpio_pin_read(configGPIO_INT_SDCard) == 0)
        {
            if(G_SDCard_FileIsOpen == 1)
            {
                //标志位 清零
                G_SDCard_FileIsOpen = 0;
                
                //通知 关闭文件操作任务
                xTaskNotifyFromISR(xTaskHandle_SDCard_Close,0,eNoAction,&xHigherPriorityTaskWoken);            
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            

            //任务分析
            
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



/*
*********************************************************************************************************
*                                       IMU_A(U4) 数据接收 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* IMU_A(U4) 中断 响应事件                                                 */
/*----------------------------------------------------------------------*/
static void vINTHandler_IMUA(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(G_SDCard_FileIsOpen == 1)
    {
        //记录时间
        memcpy(G_IMU_Data_A+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
        memcpy(G_IMU_Data_A+6,&G_MicroSecond,sizeof(G_MicroSecond));       
        
        //通知任务进行数据采集
        xTaskNotifyFromISR(xTaskHandle_CollectData_IMUA,0,eNoAction,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    
    }
}

/*-----------------------------------------------------------------------*/
/* IMU_A(U4) 中断初始化                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_IMUA(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //下拉 常态低电平
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_IMUA, &in_config, vINTHandler_IMUA);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* IMU_A(U4) 中断 启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_IMUA(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_IMUA, true);
    return 0;
}



/*
*********************************************************************************************************
*                                       IMU_B(U5) 数据接收 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* IMU_B(U5) 中断 响应事件                                                 */
/*----------------------------------------------------------------------*/
static void vINTHandler_IMUB(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t actio)
{
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(G_SDCard_FileIsOpen == 1)
    {
        //记录时间
        memcpy(G_IMU_Data_B+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
        memcpy(G_IMU_Data_B+6,&G_MicroSecond,sizeof(G_MicroSecond));    
        
        //通知任务进行数据采集
        xTaskNotifyFromISR(xTaskHandle_CollectData_IMUB,0,eNoAction,&xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    
    }
}

/*-----------------------------------------------------------------------*/
/* IMU_B(U5) 中断初始化                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_IMUB(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_PULLDOWN;											    //下拉 常态低电平
	
	err_code = nrfx_gpiote_in_init(configGPIO_INT_IMUB, &in_config, vINTHandler_IMUB);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* IMU_B(U5)) 中断 启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_IMUB(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_IMUB, true);
    return 0;
}







/*
*********************************************************************************************************
*                                       GPS 1pps 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* 1pps 中断 响应事件                                                    */
/*----------------------------------------------------------------------*/

static void vINTHandler_PPS(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    //收到GPS 1PPS秒脉冲
    G_GPSWeekSecond++;
    G_MicroSecond = 0;
    if(G_SDCard_FileIsOpen == 1)
    {
        nrf_gpio_pin_toggle(configGPIO_LED_R);
    }
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


extern uint16_t    RXG_MicroSecond; 


/*
*********************************************************************************************************
*                                       UWB 中断
*********************************************************************************************************
*/

/*-----------------------------------------------------------------------*/
/* UWB 中断 响应事件                                                    */
/*----------------------------------------------------------------------*/

static void vINTHandler_UWB(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{ 
    if(G_SDCard_FileIsOpen == 1)
    {
        RXG_MicroSecond = G_MicroSecond;
        
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(xTaskHandle_UWB_EventHandler,0,eNoAction,&xHigherPriorityTaskWoken);            
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}


/*-----------------------------------------------------------------------*/
/* UWB中断初始化                                                    */
/*----------------------------------------------------------------------*/ 
uint8_t ucINTInital_UWB(void)
{	
	uint8_t err_code;	
	nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);    	//上升沿有效
	in_config.pull = NRF_GPIO_PIN_NOPULL;											    //下拉 常态低电平
    
	err_code = nrfx_gpiote_in_init(configGPIO_INT_UWB, &in_config, vINTHandler_UWB);	
	return err_code;
}

/*-----------------------------------------------------------------------*/
/* UWB中断 启动                                                     */
/*----------------------------------------------------------------------*/
uint8_t ucINTStart_UWB(void)
{	
    nrfx_gpiote_in_event_enable(configGPIO_INT_UWB, true);
    return 0;
}


/*******************************************************************************************************/












