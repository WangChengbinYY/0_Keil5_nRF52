/*
*********************************************************************************************************
*
*    模块名称 : FreeRTOS多任务实现
*    文件名称 : Leo_FreeRTOS_TASK
*    版    本 : V1.0
*    说    明 : 项目中所有任务的建立
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-19     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/

#include "Leo_FreeRTOS_TASK.h"
#include "Leo_INT.h"
#include "Leo_TIMER.h"
#include "Leo_SDCard.h"

/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */
#define taskPRIO_INIT                        0          /* 系统初始化：仅在开始执行一次，初始化失败则进入死循环LED不听闪烁*/
#define taskPRIO_LED_SLOW                    0          /* LED慢闪(1s一闪)：(代表GPS初始定位成功，并获取时间)   */
#define taskPRIO_LED_QUICK                   0          /* LED快闪(500ms)：(代表SDCard的存储或停止操作成功)   */

/* 1级 */
#define taskPRIO_GPS_RxData                  1          //接收GPS数据并解析，解析成功，通知存储 
#define taskPRIO_SDCard_Start                1          //SDCard建立文件成功  标志位置1  数据会被存储
#define taskPRIO_SDCard_End                  1          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 2级 */
#define taskPRIO_SDCard_Save                 2          //SDCard存储数据

/* 3级 */
#define taskPRIO_CollectData                 3          //采集IMU数据、压力数据，成功则通知存储

/* 4级 */
#define taskPRIO_UWB_RxData                  4          //采集UWB测距数据，成功则通知存储




/*======================================= 全局变量定义 待定！！！！！！==================================================*/
/* 全局变量_时间参数 */
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0

/* 全局变量_SDCard文件操作标识 */                                                        
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0

/* 全局变量_IMU数据采集的 SPI2 实例（UWB 用SPI0；SDCard 用SPI1； IMU 和 压力传感器 用SPI2） */
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);  			//MPU9255使用的SPI实例		
uint8_t         G_MPU9255_SPI_xfer_Done = 1;


uint8_t	    G_MPU9255_MAG_ASAXYZ[6];            //AK8963磁强计读出的灵敏度修正参数数据 2~4
uint8_t	    G_MPU9255_MAG_ASAXYZ_IsValid;       /*当前数据是否有效 1有效 0无效*/

uint8_t     G_WRONG_Record[10];                 //采集过程数据出错的记录

uint8_t	    G_MPU9255_Data[28];                 //MPU9255传感器存放的数据
uint8_t	    G_MPU9255_Counter;                  //MPU9255中断触发的计数器

//uint8_t     G_GPS_Data[33];                     //GPS接收数据

//uint8_t     G_FOOTPressure_Data[25];            //足部压力传感器数据
//uint8_t	    G_FOOTPressure_Counter;             //足部压力传感器数据的计数器

//uint8_t     G_UWBDistance_Data[12];             //UWB测距传感器数据
//uint8_t	    G_UWBDistance_Counter;              //UWB测距传感器数据的计数器


//uint8_t	    G_SDCDBuffer1[1024];               //双缓存buffer
//uint16_t	G_SDCDBuffer1_NUM;	
//uint8_t	    G_SDCDBuffer2[1024];
//uint16_t	G_SDCDBuffer2_NUM;	

///**
// * 全局变量_系统控制   待完善
//*/
//uint8_t     G_Ctrl_DataSave;                    /* 数据存储控制标志位 1存储，0不存储 */


/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_TaskINIT            = NULL;         /*系统初始化任务       句柄 */
TaskHandle_t    xTaskHandle_LEDSLOW             = NULL;
TaskHandle_t    xTaskHandle_LEDQUICK            = NULL;         //3s快闪
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*GPS数据采集任务       句柄*/
TaskHandle_t    xTaskHandle_SDCard_Start        = NULL;         /*开始存储数据           句柄 */
TaskHandle_t    xTaskHandle_SDCard_End          = NULL;         /*停止存储数据           句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         //SDCard存储数据
TaskHandle_t    xTaskHandle_CollectData         = NULL;         //采集数据 IMU 压力
TaskHandle_t    xTaskHandle_UWB_RxData          = NULL;         //采集UWB测距数据


/**
 * 全局变量_资源互斥量  待完善  
 *      要分别考虑几个数据存储的使用，IMU 压力 一组；测距 一组；GPS 一组
*/
SemaphoreHandle_t   xMutex_SDCDBuffer_1         = NULL;



/**
 * 全局变量初始化函数   待完善  
*/
void vInitial_Variable(void)
{
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
                                                        //由 外部GPS的1PPS校准 1PPS触发时 将其置0
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0
    
    
//    
//    
//    //AK8963磁强计读出的灵敏度修正参数 2~4
//    memset(G_MPU9255_MAG_ASAXYZ,0,6);
//	G_MPU9255_MAG_ASAXYZ[0] = 0xC1;
//	G_MPU9255_MAG_ASAXYZ[1] = 0xC2;
//    G_MPU9255_MAG_ASAXYZ[5] = 0xFF;   
//    G_MPU9255_MAG_ASAXYZ_IsValid = 0;
//    
//    //采集过程数据出错的记录 
//    memset(G_WRONG_Record,0,10);
//    G_WRONG_Record[0] = 0xC3;
//	G_WRONG_Record[1] = 0xC4;
//    G_WRONG_Record[9] = 0xFF;
//    
//    //MPU9255传感器存放的数据
//    memset(G_MPU9255_Data,0,28);
//    G_MPU9255_Data[0] = 0xA1;
//	G_MPU9255_Data[1] = 0xA2;
//    G_MPU9255_Data[27] = 0xFF;
//    G_MPU9255_Counter = 0;                              //MPU9255中断触发的计数器
//    
//    //GPS接收数据
//    memset(G_GPS_Data,0,33);
//    G_GPS_Data[0] = 0xA3;
//    G_GPS_Data[1] = 0xA4;
//    G_GPS_Data[32] = 0xFF;

//    //足部压力传感器数据
//    memset(G_FOOTPressure_Data,0,25);
//    G_FOOTPressure_Data[0] = 0xA5;
//	G_FOOTPressure_Data[1] = 0xA6;
//    G_FOOTPressure_Data[24] = 0xFF;
//    G_FOOTPressure_Counter = 0;
//    
//    //UWB测距传感器数据
//    memset(G_UWBDistance_Data,0,12);
//    G_UWBDistance_Data[0] = 0xA7;
//	G_UWBDistance_Data[1] = 0xA8;
//    G_UWBDistance_Data[11] = 0xFF;
//    G_UWBDistance_Counter = 0;
//    
//    //时间计数
//    G_GPSWeekSecond = 0;
//    G_MicroSecond = 0;    

//    //SDCard存储buffer
//    memset(G_SDCDBuffer1,0,1024);
//    memset(G_SDCDBuffer2,0,1024);    
//    G_SDCDBuffer1_NUM = 0;
//    G_SDCDBuffer2_NUM =0;

//    /*系统控制相关*/
//    G_Ctrl_DataSave = 0;
 
}






/*======================================= 任务实现 new==================================================*/
/*------------------------------------------------------------
 *SPI的回调函数                                                                              
 *------------------------------------------------------------*/
//static void Leo_MPU9255_SPI_Event_Handler(nrf_drv_spi_evt_t const * p_event,void * p_context){
//	G_MPU9255_SPI_xfer_Done = 1;
//}



/*------------------------------------------------------------
 *初始化任务   函数  只执行一次                                                                              
 *------------------------------------------------------------*/
static void vTask_StartINIT(void *pvParameters)
{
    uint8_t err_code = 0;
    BaseType_t txResult = pdPASS;

/**
 * 0. 全局变量初始化    
 */    
    vInitial_Variable();    
    
/**
 * 1. GPIO管脚设定    
 */
    err_code |= nrfx_gpiote_init();
    
    /* (1) LED 管脚 */
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
//    nrfx_gpiote_out_clear(configGPIO_LED_R);  //输出0，点亮LED    
//    nrf_delay_ms(5000);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //输出1，LED灯灭    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();
    
    /* (2) INT中断管脚初始化 */    
    err_code |= ucINTInital_SDCard();    /* SDCard中断管脚初始化 */    
    err_code |= ucINTInital_PPS();       /* 1PPS秒脉冲中断管脚初始化 */
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),err_code);   
    NRF_LOG_FLUSH();    

/**
 * 2. 计时器初始化    
 */
    /* (1) 1ms 计时器 初始化 使用的TIMR3 */   
    err_code |= ucTimerInitial_3();      /* TIMER3 计数器初始化*/ 
    err_code |= ucTimerInitial_4();
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),err_code);  
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/    

/**
 * 3. 初始化SDCard 并建立记录文件    
 */
    err_code |= ucSDCard_INIT();   
    err_code |= ucSDCard_OpenFile();      
    
    nrf_delay_ms(1000);
    err_code |= ucSDCard_SaveData(&err_code,sizeof(err_code));
    err_code |= ucSDCard_CloseFile();
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();   
    
/**
 * 4. 初始化 IMU传感器 
 */
    /* (1) 初始化第一个MPU9255_A  */   
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_MPU1_CS;               //第一个IMU的nCS管脚
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//系统SPI中断权限默认设定为 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPI使用的范围为 100KHz~1MHz
    //SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//依据配置参数 对 实例spi 进行初始化 
	err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    NRF_LOG_INFO(("||Initialize||-->SPI_IMU_1------>error  0x%x"),err_code); 
    NRF_LOG_FLUSH();      
    //针对IMU_1初始化
    err_code |= ucMPU9255_INIT();
    NRF_LOG_INFO(("||Initialize||-->MPU9255_A-------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();   
    //卸载SPI
    nrf_drv_spi_uninit(&SPI_CollectData);	
  
    /* (2) 初始化第二个MPU9255_B  */   
 	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_MPU2_CS;               //第一个IMU的nCS管脚
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//系统SPI中断权限默认设定为 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPI使用的范围为 100KHz~1MHz
    //SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//依据配置参数 对 实例spi 进行初始化 
    err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    NRF_LOG_INFO(("||Initialize||-->SPI_IMU_2------>error  0x%x"),err_code); 
    NRF_LOG_FLUSH();      
    //针对IMU_1初始化
    err_code |= ucMPU9255_INIT();
    NRF_LOG_INFO(("||Initialize||-->MPU9255_B-------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();   
    //卸载SPI
    nrf_drv_spi_uninit(&SPI_CollectData);	       
    
   /* (3) 配置正确的SPI数据采集  */   
   //第一个SPI 片选
    nrfx_gpiote_out_uninit(configGPIO_SPI_MPU1_CS);
    err_code |= nrfx_gpiote_out_init(configGPIO_SPI_MPU1_CS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_MPU1_CS);  //输出1     
   //第二个SPI 片选
    nrfx_gpiote_out_uninit(configGPIO_SPI_MPU2_CS);
    err_code |= nrfx_gpiote_out_init(configGPIO_SPI_MPU2_CS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_MPU2_CS);  //输出1       
    NRF_LOG_INFO(("||Initialize||-->SPI_CollectData->error  0x%x"),err_code); 
    NRF_LOG_FLUSH(); 
    
    
    
// 
///*----------------------------------------------------------------------------*/ 
//    
//    char tcTest[] = "This is just a test!";
//    err_code |= ucSDCard_SaveData(tcTest,sizeof(tcTest)); 
//    NRF_LOG_INFO(("||Initialize||-->SDCard Save Test--->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH(); 


///**
// * 6. 初始化 GPS串口接收    
// */
//    err_code |= ucGPS_INTUART();    
//    NRF_LOG_INFO(("||Initialize||-->GPS Uart------->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();
///*----------------------------------------------------------------------------*/ 

///**
// * 7. 对上面初始化的结果进行LED灯显示
// *      红灯亮：初始化完成正确；
// *      红灯闪：初始化错误！ 
// */
//    if(err_code != 0)
//    {
//        NRF_LOG_INFO("Warning Warning Warning......Initialization is Wrong!!!!!"); 
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        }        
//    }else
//    {
//        //正确，红灯亮
//        nrfx_gpiote_out_clear(configGPIO_LED_R);
//    }
///*----------------------------------------------------------------------------*/ 
//    
///**
// * 8. 和任务相关的初始化 
// */    
//    /*(1) 互斥量的建立 */
//    xMutex_SDCDBuffer_1    = xSemaphoreCreateMutex();
//    if(xMutex_SDCDBuffer_1 == NULL)
//    {
//        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
//        NRF_LOG_INFO("Warning Warning Warning......MutexCreate is Wrong!!!!!"); 
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        }        
//    }
//    
//    /*(2) 单次时间任务的建立 启动中断等 */
//    xTimers_StartINT = xTimerCreate("StartINT",             /* 定时器名字 */
//                                  30000,                    /* 定时器周期,单位时钟节拍 */
//                                  pdFALSE,                  /* 周期性 */
//                                  (void *) 1,               /* 定时器ID */
//                                  TimerFunction_StartINT);  /* 定时器回调函数 */
//     if(xTimers_StartINT == NULL)
//     {
//         /* The timer was not created. */
//        NRF_LOG_INFO("||Warning||-->StartINT Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//     }                                 
//                                  
//                                  
//    /*(3) 循环时间任务的建立 SDCard存储任务 */
//    xTimers_StarSDCard = xTimerCreate("SDCard",            /* 定时器名字 */
//                              10,                /* 定时器周期,单位时钟节拍 */
//                              pdTRUE,          /* 周期性 */
//                              (void *) 2,      /* 定时器ID */
//                              TimerFunction_SDCardSave); /* 定时器回调函数 */
//     if(xTimers_StarSDCard == NULL)
//     {
//         /* The timer was not created. */
//        NRF_LOG_INFO("||Warning||-->SDCard Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//     }                               
//                                  
//    //延迟启动时间任务
//    if((xTimerStart(xTimers_StartINT,0) != pdPASS)||(xTimerStart(xTimers_StarSDCard,0) != pdPASS))
//     {
//         /* The timer could not be set into the Active
//         state. */
//        NRF_LOG_INFO("||Warning||-->SDCard_Timer Start is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        }
//     }                              
//      
//         
//    /*(4) MPU9255任务建立 */
//    txResult = xTaskCreate(vTask_MPU9255_RxData,
//                            "MPU9255",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_MPU9255_RxData,
//                            &xTaskHandle_MPU9255_RxData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskMPU9255 Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }
//    
//    /*(5) GPS 任务建立 */
//    txResult = xTaskCreate(vTask_GPS_RxData,
//                            "GPS",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_GPS_RxData,
//                            &xTaskHandle_GPS_RxData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskGPS Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }    
//    
//    /*(6) UWB任务建立 */
//    txResult = xTaskCreate(vTask_UWB_RxData,
//                            "UWB",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_UWB_RxData,
//                            &xTaskHandle_UWB_GetData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskUWB Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }   
//    
//    /*(7) FootPres任务建立 */
//    txResult = xTaskCreate(vTask_FootPres_RxData,
//                            "FootPres",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_FootPres_RxData,
//                            &xTaskHandle_FootPres_GetData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskFootPres Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }    
//    
//    /*(8) 启动 停止存储的任务建立 */  
//    txResult = xTaskCreate(vTask_DataSave_Start,
//                            "SaveStart",
//                            configMINIMAL_STACK_SIZE,
//                            NULL,
//                            taskPRIO_DataSave_Start,
//                            &xTaskHandle_DataSave_Start);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->SaveStart Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }  
//    txResult = xTaskCreate(vTask_DataSave_End,
//                            "SaveEnd",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_DataSave_End,
//                            &xTaskHandle_DataSave_End);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->SaveEnd Create is failed!!");
//        NRF_LOG_FLUSH();
//        //出错，死循环闪烁
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }  
///*----------------------------------------------------------------------------*/      
//    
/**
 * 9. 删除启动任务    
 */
    vTaskDelete(xTaskHandle_TaskINIT);
    NRF_LOG_INFO("||  Start  ||-->TaskINI is Delete!!");
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/    

}

   




/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    BaseType_t txResult = pdPASS;
    txResult = xTaskCreate(vTask_StartINIT,
                            "StartTask",
                            configMINIMAL_STACK_SIZE+200,
                            NULL,
                            taskPRIO_INIT,
                            &xTaskHandle_TaskINIT);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskINI Create is failed!!");
        NRF_LOG_FLUSH();
        return 1;
    }else
    {
        return 0;
    }
}





















///**
// * 开始数据存储   函数
//*/
//static void vTask_DataSave_Start(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/        
//        G_Ctrl_DataSave = 1;       
//        
//    }
//}

///**
// * 结束数据存储   函数
//*/
//static void vTask_DataSave_End(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/        
//        G_Ctrl_DataSave = 0;       
//        vTaskDelay( (TickType_t)100 );
//        ucSDCard_CloseFile();        
//    }
//}


///**
// * MPU9255数据采集任务   函数
//*/
//static void vTask_MPU9255_RxData(void *pvParameters)
//{
//    
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/        
//         /**
//         *(2) 采集MPU9255数据 */
//        memcpy(G_MPU9255_Data+2,&G_GPSWeekSecond,4);
//        memcpy(G_MPU9255_Data+6,&G_MicroSecond,2);        
//        memcpy(G_MPU9255_Data+8,&G_MPU9255_Counter,1); 
//        
//        /* 采集传感器数据 */    
//        Leo_MPU9255_Read_ACC();
//        Leo_MPU9255_Read_Gyro();
//        Leo_MPU9255_Read_Magnetic(); 
//        
//        /*若是可以存储 */
//        if(G_Ctrl_DataSave == 1)
//        {
//            /* 获取资源存放数据   等待2ms 一个周期5ms*/
//            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//            {   
//                 /*  有漏洞，有可能缓存溢出  如果 存储不及时的话！！ */
//                if(G_SDCDBuffer1_NUM + sizeof(G_MPU9255_Data) > 1024)
//                {
//                    NRF_LOG_INFO("MPU9255 Can't Save, It's Full !!!!!!!!!!!!!!!!!");
//                    NRF_LOG_FLUSH();  
//                }else
//                {
//                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_MPU9255_Data,sizeof(G_MPU9255_Data)); 
//                    G_SDCDBuffer1_NUM += sizeof(G_MPU9255_Data);
//                }               
//                 
//                /* 释放资源 */
//                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
//            }else
//            {
//                /* 未获取到资源的使用权限 */
//                NRF_LOG_INFO("MPU9255 Use Buffer is Busy!!!!!!!!!!!!!!!!!");
//                NRF_LOG_FLUSH();            
//            }            
//        }
//        

//        //test        
////        if((G_MPU9255_Counter % 255) == 0)
////        {
////            NRF_LOG_INFO("MPU9255 Data is OK!___5s");
////        }        
//    }
//}


///**
// * GPS数据采集任务   函数
//*/
//static void vTask_GPS_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知
//         */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/   
//    }    
//    
//}

///**
// * UWB数据采集任务   函数
//*/    
//static void vTask_UWB_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知
//         */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/   
//    } 
//}



///**
// * 压力传感器数据采集任务   函数
//*/
//static void vTask_FootPres_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) 等待任务通知     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/        
//         /**
//         *(2) 采集MPU9255数据 */
//        
//        /*若是可以存储 */
//        if(G_Ctrl_DataSave == 1)
//        {
//            /* 获取资源存放数据   等待2ms 一个周期5ms*/
//            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//            {   
//                 /*  有漏洞，有可能缓存溢出  如果 存储不及时的话！！ */
//                if(G_SDCDBuffer1_NUM + sizeof(G_FOOTPressure_Data) > 1024)
//                {
//                    NRF_LOG_INFO("G_FOOTPressure_Data Can't Save, It's Full !!!!!!!!!!!!!!!!!");
//                    NRF_LOG_FLUSH();  
//                }else
//                {
//                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_FOOTPressure_Data,sizeof(G_FOOTPressure_Data)); 
//                    G_SDCDBuffer1_NUM += sizeof(G_FOOTPressure_Data);
//                }               
//                 
//                /* 释放资源 */
//                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
//            }else
//            {
//                /* 未获取到资源的使用权限 */
//                NRF_LOG_INFO("G_FOOTPressure_Data Use Buffer is Busy!!!!!!!!!!!!!!!!!");
//                NRF_LOG_FLUSH();            
//            }            
//        }
//            
//    }
//}


///**
// * 中断启动时间任务   函数
//*/
//static void TimerFunction_StartINT( TimerHandle_t xTimer )
//{
//    uint8_t err_code = 0;
///*1. 启动计时器*/
//#if configTIMER3_ENABLE    
//    /* TIMER3 计数器启动*/ 
//    err_code |= ucTimerStart_3();
//#endif 
//    NRF_LOG_INFO(("||  Start  ||-->TIMER--------->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();    


///*2. 启动外部中断*/
//    err_code |= ucINTStart_SDCard();    /* SDCard中断 启动 */    
//    err_code |= ucINTStart_MPU9255();   /* MPU9255采集中断 启动 */
//    err_code |= ucINTStart_PPS();       /* 1PPS秒脉冲中断 启动 */
//    NRF_LOG_INFO(("||  Start  ||-->GPS Uart------->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();
//    
///*3. 数据存储启动*/    
//    G_Ctrl_DataSave = 1;
//    nrfx_gpiote_out_clear(configGPIO_LED_G);
//}

///**
// * SDCard循环存储任务   函数
//*/
//static void TimerFunction_SDCardSave( TimerHandle_t xTimer )
//{
//    uint8_t     erro_code = 0;
//    if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//    {
//        if(G_SDCDBuffer1_NUM > 0)
//        {
//            memcpy(G_SDCDBuffer2,G_SDCDBuffer1,G_SDCDBuffer1_NUM);
//            G_SDCDBuffer2_NUM = G_SDCDBuffer1_NUM;
//            G_SDCDBuffer1_NUM = 0;             
//        }
//        /* 释放资源 */
//        xSemaphoreGive( xMutex_SDCDBuffer_1 );          
//                    
//        if(G_Ctrl_DataSave == 1)
//        {
//            erro_code = ucSDCard_SaveData(G_SDCDBuffer2,G_SDCDBuffer2_NUM);  
//            if(erro_code != 0)
//            {
//                NRF_LOG_INFO("SDCard DataSave is Wrong Wrong !!!!!!!!!!!!!");
//                NRF_LOG_FLUSH(); 
//            }
//        }  
//    }else
//    {
//        /* 未获取到资源的使用权限 */
//        NRF_LOG_INFO("TimerFunction_FOOT Use Buffer1 is Busy!!!!!!!!!!!!!!!!!");
//        NRF_LOG_FLUSH();            
//    } 
//}














/*******************************************************************************************************/





/*
*********************************************************************************************************
*                                    自己任务的 相关设置
*********************************************************************************************************
*/



/* 多任务测试 */

//TaskHandle_t    xTaskHandle_Test;

//void vTaskFunction_TaskTest(void * pvParameter)
//{
//    UNUSED_PARAMETER(pvParameter);
//    while (true)
//    {   
//       
//        
//        NRF_LOG_INFO("=================================================");
//        NRF_LOG_FLUSH();
//        /* Delay a task for a given number of ticks */
//        vTaskDelay(1000);

//        /* Tasks must be implemented to never return... */
//    }
//}


//void vTask_Create(void)
//{


//    if (pdPASS != xTaskCreate(vTaskFunction_TaskTest, "TTest", 256, NULL, 2, &xTaskHandle_Test))
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }  
//    NRF_LOG_INFO("||OS_Task   ||---->>xTaskCreate---->>completed!");

//}
















