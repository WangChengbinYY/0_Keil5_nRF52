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



/*========================== 全局变量定义！================================*/
/* 全局变量_时间参数 */
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0

/* 全局变量_SDCard文件操作标识 */                                                        
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0

/* 全局变量_IMU数据采集的 SPI2 实例（UWB 用SPI0；SDCard 用SPI1； IMU 和 压力传感器 用SPI2） */
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	

uint8_t	    G_CollectData[512];                 //SDCard要储存数据的缓存
uint16_t	    G_CollectData_Counter;                 

uint8_t	    G_IMU_Data1[28];                    //第一组MPU9255存放的数据
uint8_t	    G_IMU_Data2[28];                    //第二组MPU9255存放的数据
uint8_t	    G_IMUData_Counter;                  //MPU9255中断触发的计数器	    








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










/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */
#define taskPRIO_INIT                        0          /* 系统初始化：仅在开始执行一次，初始化失败则进入死循环LED不听闪烁*/
#define taskPRIO_LED_SLOW                    0          /* LED慢闪(1s一闪)：(代表GPS初始定位成功，并获取时间)   */
#define taskPRIO_LED_QUICK                   0          /* LED快闪(500ms)：(代表SDCard的存储或停止操作成功)   */

/* 1级 */
#define taskPRIO_GPS_RxData                  1          //接收GPS数据并解析，解析成功，通知存储 
#define taskPRIO_SDCard_Open                 1          //SDCard建立文件成功  标志位置1  数据会被存储
#define taskPRIO_SDCard_Close                1          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 2级 */
#define taskPRIO_SDCard_Save                 2          //SDCard存储数据

/* 3级 */
#define taskPRIO_CollectData                 3          //采集IMU数据、压力数据，成功则通知存储

/* 4级 */
#define taskPRIO_UWB_RxData                  4          //采集UWB测距数据，成功则通知存储


/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_TaskINIT            = NULL;         /*系统初始化任务       句柄 */
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*5ms触发的采集任务    句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */
TaskHandle_t    xTaskHandle_SDCard_Open         = NULL;         /*SDCard 新建文件任务  句柄 */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */


/**
 * 全局变量_互斥量_SDCard缓存  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;





/*======================================= 全局变量定义 待定！！！！！！==================================================*/
/* 全局变量_时间参数 */
//uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
//uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0

///* 全局变量_SDCard文件操作标识 */                                                        
//uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0

///* 全局变量_IMU数据采集的 SPI2 实例（UWB 用SPI0；SDCard 用SPI1； IMU 和 压力传感器 用SPI2） */
//nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);  			//MPU9255使用的SPI实例		
//uint8_t         G_MPU9255_SPI_xfer_Done = 1;

//uint8_t     G_WRONG_Record[10];                 //采集过程数据出错的记录

//uint8_t	    G_IMU_Data[28];                 //MPU9255传感器存放的数据
//uint8_t	    G_IMUData_Counter;                  //MPU9255中断触发的计数器

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
 * 全局变量初始化函数   待完善  
*/
static void vINIT_Variable(void)
{
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
                                                        //由 外部GPS的1PPS校准 1PPS触发时 将其置0
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0
    
    memset(G_CollectData,0,512);
    G_CollectData_Counter = 0;
    
    //IMU数据
    memset(G_IMU_Data1,0,28);
    G_IMU_Data1[0] = 0xA1;
	G_IMU_Data1[1] = 0xA2;
    G_IMU_Data1[27] = 0xFF;
    memset(G_IMU_Data2,0,28);
    G_IMU_Data2[0] = 0xA1;
	G_IMU_Data2[1] = 0xA3;
    G_IMU_Data2[27] = 0xFF;
    G_IMUData_Counter    = 0;                     
    


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
 
}


/*------------------------------------------------------------
 *外设相关的初始化，主要包括：
 *  1、全局变量初始化；
 *  2、管教配置：LED 中断；定时器等
 *  3、各类传感器(IMU GPS SDCard)等的初始化
 *------------------------------------------------------------*/
static uint8_t ucINIT_Peripheral()
{
    uint8_t err_code = 0;
/**
 * 0. 全局变量初始化    
 */    
    vINIT_Variable();    
    
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

/**
 * 3. 初始化SDCard 并建立记录文件    
 */
    err_code |= ucSDCard_INIT();   
    err_code |= ucSDCard_OpenFile();      
    //LeoDebug
//    nrf_delay_ms(1000);
//    err_code |= ucSDCard_SaveData(&err_code,sizeof(err_code));
//    err_code |= ucSDCard_CloseFile();
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();   
    
/**
 * 4. 初始化 两个MPU 
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
//    SPI_config.ss_pin			= configGPIO_SPI_MPU2_CS;               //第二个IMU的nCS管脚
//	//依据配置参数 对 实例spi 进行初始化 
//    err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
//    NRF_LOG_INFO(("||Initialize||-->SPI_IMU_2------>error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();      
//    //针对IMU_1初始化
//    err_code |= ucMPU9255_INIT();
//    NRF_LOG_INFO(("||Initialize||-->MPU9255_B-------->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();   
//    //卸载SPI
//    nrf_drv_spi_uninit(&SPI_CollectData);	       

/**
 * 5. 配置正确的SPI，等待片选管脚设置 
 */
    SPI_config.ss_pin			= NRF_DRV_SPI_PIN_NOT_USED;         //不使用nCS管脚
    SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;
    err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
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
    
    
    return err_code;
}




/*------------------------------------------------------------
 *SDCard新建文件任务 函数
 *------------------------------------------------------------*/
static void vTask_SDCard_Open(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        /*(1) 等待任务通知     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/
        
        erro_code = ucSDCard_OpenFile();
        if(erro_code == 0)
        {
            for(i=0;i<15;i++)
            {
                nrfx_gpiote_out_toggle(configGPIO_LED_R);
                nrf_delay_ms(200);
            } 
            G_SDCard_FileIsOpen = 1;
            nrfx_gpiote_out_clear(configGPIO_LED_R);
        }        
    }
}

/*------------------------------------------------------------
 *SDCard关闭文件任务 函数
 *------------------------------------------------------------*/
static void vTask_SDCard_Close(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        /*(1) 等待任务通知     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/
        
        G_SDCard_FileIsOpen = 0;
        erro_code = ucSDCard_CloseFile();
        if(erro_code == 0)
        {
            for(i=0;i<15;i++)
            {
                nrfx_gpiote_out_toggle(configGPIO_LED_R);
                nrf_delay_ms(200);
            } 
            nrfx_gpiote_out_clear(configGPIO_LED_R);
            
            NRF_LOG_INFO("File is Closed!!!");
        }        
    }
}

/*------------------------------------------------------------
 *SDCard存储任务 函数
 *------------------------------------------------------------*/
static void vTask_SDCard_Save(void *pvParameters)
{
    uint8_t tData[512] = {0};
    uint16_t tData_Count = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        /*(1) 等待任务通知     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/
        
        /*(2) 获取资源     */
//        if(G_CollectData_Counter >= 256)
//        {

            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //释放资源
                xSemaphoreGive( xMutex_SDCDBuffer ); 
//                NRF_LOG_INFO("%d",tData_Count);
                //存储 文件如果打开才往里面存储
                if(G_SDCard_FileIsOpen == 1)
                {                    
                    erro_code = ucSDCard_SaveData(tData,tData_Count);                    
                }
                //LEODEBUG
                if(erro_code != 0)
                {
                    NRF_LOG_INFO("SDCard Save is Wrong!!!!!!!!!! %d",erro_code);
                    NRF_LOG_FLUSH();                     
                }                    
            }else
            {
                //LOEDEBUG
                NRF_LOG_INFO("SDCard_Buffer is Busy for SDCard!!!!!!!!!!");
                NRF_LOG_FLUSH(); 
            }
            
//        }else{
//            continue;
//        }
    }        
}




/*------------------------------------------------------------
 *外部传感器数据采集，主要包括：
 *  MPU9255A、MPU9255B、压力传感器
 *------------------------------------------------------------*/
static void vTask_CollectData(void *pvParameters)
{
    while(1)
    {
        /*(1) 等待任务通知     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/      
        
        
        /*(2) 采集MPU9255A 的数据 */ 
        nrfx_gpiote_out_clear(configGPIO_SPI_MPU1_CS);    
        Leo_MPU9255_Read_ACC(G_IMU_Data1);
        Leo_MPU9255_Read_Gyro(G_IMU_Data1);
        Leo_MPU9255_Read_Magnetic(G_IMU_Data1);  
        nrfx_gpiote_out_set(configGPIO_SPI_MPU1_CS);          
        /*(3) 采集MPU9255B 的数据 */        
//        nrfx_gpiote_out_clear(configGPIO_SPI_MPU2_CS);    
//        Leo_MPU9255_Read_ACC(G_IMU_Data2);
//        Leo_MPU9255_Read_Gyro(G_IMU_Data2);
//        Leo_MPU9255_Read_Magnetic(G_IMU_Data2);  
//        nrfx_gpiote_out_set(configGPIO_SPI_MPU2_CS);          
        /*(4) 采集压力传感器 的数据  未完成*/      
        
        
        /*(5) 都采集完了,整体存储 
         *    等待3ms，如果还没有释放，则放弃此次存储*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 4 ) == pdTRUE)
        {
            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data1,sizeof(G_IMU_Data1));
            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data1);
//            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data2,sizeof(G_IMU_Data2));
//            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data2);        
            //释放资源
            xSemaphoreGive( xMutex_SDCDBuffer ); 
            //通知 SDCard存储任务
            xTaskNotify(xTaskHandle_SDCard_Save,     
                        0,              
                        eNoAction);                
        }else
        {
            //LOEDEBUG
            NRF_LOG_INFO("SDCard_Buffer is Busy for IMU!!!!!!!!!!!!");
            NRF_LOG_FLUSH(); 
        }

    }
}





/*------------------------------------------------------------
 *任务相关的初始化，主要包括：
 *  1、互斥量的初始化
 *  2、各类任务的初始化及启动
 *------------------------------------------------------------*/
static uint8_t ucINIT_Task()
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    
    /*(1) 互斥量的建立_SDCard缓存 */
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    /*(2) 建立采集任务 */
    txResult = xTaskCreate(vTask_CollectData,
                            "CollectData",
                            configMINIMAL_STACK_SIZE+200,
                            NULL,
                            taskPRIO_CollectData,
                            &xTaskHandle_CollectData);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }
    
    /*(3) 建立SDCard存储任务 */    
    txResult = xTaskCreate(vTask_SDCard_Save,
                            "SDCardSave",
                            configMINIMAL_STACK_SIZE+400,
                            NULL,
                            taskPRIO_SDCard_Save,
                            &xTaskHandle_SDCard_Save);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }    
    
    /*(4) 建立SDCard 新建文件任务 */      
    txResult = xTaskCreate(vTask_SDCard_Open,
                            "SDCardOpen",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            taskPRIO_SDCard_Open,
                            &xTaskHandle_SDCard_Open);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }     
    
    /*(4) 建立SDCard 关闭文件任务 */      
    txResult = xTaskCreate(vTask_SDCard_Close,
                            "SDCardClose",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            taskPRIO_SDCard_Close,
                            &xTaskHandle_SDCard_Close);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }       
    
    
    return erro_code;  
}
    



/*------------------------------------------------------------
 *初始化任务  只执行一次                                                                              
 *------------------------------------------------------------*/
static void vTask_StartINIT(void *pvParameters)
{
    uint8_t err_code = 0;
    BaseType_t txResult = pdPASS;
    
/**
 * 0. 外设相关初始化
 *      初始化成功，红灯亮；
 *      初始化错误，红灯灭，删除初始化任务并退出    
 */     
    err_code |= ucINIT_Peripheral();
    if(err_code != 0)
    {
        vTaskDelete(xTaskHandle_TaskINIT);
        NRF_LOG_INFO("Peripheral Initialization is Wrong!!");
        NRF_LOG_FLUSH();
        return ;
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R);
    }    

/**
 * 1. 任务相关初始化（建立好后，自动启动）
 *      任务初始化成功，红灯亮；
 *      任务初始化错误，红灯灭，删除初始化任务并退出       
 */
    err_code |= ucINIT_Task();
    if(err_code != 0)
    {
        vTaskDelete(xTaskHandle_TaskINIT);
        NRF_LOG_INFO("TASK Initialization is Wrong!!");
        NRF_LOG_FLUSH();
        return ;
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R);
    }       
/**
 * 2. 中断及定时器 启动  未完成
 */
    //(1)计时器启动
    err_code |= ucTimerStart_3();      /* TIMER3 计数器初始化*/ 
    err_code |= ucTimerStart_4();
    
    //(2)中断启动
    ucINTStart_SDCard();
/**
 * 3. 删除启动任务    
 */
    vTaskDelete(xTaskHandle_TaskINIT);
    NRF_LOG_INFO("||  Start  ||-->TaskINI is Delete!!");
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/      
    
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
//        memcpy(G_IMU_Data+2,&G_GPSWeekSecond,4);
//        memcpy(G_IMU_Data+6,&G_MicroSecond,2);        
//        memcpy(G_IMU_Data+8,&G_IMUData_Counter,1); 
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
//                if(G_SDCDBuffer1_NUM + sizeof(G_IMU_Data) > 1024)
//                {
//                    NRF_LOG_INFO("MPU9255 Can't Save, It's Full !!!!!!!!!!!!!!!!!");
//                    NRF_LOG_FLUSH();  
//                }else
//                {
//                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_IMU_Data,sizeof(G_IMU_Data)); 
//                    G_SDCDBuffer1_NUM += sizeof(G_IMU_Data);
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
////        if((G_IMUData_Counter % 255) == 0)
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
















