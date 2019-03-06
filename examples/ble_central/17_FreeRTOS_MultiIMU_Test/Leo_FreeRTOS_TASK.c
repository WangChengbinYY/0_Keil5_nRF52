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
#define taskPRIO_SDCard_Close                0          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 1级 */
#define taskPRIO_GPS_RxData                  1          //接收GPS数据并解析，解析成功，通知存储 

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
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*5ms触发的采集任务    句柄 */



/**
 * 全局变量_互斥量_SDCard缓存  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;

/*=========================================== 任务实现 ==============================================*/

/*------------------------------------------------------------
 *SDCard关闭文件任务 函数
 *------------------------------------------------------------*/
static void vTask_SDCard_Close(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        //(1) 等待任务通知     
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                        portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/     
        
        //(3)关闭文件存储
        erro_code = ucSDCard_CloseFile();
        
        //(4)LED灯快闪
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
        if(G_CollectData_Counter > 0)
        {
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
            
        }else{
            continue;
        }
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
        
        //(2)记录整秒数据
        memcpy(G_GPSWeekSecond_Data+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
        memcpy(G_CollectData+G_CollectData_Counter,G_GPSWeekSecond_Data,sizeof(G_GPSWeekSecond_Data));
        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSWeekSecond_Data);        
        
        //(3)采集IMU_A 的数据  
        memcpy(G_IMU_Data_A+2,&G_MicroSecond,sizeof(G_MicroSecond));
        nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS);    
        Leo_MPU9255_Read_ACC(G_IMU_Data_A+5);
        Leo_MPU9255_Read_Gyro(G_IMU_Data_A+11);
        Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+17);  
        nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);   
        
 
        
        /*(4) 采集压力传感器 的数据  未完成*/      
        
        
        /*(5) 都采集完了,整体存储 
         *    等待4ms，如果还没有释放，则放弃此次存储*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 4 ) == pdTRUE)
        {
            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data_A,sizeof(G_IMU_Data_A));
            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data_A);
       
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

/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    
    /*(1) 互斥量的建立_SDCard缓存 */
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    /*(2) 建立SDCard存储任务 */    
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
    
    /*(3) 建立SDCard 关闭文件任务 */      
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

    
    //(4) 建立采集任务 5ms   待改写成 时间任务
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
    
    return erro_code;    
}




