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


//全局变量_时间参数 
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0
uint8_t	    G_GPSWeekSecond_Data[7];          //用于数据采集时，记录的当时时刻的时间

//全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
uint8_t	    G_MAG_Coeffi[6]; 

//全局变量_IMU_A(U4)和IMU_B(U5)存放的缓存                
uint8_t	    G_IMU_Data_A[24];                   //第一组IMU_A(U4)存放的数据
uint8_t	    G_IMU_Data_B[24];                   //第二组IMU_A(U5)存放的数据
uint8_t	    G_IMUDataA_Counter;                  //MPU9255中断触发的计数器	    
uint8_t	    G_IMUDataB_Counter;


// 全局变量_SDCard存储缓存                                                         
uint8_t	    G_CollectData[512];                 //SDCard要储存数据的缓存
uint16_t    G_CollectData_Counter;  
// 全局变量_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0




/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */


/* 1级 */
#define taskPRIO_GPS_RxData                  1          //接收GPS数据并解析，解析成功，通知存储 

/* 2级 */
//5ms 循环的时间 任务

/* 3级 */
#define taskPRIO_SDCard_Save                 3          //SDCard存储数据

/* 4级 */
#define taskPRIO_SDCard_Close                4          //SDCard关闭文件成功  标志位置0  数据不会存储

/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
//TaskHandle_t    xTaskHandle_CollectData         = NULL;         
TimerHandle_t     xTimerHandle_CollectData        = NULL;         /*5ms触发的采集任务    句柄 */


/**
 * 全局变量_互斥量_SDCard缓存  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;




/**
 * 全局变量初始化函数   待完善  
*/
void vINIT_Variable(void)
{
    //全局变量_时间参数 
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
    //数据采集的整秒记录
    memset(G_GPSWeekSecond_Data,0,7);
    G_GPSWeekSecond_Data[0] = 0xA0;
	G_GPSWeekSecond_Data[1] = 0xA0;
    G_GPSWeekSecond_Data[6] = 0xFF;   
    
    //全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
    memset(G_MAG_Coeffi,0,6);
    G_MAG_Coeffi[5] = 0xFF;     
    
    //全局变量_IMU_A(U4)数据
    memset(G_IMU_Data_A,0,24);
    G_IMU_Data_A[0] = 0xB1;
	G_IMU_Data_A[1] = 0xB1;
    G_IMU_Data_A[23] = 0xFF;
    memset(G_IMU_Data_B,0,24);
    G_IMU_Data_B[0] = 0xB2;
	G_IMU_Data_B[1] = 0xB2;
    G_IMU_Data_B[23] = 0xFF;
    G_IMUDataA_Counter = 0;                  //IMU采集的次数计数值	    
    G_IMUDataB_Counter = 0;      
    
    // 全局变量_SDCard存储缓存        
    memset(G_CollectData,0,512);
    G_CollectData_Counter = 0;    
    //全局变量_SDCard文件操作标识 
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0 
}





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
            nrfx_gpiote_out_set(configGPIO_LED_R);
            
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
        if((G_CollectData_Counter > 0) && (G_SDCard_FileIsOpen == 1) )
        {
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //释放资源
                xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                erro_code = ucSDCard_SaveData(tData,tData_Count);                    

                //LEODEBUG
                if(erro_code != 0)
                {
                    NRF_LOG_INFO("SDCard Save is Wrong!!!!!!!!!! %d",erro_code);
                    NRF_LOG_FLUSH();                     
                }                    
            }else
            {
                //LOEDEBUG
                NRF_LOG_INFO("SDCard_Buffer is Busy for Save!!!!!!!!!!");
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
static void vTimer_CollectData(xTimerHandle pxTimer)
{
    //(1)记录整秒数据
    memcpy(G_GPSWeekSecond_Data+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));       
    
    //(2)采集IMU_A 的数据
    //记录 ms 数据
    memcpy(G_IMU_Data_A+2,&G_MicroSecond,sizeof(G_MicroSecond));
    //选择IMU_A nCS管脚
    nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
    nrf_delay_us(1); 
    //采集IMU_A 的数据
    Leo_MPU9255_Read_ACC(G_IMU_Data_A+5);
    Leo_MPU9255_Read_Gyro(G_IMU_Data_A+11);
    Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+17);
    //关闭IMU_A nCS管脚
    nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);   
    

    
    /*(4) 采集压力传感器 的数据  未完成*/      
    
    
    /*(5) 都采集完了,整体存储 等待2ms，如果还没有释放，则放弃此次存储*/
    if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
    {
        //整秒数据 存入缓存
        memcpy(G_CollectData+G_CollectData_Counter,G_GPSWeekSecond_Data,sizeof(G_GPSWeekSecond_Data));
        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSWeekSecond_Data); 
        //IMU_A数据 存入缓存
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

/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    TickType_t xTimer = 5;
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

    
    //(4) 建立采集任务 5ms定时器 
    xTimerHandle_CollectData = xTimerCreate("5ms",
                                            xTimer,
                                            pdTRUE,
                                            (void *)1,
                                            vTimer_CollectData);
    if(xTimerHandle_CollectData == NULL)
    {
        erro_code = 1;
    }else{
        if(xTimerStart(xTimerHandle_CollectData,1000) != pdPASS)
        {
            erro_code = 1;
            NRF_LOG_INFO("vTimer_CollectData Start is Wrong！");
            NRF_LOG_FLUSH(); 
        }else{
            NRF_LOG_INFO("vTimer_CollectData Start is OK！");
            NRF_LOG_FLUSH(); 
        }
        
    }    
    return erro_code;    
}




