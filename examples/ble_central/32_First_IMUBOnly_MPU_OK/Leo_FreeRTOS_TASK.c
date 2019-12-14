/*
*********************************************************************************************************
*
*    模块名称 : FreeRTOS多任务实现
*    文件名称 :Leo_FreeRTOS_TASK
*    版    本 : V1.0
*    说    明 : 项目中所有任务的建立
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-19     WangCb   
*				 V1.5    2019-03-19     WangCb
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/

#include "Leo_FreeRTOS_TASK.h"


//全局变量_参数相关_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;           //标记是否已经打开文件 没打开，默认为0
uint8_t     G_UWB_Time;                    //UWB的采样频率 是 IMU的几倍 2

//全局变量_参数相关_时间 
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint8_t     G_GPSWeekSecond_IsValid;

uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0
uint16_t    G_MicroSecond_Saved;

//全局变量_参数存储_整秒时间(有GPS天内秒，无从0开始)
uint8_t	    G_A0A0_Time_Seconds[7]; 
uint8_t	    G_A0A0_Time_Seconds_IsReady;            //任何写入缓存区数据前，判断一下，时间是否已更新，如更新，先存时间数据

//全局变量_参数存储_磁强计修正参数  IMUA(MPU9255) or IMUB(MPU9255)
uint8_t	    G_A1A1_A2A2_MAG_Coeffi[5]; 

//IMUA-(MPU9250)(包含磁强计，后面跟着压力传感器数据)
uint8_t	    G_B1B1_IMUA_MPU_Pressure[4];

//IMUA-(MPU9250)(包含磁强计，不带压力传感器)
uint8_t	    G_B2B2_IMUA_MPU[4];

//IMUA-(MPU9250)，仅采集MPU的磁强计数据
uint8_t	    G_B3B3_Magnetic_IMUAMPU[4];

//IMUB-(MPU9250)的数据(包含磁强计，后面跟着压力传感器数据)
uint8_t	    G_B4B4_IMUB_MPU_Pressure[4];

//IMUB-(MPU9250)(包含磁强计，不带压力传感器)
uint8_t	    G_B5B5_IMUB_MPU[4];

//IMUB-(ADIS)(后面跟着压力传感器数据)
uint8_t	    G_B6B6_IMUB_ADIS_Pressure[4];

//IMUB-(ADIS)(不包含压力传感器数据)
uint8_t	    G_B7B7_IMUB_ADIS[4];

//IMUB-(MTi)(后面跟着压力传感器数据)
uint8_t	    G_B8B8_IMUB_MTi_Pressure[4];

//IMUB-(MTi)(不包含压力传感器数据)
uint8_t	    G_B9B9_IMUB_MTi[4];

//全局变量_数据存储_UWB测距数据
uint8_t     G_C1C1_UWB[6];

//全局变量_数据存储_GPS定位数据
uint8_t     G_C2C2_GPS[14];

//全局变量_数据存储_GPS串口接收缓存
uint8_t     G_Uart_Buffer1[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer2[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer_Number;

// 全局变量_数据存储_SDCard存储缓存  
uint8_t     G_SDCard_CirBuffer[configSDCard_BufferSize];
uint8_t*    G_SDCard_CB_pSave = NULL;
uint8_t*    G_SDCard_CB_pLoad = NULL;
uint16_t    G_SDCard_CB_Counter; 

/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */
#define taskPRIO_SDCard_Close                0          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 1级 */
#define taskPRIO_SDCard_Save                 1          //SDCard存储数据 

/* 2级 */
#define taskPRIO_GPS_RxData                  2          //接收GPS数据并解析	

/* 3级 */
//#define taskPRIO_UWB_Start                   3          //UWB的启动

/* 4级 */
#define taskPRIO_CollectData_IMUA            4          //采集 IMUA(U4)的数据，存储主循环 (包括压力传感器数据)

/* 5级 */
#define taskPRIO_CollectData_IMUB            5          //采集 IMUB(U5)的数据 以ADIS或MTI的采集为主 (包括压力传感器数据)		         

/* 6级 */
#define taskPRIO_UWB_EventHandler            6			//UWB响应端 的任务等级 

/* 7级 */
#define taskPRIO_TaskStart                   7          //启动任务 

/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*解析GPS串口数据       句柄 */
//TaskHandle_t    xTaskHandle_UWB_Start           = NULL;
TaskHandle_t    xTaskHandle_CollectData_IMUB    = NULL;         //IMUB(U5) 副IMU数据采集任务
TaskHandle_t    xTaskHandle_CollectData_IMUA    = NULL;         //IMUA(U4) 主采集循环任务
TaskHandle_t    xTaskHandle_UWB_EventHandler    = NULL; 
TaskHandle_t    xTaskHandle_TaskStart           = NULL;

///* 全局变量_互斥量_SDCard缓存——同时也是SPI的使用互斥量  */
SemaphoreHandle_t   xMutex_SDCard_CirBuffer     = NULL;
//二值信号量，用于 GPS数据解析的缓存使用
SemaphoreHandle_t   xSemaphore_GPSBuffer        = NULL;

/**
 * 全局变量初始化函数   待完善  
*/
void vINIT_Variable(void)
{
//全局变量_参数相关_SDCard文件操作标识                                                         
    G_SDCard_FileIsOpen = 0;           //标记是否已经打开文件 没打开，默认为0
    G_UWB_Time = 0;

//全局变量_参数相关_时间 
    G_GPSWeekSecond = 0;                   //GPS周内秒数据
    G_GPSWeekSecond_IsValid = 0;
    G_MicroSecond = 0;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0
    G_MicroSecond_Saved = 0;     
//全局变量_参数存储_整秒时间(有GPS天内秒，无从0开始)
    memset(G_A0A0_Time_Seconds,0,7);
    G_A0A0_Time_Seconds[0] = 0xA0;
    G_A0A0_Time_Seconds[1] = 0xA0;
    G_A0A0_Time_Seconds_IsReady = 0;            //任何写入缓存区数据前，判断一下，时间是否已更新，如更新，先存时间数据

//全局变量_参数存储_磁强计修正参数_(MPU92) IMUA or IMUB
    memset(G_A1A1_A2A2_MAG_Coeffi,0,5);

    memset(G_B1B1_IMUA_MPU_Pressure,0,4);
    G_B1B1_IMUA_MPU_Pressure[0] = 0xB1;
    G_B1B1_IMUA_MPU_Pressure[1] = 0xB1;
    
    memset(G_B2B2_IMUA_MPU,0,4);
    G_B2B2_IMUA_MPU[0] = 0xB2;
    G_B2B2_IMUA_MPU[1] = 0xB2;

    memset(G_B3B3_Magnetic_IMUAMPU,0,4);
    G_B3B3_Magnetic_IMUAMPU[0] = 0xB3;
    G_B3B3_Magnetic_IMUAMPU[1] = 0xB3;

    memset(G_B4B4_IMUB_MPU_Pressure,0,4);
    G_B4B4_IMUB_MPU_Pressure[0] = 0xB4;
    G_B4B4_IMUB_MPU_Pressure[1] = 0xB4;

    memset(G_B5B5_IMUB_MPU,0,4);
    G_B5B5_IMUB_MPU[0] = 0xB5;
    G_B5B5_IMUB_MPU[1] = 0xB5;
    
    memset(G_B6B6_IMUB_ADIS_Pressure,0,4);
    G_B6B6_IMUB_ADIS_Pressure[0] = 0xB6;
    G_B6B6_IMUB_ADIS_Pressure[1] = 0xB6;
    
    memset(G_B7B7_IMUB_ADIS,0,4);
    G_B7B7_IMUB_ADIS[0] = 0xB7;
    G_B7B7_IMUB_ADIS[1] = 0xB7;
    
    memset(G_B8B8_IMUB_MTi_Pressure,0,4);
    G_B8B8_IMUB_MTi_Pressure[0] = 0xB8;
    G_B8B8_IMUB_MTi_Pressure[1] = 0xB8;
    
    memset(G_B9B9_IMUB_MTi,0,4);
    G_B9B9_IMUB_MTi[0] = 0xB9;
    G_B9B9_IMUB_MTi[1] = 0xB9;
    
    memset(G_C1C1_UWB,0,6);
    G_C1C1_UWB[0] = 0xC1;
    G_C1C1_UWB[1] = 0xC1;
    
    memset(G_C2C2_GPS,0,6);
    G_C2C2_GPS[0] = 0xC2;
    G_C2C2_GPS[1] = 0xC2;

//全局变量_数据存储_GPS串口接收缓存
    memset(G_Uart_Buffer1,0,configBufferUART_RX_SIZE);    
    memset(G_Uart_Buffer2,0,configBufferUART_RX_SIZE); 
    G_Uart_Buffer_Number = 0;

// 全局变量_数据存储_SDCard存储缓存  
    memset(G_SDCard_CirBuffer,0,sizeof(G_SDCard_CirBuffer));
    G_SDCard_CB_pSave = G_SDCard_CirBuffer;
    G_SDCard_CB_pLoad = G_SDCard_CirBuffer;
    G_SDCard_CB_Counter = 0;       
}


//环形缓存区 存储数据    
void ucCircleBuffer_SaveData(uint8_t const* pBuffer,uint16_t mSize)
{
    if((G_SDCard_CB_pSave + mSize) <= (G_SDCard_CirBuffer + configSDCard_BufferSize))
    {
        memcpy(G_SDCard_CB_pSave,pBuffer,mSize);
        G_SDCard_CB_pSave += mSize;
        G_SDCard_CB_Counter += mSize;
        if(G_SDCard_CB_pSave == (G_SDCard_CirBuffer + configSDCard_BufferSize))
        {
            G_SDCard_CB_pSave = G_SDCard_CirBuffer;
        }
    }else
    {
        uint16_t tFirstSize,tSecondSize;
        tFirstSize = G_SDCard_CirBuffer + configSDCard_BufferSize - G_SDCard_CB_pSave;
        tSecondSize = mSize-tFirstSize;
        memcpy(G_SDCard_CB_pSave,pBuffer,tFirstSize);
        G_SDCard_CB_pSave = G_SDCard_CirBuffer;
        memcpy(G_SDCard_CB_pSave,(pBuffer+tFirstSize),tSecondSize);
        G_SDCard_CB_pSave += tSecondSize;
        G_SDCard_CB_Counter += mSize;
    }
}

uint8_t vTime_Seconds_Save(void)
{
    G_MicroSecond_Saved = G_MicroSecond;
    //先存储时间数据
    if(G_A0A0_Time_Seconds_IsReady == 1)
    {
        //防止缓存溢出   
        if((sizeof(G_A0A0_Time_Seconds)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
        {
            memcpy(G_A0A0_Time_Seconds+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));    
            ucCircleBuffer_SaveData(G_A0A0_Time_Seconds,sizeof(G_A0A0_Time_Seconds));  
            G_A0A0_Time_Seconds_IsReady = 0;
            G_MicroSecond_Saved = G_MicroSecond;
            return 0;
        }else
        {
            //丢包
            NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_A0A0_Time_Seconds!");
            NRF_LOG_FLUSH();
            return 1;
        } 
    }else
        return 1;    
}


/*=========================================== 任务实现 ==============================================*/
/*------------------------------------------------------------
 *  UWB启动响应任务 函数
 *------------------------------------------------------------*/
static void vTask_UWB_Start(void *pvParameters)
{
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
        if(G_SDCard_FileIsOpen == 1)
        {  
            if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 3 ) == pdTRUE)
            { 
                vSS_INIT_Start();                
                nrf_delay_us(1000); 
                xSemaphoreGive( xMutex_SDCard_CirBuffer );                  
            }else
            {
                NRF_LOG_INFO("  xMutex_IMUSPI UWB_Start TimeOUT");
                NRF_LOG_FLUSH();
            }  
        }
    }
}


/*------------------------------------------------------------
 *  UWB中断响应任务 函数
 *------------------------------------------------------------*/
static void vTask_UWB_EventHandler(void *pvParameters)
{
    uint8_t  error_code_UWB = 0;
    uint16_t tDistance = 0;
    uint8_t  tNumber = 0;
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
        
        if(G_SDCard_FileIsOpen == 1)
        {             
            if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 3 ) == pdTRUE)
            {            
#if configUWB_INIT
//发起端的中断响应处
                error_code_UWB = ucSS_INIT_Handler(&tDistance,&tNumber);
                if(error_code_UWB == 0)
                {
                    G_MicroSecond_Saved = G_MicroSecond;
                    //先存储时间数据
                    vTime_Seconds_Save();
                    
                    memcpy(G_C1C1_UWB+2,&G_MicroSecond_Saved,sizeof(G_MicroSecond_Saved));  
                    memcpy(G_C1C1_UWB+4,&tDistance,sizeof(tDistance));
                                        
                    //UWB 数据存入缓存区                   
                    if((6+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                    {
                        ucCircleBuffer_SaveData(G_C1C1_UWB,sizeof(G_C1C1_UWB));  
                    }else
                    {
                        //丢包
                        NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_UWB_Data!");
                        NRF_LOG_FLUSH();
                    }   
                  
                }else
                {
                    //UWB接收错误
                    NRF_LOG_INFO("  UWB Receive is Wrong!");
                    NRF_LOG_FLUSH();                    
                }
#else
//接收端的中断响应处理
                ucSS_RESP_Handler();                    
#endif
                
                xSemaphoreGive( xMutex_SDCard_CirBuffer ); 
            }else
            {
                NRF_LOG_INFO("  xMutex_IMUSPI UWB_EventHandler TimeOUT");
                NRF_LOG_FLUSH();
            }  
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
        //(1) 等待任务通知     
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/     
		
        //等待一下，以防SDCard后台没有存储完成	
        nrf_delay_ms(300); 
        
        erro_code = ucSDCard_CloseFile();
            
        if(erro_code == 0)
        {
            //文件关闭正确 闪烁3s 关灯
            for(i=0;i<15;i++)
            {
                nrfx_gpiote_out_toggle(configGPIO_LED_R);
                nrf_delay_ms(200);
            } 
            nrfx_gpiote_out_set(configGPIO_LED_R);
            
            NRF_LOG_INFO("File Close is Over");
            NRF_LOG_FLUSH();  
        }else
        {
            //文件关闭错误 死循环 不停闪烁
            NRF_LOG_INFO("File Close is Wrong!!!");
            NRF_LOG_FLUSH(); 
            while(1)
            {
                nrf_delay_ms(100);
                nrfx_gpiote_out_toggle(configGPIO_LED_R);            
            }
        }     	
    }
}

//堆栈溢出 提示！！！
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    NRF_LOG_INFO("OverFlow OverFlow!!!");
    NRF_LOG_FLUSH();     
}


/*------------------------------------------------------------
 *SDCard存储任务 函数
 *------------------------------------------------------------*/
static void vTask_SDCard_Save(void *pvParameters)
{
    uint8_t erro_code = 0;
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);
    
        if(G_SDCard_FileIsOpen == 1)
        {
            while(G_SDCard_CB_Counter >= configSDCard_SaveSize)
            {
                if(G_SDCard_CB_Counter >= 2048)
                    NRF_LOG_RAW_INFO("%d   ",G_SDCard_CB_Counter);
            
                G_SDCard_CB_Counter -= configSDCard_SaveSize;
                erro_code = ucSDCard_SaveData(G_SDCard_CB_pLoad,configSDCard_SaveSize);

                if(erro_code != 0)
                {
                    //存储错误，停止采集，并警告
                    G_SDCard_FileIsOpen = 0;
                    while(1)
                    {
                        nrf_delay_ms(100);
                        nrfx_gpiote_out_toggle(configGPIO_LED_R);            
                    }
                }else
                {
                    G_SDCard_CB_pLoad += configSDCard_SaveSize;
                    if(G_SDCard_CB_pLoad == (G_SDCard_CirBuffer + configSDCard_BufferSize))
                        G_SDCard_CB_pLoad = G_SDCard_CirBuffer;
                } 
            }
        }      
    }
}

/*------------------------------------------------------------
 *GPS 数据解析任务 函数
 *------------------------------------------------------------*/
static void vTask_GPSData_Decode(void *pvParameters)
{
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);       
   
        //若是在存储状态下 则进行解析
        if(G_SDCard_FileIsOpen == 1)
        {
            memset(G_Uart_Buffer2,0,configBufferUART_RX_SIZE);
            //获取数据
            memcpy(G_Uart_Buffer2,G_Uart_Buffer1,G_Uart_Buffer_Number);
            G_Uart_Buffer_Number = 0;        
           
            //进行解析
            enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)G_Uart_Buffer2);  
        
            //解析 GGA 语句 获取定位信息           
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
            {                
                //GGA语句 解析成功
                struct minmea_sentence_gga mGGA;
                if(minmea_parse_gga(&mGGA,(char*)G_Uart_Buffer2))
                {
                    //定位数据有效
                    if(mGGA.fix_quality ==1 )
                    {                           
                        //记录定位数据
                        memcpy(G_C2C2_GPS+2,&mGGA.latitude.value,sizeof(mGGA.latitude.value));
                        memcpy(G_C2C2_GPS+6,&mGGA.longitude.value,sizeof(mGGA.longitude.value));
                        int16_t mHigh;
                        uint16_t mHDop;
                        mHigh = mGGA.altitude.value;
                        mHDop = mGGA.hdop.value;
                        memcpy(G_C2C2_GPS+10,&mHigh,sizeof(mHigh));
                        memcpy(G_C2C2_GPS+12,&mHDop,sizeof(mHDop));
                        
                        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 50 ) == pdTRUE)
                        {      
                            G_GPSWeekSecond = mGGA.time.hours*3600+mGGA.time.minutes*60+mGGA.time.seconds;

                            G_GPSWeekSecond_IsValid = 1;
                            //直接存储当前GPS时间
                            memcpy(G_A0A0_Time_Seconds+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
                            ucCircleBuffer_SaveData(G_A0A0_Time_Seconds,sizeof(G_A0A0_Time_Seconds));  
                            G_A0A0_Time_Seconds_IsReady = 0;   
                   
                        
                            //再存储GPS数据
                            if((sizeof(G_C2C2_GPS)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                            {
                                ucCircleBuffer_SaveData(G_C2C2_GPS,sizeof(G_C2C2_GPS));                                     
                            }else
                            {
                                //丢包
                                NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_GPSData!");
                                NRF_LOG_FLUSH();
                            }                                 
  
                            xSemaphoreGive( xMutex_SDCard_CirBuffer ); 
                        }
                    }else
                    {
                        G_GPSWeekSecond_IsValid = 0; 
                    }
                }
                
            }
        }
    }
}


/*------------------------------------------------------------
 *  IMUA(U4) 采集MPU9255
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUA(void *pvParameters)
{
    uint8_t erro_IMUA = 0;
    uint8_t tIMUA_MPU_Data[22] = {0};
	nrf_saadc_value_t tSAResult[4] = {0}; 
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
   
        //等待 共用SPI互斥量的释放
        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 4 ) == pdTRUE)
        { 
            //采集IMUB MPU92数据
            erro_IMUA = Leo_MPU9255_Read_ALLData(tIMUA_MPU_Data);
            
            //采集压力传感器 AD 数据
            nrfx_saadc_sample_convert(0,tSAResult);
            nrfx_saadc_sample_convert(1,tSAResult+1);        
            nrfx_saadc_sample_convert(2,tSAResult+2);  
            nrfx_saadc_sample_convert(3,tSAResult+3);  
            
            //如果采集正确 则存入缓存 
            if(erro_IMUA == 0)
            {
                //先存储时间数据
                vTime_Seconds_Save();
                
                //再存入 IMUB数据包 28个字节                
                if((28+G_SDCard_CB_Counter) <= configSDCard_BufferSize)   //判断缓存是否溢出
                {
                    //存入 头标识和时间
                    memcpy(G_B1B1_IMUA_MPU_Pressure+2,&G_MicroSecond_Saved,sizeof(G_MicroSecond_Saved));
                    ucCircleBuffer_SaveData(G_B1B1_IMUA_MPU_Pressure,sizeof(G_B1B1_IMUA_MPU_Pressure));
                    //存入 IMUB数据                    
                    ucCircleBuffer_SaveData(tIMUA_MPU_Data,sizeof(tIMUA_MPU_Data));                      
                    //存入 压力数据                    
                    ucCircleBuffer_SaveData((uint8_t *)tSAResult,sizeof(tSAResult));    
                }else
                {
                    //丢包
                    NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUB_Data!");
                    NRF_LOG_FLUSH();
                }                 
                
            }else
            {
                //IMUB 数据采集错误
                NRF_LOG_INFO("  IMU_B CellectData is Wrong!");
                NRF_LOG_FLUSH();                   
            }   
            
#if configUWB_INIT             
            //UWB测距启动
//            G_UWB_Time = G_UWB_Time+1;
//            if (G_UWB_Time == 2)
//            {
//                G_UWB_Time = 0;
                if(G_SDCard_CB_Counter < configSDCard_SaveSize)
                {
                    vSS_INIT_Start();   
                }                
//            }
#endif 
            
            //释放资源
            xSemaphoreGive( xMutex_SDCard_CirBuffer );   
            
            //若是数据量到了，通知SDCard进行存储
            if(G_SDCard_CB_Counter >= configSDCard_SaveSize)
            {               
                //通知 SDCard存储
                BaseType_t xReturn = pdPASS;
                xReturn = xTaskNotify(xTaskHandle_SDCard_Save,0,eSetValueWithoutOverwrite);  
                if(xReturn == pdFAIL)
                {
                    //SDCard 通知存储失败
                    NRF_LOG_INFO("  xTaskNotify_SDCard is Wrong!");
                    NRF_LOG_FLUSH();
                }
            }                        
        }else
        {
            NRF_LOG_INFO("  xMutex_SDCDBuffer IMUA TimeOUT");
            NRF_LOG_FLUSH();
        }  
    }        
}



/*------------------------------------------------------------
 *  IMUB(U5) MPU92 Firs板子 主采集任务
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUB(void *pvParameters)
{
    uint8_t erro_IMUB = 0;
    uint8_t tIMUB_MPU_Data[22] = {0};
	nrf_saadc_value_t tSAResult[4] = {0}; 
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
   
        //等待 共用SPI互斥量的释放
        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 4 ) == pdTRUE)
        { 
            //采集IMUB MPU92数据
            erro_IMUB = Leo_MPU9255_Read_ALLData(tIMUB_MPU_Data);
            
            //采集压力传感器 AD 数据
            nrfx_saadc_sample_convert(0,tSAResult);
            nrfx_saadc_sample_convert(1,tSAResult+1);        
            nrfx_saadc_sample_convert(2,tSAResult+2);  
            nrfx_saadc_sample_convert(3,tSAResult+3);  
            
            //如果采集正确 则存入缓存 
            if(erro_IMUB == 0)
            {
                //先存储时间数据
                vTime_Seconds_Save();
                
                //再存入 IMUB数据包 28个字节                
                if((28+G_SDCard_CB_Counter) <= configSDCard_BufferSize)   //判断缓存是否溢出
                {
                    //存入 头标识和时间
                    memcpy(G_B4B4_IMUB_MPU_Pressure+2,&G_MicroSecond_Saved,sizeof(G_MicroSecond_Saved));
                    ucCircleBuffer_SaveData(G_B4B4_IMUB_MPU_Pressure,sizeof(G_B4B4_IMUB_MPU_Pressure));
                    //存入 IMUB数据                    
                    ucCircleBuffer_SaveData(tIMUB_MPU_Data,sizeof(tIMUB_MPU_Data));                      
                    //存入 压力数据                    
                    ucCircleBuffer_SaveData((uint8_t *)tSAResult,sizeof(tSAResult));    
                }else
                {
                    //丢包
                    NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUB_Data!");
                    NRF_LOG_FLUSH();
                }                 
                
            }else
            {
                //IMUB 数据采集错误
                NRF_LOG_INFO("  IMU_B CellectData is Wrong!");
                NRF_LOG_FLUSH();                   
            }   
            
#if configUWB_INIT             
            //UWB测距启动
//            G_UWB_Time = G_UWB_Time+1;
//            if (G_UWB_Time == 2)
//            {
//                G_UWB_Time = 0;
                if(G_SDCard_CB_Counter < configSDCard_SaveSize)
                {
                    vSS_INIT_Start();   
                }                
//            }
#endif 
            
            //释放资源
            xSemaphoreGive( xMutex_SDCard_CirBuffer );   
            
            //若是数据量到了，通知SDCard进行存储
            if(G_SDCard_CB_Counter >= configSDCard_SaveSize)
            {               
                //通知 SDCard存储
                BaseType_t xReturn = pdPASS;
                xReturn = xTaskNotify(xTaskHandle_SDCard_Save,0,eSetValueWithoutOverwrite);  
                if(xReturn == pdFAIL)
                {
                    //SDCard 通知存储失败
                    NRF_LOG_INFO("  xTaskNotify_SDCard is Wrong!");
                    NRF_LOG_FLUSH();
                }
            }                        
        }else
        {
            NRF_LOG_INFO("  xMutex_SDCDBuffer IMUA TimeOUT");
            NRF_LOG_FLUSH();
        }  
    }        
}



/*
 * 启动任务 */
static void vTask_TaskStart(void *pvParameters)
{
    uint8_t erro_code = 0;
    BaseType_t txResult;
    
//1.初始化任务        
    //1.初始化任务    
    /*(1) 建立SDCard存储任务 */    
    txResult = xTaskCreate(vTask_SDCard_Save,
                            "SDCardSave",
                            configMINIMAL_STACK_SIZE+256,
                            NULL,
                            taskPRIO_SDCard_Save,
                            &xTaskHandle_SDCard_Save);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }     
    
   /*(2) 建立SDCard 关闭文件任务 */      
    txResult = xTaskCreate(vTask_SDCard_Close,
                           "SDCardClose",
                           configMINIMAL_STACK_SIZE+64,
                           NULL,
                           taskPRIO_SDCard_Close,
                           &xTaskHandle_SDCard_Close);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }     

    /*(3) 建立GPS 数据解析任务 */      
    txResult = xTaskCreate(vTask_GPSData_Decode,
                           "GPSDecode",
                           configMINIMAL_STACK_SIZE+256,
                           NULL,
                           taskPRIO_GPS_RxData,
                           &xTaskHandle_GPS_RxData);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }   
    
    /*(4) 建立UWB 测距响应端任务 */      
    txResult = xTaskCreate(vTask_UWB_EventHandler,
                           "UWBResp",
                           configMINIMAL_STACK_SIZE+256,
                           NULL,
                           taskPRIO_UWB_EventHandler,
                           &xTaskHandle_UWB_EventHandler);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }   
    
    //(5) 建立IMUA 主采集任务  
    txResult = xTaskCreate(vTask_CollectData_IMUA,
                           "CollData_IMUA",
                           configMINIMAL_STACK_SIZE+128,
                           NULL,
                           taskPRIO_CollectData_IMUA,
                           &xTaskHandle_CollectData_IMUA);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }     
    
    
    //(6) 建立IMUB 采集任务  
//    txResult = xTaskCreate(vTask_CollectData_IMUB,
//                           "CollData_IMUB",
//                           configMINIMAL_STACK_SIZE+128,
//                           NULL,
//                           taskPRIO_CollectData_IMUB,
//                           &xTaskHandle_CollectData_IMUB);
//    if(txResult != pdPASS)
//    {
//       erro_code = 1;
//    }     
    
      
    
//判断结果，不正确，则红灯循环闪烁，否则常亮
    if(erro_code != 0)
    {
        NRF_LOG_INFO(("||Initialize||-->Task Initializaiton is Wrong!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH(); 
        while(1)
        {
            nrf_delay_ms(100);
            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
        }
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R);
        NRF_LOG_INFO(("||Initialize||-->Task Initializaiton is OK!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH();  
    }
    
    
//2.初始化外设      
    //（1）全局变量初始化
    vINIT_Variable();   
    
    //（2） GPIO管脚初始化    
    erro_code |= nrfx_gpiote_init();    
    //LED 管脚 
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    erro_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //输出1，LED灯灭    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH();      
    
    
    //（3） 初始化SDCard 并建立存储文件  
    erro_code |= ucSDCard_INIT();  
    if(erro_code == 0)
    {
        G_SDCard_FileIsOpen = 1;
    }
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
 
    //（4） 初始化 IMUB(MPU92)only——First板子 
//    erro_code |= ucIMU_INIT_IMUBOnly_MPU(); 
//    NRF_LOG_INFO(("||Initialize||-->IMUA(MPU)_IMUB(ADIS)->error  0x%x"),erro_code);  
    
    //（4） 初始化 IMUA(MPU92)only——First板子 
    erro_code |= ucIMU_INIT_IMUAOnly_MPU(); 
    NRF_LOG_INFO(("||Initialize||-->IMUA(MPU)_IMUB(ADIS)->error  0x%x"),erro_code);  
    
    //（5） 初始化 SAADC 压力传感器
    erro_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),erro_code); 
    NRF_LOG_FLUSH(); 
    
    //（6）初始化 UWB
#if configUWB_INIT
    erro_code |= ucSS_INIT_Initial();
#else
    erro_code |= ucSS_RESP_Initial();
#endif
    NRF_LOG_INFO(("||Initialize||-->UWB------------->error  0x%x"),erro_code);     
    NRF_LOG_FLUSH(); 
    
    //（7）初始化计时器并启动 
    erro_code |= ucTimerInitial_3();        //1ms 计时 
    erro_code |= ucTimerStart_3();          
//    erro_code |= ucTimerInitial_4();        //FreeRTOS 任务分析用
//    erro_code |= ucTimerStart_4();
     
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
    
    //（8）初始化中断并启动
    erro_code |= ucINTInital_SDCard();    /* SDCard中断管脚初始化 */    
    erro_code |= ucINTInital_PPS();       /* 1PPS秒脉冲中断管脚初始化 */
    erro_code |= ucINTInital_UWB();
    erro_code |= ucINTInital_IMUA();
    //erro_code |= ucINTInital_IMUB();
    ucINTStart_SDCard();
    ucINTStart_PPS();
    ucINTStart_UWB();
    ucINTStart_IMUA();
    //ucINTStart_IMUB();
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),erro_code);   
    NRF_LOG_FLUSH(); 
    
    //（9）初始化串口并启动
    erro_code |= ucUARTInital_GPS();
    NRF_LOG_INFO(("||Initialize||-->GPS_Uart-------->error  0x%x"),erro_code); 
    
//判断结果，不正确，则红灯循环闪烁，否则常亮
    if(erro_code != 0)
    {
        NRF_LOG_INFO(("||Initialize||-->Task Initializaiton is Wrong!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH();
        G_SDCard_FileIsOpen = 0;
        while(1)
        {
            nrf_delay_ms(100);
            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
        }
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R); 
        NRF_LOG_INFO(("||Initialize||-->Task Initializaiton is OK!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH();        
    }    
    
//TEST 用于测试任务申请空间是否不够用
//    uint8_t pcWriteBuffer[200];
//    NRF_LOG_INFO("=================================================");
//    NRF_LOG_INFO("\nname      namestate  priority   rest   number");
//    vTaskList((char *)&pcWriteBuffer);
//    NRF_LOG_INFO("\n%s",pcWriteBuffer);
//    NRF_LOG_FLUSH();

//    NRF_LOG_INFO("=================================================");
//    NRF_LOG_INFO("\nname       counter         reate");
//    vTaskGetRunTimeStats((char *)&pcWriteBuffer);
//    NRF_LOG_RAW_INFO("\n%s",pcWriteBuffer);
//    NRF_LOG_FLUSH();    
    
    
//3.删除任务释放空间
    vTaskDelete(xTaskHandle_TaskStart); 
}





/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;

    /*(1) 互斥量的建立 */
    //_SDCard缓存
    xMutex_SDCard_CirBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCard_CirBuffer == NULL)
    {
        erro_code = 1;
    }       
    
    //GPS数据缓存 的 二值信号量
    xSemaphore_GPSBuffer = xSemaphoreCreateBinary();
    if(xSemaphore_GPSBuffer == NULL)
    {
        erro_code = 1;
    }
    
    /*(2) 建立启动任务 */    
    txResult = xTaskCreate(vTask_TaskStart,
                            "TaskStart",
                            configMINIMAL_STACK_SIZE+128,
                            NULL,
                            taskPRIO_TaskStart,
                            &xTaskHandle_TaskStart);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }   
       
//判断结果，不正确，则红灯循环闪烁，否则常亮
    if(erro_code != 0)
    {
        NRF_LOG_INFO(("||Initialize||-->vTask_CreatTask is Wrong!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH();
        while(1)
        {
            nrf_delay_ms(100);
            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
        }
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R); 
        NRF_LOG_INFO(("||Initialize||-->vTask_CreatTask is OK!!->error  0x%x"),erro_code);
        NRF_LOG_FLUSH();        
    }  
    
   
    return erro_code;    
}


