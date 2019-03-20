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
#include "Leo_INT.h"
#include "Leo_TIMER.h"
#include "Leo_SDCard.h"
#include "minmea.h"
#include "Leo_UWB.h"


//全局变量_时间参数 
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0

//全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
uint8_t	    G_MAG_Coeffi[6]; 

//全局变量_IMU_A(U4)和IMU_B(U5)存放的缓存                
uint8_t	    G_IMU_Data_A[27];                   //第一组IMU_A(U4)存放的数据
uint8_t	    G_IMU_Data_B[27];                   //第二组IMU_A(U5)存放的数据

//全局变量_压力传感器数据
uint8_t     G_FOOTPresure[17];

//全局变量_GPS定位数据
uint8_t     G_GPSData[41];
uint8_t     G_GPSData_IsReady;
//全局变量_UWB测距数据
uint8_t     G_UWBData[12];
uint8_t     G_UWBData_IsReady;
uint8_t     G_UWBData_IsComing;

// 全局变量_SDCard存储缓存                                                         
uint8_t	    G_CollectData[configBuffer_SDCard_Max];                 //SDCard要储存数据的缓存
uint16_t    G_CollectData_Counter;  
// 全局变量_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0


extern uint8_t G_UART_Buffer2[128];
extern uint8_t G_UART_Buffer2_Counter;


//TEST
uint32_t mNumber_Collect = 0;
uint32_t mNumber_UWB = 0;



/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */


/* 1级 */
#define taskPRIO_SDCard_Close                1          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 2级 */
#define taskPRIO_GPS_RxData                  2          //接收GPS数据并解析，解析成功，通知存储

/* 3级 */
#define taskPRIO_CollectData           	     3 	

/* 4级 */
//#define taskPRIO_UWB_EventHandler            4					//UWB响应端 的任务等级			         

/* 5级 */
#define taskPRIO_SDCard_Save                 5          //SDCard存储数据 

/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*解析GPS串口数据       句柄 */
TaskHandle_t    xTaskHandle_UWB_EventHandler    = NULL;         
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*10ms触发的采集任务    句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */

/* 全局变量_互斥量_SDCard缓存  */
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;
//二值信号量，用于 GPS数据解析的缓存使用
SemaphoreHandle_t   xSemaphore_GPSBuffer        = NULL;


/**
 * 全局变量初始化函数   待完善  
*/
void vINIT_Variable(void)
{
    //全局变量_时间参数 
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
    
    //全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
    memset(G_MAG_Coeffi,0,6);
    G_MAG_Coeffi[5] = 0xFF;     
    
    //全局变量_IMU_A(U4)数据
    memset(G_IMU_Data_A,0,27);
    G_IMU_Data_A[0] = 0xB1;
    G_IMU_Data_A[1] = 0xB1;
    G_IMU_Data_A[26] = 0xFF;
    memset(G_IMU_Data_B,0,27);
    G_IMU_Data_B[0] = 0xB2;
		G_IMU_Data_B[1] = 0xB2;
    G_IMU_Data_B[26] = 0xFF;     
    
    //全局变量_压力传感器数据
    memset(G_FOOTPresure,0,17);
    G_FOOTPresure[0] = 0xC1;
		G_FOOTPresure[1] = 0xC1;
    G_FOOTPresure[16] = 0xFF;    
  
    //全局变量_GPS定位数据
    memset(G_GPSData,0,41);
    G_GPSData[0] = 0xD1;
    G_GPSData[1] = 0xD1;
    G_GPSData[40] = 0xFF;  
    G_GPSData_IsReady = 0;
    //全局变量_UWB测距数据
    memset(G_UWBData,0,12);
    G_UWBData[0] = 0xE1;
    G_UWBData[1] = 0xE1;
    G_UWBData[11] = 0xFF; 
    G_UWBData_IsReady = 0;
    G_UWBData_IsComing = 0;
    // 全局变量_SDCard存储缓存        
    memset(G_CollectData,0,configBuffer_SDCard_Max);
    G_CollectData_Counter = 0;    
    //全局变量_SDCard文件操作标识 
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0 
}





///*=========================================== 任务实现 ==============================================*/
///*------------------------------------------------------------
// *SDCard关闭文件任务 函数
// *------------------------------------------------------------*/
//static void vTask_UWB_EventHandler(void *pvParameters)
//{
//    uint8_t  error_code_UWB = 0;
//    uint16_t tDistance = 1000;;
//    uint8_t  tNumber = 50;
//    while(1)
//    {
//        xTaskNotifyWait(0x00000000,     
//                0xFFFFFFFF,     
//                NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
//                portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/ 

//        
//        if(G_SDCard_FileIsOpen == 1)
//        { 

////        NRF_LOG_INFO("vTask_UWB_EventHandler  RX NOTE!");
////        NRF_LOG_FLUSH();
//            
//            error_code_UWB = ucSS_INIT_Handler(&tDistance,&tNumber);
//            
//            //假装获取了数据进行 试验存储
//            //(1)存储时间 和数据
//            memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
//            memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));  
//            memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
//            memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
//            
//			G_UWBData_IsReady = 1;  
//        }     
//    }
//}




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
			
        nrf_delay_ms(200);
        
//		NRF_LOG_INFO("File Close is OK!!!");
//        NRF_LOG_FLUSH();  
        
        erro_code = ucSDCard_CloseFile();
            
        if(erro_code == 0)
            {
                for(i=0;i<15;i++)
                {
                    nrfx_gpiote_out_toggle(configGPIO_LED_R);
                    nrf_delay_ms(200);
                } 
                nrfx_gpiote_out_set(configGPIO_LED_R);
                
                NRF_LOG_INFO("File Close is OK!!!");
                NRF_LOG_FLUSH();  
            }else
		        {
				        NRF_LOG_INFO("File Close is Wrong!!!");
                NRF_LOG_FLUSH(); 
				        while(1)
				            {
					            nrf_delay_ms(150);
					            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
				            }
			      }	
    }
}



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
        xTaskNotifyWait(0x00000000,     
                0xFFFFFFFF,     
                NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/ 
        
//        NRF_LOG_INFO("vTask_SDCard_Save  RX NOTE!");
//        NRF_LOG_FLUSH(); 
//        
//        nrf_delay_ms(5);
         
        
        if(G_SDCard_FileIsOpen == 1)
        {  
//        NRF_LOG_INFO("vTask_SDCard_Save  RX NOTE!");
//        NRF_LOG_FLUSH();
            
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 10 ) == pdTRUE)
            {
                erro_code = ucSDCard_SaveData(G_CollectData,G_CollectData_Counter); 
                G_CollectData_Counter = 0;
                //释放资源
                xSemaphoreGive( xMutex_SDCDBuffer ); 
			}
            
            if(erro_code != 0)
            {
                NRF_LOG_INFO("SDCard Save is Wrong!!!!!!!!!! %d",erro_code);
                NRF_LOG_FLUSH(); 
                while(1)
                {
                    nrf_delay_ms(100);
                    nrfx_gpiote_out_toggle(configGPIO_LED_R);            
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
        uint8_t mChar;

        
        
        
//            NRF_LOG_INFO("                     vTask_GPSData_Decode  RX NOTE!");
//            NRF_LOG_FLUSH(); 
            
            nrf_delay_ms(2);
                        G_GPSData_IsReady = 1;
            
        /*    
        //若是在存储状态下 则进行解析
        if(G_SDCard_FileIsOpen == 1)
        {
            //获取数据
            memcpy(mData,G_UART_Buffer2,G_UART_Buffer2_Counter);
        
            //进行解析
            enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mData);  

            //解析 RMC语句 获取 时间信息 年月日秒
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
            {
                struct minmea_sentence_rmc mRMC;
                if(minmea_parse_rmc(&mRMC,(char*)mData))
                {  
                    //从有效数据中 读取 时间信息
                    if(mRMC.valid == 1)
                    {
                        //将年月日时分秒 转化为 周内秒
                        UTC2GPS(mRMC.date.year,mRMC.date.month,mRMC.date.day,mRMC.time.hours,mRMC.time.minutes,mRMC.time.seconds,&G_GPSWeekSecond);
                        memcpy(G_GPSData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
                    }           
                }        
            }
        
            //解析 GGA 语句 获取定位信息           
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
            {
                //GGA语句 解析成功
                struct minmea_sentence_gga mGGA;
                if(minmea_parse_gga(&mGGA,(char*)mData))
                {
                    if(mGGA.fix_quality ==1 || mGGA.fix_quality == 2)
                    {   
                        //获取有效数据
                        memcpy(G_GPSData+6,&G_MicroSecond,sizeof(G_MicroSecond));
                        memcpy(G_GPSData+8,&mGGA.longitude.value,sizeof(mGGA.longitude.value));
                        memcpy(G_GPSData+12,&mGGA.longitude.scale,sizeof(mGGA.longitude.scale));
                        memcpy(G_GPSData+16,&mGGA.latitude.value,sizeof(mGGA.latitude.value));
                        memcpy(G_GPSData+20,&mGGA.latitude.scale,sizeof(mGGA.latitude.scale));
                        memcpy(G_GPSData+24,&mGGA.altitude.value,sizeof(mGGA.altitude.value));
                        memcpy(G_GPSData+28,&mGGA.altitude.scale,sizeof(mGGA.altitude.scale));
                        memcpy(G_GPSData+32,&mGGA.hdop.value,sizeof(mGGA.hdop.value));
                        memcpy(G_GPSData+36,&mGGA.hdop.scale,sizeof(mGGA.hdop.scale));

                        //获取 互斥量 存入数据
                        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 500 ) == pdTRUE)
                        {
                            //以防缓存区溢出
                            if((G_CollectData_Counter + sizeof(G_GPSData)) <= configBuffer_SDCard_Max)
                            {
                                memcpy(G_CollectData+G_CollectData_Counter,G_GPSData,sizeof(G_GPSData));
                                G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSData); 
                            }                            
                            //释放资源
                            xSemaphoreGive( xMutex_SDCDBuffer );  
                            
                            if(G_CollectData_Counter >= configBuffer_SDCard_Save)
                            {
                                xTaskNotify(xTaskHandle_SDCard_Save,     
                                            0,              
                                            eNoAction); 
                            }
                            
                            
                         }else
                        {
                            //LOEDEBUG
                            NRF_LOG_INFO("SDCard_Buffer is Busy for GPS!!!!!!!!!!!!");
                            NRF_LOG_FLUSH(); 
                         }
                    }
                }
                
            }
        }
        */
//        }
    }
}

/*------------------------------------------------------------
 *外部传感器数据采集，主要包括：
 *  MPU9255A、MPU9255B、压力传感器
 *------------------------------------------------------------*/
static void vTask_CollectData(void *pvParameters)
{
    uint8_t error_code_Foot = 0;
	nrf_saadc_value_t tSAResult[4] = {0};
    while(1)
    {
        xTaskNotifyWait(0x00000000,     
                0xFFFFFFFF,     
                NULL,                 /* 保存ulNotifiedValue到变量ulValue中 如果不用可以设为NULL */
                portMAX_DELAY);       /* 最大允许延迟时间 portMAX_DELAY 表示永远等待*/ 
//TEST        
 
//        nrf_delay_ms(2);
//        //通知 SDCard存储
//        xTaskNotify(xTaskHandle_SDCard_Save,0, eNoAction);  
        

        
        if(G_SDCard_FileIsOpen == 1)
        {
//        NRF_LOG_INFO("vTimer_CollectData  RX NOTE!");
//        NRF_LOG_FLUSH();
            
    //1. 采集IMU数据        
            //(1)记录时间数据
            memcpy(G_IMU_Data_A+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_IMU_Data_A+6,&G_MicroSecond,sizeof(G_MicroSecond));       
            
            //(2)采集IMU_A 的数据
            //选择IMU_A nCS管脚
            nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
            nrf_delay_us(1); 
            //采集IMU_A 的数据
            Leo_MPU9255_Read_ACC(G_IMU_Data_A+8);
            Leo_MPU9255_Read_Gyro(G_IMU_Data_A+14);
            Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+20);
            //关闭IMU_A nCS管脚
            nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);    
            
    //2. 采集压力传感器数据
            //(1)记录时间数据
            memcpy(G_FOOTPresure+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_FOOTPresure+6,&G_MicroSecond,sizeof(G_MicroSecond));   
            //(2)采集 AD 数据  
            //采集 AD 通道的数据
            error_code_Foot |= nrfx_saadc_sample_convert(0,tSAResult);
            error_code_Foot |= nrfx_saadc_sample_convert(1,tSAResult+1);        
            error_code_Foot |= nrfx_saadc_sample_convert(2,tSAResult+2);  
            error_code_Foot |= nrfx_saadc_sample_convert(3,tSAResult+3);    
            if(error_code_Foot == 0)
            {
                memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult));
            }            
            
            //vSS_INIT_Start();
            
    //4. 数据进行存储        
            /*都采集完了,整体存储 等待10ms，如果还没有释放，则放弃此次存储*/
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 10 ) == pdTRUE)
            {
                //防止缓存溢出
                if((sizeof(G_IMU_Data_A)+sizeof(G_FOOTPresure)+G_CollectData_Counter)<=configBuffer_SDCard_Max)
                {
                    //IMU_A数据 存入缓存
                    memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data_A,sizeof(G_IMU_Data_A));
                    G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data_A);
                    //压力传感器数据 存入缓存
                    if(error_code_Foot == 0)
                    {
                        memcpy(G_CollectData+G_CollectData_Counter,G_FOOTPresure,sizeof(G_FOOTPresure));
                        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_FOOTPresure);
                    }
                }
                //存入GPS数据
                if(G_GPSData_IsReady == 1)
                {
                    G_GPSData_IsReady = 0;
                    if((sizeof(G_GPSData)+G_CollectData_Counter)<=configBuffer_SDCard_Max)
                    {
                        memcpy(G_CollectData+G_CollectData_Counter,G_GPSData,sizeof(G_GPSData));
                        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSData);
                    }
                    NRF_LOG_INFO("                           I Have G_GPSData!");
                }
                //存入UWB数据
                if(G_UWBData_IsReady == 1)
                {
                    G_UWBData_IsReady = 0;
                    if((sizeof(G_UWBData)+G_CollectData_Counter)<=configBuffer_SDCard_Max)
                    {
                        memcpy(G_CollectData+G_CollectData_Counter,G_UWBData,sizeof(G_UWBData));
                        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_UWBData);
                    }
                    
                    if((G_MicroSecond % 500) ==0 )
                    {   
                        NRF_LOG_INFO("I Have UWB Data!");
                        NRF_LOG_FLUSH();
                    }
                   
                }			
                //释放资源
                xSemaphoreGive( xMutex_SDCDBuffer ); 
            }
            
            
            if(G_CollectData_Counter > configBuffer_SDCard_Save)
            {
                //通知 SDCard存储
                xTaskNotify(xTaskHandle_SDCard_Save,0,eNoAction);   
            }else
            {
                uint16 tDistance;
                uint8_t tNumber;
                ucSS_INIT_RUN(tDistance,tNumber);
                memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
                memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));  
                memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
                memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
                G_UWBData_IsReady = 1;
            }
            
            
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
    TickType_t xTimer = 200;
    /*(1) 互斥量的建立 */
    //_SDCard缓存
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    //GPS数据缓存 的 二值信号量
    xSemaphore_GPSBuffer = xSemaphoreCreateBinary();
    if(xSemaphore_GPSBuffer == NULL)
    {
        erro_code = 1;
    }     
    
    /*(2) 建立SDCard存储任务 */    
    txResult = xTaskCreate(vTask_SDCard_Save,
                            "SDCardSave",
                            configMINIMAL_STACK_SIZE+1024,
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

    /*(4) 建立GPS 数据解析任务 */      
   txResult = xTaskCreate(vTask_GPSData_Decode,
                           "GPSDecode",
                           configMINIMAL_STACK_SIZE,
                           NULL,
                           taskPRIO_GPS_RxData,
                           &xTaskHandle_GPS_RxData);
   if(txResult != pdPASS)
   {
       erro_code = 1;
   }   
    
    /*(5) 建立UWB 测距响应端任务 */      
//    txResult = xTaskCreate(vTask_UWB_EventHandler,
//                           "UWBResp",
//                           configMINIMAL_STACK_SIZE,
//                           NULL,
//                           taskPRIO_UWB_EventHandler,
//                           &xTaskHandle_UWB_EventHandler);
//    if(txResult != pdPASS)
//    {
//       erro_code = 1;
//    }   
    
    //(6) 建立采集任务  
    txResult = xTaskCreate(vTask_CollectData,
                           "CollectData",
                           configMINIMAL_STACK_SIZE,
                           NULL,
                           taskPRIO_CollectData,
                           &xTaskHandle_CollectData);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    } 
    
    
    
//    (6) 建立采集任务 10ms定时器 
    // xTimerHandle_CollectData = xTimerCreate("10ms",
                                            // xTimer,
                                            // pdTRUE,
                                            // (void *)1,
                                            // vTimer_CollectData);
    // if(xTimerHandle_CollectData == NULL)
    // {
        // erro_code = 1;
    // }else{
        // if(xTimerStart(xTimerHandle_CollectData,1000) != pdPASS)
        // {
            // erro_code = 1;
            // NRF_LOG_INFO("vTimer_CollectData Start is Wrong!");
            // NRF_LOG_FLUSH(); 
        // }else{
            // NRF_LOG_INFO("vTimer_CollectData Start is OK!");
            // NRF_LOG_FLUSH(); 
        // }
        
    // }    
    
    
//    NRF_LOG_INFO("TEST:   SDCard INT is ok!");
//    NRF_LOG_FLUSH();

//    uint8_t pcWriteBuffer[300];
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
        
    
    
    
    return erro_code;    
}


