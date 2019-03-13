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
uint8_t	    G_IMUDataA_Counter;                  //MPU9255中断触发的计数器	    
uint8_t	    G_IMUDataB_Counter;

//全局变量_压力传感器数据
uint8_t     G_FOOTPresure[17];

//全局变量_GPS定位数据
uint8_t     G_GPSData[41];

//全局变量_UWB测距数据
uint8_t     G_UWBData[12];


// 全局变量_SDCard存储缓存                                                         
uint8_t	    G_CollectData[512];                 //SDCard要储存数据的缓存
uint16_t    G_CollectData_Counter;  
// 全局变量_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0


extern uint8_t G_UART_Buffer2[512];
extern uint8_t G_UART_Buffer2_Counter;




/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */
#define taskPRIO_SDCard_Close                0          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 1级 */
#define taskPRIO_GPS_RxData                  1          //接收GPS数据并解析，解析成功，通知存储 

/* 2级 */


/* 3级 */
//10ms 循环的时间 采集任务                   3

/* 4级 */
#define taskPRIO_SDCard_Save                 4          //SDCard存储数据

/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*解析GPS串口数据       句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */

        
TimerHandle_t     xTimerHandle_CollectData        = NULL;         /*5ms触发的采集任务    句柄 */


/**
 * 全局变量_互斥量_SDCard缓存  
*/
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
    G_IMUDataA_Counter = 0;                  //IMU采集的次数计数值	    
    G_IMUDataB_Counter = 0;      
    
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
   
    //全局变量_UWB测距数据
    memset(G_UWBData,0,12);
    G_UWBData[0] = 0xE1;
	G_UWBData[1] = 0xE1;
    G_UWBData[11] = 0xFF; 
    
    
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
            NRF_LOG_FLUSH();  
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
        if((G_CollectData_Counter > 256) && (G_SDCard_FileIsOpen == 1) )
        {
            uint16_t    tTestStart = 0;   
            tTestStart = G_MicroSecond;
            
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //释放资源
                xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                erro_code = ucSDCard_SaveData(tData,tData_Count); 
                //TEST
                //erro_code = ucSDCard_SaveData(tData,1024);                 

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
            
            tTestStart = G_MicroSecond-tTestStart;
            NRF_LOG_INFO("The Time of SDCard is %d ms and Number is %d!",tTestStart,tData_Count);
            NRF_LOG_FLUSH(); 
            
        }else{
            continue;
        }
        

    }        
}

/*------------------------------------------------------------
 *GPS 数据解析任务 函数
 *------------------------------------------------------------*/
static void vTask_GPSData_Decode(void *pvParameters)
{
    uint8_t mData[128];
    while(1)
    {
        memset(mData,0,128);
        //二值信号量 等待
        xSemaphoreTake(xSemaphore_GPSBuffer, portMAX_DELAY);
        
        //获取数据
        memcpy(mData,G_UART_Buffer2,G_UART_Buffer2_Counter);
//        NRF_LOG_INFO("%s",mData);
        
        //进行解析
        enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mData);  

        //解析 RMC语句
        if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
        {
            //TEST
//            NRF_LOG_INFO("Find the RMC!");
//            NRF_LOG_FLUSH();
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
        
        //解析 GGA 语句            
        if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
        {
            //TEST
//            NRF_LOG_INFO("Find the GGA!");
//            NRF_LOG_FLUSH();
            
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
                    
                    //TEST
//                    NRF_LOG_INFO("The Lat is %d, The Lon is %d!",mGGA.latitude.value,mGGA.longitude.value);
//                    NRF_LOG_FLUSH();
                    
                    if(G_SDCard_FileIsOpen == 1)
                    {
                        //获取 互斥量 存入数据
                        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
                        {
                            memcpy(G_CollectData+G_CollectData_Counter,G_GPSData,sizeof(G_GPSData));
                            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSData); 
                            
                            //释放资源
                            xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                            //通知 SDCard存储任务
                            xTaskNotify(xTaskHandle_SDCard_Save,     
                                        0,              
                                        eNoAction);  
                            
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
        
    }
}




/*------------------------------------------------------------
 *外部传感器数据采集，主要包括：
 *  MPU9255A、MPU9255B、压力传感器
 *------------------------------------------------------------*/
static void vTimer_CollectData(xTimerHandle pxTimer)
{
    if(G_SDCard_FileIsOpen == 1)
    {
        uint16_t    tTestStart = 0;   
        tTestStart = G_MicroSecond;
        uint8_t error_code_Foot = 0;
        uint8_t error_code_UWB = 0;
        uint16_t tDistance = 0;
        uint8_t  tNumber = 0;
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
        nrf_saadc_value_t tSAResult[4] = {0}; 
        //采集 AD 通道的数据
        error_code_Foot |= nrfx_saadc_sample_convert(0,tSAResult);
        error_code_Foot |= nrfx_saadc_sample_convert(1,tSAResult+1);        
        error_code_Foot |= nrfx_saadc_sample_convert(2,tSAResult+2);  
        error_code_Foot |= nrfx_saadc_sample_convert(3,tSAResult+3);    
        if(error_code_Foot == 0)
        {
            memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult));
        }
        
        //转换
//        for(i=0;i<4;i++)
//        {
//            sVoltResult[i] = tSAResult[i]*3.6/1024.0;
//            tTest[i] = (nrf_saadc_value_t)(sVoltResult[i]*100);            
//        }
//        NRF_LOG_INFO("AD error:%d,Point6:%d;Point7:%d;Point5:%d;Point2:%d;",error_code,tTest[0],tTest[1],tTest[2],tTest[3]);
//        NRF_LOG_FLUSH(); 
        
//3. 采集 UWB测距 数据       
        error_code_UWB = ucSS_INIT_RUN(&tDistance,&tNumber);
//        NRF_LOG_INFO("The UWB  %d!",error_code_UWB);
//        NRF_LOG_FLUSH(); 
        if(error_code_UWB == 0)
        {
            //记录时间数据
            memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
            memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));
            //记录测量 的次数序号
            memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
            //记录测量 的距离数据
            memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
//            NRF_LOG_INFO("The %d of Distance is %d mm!",tNumber,tDistance);
//            NRF_LOG_FLUSH(); 
        }

//4. 采集数据进行存储        
        /*都采集完了,整体存储 等待2ms，如果还没有释放，则放弃此次存储*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
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
            //UWB测距数据存入缓存
            if(error_code_UWB == 0)
            {
                memcpy(G_CollectData+G_CollectData_Counter,G_UWBData,sizeof(G_UWBData));
                G_CollectData_Counter = G_CollectData_Counter + sizeof(G_UWBData);                
            }         
            
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
        
        
        
        tTestStart = G_MicroSecond-tTestStart;
//        NRF_LOG_INFO("             The Time of uesed is %d ms!",tTestStart);
//        NRF_LOG_FLUSH(); 
    }
}

/*-----------------------------------------------------------------------*/
/* 创建任务                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    TickType_t xTimer = 10;
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

    /*(4) 建立GPS 数据解析任务 */      
    txResult = xTaskCreate(vTask_GPSData_Decode,
                            "GPSDecode",
                            configMINIMAL_STACK_SIZE+120,
                            NULL,
                            taskPRIO_GPS_RxData,
                            &xTaskHandle_GPS_RxData);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }       
    
    
    //(4) 建立采集任务 10ms定时器 
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
            NRF_LOG_INFO("vTimer_CollectData Start is Wrong!");
            NRF_LOG_FLUSH(); 
        }else{
            NRF_LOG_INFO("vTimer_CollectData Start is OK!");
            NRF_LOG_FLUSH(); 
        }
        
    }    
    
    
    
    return erro_code;    
}




