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




//全局变量_时间参数 
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0

//全局变量和IMU_B(U5)存放的缓存    
uint8_t	    G_IMU_Data_B_ADIS[25];               //第二组IMU_A(U5)存放的数据

//全局变量_压力传感器数据
uint8_t     G_FOOTPresure[17];


// 全局变量_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;                                    //标记是否已经打开文件 没打开，默认为0

// 全局变量_SDCard存储缓存  
uint8_t     G_SDCard_CirBuffer[configSDCard_BufferSize];
uint8_t*    G_SDCard_CB_pSave = NULL;
uint8_t*    G_SDCard_CB_pLoad = NULL;
uint16_t    G_SDCard_CB_Counter; 


/*=========================================== 任务优先级设定 ============================================*/
/* 0级 */
#define taskPRIO_SDCard_Close                0          //SDCard关闭文件成功  标志位置0  数据不会存储

/* 1级 */
#define taskPRIO_SDCard_Save                 1          //SDCard存储数据 

/* 5级 */
#define taskPRIO_CollectData_IMUB            5          //采集 IMUB(U5)的数据 以ADIS或MTI的采集为主				         

/* 7级 */
#define taskPRIO_TaskStart                   7          //启动任务 

/*=========================================== 任务相关变量 ============================================*/
/**
 * 全局变量_任务函数句柄
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard 关闭文件任务  句柄 */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard存储任务       句柄 */

TaskHandle_t    xTaskHandle_CollectData_IMUB    = NULL;         //IMUB(U5) 副IMU数据采集任务

TaskHandle_t    xTaskHandle_TaskStart           = NULL;

///* 全局变量_互斥量_SDCard缓存  */
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;
/* 全局变量_互斥量_SPI IMU使用缓存  */
SemaphoreHandle_t   xMutex_IMUSPI               = NULL;


/**
 * 全局变量初始化函数   待完善  
*/
void vINIT_Variable(void)
{
    //全局变量_时间参数 
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
    
    
    memset(G_IMU_Data_B_ADIS,0,25);
    G_IMU_Data_B_ADIS[0] = 0xB3;
    G_IMU_Data_B_ADIS[1] = 0xB3;
    G_IMU_Data_B_ADIS[24] = 0xFF;     
    
    //全局变量_压力传感器数据
    memset(G_FOOTPresure,0,17);
    G_FOOTPresure[0] = 0xC1;
    G_FOOTPresure[1] = 0xC1;
    G_FOOTPresure[16] = 0xFF;    
  
    
    //全局变量_SDCard文件操作标识 
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0 

    //SDCard存储缓存
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
        nrf_delay_ms(200); 
        
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


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    NRF_LOG_INFO("OverFlow OverFlow!!!");
    NRF_LOG_FLUSH(); 
    
}


/*------------------------------------------------------------
 *SDCard存储任务 函数
 *------------------------------------------------------------*/
//static void vTimerTask_SDCard_Save( TimerHandle_t xTimer )
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
 *  IMUB(U5) 副IMU数据采集任务及存储
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUB(void *pvParameters)
{
    uint8_t erro_IMUB = 0;
    uint8_t tIMUB_ADIS_Data[22] = {0};
	nrf_saadc_value_t tSAResult[4] = {0};
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 

//1. 采集 IMUB 的数据        
        //等待 共用SPI互斥量的释放
        if(xSemaphoreTake( xMutex_IMUSPI, ( TickType_t ) 4 ) == pdTRUE)
        { 
            //打开IMU_B的片选管脚

            memcpy(G_IMU_Data_B_ADIS+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_IMU_Data_B_ADIS+6,&G_MicroSecond,sizeof(G_MicroSecond));             
            erro_IMUB = Leo_ADIS_Read_ALLData(tIMUB_ADIS_Data,22);
            memcpy(G_IMU_Data_B_ADIS+8,tIMUB_ADIS_Data+4,16);    
        
            xSemaphoreGive( xMutex_IMUSPI ); 
        }else
        {
            NRF_LOG_INFO("  xMutex_IMUSPI IMUB TimeOUT");
            NRF_LOG_FLUSH();
        } 

            
//2. 采集压力传感器数据
        //采集 AD 通道的数据
        nrfx_saadc_sample_convert(0,tSAResult);
        nrfx_saadc_sample_convert(1,tSAResult+1);        
        nrfx_saadc_sample_convert(2,tSAResult+2);  
        nrfx_saadc_sample_convert(3,tSAResult+3);    
        //问题：没有判断采集数据的正确性!          
        memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult)); 
            
//3. 进入存储判断       
        /*都采集完了,整体存储 等待5ms(IMUA 200Hz)，如果还没有释放，则放弃此次存储*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
        {
            //(1) 存入IMUB 的数据

            //防止缓存溢出
            if((sizeof(G_IMU_Data_B_ADIS)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
            {
                //IMU_A数据 存入缓存
                if(erro_IMUB == 0)
                {
                    ucCircleBuffer_SaveData(G_IMU_Data_B_ADIS,sizeof(G_IMU_Data_B_ADIS));
                }else
                {
                    //IMUB 数据采集错误
                    NRF_LOG_INFO("  IMU_B CellectData is Wrong!");
                    NRF_LOG_FLUSH();                    
                }
            }else
            {
                //丢包
                NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUA!");
                NRF_LOG_FLUSH();
            }

            
            //(2) 存入压力传感器数据
            if((sizeof(G_FOOTPresure)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
            {
                //压力传感器 数据 存入缓存
                ucCircleBuffer_SaveData(G_FOOTPresure,sizeof(G_FOOTPresure));
            }else
            {
                //丢包
                NRF_LOG_INFO("  SDCard Buffer is OverFlow_FOOT!");
                NRF_LOG_FLUSH();
            }    
          
            //释放资源
            xSemaphoreGive( xMutex_SDCDBuffer );   
        }else
        {
            NRF_LOG_INFO("  xMutex_SDCDBuffer IMUA TimeOUT");
            NRF_LOG_FLUSH();
        }  
            
         
//5. 将缓存数据存入 SDCard中          
        if(G_SDCard_CB_Counter >= configSDCard_SaveSize)
        {               
            //通知 SDCard存储
            BaseType_t xReturn = pdPASS;
            xReturn = xTaskNotify(xTaskHandle_SDCard_Save,0,eSetValueWithoutOverwrite);  
            if(xReturn == pdFAIL)
            {
                //TEST
//                    NRF_LOG_INFO("     MessageOverFlow_____vTask_SDCarSave");
//                    NRF_LOG_FLUSH(); 
            }
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
    
    //(6) 建立IMUB 采集任务  
    txResult = xTaskCreate(vTask_CollectData_IMUB,
                           "CollData_IMUB",
                           configMINIMAL_STACK_SIZE+128,
                           NULL,
                           taskPRIO_CollectData_IMUB,
                           &xTaskHandle_CollectData_IMUB);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }     
  
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
 
    //（4） 初始化 IMU 
    //这里仅初始化IMU_B 的ADIS
    erro_code |= ucIMU_INIT_MPU_ADIS();
    NRF_LOG_INFO(("||Initialize||-->IMU------------>error  0x%x"),erro_code);    
    
    //（5） 初始化 SAADC 压力传感器
    erro_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),erro_code); 
    NRF_LOG_FLUSH(); 
    
    //（6）初始化 UWB
    
    //（7）初始化计时器并启动
    erro_code |= ucTimerInitial_3();        //1ms 计时 
    erro_code |= ucTimerStart_3();     
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
    
    //（8）初始化中断并启动
    erro_code |= ucINTInital_SDCard();    /* SDCard中断管脚初始化 */    
    erro_code |= ucINTInital_IMUB();
    ucINTStart_SDCard();
    ucINTStart_IMUB();
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),erro_code);   
    NRF_LOG_FLUSH(); 
    
    //（9）初始化串口并启动
    
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
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    //SPI数据采集
    xMutex_IMUSPI = xSemaphoreCreateMutex();
    if(xMutex_IMUSPI == NULL)
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


