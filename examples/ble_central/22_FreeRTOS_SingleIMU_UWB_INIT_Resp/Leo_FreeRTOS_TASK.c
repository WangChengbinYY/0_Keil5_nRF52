/*
*********************************************************************************************************
*
*    ģ������ : FreeRTOS������ʵ��
*    �ļ����� :Leo_FreeRTOS_TASK
*    ��    �� : V1.0
*    ˵    �� : ��Ŀ����������Ľ���
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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
#include "Leo_UWB.h"
#include "Leo_SAADC.h"
#include "Leo_UART.h"
#include "Leo_IMU.h"


//ȫ�ֱ���_ʱ����� 
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ǿ����������
uint8_t	    G_MAG_Coeffi[6]; 

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ŵĻ���                
uint8_t	    G_IMU_Data_A[27];                   //��һ��IMU_A(U4)��ŵ�����
uint8_t	    G_IMU_Data_B[27];                   //�ڶ���IMU_A(U5)��ŵ�����

//ȫ�ֱ���_ѹ������������
uint8_t     G_FOOTPresure[17];

//ȫ�ֱ���_GPS��λ����
uint8_t     G_GPSData[41];
uint8_t     G_GPSData_IsReady;
//ȫ�ֱ���_UWB�������
uint8_t     G_UWBData[12];
uint8_t     G_UWBData_IsReady;

// ȫ�ֱ���_SDCard�洢����                                                         
uint8_t	    G_CollectData1[configBuffer_SDCard_Max];                 //SDCardҪ�������ݵĻ���
uint16_t    G_CollectData1_Counter;  
uint8_t	    G_CollectData2[configBuffer_SDCard_Max];                 //SDCardҪ�������ݵĻ���


uint16_t    G_Collect_PreTime;                                      //ǰһ�β����ļ��ʱ��

uint8_t     G_SDCardSave_IsOverflowe;

// ȫ�ֱ���_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0


uint8_t     G_Uart_Buffer1[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer2[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer_Number;


//TEST
uint32_t mNumber_Collect = 0;
uint32_t mNumber_UWB = 0;



/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */


/* 1�� */
#define taskPRIO_SDCard_Close                1          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 2�� */
#define taskPRIO_SDCard_Save                 2          //SDCard�洢����

/* 3�� */
#define taskPRIO_GPS_RxData                  3          //����GPS���ݲ������������ɹ���֪ͨ�洢	

/* 4�� */
#define taskPRIO_CollectData           	     4 			         

/* 5�� */
#define taskPRIO_UWB_EventHandler            5			//UWB��Ӧ�� ������ȼ� 

/* 6�� */
#define taskPRIO_TaskStart                   6          //�������� 

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*����GPS��������       ��� */
TaskHandle_t    xTaskHandle_UWB_EventHandler    = NULL;         
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*10ms�����Ĳɼ�����    ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */
TaskHandle_t    xTaskHandle_TaskStart           = NULL;



/* ȫ�ֱ���_������_SDCard����  */
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;
//��ֵ�ź��������� GPS���ݽ����Ļ���ʹ��
SemaphoreHandle_t   xSemaphore_GPSBuffer        = NULL;


/**
 * ȫ�ֱ�����ʼ������   ������  
*/
void vINIT_Variable(void)
{
    //ȫ�ֱ���_ʱ����� 
    G_GPSWeekSecond     = 0;                    //GPS����������
    G_MicroSecond       = 0;                    //nRF52ʱ����������Ƶ� 1s��1000����ֵ��
    
    //ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ǿ����������
    memset(G_MAG_Coeffi,0,6);
    G_MAG_Coeffi[5] = 0xFF;     
    
    //ȫ�ֱ���_IMU_A(U4)����
    memset(G_IMU_Data_A,0,27);
    G_IMU_Data_A[0] = 0xB1;
    G_IMU_Data_A[1] = 0xB1;
    G_IMU_Data_A[26] = 0xFF;
    memset(G_IMU_Data_B,0,27);
    G_IMU_Data_B[0] = 0xB2;
		G_IMU_Data_B[1] = 0xB2;
    G_IMU_Data_B[26] = 0xFF;     
    
    //ȫ�ֱ���_ѹ������������
    memset(G_FOOTPresure,0,17);
    G_FOOTPresure[0] = 0xC1;
		G_FOOTPresure[1] = 0xC1;
    G_FOOTPresure[16] = 0xFF;    
  
    //ȫ�ֱ���_GPS��λ����
    memset(G_GPSData,0,41);
    G_GPSData[0] = 0xD1;
    G_GPSData[1] = 0xD1;
    G_GPSData[40] = 0xFF;  
    G_GPSData_IsReady = 0;
    
    //ȫ�ֱ���_UWB�������
    memset(G_UWBData,0,12);
    G_UWBData[0] = 0xE1;
    G_UWBData[1] = 0xE1;
    G_UWBData[11] = 0xFF; 
    G_UWBData_IsReady = 0;
    
    // ȫ�ֱ���_SDCard�洢����        
    memset(G_CollectData1,0,configBuffer_SDCard_Max);
    memset(G_CollectData2,0,configBuffer_SDCard_Max);
    G_CollectData1_Counter = 0;    
    G_Collect_PreTime = 0;
    G_SDCardSave_IsOverflowe = 0;
    
    //ȫ�ֱ���_SDCard�ļ�������ʶ 
    G_SDCard_FileIsOpen = 0;                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0 

    //ȫ�ֱ���_���ڽ��ջ��� 
    memset(G_Uart_Buffer1,0,configBufferUART_RX_SIZE);    
    memset(G_Uart_Buffer1,0,configBufferUART_RX_SIZE); 
    G_Uart_Buffer_Number = 0;
    
}





/*=========================================== ����ʵ�� ==============================================*/
/*------------------------------------------------------------
 *SDCard�ر��ļ����� ����
 *------------------------------------------------------------*/
static void vTask_UWB_EventHandler(void *pvParameters)
{
    uint8_t  error_code_UWB = 0;
    uint16_t tDistance = 1000;;
    uint8_t  tNumber = 50;
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
        
        if(G_SDCard_FileIsOpen == 1)
        { 
            
            //TEST
//            NRF_LOG_INFO("2 UWB Resp");
//            NRF_LOG_FLUSH();             
            
            error_code_UWB = ucSS_INIT_Handler(&tDistance,&tNumber);
            
            //��װ��ȡ�����ݽ��� ����洢
            //(1)�洢ʱ�� ������
            memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));  
            memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
            memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
            
			G_UWBData_IsReady = 1;              
        }     
    }
}




/*------------------------------------------------------------
 *SDCard�ر��ļ����� ����
 *------------------------------------------------------------*/
static void vTask_SDCard_Close(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        //(1) �ȴ�����֪ͨ     
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/     
			
        nrf_delay_ms(200);
        
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

//��������
//void vApplicationIdleHook(void)
//{
//    NRF_LOG_FLUSH();
//}



/*------------------------------------------------------------
 *SDCard�洢���� ����
 *------------------------------------------------------------*/
static void vTask_SDCard_Save(void *pvParameters)
{
    uint16_t mTime = 0;
    
    uint8_t erro_code = 0;    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);
        
        if(G_SDCard_FileIsOpen == 1)
        {  
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 10 ) == pdTRUE)
            {
                mTime = G_MicroSecond;
                memcpy(G_CollectData2,G_CollectData1,configBuffer_SDCard_Save);
                if(G_CollectData1_Counter > configBuffer_SDCard_Save)
                {
                    G_CollectData1_Counter = G_CollectData1_Counter - configBuffer_SDCard_Save;
                    memcpy(G_CollectData1,(G_CollectData1+configBuffer_SDCard_Save),G_CollectData1_Counter);
                }                
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer );               
                
                erro_code = ucSDCard_SaveData(G_CollectData1,configBuffer_SDCard_Save); 
                
                if(G_MicroSecond < mTime)            
                    mTime = G_MicroSecond +1000 - mTime;
                else
                    mTime = G_MicroSecond  - mTime; 
                if(mTime > 100)
                {
                    //G_SDCardSave_IsOverflowe = 5;
                
                    //TEST
                    NRF_LOG_INFO("    3 used ms: %d, MS:%d, S:%d",mTime,G_MicroSecond,G_GPSWeekSecond);
                    NRF_LOG_FLUSH(); 
                }
			}
            
            if(erro_code != 0)
            {
                //�洢����ֹͣ�ɼ���������
                G_SDCard_FileIsOpen = 0;
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
 *GPS ���ݽ������� ����
 *------------------------------------------------------------*/
static void vTask_GPSData_Decode(void *pvParameters)
{
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);       
   
        //�����ڴ洢״̬�� ����н���
        if(G_SDCard_FileIsOpen == 1)
        {
            memset(G_Uart_Buffer2,0,configBufferUART_RX_SIZE);
            //��ȡ����
            memcpy(G_Uart_Buffer2,G_Uart_Buffer1,G_Uart_Buffer_Number);
            G_Uart_Buffer_Number = 0;
        
            //���н���
            enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)G_Uart_Buffer2);  

            //���� RMC��� ��ȡ ʱ����Ϣ ��������
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
            {
                NRF_LOG_INFO("GPS  RMC");
                NRF_LOG_FLUSH(); 
                struct minmea_sentence_rmc mRMC;
                if(minmea_parse_rmc(&mRMC,(char*)G_Uart_Buffer2))
                {  
                    //����Ч������ ��ȡ ʱ����Ϣ
                    if(mRMC.valid == 1)
                    {
                        //��������ʱ���� ת��Ϊ ������
                        UTC2GPS(mRMC.date.year,mRMC.date.month,mRMC.date.day,mRMC.time.hours,mRMC.time.minutes,mRMC.time.seconds,&G_GPSWeekSecond);
                        memcpy(G_GPSData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
                    }           
                }        
            }
        
            //���� GGA ��� ��ȡ��λ��Ϣ           
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
            {
                NRF_LOG_INFO("GPS  GGA");
                NRF_LOG_FLUSH(); 
                //GGA��� �����ɹ�
                struct minmea_sentence_gga mGGA;
                if(minmea_parse_gga(&mGGA,(char*)G_Uart_Buffer2))
                {
                    if(mGGA.fix_quality ==1 || mGGA.fix_quality == 2)
                    {   
                        //��ȡ��Ч����
                        memcpy(G_GPSData+6,&G_MicroSecond,sizeof(G_MicroSecond));
                        memcpy(G_GPSData+8,&mGGA.longitude.value,sizeof(mGGA.longitude.value));
                        memcpy(G_GPSData+12,&mGGA.longitude.scale,sizeof(mGGA.longitude.scale));
                        memcpy(G_GPSData+16,&mGGA.latitude.value,sizeof(mGGA.latitude.value));
                        memcpy(G_GPSData+20,&mGGA.latitude.scale,sizeof(mGGA.latitude.scale));
                        memcpy(G_GPSData+24,&mGGA.altitude.value,sizeof(mGGA.altitude.value));
                        memcpy(G_GPSData+28,&mGGA.altitude.scale,sizeof(mGGA.altitude.scale));
                        memcpy(G_GPSData+32,&mGGA.hdop.value,sizeof(mGGA.hdop.value));
                        memcpy(G_GPSData+36,&mGGA.hdop.scale,sizeof(mGGA.hdop.scale));

                        G_GPSData_IsReady = 1;
                        
//                        //��ȡ ������ ��������
//                        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 500 ) == pdTRUE)
//                        {
//                            //�Է����������
//                            if((G_CollectData1_Counter + sizeof(G_GPSData)) <= configBuffer_SDCard_Max)
//                            {
//                                memcpy(G_CollectData1+G_CollectData1_Counter,G_GPSData,sizeof(G_GPSData));
//                                G_CollectData1_Counter = G_CollectData1_Counter + sizeof(G_GPSData); 
//                            }                            
//                            //�ͷ���Դ
//                            xSemaphoreGive( xMutex_SDCDBuffer );  
//                            
//                            if(G_CollectData1_Counter >= configBuffer_SDCard_Save)
//                            {
//                                xTaskNotify(xTaskHandle_SDCard_Save,     
//                                            0,              
//                                            eNoAction); 
//                            }
//                            
//                            
//                         }else
//                        {
//                            //LOEDEBUG
//                            NRF_LOG_INFO("SDCard_Buffer is Busy for GPS!!!!!!!!!!!!");
//                            NRF_LOG_FLUSH(); 
//                         }
                    }
                }
                
            }
        }

        
    }
}



/*------------------------------------------------------------
 *�ⲿ���������ݲɼ�����Ҫ������
 *  MPU9255A��MPU9255B��ѹ��������
 *------------------------------------------------------------*/
static void vTask_CollectData(void *pvParameters)
{
    uint8_t mtest = 0;
    
    uint16_t TempTime = 0;    
    uint8_t error_code_Foot = 0;
	nrf_saadc_value_t tSAResult[4] = {0};
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);               
        
        if(G_SDCard_FileIsOpen == 1)
        {
    //0.ȷ�����β������������10ms
            if(G_MicroSecond < G_Collect_PreTime)            
                TempTime = G_MicroSecond +1000 - G_Collect_PreTime;
            else
                TempTime = G_MicroSecond  - G_Collect_PreTime;            
            G_Collect_PreTime = G_MicroSecond;            
            if(TempTime<10)
            {
                vTaskDelay(10-TempTime);
                mtest = 10-TempTime;
            }
            
//            //TEST
//            NRF_LOG_INFO("1 CollectData Wait ms:%d",mtest);
//            NRF_LOG_FLUSH();   
            
    //1. �ɼ�IMU����        
            //(1)��¼ʱ������
            memcpy(G_IMU_Data_A+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_IMU_Data_A+6,&G_MicroSecond,sizeof(G_MicroSecond));       
            
            //(2)�ɼ�IMU_A ������
            //ѡ��IMU_A nCS�ܽ�
//            nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
//            nrf_delay_us(1); 
//            //�ɼ�IMU_A ������
//            Leo_MPU9255_Read_ACC(G_IMU_Data_A+8);
//            Leo_MPU9255_Read_Gyro(G_IMU_Data_A+14);
//            Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+20);
//            //�ر�IMU_A nCS�ܽ�
//            nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);    
            
    //2. �ɼ�ѹ������������
            //(1)��¼ʱ������
            memcpy(G_FOOTPresure+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_FOOTPresure+6,&G_MicroSecond,sizeof(G_MicroSecond));   
            //(2)�ɼ� AD ����  
            //�ɼ� AD ͨ��������
            error_code_Foot |= nrfx_saadc_sample_convert(0,tSAResult);
            error_code_Foot |= nrfx_saadc_sample_convert(1,tSAResult+1);        
            error_code_Foot |= nrfx_saadc_sample_convert(2,tSAResult+2);  
            error_code_Foot |= nrfx_saadc_sample_convert(3,tSAResult+3);    
            if(error_code_Foot == 0)
            {
                memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult));
            }   
            
    //4. ���ݷ��뻺��        
            /*���ɼ�����,����洢 �ȴ�10ms�������û���ͷţ�������˴δ洢*/
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 10 ) == pdTRUE)
            {
                //��ֹ�������
                if((sizeof(G_IMU_Data_A)+sizeof(G_FOOTPresure)+G_CollectData1_Counter)<=configBuffer_SDCard_Max)
                {
                    //IMU_A���� ���뻺��
                    memcpy(G_CollectData1+G_CollectData1_Counter,G_IMU_Data_A,sizeof(G_IMU_Data_A));
                    G_CollectData1_Counter = G_CollectData1_Counter + sizeof(G_IMU_Data_A);
                    //ѹ������������ ���뻺��
                    if(error_code_Foot == 0)
                    {
                        memcpy(G_CollectData1+G_CollectData1_Counter,G_FOOTPresure,sizeof(G_FOOTPresure));
                        G_CollectData1_Counter = G_CollectData1_Counter + sizeof(G_FOOTPresure);
                    }
                }
                
                //����GPS����
                if(G_GPSData_IsReady == 1)
                {
                    G_GPSData_IsReady = 0;
                    if((sizeof(G_GPSData)+G_CollectData1_Counter)<=configBuffer_SDCard_Max)
                    {
                        memcpy(G_CollectData1+G_CollectData1_Counter,G_GPSData,sizeof(G_GPSData));
                        G_CollectData1_Counter = G_CollectData1_Counter + sizeof(G_GPSData);
//                        NRF_LOG_INFO("I have GPS Data!");
//                        NRF_LOG_FLUSH(); 
                    }
                }
                
                //����UWB����
                if(G_UWBData_IsReady == 1)
                {
                    G_UWBData_IsReady = 0;
                    if((sizeof(G_UWBData)+G_CollectData1_Counter)<=configBuffer_SDCard_Max)
                    {
                        memcpy(G_CollectData1+G_CollectData1_Counter,G_UWBData,sizeof(G_UWBData));
                        G_CollectData1_Counter = G_CollectData1_Counter + sizeof(G_UWBData);
                    }  
                }			
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
            }else
            {
                //TEST
                NRF_LOG_INFO("CollectData wait Buffer is Wrong!");
                NRF_LOG_FLUSH(); 
            }
            
    //5. ����UWB���          
            vSS_INIT_Start();
            
    //6. ���������ݴ��� SDCard��          
            //����SDCard ���� ����UWB���
            if(G_CollectData1_Counter > configBuffer_SDCard_Save)
            {               
                //֪ͨ SDCard�洢
                xTaskNotify(xTaskHandle_SDCard_Save,0,eNoAction);   
            }
            
        }
	}
}


/*
 * �������� */
static void vTask_TaskStart(void *pvParameters)
{
    uint8_t erro_code = 0;
    BaseType_t txResult;
    
//1.��ʼ������    
    /*(1) ����SDCard�洢���� */    
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
    
   /*(2) ����SDCard �ر��ļ����� */      
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

    /*(3) ����GPS ���ݽ������� */      
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
    
    /*(4) ����UWB �����Ӧ������ */      
    txResult = xTaskCreate(vTask_UWB_EventHandler,
                           "UWBResp",
                           configMINIMAL_STACK_SIZE,
                           NULL,
                           taskPRIO_UWB_EventHandler,
                           &xTaskHandle_UWB_EventHandler);
    if(txResult != pdPASS)
    {
       erro_code = 1;
    }   
    
    //(5) �����ɼ�����  
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

//�жϽ��������ȷ������ѭ����˸��������
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
    
    
//2.��ʼ������      
    //��1��ȫ�ֱ�����ʼ��
    vINIT_Variable();   
    
    //��2�� GPIO�ܽų�ʼ��    
    erro_code |= nrfx_gpiote_init();    
    //LED �ܽ� 
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    erro_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //���1��LED����    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH();    
    
    //��3�� ��ʼ��SDCard �������洢�ļ�  
    erro_code |= ucSDCard_INIT();  
    if(erro_code == 0)
    {
        G_SDCard_FileIsOpen = 1;
    }
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
 
    //��4�� ��ʼ�� IMU 
//    erro_code |= ucIMUInitial();
//    NRF_LOG_INFO(("||Initialize||-->IMU------------>error  0x%x"),erro_code);    
    
    //��5�� ��ʼ�� SAADC ѹ��������
    erro_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),erro_code); 
    NRF_LOG_FLUSH(); 
    
    //��6����ʼ�� UWB
    erro_code |= ucSS_INIT_Initial();
    NRF_LOG_INFO(("||Initialize||-->UWB------------->error  0x%x"),erro_code);     
    NRF_LOG_FLUSH(); 
    
    //��7����ʼ����ʱ��������
    erro_code |= ucTimerInitial_2();
    erro_code |= ucTimerInitial_3();      /* TIMER3 ��������ʼ��*/ 
    erro_code |= ucTimerInitial_4();
    erro_code |= ucTimerStart_2();
    erro_code |= ucTimerStart_3();      /* TIMER3 ��������ʼ��*/ 
    erro_code |= ucTimerStart_4(); 
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
    
    //��8����ʼ���жϲ�����
    erro_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    erro_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    erro_code |= ucINTInital_UWB();
    ucINTStart_SDCard();
    ucINTStart_PPS();
    ucINTStart_UWB();    
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),erro_code);   
    NRF_LOG_FLUSH(); 
    
    //��9����ʼ�����ڲ�����
    erro_code |= ucUARTInital_GPS();
    NRF_LOG_INFO(("||Initialize||-->GPS_Uart-------->error  0x%x"),erro_code); 
    
//�жϽ��������ȷ������ѭ����˸��������
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
    
    
    
    
//3.ɾ�������ͷſռ�
    vTaskDelete(xTaskHandle_TaskStart); 
}





/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;

    /*(1) �������Ľ��� */
    //_SDCard����
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    //GPS���ݻ��� �� ��ֵ�ź���
    xSemaphore_GPSBuffer = xSemaphoreCreateBinary();
    if(xSemaphore_GPSBuffer == NULL)
    {
        erro_code = 1;
    }
    
    /*(2) ������������ */    
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
       
//�жϽ��������ȷ������ѭ����˸��������
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


