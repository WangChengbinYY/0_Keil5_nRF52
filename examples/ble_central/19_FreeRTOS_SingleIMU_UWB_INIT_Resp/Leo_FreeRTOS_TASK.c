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


//ȫ�ֱ���_ʱ����� 
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ǿ����������
uint8_t	    G_MAG_Coeffi[6]; 

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ŵĻ���                
uint8_t	    G_IMU_Data_A[27];                   //��һ��IMU_A(U4)��ŵ�����
uint8_t	    G_IMU_Data_B[27];                   //�ڶ���IMU_A(U5)��ŵ�����
uint8_t	    G_IMUDataA_Counter;                  //MPU9255�жϴ����ļ�����	    
uint8_t	    G_IMUDataB_Counter;

//ȫ�ֱ���_ѹ������������
uint8_t     G_FOOTPresure[17];

//ȫ�ֱ���_GPS��λ����
uint8_t     G_GPSData[41];

//ȫ�ֱ���_UWB�������
uint8_t     G_UWBData[12];


// ȫ�ֱ���_SDCard�洢����                                                         
uint8_t	    G_CollectData[configBuffer_SDCard_Max];                 //SDCardҪ�������ݵĻ���
uint16_t    G_CollectData_Counter;  
// ȫ�ֱ���_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0


extern uint8_t G_UART_Buffer2[128];
extern uint8_t G_UART_Buffer2_Counter;


//TEST
uint32_t mNumber_Collect = 0;
uint32_t mNumber_UWB = 0;



/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */


/* 1�� */
#define taskPRIO_SDCard_Close                1          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 2�� */
#define taskPRIO_GPS_RxData                  2          //����GPS���ݲ������������ɹ���֪ͨ�洢

/* 3�� */
//10ms ѭ����ʱ�� �ɼ�����                    3

/* 4�� */
#define taskPRIO_UWB_EventHandler            4          //UWB��Ӧ�� ������ȼ�

/* 5�� */
#define taskPRIO_SDCard_Save                 5          //SDCard�洢���� 

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_UWB_EventHandler    = NULL;         
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*����GPS��������       ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */

        
TimerHandle_t     xTimerHandle_CollectData        = NULL;         /*5ms�����Ĳɼ�����    ��� */


/* ȫ�ֱ���_������_SDCard����  */
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;
//��ֵ�ź��������� GPS���ݽ����Ļ���ʹ��
SemaphoreHandle_t   xSemaphore_GPSBuffer        = NULL;
/* ȫ�ֱ���_������_ �ɼ�Ȩ��  */
SemaphoreHandle_t   xMutex_SPI                  = NULL;


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
    G_IMUDataA_Counter = 0;                  //IMU�ɼ��Ĵ�������ֵ	    
    G_IMUDataB_Counter = 0;      
    
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
   
    //ȫ�ֱ���_UWB�������
    memset(G_UWBData,0,12);
    G_UWBData[0] = 0xE1;
	G_UWBData[1] = 0xE1;
    G_UWBData[11] = 0xFF; 
    
    
    // ȫ�ֱ���_SDCard�洢����        
    memset(G_CollectData,0,configBuffer_SDCard_Max);
    G_CollectData_Counter = 0;    
    //ȫ�ֱ���_SDCard�ļ�������ʶ 
    G_SDCard_FileIsOpen = 0;                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0 
}





/*=========================================== ����ʵ�� ==============================================*/
/*------------------------------------------------------------
 *SDCard�ر��ļ����� ����
 *------------------------------------------------------------*/
static void vTask_UWB_EventHandler(void *pvParameters)
{
    uint8_t error_code_UWB = 0;
    uint16_t tDistance = 100;
    uint8_t  tNumber = 5;
    while(1)
    {
        xTaskNotifyWait(0x00000000,     
                0xFFFFFFFF,     
                NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/  
//TEST
//NRF_LOG_INFO("UWB INIT Handler!");
//NRF_LOG_FLUSH();  

    //0. ��ȡȨ�� ������ ����10ms �����˴� �ɼ�     
//        if(xSemaphoreTake( xMutex_SPI, ( TickType_t ) 10 ) == pdTRUE)
//        {
//TEST
//NRF_LOG_INFO(" UWB Have SPI");
//NRF_LOG_FLUSH();            
        if(G_SDCard_FileIsOpen == 1)
        {            
            error_code_UWB = ucSS_INIT_Handler(&tDistance,&tNumber);
            
            //��װ��ȡ�����ݽ��� ����洢
            //(1)�洢ʱ�� ������
            memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));  
            memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
            memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
            
            //�ͷ���Դ
//            xSemaphoreGive( xMutex_SPI ); 
//TEST
//NRF_LOG_INFO(" UWB Release SPI");
//NRF_LOG_FLUSH();             
            //TEST
//            NRF_LOG_INFO("UWB Handler is MSecond %d ",G_MicroSecond);
//            NRF_LOG_FLUSH(); 
            
            //(2)��ȡ��Դ �洢����
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
            {
                //��ֹ�������
                if((sizeof(G_UWBData)+G_CollectData_Counter)<=configBuffer_SDCard_Max)
                {
                    //UWB���� ���뻺��
                    memcpy(G_CollectData+G_CollectData_Counter,G_UWBData,sizeof(G_UWBData));
                    G_CollectData_Counter = G_CollectData_Counter + sizeof(G_UWBData);
                }
                
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                //֪ͨ SDCard�洢����            
                if(G_CollectData_Counter >= 2048)
                {
                    xTaskNotify(xTaskHandle_SDCard_Save,     
                                0,              
                                eNoAction);   
                }                
               
            }else
            {
                //LOEDEBUG
                NRF_LOG_INFO("SDCard_Buffer is Busy for UWB UWB UWB UWB!!!");
                NRF_LOG_FLUSH(); 
            }    
//        }else
//        {
//            //LOEDEBUG
//            NRF_LOG_INFO("UWB   Data  Giveup!!");
//            NRF_LOG_FLUSH(); 
//        }  
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
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/     
        
        //(2)��ȡ��Դ �ȴ�500ms
//        if(xSemaphoreTake( xMutex_SPI, ( TickType_t ) 500 ) == pdTRUE)
//        {
            //(3)�ر��ļ��洢
            erro_code = ucSDCard_CloseFile();
            
            //������ɣ��ͷ�Ȩ��
//            xSemaphoreGive( xMutex_SPI ); 
//TEST
//NRF_LOG_INFO(" CollectData Release SPI");
//NRF_LOG_FLUSH();             
            //(4)LED�ƿ���
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
        
//        }else
//        {
//            //LOEDEBUG
//            NRF_LOG_INFO("SDCard Close  Giveup!!");
//            NRF_LOG_FLUSH(); 
//        }
    }
}

/*------------------------------------------------------------
 *SDCard�洢���� ����
 *------------------------------------------------------------*/
static void vTask_SDCard_Save(void *pvParameters)
{
    uint8_t tData[configBuffer_SDCard_Max] = {0};
    uint16_t tData_Count = 0;
    uint8_t erro_code = 0;
//TEST
    uint16_t    tTestStart = 0;   
    tTestStart = G_MicroSecond;       
    
    while(1)
    {
        /*(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/
        
        /*(2) �ж��Ƿ�洢     */
        if((G_CollectData_Counter >= configBuffer_SDCard_Save) && (G_SDCard_FileIsOpen == 1) )
        {                
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 10 ) == pdTRUE)
            {
                //(3)��ȡ1024���ֽ�
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                if(tData_Count > configBuffer_SDCard_Save)
                {
                    G_CollectData_Counter = tData_Count - configBuffer_SDCard_Save;
                    memcpy(G_CollectData,tData+configBuffer_SDCard_Save,G_CollectData_Counter);
                }
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
    //TEST
    //NRF_LOG_INFO("SDCard Save!");
    //NRF_LOG_FLUSH(); 
                    
                //��3����ȡ����Ȩ�� �ӳ�20ms
//                if(xSemaphoreTake( xMutex_SPI, ( TickType_t ) 20 ) == pdTRUE)
//                {
//TEST
//NRF_LOG_INFO(" SDCardSave Have SPI");
//NRF_LOG_FLUSH();  
                    //xTimerStop(xTimerHandle_CollectData,0);                   
                    tTestStart = G_MicroSecond;
                    erro_code = ucSDCard_SaveData(tData,configBuffer_SDCard_Save); 
                    //LEODEBUG
                    if(erro_code != 0)
                    {
                        NRF_LOG_INFO("SDCard Save is Wrong!!!!!!!!!! %d",erro_code);
                        NRF_LOG_FLUSH();                     
                    }
                    //xTimerStart(xTimerHandle_CollectData,0);
                    
                    //�ͷ���Դ
//                    xSemaphoreGive( xMutex_SPI ); 
//TEST
//NRF_LOG_INFO(" SDCardSave Release SPI");
//NRF_LOG_FLUSH();                     
                    
//                }else
//                {
//                    //LOEDEBUG
//                    NRF_LOG_INFO("SDCard Save Giveup!!");
//                    NRF_LOG_FLUSH(); 
//                }                      
                    
            }else
            {
                //LOEDEBUG
                NRF_LOG_INFO("SDCard_Buffer is Busy for Save!!!!!!!!!!");
                NRF_LOG_FLUSH(); 
            }
                
//                //TEST
                tTestStart = G_MicroSecond-tTestStart;
//                if(tTestStart > 10)
                {
                    NRF_LOG_INFO("The Time of SDCard is %d ms and Number is %d!  %d",tTestStart,tData_Count,G_GPSWeekSecond);
                    NRF_LOG_FLUSH(); 
                }
         }else{
                continue;
         }

    }        
}

/*------------------------------------------------------------
 *GPS ���ݽ������� ����
 *------------------------------------------------------------*/
static void vTask_GPSData_Decode(void *pvParameters)
{
    uint8_t mData[128];
    while(1)
    {
        memset(mData,0,128);
        //��ֵ�ź��� �ȴ�
        xSemaphoreTake(xSemaphore_GPSBuffer, portMAX_DELAY);
////TEST
//NRF_LOG_INFO("GPS Decode!");
//NRF_LOG_FLUSH(); 
        
        //�����ڴ洢״̬�� ����н���
        if(G_SDCard_FileIsOpen == 1)
        {
            //��ȡ����
            memcpy(mData,G_UART_Buffer2,G_UART_Buffer2_Counter);
        
            //���н���
            enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mData);  

            //���� RMC��� ��ȡ ʱ����Ϣ ��������
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
            {
                struct minmea_sentence_rmc mRMC;
                if(minmea_parse_rmc(&mRMC,(char*)mData))
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
                //GGA��� �����ɹ�
                struct minmea_sentence_gga mGGA;
                if(minmea_parse_gga(&mGGA,(char*)mData))
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

                        //��ȡ ������ ��������
                        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 500 ) == pdTRUE)
                        {
                            //�Է����������
                            if((G_CollectData_Counter + sizeof(G_GPSData)) <= configBuffer_SDCard_Max)
                            {
                                memcpy(G_CollectData+G_CollectData_Counter,G_GPSData,sizeof(G_GPSData));
                                G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSData); 
                            }                            
                            //�ͷ���Դ
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
        
    }
}




/*------------------------------------------------------------
 *�ⲿ���������ݲɼ�����Ҫ������
 *  MPU9255A��MPU9255B��ѹ��������
 *------------------------------------------------------------*/
static void vTimer_CollectData(xTimerHandle pxTimer)
{
    uint16_t    tTestStart = 0;   
    tTestStart = G_MicroSecond;
    uint8_t error_code_Foot = 0;
    uint16_t tDistance = 100;
    uint8_t  tNumber = 5;
    
    
//0.���ݴ洢״̬    
    if(G_SDCard_FileIsOpen == 1)
    {
//TEST
//NRF_LOG_INFO("Collect Data!");
//NRF_LOG_FLUSH();    
        
    //0. ��ȡȨ�� ������ ����10ms �����˴� �ɼ�     
//        if(xSemaphoreTake( xMutex_SPI, ( TickType_t ) 10 ) == pdTRUE)
//        {
//TEST
//NRF_LOG_INFO(" CollectData Have SPI");
//NRF_LOG_FLUSH();            
    //1. �ɼ�IMU����        
            //(1)��¼ʱ������
            memcpy(G_IMU_Data_A+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_IMU_Data_A+6,&G_MicroSecond,sizeof(G_MicroSecond));       
            
            //(2)�ɼ�IMU_A ������
            //ѡ��IMU_A nCS�ܽ�
            nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
            nrf_delay_us(1); 
            //�ɼ�IMU_A ������
            Leo_MPU9255_Read_ACC(G_IMU_Data_A+8);
            Leo_MPU9255_Read_Gyro(G_IMU_Data_A+14);
            Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+20);
            //�ر�IMU_A nCS�ܽ�
            nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);      

    //2. �ɼ�ѹ������������
            //(1)��¼ʱ������
            memcpy(G_FOOTPresure+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond)); 
            memcpy(G_FOOTPresure+6,&G_MicroSecond,sizeof(G_MicroSecond));   
            //(2)�ɼ� AD ����        
            nrf_saadc_value_t tSAResult[4] = {0}; 
            //�ɼ� AD ͨ��������
            error_code_Foot |= nrfx_saadc_sample_convert(0,tSAResult);
            error_code_Foot |= nrfx_saadc_sample_convert(1,tSAResult+1);        
            error_code_Foot |= nrfx_saadc_sample_convert(2,tSAResult+2);  
            error_code_Foot |= nrfx_saadc_sample_convert(3,tSAResult+3);    
            if(error_code_Foot == 0)
            {
                memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult));
            }
    //3. �ɼ���ɣ��ͷ�Ȩ��
//            xSemaphoreGive( xMutex_SPI ); 
//TEST
//NRF_LOG_INFO(" CollectData Release SPI");
//NRF_LOG_FLUSH(); 
            
    //4. �ɼ����ݽ��д洢        
            /*���ɼ�����,����洢 �ȴ�5ms�������û���ͷţ�������˴δ洢*/
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
            {
                //��ֹ�������
                if((sizeof(G_IMU_Data_A)+sizeof(G_FOOTPresure)+G_CollectData_Counter)<=configBuffer_SDCard_Max)
                {
                    //IMU_A���� ���뻺��
                    memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data_A,sizeof(G_IMU_Data_A));
                    G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data_A);
                    //ѹ������������ ���뻺��
                    if(error_code_Foot == 0)
                    {
                        memcpy(G_CollectData+G_CollectData_Counter,G_FOOTPresure,sizeof(G_FOOTPresure));
                        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_FOOTPresure);
                    }
                }
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                if(G_CollectData_Counter >= configBuffer_SDCard_Save)
                {
    //TEST
    //NRF_LOG_INFO("Collect Note SDCard 512 Save!");
    //NRF_LOG_FLUSH();                 
                    //֪ͨ SDCard�洢����
                    xTaskNotify(xTaskHandle_SDCard_Save,     
                                0,              
                                eNoAction);   
                }else
                {
    //TEST
    //NRF_LOG_INFO("UWB Start!");
    //NRF_LOG_FLUSH();                 
                    //Ϊ�˱����ͻ������SDCard�洢�������ڣ������в�࣡��
                    vSS_INIT_Start();
                }    
                   
            }else
            {
                //LOEDEBUG
                NRF_LOG_INFO("SDCard_Buffer is Busy for IMU!!!!!!!!!!!!");
                NRF_LOG_FLUSH(); 
            }        
           
    //        tTestStart = G_MicroSecond-tTestStart;
    //        NRF_LOG_INFO("             The Time of uesed is %d ms!",tTestStart);
    //        NRF_LOG_FLUSH(); 
//        }else
//        {
//            //LOEDEBUG
////            NRF_LOG_INFO("Collect   Data  Giveup!!  Second %d",G_GPSWeekSecond);
////            NRF_LOG_FLUSH(); 
//        }        
    }
}

/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    TickType_t xTimer = 10;
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
    //�ɼ�Ȩ�޻�����
    xMutex_SPI = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    
    
    
    
    /*(2) ����SDCard�洢���� */    
    txResult = xTaskCreate(vTask_SDCard_Save,
                            "SDCardSave",
                            configMINIMAL_STACK_SIZE+600,
                            NULL,
                            taskPRIO_SDCard_Save,
                            &xTaskHandle_SDCard_Save);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }     
    
    /*(3) ����SDCard �ر��ļ����� */      
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

    /*(4) ����GPS ���ݽ������� */      
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
    
    /*(5) ����UWB �����Ӧ������ */      
    txResult = xTaskCreate(vTask_UWB_EventHandler,
                            "UWBResp",
                            configMINIMAL_STACK_SIZE+120,
                            NULL,
                            taskPRIO_UWB_EventHandler,
                            &xTaskHandle_UWB_EventHandler);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }     

    
    
    //(6) �����ɼ����� 10ms��ʱ�� 
    xTimerHandle_CollectData = xTimerCreate("10ms",
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


