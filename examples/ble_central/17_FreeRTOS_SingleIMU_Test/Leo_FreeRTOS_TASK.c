/*
*********************************************************************************************************
*
*    ģ������ : FreeRTOS������ʵ��
*    �ļ����� : Leo_FreeRTOS_TASK
*    ��    �� : V1.0
*    ˵    �� : ��Ŀ����������Ľ���
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
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
uint8_t	    G_CollectData[512];                 //SDCardҪ�������ݵĻ���
uint16_t    G_CollectData_Counter;  
// ȫ�ֱ���_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0


extern uint8_t G_UART_Buffer2[512];
extern uint8_t G_UART_Buffer2_Counter;




/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */
#define taskPRIO_SDCard_Close                0          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 1�� */
#define taskPRIO_GPS_RxData                  1          //����GPS���ݲ������������ɹ���֪ͨ�洢 

/* 2�� */


/* 3�� */
//10ms ѭ����ʱ�� �ɼ�����                   3

/* 4�� */
#define taskPRIO_SDCard_Save                 4          //SDCard�洢����

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*����GPS��������       ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */

        
TimerHandle_t     xTimerHandle_CollectData        = NULL;         /*5ms�����Ĳɼ�����    ��� */


/**
 * ȫ�ֱ���_������_SDCard����  
*/
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
    memset(G_CollectData,0,512);
    G_CollectData_Counter = 0;    
    //ȫ�ֱ���_SDCard�ļ�������ʶ 
    G_SDCard_FileIsOpen = 0;                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0 
}





/*=========================================== ����ʵ�� ==============================================*/

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
        
        //(3)�ر��ļ��洢
        erro_code = ucSDCard_CloseFile();
        
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
    }
}

/*------------------------------------------------------------
 *SDCard�洢���� ����
 *------------------------------------------------------------*/
static void vTask_SDCard_Save(void *pvParameters)
{
    uint8_t tData[512] = {0};
    uint16_t tData_Count = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        /*(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/
        
        /*(2) ��ȡ��Դ     */
        if((G_CollectData_Counter > 256) && (G_SDCard_FileIsOpen == 1) )
        {
            uint16_t    tTestStart = 0;   
            tTestStart = G_MicroSecond;
            
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //�ͷ���Դ
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
        
        //��ȡ����
        memcpy(mData,G_UART_Buffer2,G_UART_Buffer2_Counter);
//        NRF_LOG_INFO("%s",mData);
        
        //���н���
        enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mData);  

        //���� RMC���
        if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
        {
            //TEST
//            NRF_LOG_INFO("Find the RMC!");
//            NRF_LOG_FLUSH();
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
        
        //���� GGA ���            
        if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
        {
            //TEST
//            NRF_LOG_INFO("Find the GGA!");
//            NRF_LOG_FLUSH();
            
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
                    
                    //TEST
//                    NRF_LOG_INFO("The Lat is %d, The Lon is %d!",mGGA.latitude.value,mGGA.longitude.value);
//                    NRF_LOG_FLUSH();
                    
                    if(G_SDCard_FileIsOpen == 1)
                    {
                        //��ȡ ������ ��������
                        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
                        {
                            memcpy(G_CollectData+G_CollectData_Counter,G_GPSData,sizeof(G_GPSData));
                            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSData); 
                            
                            //�ͷ���Դ
                            xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                            //֪ͨ SDCard�洢����
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
 *�ⲿ���������ݲɼ�����Ҫ������
 *  MPU9255A��MPU9255B��ѹ��������
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
        
        //ת��
//        for(i=0;i<4;i++)
//        {
//            sVoltResult[i] = tSAResult[i]*3.6/1024.0;
//            tTest[i] = (nrf_saadc_value_t)(sVoltResult[i]*100);            
//        }
//        NRF_LOG_INFO("AD error:%d,Point6:%d;Point7:%d;Point5:%d;Point2:%d;",error_code,tTest[0],tTest[1],tTest[2],tTest[3]);
//        NRF_LOG_FLUSH(); 
        
//3. �ɼ� UWB��� ����       
        error_code_UWB = ucSS_INIT_RUN(&tDistance,&tNumber);
//        NRF_LOG_INFO("The UWB  %d!",error_code_UWB);
//        NRF_LOG_FLUSH(); 
        if(error_code_UWB == 0)
        {
            //��¼ʱ������
            memcpy(G_UWBData+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
            memcpy(G_UWBData+6,&G_MicroSecond,sizeof(G_MicroSecond));
            //��¼���� �Ĵ������
            memcpy(G_UWBData+8,&tNumber,sizeof(tNumber));
            //��¼���� �ľ�������
            memcpy(G_UWBData+9,&tDistance,sizeof(tDistance));
//            NRF_LOG_INFO("The %d of Distance is %d mm!",tNumber,tDistance);
//            NRF_LOG_FLUSH(); 
        }

//4. �ɼ����ݽ��д洢        
        /*���ɼ�����,����洢 �ȴ�2ms�������û���ͷţ�������˴δ洢*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
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
            //UWB������ݴ��뻺��
            if(error_code_UWB == 0)
            {
                memcpy(G_CollectData+G_CollectData_Counter,G_UWBData,sizeof(G_UWBData));
                G_CollectData_Counter = G_CollectData_Counter + sizeof(G_UWBData);                
            }         
            
            //�ͷ���Դ
            xSemaphoreGive( xMutex_SDCDBuffer ); 
            
            //֪ͨ SDCard�洢����
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

    
    /*(2) ����SDCard�洢���� */    
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
    
    
    //(4) �����ɼ����� 10ms��ʱ�� 
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




