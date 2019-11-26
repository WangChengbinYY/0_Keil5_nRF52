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


//ȫ�ֱ���_�������_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;           //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0

//ȫ�ֱ���_�������_ʱ�� 
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint8_t     G_GPSWeekSecond_IsValid;
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
uint8_t     G_MicroSecond_1PPS_IsUsed;          //GPS ��1pps�Ƿ���ʹ�ù���

//ȫ�ֱ���_�����洢_����ʱ��(��GPS�����룬�޴�0��ʼ)
uint8_t	    G_Time_Seconds[7]; 
uint8_t	    G_Time_Seconds_IsReady;            //�κ�д�뻺��������ǰ���ж�һ�£�ʱ���Ƿ��Ѹ��£�����£��ȴ�ʱ������

//ȫ�ֱ���_�����洢_��ǿ����������  IMUA(MPU9255) or IMUB(MPU9255)
uint8_t	    G_MAG_Coeffi[5]; 

//ȫ�ֱ���_���ݴ洢_IMUB MPU9255
uint8_t     G_IMUB_MPU92[4];    //ͬʱ����ʱ����BΪ�������䷢��UWB��࣬��ѹ���������Ĳɼ�

//ȫ�ֱ���_���ݴ洢_IMUB�����Ƚ�����ADIS
uint8_t	    G_IMUB_ADIS[4];     //�洢��ʶͷ��msʱ�䣬����ֱ�Ӵ洢

//ȫ�ֱ���_���ݴ洢_IMUA�����Ƚ�����MPU9250
uint8_t     G_IMUA_MPU92[4];    //��ΪҪ����UWB��࣬���Բ��洢ѹ������������
uint8_t     G_IMUA_MPU92_Magnetic[4];       //���洢IMUA�Ĵ�ǿ������


//ȫ�ֱ���_���ݴ洢_UWB�������
uint8_t     G_UWB_Data[6];


//ȫ�ֱ���_���ݴ洢_GPS��λ����
uint8_t     G_GPSData[14];

//ȫ�ֱ���_���ݴ洢_GPS���ڽ��ջ���
uint8_t     G_Uart_Buffer1[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer2[configBufferUART_RX_SIZE];
uint8_t     G_Uart_Buffer_Number;

// ȫ�ֱ���_���ݴ洢_SDCard�洢����  
uint8_t     G_SDCard_CirBuffer[configSDCard_BufferSize];
uint8_t*    G_SDCard_CB_pSave = NULL;
uint8_t*    G_SDCard_CB_pLoad = NULL;
uint16_t    G_SDCard_CB_Counter; 

/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */
#define taskPRIO_SDCard_Close                0          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 1�� */
#define taskPRIO_SDCard_Save                 1          //SDCard�洢���� 

/* 2�� */
#define taskPRIO_GPS_RxData                  2          //����GPS���ݲ�����	

/* 3�� */
//#define taskPRIO_UWB_Start                   3          //UWB������

/* 4�� */
#define taskPRIO_CollectData_IMUA            4          //�ɼ� IMUA(U4)�����ݣ��洢��ѭ�� (����ѹ������������)

/* 5�� */
#define taskPRIO_CollectData_IMUB            5          //�ɼ� IMUB(U5)������ ��ADIS��MTI�Ĳɼ�Ϊ�� (����ѹ������������)		         

/* 6�� */
#define taskPRIO_UWB_EventHandler            6			//UWB��Ӧ�� ������ȼ� 

/* 7�� */
#define taskPRIO_TaskStart                   7          //�������� 

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*����GPS��������       ��� */
//TaskHandle_t    xTaskHandle_UWB_Start           = NULL;
TaskHandle_t    xTaskHandle_CollectData_IMUB    = NULL;         //IMUB(U5) ��IMU���ݲɼ�����
TaskHandle_t    xTaskHandle_CollectData_IMUA    = NULL;         //IMUA(U4) ���ɼ�ѭ������
TaskHandle_t    xTaskHandle_UWB_EventHandler    = NULL; 
TaskHandle_t    xTaskHandle_TaskStart           = NULL;

///* ȫ�ֱ���_������_SDCard���桪��ͬʱҲ��SPI��ʹ�û�����  */
SemaphoreHandle_t   xMutex_SDCard_CirBuffer     = NULL;
//��ֵ�ź��������� GPS���ݽ����Ļ���ʹ��
SemaphoreHandle_t   xSemaphore_GPSBuffer        = NULL;

/**
 * ȫ�ֱ�����ʼ������   ������  
*/
void vINIT_Variable(void)
{
//ȫ�ֱ���_�������_SDCard�ļ�������ʶ                                                         
    G_SDCard_FileIsOpen = 0;           //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0

//ȫ�ֱ���_�������_ʱ�� 
    G_GPSWeekSecond = 0;                   //GPS����������
    G_GPSWeekSecond_IsValid = 0;
    G_MicroSecond = 0;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
    G_MicroSecond_1PPS_IsUsed = 0; 
//ȫ�ֱ���_�����洢_����ʱ��(��GPS�����룬�޴�0��ʼ)
    memset(G_Time_Seconds,0,7);
    G_Time_Seconds[0] = 0xC0;
    G_Time_Seconds[1] = 0xC0;
    G_Time_Seconds_IsReady = 0;            //�κ�д�뻺��������ǰ���ж�һ�£�ʱ���Ƿ��Ѹ��£�����£��ȴ�ʱ������

//ȫ�ֱ���_�����洢_��ǿ����������_IMUA(MPU9255)
    memset(G_MAG_Coeffi,0,5);
    G_MAG_Coeffi[0] = 0xC1;
    G_MAG_Coeffi[1] = 0xC1;
    
//ȫ�ֱ���_���ݴ洢_IMUB�����Ƚ�����ADIS
    memset(G_IMUB_ADIS,0,4);
    G_IMUB_ADIS[0] = 0xA1;
    G_IMUB_ADIS[1] = 0xBB;

//ȫ�ֱ���_���ݴ洢_IMUA����MPU9250  ���洢ѹ��������������
    memset(G_IMUA_MPU92,0,4);
    G_IMUA_MPU92[0] = 0xA3;
    G_IMUA_MPU92[1] = 0xAB;
    
//ȫ�ֱ���_���ݴ洢_IMUB����MPU9250   �洢ѹ��������������
    memset(G_IMUB_MPU92,0,4);
    G_IMUB_MPU92[0] = 0xA5;
    G_IMUB_MPU92[1] = 0xAA;
    
//ȫ�ֱ���_���ݴ洢_IMUA����MPU9250  ���洢���еĴ�ǿ������
    memset(G_IMUA_MPU92_Magnetic,0,4);
    G_IMUA_MPU92_Magnetic[0] = 0xA4;
    G_IMUA_MPU92_Magnetic[1] = 0xAA;

//ȫ�ֱ���_���ݴ洢_UWB�������
    memset(G_UWB_Data,0,6);
    G_UWB_Data[0] = 0xA2;
    G_UWB_Data[1] = 0xA2;

//ȫ�ֱ���_���ݴ洢_GPS�������
    memset(G_GPSData,0,13);
    G_GPSData[0] = 0xB1;
    G_GPSData[1] = 0xB1;

//ȫ�ֱ���_���ݴ洢_GPS���ڽ��ջ���
    memset(G_Uart_Buffer1,0,configBufferUART_RX_SIZE);    
    memset(G_Uart_Buffer2,0,configBufferUART_RX_SIZE); 
    G_Uart_Buffer_Number = 0;

// ȫ�ֱ���_���ݴ洢_SDCard�洢����  
    memset(G_SDCard_CirBuffer,0,sizeof(G_SDCard_CirBuffer));
    G_SDCard_CB_pSave = G_SDCard_CirBuffer;
    G_SDCard_CB_pLoad = G_SDCard_CirBuffer;
    G_SDCard_CB_Counter = 0;       
}


//���λ����� �洢����    
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


/*=========================================== ����ʵ�� ==============================================*/
/*------------------------------------------------------------
 *  UWB������Ӧ���� ����
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
 *  UWB�ж���Ӧ���� ����
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
//����˵��ж���Ӧ��
                error_code_UWB = ucSS_INIT_Handler(&tDistance,&tNumber);
                if(error_code_UWB == 0)
                {
                    memcpy(G_UWB_Data+2,&G_MicroSecond,sizeof(G_MicroSecond));  
                    memcpy(G_UWB_Data+4,&tDistance,sizeof(tDistance));
                    
                    //�ȴ洢ʱ������
                    if(G_Time_Seconds_IsReady == 1)
                    {
                        //��ֹ�������   
                        if((sizeof(G_Time_Seconds)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                        {
                            ucCircleBuffer_SaveData(G_Time_Seconds,sizeof(G_Time_Seconds));  
                            G_Time_Seconds_IsReady = 0;
                        }else
                        {
                            //����
                            NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_Time_Seconds!");
                            NRF_LOG_FLUSH();
                        } 
                    }
                    
                    //UWB ���ݴ��뻺����                   
                    if((sizeof(G_UWB_Data)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                    {
                        ucCircleBuffer_SaveData(G_UWB_Data,sizeof(G_UWB_Data));  
                    }else
                    {
                        //����
                        NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_UWB_Data!");
                        NRF_LOG_FLUSH();
                    }                    
                  
                }else
                {
                    //UWB���մ���
                    NRF_LOG_INFO("  UWB Receive is Wrong!");
                    NRF_LOG_FLUSH();                    
                }
#else
//���ն˵��ж���Ӧ����
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
		
        //�ȴ�һ�£��Է�SDCard��̨û�д洢���	
        nrf_delay_ms(300); 
        
        erro_code = ucSDCard_CloseFile();
            
        if(erro_code == 0)
        {
            //�ļ��ر���ȷ ��˸3s �ص�
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
            //�ļ��رմ��� ��ѭ�� ��ͣ��˸
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

//��ջ��� ��ʾ������
void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    NRF_LOG_INFO("OverFlow OverFlow!!!");
    NRF_LOG_FLUSH();     
}


/*------------------------------------------------------------
 *SDCard�洢���� ����
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
                    //�洢����ֹͣ�ɼ���������
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
        
            //���� GGA ��� ��ȡ��λ��Ϣ           
            if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
            {                
                //GGA��� �����ɹ�
                struct minmea_sentence_gga mGGA;
                if(minmea_parse_gga(&mGGA,(char*)G_Uart_Buffer2))
                {
                    //��λ������Ч
                    if(mGGA.fix_quality ==1 )
                    {    
                        //��ȡ��Ч������ʱ��
                        G_GPSWeekSecond = mGGA.time.hours*3600+mGGA.time.minutes*60+mGGA.time.seconds;
                        G_GPSWeekSecond_IsValid = 1;                        
                        
                        //��¼��λ����
                        memcpy(G_GPSData+2,&mGGA.latitude.value,sizeof(mGGA.latitude.value));
                        memcpy(G_GPSData+6,&mGGA.longitude.value,sizeof(mGGA.longitude.value));
                        int16_t mHigh;
                        uint16_t mHDop;
                        mHigh = mGGA.altitude.value;
                        mHDop = mGGA.hdop.value;
                        memcpy(G_GPSData+10,&mHigh,sizeof(mHigh));
                        memcpy(G_GPSData+12,&mHDop,sizeof(mHDop));
                        
                        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 50 ) == pdTRUE)
                        {      
                            //�ȴ洢ʱ������
                            if(G_Time_Seconds_IsReady == 1)
                            {
                                //��ֹ�������   
                                if((sizeof(G_Time_Seconds)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                                {
                                    ucCircleBuffer_SaveData(G_Time_Seconds,sizeof(G_Time_Seconds));  
                                    G_Time_Seconds_IsReady = 0;
                                }else
                                {
                                    //����
                                    NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_Time_Seconds!");
                                    NRF_LOG_FLUSH();
                                } 
                            }
                        
                        
                            if((sizeof(G_GPSData)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                            {
                                ucCircleBuffer_SaveData(G_GPSData,sizeof(G_GPSData));                                     
                            }else
                            {
                                //����
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
 *  IMUA(U4) �ɼ�MPU9255  ��ȡȫ�����ݣ������洢Magnetic����
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUA(void *pvParameters)
{
    uint8_t error_IMUA = 0;
    uint8_t tIMUA_MPU92_Data[22] = {0};
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY);               
        
        //�ȴ� ����SPI���������ͷ�
        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 4 ) == pdTRUE)
        {   
            //�ɼ�IMUA MPU92���� ���ܵ����ɼ���ǿ�ƣ���ˣ�����������ȫ���ɼ�
            nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
            nrf_delay_us(2);
            error_IMUA = Leo_MPU9255_Read_ALLData(tIMUA_MPU92_Data);     
            nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);                    
            
            //����ɼ���ȷ ����뻺�� 
            if(error_IMUA == 0)
            {   
                //�ȴ洢ʱ������
                if(G_Time_Seconds_IsReady == 1)
                {  
                    //��ֹ�������   
                    if((sizeof(G_Time_Seconds)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                    {   
                        ucCircleBuffer_SaveData(G_Time_Seconds,sizeof(G_Time_Seconds));  
                        G_Time_Seconds_IsReady = 0;
                    }else
                    {
                        //����
                        NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_Time_Seconds!");
                        NRF_LOG_FLUSH();
                    } 
                }
                
                //�洢��ǿ������           
                if((26+G_SDCard_CB_Counter) <= configSDCard_BufferSize)   //�жϻ����Ƿ����
                {
                    //�洢  ��ǿ������ ͷ��ʶ��ʱ��
                    ucCircleBuffer_SaveData(G_IMUA_MPU92_Magnetic,sizeof(G_IMUA_MPU92_Magnetic));    
                    ucCircleBuffer_SaveData(tIMUA_MPU92_Data+15,6);                    
                }else
                {
                    //����
                    NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUA_Data!");
                    NRF_LOG_FLUSH();
                }                 
                
            }else
            {
                //IMUB ���ݲɼ�����
                NRF_LOG_INFO("  IMU_A CellectData is Wrong!");
                NRF_LOG_FLUSH();                   
            }   
           
            //�ͷ���Դ
            xSemaphoreGive( xMutex_SDCard_CirBuffer );     
        }else
        {
            NRF_LOG_INFO("  xMutex_SDCDBuffer IMUA TimeOUT");
            NRF_LOG_FLUSH();
        }  
        
	}
}



/*------------------------------------------------------------
 *  IMUB(U5) MPU92 Firs���� ���ɼ�����
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUB(void *pvParameters)
{
    uint8_t erro_IMUB = 0;
    uint8_t tIMUB_MPU_Data[22] = {0};
	nrf_saadc_value_t tSAResult[4] = {0}; 
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 
   
        //�ȴ� ����SPI���������ͷ�
        if(xSemaphoreTake( xMutex_SDCard_CirBuffer, ( TickType_t ) 4 ) == pdTRUE)
        { 
            //�ɼ�IMUB MPU92����
            erro_IMUB = Leo_MPU9255_Read_ALLData(tIMUB_MPU_Data);
            
            //�ɼ�ѹ�������� AD ����
            nrfx_saadc_sample_convert(0,tSAResult);
            nrfx_saadc_sample_convert(1,tSAResult+1);        
            nrfx_saadc_sample_convert(2,tSAResult+2);  
            nrfx_saadc_sample_convert(3,tSAResult+3);  
            
            //����ɼ���ȷ ����뻺�� 
            if(erro_IMUB == 0)
            {
                //�ȴ洢ʱ������
                if(G_Time_Seconds_IsReady == 1)
                {
                    //��ֹ�������   
                    if((sizeof(G_Time_Seconds)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
                    {
                        ucCircleBuffer_SaveData(G_Time_Seconds,sizeof(G_Time_Seconds));  
                        G_Time_Seconds_IsReady = 0;
                    }else
                    {
                        //����
                        NRF_LOG_INFO("  SDCard Buffer is OverFlow_G_Time_Seconds!");
                        NRF_LOG_FLUSH();
                    } 
                }
                
                //�ٴ��� IMUB���ݰ� 28���ֽ�                
                if((28+G_SDCard_CB_Counter) <= configSDCard_BufferSize)   //�жϻ����Ƿ����
                {
                    //���� ͷ��ʶ��ʱ��
                    ucCircleBuffer_SaveData(G_IMUB_MPU92,sizeof(G_IMUB_MPU92));
                    //���� IMUB����                    
                    ucCircleBuffer_SaveData(tIMUB_MPU_Data,sizeof(tIMUB_MPU_Data));                      
                    //���� ѹ������                    
                    ucCircleBuffer_SaveData((uint8_t *)tSAResult,sizeof(tSAResult));    
                }else
                {
                    //����
                    NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUB_Data!");
                    NRF_LOG_FLUSH();
                }                 
                
            }else
            {
                //IMUB ���ݲɼ�����
                NRF_LOG_INFO("  IMU_B CellectData is Wrong!");
                NRF_LOG_FLUSH();                   
            }   
            
#if configUWB_INIT             
            //UWB�������
            if(G_SDCard_CB_Counter < configSDCard_SaveSize)
            {
                vSS_INIT_Start();   
            }                
#endif 
            
            //�ͷ���Դ
            xSemaphoreGive( xMutex_SDCard_CirBuffer );   
            
            //�������������ˣ�֪ͨSDCard���д洢
            if(G_SDCard_CB_Counter >= configSDCard_SaveSize)
            {               
                //֪ͨ SDCard�洢
                BaseType_t xReturn = pdPASS;
                xReturn = xTaskNotify(xTaskHandle_SDCard_Save,0,eSetValueWithoutOverwrite);  
                if(xReturn == pdFAIL)
                {
                    //SDCard ֪ͨ�洢ʧ��
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
 * �������� */
static void vTask_TaskStart(void *pvParameters)
{
    uint8_t erro_code = 0;
    BaseType_t txResult;
    
//1.��ʼ������        
    //1.��ʼ������    
    /*(1) ����SDCard�洢���� */    
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
    
   /*(2) ����SDCard �ر��ļ����� */      
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

    /*(3) ����GPS ���ݽ������� */      
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
    
    /*(4) ����UWB �����Ӧ������ */      
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
    
    //(5) ����IMUA ���ɼ�����  
//    txResult = xTaskCreate(vTask_CollectData_IMUA,
//                           "CollData_IMUA",
//                           configMINIMAL_STACK_SIZE+256,
//                           NULL,
//                           taskPRIO_CollectData_IMUA,
//                           &xTaskHandle_CollectData_IMUA);
//    if(txResult != pdPASS)
//    {
//       erro_code = 1;
//    }     
    
    
    //(6) ����IMUB(ADIS) �ɼ�����  
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
 
    //��4�� ��ʼ�� IMUB(MPU92)only����First���� 
    erro_code |= ucIMU_INIT_IMUBOnly_MPU(); 
    NRF_LOG_INFO(("||Initialize||-->IMUA(MPU)_IMUB(ADIS)->error  0x%x"),erro_code);  
    
    //��5�� ��ʼ�� SAADC ѹ��������
    erro_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),erro_code); 
    NRF_LOG_FLUSH(); 
    
    //��6����ʼ�� UWB
#if configUWB_INIT
    erro_code |= ucSS_INIT_Initial();
#else
    erro_code |= ucSS_RESP_Initial();
#endif
    NRF_LOG_INFO(("||Initialize||-->UWB------------->error  0x%x"),erro_code);     
    NRF_LOG_FLUSH(); 
    
    //��7����ʼ����ʱ�������� 
    erro_code |= ucTimerInitial_3();        //1ms ��ʱ 
    erro_code |= ucTimerStart_3();          
//    erro_code |= ucTimerInitial_4();        //FreeRTOS ���������
//    erro_code |= ucTimerStart_4();
     
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
    
    //��8����ʼ���жϲ�����
    erro_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    erro_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    erro_code |= ucINTInital_UWB();
    //erro_code |= ucINTInital_IMUA();
    erro_code |= ucINTInital_IMUB();
    ucINTStart_SDCard();
    ucINTStart_PPS();
    ucINTStart_UWB();
    //ucINTStart_IMUA();
    ucINTStart_IMUB();
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
    
//TEST ���ڲ�����������ռ��Ƿ񲻹���
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
    xMutex_SDCard_CirBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCard_CirBuffer == NULL)
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


