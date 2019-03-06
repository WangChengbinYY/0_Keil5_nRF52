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



/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */
#define taskPRIO_SDCard_Close                0          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 1�� */
#define taskPRIO_GPS_RxData                  1          //����GPS���ݲ������������ɹ���֪ͨ�洢 

/* 2�� */
#define taskPRIO_SDCard_Save                 2          //SDCard�洢����

/* 3�� */
#define taskPRIO_CollectData                 3          //�ɼ�IMU���ݡ�ѹ�����ݣ��ɹ���֪ͨ�洢

/* 4�� */
#define taskPRIO_UWB_RxData                  4          //�ɼ�UWB������ݣ��ɹ���֪ͨ�洢


/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*5ms�����Ĳɼ�����    ��� */



/**
 * ȫ�ֱ���_������_SDCard����  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;

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
            nrfx_gpiote_out_clear(configGPIO_LED_R);
            
            NRF_LOG_INFO("File is Closed!!!");
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
        if(G_CollectData_Counter > 0)
        {
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
//                NRF_LOG_INFO("%d",tData_Count);
                //�洢 �ļ�����򿪲�������洢
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
 *�ⲿ���������ݲɼ�����Ҫ������
 *  MPU9255A��MPU9255B��ѹ��������
 *------------------------------------------------------------*/
static void vTask_CollectData(void *pvParameters)
{
    while(1)
    {
        /*(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/      
        
        //(2)��¼��������
        memcpy(G_GPSWeekSecond_Data+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));
        memcpy(G_CollectData+G_CollectData_Counter,G_GPSWeekSecond_Data,sizeof(G_GPSWeekSecond_Data));
        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSWeekSecond_Data);        
        
        //(3)�ɼ�IMU_A ������  
        memcpy(G_IMU_Data_A+2,&G_MicroSecond,sizeof(G_MicroSecond));
        nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS);    
        Leo_MPU9255_Read_ACC(G_IMU_Data_A+5);
        Leo_MPU9255_Read_Gyro(G_IMU_Data_A+11);
        Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+17);  
        nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);   
        
 
        
        /*(4) �ɼ�ѹ�������� ������  δ���*/      
        
        
        /*(5) ���ɼ�����,����洢 
         *    �ȴ�4ms�������û���ͷţ�������˴δ洢*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 4 ) == pdTRUE)
        {
            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data_A,sizeof(G_IMU_Data_A));
            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data_A);
       
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

    }
}

/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    
    /*(1) �������Ľ���_SDCard���� */
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
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

    
    //(4) �����ɼ����� 5ms   ����д�� ʱ������
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




