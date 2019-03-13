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
#include "Leo_SDCard.h"


/*=========================================== �������ȼ��趨 ============================================*/

/*------------------------- 0�� -------------------------*/
/* �������� */
#define taskPRIO_LOG                         0          /* LOG��� */   
#define taskPRIO_LED                         0          /* LEDGreen��˸��(�����е�һ���յ�GPS��ȷʱ��)   */
#define taskPRIO_INIT                        0          /* ϵͳ��ʼ����
                                                         *   ���ڿ�ʼִ��һ�Σ���ʼ��ʧ���������ѭ����
                                                         *   �ɹ���ɾ������                             */
/*------------------------- 1�� -------------------------*/
           //����GPS����
#define taskPRIO_GPS_RxData                  1           //SDCard�洢����
#define taskPRIO_DataSave_Start              1
#define taskPRIO_DataSave_End                1

/*------------------------- 2�� -------------------------*/
#define taskPRIO_MPU9255_RxData              2           //����IMU����
#define taskPRIO_FootPres_RxData             2
#define taskPRIO_UWB_RxData                  2          //UWB���

/*------------------------- 3�� -------------------------*/

#define taskPRIO_SDCard_SaveData             3   

/*------------------------- 4��(���) -------------------------*/
// FootBLE ʹ�õ�Ȩ�� ѹ������������
         
/*-----------------------------------------------------------------------------------------------------*/



/*======================================= ȫ�ֱ������� new==================================================*/

/*���ݴ洢���*/
uint8_t	    G_MPU9255_MAG_ASAXYZ[6];            //AK8963��ǿ�ƶ����������������������� 2~4
uint8_t	    G_MPU9255_MAG_ASAXYZ_IsValid;       /*��ǰ�����Ƿ���Ч 1��Ч 0��Ч*/

uint8_t     G_WRONG_Record[10];                 //�ɼ��������ݳ���ļ�¼

uint8_t	    G_MPU9255_Data[28];                 //MPU9255��������ŵ�����
uint8_t	    G_MPU9255_Counter;                  //MPU9255�жϴ����ļ�����

uint8_t     G_GPS_Data[33];                     //GPS��������

uint8_t     G_FOOTPressure_Data[25];            //�㲿ѹ������������
uint8_t	    G_FOOTPressure_Counter;             //�㲿ѹ�����������ݵļ�����

uint8_t     G_UWBDistance_Data[12];             //UWB��ഫ��������
uint8_t	    G_UWBDistance_Counter;              //UWB��ഫ�������ݵļ�����

uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ��
                                                        //�� �ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
uint8_t	    G_SDCDBuffer1[1024];               //˫����buffer
uint16_t	G_SDCDBuffer1_NUM;	
uint8_t	    G_SDCDBuffer2[1024];
uint16_t	G_SDCDBuffer2_NUM;	

/*ϵͳ�������*/
uint8_t     G_Ctrl_DataSave;                    /* ���ݴ洢���Ʊ�־λ 1�洢��0���洢 */



/* ���������  */
TaskHandle_t    xTaskHandle_TaskINIT            = NULL;         /*ϵͳ��ʼ������       ��� */
TaskHandle_t    xTaskHandle_MPU9255_RxData      = NULL;         /*MPU9255���ݲɼ�����   ���*/
TaskHandle_t    xTaskHandle_GPS_RxData          = NULL;         /*GPS���ݲɼ�����       ���*/
TaskHandle_t    xTaskHandle_UWB_GetData         = NULL;         /*UWB�������           ��� */
TaskHandle_t    xTaskHandle_FootPres_GetData    = NULL;         /*UWB�������           ��� */

TaskHandle_t    xTaskHandle_DataSave_Start      = NULL;         /*��ʼ�洢����           ��� */
TaskHandle_t    xTaskHandle_DataSave_End        = NULL;         /*ֹͣ�洢����           ��� */

/* ʱ���������  */
TimerHandle_t   xTimers_StartINT                = NULL;         /*�ж�����ʱ������ ��BLE��ʼ������Ԥ��ʱ�� ��� */
TimerHandle_t   xTimers_StarSDCard              = NULL;         /*SDCard ѭ���洢ʱ������  ���*/


/* ��Դ��� ������ */
SemaphoreHandle_t   xMutex_SDCDBuffer_1         = NULL;



/**
 * ȫ�ֱ�����ʼ��   ����
*/
void vInitial_Variable(void)
{
    //AK8963��ǿ�ƶ������������������� 2~4
    memset(G_MPU9255_MAG_ASAXYZ,0,6);
	G_MPU9255_MAG_ASAXYZ[0] = 0xC1;
	G_MPU9255_MAG_ASAXYZ[1] = 0xC2;
    G_MPU9255_MAG_ASAXYZ[5] = 0xFF;   
    G_MPU9255_MAG_ASAXYZ_IsValid = 0;
    
    //�ɼ��������ݳ���ļ�¼ 
    memset(G_WRONG_Record,0,10);
    G_WRONG_Record[0] = 0xC3;
	G_WRONG_Record[1] = 0xC4;
    G_WRONG_Record[9] = 0xFF;
    
    //MPU9255��������ŵ�����
    memset(G_MPU9255_Data,0,28);
    G_MPU9255_Data[0] = 0xA1;
	G_MPU9255_Data[1] = 0xA2;
    G_MPU9255_Data[27] = 0xFF;
    G_MPU9255_Counter = 0;                              //MPU9255�жϴ����ļ�����
    
    //GPS��������
    memset(G_GPS_Data,0,33);
    G_GPS_Data[0] = 0xA3;
    G_GPS_Data[1] = 0xA4;
    G_GPS_Data[32] = 0xFF;

    //�㲿ѹ������������
    memset(G_FOOTPressure_Data,0,25);
    G_FOOTPressure_Data[0] = 0xA5;
	G_FOOTPressure_Data[1] = 0xA6;
    G_FOOTPressure_Data[24] = 0xFF;
    G_FOOTPressure_Counter = 0;
    
    //UWB��ഫ��������
    memset(G_UWBDistance_Data,0,12);
    G_UWBDistance_Data[0] = 0xA7;
	G_UWBDistance_Data[1] = 0xA8;
    G_UWBDistance_Data[11] = 0xFF;
    G_UWBDistance_Counter = 0;
    
    //ʱ�����
    G_GPSWeekSecond = 0;
    G_MicroSecond = 0;    

    //SDCard�洢buffer
    memset(G_SDCDBuffer1,0,1024);
    memset(G_SDCDBuffer2,0,1024);    
    G_SDCDBuffer1_NUM = 0;
    G_SDCDBuffer2_NUM =0;

    /*ϵͳ�������*/
    G_Ctrl_DataSave = 0;
 
}






/*======================================= ����ʵ�� new==================================================*/
/**
 * ��ʼ���ݴ洢   ����
*/
static void vTask_DataSave_Start(void *pvParameters)
{
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
        G_Ctrl_DataSave = 1;       
        
    }
}

/**
 * �������ݴ洢   ����
*/
static void vTask_DataSave_End(void *pvParameters)
{
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
        G_Ctrl_DataSave = 0;       
        vTaskDelay( (TickType_t)100 );
        ucSDCard_CloseFile();        
    }
}


/**
 * MPU9255���ݲɼ�����   ����
*/
static void vTask_MPU9255_RxData(void *pvParameters)
{
    
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
         /**
         *(2) �ɼ�MPU9255���� */
        memcpy(G_MPU9255_Data+2,&G_GPSWeekSecond,4);
        memcpy(G_MPU9255_Data+6,&G_MicroSecond,2);        
        memcpy(G_MPU9255_Data+8,&G_MPU9255_Counter,1); 
        
        /* �ɼ����������� */    
        Leo_MPU9255_Read_ACC();
        Leo_MPU9255_Read_Gyro();
        Leo_MPU9255_Read_Magnetic(); 
        
        /*���ǿ��Դ洢 */
        if(G_Ctrl_DataSave == 1)
        {
            /* ��ȡ��Դ�������   �ȴ�2ms һ������5ms*/
            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
            {   
                 /*  ��©�����п��ܻ������  ��� �洢����ʱ�Ļ����� */
                if(G_SDCDBuffer1_NUM + sizeof(G_MPU9255_Data) > 1024)
                {
                    NRF_LOG_INFO("MPU9255 Can't Save, It's Full !!!!!!!!!!!!!!!!!");
                    NRF_LOG_FLUSH();  
                }else
                {
                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_MPU9255_Data,sizeof(G_MPU9255_Data)); 
                    G_SDCDBuffer1_NUM += sizeof(G_MPU9255_Data);
                }               
                 
                /* �ͷ���Դ */
                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
            }else
            {
                /* δ��ȡ����Դ��ʹ��Ȩ�� */
                NRF_LOG_INFO("MPU9255 Use Buffer is Busy!!!!!!!!!!!!!!!!!");
                NRF_LOG_FLUSH();            
            }            
        }
        

        //test        
//        if((G_MPU9255_Counter % 255) == 0)
//        {
//            NRF_LOG_INFO("MPU9255 Data is OK!___5s");
//        }        
    }
}


/**
 * GPS���ݲɼ�����   ����
*/
static void vTask_GPS_RxData(void *pvParameters)
{
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ
         */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/   
    }    
    
}

/**
 * UWB���ݲɼ�����   ����
*/    
static void vTask_UWB_RxData(void *pvParameters)
{
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ
         */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/   
    } 
}



/**
 * ѹ�����������ݲɼ�����   ����
*/
static void vTask_FootPres_RxData(void *pvParameters)
{
    while(1)
    {
        /**
         *(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
         /**
         *(2) �ɼ�MPU9255���� */
        
        /*���ǿ��Դ洢 */
        if(G_Ctrl_DataSave == 1)
        {
            /* ��ȡ��Դ�������   �ȴ�2ms һ������5ms*/
            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
            {   
                 /*  ��©�����п��ܻ������  ��� �洢����ʱ�Ļ����� */
                if(G_SDCDBuffer1_NUM + sizeof(G_FOOTPressure_Data) > 1024)
                {
                    NRF_LOG_INFO("G_FOOTPressure_Data Can't Save, It's Full !!!!!!!!!!!!!!!!!");
                    NRF_LOG_FLUSH();  
                }else
                {
                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_FOOTPressure_Data,sizeof(G_FOOTPressure_Data)); 
                    G_SDCDBuffer1_NUM += sizeof(G_FOOTPressure_Data);
                }               
                 
                /* �ͷ���Դ */
                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
            }else
            {
                /* δ��ȡ����Դ��ʹ��Ȩ�� */
                NRF_LOG_INFO("G_FOOTPressure_Data Use Buffer is Busy!!!!!!!!!!!!!!!!!");
                NRF_LOG_FLUSH();            
            }            
        }
            
    }
}


/**
 * �ж�����ʱ������   ����
*/
static void TimerFunction_StartINT( TimerHandle_t xTimer )
{
    uint8_t err_code = 0;
/*1. ������ʱ��*/
#if configTIMER3_ENABLE    
    /* TIMER3 ����������*/ 
    err_code |= ucTimerStart_3();
#endif 
    NRF_LOG_INFO(("||  Start  ||-->TIMER--------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();    


/*2. �����ⲿ�ж�*/
    err_code |= ucINTStart_SDCard();    /* SDCard�ж� ���� */    
    err_code |= ucINTStart_MPU9255();   /* MPU9255�ɼ��ж� ���� */
    err_code |= ucINTStart_PPS();       /* 1PPS�������ж� ���� */
    NRF_LOG_INFO(("||  Start  ||-->GPS Uart------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();
    
/*3. ���ݴ洢����*/    
    G_Ctrl_DataSave = 1;
    nrfx_gpiote_out_clear(configGPIO_LED_G);
}

/**
 * SDCardѭ���洢����   ����
*/
static void TimerFunction_SDCardSave( TimerHandle_t xTimer )
{
    uint8_t     erro_code = 0;
    if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
    {
        if(G_SDCDBuffer1_NUM > 0)
        {
            memcpy(G_SDCDBuffer2,G_SDCDBuffer1,G_SDCDBuffer1_NUM);
            G_SDCDBuffer2_NUM = G_SDCDBuffer1_NUM;
            G_SDCDBuffer1_NUM = 0;             
        }
        /* �ͷ���Դ */
        xSemaphoreGive( xMutex_SDCDBuffer_1 );          
                    
        if(G_Ctrl_DataSave == 1)
        {
            erro_code = ucSDCard_SaveData(G_SDCDBuffer2,G_SDCDBuffer2_NUM);  
            if(erro_code != 0)
            {
                NRF_LOG_INFO("SDCard DataSave is Wrong Wrong !!!!!!!!!!!!!");
                NRF_LOG_FLUSH(); 
            }
        }  
    }else
    {
        /* δ��ȡ����Դ��ʹ��Ȩ�� */
        NRF_LOG_INFO("TimerFunction_FOOT Use Buffer1 is Busy!!!!!!!!!!!!!!!!!");
        NRF_LOG_FLUSH();            
    } 
}



/**
 * ��ʼ������   ����  ִֻ��һ�� 
*/
static void vTask_StartINIT(void *pvParameters)
{
    uint8_t err_code = 0;
    BaseType_t txResult = pdPASS;

/**
 * 0. ȫ�ֱ�����ʼ��    
 */    
    vInitial_Variable();    
    
/**
 * 1. GPIO�ܽ��趨    
 */
    err_code |= nrfx_gpiote_init();
    
    /* (1) LED �ܽ� */
    nrfx_gpiote_out_config_t tconfigGPIO_OUT_LEDR =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT_LEDR);
    nrfx_gpiote_out_config_t tconfigGPIO_OUT_LEDG =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_G,&tconfigGPIO_OUT_LEDG);  
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();
    
    /* (2) INT�жϹܽų�ʼ�� */    
    err_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    err_code |= ucINTInital_MPU9255();   /* MPU9255�ɼ��жϹܽų�ʼ�� */
    err_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),err_code);   
    NRF_LOG_FLUSH();    
/*----------------------------------------------------------------------------*/

/**
 * 2. ��ʱ����ʼ��    
 */
    /* (1) 1ms ��ʱ�� ��ʼ�� ʹ�õ�TIMR3 */ 
#if configTIMER3_ENABLE    
    err_code |= ucTimerInitial_3();      /* TIMER3 ��������ʼ��*/ 
#endif 

    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),err_code);  
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/    


/**
 * 3. ��ʼ��MPU9255    MPU9255��ʼ���������SDCard֮ǰ����֪��Ϊɶ������
 */
    err_code |= ucMPU9255_INIT();
    NRF_LOG_INFO(("||Initialize||-->MPU9255-------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();   
    nrf_delay_ms(10);    
/*----------------------------------------------------------------------------*/ 

/**
 * 4. ��ʼ��SDCard ��������¼�ļ�    
 */
    err_code |= ucSDCard_INIT();    
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();    
/*----------------------------------------------------------------------------*/ 
    
    char tcTest[] = "This is just a test!";
    err_code |= ucSDCard_SaveData(tcTest,sizeof(tcTest)); 
    NRF_LOG_INFO(("||Initialize||-->SDCard Save Test--->error  0x%x"),err_code); 
    NRF_LOG_FLUSH(); 


/**
 * 6. ��ʼ�� GPS���ڽ���    
 */
    err_code |= ucGPS_INTUART();    
    NRF_LOG_INFO(("||Initialize||-->GPS Uart------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/ 

/**
 * 7. �������ʼ���Ľ������LED����ʾ
 *      ���������ʼ�������ȷ��
 *      ���������ʼ������ 
 */
    if(err_code != 0)
    {
        NRF_LOG_INFO("Warning Warning Warning......Initialization is Wrong!!!!!"); 
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        }        
    }else
    {
        //��ȷ�������
        nrfx_gpiote_out_clear(configGPIO_LED_R);
    }
/*----------------------------------------------------------------------------*/ 
    
/**
 * 8. ��������صĳ�ʼ�� 
 */    
    /*(1) �������Ľ��� */
    xMutex_SDCDBuffer_1    = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer_1 == NULL)
    {
        /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
        NRF_LOG_INFO("Warning Warning Warning......MutexCreate is Wrong!!!!!"); 
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        }        
    }
    
    /*(2) ����ʱ������Ľ��� �����жϵ� */
    xTimers_StartINT = xTimerCreate("StartINT",             /* ��ʱ������ */
                                  30000,                    /* ��ʱ������,��λʱ�ӽ��� */
                                  pdFALSE,                  /* ������ */
                                  (void *) 1,               /* ��ʱ��ID */
                                  TimerFunction_StartINT);  /* ��ʱ���ص����� */
     if(xTimers_StartINT == NULL)
     {
         /* The timer was not created. */
        NRF_LOG_INFO("||Warning||-->StartINT Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
     }                                 
                                  
                                  
    /*(3) ѭ��ʱ������Ľ��� SDCard�洢���� */
    xTimers_StarSDCard = xTimerCreate("SDCard",            /* ��ʱ������ */
                              10,                /* ��ʱ������,��λʱ�ӽ��� */
                              pdTRUE,          /* ������ */
                              (void *) 2,      /* ��ʱ��ID */
                              TimerFunction_SDCardSave); /* ��ʱ���ص����� */
     if(xTimers_StarSDCard == NULL)
     {
         /* The timer was not created. */
        NRF_LOG_INFO("||Warning||-->SDCard Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
     }                               
                                  
    //�ӳ�����ʱ������
    if((xTimerStart(xTimers_StartINT,0) != pdPASS)||(xTimerStart(xTimers_StarSDCard,0) != pdPASS))
     {
         /* The timer could not be set into the Active
         state. */
        NRF_LOG_INFO("||Warning||-->SDCard_Timer Start is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        }
     }                              
      
         
    /*(4) MPU9255������ */
    txResult = xTaskCreate(vTask_MPU9255_RxData,
                            "MPU9255",
                            configMINIMAL_STACK_SIZE+100,
                            NULL,
                            taskPRIO_MPU9255_RxData,
                            &xTaskHandle_MPU9255_RxData);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskMPU9255 Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }
    
    /*(5) GPS ������ */
    txResult = xTaskCreate(vTask_GPS_RxData,
                            "GPS",
                            configMINIMAL_STACK_SIZE+100,
                            NULL,
                            taskPRIO_GPS_RxData,
                            &xTaskHandle_GPS_RxData);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskGPS Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }    
    
    /*(6) UWB������ */
    txResult = xTaskCreate(vTask_UWB_RxData,
                            "UWB",
                            configMINIMAL_STACK_SIZE+100,
                            NULL,
                            taskPRIO_UWB_RxData,
                            &xTaskHandle_UWB_GetData);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskUWB Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }   
    
    /*(7) FootPres������ */
    txResult = xTaskCreate(vTask_FootPres_RxData,
                            "FootPres",
                            configMINIMAL_STACK_SIZE+100,
                            NULL,
                            taskPRIO_FootPres_RxData,
                            &xTaskHandle_FootPres_GetData);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskFootPres Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }    
    
    /*(8) ���� ֹͣ�洢�������� */  
    txResult = xTaskCreate(vTask_DataSave_Start,
                            "SaveStart",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            taskPRIO_DataSave_Start,
                            &xTaskHandle_DataSave_Start);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->SaveStart Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }  
    txResult = xTaskCreate(vTask_DataSave_End,
                            "SaveEnd",
                            configMINIMAL_STACK_SIZE+100,
                            NULL,
                            taskPRIO_DataSave_End,
                            &xTaskHandle_DataSave_End);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->SaveEnd Create is failed!!");
        NRF_LOG_FLUSH();
        //������ѭ����˸
        while(1)
        {
            nrf_delay_ms(250);
            nrf_gpio_pin_toggle(configGPIO_LED_R);            
        } 
    }  
/*----------------------------------------------------------------------------*/      
    
/**
 * 9. ɾ����������    
 */
    vTaskDelete(xTaskHandle_TaskINIT);
    NRF_LOG_INFO("||  Start  ||-->TaskINI is Delete!!");
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/    

}

   




/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    BaseType_t txResult = pdPASS;
    txResult = xTaskCreate(vTask_StartINIT,
                            "StartTask",
                            configMINIMAL_STACK_SIZE+200,
                            NULL,
                            taskPRIO_INIT,
                            &xTaskHandle_TaskINIT);
    if(txResult != pdPASS)
    {
        NRF_LOG_INFO("||Warning||-->TaskINI Create is failed!!");
        NRF_LOG_FLUSH();
        return 1;
    }else
    {
        return 0;
    }
}



























/*******************************************************************************************************/





/*
*********************************************************************************************************
*                                    �Լ������ �������
*********************************************************************************************************
*/



/* ��������� */

//TaskHandle_t    xTaskHandle_Test;

//void vTaskFunction_TaskTest(void * pvParameter)
//{
//    UNUSED_PARAMETER(pvParameter);
//    while (true)
//    {   
//       
//        
//        NRF_LOG_INFO("=================================================");
//        NRF_LOG_FLUSH();
//        /* Delay a task for a given number of ticks */
//        vTaskDelay(1000);

//        /* Tasks must be implemented to never return... */
//    }
//}


//void vTask_Create(void)
//{


//    if (pdPASS != xTaskCreate(vTaskFunction_TaskTest, "TTest", 256, NULL, 2, &xTaskHandle_Test))
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }  
//    NRF_LOG_INFO("||OS_Task   ||---->>xTaskCreate---->>completed!");

//}
















