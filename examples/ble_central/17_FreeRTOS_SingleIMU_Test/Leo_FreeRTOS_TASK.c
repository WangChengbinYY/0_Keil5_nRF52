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


//ȫ�ֱ���_ʱ����� 
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
uint8_t	    G_GPSWeekSecond_Data[7];          //�������ݲɼ�ʱ����¼�ĵ�ʱʱ�̵�ʱ��

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ǿ����������
uint8_t	    G_MAG_Coeffi[6]; 

//ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ŵĻ���                
uint8_t	    G_IMU_Data_A[24];                   //��һ��IMU_A(U4)��ŵ�����
uint8_t	    G_IMU_Data_B[24];                   //�ڶ���IMU_A(U5)��ŵ�����
uint8_t	    G_IMUDataA_Counter;                  //MPU9255�жϴ����ļ�����	    
uint8_t	    G_IMUDataB_Counter;


// ȫ�ֱ���_SDCard�洢����                                                         
uint8_t	    G_CollectData[512];                 //SDCardҪ�������ݵĻ���
uint16_t    G_CollectData_Counter;  
// ȫ�ֱ���_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0




/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */


/* 1�� */
#define taskPRIO_GPS_RxData                  1          //����GPS���ݲ������������ɹ���֪ͨ�洢 

/* 2�� */
//5ms ѭ����ʱ�� ����

/* 3�� */
#define taskPRIO_SDCard_Save                 3          //SDCard�洢����

/* 4�� */
#define taskPRIO_SDCard_Close                4          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
//TaskHandle_t    xTaskHandle_CollectData         = NULL;         
TimerHandle_t     xTimerHandle_CollectData        = NULL;         /*5ms�����Ĳɼ�����    ��� */


/**
 * ȫ�ֱ���_������_SDCard����  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;




/**
 * ȫ�ֱ�����ʼ������   ������  
*/
void vINIT_Variable(void)
{
    //ȫ�ֱ���_ʱ����� 
    G_GPSWeekSecond     = 0;                    //GPS����������
    G_MicroSecond       = 0;                    //nRF52ʱ����������Ƶ� 1s��1000����ֵ��
    //���ݲɼ��������¼
    memset(G_GPSWeekSecond_Data,0,7);
    G_GPSWeekSecond_Data[0] = 0xA0;
	G_GPSWeekSecond_Data[1] = 0xA0;
    G_GPSWeekSecond_Data[6] = 0xFF;   
    
    //ȫ�ֱ���_IMU_A(U4)��IMU_B(U5)��ǿ����������
    memset(G_MAG_Coeffi,0,6);
    G_MAG_Coeffi[5] = 0xFF;     
    
    //ȫ�ֱ���_IMU_A(U4)����
    memset(G_IMU_Data_A,0,24);
    G_IMU_Data_A[0] = 0xB1;
	G_IMU_Data_A[1] = 0xB1;
    G_IMU_Data_A[23] = 0xFF;
    memset(G_IMU_Data_B,0,24);
    G_IMU_Data_B[0] = 0xB2;
	G_IMU_Data_B[1] = 0xB2;
    G_IMU_Data_B[23] = 0xFF;
    G_IMUDataA_Counter = 0;                  //IMU�ɼ��Ĵ�������ֵ	    
    G_IMUDataB_Counter = 0;      
    
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
        if((G_CollectData_Counter > 0) && (G_SDCard_FileIsOpen == 1) )
        {
            if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
            {
                memcpy(tData,G_CollectData,G_CollectData_Counter);
                tData_Count = G_CollectData_Counter;
                G_CollectData_Counter = 0; 
                //�ͷ���Դ
                xSemaphoreGive( xMutex_SDCDBuffer ); 
                
                erro_code = ucSDCard_SaveData(tData,tData_Count);                    

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
            
        }else{
            continue;
        }
    }        
}


/*------------------------------------------------------------
 *�ⲿ���������ݲɼ�����Ҫ������
 *  MPU9255A��MPU9255B��ѹ��������
 *------------------------------------------------------------*/
static void vTimer_CollectData(xTimerHandle pxTimer)
{
    //(1)��¼��������
    memcpy(G_GPSWeekSecond_Data+2,&G_GPSWeekSecond,sizeof(G_GPSWeekSecond));       
    
    //(2)�ɼ�IMU_A ������
    //��¼ ms ����
    memcpy(G_IMU_Data_A+2,&G_MicroSecond,sizeof(G_MicroSecond));
    //ѡ��IMU_A nCS�ܽ�
    nrfx_gpiote_out_clear(configGPIO_SPI_IMUA_nCS); 
    nrf_delay_us(1); 
    //�ɼ�IMU_A ������
    Leo_MPU9255_Read_ACC(G_IMU_Data_A+5);
    Leo_MPU9255_Read_Gyro(G_IMU_Data_A+11);
    Leo_MPU9255_Read_Magnetic(G_IMU_Data_A+17);
    //�ر�IMU_A nCS�ܽ�
    nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);   
    

    
    /*(4) �ɼ�ѹ�������� ������  δ���*/      
    
    
    /*(5) ���ɼ�����,����洢 �ȴ�2ms�������û���ͷţ�������˴δ洢*/
    if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 2 ) == pdTRUE)
    {
        //�������� ���뻺��
        memcpy(G_CollectData+G_CollectData_Counter,G_GPSWeekSecond_Data,sizeof(G_GPSWeekSecond_Data));
        G_CollectData_Counter = G_CollectData_Counter + sizeof(G_GPSWeekSecond_Data); 
        //IMU_A���� ���뻺��
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

/*-----------------------------------------------------------------------*/
/* ��������                                                              */
/*-----------------------------------------------------------------------*/
uint8_t vTask_CreatTask(void)
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    TickType_t xTimer = 5;
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

    
    //(4) �����ɼ����� 5ms��ʱ�� 
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
            NRF_LOG_INFO("vTimer_CollectData Start is Wrong��");
            NRF_LOG_FLUSH(); 
        }else{
            NRF_LOG_INFO("vTimer_CollectData Start is OK��");
            NRF_LOG_FLUSH(); 
        }
        
    }    
    return erro_code;    
}




