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




//ȫ�ֱ���_ʱ����� 
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0

//ȫ�ֱ�����IMU_B(U5)��ŵĻ���    
uint8_t	    G_IMU_Data_B_ADIS[25];               //�ڶ���IMU_A(U5)��ŵ�����

//ȫ�ֱ���_ѹ������������
uint8_t     G_FOOTPresure[17];


// ȫ�ֱ���_SDCard�ļ�������ʶ                                                         
uint8_t     G_SDCard_FileIsOpen;                                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0

// ȫ�ֱ���_SDCard�洢����  
uint8_t     G_SDCard_CirBuffer[configSDCard_BufferSize];
uint8_t*    G_SDCard_CB_pSave = NULL;
uint8_t*    G_SDCard_CB_pLoad = NULL;
uint16_t    G_SDCard_CB_Counter; 


/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */
#define taskPRIO_SDCard_Close                0          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

/* 1�� */
#define taskPRIO_SDCard_Save                 1          //SDCard�洢���� 

/* 5�� */
#define taskPRIO_CollectData_IMUB            5          //�ɼ� IMUB(U5)������ ��ADIS��MTI�Ĳɼ�Ϊ��				         

/* 7�� */
#define taskPRIO_TaskStart                   7          //�������� 

/*=========================================== ������ر��� ============================================*/
/**
 * ȫ�ֱ���_���������
*/
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */

TaskHandle_t    xTaskHandle_CollectData_IMUB    = NULL;         //IMUB(U5) ��IMU���ݲɼ�����

TaskHandle_t    xTaskHandle_TaskStart           = NULL;

///* ȫ�ֱ���_������_SDCard����  */
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;
/* ȫ�ֱ���_������_SPI IMUʹ�û���  */
SemaphoreHandle_t   xMutex_IMUSPI               = NULL;


/**
 * ȫ�ֱ�����ʼ������   ������  
*/
void vINIT_Variable(void)
{
    //ȫ�ֱ���_ʱ����� 
    G_GPSWeekSecond     = 0;                    //GPS����������
    G_MicroSecond       = 0;                    //nRF52ʱ����������Ƶ� 1s��1000����ֵ��
    
    
    memset(G_IMU_Data_B_ADIS,0,25);
    G_IMU_Data_B_ADIS[0] = 0xB3;
    G_IMU_Data_B_ADIS[1] = 0xB3;
    G_IMU_Data_B_ADIS[24] = 0xFF;     
    
    //ȫ�ֱ���_ѹ������������
    memset(G_FOOTPresure,0,17);
    G_FOOTPresure[0] = 0xC1;
    G_FOOTPresure[1] = 0xC1;
    G_FOOTPresure[16] = 0xFF;    
  
    
    //ȫ�ֱ���_SDCard�ļ�������ʶ 
    G_SDCard_FileIsOpen = 0;                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0 

    //SDCard�洢����
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
        nrf_delay_ms(200); 
        
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


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    NRF_LOG_INFO("OverFlow OverFlow!!!");
    NRF_LOG_FLUSH(); 
    
}


/*------------------------------------------------------------
 *SDCard�洢���� ����
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
 *  IMUB(U5) ��IMU���ݲɼ����񼰴洢
 *------------------------------------------------------------*/
static void vTask_CollectData_IMUB(void *pvParameters)
{
    uint8_t erro_IMUB = 0;
    uint8_t tIMUB_ADIS_Data[22] = {0};
	nrf_saadc_value_t tSAResult[4] = {0};
    
    while(1)
    {
        xTaskNotifyWait(0x00000000,0xFFFFFFFF,NULL,portMAX_DELAY); 

//1. �ɼ� IMUB ������        
        //�ȴ� ����SPI���������ͷ�
        if(xSemaphoreTake( xMutex_IMUSPI, ( TickType_t ) 4 ) == pdTRUE)
        { 
            //��IMU_B��Ƭѡ�ܽ�

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

            
//2. �ɼ�ѹ������������
        //�ɼ� AD ͨ��������
        nrfx_saadc_sample_convert(0,tSAResult);
        nrfx_saadc_sample_convert(1,tSAResult+1);        
        nrfx_saadc_sample_convert(2,tSAResult+2);  
        nrfx_saadc_sample_convert(3,tSAResult+3);    
        //���⣺û���жϲɼ����ݵ���ȷ��!          
        memcpy(G_FOOTPresure+8,tSAResult,sizeof(tSAResult)); 
            
//3. ����洢�ж�       
        /*���ɼ�����,����洢 �ȴ�5ms(IMUA 200Hz)�������û���ͷţ�������˴δ洢*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 5 ) == pdTRUE)
        {
            //(1) ����IMUB ������

            //��ֹ�������
            if((sizeof(G_IMU_Data_B_ADIS)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
            {
                //IMU_A���� ���뻺��
                if(erro_IMUB == 0)
                {
                    ucCircleBuffer_SaveData(G_IMU_Data_B_ADIS,sizeof(G_IMU_Data_B_ADIS));
                }else
                {
                    //IMUB ���ݲɼ�����
                    NRF_LOG_INFO("  IMU_B CellectData is Wrong!");
                    NRF_LOG_FLUSH();                    
                }
            }else
            {
                //����
                NRF_LOG_INFO("  SDCard Buffer is OverFlow_IMUA!");
                NRF_LOG_FLUSH();
            }

            
            //(2) ����ѹ������������
            if((sizeof(G_FOOTPresure)+G_SDCard_CB_Counter) <= configSDCard_BufferSize)
            {
                //ѹ�������� ���� ���뻺��
                ucCircleBuffer_SaveData(G_FOOTPresure,sizeof(G_FOOTPresure));
            }else
            {
                //����
                NRF_LOG_INFO("  SDCard Buffer is OverFlow_FOOT!");
                NRF_LOG_FLUSH();
            }    
          
            //�ͷ���Դ
            xSemaphoreGive( xMutex_SDCDBuffer );   
        }else
        {
            NRF_LOG_INFO("  xMutex_SDCDBuffer IMUA TimeOUT");
            NRF_LOG_FLUSH();
        }  
            
         
//5. ���������ݴ��� SDCard��          
        if(G_SDCard_CB_Counter >= configSDCard_SaveSize)
        {               
            //֪ͨ SDCard�洢
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
    
    //(6) ����IMUB �ɼ�����  
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
 
    //��4�� ��ʼ�� IMU 
    //�������ʼ��IMU_B ��ADIS
    erro_code |= ucIMU_INIT_MPU_ADIS();
    NRF_LOG_INFO(("||Initialize||-->IMU------------>error  0x%x"),erro_code);    
    
    //��5�� ��ʼ�� SAADC ѹ��������
    erro_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),erro_code); 
    NRF_LOG_FLUSH(); 
    
    //��6����ʼ�� UWB
    
    //��7����ʼ����ʱ��������
    erro_code |= ucTimerInitial_3();        //1ms ��ʱ 
    erro_code |= ucTimerStart_3();     
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),erro_code);
    NRF_LOG_FLUSH(); 
    
    //��8����ʼ���жϲ�����
    erro_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    erro_code |= ucINTInital_IMUB();
    ucINTStart_SDCard();
    ucINTStart_IMUB();
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),erro_code);   
    NRF_LOG_FLUSH(); 
    
    //��9����ʼ�����ڲ�����
    
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
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    //SPI���ݲɼ�
    xMutex_IMUSPI = xSemaphoreCreateMutex();
    if(xMutex_IMUSPI == NULL)
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


