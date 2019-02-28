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



/*========================== ȫ�ֱ������壡================================*/
/* ȫ�ֱ���_ʱ����� */
uint32_t    G_GPSWeekSecond;                   //GPS����������
uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0

/* ȫ�ֱ���_SDCard�ļ�������ʶ */                                                        
uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0

/* ȫ�ֱ���_IMU���ݲɼ��� SPI2 ʵ����UWB ��SPI0��SDCard ��SPI1�� IMU �� ѹ�������� ��SPI2�� */
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	

uint8_t	    G_CollectData[512];                 //SDCardҪ�������ݵĻ���
uint16_t	    G_CollectData_Counter;                 

uint8_t	    G_IMU_Data1[28];                    //��һ��MPU9255��ŵ�����
uint8_t	    G_IMU_Data2[28];                    //�ڶ���MPU9255��ŵ�����
uint8_t	    G_IMUData_Counter;                  //MPU9255�жϴ����ļ�����	    








//uint8_t     G_GPS_Data[33];                     //GPS��������

//uint8_t     G_FOOTPressure_Data[25];            //�㲿ѹ������������
//uint8_t	    G_FOOTPressure_Counter;             //�㲿ѹ�����������ݵļ�����

//uint8_t     G_UWBDistance_Data[12];             //UWB��ഫ��������
//uint8_t	    G_UWBDistance_Counter;              //UWB��ഫ�������ݵļ�����


//uint8_t	    G_SDCDBuffer1[1024];               //˫����buffer
//uint16_t	G_SDCDBuffer1_NUM;	
//uint8_t	    G_SDCDBuffer2[1024];
//uint16_t	G_SDCDBuffer2_NUM;	

///**
// * ȫ�ֱ���_ϵͳ����   ������
//*/
//uint8_t     G_Ctrl_DataSave;                    /* ���ݴ洢���Ʊ�־λ 1�洢��0���洢 */










/*=========================================== �������ȼ��趨 ============================================*/
/* 0�� */
#define taskPRIO_INIT                        0          /* ϵͳ��ʼ�������ڿ�ʼִ��һ�Σ���ʼ��ʧ���������ѭ��LED������˸*/
#define taskPRIO_LED_SLOW                    0          /* LED����(1sһ��)��(����GPS��ʼ��λ�ɹ�������ȡʱ��)   */
#define taskPRIO_LED_QUICK                   0          /* LED����(500ms)��(����SDCard�Ĵ洢��ֹͣ�����ɹ�)   */

/* 1�� */
#define taskPRIO_GPS_RxData                  1          //����GPS���ݲ������������ɹ���֪ͨ�洢 
#define taskPRIO_SDCard_Open                 1          //SDCard�����ļ��ɹ�  ��־λ��1  ���ݻᱻ�洢
#define taskPRIO_SDCard_Close                1          //SDCard�ر��ļ��ɹ�  ��־λ��0  ���ݲ���洢

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
TaskHandle_t    xTaskHandle_TaskINIT            = NULL;         /*ϵͳ��ʼ������       ��� */
TaskHandle_t    xTaskHandle_CollectData         = NULL;         /*5ms�����Ĳɼ�����    ��� */
TaskHandle_t    xTaskHandle_SDCard_Save         = NULL;         /*SDCard�洢����       ��� */
TaskHandle_t    xTaskHandle_SDCard_Open         = NULL;         /*SDCard �½��ļ�����  ��� */
TaskHandle_t    xTaskHandle_SDCard_Close        = NULL;         /*SDCard �ر��ļ�����  ��� */


/**
 * ȫ�ֱ���_������_SDCard����  
*/
SemaphoreHandle_t   xMutex_SDCDBuffer           = NULL;





/*======================================= ȫ�ֱ������� ����������������==================================================*/
/* ȫ�ֱ���_ʱ����� */
//uint32_t    G_GPSWeekSecond;                   //GPS����������
//uint16_t    G_MicroSecond;                     //nRF52ʱ����������Ƶ� 1s��1000����ֵ�����ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0

///* ȫ�ֱ���_SDCard�ļ�������ʶ */                                                        
//uint8_t     G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0

///* ȫ�ֱ���_IMU���ݲɼ��� SPI2 ʵ����UWB ��SPI0��SDCard ��SPI1�� IMU �� ѹ�������� ��SPI2�� */
//nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);  			//MPU9255ʹ�õ�SPIʵ��		
//uint8_t         G_MPU9255_SPI_xfer_Done = 1;

//uint8_t     G_WRONG_Record[10];                 //�ɼ��������ݳ���ļ�¼

//uint8_t	    G_IMU_Data[28];                 //MPU9255��������ŵ�����
//uint8_t	    G_IMUData_Counter;                  //MPU9255�жϴ����ļ�����

//uint8_t     G_GPS_Data[33];                     //GPS��������

//uint8_t     G_FOOTPressure_Data[25];            //�㲿ѹ������������
//uint8_t	    G_FOOTPressure_Counter;             //�㲿ѹ�����������ݵļ�����

//uint8_t     G_UWBDistance_Data[12];             //UWB��ഫ��������
//uint8_t	    G_UWBDistance_Counter;              //UWB��ഫ�������ݵļ�����


//uint8_t	    G_SDCDBuffer1[1024];               //˫����buffer
//uint16_t	G_SDCDBuffer1_NUM;	
//uint8_t	    G_SDCDBuffer2[1024];
//uint16_t	G_SDCDBuffer2_NUM;	

///**
// * ȫ�ֱ���_ϵͳ����   ������
//*/
//uint8_t     G_Ctrl_DataSave;                    /* ���ݴ洢���Ʊ�־λ 1�洢��0���洢 */






/**
 * ȫ�ֱ�����ʼ������   ������  
*/
static void vINIT_Variable(void)
{
    G_GPSWeekSecond     = 0;                    //GPS����������
    G_MicroSecond       = 0;                    //nRF52ʱ����������Ƶ� 1s��1000����ֵ��
                                                        //�� �ⲿGPS��1PPSУ׼ 1PPS����ʱ ������0
    G_SDCard_FileIsOpen = 0;                    //����Ƿ��Ѿ����ļ� û�򿪣�Ĭ��Ϊ0
    
    memset(G_CollectData,0,512);
    G_CollectData_Counter = 0;
    
    //IMU����
    memset(G_IMU_Data1,0,28);
    G_IMU_Data1[0] = 0xA1;
	G_IMU_Data1[1] = 0xA2;
    G_IMU_Data1[27] = 0xFF;
    memset(G_IMU_Data2,0,28);
    G_IMU_Data2[0] = 0xA1;
	G_IMU_Data2[1] = 0xA3;
    G_IMU_Data2[27] = 0xFF;
    G_IMUData_Counter    = 0;                     
    


//    
//    //GPS��������
//    memset(G_GPS_Data,0,33);
//    G_GPS_Data[0] = 0xA3;
//    G_GPS_Data[1] = 0xA4;
//    G_GPS_Data[32] = 0xFF;

//    //�㲿ѹ������������
//    memset(G_FOOTPressure_Data,0,25);
//    G_FOOTPressure_Data[0] = 0xA5;
//	G_FOOTPressure_Data[1] = 0xA6;
//    G_FOOTPressure_Data[24] = 0xFF;
//    G_FOOTPressure_Counter = 0;
//    
//    //UWB��ഫ��������
//    memset(G_UWBDistance_Data,0,12);
//    G_UWBDistance_Data[0] = 0xA7;
//	G_UWBDistance_Data[1] = 0xA8;
//    G_UWBDistance_Data[11] = 0xFF;
//    G_UWBDistance_Counter = 0;
 
}


/*------------------------------------------------------------
 *������صĳ�ʼ������Ҫ������
 *  1��ȫ�ֱ�����ʼ����
 *  2���ܽ����ã�LED �жϣ���ʱ����
 *  3�����ഫ����(IMU GPS SDCard)�ȵĳ�ʼ��
 *------------------------------------------------------------*/
static uint8_t ucINIT_Peripheral()
{
    uint8_t err_code = 0;
/**
 * 0. ȫ�ֱ�����ʼ��    
 */    
    vINIT_Variable();    
    
/**
 * 1. GPIO�ܽ��趨    
 */
    err_code |= nrfx_gpiote_init();
    
    /* (1) LED �ܽ� */
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
//    nrfx_gpiote_out_clear(configGPIO_LED_R);  //���0������LED    
//    nrf_delay_ms(5000);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //���1��LED����    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();
    
    /* (2) INT�жϹܽų�ʼ�� */    
    err_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    err_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),err_code);   
    NRF_LOG_FLUSH();    

/**
 * 2. ��ʱ����ʼ��    
 */
    /* (1) 1ms ��ʱ�� ��ʼ�� ʹ�õ�TIMR3 */   
    err_code |= ucTimerInitial_3();      /* TIMER3 ��������ʼ��*/ 
    err_code |= ucTimerInitial_4();
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),err_code);  
    NRF_LOG_FLUSH();

/**
 * 3. ��ʼ��SDCard ��������¼�ļ�    
 */
    err_code |= ucSDCard_INIT();   
    err_code |= ucSDCard_OpenFile();      
    //LeoDebug
//    nrf_delay_ms(1000);
//    err_code |= ucSDCard_SaveData(&err_code,sizeof(err_code));
//    err_code |= ucSDCard_CloseFile();
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code); 
    NRF_LOG_FLUSH();   
    
/**
 * 4. ��ʼ�� ����MPU 
 */
    /* (1) ��ʼ����һ��MPU9255_A  */   
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_MPU1_CS;               //��һ��IMU��nCS�ܽ�
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//ϵͳSPI�ж�Ȩ��Ĭ���趨Ϊ 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPIʹ�õķ�ΧΪ 100KHz~1MHz
    //SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//�������ò��� �� ʵ��spi ���г�ʼ�� 
	err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    NRF_LOG_INFO(("||Initialize||-->SPI_IMU_1------>error  0x%x"),err_code); 
    NRF_LOG_FLUSH();      
    //���IMU_1��ʼ��
    err_code |= ucMPU9255_INIT();
    NRF_LOG_INFO(("||Initialize||-->MPU9255_A-------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();   
    //ж��SPI
    nrf_drv_spi_uninit(&SPI_CollectData);	
  
    /* (2) ��ʼ���ڶ���MPU9255_B  */  
//    SPI_config.ss_pin			= configGPIO_SPI_MPU2_CS;               //�ڶ���IMU��nCS�ܽ�
//	//�������ò��� �� ʵ��spi ���г�ʼ�� 
//    err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
//    NRF_LOG_INFO(("||Initialize||-->SPI_IMU_2------>error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();      
//    //���IMU_1��ʼ��
//    err_code |= ucMPU9255_INIT();
//    NRF_LOG_INFO(("||Initialize||-->MPU9255_B-------->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();   
//    //ж��SPI
//    nrf_drv_spi_uninit(&SPI_CollectData);	       

/**
 * 5. ������ȷ��SPI���ȴ�Ƭѡ�ܽ����� 
 */
    SPI_config.ss_pin			= NRF_DRV_SPI_PIN_NOT_USED;         //��ʹ��nCS�ܽ�
    SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;
    err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
   //��һ��SPI Ƭѡ
    nrfx_gpiote_out_uninit(configGPIO_SPI_MPU1_CS);
    err_code |= nrfx_gpiote_out_init(configGPIO_SPI_MPU1_CS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_MPU1_CS);  //���1     
   //�ڶ���SPI Ƭѡ
    nrfx_gpiote_out_uninit(configGPIO_SPI_MPU2_CS);
    err_code |= nrfx_gpiote_out_init(configGPIO_SPI_MPU2_CS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_MPU2_CS);  //���1       
    NRF_LOG_INFO(("||Initialize||-->SPI_CollectData->error  0x%x"),err_code); 
    NRF_LOG_FLUSH(); 
    
    
    return err_code;
}




/*------------------------------------------------------------
 *SDCard�½��ļ����� ����
 *------------------------------------------------------------*/
static void vTask_SDCard_Open(void *pvParameters)
{
    uint8_t i = 0;
    uint8_t erro_code = 0;
    while(1)
    {
        /*(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/
        
        erro_code = ucSDCard_OpenFile();
        if(erro_code == 0)
        {
            for(i=0;i<15;i++)
            {
                nrfx_gpiote_out_toggle(configGPIO_LED_R);
                nrf_delay_ms(200);
            } 
            G_SDCard_FileIsOpen = 1;
            nrfx_gpiote_out_clear(configGPIO_LED_R);
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
        /*(1) �ȴ�����֪ͨ     */
        xTaskNotifyWait(0x00000000,     
                        0xFFFFFFFF,     
                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/
        
        G_SDCard_FileIsOpen = 0;
        erro_code = ucSDCard_CloseFile();
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
//        if(G_CollectData_Counter >= 256)
//        {

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
            
//        }else{
//            continue;
//        }
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
        
        
        /*(2) �ɼ�MPU9255A ������ */ 
        nrfx_gpiote_out_clear(configGPIO_SPI_MPU1_CS);    
        Leo_MPU9255_Read_ACC(G_IMU_Data1);
        Leo_MPU9255_Read_Gyro(G_IMU_Data1);
        Leo_MPU9255_Read_Magnetic(G_IMU_Data1);  
        nrfx_gpiote_out_set(configGPIO_SPI_MPU1_CS);          
        /*(3) �ɼ�MPU9255B ������ */        
//        nrfx_gpiote_out_clear(configGPIO_SPI_MPU2_CS);    
//        Leo_MPU9255_Read_ACC(G_IMU_Data2);
//        Leo_MPU9255_Read_Gyro(G_IMU_Data2);
//        Leo_MPU9255_Read_Magnetic(G_IMU_Data2);  
//        nrfx_gpiote_out_set(configGPIO_SPI_MPU2_CS);          
        /*(4) �ɼ�ѹ�������� ������  δ���*/      
        
        
        /*(5) ���ɼ�����,����洢 
         *    �ȴ�3ms�������û���ͷţ�������˴δ洢*/
        if(xSemaphoreTake( xMutex_SDCDBuffer, ( TickType_t ) 4 ) == pdTRUE)
        {
            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data1,sizeof(G_IMU_Data1));
            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data1);
//            memcpy(G_CollectData+G_CollectData_Counter,G_IMU_Data2,sizeof(G_IMU_Data2));
//            G_CollectData_Counter = G_CollectData_Counter + sizeof(G_IMU_Data2);        
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





/*------------------------------------------------------------
 *������صĳ�ʼ������Ҫ������
 *  1���������ĳ�ʼ��
 *  2����������ĳ�ʼ��������
 *------------------------------------------------------------*/
static uint8_t ucINIT_Task()
{
    uint8_t erro_code = 0;
    BaseType_t txResult = pdPASS;
    
    /*(1) �������Ľ���_SDCard���� */
    xMutex_SDCDBuffer = xSemaphoreCreateMutex();
    if(xMutex_SDCDBuffer == NULL)
    {
        erro_code = 1;
    }
    
    /*(2) �����ɼ����� */
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
    
    /*(3) ����SDCard�洢���� */    
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
    
    /*(4) ����SDCard �½��ļ����� */      
    txResult = xTaskCreate(vTask_SDCard_Open,
                            "SDCardOpen",
                            configMINIMAL_STACK_SIZE,
                            NULL,
                            taskPRIO_SDCard_Open,
                            &xTaskHandle_SDCard_Open);
    if(txResult != pdPASS)
    {
        erro_code = 1;
    }     
    
    /*(4) ����SDCard �ر��ļ����� */      
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
    
    
    return erro_code;  
}
    



/*------------------------------------------------------------
 *��ʼ������  ִֻ��һ��                                                                              
 *------------------------------------------------------------*/
static void vTask_StartINIT(void *pvParameters)
{
    uint8_t err_code = 0;
    BaseType_t txResult = pdPASS;
    
/**
 * 0. ������س�ʼ��
 *      ��ʼ���ɹ����������
 *      ��ʼ�����󣬺����ɾ����ʼ�������˳�    
 */     
    err_code |= ucINIT_Peripheral();
    if(err_code != 0)
    {
        vTaskDelete(xTaskHandle_TaskINIT);
        NRF_LOG_INFO("Peripheral Initialization is Wrong!!");
        NRF_LOG_FLUSH();
        return ;
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R);
    }    

/**
 * 1. ������س�ʼ���������ú��Զ�������
 *      �����ʼ���ɹ����������
 *      �����ʼ�����󣬺����ɾ����ʼ�������˳�       
 */
    err_code |= ucINIT_Task();
    if(err_code != 0)
    {
        vTaskDelete(xTaskHandle_TaskINIT);
        NRF_LOG_INFO("TASK Initialization is Wrong!!");
        NRF_LOG_FLUSH();
        return ;
    }else{
        nrfx_gpiote_out_clear(configGPIO_LED_R);
    }       
/**
 * 2. �жϼ���ʱ�� ����  δ���
 */
    //(1)��ʱ������
    err_code |= ucTimerStart_3();      /* TIMER3 ��������ʼ��*/ 
    err_code |= ucTimerStart_4();
    
    //(2)�ж�����
    ucINTStart_SDCard();
/**
 * 3. ɾ����������    
 */
    vTaskDelete(xTaskHandle_TaskINIT);
    NRF_LOG_INFO("||  Start  ||-->TaskINI is Delete!!");
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/      
    
// 
///*----------------------------------------------------------------------------*/ 
//    
//    char tcTest[] = "This is just a test!";
//    err_code |= ucSDCard_SaveData(tcTest,sizeof(tcTest)); 
//    NRF_LOG_INFO(("||Initialize||-->SDCard Save Test--->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH(); 


///**
// * 6. ��ʼ�� GPS���ڽ���    
// */
//    err_code |= ucGPS_INTUART();    
//    NRF_LOG_INFO(("||Initialize||-->GPS Uart------->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();
///*----------------------------------------------------------------------------*/ 

///**
// * 7. �������ʼ���Ľ������LED����ʾ
// *      ���������ʼ�������ȷ��
// *      ���������ʼ������ 
// */
//    if(err_code != 0)
//    {
//        NRF_LOG_INFO("Warning Warning Warning......Initialization is Wrong!!!!!"); 
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        }        
//    }else
//    {
//        //��ȷ�������
//        nrfx_gpiote_out_clear(configGPIO_LED_R);
//    }
///*----------------------------------------------------------------------------*/ 
//    
///**
// * 8. ��������صĳ�ʼ�� 
// */    

//    
//    /*(2) ����ʱ������Ľ��� �����жϵ� */
//    xTimers_StartINT = xTimerCreate("StartINT",             /* ��ʱ������ */
//                                  30000,                    /* ��ʱ������,��λʱ�ӽ��� */
//                                  pdFALSE,                  /* ������ */
//                                  (void *) 1,               /* ��ʱ��ID */
//                                  TimerFunction_StartINT);  /* ��ʱ���ص����� */
//     if(xTimers_StartINT == NULL)
//     {
//         /* The timer was not created. */
//        NRF_LOG_INFO("||Warning||-->StartINT Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//     }                                 
//                                  
//                                  
//    /*(3) ѭ��ʱ������Ľ��� SDCard�洢���� */
//    xTimers_StarSDCard = xTimerCreate("SDCard",            /* ��ʱ������ */
//                              10,                /* ��ʱ������,��λʱ�ӽ��� */
//                              pdTRUE,          /* ������ */
//                              (void *) 2,      /* ��ʱ��ID */
//                              TimerFunction_SDCardSave); /* ��ʱ���ص����� */
//     if(xTimers_StarSDCard == NULL)
//     {
//         /* The timer was not created. */
//        NRF_LOG_INFO("||Warning||-->SDCard Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//     }                               
//                                  
//    //�ӳ�����ʱ������
//    if((xTimerStart(xTimers_StartINT,0) != pdPASS)||(xTimerStart(xTimers_StarSDCard,0) != pdPASS))
//     {
//         /* The timer could not be set into the Active
//         state. */
//        NRF_LOG_INFO("||Warning||-->SDCard_Timer Start is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        }
//     }                              
//      
//         
//    /*(4) MPU9255������ */
//    txResult = xTaskCreate(vTask_MPU9255_RxData,
//                            "MPU9255",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_MPU9255_RxData,
//                            &xTaskHandle_MPU9255_RxData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskMPU9255 Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }
//    
//    /*(5) GPS ������ */
//    txResult = xTaskCreate(vTask_GPS_RxData,
//                            "GPS",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_GPS_RxData,
//                            &xTaskHandle_GPS_RxData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskGPS Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }    
//    
//    /*(6) UWB������ */
//    txResult = xTaskCreate(vTask_UWB_RxData,
//                            "UWB",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_UWB_RxData,
//                            &xTaskHandle_UWB_GetData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskUWB Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }   
//    
//    /*(7) FootPres������ */
//    txResult = xTaskCreate(vTask_FootPres_RxData,
//                            "FootPres",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_FootPres_RxData,
//                            &xTaskHandle_FootPres_GetData);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->TaskFootPres Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }    
//    
//    /*(8) ���� ֹͣ�洢�������� */  
//    txResult = xTaskCreate(vTask_DataSave_Start,
//                            "SaveStart",
//                            configMINIMAL_STACK_SIZE,
//                            NULL,
//                            taskPRIO_DataSave_Start,
//                            &xTaskHandle_DataSave_Start);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->SaveStart Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }  
//    txResult = xTaskCreate(vTask_DataSave_End,
//                            "SaveEnd",
//                            configMINIMAL_STACK_SIZE+100,
//                            NULL,
//                            taskPRIO_DataSave_End,
//                            &xTaskHandle_DataSave_End);
//    if(txResult != pdPASS)
//    {
//        NRF_LOG_INFO("||Warning||-->SaveEnd Create is failed!!");
//        NRF_LOG_FLUSH();
//        //������ѭ����˸
//        while(1)
//        {
//            nrf_delay_ms(250);
//            nrf_gpio_pin_toggle(configGPIO_LED_R);            
//        } 
//    }  
///*----------------------------------------------------------------------------*/      
//    
  

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





















///**
// * ��ʼ���ݴ洢   ����
//*/
//static void vTask_DataSave_Start(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
//        G_Ctrl_DataSave = 1;       
//        
//    }
//}

///**
// * �������ݴ洢   ����
//*/
//static void vTask_DataSave_End(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
//        G_Ctrl_DataSave = 0;       
//        vTaskDelay( (TickType_t)100 );
//        ucSDCard_CloseFile();        
//    }
//}


///**
// * MPU9255���ݲɼ�����   ����
//*/
//static void vTask_MPU9255_RxData(void *pvParameters)
//{
//    
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
//         /**
//         *(2) �ɼ�MPU9255���� */
//        memcpy(G_IMU_Data+2,&G_GPSWeekSecond,4);
//        memcpy(G_IMU_Data+6,&G_MicroSecond,2);        
//        memcpy(G_IMU_Data+8,&G_IMUData_Counter,1); 
//        
//        /* �ɼ����������� */    
//        Leo_MPU9255_Read_ACC();
//        Leo_MPU9255_Read_Gyro();
//        Leo_MPU9255_Read_Magnetic(); 
//        
//        /*���ǿ��Դ洢 */
//        if(G_Ctrl_DataSave == 1)
//        {
//            /* ��ȡ��Դ�������   �ȴ�2ms һ������5ms*/
//            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//            {   
//                 /*  ��©�����п��ܻ������  ��� �洢����ʱ�Ļ����� */
//                if(G_SDCDBuffer1_NUM + sizeof(G_IMU_Data) > 1024)
//                {
//                    NRF_LOG_INFO("MPU9255 Can't Save, It's Full !!!!!!!!!!!!!!!!!");
//                    NRF_LOG_FLUSH();  
//                }else
//                {
//                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_IMU_Data,sizeof(G_IMU_Data)); 
//                    G_SDCDBuffer1_NUM += sizeof(G_IMU_Data);
//                }               
//                 
//                /* �ͷ���Դ */
//                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
//            }else
//            {
//                /* δ��ȡ����Դ��ʹ��Ȩ�� */
//                NRF_LOG_INFO("MPU9255 Use Buffer is Busy!!!!!!!!!!!!!!!!!");
//                NRF_LOG_FLUSH();            
//            }            
//        }
//        

//        //test        
////        if((G_IMUData_Counter % 255) == 0)
////        {
////            NRF_LOG_INFO("MPU9255 Data is OK!___5s");
////        }        
//    }
//}


///**
// * GPS���ݲɼ�����   ����
//*/
//static void vTask_GPS_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ
//         */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/   
//    }    
//    
//}

///**
// * UWB���ݲɼ�����   ����
//*/    
//static void vTask_UWB_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ
//         */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/   
//    } 
//}



///**
// * ѹ�����������ݲɼ�����   ����
//*/
//static void vTask_FootPres_RxData(void *pvParameters)
//{
//    while(1)
//    {
//        /**
//         *(1) �ȴ�����֪ͨ     */
//        xTaskNotifyWait(0x00000000,     
//                        0xFFFFFFFF,     
//                        NULL,                 /* ����ulNotifiedValue������ulValue�� ������ÿ�����ΪNULL */
//                        portMAX_DELAY);       /* ��������ӳ�ʱ�� portMAX_DELAY ��ʾ��Զ�ȴ�*/        
//         /**
//         *(2) �ɼ�MPU9255���� */
//        
//        /*���ǿ��Դ洢 */
//        if(G_Ctrl_DataSave == 1)
//        {
//            /* ��ȡ��Դ�������   �ȴ�2ms һ������5ms*/
//            if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//            {   
//                 /*  ��©�����п��ܻ������  ��� �洢����ʱ�Ļ����� */
//                if(G_SDCDBuffer1_NUM + sizeof(G_FOOTPressure_Data) > 1024)
//                {
//                    NRF_LOG_INFO("G_FOOTPressure_Data Can't Save, It's Full !!!!!!!!!!!!!!!!!");
//                    NRF_LOG_FLUSH();  
//                }else
//                {
//                    memcpy(G_SDCDBuffer1+G_SDCDBuffer1_NUM,G_FOOTPressure_Data,sizeof(G_FOOTPressure_Data)); 
//                    G_SDCDBuffer1_NUM += sizeof(G_FOOTPressure_Data);
//                }               
//                 
//                /* �ͷ���Դ */
//                xSemaphoreGive( xMutex_SDCDBuffer_1 );            
//            }else
//            {
//                /* δ��ȡ����Դ��ʹ��Ȩ�� */
//                NRF_LOG_INFO("G_FOOTPressure_Data Use Buffer is Busy!!!!!!!!!!!!!!!!!");
//                NRF_LOG_FLUSH();            
//            }            
//        }
//            
//    }
//}


///**
// * �ж�����ʱ������   ����
//*/
//static void TimerFunction_StartINT( TimerHandle_t xTimer )
//{
//    uint8_t err_code = 0;
///*1. ������ʱ��*/
//#if configTIMER3_ENABLE    
//    /* TIMER3 ����������*/ 
//    err_code |= ucTimerStart_3();
//#endif 
//    NRF_LOG_INFO(("||  Start  ||-->TIMER--------->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();    


///*2. �����ⲿ�ж�*/
//    err_code |= ucINTStart_SDCard();    /* SDCard�ж� ���� */    
//    err_code |= ucINTStart_MPU9255();   /* MPU9255�ɼ��ж� ���� */
//    err_code |= ucINTStart_PPS();       /* 1PPS�������ж� ���� */
//    NRF_LOG_INFO(("||  Start  ||-->GPS Uart------->error  0x%x"),err_code); 
//    NRF_LOG_FLUSH();
//    
///*3. ���ݴ洢����*/    
//    G_Ctrl_DataSave = 1;
//    nrfx_gpiote_out_clear(configGPIO_LED_G);
//}

///**
// * SDCardѭ���洢����   ����
//*/
//static void TimerFunction_SDCardSave( TimerHandle_t xTimer )
//{
//    uint8_t     erro_code = 0;
//    if(xSemaphoreTake( xMutex_SDCDBuffer_1, ( TickType_t ) 2 ) == pdTRUE)
//    {
//        if(G_SDCDBuffer1_NUM > 0)
//        {
//            memcpy(G_SDCDBuffer2,G_SDCDBuffer1,G_SDCDBuffer1_NUM);
//            G_SDCDBuffer2_NUM = G_SDCDBuffer1_NUM;
//            G_SDCDBuffer1_NUM = 0;             
//        }
//        /* �ͷ���Դ */
//        xSemaphoreGive( xMutex_SDCDBuffer_1 );          
//                    
//        if(G_Ctrl_DataSave == 1)
//        {
//            erro_code = ucSDCard_SaveData(G_SDCDBuffer2,G_SDCDBuffer2_NUM);  
//            if(erro_code != 0)
//            {
//                NRF_LOG_INFO("SDCard DataSave is Wrong Wrong !!!!!!!!!!!!!");
//                NRF_LOG_FLUSH(); 
//            }
//        }  
//    }else
//    {
//        /* δ��ȡ����Դ��ʹ��Ȩ�� */
//        NRF_LOG_INFO("TimerFunction_FOOT Use Buffer1 is Busy!!!!!!!!!!!!!!!!!");
//        NRF_LOG_FLUSH();            
//    } 
//}














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
















