/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ��������
*    �ļ����� : Leo_UART
*    ��    �� : V1.0
*    ˵    �� : �ⲿ�����������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-09     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_UART.h"
#include "minmea.h"



uint8_t G_UART_Buffer1[128];
uint8_t G_UART_Buffer1_Counter;
uint8_t G_UART_Buffer2[128];
uint8_t G_UART_Buffer2_Counter;

extern SemaphoreHandle_t    xSemaphore_GPSBuffer;
extern uint8_t              G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ�

/* ���ڻص����� */
static void vUART_GPS_EventHandler(app_uart_evt_t * p_event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //�����յ�����
    if ((p_event->evt_type == APP_UART_DATA_READY) && (G_SDCard_FileIsOpen == 1))
	{
        uint8_t mChar;
        while(app_uart_get(&mChar) == NRF_SUCCESS)
        {
            //�����������ˣ�ֱ�����֮ǰ�洢��
            if(G_UART_Buffer1_Counter == 128)
            {
                G_UART_Buffer1_Counter = 0;
                NRF_LOG_INFO("G_Uart_Buffer1 is fulling!");
                NRF_LOG_FLUSH(); 
            }
            
            //���봮�ڻ�����1��
            G_UART_Buffer1[G_UART_Buffer1_Counter] = mChar;
            G_UART_Buffer1_Counter = G_UART_Buffer1_Counter + 1;
            
            //�ж��Ƿ��յ���Ϣ�Ľ�β ��һ�����������
             if((G_UART_Buffer1_Counter > 15) && 
                (G_UART_Buffer1[G_UART_Buffer1_Counter-2] == '\r') && 
                (G_UART_Buffer1[G_UART_Buffer1_Counter-1] == '\n')) 
             {                 
                
                memset(G_UART_Buffer2,0,sizeof(G_UART_Buffer2));
                
                memcpy(G_UART_Buffer2,G_UART_Buffer1,G_UART_Buffer1_Counter);
                G_UART_Buffer2_Counter = G_UART_Buffer1_Counter;
                G_UART_Buffer1_Counter = 0;
                //NRF_LOG_INFO("%s",G_UART_Buffer2);
                
                 //ͨ�������� �ź��� ֪ͨ ������� Э����� 
                 xSemaphoreGiveFromISR(xSemaphore_GPSBuffer,&xHigherPriorityTaskWoken);
                 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                 
            }
            
        }
        
    }
}

/* GPS���ڳ�ʼ��
 *    ���� 0 �ɹ������� 1 ʧ��  */    
uint8_t ucUARTInital_GPS(void)
{
    uint8_t err_code = 0;
    const app_uart_comm_params_t comm_params =
    {
          configGPIO_UART_GPS_RXD,
          configGPIO_UART_GPS_TXD,
          0,                                //���� �������ò�������
          0,                                //���� �������ò�������
          APP_UART_FLOW_CONTROL_DISABLED,   //��ֹ����
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600 //������115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         128,      
                         128,
                         vUART_GPS_EventHandler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
    
    memset(G_UART_Buffer1,0,sizeof(G_UART_Buffer1));
    memset(G_UART_Buffer2,0,sizeof(G_UART_Buffer2));
    G_UART_Buffer1_Counter = 0;
    G_UART_Buffer2_Counter = 0;
  
    return err_code;
}    
    


/**
 * GPS UTCʱ��ת ������  */
void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek)
{ 
/*****Э������ʱת��ΪGPS�������ʾ*****///����ʱ��ӦΪЭ������ʱ��������ʱ��-8������ʱ��ΪGPS�ܺ�����
    int DayofYear = 0;
    int DayofMonth = 0;


    for(int i = 1980; i < year; i++) //��1980�굽��ǰ�����һ�꾭��������
    {
        if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
        DayofYear += 366;
        else
        DayofYear += 365;
    }
    for(int i = 1; i < month; i++)//��һ�µ���ǰ�µ���һ�¾���������
    {
        if(i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i ==12)
            DayofMonth += 31;
        else 
            if(i == 4 || i == 6 || i == 9 || i == 11)
                DayofMonth += 30;
            else
            {
                if((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
                    DayofMonth += 29;
                else
                    DayofMonth += 28;
            }
    }
    
    int Day;
    Day = DayofMonth + day + DayofYear-6;
    //*weekNo = Day/7;
    *secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second+18;//18��ʾ����
    return ;
}


///**
// * GPS���ݽ���  */
//uint8_t ucGPSData_Decode(uint8_t * pData,uint8_t uNumber)
//{
//    enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)pData);   
//    
//                
//}


