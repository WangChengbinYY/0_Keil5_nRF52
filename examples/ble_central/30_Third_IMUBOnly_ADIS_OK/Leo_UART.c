/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ��������
*    �ļ����� :Leo_UART
*    ��    �� : V1.0
*    ˵    �� : �ⲿ�����������
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-09     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_UART.h"



extern TaskHandle_t    xTaskHandle_GPS_RxData; 
extern uint8_t         G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ�
extern uint16_t        G_MicroSecond; 
extern uint8_t         G_Uart_Buffer1[configBufferUART_RX_SIZE];
extern uint8_t         G_Uart_Buffer_Number;





/* ���ڻص����� */
static void vUART_GPS_EventHandler(app_uart_evt_t * p_event)
{
    uint16_t tIsEnd = 0;
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:                
            while(app_uart_get(G_Uart_Buffer1+G_Uart_Buffer_Number) == NRF_SUCCESS)
            {
                if((char)(*(G_Uart_Buffer1+G_Uart_Buffer_Number)) == '\n')
                    tIsEnd = 1;
                G_Uart_Buffer_Number++;
                if(G_Uart_Buffer_Number == configBufferUART_RX_SIZE)
                    G_Uart_Buffer_Number = 0;
            }
                        
            if((G_SDCard_FileIsOpen == 1)&&(tIsEnd == 1))
            {
                tIsEnd = 0;
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                xTaskNotifyFromISR(xTaskHandle_GPS_RxData,0,eNoAction,&xHigherPriorityTaskWoken);            
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
            }
    		break;
    	default:
    		break;
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
                         256,      
                         128,
                         vUART_GPS_EventHandler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
    
//    nrf_delay_ms(20);  
//    uint8_t mGLL[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B};
//    uint8_t mGSA[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32};
//    uint8_t mGSV[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39};
//    uint8_t mRMC[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40};
//    uint8_t mVTG[16]={0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47};
//    uint8_t i = 0;
//    
//    err_code |= nrf_drv_uart_tx(&app_uart_inst, mGLL, 16);
//    nrf_delay_ms(20);
//    err_code |= nrf_drv_uart_tx(&app_uart_inst, mGSA, 16);
//    nrf_delay_ms(20);
//    err_code |= nrf_drv_uart_tx(&app_uart_inst, mGSV, 16);
//    nrf_delay_ms(20);
//    err_code |= nrf_drv_uart_tx(&app_uart_inst, mRMC, 16);
//    nrf_delay_ms(20);
//    err_code |= nrf_drv_uart_tx(&app_uart_inst, mVTG, 16);
//    nrf_delay_ms(20);
    
//    for(i=0;i<16;i++)
//        err_code |= app_uart_put(mGLL[i]);
//    for(i=0;i<16;i++)
//        err_code |= app_uart_put(mGSA[i]);    
//    for(i=0;i<16;i++)
//        err_code |= app_uart_put(mGSV[i]);
//    for(i=0;i<16;i++)
//        err_code |= app_uart_put(mRMC[i]);    
//    for(i=0;i<16;i++)
//        err_code |= app_uart_put(mVTG[i]); 

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





