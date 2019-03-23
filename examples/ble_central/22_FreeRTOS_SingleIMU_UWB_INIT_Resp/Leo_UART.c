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
#include "minmea.h"


extern TaskHandle_t    xTaskHandle_GPS_RxData; 
extern uint8_t         G_SDCard_FileIsOpen;               //����Ƿ��Ѿ����ļ�
extern uint16_t        G_MicroSecond; 






#define UART_RX_BUF_SIZE 128      
#define UART_TX_BUF_SIZE 128



//uint8_t G_Uart_Buffer1[UART_RX_BUF_SIZE];
//uint8_t G_Uart_Buffer2[UART_RX_BUF_SIZE];
//uint8_t G_Uart_Buffer_Number;

//nrfx_uart_t   G_Uart_GPS = NRFX_UART_INSTANCE(0);


//static void vUART_GPS_EventHandler(nrfx_uart_event_t const * p_event,void *p_context)
//{
//    switch (p_event->type)
//    {
//    	case NRFX_UART_EVT_RX_DONE:
//            if(G_Uart_Buffer_Number == 1)
//                G_Uart_Buffer_Number = 2;
//            else
//                G_Uart_Buffer_Number = 1;
//            
//            ucUART_GPS_RX();
//            
//            //�ռ����̶����ȵ��ַ�
//            NRF_LOG_INFO("                     Uart RX %d data",UART_RX_BUF_SIZE);
//            NRF_LOG_FLUSH();  
//        
//    		break;
//            
//    	case NRFX_UART_EVT_ERROR:
//            NRF_LOG_INFO("                     Uart RX error!!!!");
//            NRF_LOG_FLUSH(); 
//            ucUART_GPS_RX();
//    		break;
//    	default:
//    		break;
//    }
//    
//    
//    
//    if(p_event->type == NRFX_UART_EVT_RX_DONE)
//    {
//        if(G_Uart_Buffer_Number == 1)
//            G_Uart_Buffer_Number = 2;
//        else
//            G_Uart_Buffer_Number = 1;
//        
//        //ucUART_GPS_RX();
//        
//        //�ռ����̶����ȵ��ַ�
//        NRF_LOG_INFO("                     Uart RX %d data",UART_RX_BUF_SIZE);
//        NRF_LOG_FLUSH();  
//        
//    }
//}
//                                           

///* GPS���ڳ�ʼ��
// *    ���� 0 �ɹ������� 1 ʧ��  */    
//uint8_t ucUART_GPS_Initial(void)
//{
//    uint8_t erro_code = 0;
//    nrfx_uart_config_t tUartConfig =  NRFX_UART_DEFAULT_CONFIG;
//    tUartConfig.pseltxd = configGPIO_UART_GPS_TXD;
//    tUartConfig.pselrxd = configGPIO_UART_GPS_RXD;
//    
//    erro_code = nrfx_uart_init(&G_Uart_GPS,&tUartConfig,vUART_GPS_EventHandler);
//    
//    G_Uart_Buffer_Number = 1;
//    
//    return erro_code;  
//}


///* GPS���ڽ����ַ�
// * @param[in] p_instance Pointer to the driver instance structure.
// * @param[in] p_data     Pointer to data.
// * @param[in] length     Number of bytes to receive.
// *
// * @retval    NRFX_SUCCESS If initialization was successful.
// * @retval    NRFX_ERROR_BUSY If the driver is already receiving
// *                            (and the secondary buffer has already been set
// *                            in non-blocking mode).
// * @retval    NRFX_ERROR_FORBIDDEN If the transfer was aborted from a different context
// *                                (blocking mode only, also see @ref nrfx_uart_rx_disable).
// * @retval    NRFX_ERROR_INTERNAL If UART peripheral reported an error.  */   
//uint8_t ucUART_GPS_RX(void)
//{
//    if(G_Uart_Buffer_Number == 1)
//        return nrfx_uart_rx(&G_Uart_GPS,G_Uart_Buffer1,UART_RX_BUF_SIZE);
//    else
//        return nrfx_uart_rx(&G_Uart_GPS,G_Uart_Buffer2,UART_RX_BUF_SIZE);
//}







/* ���ڻص����� */
static void vUART_GPS_EventHandler(app_uart_evt_t * p_event)
{
    uint16_t tTime = 0;
    uint8_t mChar;
    switch (p_event->evt_type)
    {

        case APP_UART_DATA_READY:
//            if(G_SDCard_FileIsOpen == 1)
//            {
//                tTime =   G_MicroSecond;
                
            while(app_uart_get(&mChar) == NRF_SUCCESS)
            ;
//                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//                xTaskNotifyFromISR(xTaskHandle_GPS_RxData,0,eNoAction,&xHigherPriorityTaskWoken);            
//                portYIELD_FROM_ISR(xHigherPriorityTaskWoken); 
//            tTime = G_MicroSecond - tTime;
//            NRF_LOG_INFO("                     Uart have data  %d",tTime );

//            }
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
                         UART_RX_BUF_SIZE,      
                         UART_TX_BUF_SIZE,
                         vUART_GPS_EventHandler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);
  
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


