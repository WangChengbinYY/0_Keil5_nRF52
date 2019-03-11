/*
*********************************************************************************************************
*
*    模块名称 : 外部串口配置
*    文件名称 : Leo_UART
*    版    本 : V1.0
*    说    明 : 外部串口设置相关
*
*    修改记录 :
*        版本号    日期          作者     
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
extern uint8_t              G_SDCard_FileIsOpen;               //标记是否已经打开文件

/* 串口回调函数 */
static void vUART_GPS_EventHandler(app_uart_evt_t * p_event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //串口收到数据
    if ((p_event->evt_type == APP_UART_DATA_READY) && (G_SDCard_FileIsOpen == 1))
	{
        uint8_t mChar;
        while(app_uart_get(&mChar) == NRF_SUCCESS)
        {
            //若缓存区满了，直接清空之前存储的
            if(G_UART_Buffer1_Counter == 128)
            {
                G_UART_Buffer1_Counter = 0;
                NRF_LOG_INFO("G_Uart_Buffer1 is fulling!");
                NRF_LOG_FLUSH(); 
            }
            
            //存入串口缓存区1中
            G_UART_Buffer1[G_UART_Buffer1_Counter] = mChar;
            G_UART_Buffer1_Counter = G_UART_Buffer1_Counter + 1;
            
            //判断是否收到信息的结尾 是一条完整的语句
             if((G_UART_Buffer1_Counter > 15) && 
                (G_UART_Buffer1[G_UART_Buffer1_Counter-2] == '\r') && 
                (G_UART_Buffer1[G_UART_Buffer1_Counter-1] == '\n')) 
             {                 
                
                memset(G_UART_Buffer2,0,sizeof(G_UART_Buffer2));
                
                memcpy(G_UART_Buffer2,G_UART_Buffer1,G_UART_Buffer1_Counter);
                G_UART_Buffer2_Counter = G_UART_Buffer1_Counter;
                G_UART_Buffer1_Counter = 0;
                //NRF_LOG_INFO("%s",G_UART_Buffer2);
                
                 //通过二进制 信号量 通知 任务进行 协议解析 
                 xSemaphoreGiveFromISR(xSemaphore_GPSBuffer,&xHigherPriorityTaskWoken);
                 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                 
            }
            
        }
        
    }
}

/* GPS串口初始化
 *    返回 0 成功；返回 1 失败  */    
uint8_t ucUARTInital_GPS(void)
{
    uint8_t err_code = 0;
    const app_uart_comm_params_t comm_params =
    {
          configGPIO_UART_GPS_RXD,
          configGPIO_UART_GPS_TXD,
          0,                                //仅在 流控启用才有作用
          0,                                //仅在 流控启用才有作用
          APP_UART_FLOW_CONTROL_DISABLED,   //禁止流控
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600 //波特率115200
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
 * GPS UTC时间转 周内秒  */
void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek)
{ 
/*****协调世界时转换为GPS的周秒表示*****///输入时间应为协调世界时，即当地时间-8，返回时间为GPS周和周秒
    int DayofYear = 0;
    int DayofMonth = 0;


    for(int i = 1980; i < year; i++) //从1980年到当前年的上一年经过的天数
    {
        if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
        DayofYear += 366;
        else
        DayofYear += 365;
    }
    for(int i = 1; i < month; i++)//从一月到当前月的上一月经历的天数
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
    *secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second+18;//18表示跳秒
    return ;
}


///**
// * GPS数据解析  */
//uint8_t ucGPSData_Decode(uint8_t * pData,uint8_t uNumber)
//{
//    enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)pData);   
//    
//                
//}


