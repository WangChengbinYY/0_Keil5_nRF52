/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_nRF52_GPS     
 * 平台    ：nRF52832 
 * 描述    ：通过串口接收GPS数据并解析  
 * 作者    ：王成宾
**********************************************************************/

#include "Leo_nRF52_GPS.h"


//GPS数据串口接收缓存区 
static struct Leo_gps_buffer    G_GPS_Buffer;               //GPS串口缓存区

extern uint8_t                  G_GPS_Data[35];
extern uint8_t                  G_GPS_Data_RMCIsValid;
extern uint8_t                  G_GPS_Data_GGAIsValid;

extern uint32_t                 G_GPSWeekSecond;
extern uint16_t                 G_MicroSecond;



static void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek)
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


//@brief 从GPS环形缓存区 G_GPS_Buffer 中 读取一个字符
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*    int8_t* mChar   读取到的字符
//<*返回值说明:
//<*    0：读取成功；1：读取失败 或 没有数据
/*--------------------------------------------------------------------------*/
static uint8_t Leo_GPS_Buffer_LoadOneByte(int8_t* mChar)
{
    if(G_GPS_Buffer.Number == 0)
        return 1;
    
    *mChar = *G_GPS_Buffer.pLoad;
    if(G_GPS_Buffer.pLoad == G_GPS_Buffer.pEnd)
        G_GPS_Buffer.pLoad = G_GPS_Buffer.pStart;
    else
        G_GPS_Buffer.pLoad++;
    G_GPS_Buffer.Number--;
    return 0;
}


//@brief 往GPS环形缓存区 G_GPS_Buffer 中 存入一个字符
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*返回值说明:
/*--------------------------------------------------------------------------*/
static void Leo_GPS_Buffer_SaveOneByte(int8_t mChar)
{
    *G_GPS_Buffer.pSave = mChar;
    if(G_GPS_Buffer.pSave == G_GPS_Buffer.pEnd)
         G_GPS_Buffer.pSave = G_GPS_Buffer.pStart;
    else
         G_GPS_Buffer.pSave++;
    
    G_GPS_Buffer.Number++;
    //如果环形缓存区溢出 覆盖之前的数据
    if(G_GPS_Buffer.Number == LEO_GPS_BUFFER_MAXLENGTH)
    {
        int8_t mTemp;
        Leo_GPS_Buffer_LoadOneByte(&mTemp);
    }
    
}


//@brief 对存入G_GPS_Sentence内的完整GPS语句 进行解析
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*返回值说明:
//<*    0 解析成功，获取数据
//<*    1 数据解析错误，有可能语句不完整、奇偶检校错误 或 数据判断错误！
/*--------------------------------------------------------------------------*/
uint8_t Leo_GPS_Decode(void)
{
    //出去缓存内 不完整的 字符
    int8_t mTemp;
    while((*G_GPS_Buffer.pLoad != '$') && (G_GPS_Buffer.Number > 0))
    {
        Leo_GPS_Buffer_LoadOneByte(&mTemp);     
    }
    
    //初步判断目前存储的字符个数，太少就直接放弃解析
    if(G_GPS_Buffer.Number < 40)    
        return 1;
    
    //寻找完整的语句，并解析 进入这一步，意味着存储的第一个字节肯定为'$'
    int8_t*     mpLoad = G_GPS_Buffer.pLoad;
    uint16_t    mNumber = 0;
    uint16_t    i = 0;
    uint8_t     checksum = 0x00;            //奇偶检校
    
    mpLoad++;   //越过'$'
    mNumber++;
    while((*mpLoad != '*') && (mNumber < G_GPS_Buffer.Number))
    {
        checksum ^= *mpLoad;
        mpLoad++;
        mNumber++;
    }
    //没有找到 '*' 或者 语句不完整(缺少 奇偶检校位) 
    if((G_GPS_Buffer.Number-mNumber) < 2) 
        return 1;
    
    //找到 '*' 并且语句完整，读到奇偶检校位
    mpLoad++;   //越过'*' 读取奇偶检校位的高位
    mNumber++;
    int16_t     upper = hex2int(*mpLoad);
    if(upper == -1)  //奇偶检校位数据读取错误
    {
        //放弃当前mpLoad之前的所有字符
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);  
        return 1;
    }    
    
    mpLoad++;   //读取奇偶检校位的低位
    mNumber++;
    int16_t     lower = hex2int(*mpLoad);
    if(lower == -1)  //奇偶检校位数据读取错误
    {
        //放弃当前mpLoad之前的所有字符
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);         
        return 1;
    } 
    
    //奇偶检校数据 读取 正确 进行比对
    int expected = (upper << 4) | lower;
    if(checksum != (uint8_t)expected)
    {
        //奇偶检校对不上，放弃当前语句
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);        
        return 1;
    }
    
    //将整条完整的语句读取出来
    mpLoad++;   
    mNumber++;
		//本来是应该依据 mNumber 动态分配的，这里不支持，就简单声明，GPS语句最大长度不超83
    int8_t mTempSentence[90] = {0};
    for(i=0;i<mNumber;i++)
        Leo_GPS_Buffer_LoadOneByte(&(mTempSentence[i]));    
    
       
    enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mTempSentence);    
    if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
    {
        //RMC语句 解析成功
        struct minmea_sentence_rmc mRMC;
        if(minmea_parse_rmc(&mRMC,(char*)mTempSentence))
        {  
            //将有效数据存入 全局变量 G_GPS_Data[34] 中
            if(mRMC.valid == 1)
            {
                //GPRMC 定位信息有效 
                G_GPS_Data[2] = mRMC.date.month;
                G_GPS_Data[3] = mRMC.date.day;
                G_GPS_Data[4] = mRMC.time.hours;
                G_GPS_Data[5] = mRMC.time.minutes;
                G_GPS_Data[6] = mRMC.time.seconds;
                memcpy(G_GPS_Data+7,&G_MicroSecond,2);
                memcpy(G_GPS_Data+9,&mRMC.longitude.value,4);
                memcpy(G_GPS_Data+13,&mRMC.longitude.scale,4);
                memcpy(G_GPS_Data+17,&mRMC.latitude.value,4);
                memcpy(G_GPS_Data+21,&mRMC.latitude.scale,4);    
                
                //此处待添加 将年月日时分秒 转化为 周内秒
                UTC2GPS(mRMC.date.year,mRMC.date.month,mRMC.date.day,mRMC.time.hours,mRMC.time.minutes,mRMC.time.seconds,&G_GPSWeekSecond);
                G_GPS_Data_RMCIsValid = 1;
            }           
            //GPSRMC 定位信息无效 
            return 0;
        }        
    }
    if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
    {
        //GGA语句 解析成功
        struct minmea_sentence_gga mGGA;
        if(minmea_parse_gga(&mGGA,(char*)mTempSentence))
        {
            //将有效数据存入 全局变量 G_GPS_Data[34] 中
            if(mGGA.fix_quality ==1 || mGGA.fix_quality == 2)
            {
                //GPGGA 定位信息有效 GPS状态：0=未定位，1=非差分定位，2=差分定位，6=正在估算
                //读取其中的 高程信息 其它暂时不读取  这个不 标 数据有效，以RMC为主
                memcpy(G_GPS_Data+25,&mGGA.altitude.value,4);
                memcpy(G_GPS_Data+29,&mGGA.altitude.scale,4);  
                G_GPS_Data_GGAIsValid = 1;
            }
            //GPGGA 定位信息无效 
            return 0;
        }
    } 
    
    //其它语句 就不解析了
    return 1; 
}


//@brief GPS通信串口 回调函数
/*--------------------------------------------------------------------------*/
//<*参数说明:    无
//<*返回值说明:  无
/*--------------------------------------------------------------------------*/
static void Leo_UART_Event_Handler(app_uart_evt_t * p_event)
{
    //解析放这里不知道会不会出问题
    if (p_event->evt_type == APP_UART_DATA_READY)
	{
        //串口接收到数据
        int8_t mChar;
        while(app_uart_get(&mChar) == NRF_SUCCESS)
        {
            Leo_GPS_Buffer_SaveOneByte(mChar);
        }

	}
}




//@brief GPS串口通信 初始化
/*--------------------------------------------------------------------------*/
//<*参数说明:
//<*
//<*返回值说明:
//<*    0:      初始化成功
//<*    其它:   失败
/*--------------------------------------------------------------------------*/
uint8_t Leo_nRF52_GPS_Initial(void)
{
    //GPS串口读取 初始化
    uint32_t err_code = 0;
    const app_uart_comm_params_t comm_params =
    {
          Leo_nRF52_GPS_UART_RXD,
          Leo_nRF52_GPS_UART_TXD,
          Leo_nRF52_GPS_UART_RTS,                                //仅在 流控启用才有作用
          Leo_nRF52_GPS_UART_CTS,                                //仅在 流控启用才有作用
          APP_UART_FLOW_CONTROL_DISABLED,   //禁止流控
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600 //波特率115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         512,      
                         512,
                         Leo_UART_Event_Handler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
    
    //初始化 GPS接收数据的  环形缓冲区
    memset(G_GPS_Buffer.buffer,0,LEO_GPS_BUFFER_MAXLENGTH);
    G_GPS_Buffer.pSave = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pLoad = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pStart = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pEnd = G_GPS_Buffer.buffer + LEO_GPS_BUFFER_MAXLENGTH -1;       
    G_GPS_Buffer.Number = 0;
    return err_code;
}




