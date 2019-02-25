/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_nRF52_GPS     
 * ƽ̨    ��nRF52832 
 * ����    ��ͨ�����ڽ���GPS���ݲ�����  
 * ����    �����ɱ�
**********************************************************************/

#include "Leo_nRF52_GPS.h"


//GPS���ݴ��ڽ��ջ����� 
static struct Leo_gps_buffer    G_GPS_Buffer;               //GPS���ڻ�����

extern uint8_t                  G_GPS_Data[35];
extern uint8_t                  G_GPS_Data_RMCIsValid;
extern uint8_t                  G_GPS_Data_GGAIsValid;

extern uint32_t                 G_GPSWeekSecond;
extern uint16_t                 G_MicroSecond;



static void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek)
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


//@brief ��GPS���λ����� G_GPS_Buffer �� ��ȡһ���ַ�
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*    int8_t* mChar   ��ȡ�����ַ�
//<*����ֵ˵��:
//<*    0����ȡ�ɹ���1����ȡʧ�� �� û������
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


//@brief ��GPS���λ����� G_GPS_Buffer �� ����һ���ַ�
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*����ֵ˵��:
/*--------------------------------------------------------------------------*/
static void Leo_GPS_Buffer_SaveOneByte(int8_t mChar)
{
    *G_GPS_Buffer.pSave = mChar;
    if(G_GPS_Buffer.pSave == G_GPS_Buffer.pEnd)
         G_GPS_Buffer.pSave = G_GPS_Buffer.pStart;
    else
         G_GPS_Buffer.pSave++;
    
    G_GPS_Buffer.Number++;
    //������λ�������� ����֮ǰ������
    if(G_GPS_Buffer.Number == LEO_GPS_BUFFER_MAXLENGTH)
    {
        int8_t mTemp;
        Leo_GPS_Buffer_LoadOneByte(&mTemp);
    }
    
}


//@brief �Դ���G_GPS_Sentence�ڵ�����GPS��� ���н���
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*����ֵ˵��:
//<*    0 �����ɹ�����ȡ����
//<*    1 ���ݽ��������п�����䲻��������ż��У���� �� �����жϴ���
/*--------------------------------------------------------------------------*/
uint8_t Leo_GPS_Decode(void)
{
    //��ȥ������ �������� �ַ�
    int8_t mTemp;
    while((*G_GPS_Buffer.pLoad != '$') && (G_GPS_Buffer.Number > 0))
    {
        Leo_GPS_Buffer_LoadOneByte(&mTemp);     
    }
    
    //�����ж�Ŀǰ�洢���ַ�������̫�پ�ֱ�ӷ�������
    if(G_GPS_Buffer.Number < 40)    
        return 1;
    
    //Ѱ����������䣬������ ������һ������ζ�Ŵ洢�ĵ�һ���ֽڿ϶�Ϊ'$'
    int8_t*     mpLoad = G_GPS_Buffer.pLoad;
    uint16_t    mNumber = 0;
    uint16_t    i = 0;
    uint8_t     checksum = 0x00;            //��ż��У
    
    mpLoad++;   //Խ��'$'
    mNumber++;
    while((*mpLoad != '*') && (mNumber < G_GPS_Buffer.Number))
    {
        checksum ^= *mpLoad;
        mpLoad++;
        mNumber++;
    }
    //û���ҵ� '*' ���� ��䲻����(ȱ�� ��ż��Уλ) 
    if((G_GPS_Buffer.Number-mNumber) < 2) 
        return 1;
    
    //�ҵ� '*' �������������������ż��Уλ
    mpLoad++;   //Խ��'*' ��ȡ��ż��Уλ�ĸ�λ
    mNumber++;
    int16_t     upper = hex2int(*mpLoad);
    if(upper == -1)  //��ż��Уλ���ݶ�ȡ����
    {
        //������ǰmpLoad֮ǰ�������ַ�
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);  
        return 1;
    }    
    
    mpLoad++;   //��ȡ��ż��Уλ�ĵ�λ
    mNumber++;
    int16_t     lower = hex2int(*mpLoad);
    if(lower == -1)  //��ż��Уλ���ݶ�ȡ����
    {
        //������ǰmpLoad֮ǰ�������ַ�
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);         
        return 1;
    } 
    
    //��ż��У���� ��ȡ ��ȷ ���бȶ�
    int expected = (upper << 4) | lower;
    if(checksum != (uint8_t)expected)
    {
        //��ż��У�Բ��ϣ�������ǰ���
        mpLoad++;   
        mNumber++;     
        for(i=0;i<mNumber;i++)
            Leo_GPS_Buffer_LoadOneByte(&mTemp);        
        return 1;
    }
    
    //����������������ȡ����
    mpLoad++;   
    mNumber++;
		//������Ӧ������ mNumber ��̬����ģ����ﲻ֧�֣��ͼ�������GPS�����󳤶Ȳ���83
    int8_t mTempSentence[90] = {0};
    for(i=0;i<mNumber;i++)
        Leo_GPS_Buffer_LoadOneByte(&(mTempSentence[i]));    
    
       
    enum minmea_sentence_id mGPS_Sentence_ID = minmea_sentence_id((char*)mTempSentence);    
    if(mGPS_Sentence_ID == MINMEA_SENTENCE_RMC)
    {
        //RMC��� �����ɹ�
        struct minmea_sentence_rmc mRMC;
        if(minmea_parse_rmc(&mRMC,(char*)mTempSentence))
        {  
            //����Ч���ݴ��� ȫ�ֱ��� G_GPS_Data[34] ��
            if(mRMC.valid == 1)
            {
                //GPRMC ��λ��Ϣ��Ч 
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
                
                //�˴������ ��������ʱ���� ת��Ϊ ������
                UTC2GPS(mRMC.date.year,mRMC.date.month,mRMC.date.day,mRMC.time.hours,mRMC.time.minutes,mRMC.time.seconds,&G_GPSWeekSecond);
                G_GPS_Data_RMCIsValid = 1;
            }           
            //GPSRMC ��λ��Ϣ��Ч 
            return 0;
        }        
    }
    if(mGPS_Sentence_ID == MINMEA_SENTENCE_GGA)
    {
        //GGA��� �����ɹ�
        struct minmea_sentence_gga mGGA;
        if(minmea_parse_gga(&mGGA,(char*)mTempSentence))
        {
            //����Ч���ݴ��� ȫ�ֱ��� G_GPS_Data[34] ��
            if(mGGA.fix_quality ==1 || mGGA.fix_quality == 2)
            {
                //GPGGA ��λ��Ϣ��Ч GPS״̬��0=δ��λ��1=�ǲ�ֶ�λ��2=��ֶ�λ��6=���ڹ���
                //��ȡ���е� �߳���Ϣ ������ʱ����ȡ  ����� �� ������Ч����RMCΪ��
                memcpy(G_GPS_Data+25,&mGGA.altitude.value,4);
                memcpy(G_GPS_Data+29,&mGGA.altitude.scale,4);  
                G_GPS_Data_GGAIsValid = 1;
            }
            //GPGGA ��λ��Ϣ��Ч 
            return 0;
        }
    } 
    
    //������� �Ͳ�������
    return 1; 
}


//@brief GPSͨ�Ŵ��� �ص�����
/*--------------------------------------------------------------------------*/
//<*����˵��:    ��
//<*����ֵ˵��:  ��
/*--------------------------------------------------------------------------*/
static void Leo_UART_Event_Handler(app_uart_evt_t * p_event)
{
    //���������ﲻ֪���᲻�������
    if (p_event->evt_type == APP_UART_DATA_READY)
	{
        //���ڽ��յ�����
        int8_t mChar;
        while(app_uart_get(&mChar) == NRF_SUCCESS)
        {
            Leo_GPS_Buffer_SaveOneByte(mChar);
        }

	}
}




//@brief GPS����ͨ�� ��ʼ��
/*--------------------------------------------------------------------------*/
//<*����˵��:
//<*
//<*����ֵ˵��:
//<*    0:      ��ʼ���ɹ�
//<*    ����:   ʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_nRF52_GPS_Initial(void)
{
    //GPS���ڶ�ȡ ��ʼ��
    uint32_t err_code = 0;
    const app_uart_comm_params_t comm_params =
    {
          Leo_nRF52_GPS_UART_RXD,
          Leo_nRF52_GPS_UART_TXD,
          Leo_nRF52_GPS_UART_RTS,                                //���� �������ò�������
          Leo_nRF52_GPS_UART_CTS,                                //���� �������ò�������
          APP_UART_FLOW_CONTROL_DISABLED,   //��ֹ����
          false,
          UART_BAUDRATE_BAUDRATE_Baud9600 //������115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         512,      
                         512,
                         Leo_UART_Event_Handler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
    
    //��ʼ�� GPS�������ݵ�  ���λ�����
    memset(G_GPS_Buffer.buffer,0,LEO_GPS_BUFFER_MAXLENGTH);
    G_GPS_Buffer.pSave = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pLoad = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pStart = G_GPS_Buffer.buffer;
    G_GPS_Buffer.pEnd = G_GPS_Buffer.buffer + LEO_GPS_BUFFER_MAXLENGTH -1;       
    G_GPS_Buffer.Number = 0;
    return err_code;
}




