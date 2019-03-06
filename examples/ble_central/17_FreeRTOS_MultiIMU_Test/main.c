/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include "Leo_Includes.h"



/*========================== ȫ�ֱ������壡================================*/
//ȫ�ֱ���_IMU���ݲɼ��� SPI2 ʵ����UWB ��SPI0��SDCard ��SPI1����IMU ����SPI2�� 
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	

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



/**
 * ȫ�ֱ�����ʼ������   ������  
*/
static void vINIT_Variable(void)
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




int main(void)
{  

    ret_code_t err_code = 0;
    
//0. ��ʼ��_LOG��־���     
    err_code |= NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),err_code);

//1. ȫ�ֱ�����ʼ��
    vINIT_Variable();        
    
//2. GPIO�ܽų�ʼ��    
    err_code |= nrfx_gpiote_init();    
    //(1) LED �ܽ� 
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //���1��LED����    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);   
    //(2) INT�жϹܽų�ʼ��    
    err_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    err_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),err_code);     
      
//3. ��ʱ����ʼ�� 
    /* (1) 1ms ��ʱ�� ��ʼ�� ʹ�õ�TIMR3 */   
    err_code |= ucTimerInitial_3();      /* TIMER3 ��������ʼ��*/ 
    err_code |= ucTimerInitial_4();
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),err_code);      
    
//4. ��ʼ��SDCard �������洢�ļ�  
    err_code |= ucSDCard_INIT();  
    if(err_code == 0)
    {
        G_SDCard_FileIsOpen = 1;
    }
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code);     
   
//5. ��ʼ�� IMU 
//  MPU9250��ʼ��������SPI��I2C���п���ʱ������Ƭѡ�ܽ�����ʧ�ܣ�����ֻ���ظ� SPI init��uint�ˣ�����
//  ��ǰ�汾������һ��MPU9255(IMU_A) ���濼������MTi��ADIS   
    //(1) ��ʼ��IMU_A��SPI���� 
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_IMUA_nCS;               //��һ��IMU��nCS�ܽ�
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//ϵͳSPI�ж�Ȩ��Ĭ���趨Ϊ 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPIʹ�õķ�ΧΪ 100KHz~1MHz
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//�������ò��� �� ʵ��spi ���г�ʼ�� 
	err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    //��ʼ�� IMU_A
    G_MAG_Coeffi[0] = 0xA1;
    G_MAG_Coeffi[1] = 0xA1;
    G_MAG_Coeffi[5] = 0xFF;
    err_code |= ucMPU9255_INIT();    
    NRF_LOG_INFO(("||Initialize||-->IMU_A(U4)------->error  0x%x"),err_code);
    
//6. ��ʼ�� ѹ���������Ĺܽ�


//7. ��ʼ�� UWB
    


//8. �����жϺͶ�ʱ��
    //(1)��ʱ������
    err_code |= ucTimerStart_3();      /* TIMER3 ��������ʼ��*/ 
    err_code |= ucTimerStart_4();   
    //(2)�ж�����
    ucINTStart_SDCard();
    ucINTStart_PPS();

//9. �жϳ�ʼ�����������ȷ������ѭ����˸��������
    if(err_code != 0)
    {
        NRF_LOG_INFO(("||Initialize||-->Initializaiton is Wrong!!->error  0x%x"),err_code);
        NRF_LOG_FLUSH();        
        while(1)
        {
            nrf_delay_ms(500);
            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
        }
    }else{
        NRF_LOG_INFO(("||Initialize||-->Initializaiton is OK!!->error  0x%x"),err_code);
        NRF_LOG_FLUSH();
        nrfx_gpiote_out_clear(configGPIO_LED_R); 
    }
    
//10. ��������       
    err_code |= vTask_CreatTask();
    NRF_LOG_INFO(("||Initialize||-->Task_Creat---->error  0x%x"),err_code);
    NRF_LOG_FLUSH();  

//11. ����_FreeRTOS ����ѭ��ִ��  
    vTaskStartScheduler();
    
//12. ����ѭ������  */
    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
    NRF_LOG_FLUSH();
    for (;;)
    {
        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        __WFE();
    }    

}
