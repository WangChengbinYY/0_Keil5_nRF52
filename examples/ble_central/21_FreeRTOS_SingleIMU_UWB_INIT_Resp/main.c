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
#include "Leo_UART.h"
#include "Leo_IMU.h"
#include "Leo_FreeRTOS_TASK.h"
#include "Leo_SAADC.h"
#include "Leo_UWB.h"

/*========================== ȫ�ֱ������壡================================*/
extern uint8_t     G_SDCard_FileIsOpen;  
extern uint8_t	   G_MAG_Coeffi[6]; 





int main(void)
{  
    ret_code_t error_code = 0;
    uint8_t   mTest[10]={0};
//0. ��ʼ��_LOG��־���     
    error_code |= NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),error_code);

//--------����ר��-----------------------  

//---------------------------------------   
   
//1. ȫ�ֱ�����ʼ��
    vINIT_Variable();        
    
//2. GPIO�ܽų�ʼ��    
    error_code |= nrfx_gpiote_init();    
    //(1) LED �ܽ� 
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    error_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //���1��LED����    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),error_code);   
    //(2) INT�жϹܽų�ʼ��    
    error_code |= ucINTInital_SDCard();    /* SDCard�жϹܽų�ʼ�� */    
    error_code |= ucINTInital_PPS();       /* 1PPS�������жϹܽų�ʼ�� */
    error_code |= ucINTInital_UWB();
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),error_code);     




//3. ��ʱ����ʼ�� 
    /* (1) 1ms ��ʱ�� ��ʼ�� ʹ�õ�TIMR3 */   
    error_code |= ucTimerInitial_2();
    error_code |= ucTimerInitial_3();      /* TIMER3 ��������ʼ��*/ 
    error_code |= ucTimerInitial_4();
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),error_code);      
    
//4. ��ʼ��SDCard �������洢�ļ�  
    //error_code |= ucSDCard_INIT();  
    nrf_delay_ms(10);
    //error_code |= ucSDCard_SaveData(mTest,sizeof(mTest));
    if(error_code == 0)
    {
        G_SDCard_FileIsOpen = 1;
    }
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),error_code);     
   
//5. ��ʼ�� IMU 
//    error_code |= ucIMUInitial();
//    NRF_LOG_INFO(("||Initialize||-->IMU------------>error  0x%x"),error_code);      
    
//6. ��ʼ��GPS����
    error_code |= ucUARTInital_GPS();
    NRF_LOG_INFO(("||Initialize||-->GPS_Uart-------->error  0x%x"),error_code); 
    NRF_LOG_FLUSH(); 
    
//7. ��ʼ�� SAADC ѹ��������
    error_code |= ucSAADCInitial();
    NRF_LOG_INFO(("||Initialize||-->SAADC----------->error  0x%x"),error_code); 
    NRF_LOG_FLUSH();   

//8. ��ʼ�� UWB
    error_code |= ucSS_INIT_Initial();
    NRF_LOG_INFO(("||Initialize||-->UWB------------->error  0x%x"),error_code); 
    NRF_LOG_FLUSH();     


//9. �����жϺͶ�ʱ��
    //(1)��ʱ������
    error_code |= ucTimerStart_2();
    error_code |= ucTimerStart_3();      /* TIMER3 ��������ʼ��*/ 
    error_code |= ucTimerStart_4();   
    //(2)�ж�����
    ucINTStart_SDCard();
    ucINTStart_PPS();
    ucINTStart_UWB();
      

//10. ��������       
    error_code |= vTask_CreatTask();
    NRF_LOG_INFO(("||Initialize||-->Task_Creat---->error  0x%x"),error_code);
    NRF_LOG_FLUSH(); 
    
//11.�жϳ�ʼ�����������ȷ������ѭ����˸��������
    if(error_code != 0)
    {
        NRF_LOG_INFO(("||Initialize||-->Initializaiton is Wrong!!->error  0x%x"),error_code);
        NRF_LOG_FLUSH();        
        while(1)
        {
            nrf_delay_ms(150);
            nrfx_gpiote_out_toggle(configGPIO_LED_R);            
        }
    }else{
        NRF_LOG_INFO(("||Initialize||-->Initializaiton is OK!!->error  0x%x"),error_code);
        NRF_LOG_FLUSH();
        nrfx_gpiote_out_clear(configGPIO_LED_R); 
    }

//12. ����_FreeRTOS ����ѭ��ִ��  
    vTaskStartScheduler();
    
//13. ����ѭ������  */
    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
    NRF_LOG_FLUSH();
    for (;;)
    {
        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        __WFE();
    }    

}
