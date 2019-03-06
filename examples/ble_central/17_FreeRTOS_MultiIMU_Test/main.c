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



/*========================== 全局变量定义！================================*/
//全局变量_IMU数据采集的 SPI2 实例（UWB 用SPI0；SDCard 用SPI1；多IMU 复用SPI2） 
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	

//全局变量_时间参数 
uint32_t    G_GPSWeekSecond;                   //GPS周内秒数据
uint16_t    G_MicroSecond;                     //nRF52时间计数器控制的 1s的1000计数值，由外部GPS的1PPS校准 1PPS触发时 将其置0
uint8_t	    G_GPSWeekSecond_Data[7];          //用于数据采集时，记录的当时时刻的时间

//全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
uint8_t	    G_MAG_Coeffi[6]; 

//全局变量_IMU_A(U4)和IMU_B(U5)存放的缓存                
uint8_t	    G_IMU_Data_A[24];                   //第一组IMU_A(U4)存放的数据
uint8_t	    G_IMU_Data_B[24];                   //第二组IMU_A(U5)存放的数据
uint8_t	    G_IMUDataA_Counter;                  //MPU9255中断触发的计数器	    
uint8_t	    G_IMUDataB_Counter;


// 全局变量_SDCard存储缓存                                                         
uint8_t	    G_CollectData[512];                 //SDCard要储存数据的缓存
uint16_t    G_CollectData_Counter;  
// 全局变量_SDCard文件操作标识                                                         
uint8_t     G_SDCard_FileIsOpen;               //标记是否已经打开文件 没打开，默认为0



/**
 * 全局变量初始化函数   待完善  
*/
static void vINIT_Variable(void)
{
    //全局变量_时间参数 
    G_GPSWeekSecond     = 0;                    //GPS周内秒数据
    G_MicroSecond       = 0;                    //nRF52时间计数器控制的 1s的1000计数值，
    //数据采集的整秒记录
    memset(G_GPSWeekSecond_Data,0,7);
    G_GPSWeekSecond_Data[0] = 0xA0;
	G_GPSWeekSecond_Data[1] = 0xA0;
    G_GPSWeekSecond_Data[6] = 0xFF;   
    
    //全局变量_IMU_A(U4)和IMU_B(U5)磁强计修正参数
    memset(G_MAG_Coeffi,0,6);
    G_MAG_Coeffi[5] = 0xFF;     
    
    //全局变量_IMU_A(U4)数据
    memset(G_IMU_Data_A,0,24);
    G_IMU_Data_A[0] = 0xB1;
	G_IMU_Data_A[1] = 0xB1;
    G_IMU_Data_A[23] = 0xFF;
    memset(G_IMU_Data_B,0,24);
    G_IMU_Data_B[0] = 0xB2;
	G_IMU_Data_B[1] = 0xB2;
    G_IMU_Data_B[23] = 0xFF;
    G_IMUDataA_Counter = 0;                  //IMU采集的次数计数值	    
    G_IMUDataB_Counter = 0;      
    
    // 全局变量_SDCard存储缓存        
    memset(G_CollectData,0,512);
    G_CollectData_Counter = 0;    
    //全局变量_SDCard文件操作标识 
    G_SDCard_FileIsOpen = 0;                    //标记是否已经打开文件 没打开，默认为0



 
}




int main(void)
{  

    ret_code_t err_code = 0;
    
//0. 初始化_LOG日志输出     
    err_code |= NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),err_code);

//1. 全局变量初始化
    vINIT_Variable();        
    
//2. GPIO管脚初始化    
    err_code |= nrfx_gpiote_init();    
    //(1) LED 管脚 
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    err_code |= nrfx_gpiote_out_init(configGPIO_LED_R,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_LED_R);  //输出1，LED灯灭    
    NRF_LOG_INFO(("||Initialize||-->LED----------->error  0x%x"),err_code);   
    //(2) INT中断管脚初始化    
    err_code |= ucINTInital_SDCard();    /* SDCard中断管脚初始化 */    
    err_code |= ucINTInital_PPS();       /* 1PPS秒脉冲中断管脚初始化 */
    NRF_LOG_INFO(("||Initialize||-->INT----------->error  0x%x"),err_code);     
      
//3. 计时器初始化 
    /* (1) 1ms 计时器 初始化 使用的TIMR3 */   
    err_code |= ucTimerInitial_3();      /* TIMER3 计数器初始化*/ 
    err_code |= ucTimerInitial_4();
    NRF_LOG_INFO(("||Initialize||-->TIMER---------->error  0x%x"),err_code);      
    
//4. 初始化SDCard 并建立存储文件  
    err_code |= ucSDCard_INIT();  
    if(err_code == 0)
    {
        G_SDCard_FileIsOpen = 1;
    }
    NRF_LOG_INFO(("||Initialize||-->SDCard--------->error  0x%x"),err_code);     
   
//5. 初始化 IMU 
//  MPU9250初始化，利用SPI对I2C进行控制时，利用片选管脚总是失败！！，只能重复 SPI init和uint了！！！
//  当前版本仅考虑一个MPU9255(IMU_A) 后面考虑增加MTi和ADIS   
    //(1) 初始化IMU_A的SPI配置 
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_IMUA_nCS;               //第一个IMU的nCS管脚
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//系统SPI中断权限默认设定为 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPI使用的范围为 100KHz~1MHz
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//依据配置参数 对 实例spi 进行初始化 
	err_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    //初始化 IMU_A
    G_MAG_Coeffi[0] = 0xA1;
    G_MAG_Coeffi[1] = 0xA1;
    G_MAG_Coeffi[5] = 0xFF;
    err_code |= ucMPU9255_INIT();    
    NRF_LOG_INFO(("||Initialize||-->IMU_A(U4)------->error  0x%x"),err_code);
    
//6. 初始化 压力传感器的管脚


//7. 初始化 UWB
    


//8. 启动中断和定时器
    //(1)计时器启动
    err_code |= ucTimerStart_3();      /* TIMER3 计数器初始化*/ 
    err_code |= ucTimerStart_4();   
    //(2)中断启动
    ucINTStart_SDCard();
    ucINTStart_PPS();

//9. 判断初始化结果，不正确，则红灯循环闪烁，否则常亮
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
    
//10. 建立任务       
    err_code |= vTask_CreatTask();
    NRF_LOG_INFO(("||Initialize||-->Task_Creat---->error  0x%x"),err_code);
    NRF_LOG_FLUSH();  

//11. 启动_FreeRTOS 任务循环执行  
    vTaskStartScheduler();
    
//12. 任务循环出错  */
    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
    NRF_LOG_FLUSH();
    for (;;)
    {
        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        __WFE();
    }    

}
