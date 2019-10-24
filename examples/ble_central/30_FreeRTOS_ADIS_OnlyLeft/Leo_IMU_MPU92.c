/*
*********************************************************************************************************
*
*    模块名称 : 外部传感器 IMU(MPU9255)
*    文件名称 : Leo_IMU_MPU92
*    版    本 : V1.0
*    说    明 : 外部传感器 IMU(MPU9255)
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-10     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/

#include "Leo_IMU_MPU92.h"



//全局变量_IMU数据采集的 SPI2 实例（UWB 用SPI0；SDCard 用SPI1；多IMU 复用SPI2） 
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	




/**
不用MPU9250，只用ADIS    IMU_A 对应MPU9250  IMU_B 对应ADIS
*/
uint8_t ucIMU_INIT_MPU_ADIS(void)
{
    uint8_t error_code = 0;
    
//(1)直接将IMU_A的片选置高  MPU9250
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUA_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);  //输出1  
    nrf_delay_ms(10);    

//(2)将IMU_B的片选 置低  ADIS 然后初始化SPI接口
    nrfx_gpiote_out_uninit(configGPIO_SPI_IMUB_nCS);    
    
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_IMUB_nCS;               //第一个IMU的nCS管脚
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//系统SPI中断权限默认设定为 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;				
	SPI_config.mode             = NRF_DRV_SPI_MODE_3;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	    

    error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);
    nrf_delay_ms(1);    
    NRF_LOG_INFO(("||Initialize||-->IMU_B_ADIS_SPI--->error  0x%x"),error_code);
    NRF_LOG_FLUSH();     

    return error_code;  
    
}


// ADIS 读取数据
uint8_t Leo_ADIS_Read_ALLData(uint8_t * Data,uint8_t Length)
{
    uint8_t erro_code = 0;
    uint8_t tIMUB_ADIS_ADD[2] = {0};
    tIMUB_ADIS_ADD[0] = 0x68;tIMUB_ADIS_ADD[1] = 0x00;
    
    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tIMUB_ADIS_ADD,2, Data,Length);
    return erro_code;
}

