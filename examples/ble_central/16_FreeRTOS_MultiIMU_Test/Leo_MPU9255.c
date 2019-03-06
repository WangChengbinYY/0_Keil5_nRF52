/*
*********************************************************************************************************
*
*    模块名称 : 外部传感器 MPU9255
*    文件名称 : Leo_MPU9255
*    版    本 : V1.0
*    说    明 : 外部传感器 MPU9255
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-18     Leo
*        V1.1    2019-02-24     Leo
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_MPU9255.h"



extern nrf_drv_spi_t    SPI_CollectData;  			//IMU数据采集使用的SPI实例		
extern uint8_t	    G_CollectData[512];                 //SDCard要储存数据的缓存
extern uint16_t	    G_CollectData_Counter;    
extern uint8_t	    G_MAG_Coeffi[6]; 



//@brief  nRF52<--SPI-->MPU9255	nRF52读取MPU9255寄存器的数据
/*--------------------------------------------------------------------------*/
//<* 	参数说明：
//<*		uint8_t mRegisterAddress		MPU9255寄存器地址
//<*		uint8_t *mData							读取数据存放的地址
//<*		uint8_t mLength							需要读取的字节个数		
//<*	返回值说明：
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它					读取失败
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_SPI_ReadBytes(uint8_t mRegisterAddress, uint8_t *mData,uint8_t mLength )
{
	uint32_t error_code = 0;
	uint8_t i;	
	uint8_t w2_data[8] = {0};	
	uint8_t r2_data[mLength+1];
	uint8_t read_number;
	read_number = mLength + 1;
	// SPI 读取操作需要 在首位 置 1
	w2_data[0] = configRegister_MPU_SPIReadBit|mRegisterAddress;

    //G_MPU9255_SPI_xfer_Done = 0;
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&SPI_CollectData, w2_data,2, r2_data, read_number));	
//	while(G_MPU9255_SPI_xfer_Done == 0)
//		__WFE();	

	if(error_code == NRF_SUCCESS)
	{
			for(i=0;i<mLength;i++)
		{
				mData[i] = r2_data[i+1];
		}
	}
	return error_code;
}


//@brief nRF52<--SPI-->MPU9255	nRF52往MPU9255的寄存器内写数据
/*--------------------------------------------------------------------------*/
//<* 	参数说明：
//<*		uint8_t mRegisterAddress		MPU9255寄存器地址
//<*		uint8_t mData								要写入得 一个 字节
//<*	返回值说明：
//<*		NRF_SUCCESS		写入成功 (0)
//<*		其它					写入失败
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_SPI_WriteOneByte(uint8_t mRegisterAddress, uint8_t mData)
{
	uint32_t error_code = 0;
	uint8_t w2_data[2] = {0};
	uint8_t	r2_data[2] = {0};
	w2_data[0] = mRegisterAddress;
	w2_data[1] = mData;
    
    //G_MPU9255_SPI_xfer_Done = 0;
	error_code = nrf_drv_spi_transfer(&SPI_CollectData, w2_data, 2, r2_data, 2);
//	while(G_MPU9255_SPI_xfer_Done == 0)
//		__WFE();	

	return error_code;	
}



//@brief nRF52<--SPI-->MPU9255  nRF52对MPU9255内 AK8963进行 写操作
/*--------------------------------------------------------------------------*/
//<* 	参数说明：
//<*		uint8_t mRegisterAddress				AK8963寄存器地址
//<*		uint8_t mData										所要写入的数据
//<*	返回值说明：
//<*		NRF_SUCCESS		写入成功 (0)
//<*		1							通过SPI写入AK8963的I2C超时！
//<*		2							通过SPI写入AK8963的I2C 返回一个NACK信号
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_AK8963_SPI_WriteOneByte(uint8_t mRegisterAddress, uint8_t mData)
{
	uint8_t		error_code = 0;
	uint8_t 	status = 0;
	uint16_t	timeout = 0;
	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,MPU9255_AK8963_I2C_ADDR);
	nrf_delay_ms(1);
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,mRegisterAddress);
	nrf_delay_ms(1);
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_DO,mData);
	nrf_delay_ms(1);
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_EN);
	nrf_delay_ms(1);
	
	do{
		if(timeout++ > 200)
			return 1;
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_STATUS,&status,1);
		nrf_delay_ms(1);		
	}while((status & MPU9255_I2C_MST_STATUS_SLV4_DONE) == 0);
	
	if(status & MPU9255_I2C_MST_STATUS_SLV4_NACK)
		return 2;
	
	return error_code;
}


//@brief nRF52<--SPI-->MPU9255  nRF52对MPU9255内 AK8963进行 读操作
/*--------------------------------------------------------------------------*/
//<* 	参数说明：
//<*		uint8_t mRegisterAddress				AK8963寄存器地址
//<*		uint8_t * mData									取的数据
//<*		uint8_t mLength   							所要读取的字节数
//<*	返回值说明：
//<*		NRF_SUCCESS		读取成功 (0)
//<*		1							通过SPI读取AK8963的I2C超时！
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_AK8963_SPI_ReadBytes(uint8_t mRegisterAddress, uint8_t *mData,uint8_t mLength )
{
	uint8_t		error_code = 0;
	uint8_t 	index = 0;
	uint8_t		status = 0;
	uint16_t	timeout = 0;
	uint8_t		tmp = 0;
	
	tmp = MPU9255_AK8963_I2C_ADDR | 0x80;
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,tmp);
	nrf_delay_ms(1);
	while(index < mLength)
	{
		tmp = mRegisterAddress + index;
		error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,tmp);
		nrf_delay_ms(1);
		
		error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_EN);
		nrf_delay_ms(10);			
		
		do{
			if(timeout++ > 50)
				return 1;
			error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_STATUS,&status,1);
			nrf_delay_ms(1);			
		}while((status & MPU9255_I2C_MST_STATUS_SLV4_DONE) == 0);
		
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_DI,mData+index,1);
		nrf_delay_ms(1);
		
		index++;
	}
	
	return error_code;	
}

/*--------------------------------------------------------------------------*/
/********************************对外部接口函数实现***************************/
/*--------------------------------------------------------------------------*/

//@brief nRF52<--SPI-->MPU9255  nRF52对MPU9255 初始化
/*--------------------------------------------------------------------------*/
//<* 	参数说明：
//<*	返回值说明：
//<*		NRF_SUCCESS		初始化成功 (0)
//<*		其它					失败
/*--------------------------------------------------------------------------*/
uint8_t ucMPU9255_INIT(void)
{
	uint8_t error_code = 0;
	uint8_t mAK8963_ID = 0;		
	uint8_t mMPU9255_ID = 0;    
   
	//=====================================设置MPU9255============================================
	//器件重置
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_RESET);//reset
	nrf_delay_ms(50);
		
	//重置器件的 数字信号路径
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SIGNAL_PATH_RESET,MPU9255_SIGNAL_PATH_RESET_GYRO|MPU9255_SIGNAL_PATH_RESET_ACCEL|MPU9255_SIGNAL_PATH_RESET_TEMP); 
	nrf_delay_ms(3);
	
	//配置时钟源
	//Auto selects the best available clock source – PLL if ready, else use the Internal oscillator 此处选用Z陀螺内部时钟 PLL锁相环
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_CLOCK_CLKSEL);  
	nrf_delay_ms(3);

	//设置传感器中断管脚的配置
	//中断状态：[7]高电平有效；[6]引脚上拉；[5]引脚输出50us宽度的脉冲；[4]任何读取操作清除中断状态；
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_PIN,MPU9255_INT_PIN_LATCH_INT_EN|MPU9255_INT_PIN_INT_ANYRD_2CLEAR);	 
	nrf_delay_ms(3);
	
	//设置传感器中断使能配置：有任何新的原始数据产生，都会产生中断
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_ENABLE,MPU9255_INT_ENABLE_RAW_RDY_EN);	
	nrf_delay_ms(3);	
	
	//打开所有陀螺和加计
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_2,MPU9255_PWR_MGMT_2_ACCEL_XYZ|MPU9255_PWR_MGMT_2_GYRO_XYZ);	
	nrf_delay_ms(3);	
	
	//设置陀螺器件参数，包括量程 和 Fchoice_b(用于后面采样频率和低通滤波)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_GYRO_CONFIG,MPU9255_GYRO_CONFIG_FS | MPU9255_GYRO_CONFIG_Fb); 
	nrf_delay_ms(3);	
	
	//设置加速度计参数1:量程
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG1,MPU9255_ACCEL_CONFIG1_FS);
	nrf_delay_ms(3);	
	
	//设置加速度计参数2:DPLF
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG2,MPU9255_ACCEL_CONFIG2_FCHOICE_B|MPU9255_ACCEL_CONFIG2_DLPFCFG);		
	nrf_delay_ms(3);		
	
	//设置MPU9255采样频率
	//1.在前面陀螺参数设置的基础上，设置陀螺 和 温度传感器的原始数据采样频率 和 低通数字滤波参数  Gyro DLPF(0x03(41Hz,5.9ms,1kHz);)) Temp_DLPF(42Hz 4.8ms)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_CONFIG,MPU9255_CONFIG_FIFO_MODE | MPU9255_CONFIG_EXT_SYNC_SET | MPU9255_CONFIG_DLPF_CFG);  
	nrf_delay_ms(3);		
	//2.设置系统输出频率(分频参数)，：SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	//	系统实际原始数据采集1kHz，输出是250Hz <***这里不清楚是取平均输出还是什么？？？***>
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SMPLRT_DIV,MPU9255_SMPLRT_DIV_Rate); 
	nrf_delay_ms(3);		
	
	//=====================================设置AK8963============================================			
	//设置位主I2C模式，便于开始对AK8963进行设置
//	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_MST_EN); 
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,0x30); 
	nrf_delay_ms(50);
	//设置 MPU9255 I2C通信：中断等待；there is a stop between reads；时钟速率 400kHz(这个主机和从机需要一致),
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_CTRL,MPU9255_I2C_MST_CTRL_MULT_MST_EN|MPU9255_I2C_MST_CTRL_WAIT_FOR_ES|MPU9255_I2C_MST_CTRL_I2C_MST_P_NSR|MPU9255_I2C_MST_CTRL_I2C_MST_CLK); 
	nrf_delay_ms(20);	
	
	//reset AK8963 软件复位重启
    //  很奇怪，这里用片选管脚控制时，总会失败，不清楚为什么，初始化只能用 SPI的初始化和uint 
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_RSV,MPU9255_AK8963_RSV_SRST);
	nrf_delay_ms(10);
	
	//POWER_DOWN 模式关闭 ，输出模式为16位
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);

	//保险丝ROM访问模式 ，输出模式为16位，便于首先读出 磁强计数值修正参数
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_FUSE_ROM|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);	
		
	//读取磁强计数值修正参数 
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_ASAX,&G_MAG_Coeffi[2],3);    
	nrf_delay_ms(3);	
    if(error_code == 0)
    {
        memcpy(G_CollectData+G_CollectData_Counter,G_MAG_Coeffi,6);
        G_CollectData_Counter = G_CollectData_Counter + 6;
        NRF_LOG_INFO("		AK8963 Config is ：0x%x %x %x",G_MAG_Coeffi[2],G_MAG_Coeffi[3],G_MAG_Coeffi[4]);
    }        
	//试验 输出读出的 
	//Left的修正参数为：0xAF 0xB2 0xA6  这里要注意，不同的芯片，此处的参数不一样
	//Right的修正参数为：0xAB 0xAC 0xA1  

		
	//读取修正参数完成后，再次 POWER_DOWN
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(3);	
	
	//设置 AK8963输出模式为16位，连续输出模式2 100Hz
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_BIT|MPU9255_AK8963_CNTL1_CONTINU_MEASURE2);
	nrf_delay_ms(3);	
		
	//设置 磁强计 数据采集参数，系统采样频率为250Hz，此处5分频，磁强计采样频率 50Hz  同时关闭SLV4的传输功能
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_REG_DIS|MPU9255_I2C_SLV4_CTRL_MST_DLY);
	nrf_delay_ms(3);		
		
	//设置 磁强计 数据采集延迟控制
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_DELAY_CTRL,MPU9255_I2C_MST_DELAY_CTRL_SHADOW|MPU9255_I2C_MST_DELAY_CTRL_SLV0_EN);
	nrf_delay_ms(3);		
		
	//关闭SLV4的传输功能
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,0x00);
	nrf_delay_ms(3);	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,0x00);
	nrf_delay_ms(3);	

	//设置 SLV0的数据读取功能
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_ADDR,MPU9255_AK8963_I2C_ADDR|0x80);
	nrf_delay_ms(3);		
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_REG,MPU9255_AK8963_ST1);
	nrf_delay_ms(3);		
	//设置 用于传输的外部寄存器和传输的字节长度
	//Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_CTRL,MPU9255_I2C_SLV0_CTRL_EN|MPU9255_I2C_SLV0_CTRL_LENG|MPU9255_I2C_SLV0_CTRL_BYTE_SW|MPU9255_I2C_SLV0_CTRL_REG_DIS|MPU9255_I2C_SLV0_CTRL_GRP);
	nrf_delay_ms(3);		

    //试验读取MPU9255设备ID 0x73 正确返回值为：1
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_WHO_AM_I,&mMPU9255_ID,1); 
	nrf_delay_ms(3);		
	NRF_LOG_INFO("		MPU9255 Device ID_0x73 is ：0x%x",mMPU9255_ID);
	
	//试验读取AK8963设备ID 0x48 正确返回值为：1
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_WIA, &mAK8963_ID,1 );	
	NRF_LOG_INFO("		AK8963 Device ID_0x48 is ：0x%x",mAK8963_ID);
	nrf_delay_ms(3);		
	
	return error_code;
}


//@brief nRF52<--SPI-->MPU9255  nRF52读取MPU9255内 加速度计数据
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*		读取的数据放入全局变量中
//<*	返回值说明:
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它							读取失败
/*--------------------------------------------------------------------------*/
uint8_t Leo_MPU9255_Read_ACC(uint8_t * Data)
{
	uint8_t		error_code = 0;
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, Data, 6);	
	return error_code;
	
	//	uint8_t buf[6] = {0};    		    
//	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, buf, 6);
//  *pACC_X = (buf[0] << 8) | buf[1];
//	if(*pACC_X & 0x8000) *pACC_X-=65536;
//		
//	*pACC_Y= (buf[2] << 8) | buf[3];
//  if(*pACC_Y & 0x8000) *pACC_Y-=65536;
//	
//  *pACC_Z = (buf[4] << 8) | buf[5];
//	if(*pACC_Z & 0x8000) *pACC_Z-=65536;	
}


//@brief nRF52<--SPI-->MPU9255  nRF52读取MPU9255内 陀螺计数据，并判断符号
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*		读取的数据放入全局变量中
//<*	返回值说明:
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它							读取失败
/*--------------------------------------------------------------------------*/
//uint8_t Leo_MPU9255_Read_Gyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
uint8_t Leo_MPU9255_Read_Gyro(uint8_t * Data)
{
	uint8_t		error_code = 0;
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_XOUT_H, Data, 6);
	return error_code;
//  uint8_t buf[6] = {0};    			
//  error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_XOUT_H,  buf, 6);	
//  *pGYRO_X = (buf[0] << 8) | buf[1];
//	if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
//		
//	*pGYRO_Y= (buf[2] << 8) | buf[3];
//  if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
//	
//  *pGYRO_Z = (buf[4] << 8) | buf[5];
//	if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;
}

//@brief nRF52<--SPI-->MPU9255  nRF52读取MPU9255内 陀螺计数据，并判断符号,没有做修正！！！
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*		读取的数据放入全局变量中
//<*	返回值说明:
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它					读取失败
/*--------------------------------------------------------------------------*/
//uint8_t Leo_MPU9255_Read_Magnetic(int16_t *pMAG_X , int16_t *pMAG_Y , int16_t *pMAG_Z)
uint8_t Leo_MPU9255_Read_Magnetic(uint8_t * Data)
{
	uint8_t	error_code = 0;
	uint8_t buf[8] = {0}; 
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_EXT_SENS_DATA_00,buf,8);
	*(Data) = buf[2];
	*(Data+1) = buf[1];
	*(Data+2) = buf[4];
	*(Data+3) = buf[3];
	*(Data+4) = buf[6];
	*(Data+5) = buf[5];	
	return error_code;	
	
	//进行数据整合和符号判断
//	 *pMAG_X = (buf_change[0] << 8) | buf_change[1];
//	if(*pMAG_X & 0x8000) *pMAG_X-=65536;
//		
//	*pMAG_Y= (buf_change[2] << 8) | buf_change[3];
//  if(*pMAG_Y & 0x8000) *pMAG_Y-=65536;
//	
//  *pMAG_Z = (buf_change[4] << 8) | buf_change[5];
//	if(*pMAG_Z & 0x8000) *pMAG_Z-=65536;
}

