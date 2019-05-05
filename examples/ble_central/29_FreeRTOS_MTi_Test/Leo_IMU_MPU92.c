/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ������ IMU(MPU9255)
*    �ļ����� : Leo_IMU_MPU92
*    ��    �� : V1.0
*    ˵    �� : �ⲿ������ IMU(MPU9255)
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-10     WangCb   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/

#include "Leo_IMU_MPU92.h"



//ȫ�ֱ���_IMU���ݲɼ��� SPI2 ʵ����UWB ��SPI0��SDCard ��SPI1����IMU ����SPI2�� 
nrf_drv_spi_t   SPI_CollectData = NRF_DRV_SPI_INSTANCE(configGPIO_SPI_CollectData_INSTANCE);	

extern uint8_t	        G_MAG_Coeffi[6]; 



//@brief  nRF52<--SPI-->MPU9255	nRF52��ȡMPU9255�Ĵ���������
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*		uint8_t mRegisterAddress		MPU9255�Ĵ�����ַ
//<*		uint8_t *mData							��ȡ���ݴ�ŵĵ�ַ
//<*		uint8_t mLength							��Ҫ��ȡ���ֽڸ���		
//<*	����ֵ˵����
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_SPI_ReadBytes(uint8_t mRegisterAddress, uint8_t *mData,uint8_t mLength )
{
	uint32_t error_code = 0;
	uint8_t i;	
	uint8_t w2_data[8] = {0};	
	uint8_t r2_data[mLength+1];
	uint8_t read_number;
	read_number = mLength + 1;
	// SPI ��ȡ������Ҫ ����λ �� 1
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


//@brief nRF52<--SPI-->MPU9255	nRF52��MPU9255�ļĴ�����д����
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*		uint8_t mRegisterAddress		MPU9255�Ĵ�����ַ
//<*		uint8_t mData								Ҫд��� һ�� �ֽ�
//<*	����ֵ˵����
//<*		NRF_SUCCESS		д��ɹ� (0)
//<*		����					д��ʧ��
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



//@brief nRF52<--SPI-->MPU9255  nRF52��MPU9255�� AK8963���� д����
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*		uint8_t mRegisterAddress				AK8963�Ĵ�����ַ
//<*		uint8_t mData										��Ҫд�������
//<*	����ֵ˵����
//<*		NRF_SUCCESS		д��ɹ� (0)
//<*		1							ͨ��SPIд��AK8963��I2C��ʱ��
//<*		2							ͨ��SPIд��AK8963��I2C ����һ��NACK�ź�
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_AK8963_SPI_WriteOneByte(uint8_t mRegisterAddress, uint8_t mData)
{
	uint8_t		error_code = 0;
	uint8_t 	status = 0;
	uint16_t	timeout = 0;
	
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,MPU9255_AK8963_I2C_ADDR);
	nrf_delay_ms(1);
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,mRegisterAddress);
	nrf_delay_ms(1);
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_DO,mData);
	nrf_delay_ms(1);
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_EN);
	nrf_delay_ms(1);
	
	do{
		if(timeout++ > 200)
			return 1;
		error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_STATUS,&status,1);
		nrf_delay_ms(1);		
	}while((status & MPU9255_I2C_MST_STATUS_SLV4_DONE) == 0);
	
	if(status & MPU9255_I2C_MST_STATUS_SLV4_NACK)
		return 2;
	
	return error_code;
}


//@brief nRF52<--SPI-->MPU9255  nRF52��MPU9255�� AK8963���� ������
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*		uint8_t mRegisterAddress				AK8963�Ĵ�����ַ
//<*		uint8_t * mData									ȡ������
//<*		uint8_t mLength   							��Ҫ��ȡ���ֽ���
//<*	����ֵ˵����
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		1							ͨ��SPI��ȡAK8963��I2C��ʱ��
/*--------------------------------------------------------------------------*/
static uint8_t Leo_MPU9255_AK8963_SPI_ReadBytes(uint8_t mRegisterAddress, uint8_t *mData,uint8_t mLength )
{
	uint8_t		error_code = 0;
	uint8_t 	index = 0;
	uint8_t		status = 0;
	uint16_t	timeout = 0;
	uint8_t		tmp = 0;
	
	tmp = MPU9255_AK8963_I2C_ADDR | 0x80;
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,tmp);
	nrf_delay_ms(1);
	while(index < mLength)
	{
		tmp = mRegisterAddress + index;
		error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,tmp);
		nrf_delay_ms(1);
		
		error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_EN);
		nrf_delay_ms(10);			
		
		do{
			if(timeout++ > 50)
				return 1;
			error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_STATUS,&status,1);
			nrf_delay_ms(1);			
		}while((status & MPU9255_I2C_MST_STATUS_SLV4_DONE) == 0);
		
		error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_DI,mData+index,1);
		nrf_delay_ms(1);
		
		index++;
	}
	
	return error_code;	
}

/*--------------------------------------------------------------------------*/
/********************************���ⲿ�ӿں���ʵ��***************************/
/*--------------------------------------------------------------------------*/

//@brief nRF52<--SPI-->MPU9255  nRF52��MPU9255 ��ʼ��
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*	����ֵ˵����
//<*		NRF_SUCCESS		��ʼ���ɹ� (0)
//<*		����					ʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_MPU9255_INIT(void)
{
	uint8_t error_code = 0;
	uint8_t mAK8963_ID = 0;		
	uint8_t mMPU9255_ID = 0;    
   
	//=====================================����MPU9255============================================
	//��������
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_RESET);//reset
	nrf_delay_ms(50);
		
	//���������� �����ź�·��
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_SIGNAL_PATH_RESET,MPU9255_SIGNAL_PATH_RESET_GYRO|MPU9255_SIGNAL_PATH_RESET_ACCEL|MPU9255_SIGNAL_PATH_RESET_TEMP); 
	nrf_delay_ms(3);
	
	//����ʱ��Դ
	//Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator �˴�ѡ��Z�����ڲ�ʱ�� PLL���໷
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_CLOCK_CLKSEL);  
	nrf_delay_ms(3);

	//���ô������жϹܽŵ�����
	//�ж�״̬��[7]�ߵ�ƽ��Ч��[6]����������[5]�������50us��ȵ����壻[4]�κζ�ȡ��������ж�״̬��
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_PIN,MPU9255_INT_PIN_LATCH_INT_EN|MPU9255_INT_PIN_INT_ANYRD_2CLEAR);	 
	nrf_delay_ms(3);
	
	//���ô������ж�ʹ�����ã����κ��µ�ԭʼ���ݲ�������������ж�
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_ENABLE,MPU9255_INT_ENABLE_RAW_RDY_EN);	
	nrf_delay_ms(3);	
	
	//���������ݺͼӼ�
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_2,MPU9255_PWR_MGMT_2_ACCEL_XYZ|MPU9255_PWR_MGMT_2_GYRO_XYZ);	
	nrf_delay_ms(3);	
	
	//�������������������������� �� Fchoice_b(���ں������Ƶ�ʺ͵�ͨ�˲�)
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_GYRO_CONFIG,MPU9255_GYRO_CONFIG_FS | MPU9255_GYRO_CONFIG_Fb); 
	nrf_delay_ms(3);	
	
	//���ü��ٶȼƲ���1:����
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG1,MPU9255_ACCEL_CONFIG1_FS);
	nrf_delay_ms(3);	
	
	//���ü��ٶȼƲ���2:DPLF
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG2,MPU9255_ACCEL_CONFIG2_FCHOICE_B|MPU9255_ACCEL_CONFIG2_DLPFCFG);		
	nrf_delay_ms(3);		
	
	//����MPU9255����Ƶ��
	//1.��ǰ�����ݲ������õĻ����ϣ��������� �� �¶ȴ�������ԭʼ���ݲ���Ƶ�� �� ��ͨ�����˲�����  Gyro DLPF(0x03(41Hz,5.9ms,1kHz);)) Temp_DLPF(42Hz 4.8ms)
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_CONFIG,MPU9255_CONFIG_FIFO_MODE | MPU9255_CONFIG_EXT_SYNC_SET | MPU9255_CONFIG_DLPF_CFG);  
	nrf_delay_ms(3);		
	//2.����ϵͳ���Ƶ��(��Ƶ����)����SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	//	ϵͳʵ��ԭʼ���ݲɼ�1kHz�������200Hz <***���ﲻ�����ȡƽ���������ʲô������***>
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_SMPLRT_DIV,MPU9255_SMPLRT_DIV_Rate); 
	nrf_delay_ms(30);		
	
	//=====================================����AK8963============================================			
	//����λ��I2Cģʽ�����ڿ�ʼ��AK8963��������
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_MST_EN); 
//	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,0x30); 
	nrf_delay_ms(50);
	//���� MPU9255 I2Cͨ�ţ��жϵȴ���there is a stop between reads��ʱ������ 400kHz(��������ʹӻ���Ҫһ��),
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_CTRL,MPU9255_I2C_MST_CTRL_MULT_MST_EN|MPU9255_I2C_MST_CTRL_WAIT_FOR_ES|MPU9255_I2C_MST_CTRL_I2C_MST_P_NSR|MPU9255_I2C_MST_CTRL_I2C_MST_CLK); 
	nrf_delay_ms(20);	
	
	//reset AK8963 �����λ����
    //  ����֣�������Ƭѡ�ܽſ���ʱ���ܻ�ʧ�ܣ������Ϊʲô����ʼ��ֻ���� SPI�ĳ�ʼ����uint 
	error_code |=Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_RSV,MPU9255_AK8963_RSV_SRST);
	nrf_delay_ms(10);
	
	//POWER_DOWN ģʽ�ر� �����ģʽΪ16λ
	error_code |=Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);

	//����˿ROM����ģʽ �����ģʽΪ16λ���������ȶ��� ��ǿ����ֵ��������
	error_code |=Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_FUSE_ROM|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);	
		
	//��ȡ��ǿ����ֵ�������� 
	error_code |=Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_ASAX,&G_MAG_Coeffi[2],3);    
	nrf_delay_ms(3);	
    if(error_code == 0)
    {
        NRF_LOG_INFO("		AK8963 Config is:0x%x %x %x",G_MAG_Coeffi[2],G_MAG_Coeffi[3],G_MAG_Coeffi[4]);
        NRF_LOG_FLUSH();
    }        
	//���� ��������� 
	//Left����������Ϊ��0xAF 0xB2 0xA6  ����Ҫע�⣬��ͬ��оƬ���˴��Ĳ�����һ��
	//Right����������Ϊ��0xAB 0xAC 0xA1  

		
	//��ȡ����������ɺ��ٴ� POWER_DOWN
	error_code |=Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(3);	
	
	//���� AK8963���ģʽΪ16λ���������ģʽ2 100Hz
	error_code |=Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_BIT|MPU9255_AK8963_CNTL1_CONTINU_MEASURE2);
	nrf_delay_ms(3);	
		
	//���� ��ǿ�� ���ݲɼ�������ϵͳ����Ƶ��Ϊ200Hz���˴�2��Ƶ����ǿ�Ʋ���Ƶ�� 100Hz  ͬʱ�ر�SLV4�Ĵ��书��
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_REG_DIS|MPU9255_I2C_SLV4_CTRL_MST_DLY);
	nrf_delay_ms(3);		
		
	//���� ��ǿ�� ���ݲɼ��ӳٿ���
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_DELAY_CTRL,MPU9255_I2C_MST_DELAY_CTRL_SHADOW|MPU9255_I2C_MST_DELAY_CTRL_SLV0_EN);
	nrf_delay_ms(3);		
		
	//�ر�SLV4�Ĵ��书��
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,0x00);
	nrf_delay_ms(3);	
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,0x00);
	nrf_delay_ms(3);	

	//���� SLV0�����ݶ�ȡ����
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_ADDR,MPU9255_AK8963_I2C_ADDR|0x80);
	nrf_delay_ms(3);		
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_REG,MPU9255_AK8963_ST1);
	nrf_delay_ms(3);		
	//���� ���ڴ�����ⲿ�Ĵ����ʹ�����ֽڳ���
	//Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA	
	error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_CTRL,MPU9255_I2C_SLV0_CTRL_EN|MPU9255_I2C_SLV0_CTRL_LENG|MPU9255_I2C_SLV0_CTRL_BYTE_SW|MPU9255_I2C_SLV0_CTRL_REG_DIS|MPU9255_I2C_SLV0_CTRL_GRP);
	nrf_delay_ms(3);		

    //�����ȡMPU9255�豸ID 0x73 ��ȷ����ֵΪ��1
	error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_WHO_AM_I,&mMPU9255_ID,1); 
	nrf_delay_ms(3);		
	NRF_LOG_INFO("		MPU9255 Device ID_0x73 is:0x%x",mMPU9255_ID);
	
	//�����ȡAK8963�豸ID 0x48 ��ȷ����ֵΪ��1
	error_code |=Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_WIA, &mAK8963_ID,1 );	
	NRF_LOG_INFO("		AK8963 Device ID_0x48 is:0x%x",mAK8963_ID);
	nrf_delay_ms(3);		
	
    
    //MPU9255 ˯���ɣ�������
    error_code |=Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,0x40);//reset
	nrf_delay_ms(50);
    
	return error_code;
}


//@brief nRF52<--SPI-->MPU9255  nRF52��ȡMPU9255�� ���ٶȼ�����
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		��ȡ�����ݷ���ȫ�ֱ�����
//<*	����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����							��ȡʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_MPU9255_Read_ACC(uint8_t * Data)
{
	uint8_t		error_code = 0;
	error_code =Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, Data, 6);	
	return error_code;
	
	//	uint8_t buf[6] = {0};    		    
//	error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, buf, 6);
//  *pACC_X = (buf[0] << 8) | buf[1];
//	if(*pACC_X & 0x8000) *pACC_X-=65536;
//		
//	*pACC_Y= (buf[2] << 8) | buf[3];
//  if(*pACC_Y & 0x8000) *pACC_Y-=65536;
//	
//  *pACC_Z = (buf[4] << 8) | buf[5];
//	if(*pACC_Z & 0x8000) *pACC_Z-=65536;	
}


//@brief nRF52<--SPI-->MPU9255  nRF52��ȡMPU9255�� ���ݼ����ݣ����жϷ���
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		��ȡ�����ݷ���ȫ�ֱ�����
//<*	����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����							��ȡʧ��
/*--------------------------------------------------------------------------*/
//uint8_tLeo_MPU9255_Read_Gyro(int16_t *pGYRO_X , int16_t *pGYRO_Y , int16_t *pGYRO_Z )
uint8_t Leo_MPU9255_Read_Gyro(uint8_t * Data)
{
	uint8_t		error_code = 0;
	error_code =Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_XOUT_H, Data, 6);
	return error_code;
//  uint8_t buf[6] = {0};    			
//  error_code |=Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_XOUT_H,  buf, 6);	
//  *pGYRO_X = (buf[0] << 8) | buf[1];
//	if(*pGYRO_X & 0x8000) *pGYRO_X-=65536;
//		
//	*pGYRO_Y= (buf[2] << 8) | buf[3];
//  if(*pGYRO_Y & 0x8000) *pGYRO_Y-=65536;
//	
//  *pGYRO_Z = (buf[4] << 8) | buf[5];
//	if(*pGYRO_Z & 0x8000) *pGYRO_Z-=65536;
}

//@brief nRF52<--SPI-->MPU9255  nRF52��ȡMPU9255�� ���ݼ����ݣ����жϷ���,û��������������
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		��ȡ�����ݷ���ȫ�ֱ�����
//<*	����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
/*--------------------------------------------------------------------------*/
//uint8_tLeo_MPU9255_Read_Magnetic(int16_t *pMAG_X , int16_t *pMAG_Y , int16_t *pMAG_Z)
uint8_t Leo_MPU9255_Read_Magnetic(uint8_t * Data)
{
	uint8_t	error_code = 0;
	uint8_t buf[8] = {0}; 
	error_code =Leo_MPU9255_SPI_ReadBytes(MPU9255_EXT_SENS_DATA_00,buf,8);
	*(Data) = buf[2];
	*(Data+1) = buf[1];
	*(Data+2) = buf[4];
	*(Data+3) = buf[3];
	*(Data+4) = buf[6];
	*(Data+5) = buf[5];	
	return error_code;	
	
	//�����������Ϻͷ����ж�
//	 *pMAG_X = (buf_change[0] << 8) | buf_change[1];
//	if(*pMAG_X & 0x8000) *pMAG_X-=65536;
//		
//	*pMAG_Y= (buf_change[2] << 8) | buf_change[3];
//  if(*pMAG_Y & 0x8000) *pMAG_Y-=65536;
//	
//  *pMAG_Z = (buf_change[4] << 8) | buf_change[5];
//	if(*pMAG_Z & 0x8000) *pMAG_Z-=65536;
}

uint8_t Leo_MPU9255_Read_ALLData(uint8_t * Data)
{
	uint8_t		error_code = 0;
	error_code =Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, Data,22);	
	return error_code;
}



/**
 * IMU ��ʼ�� MPU92
 *   ���� �ܽŹ���ʽ����ʼ�� IMU_A �� IMU_B
*/
uint8_t ucIMU_INIT_MPU_ADIS(void)
{
    uint8_t error_code = 0;
    //���⣺MPU9250��ʼ��������SPI��I2C���п���ʱ������Ƭѡ�ܽ�����ʧ�ܣ�����ֻ���ظ� SPI init��uint�ˣ�����  
    //      ADIS Ҳֻ�� �ظ� SPI init��uint ����ʼ���������޷�д��Ĵ�������ְ���
    
//(1) ��ʼ��IMU_A��SPI���� 
    //��ʼ��IMU_B��Ƭѡ�� ���ø�
    nrfx_gpiote_out_config_t tconfigGPIO_OUT =  NRFX_GPIOTE_CONFIG_OUT_SIMPLE(true);
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUB_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUB_nCS);       
    nrf_delay_ms(1);    
   
    //��ʼ�� IMU_A �� SPI
    nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= configGPIO_SPI_CollectData_SCK;
	SPI_config.mosi_pin 		= configGPIO_SPI_CollectData_MOSI;
	SPI_config.miso_pin 		= configGPIO_SPI_CollectData_MISO;   
    SPI_config.ss_pin			= configGPIO_SPI_IMUA_nCS;               //��һ��IMU��nCS�ܽ�
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//ϵͳSPI�ж�Ȩ��Ĭ���趨Ϊ 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//MPU9255 SPIʹ�õķ�ΧΪ 100KHz~1MHz
	SPI_config.mode             = NRF_DRV_SPI_MODE_3;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;	
	//�������ò��� �� ʵ��spi ���г�ʼ�� 
	error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	
    //��ʼ�� IMU_A
    G_MAG_Coeffi[0] = 0xA1;
    G_MAG_Coeffi[1] = 0xA1;
    G_MAG_Coeffi[5] = 0xFF;
    //error_code |= Leo_MPU9255_INIT();    
    //ucCircleBuffer_SaveData(G_MAG_Coeffi,sizeof(G_MAG_Coeffi));  
    NRF_LOG_INFO(("||Initialize||-->IMU_A(U4)------->error  0x%x"),error_code);
    //ж��SPI
    nrf_drv_spi_uninit(&SPI_CollectData);
    
    //��ʼ��IMU_A��Ƭѡ�� ���ø�
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUA_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUA_nCS);  //���1  
    nrf_delay_ms(1);
    
//(2)��ʼ�� IMU_B    
#if configIMU_MPU92_MPU92           //����ʹ������ MPU92 
    //�ָ� IMU_B nCS�ܽţ����趨 SPI
    nrfx_gpiote_out_uninit(configGPIO_SPI_IMUB_nCS);
    SPI_config.ss_pin = configGPIO_SPI_IMUB_nCS;               //�ڶ���IMU��nCS�ܽ�
    error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);
    //��ʼ�� IMU_B
    G_MAG_Coeffi[0] = 0xA2;
    G_MAG_Coeffi[1] = 0xA2;
    G_MAG_Coeffi[5] = 0xFF;
    error_code |= Leo_MPU9255_INIT();  
    ucCircleBuffer_SaveData(G_MAG_Coeffi,sizeof(G_MAG_Coeffi));     
    NRF_LOG_INFO(("||Initialize||-->IMU_B----------->error  0x%x"),error_code);
    NRF_LOG_FLUSH();
    //ж��SPI
    nrf_drv_spi_uninit(&SPI_CollectData);
    //�ر� IMU_B nCS �ܽ�    
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUB_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUB_nCS);  //���1          
    nrf_delay_us(1);   
#endif    

#if configIMU_MPU92_ADIS
    nrfx_gpiote_out_uninit(configGPIO_SPI_IMUB_nCS);
    SPI_config.ss_pin = configGPIO_SPI_IMUB_nCS; 
    error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);
    nrf_delay_ms(1);
    
    //д ���REST
//    uint8_t tRST_LSB[2] = {0};    
//    tRST_LSB[0] = 0xE8;tRST_LSB[1] = 0x80;
//    uint8_t tRST_MSB[2] = {0};    
//    tRST_MSB[0] = 0xE9;tRST_MSB[1] = 0x00;
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tRST_LSB,2,NULL,0);     
//    nrf_delay_ms(1);
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tRST_MSB,2,NULL,0);
//    nrf_delay_ms(500);
    
    //д ���ò���Ƶ�� 200 Hz
    uint8_t tHzLSB[2] = {0};
    uint8_t tHzMSB[2] = {0};
    tHzLSB[0] = 0xE4;tHzLSB[1] = 0x09; 
    tHzMSB[0] = 0xE5;tHzMSB[1] = 0x00;
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tHzLSB,2, NULL,2);	
    nrf_delay_ms(1);
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tHzMSB,2, NULL,2);	
    nrf_delay_ms(10);      
//    
//    
//    //д �洢���� ��������    
    uint8_t tGLOB_HzLSB[2] = {0};
    uint8_t tGLOB_HzMSB[2] = {0};
    tGLOB_HzLSB[0] = 0xE8;tGLOB_HzLSB[1] = 0x08; 
    tGLOB_HzMSB[0] = 0xE9;tGLOB_HzMSB[1] = 0x00;
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tGLOB_HzLSB,2, NULL,2);	
    nrf_delay_ms(1);
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tGLOB_HzMSB,2, NULL,2);	
    nrf_delay_ms(500);     

    //��ȡ״̬�Ĵ���
    uint8_t tState_ADD[2] = {0};
    uint8_t tState_Data[2] = {0};
    tState_ADD[0] = 0x02;   tState_ADD[1] = 0x00; 
    tState_Data[0] = 0x55;  tState_Data[1] = 0x66; 
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tState_ADD,2, tState_Data,2);	
    nrf_delay_ms(1);
    NRF_LOG_INFO(("%d  tState_ADD  0x%x %x"),error_code,tState_Data[0],tState_Data[1]);
    NRF_LOG_FLUSH();      
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tState_ADD,2, tState_Data,2);	
    nrf_delay_ms(1);
    NRF_LOG_INFO(("%d  tState_ADD  0x%x %x"),error_code,tState_Data[0],tState_Data[1]);
    NRF_LOG_FLUSH();      
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tState_ADD,2, tState_Data,2);	
    nrf_delay_ms(1);
    NRF_LOG_INFO(("%d  tState_ADD  0x%x %x"),error_code,tState_Data[0],tState_Data[1]);
    NRF_LOG_FLUSH();      
    
    //��ȡ����
    uint8_t tPID_ADD[2] = {0};
    uint8_t tPID_Data[2] = {0};
    tPID_ADD[0] = 0x72;tPID_ADD[1] = 0x00; 
    tPID_Data[0] = 0x55;tPID_Data[1] = 0x66; 
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tPID_ADD,2, tPID_Data,2);	
    nrf_delay_ms(1);
    NRF_LOG_INFO(("%d  tPID  0x%x %x"),error_code,tPID_Data[0],tPID_Data[1]);
    NRF_LOG_FLUSH();     
    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tPID_ADD,2, tPID_Data,2);	
    nrf_delay_ms(1);
    NRF_LOG_INFO(("%d  tPID  0x%x %x"),error_code,tPID_Data[0],tPID_Data[1]);
    NRF_LOG_FLUSH();     
    
    
    
    
    
    //д �����˲�����
//    uint8_t tFILTLSB[2] = {0};
//    uint8_t tFILTMSB[2] = {0};
//    tFILTLSB[0] = 0xCC;tFILTLSB[1] = 0x04; 
//    tFILTMSB[0] = 0xCD;tFILTMSB[1] = 0x00;
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tFILTLSB,2, NULL,0);	
//    nrf_delay_ms(1);
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tFILTMSB,2, NULL,0);	
//    nrf_delay_ms(1);      

//    uint8_t tPID_ADD[2] = {0};
//    uint8_t tPID_Data[2] = {0};
//    tPID_ADD[0] = 0x72;tPID_ADD[1] = 0x00; 
//    tPID_Data[0] = 0x55;tPID_Data[1] = 0x66; 
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tPID_ADD,2, tPID_Data,2);	
//    nrf_delay_ms(1);
//    NRF_LOG_INFO(("%d  tPID  0x%x %x"),error_code,tPID_Data[0],tPID_Data[1]);
//    NRF_LOG_FLUSH();   
//    
//    tPID_Data[0] = 0x55;tPID_Data[1] = 0x66; 
//    error_code |= nrf_drv_spi_transfer(&SPI_CollectData, tPID_ADD,2, tPID_Data,2);	
//    nrf_delay_ms(1);     
//    NRF_LOG_INFO(("%d  tPID  0x%x %x"),error_code,tPID_Data[0],tPID_Data[1]);
//    NRF_LOG_FLUSH();     
    
    
    
    
    NRF_LOG_INFO(("||Initialize||-->IMU_B_ADIS------>error  0x%x"),error_code);
    NRF_LOG_FLUSH();    
    
    //ж��SPI
    nrf_drv_spi_uninit(&SPI_CollectData);
    //�ر� IMU_B nCS �ܽ�    
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUB_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUB_nCS);  //���1          
    nrf_delay_ms(1);       
#endif   

//MTi ��ʼ��
#if configIMU_MPU92_MTi
    nrfx_gpiote_out_uninit(configGPIO_SPI_IMUB_nCS);
    SPI_config.ss_pin = configGPIO_SPI_IMUB_nCS; 
    error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);
    nrf_delay_ms(1);

//Go to Config
    uint8_t tMTi_GOTOConfig[9] = {0x03,0x00,0x00,0x00};
    uint8_t tTest[4]={0};
    error_code = nrf_drv_spi_transfer(&SPI_CollectData, tMTi_GOTOConfig,9, tTest,4);
    NRF_LOG_INFO("tMTi_GOTOConfig error_code is %d",error_code);
    NRF_LOG_HEXDUMP_INFO(tTest,4);
    NRF_LOG_FLUSH();

    uint8_t tMTi_DEVID[5] = {0xFA,0xFF,0x30,0x00,0xD1}; 
    uint8_t tMTi_Data_DEVID[9] = {0}; 
    error_code = nrf_drv_spi_transfer(&SPI_CollectData, tMTi_DEVID,5, tMTi_Data_DEVID,9);
    NRF_LOG_INFO("tMTi_MESSAGE error_code is %d",error_code);
    NRF_LOG_HEXDUMP_INFO(tMTi_Data_DEVID,9);
    NRF_LOG_FLUSH();     

//    error_code = nrf_drv_spi_transfer(&SPI_CollectData, NULL,0, tTest,9);
//    NRF_LOG_INFO("tMTi_MESSAGE error_code is %d",error_code);
//    NRF_LOG_HEXDUMP_INFO(tTest,9);
//    NRF_LOG_FLUSH();      
    

    NRF_LOG_INFO(("||Initialize||-->IMU_B_MTi------->error  0x%x"),error_code);
    NRF_LOG_FLUSH();    
    
    //ж��SPI
    nrf_drv_spi_uninit(&SPI_CollectData);
    //�ر� IMU_B nCS �ܽ�    
    error_code |= nrfx_gpiote_out_init(configGPIO_SPI_IMUB_nCS,&tconfigGPIO_OUT);
    nrfx_gpiote_out_set(configGPIO_SPI_IMUB_nCS);  //���1          
    nrf_delay_ms(1);   

#endif 

// (3) ������ȷ��IMU SPI   
    //����SPI ����nCS���趨����ʼ��
    SPI_config.ss_pin			= NRF_DRV_SPI_PIN_NOT_USED;         //��ʹ��nCS�ܽ�
    SPI_config.frequency		= NRF_DRV_SPI_FREQ_1M;    
    error_code |= nrf_drv_spi_init(&SPI_CollectData, &SPI_config, NULL,NULL);	   
    NRF_LOG_INFO(("||Initialize||-->IMU_SPI--------->error  0x%x"),error_code); 
    NRF_LOG_FLUSH();     

    return error_code;  
    
}


///**
// * IMU ��ʼ�� ADIS
// *   ���� �ܽŹ���ʽ����ʼ�� IMU_A �� IMU_B
//*/
//uint8_t ucIMU_ADIS_Initial(void)
//{
//    uint8_t erro_code = 0;
//    
//    //(1) IMU_B nCS �õ�
//    nrfx_gpiote_out_clear(configGPIO_SPI_IMUB_nCS); 
//    nrf_delay_us(1); 
//        
//    //(2) д ���REST
//    uint8_t tRST_LSB[2] = {0};    
//    tRST_LSB[0] = 0xE8;tRST_LSB[1] = 0x80;
//    uint8_t tRST_MSB[2] = {0};    
//    tRST_MSB[0] = 0xE9;tRST_MSB[1] = 0x00;
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tRST_LSB,2,NULL,0);     
//    nrf_delay_ms(1);
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tRST_MSB,2,NULL,0);
//    nrf_delay_ms(500);
//    
//    //(3) д ���ò���Ƶ�� 200 Hz
//    uint8_t tHzLSB[2] = {0};
//    uint8_t tHzMSB[2] = {0};
//    tHzLSB[0] = 0xE4;tHzLSB[1] = 0x09; 
//    tHzMSB[0] = 0xE5;tHzMSB[1] = 0x00;
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tHzLSB,2, NULL,0);	
//    nrf_delay_ms(1);
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tHzMSB,2, NULL,0);	
//    nrf_delay_ms(1);      
//    
//    //(4) д �����˲�����
//    uint8_t tFILTLSB[2] = {0};
//    uint8_t tFILTMSB[2] = {0};
//    tFILTLSB[0] = 0xCC;tFILTLSB[1] = 0x04; 
//    tFILTMSB[0] = 0xCD;tFILTMSB[1] = 0x00;
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tFILTLSB,2, NULL,0);	
//    nrf_delay_ms(1);
//    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tFILTMSB,2, NULL,0);	
//    nrf_delay_ms(1);     
//    
//    //(5) nCS IMU_B �ø�
//    nrfx_gpiote_out_set(configGPIO_SPI_IMUB_nCS); 
//    nrf_delay_us(1);   

//    return erro_code;    
//}

// ADIS ��ȡ����
uint8_t Leo_ADIS_Read_ALLData(uint8_t * Data,uint8_t Length)
{
    uint8_t erro_code = 0;
    uint8_t tIMUB_ADIS_ADD[2] = {0};
    tIMUB_ADIS_ADD[0] = 0x68;tIMUB_ADIS_ADD[1] = 0x00;
    
    erro_code |= nrf_drv_spi_transfer(&SPI_CollectData, tIMUB_ADIS_ADD,2, Data,Length);
    return erro_code;
}

