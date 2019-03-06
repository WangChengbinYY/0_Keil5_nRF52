/*
*********************************************************************************************************
*
*    ģ������ : �ⲿ������ MPU9255
*    �ļ����� : Leo_MPU9255
*    ��    �� : V1.0
*    ˵    �� : �ⲿ������ MPU9255
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-18     Leo
*        V1.1    2019-02-24     Leo
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_MPU9255.h"



extern nrf_drv_spi_t    SPI_CollectData;  			//IMU���ݲɼ�ʹ�õ�SPIʵ��		
extern uint8_t	    G_CollectData[512];                 //SDCardҪ�������ݵĻ���
extern uint16_t	    G_CollectData_Counter;    
extern uint8_t	    G_MAG_Coeffi[6]; 



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
/********************************���ⲿ�ӿں���ʵ��***************************/
/*--------------------------------------------------------------------------*/

//@brief nRF52<--SPI-->MPU9255  nRF52��MPU9255 ��ʼ��
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*	����ֵ˵����
//<*		NRF_SUCCESS		��ʼ���ɹ� (0)
//<*		����					ʧ��
/*--------------------------------------------------------------------------*/
uint8_t ucMPU9255_INIT(void)
{
	uint8_t error_code = 0;
	uint8_t mAK8963_ID = 0;		
	uint8_t mMPU9255_ID = 0;    
   
	//=====================================����MPU9255============================================
	//��������
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_RESET);//reset
	nrf_delay_ms(50);
		
	//���������� �����ź�·��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SIGNAL_PATH_RESET,MPU9255_SIGNAL_PATH_RESET_GYRO|MPU9255_SIGNAL_PATH_RESET_ACCEL|MPU9255_SIGNAL_PATH_RESET_TEMP); 
	nrf_delay_ms(3);
	
	//����ʱ��Դ
	//Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator �˴�ѡ��Z�����ڲ�ʱ�� PLL���໷
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_CLOCK_CLKSEL);  
	nrf_delay_ms(3);

	//���ô������жϹܽŵ�����
	//�ж�״̬��[7]�ߵ�ƽ��Ч��[6]����������[5]�������50us��ȵ����壻[4]�κζ�ȡ��������ж�״̬��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_PIN,MPU9255_INT_PIN_LATCH_INT_EN|MPU9255_INT_PIN_INT_ANYRD_2CLEAR);	 
	nrf_delay_ms(3);
	
	//���ô������ж�ʹ�����ã����κ��µ�ԭʼ���ݲ�������������ж�
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_ENABLE,MPU9255_INT_ENABLE_RAW_RDY_EN);	
	nrf_delay_ms(3);	
	
	//���������ݺͼӼ�
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_2,MPU9255_PWR_MGMT_2_ACCEL_XYZ|MPU9255_PWR_MGMT_2_GYRO_XYZ);	
	nrf_delay_ms(3);	
	
	//�������������������������� �� Fchoice_b(���ں������Ƶ�ʺ͵�ͨ�˲�)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_GYRO_CONFIG,MPU9255_GYRO_CONFIG_FS | MPU9255_GYRO_CONFIG_Fb); 
	nrf_delay_ms(3);	
	
	//���ü��ٶȼƲ���1:����
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG1,MPU9255_ACCEL_CONFIG1_FS);
	nrf_delay_ms(3);	
	
	//���ü��ٶȼƲ���2:DPLF
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG2,MPU9255_ACCEL_CONFIG2_FCHOICE_B|MPU9255_ACCEL_CONFIG2_DLPFCFG);		
	nrf_delay_ms(3);		
	
	//����MPU9255����Ƶ��
	//1.��ǰ�����ݲ������õĻ����ϣ��������� �� �¶ȴ�������ԭʼ���ݲ���Ƶ�� �� ��ͨ�����˲�����  Gyro DLPF(0x03(41Hz,5.9ms,1kHz);)) Temp_DLPF(42Hz 4.8ms)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_CONFIG,MPU9255_CONFIG_FIFO_MODE | MPU9255_CONFIG_EXT_SYNC_SET | MPU9255_CONFIG_DLPF_CFG);  
	nrf_delay_ms(3);		
	//2.����ϵͳ���Ƶ��(��Ƶ����)����SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	//	ϵͳʵ��ԭʼ���ݲɼ�1kHz�������250Hz <***���ﲻ�����ȡƽ���������ʲô������***>
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SMPLRT_DIV,MPU9255_SMPLRT_DIV_Rate); 
	nrf_delay_ms(3);		
	
	//=====================================����AK8963============================================			
	//����λ��I2Cģʽ�����ڿ�ʼ��AK8963��������
//	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_MST_EN); 
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,0x30); 
	nrf_delay_ms(50);
	//���� MPU9255 I2Cͨ�ţ��жϵȴ���there is a stop between reads��ʱ������ 400kHz(��������ʹӻ���Ҫһ��),
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_CTRL,MPU9255_I2C_MST_CTRL_MULT_MST_EN|MPU9255_I2C_MST_CTRL_WAIT_FOR_ES|MPU9255_I2C_MST_CTRL_I2C_MST_P_NSR|MPU9255_I2C_MST_CTRL_I2C_MST_CLK); 
	nrf_delay_ms(20);	
	
	//reset AK8963 �����λ����
    //  ����֣�������Ƭѡ�ܽſ���ʱ���ܻ�ʧ�ܣ������Ϊʲô����ʼ��ֻ���� SPI�ĳ�ʼ����uint 
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_RSV,MPU9255_AK8963_RSV_SRST);
	nrf_delay_ms(10);
	
	//POWER_DOWN ģʽ�ر� �����ģʽΪ16λ
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);

	//����˿ROM����ģʽ �����ģʽΪ16λ���������ȶ��� ��ǿ����ֵ��������
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_FUSE_ROM|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);	
		
	//��ȡ��ǿ����ֵ�������� 
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_ASAX,&G_MAG_Coeffi[2],3);    
	nrf_delay_ms(3);	
    if(error_code == 0)
    {
        memcpy(G_CollectData+G_CollectData_Counter,G_MAG_Coeffi,6);
        G_CollectData_Counter = G_CollectData_Counter + 6;
        NRF_LOG_INFO("		AK8963 Config is ��0x%x %x %x",G_MAG_Coeffi[2],G_MAG_Coeffi[3],G_MAG_Coeffi[4]);
    }        
	//���� ��������� 
	//Left����������Ϊ��0xAF 0xB2 0xA6  ����Ҫע�⣬��ͬ��оƬ���˴��Ĳ�����һ��
	//Right����������Ϊ��0xAB 0xAC 0xA1  

		
	//��ȡ����������ɺ��ٴ� POWER_DOWN
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(3);	
	
	//���� AK8963���ģʽΪ16λ���������ģʽ2 100Hz
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_BIT|MPU9255_AK8963_CNTL1_CONTINU_MEASURE2);
	nrf_delay_ms(3);	
		
	//���� ��ǿ�� ���ݲɼ�������ϵͳ����Ƶ��Ϊ250Hz���˴�5��Ƶ����ǿ�Ʋ���Ƶ�� 50Hz  ͬʱ�ر�SLV4�Ĵ��书��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_REG_DIS|MPU9255_I2C_SLV4_CTRL_MST_DLY);
	nrf_delay_ms(3);		
		
	//���� ��ǿ�� ���ݲɼ��ӳٿ���
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_DELAY_CTRL,MPU9255_I2C_MST_DELAY_CTRL_SHADOW|MPU9255_I2C_MST_DELAY_CTRL_SLV0_EN);
	nrf_delay_ms(3);		
		
	//�ر�SLV4�Ĵ��书��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,0x00);
	nrf_delay_ms(3);	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,0x00);
	nrf_delay_ms(3);	

	//���� SLV0�����ݶ�ȡ����
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_ADDR,MPU9255_AK8963_I2C_ADDR|0x80);
	nrf_delay_ms(3);		
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_REG,MPU9255_AK8963_ST1);
	nrf_delay_ms(3);		
	//���� ���ڴ�����ⲿ�Ĵ����ʹ�����ֽڳ���
	//Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_CTRL,MPU9255_I2C_SLV0_CTRL_EN|MPU9255_I2C_SLV0_CTRL_LENG|MPU9255_I2C_SLV0_CTRL_BYTE_SW|MPU9255_I2C_SLV0_CTRL_REG_DIS|MPU9255_I2C_SLV0_CTRL_GRP);
	nrf_delay_ms(3);		

    //�����ȡMPU9255�豸ID 0x73 ��ȷ����ֵΪ��1
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_WHO_AM_I,&mMPU9255_ID,1); 
	nrf_delay_ms(3);		
	NRF_LOG_INFO("		MPU9255 Device ID_0x73 is ��0x%x",mMPU9255_ID);
	
	//�����ȡAK8963�豸ID 0x48 ��ȷ����ֵΪ��1
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_WIA, &mAK8963_ID,1 );	
	NRF_LOG_INFO("		AK8963 Device ID_0x48 is ��0x%x",mAK8963_ID);
	nrf_delay_ms(3);		
	
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


//@brief nRF52<--SPI-->MPU9255  nRF52��ȡMPU9255�� ���ݼ����ݣ����жϷ���
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		��ȡ�����ݷ���ȫ�ֱ�����
//<*	����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����							��ȡʧ��
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

//@brief nRF52<--SPI-->MPU9255  nRF52��ȡMPU9255�� ���ݼ����ݣ����жϷ���,û��������������
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		��ȡ�����ݷ���ȫ�ֱ�����
//<*	����ֵ˵��:
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
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

