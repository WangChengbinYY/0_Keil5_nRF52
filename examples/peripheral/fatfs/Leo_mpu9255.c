
/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_mpu9255     
 * ƽ̨    ��mpu9255
 * ����    ��������mpu9255����ض���  
 * ����    �����ɱ�
**********************************************************************/

#include "Leo_mpu9255.h"

/*--------------------------------------------------------------------------*/
/*********************************��������***********************************/
/*--------------------------------------------------------------------------*/
//@brief ȫ�ֱ�������
/*--------------------------------------------------------------------------*/
				
extern uint8_t		G_MPU9255_MAG_ASAXYZ[7];
extern uint8_t		G_MPU9255_MAG_ASAXYZ_IsValid;

extern uint8_t		G_MPU9255_Data[32];
//extern uint8_t		G_MPU9255_Data_IsValid;

/*--------------------------------------------------------------------------*/

//@brief nRF52<--SPI-->MPU9255	ʹ����Ҫ�� ��������*/
/*--------------------------------------------------------------------------*/
static uint8_t              G_MPU9255_SPI_xfer_Done = 1;  		                                    //SPI0���ݴ���(����/����)��ɵı�־
static const nrf_drv_spi_t  SPI_MPU9255 = NRF_DRV_SPI_INSTANCE(Leo_nRF52_MPU9255_SPI);  			//MPU9255ʹ�õ�SPIʵ��		
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/*********************************�ڲ�����ʵ��********************************/
/*--------------------------------------------------------------------------*/

//@brief nRF52<--SPI-->MPU9255	SPI�¼�������
/*--------------------------------------------------------------------------*/
static void Leo_MPU9255_SPI_Event_Handler(nrf_drv_spi_evt_t const * p_event,void * p_context){
	G_MPU9255_SPI_xfer_Done = 1;
}

//@brief nRF52<--SPI-->MPU9255	��ʼ��
/*--------------------------------------------------------------------------*/
//<* 	����˵����
//<*		uint8_t mId									SPIʵ�������
//<*		uint8_t mCS_PIN							SPI��Ӧ��Ƭѡ�ܽ�
//<*	����ֵ˵����
//<*		NRF_SUCCESS		��ȡ�ɹ� (0)
//<*		����					��ȡʧ��
/*--------------------------------------------------------------------------*/
static uint8_t Leo_SPI_Init()
{
    uint32_t error_code = 0;
	nrf_drv_spi_config_t SPI_config = NRF_DRV_SPI_DEFAULT_CONFIG;
	SPI_config.sck_pin 			= Leo_nRF52_MPU9255_SPI_SCK_PIN;
	SPI_config.mosi_pin 		= Leo_nRF52_MPU9255_SPI_MOSI_PIN;
	SPI_config.miso_pin 		= Leo_nRF52_MPU9255_SPI_MISO_PIN;
	SPI_config.ss_pin			= Leo_nRF52_MPU9255_SPI_CS_PIN;
	SPI_config.irq_priority	    = SPI_DEFAULT_CONFIG_IRQ_PRIORITY;		//ϵͳSPI�ж�Ȩ��Ĭ���趨Ϊ 7 
	SPI_config.orc				= 0xFF;
	SPI_config.frequency		= NRF_DRV_SPI_FREQ_500K;				//�ϰ汾�� 500K Ȩ��2���°汾�� 4MkHz Ȩ����7����֪���Ƿ�������
	SPI_config.mode             = NRF_DRV_SPI_MODE_0;                     
    SPI_config.bit_order        = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;

	
	//�������ò��� �� ʵ��spi_1 ���г�ʼ�� 
	error_code = nrf_drv_spi_init(&SPI_MPU9255, &SPI_config, Leo_MPU9255_SPI_Event_Handler,NULL);	
}


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
	w2_data[0] = Leo_nRF52_MPU9255_SPI_READ_BIT|mRegisterAddress;

	G_MPU9255_SPI_xfer_Done = 0;
	 APP_ERROR_CHECK(nrf_drv_spi_transfer(&SPI_MPU9255, w2_data,2, r2_data, read_number));
	
	NRF_LOG_FLUSH();
	
	while(G_MPU9255_SPI_xfer_Done == 0)
		__WFE();	

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

	G_MPU9255_SPI_xfer_Done = 0;
	error_code = nrf_drv_spi_transfer(&SPI_MPU9255, w2_data, 2, r2_data, 2);
	while(G_MPU9255_SPI_xfer_Done == 0)
		__WFE();	

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
		if(timeout++ > 50)
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
uint8_t Leo_MPU9255_SPI_Initial(void)
{
	uint8_t error_code = 0;
	uint8_t mAK8963_ID = 0;		
	uint8_t mMPU9255_ID = 0;
	error_code |= Leo_SPI_Init();
	NRF_LOG_INFO("SPI_1_Init...is DONE(err_code is 0x%x)",error_code);	
	nrf_delay_ms(300); 	
	NRF_LOG_FLUSH();
    
	//=====================================����MPU9255============================================
	//��������
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_RESET);//reset
	nrf_delay_ms(100);
		
	//���������� �����ź�·��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SIGNAL_PATH_RESET,MPU9255_SIGNAL_PATH_RESET_GYRO|MPU9255_SIGNAL_PATH_RESET_ACCEL|MPU9255_SIGNAL_PATH_RESET_TEMP); 
	nrf_delay_ms(100);
	
	//����ʱ��Դ
	//Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator �˴�ѡ��Z�����ڲ�ʱ�� PLL���໷
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_CLOCK_CLKSEL);  
	nrf_delay_ms(10);

	//���ô������жϹܽŵ�����
	//�ж�״̬��[7]�ߵ�ƽ��Ч��[6]����������[5]�������50us��ȵ����壻[4]�κζ�ȡ��������ж�״̬��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_PIN,MPU9255_INT_PIN_LATCH_INT_EN|MPU9255_INT_PIN_INT_ANYRD_2CLEAR);	 
	nrf_delay_ms(10);
	
	//���ô������ж�ʹ�����ã����κ��µ�ԭʼ���ݲ�������������ж�
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_ENABLE,MPU9255_INT_ENABLE_RAW_RDY_EN);	
	nrf_delay_ms(10);	
	
	//���������ݺͼӼ�
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_2,MPU9255_PWR_MGMT_2_ACCEL_XYZ|MPU9255_PWR_MGMT_2_GYRO_XYZ);	
	nrf_delay_ms(10);	
	
	//�������������������������� �� Fchoice_b(���ں������Ƶ�ʺ͵�ͨ�˲�)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_GYRO_CONFIG,MPU9255_GYRO_CONFIG_FS | MPU9255_GYRO_CONFIG_Fb); 
	nrf_delay_ms(10);	
	
	//���ü��ٶȼƲ���1:����
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG1,MPU9255_ACCEL_CONFIG1_FS);
	nrf_delay_ms(10);	
	
	//���ü��ٶȼƲ���2:DPLF
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG2,MPU9255_ACCEL_CONFIG2_FCHOICE_B|MPU9255_ACCEL_CONFIG2_DLPFCFG);		
	nrf_delay_ms(10);		
	
	//����MPU9255����Ƶ��
	//1.��ǰ�����ݲ������õĻ����ϣ��������� �� �¶ȴ�������ԭʼ���ݲ���Ƶ�� �� ��ͨ�����˲�����  Gyro DLPF(0x03(41Hz,5.9ms,1kHz);)) Temp_DLPF(42Hz 4.8ms)
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_CONFIG,MPU9255_CONFIG_FIFO_MODE | MPU9255_CONFIG_EXT_SYNC_SET | MPU9255_CONFIG_DLPF_CFG);  
	nrf_delay_ms(10);		
	//2.����ϵͳ���Ƶ��(��Ƶ����)����SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	//	ϵͳʵ��ԭʼ���ݲɼ�1kHz�������250Hz <***���ﲻ�����ȡƽ���������ʲô������***>
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SMPLRT_DIV,MPU9255_SMPLRT_DIV_Rate); 
	nrf_delay_ms(10);		
	
	//=====================================����AK8963============================================			
	//����λ��I2Cģʽ�����ڿ�ʼ��AK8963��������
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_MST_EN); 
	nrf_delay_ms(100);
	//���� MPU9255 I2Cͨ�ţ��жϵȴ���there is a stop between reads��ʱ������ 400kHz(��������ʹӻ���Ҫһ��),
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_CTRL,MPU9255_I2C_MST_CTRL_MULT_MST_EN|MPU9255_I2C_MST_CTRL_WAIT_FOR_ES|MPU9255_I2C_MST_CTRL_I2C_MST_P_NSR|MPU9255_I2C_MST_CTRL_I2C_MST_CLK); 
	nrf_delay_ms(10);	
	
	//reset AK8963 �����λ����
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_RSV,MPU9255_AK8963_RSV_SRST);
	nrf_delay_ms(100);
	
	//POWER_DOWN ģʽ�ر� �����ģʽΪ16λ
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);

	//����˿ROM����ģʽ �����ģʽΪ16λ���������ȶ��� ��ǿ����ֵ��������
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_FUSE_ROM|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);	
		
	//��ȡ��ǿ����ֵ��������
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_ASAX,&G_MPU9255_MAG_ASAXYZ[2],3);    
	nrf_delay_ms(10);	
    if(error_code == 0)
    {
        G_MPU9255_MAG_ASAXYZ_IsValid = 1;
    }
        
	//���� ��������� 
	//Left����������Ϊ��0xAF 0xB2 0xA6  ����Ҫע�⣬��ͬ��оƬ���˴��Ĳ�����һ��
	//Right����������Ϊ��0xAB 0xAC 0xA1  

		
	//��ȡ����������ɺ��ٴ� POWER_DOWN
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_POWER_DOWN|MPU9255_AK8963_CNTL1_BIT);
	nrf_delay_ms(10);	
	
	//���� AK8963���ģʽΪ16λ���������ģʽ2 100Hz
	error_code |= Leo_MPU9255_AK8963_SPI_WriteOneByte(MPU9255_AK8963_CNTL1,MPU9255_AK8963_CNTL1_BIT|MPU9255_AK8963_CNTL1_CONTINU_MEASURE2);
	nrf_delay_ms(10);	
		
	//���� ��ǿ�� ���ݲɼ�������ϵͳ����Ƶ��Ϊ250Hz���˴�5��Ƶ����ǿ�Ʋ���Ƶ�� 50Hz  ͬʱ�ر�SLV4�Ĵ��书��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_CTRL,MPU9255_I2C_SLV4_CTRL_REG_DIS|MPU9255_I2C_SLV4_CTRL_MST_DLY);
	nrf_delay_ms(10);		
		
	//���� ��ǿ�� ���ݲɼ��ӳٿ���
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_MST_DELAY_CTRL,MPU9255_I2C_MST_DELAY_CTRL_SHADOW|MPU9255_I2C_MST_DELAY_CTRL_SLV0_EN);
	nrf_delay_ms(10);		
		
	//�ر�SLV4�Ĵ��书��
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_ADDR,0x00);
	nrf_delay_ms(10);	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV4_REG,0x00);
	nrf_delay_ms(10);	

	//���� SLV0�����ݶ�ȡ����
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_ADDR,MPU9255_AK8963_I2C_ADDR|0x80);
	nrf_delay_ms(10);		
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_REG,MPU9255_AK8963_ST1);
	nrf_delay_ms(10);		
	//���� ���ڴ�����ⲿ�Ĵ����ʹ�����ֽڳ���
	//Enable reading data from this slave at the sample rate and storing data at the first available EXT_SENS_DATA	
	error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_I2C_SLV0_CTRL,MPU9255_I2C_SLV0_CTRL_EN|MPU9255_I2C_SLV0_CTRL_LENG|MPU9255_I2C_SLV0_CTRL_BYTE_SW|MPU9255_I2C_SLV0_CTRL_REG_DIS|MPU9255_I2C_SLV0_CTRL_GRP);
	nrf_delay_ms(10);		
	
//	NRF_LOG_INFO("X Magenetic Adjustment is ��0x%x",G_MAG_ASAXYZ[0]);
//	NRF_LOG_INFO("Y Magenetic Adjustment is ��0x%x",G_MAG_ASAXYZ[1]);
//	NRF_LOG_INFO("Z Magenetic Adjustment is ��0x%x",G_MAG_ASAXYZ[2]);
	
	//�����ȡMPU9255�豸ID 0x73 ��ȷ����ֵΪ��1
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_WHO_AM_I,&mMPU9255_ID,1); 
	nrf_delay_ms(30);		
	NRF_LOG_INFO("		MPU9255 Device ID_0x73 is ��0x%x",mMPU9255_ID);
	
	//�����ȡAK8963�豸ID 0x48 ��ȷ����ֵΪ��1
	error_code |= Leo_MPU9255_AK8963_SPI_ReadBytes(MPU9255_AK8963_WIA, &mAK8963_ID,1 );	
	NRF_LOG_INFO("		AK8963 Device ID_0x48 is ��0x%x",mAK8963_ID);
	nrf_delay_ms(30);		
	
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
uint8_t Leo_MPU9255_Read_ACC(void)
{
	uint8_t		error_code = 0;
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_XOUT_H, &G_MPU9255_Data[12], 6);	
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
uint8_t Leo_MPU9255_Read_Gyro(void)
{
	uint8_t		error_code = 0;
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_XOUT_H, &G_MPU9255_Data[18], 6);
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
uint8_t Leo_MPU9255_Read_Magnetic(void)
{
	uint8_t	error_code = 0;
	uint8_t buf[8] = {0}; 
	error_code = Leo_MPU9255_SPI_ReadBytes(MPU9255_EXT_SENS_DATA_00,buf,8);
	G_MPU9255_Data[24] = buf[2];
	G_MPU9255_Data[25] = buf[1];
	G_MPU9255_Data[26] = buf[4];
	G_MPU9255_Data[27] = buf[3];
	G_MPU9255_Data[28] = buf[6];
	G_MPU9255_Data[29] = buf[5];	
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





/**********************************************************************************************
* ��  �� : 	Leo_MPU9255_SPI_Initial_Read����
*          	��ȡ������ Ĭ�ϵĳ�ʼ�� ����
* ��  �� :		��ȡ�������ֽڵĸ���
* ����ֵ : 	1 MPU9255 found on the bus and ready for operation.
						0 MPU9255 not found on the bus or communication failure.
***********************************************************************************************/ 
/*uint8_t Leo_MPU9255_SPI_Initial_Read(uint8_t mSPI_ID)
{
	uint8_t error_code = 0;
	uint8_t mASAXYZ[3] = {0};
	uint8_t tmp = 0;
	uint8_t Config = 0;
	
	//=====================================����MPU9255============================================
	//��������
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_RESET);//reset
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_PWR_MGMT_1,&Config,1);//reset
	printf("MPU9255_PWR_MGMT_1 is : 0x%x\r\n",Config);
	
	//���������� �����ź�·��
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SIGNAL_PATH_RESET,MPU9255_SIGNAL_PATH_RESET_GYRO|MPU9255_SIGNAL_PATH_RESET_ACCEL|MPU9255_SIGNAL_PATH_RESET_TEMP); 
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_SIGNAL_PATH_RESET,&Config,1);
	printf("MPU9255_SIGNAL_PATH_RESET is : 0x%x\r\n",Config);
	//����ʱ��Դ
	//Auto selects the best available clock source �C PLL if ready, else use the Internal oscillator �˴�ѡ��Z�����ڲ�ʱ�� PLL���໷
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_1,MPU9255_PWR_MGMT_1_CLOCK_CLKSEL);  
	nrf_delay_ms(10);
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_PWR_MGMT_1,&Config,1);
	printf("MPU9255_PWR_MGMT_1 is : 0x%x\r\n",Config);
	
	//���ô������жϹܽŵ�����
	//�ж�״̬��[7]�ߵ�ƽ��Ч��[6]����������[5]�������50us��ȵ����壻[4]�κζ�ȡ��������ж�״̬��
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_PIN,MPU9255_INT_PIN_INT_ANYRD_2CLEAR);	 
	nrf_delay_ms(10);
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_INT_PIN,&Config,1);
	printf("MPU9255_INT_PIN is : 0x%x\r\n",Config);
	
	//���ô������ж�ʹ�����ã����κ��µ�ԭʼ���ݲ�������������ж�
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_INT_ENABLE,MPU9255_INT_ENABLE_RAW_RDY_EN);	
	nrf_delay_ms(10);	
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_INT_ENABLE,&Config,1);
	printf("MPU9255_INT_ENABLE is : 0x%x\r\n",Config);
	
	//���������ݺͼӼ�
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_PWR_MGMT_2,MPU9255_PWR_MGMT_2_ACCEL_XYZ|MPU9255_PWR_MGMT_2_GYRO_XYZ);	
	nrf_delay_ms(10);	
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_PWR_MGMT_2,&Config,1);
	printf("MPU9255_PWR_MGMT_2 is : 0x%x\r\n",Config);
	
	//�������������������������� �� Fchoice_b(���ں������Ƶ�ʺ͵�ͨ�˲�)
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_GYRO_CONFIG,MPU9255_GYRO_CONFIG_FS | MPU9255_GYRO_CONFIG_Fb); 
	nrf_delay_ms(10);	
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_GYRO_CONFIG,&Config,1);
	printf("MPU9255_GYRO_CONFIG is : 0x%x\r\n",Config);
	
	//���ü��ٶȼƲ���1:����
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG1,MPU9255_ACCEL_CONFIG1_FS);
	nrf_delay_ms(10);	
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_CONFIG1,&Config,1);
	printf("MPU9255_ACCEL_CONFIG1 is : 0x%x\r\n",Config);
	
	//���ü��ٶȼƲ���2:DPLF
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_ACCEL_CONFIG2,MPU9255_ACCEL_CONFIG2_FCHOICE_B|MPU9255_ACCEL_CONFIG2_DLPFCFG);		
	nrf_delay_ms(10);		
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_ACCEL_CONFIG2,&Config,1);
	printf("MPU9255_ACCEL_CONFIG2 is : 0x%x\r\n",Config);
	
	//����MPU9255����Ƶ��
	//1.��ǰ�����ݲ������õĻ����ϣ��������� �� �¶ȴ�������ԭʼ���ݲ���Ƶ�� �� ��ͨ�����˲�����  Gyro DLPF(0x03(41Hz,5.9ms,1kHz);)) Temp_DLPF(42Hz 4.8ms)
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_CONFIG,MPU9255_CONFIG_FIFO_MODE | MPU9255_CONFIG_EXT_SYNC_SET | MPU9255_CONFIG_DLPF_CFG);  
	nrf_delay_ms(10);	
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_CONFIG,&Config,1);
	printf("MPU9255_CONFIG is : 0x%x\r\n",Config);	
	//2.����ϵͳ���Ƶ��(��Ƶ����)����SAMPLE_RATE=Internal_Sample_Rate / (1 + SMPLRT_DIV)
	//	ϵͳʵ��ԭʼ���ݲɼ�1kHz�������250Hz <***���ﲻ�����ȡƽ���������ʲô������***>
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_SMPLRT_DIV,MPU9255_SMPLRT_DIV_Rate); 
	nrf_delay_ms(10);		
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_SMPLRT_DIV,&Config,1);
	printf("MPU9255_SMPLRT_DIV is : 0x%x\r\n",Config);

	//����ϵͳSPI������enable spi and adk8963 iic
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_MST_EN|MPU9255_USER_CTRL_I2C_IF_DIS); 
	//error_code |= Leo_MPU9255_SPI_WriteOneByte(MPU9255_USER_CTRL,MPU9255_USER_CTRL_I2C_IF_DIS); 
	nrf_delay_ms(100);
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_USER_CTRL,&Config,1);
	printf("MPU9255_USER_CTRL is : 0x%x\r\n",Config);
	
	//��ȡ SLV���������
		nrf_delay_ms(100);
		error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_USER_CTRL,&Config,1);
	printf("MPU9255_USER_CTRL is : 0x%x\r\n",Config);
	
	
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_CTRL,&Config,1);
	printf("MPU9255_I2C_MST_CTRL is : 0x%x\r\n",Config);
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_DELAY_CTRL,&Config,1);
	printf("MPU9255_I2C_MST_DELAY_CTRL is : 0x%x\r\n",Config);
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_MST_STATUS,&Config,1);
	printf("MPU9255_I2C_MST_STATUS is : 0x%x\r\n",Config);

	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV0_ADDR,&Config,1);
	printf("MPU9255_I2C_SLV0_ADDR is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV0_REG,&Config,1);
	printf("MPU9255_I2C_SLV0_REG is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV0_CTRL,&Config,1);
	printf("MPU9255_I2C_SLV0_CTRL is : 0x%x\r\n",Config);
	
	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV1_ADDR,&Config,1);
	printf("MPU9255_I2C_SLV1_ADDR is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV1_REG,&Config,1);
	printf("MPU9255_I2C_SLV1_REG is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV1_CTRL,&Config,1);
	printf("MPU9255_I2C_SLV1_CTRL is : 0x%x\r\n",Config);

	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV2_ADDR,&Config,1);
	printf("MPU9255_I2C_SLV2_ADDR is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV2_REG,&Config,1);
	printf("MPU9255_I2C_SLV2_REG is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV2_CTRL,&Config,1);
	printf("MPU9255_I2C_SLV2_CTRL is : 0x%x\r\n",Config);

	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV3_ADDR,&Config,1);
	printf("MPU9255_I2C_SLV3_ADDR is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV3_REG,&Config,1);
	printf("MPU9255_I2C_SLV3_REG is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV3_CTRL,&Config,1);
	printf("MPU9255_I2C_SLV3_CTRL is : 0x%x\r\n",Config);


	nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_ADDR,&Config,1);
	printf("MPU9255_I2C_SLV4_ADDR is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_REG,&Config,1);
	printf("MPU9255_I2C_SLV4_REG is : 0x%x\r\n",Config);
		nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_CTRL,&Config,1);
	printf("MPU9255_I2C_SLV4_CTRL is : 0x%x\r\n",Config);
			nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_DO,&Config,1);
	printf("MPU9255_I2C_SLV4_DO is : 0x%x\r\n",Config);
			nrf_delay_ms(100);
	error_code |= Leo_MPU9255_SPI_ReadBytes(MPU9255_I2C_SLV4_DI,&Config,1);
	printf("MPU9255_I2C_SLV4_DI is : 0x%x\r\n",Config);
}

*/



