/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_nRF52_SDCard     
 * ƽ̨    ��nRF52832
 * ����    ������nRF52ƽ̨�� SDCard�ļ��洢  
 * ����    �����ɱ�
**********************************************************************/

#include "Leo_nRF52_SDCard.h"


/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(Leo_nRF52_SDCard_SPI_MOSI_PIN, Leo_nRF52_SDCard_SPI_MISO_PIN, Leo_nRF52_SDCard_SPI_SCK_PIN, Leo_nRF52_SDCard_SPI_CS_PIN)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

extern uint8_t		               G_MPU9255_Data[28];

//<*MPU9255�жϴ����ļ����������ں���ͳ���Ƿ񶪰�
extern uint32_t	                   G_MPU9255_Counter;

//<*GPS���ݽ��ջ��� �� ����������ݰ�
extern struct minmea_sentence_rmc  G_GPS_RMC;
extern uint8_t                     G_GPS_RMC_IsValide;


static FATFS fs;
static FIL file;
static FILINFO fno;

//@brief ����nRF52��SDCard����д�����ĳ�ʼ��
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*		
//<*	����ֵ˵��:
//<*		FR_OK = 0,	����ʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_nRF52_SDCard_Initial(void)
{
    uint8_t mNumberFile = 0;
    char mNameNRF52File[13];
	FRESULT ff_result;
    DSTATUS disk_state = STA_NOINIT;

    // Initialize FATFS disk I/O interface by providing the block device.
    static diskio_blkdev_t drives[] =
    {
            DISKIO_BLOCKDEV_CONFIG(NRF_BLOCKDEV_BASE_ADDR(m_block_dev_sdc, block_dev), NULL)
    };
	diskio_blockdev_register(drives, ARRAY_SIZE(drives));

    NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
       NRF_LOG_INFO("Disk initialization failed.");
       return FR_DISK_ERR;
    }else
    {
        NRF_LOG_INFO("Disk initialization is OK!");
    }
			
		NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
		NRF_LOG_INFO("Mount failed");
		return ff_result;
    }else
    {
        NRF_LOG_INFO("Mount is OK!");
    }        

    
    do
    {   
        mNumberFile++;
        if(mNumberFile > 99)
            return 1;
        
        sprintf(mNameNRF52File,"IMUGPS%d.dat",mNumberFile);

        ff_result = f_stat(mNameNRF52File,&fno);
        if(ff_result == FR_NO_FILE)
        {
            //NRF_LOG_INFO("  %s File is not existed!",mNameNRF52File);
            break;
        }else{
            if(ff_result == FR_OK)
            {
                //NRF_LOG_INFO("  %s File is existed!",mNameNRF52File);
            }else
            {
                NRF_LOG_INFO(" An error occured.!!!!!!");
                return 1;
            }
        }   
    }while(1);
        
    ff_result = f_open(&file, mNameNRF52File, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (ff_result != FR_OK)
    {
       NRF_LOG_INFO("Unable to open or create file!%s",mNameNRF52File);
       return ff_result;
    }else
	{
		NRF_LOG_INFO("  IMUFile Open is OK!%s",mNameNRF52File);
	}

    
	return  ff_result;				
}



//@brief ����nRF52 �� SDCard�� ��д������ 
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*	����ֵ˵��:
//<*		FR_OK = 0,	����ʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_nRF52_SDCard_SaveData(uint8_t* mbuffer,UINT mLength)
{
    FRESULT ff_result;
    UINT mWritenByteNum = 0; 
    ff_result = f_write(&file,mbuffer,mLength,&mWritenByteNum);
    if(mWritenByteNum < mLength)
        return 100;
    return ff_result;
}




//@brief ����nRF52 SDCard�� ������ɺ󣬹ر��ļ�
/*--------------------------------------------------------------------------*/
//<*	����˵��:
//<*	����ֵ˵��:
//<*		FR_OK = 0,	����ʧ��
/*--------------------------------------------------------------------------*/
uint8_t Leo_nRF52_SDCard_FileClose(void)
{
	FRESULT ff_result;
	ff_result = f_close(&file);
//	ff_result = f_close(&file_IMU);
//  ff_result = f_close(&file_GPS);
	return ff_result;	
}


