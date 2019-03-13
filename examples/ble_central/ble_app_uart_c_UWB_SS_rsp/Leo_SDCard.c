/*
*********************************************************************************************************
*
*    模块名称 : SDCard数据存储
*    文件名称 : Leo_FreeRTOS_TASK
*    版    本 : V1.0
*    说    明 : 利用FatFS实现数据的SDCard存储
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-19     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/

#include "Leo_SDCard.h"





/**
 * @brief  SDC block device definition
 * */
NRF_BLOCK_DEV_SDC_DEFINE(
        m_block_dev_sdc,
        NRF_BLOCK_DEV_SDC_CONFIG(
                SDC_SECTOR_SIZE,
                APP_SDCARD_CONFIG(configGPIO_SPI_SDCard_MOSI, configGPIO_SPI_SDCard_MISO, configGPIO_SPI_SDCard_SCK, configGPIO_SPI_SDCard_CS)
         ),
         NFR_BLOCK_DEV_INFO_CONFIG("Nordic", "SDC", "1.00")
);

extern uint8_t		               G_MPU9255_Data[28];

//<*MPU9255中断触发的计数器，用于后期统计是否丢包
extern uint32_t	                   G_MPU9255_Counter;

//<*GPS数据接收缓存 和 解析后的数据包
extern struct minmea_sentence_rmc  G_GPS_RMC;
extern uint8_t                     G_GPS_RMC_IsValide;


static FATFS fs;
static FIL file;
static FILINFO fno;

//@brief 基于nRF52对SDCard卡读写操作的初始化
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*		
//<*	返回值说明:
//<*		FR_OK = 0,	其它失败
/*--------------------------------------------------------------------------*/
uint8_t ucSDCard_INIT(void)
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

    //NRF_LOG_INFO("Initializing disk 0 (SDC)...");
    for (uint32_t retries = 3; retries && disk_state; --retries)
    {
        disk_state = disk_initialize(0);
    }
    if (disk_state)
    {
       //NRF_LOG_INFO("Disk initialization failed.");
       return FR_DISK_ERR;
    }else
    {
        //NRF_LOG_INFO("Disk initialization is OK!");
    }
			
    //NRF_LOG_INFO("Mounting volume...");
    ff_result = f_mount(&fs, "", 1);
    if (ff_result)
    {
		//NRF_LOG_INFO("Mount failed");
		return ff_result;
    }else
    {
        //NRF_LOG_INFO("Mount is OK!");
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
                //NRF_LOG_INFO(" An error occured.!!!!!!");
                return 1;
            }
        }   
    }while(1);
        
    ff_result = f_open(&file, mNameNRF52File, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
	if (ff_result != FR_OK)
    {
       //NRF_LOG_INFO("Unable to open or create file!%s",mNameNRF52File);
       return ff_result;
    }else
	{
		NRF_LOG_INFO("  IMUFile Open is OK!%s",mNameNRF52File);
	}

    
	return  ff_result;				
}



//@brief 基于nRF52 往 SDCard卡 内写入数据 
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*	返回值说明:
//<*		FR_OK = 0,	其它失败
/*--------------------------------------------------------------------------*/
uint8_t ucSDCard_SaveData(uint8_t* mbuffer,UINT mLength)
{
    FRESULT ff_result;
    UINT mWritenByteNum = 0; 
    ff_result = f_write(&file,mbuffer,mLength,&mWritenByteNum);
    if(mWritenByteNum < mLength)
        return 100;
    return ff_result;
}




//@brief 基于nRF52 SDCard卡 操作完成后，关闭文件
/*--------------------------------------------------------------------------*/
//<*	参数说明:
//<*	返回值说明:
//<*		FR_OK = 0,	其它失败
/*--------------------------------------------------------------------------*/
uint8_t ucSDCard_CloseFile(void)
{
	FRESULT ff_result;
	ff_result = f_close(&file);
	return ff_result;	
}







