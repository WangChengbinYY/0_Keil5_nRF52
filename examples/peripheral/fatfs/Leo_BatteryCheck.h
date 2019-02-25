/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_BatteryCheck     
 * 平台    ：nRF52832 
 * 描述    ：通过计时器实现的时钟，用于所有传感器的时间同步  
 * 作者    ：王成宾
**********************************************************************/

#ifndef Leo_BATTERYCHECK_H
#define Leo_BATTERYCHECK_H


#include "Leo_nRF52_config.h"
#include "nrf_drv_saadc.h"
#include "nrfx_saadc.h"


#ifdef __cplusplus
extern "C" {
#endif

//@brief 电量检测初始化
/*--------------------------------------------------------------------------*/
//<*参数说明:    无
//<*返回值说明:
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它					读取失败
/*--------------------------------------------------------------------------*/
uint8_t Leo_BatteryCheck_Initial(void);	


//@brief 电量检测 
/*--------------------------------------------------------------------------*/
//<*参数说明:    无
//<*返回值说明: 
//<*		NRF_SUCCESS		读取成功 (0)
//<*		其它					读取失败
/*--------------------------------------------------------------------------*/
uint8_t Leo_BatteryCheck(float* mVolt);	

    
#ifdef __cplusplus
}
#endif


#endif  /* Leo_BatteryCheck.h */	   