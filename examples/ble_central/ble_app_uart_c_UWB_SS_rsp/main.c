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
#include "Leo_TIMER.h"
#include "Leo_INT.h"
#include "Leo_FreeRTOS_FootBLE.h"
#include "Leo_FreeRTOS_TASK.h"
#include "Leo_SDCard.h"


#include "dw1001_dev.h"
#include "port_platform.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"


#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}


//--------------------------------------测试UWB代码----------------------------------


static dwt_config_t config = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    0,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (129 + 8 - 8)     /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


extern int ss_resp_run(void);

///* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
//#define PRE_TIMEOUT 1000

///* Delay between frames, in UWB microseconds. See NOTE 1 below. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 100 

///*Should be accurately calculated during calibration*/
//#define TX_ANT_DLY 16300
//#define RX_ANT_DLY 16456	

//--------------dw1000---end---------------








int main(void)
{
    // Initialize.
    bool erase_bonds;
    ret_code_t err_code;
    
/*1. 初始化_LOG日志输出 */    
    err_code = NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),err_code);
    NRF_LOG_FLUSH();
/*----------------------------------------------------------------------------*/  

    
    
//--------------------------------------测试UWB代码----------------------------------    
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); 		//irq
  
  /* Reset DW1000 */
  reset_DW1000(); 

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();			
  
  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
        NRF_LOG_INFO("||Initialize||-->UWB----------->error  1");
        NRF_LOG_FLUSH();
    //Init of DW1000 Failed
    while (1) {};
        
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  /* Configure DW1000. */
  dwt_configure(&config);

  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Set preamble timeout for expected frames. See NOTE 3 below. */
  //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
          
  dwt_setrxtimeout(0);    // set to NO receive timeout for this simple example   

  //-------------dw1000  ini------end---------------------------	

    // No RTOS task here so just call the main loop here.
    // Loop forever responding to ranging requests.
  
    NRF_LOG_INFO("||Initialize||-->UWB_Begin!!!------->");
    NRF_LOG_FLUSH();
  
    while (1)
    {
      ss_resp_run();
    }
    
    
    
    
    
    
    
    
    
    
///*2. 初始化_nrf_drv_clock FootBLE要用  */ 
//    err_code = nrf_drv_clock_init();
//    
//    /* 初始化 并启动 计时器TIMER4  FreeRTOS任务分析要用 */
//    #if configTIMER4_ENABLE    
//        ucTimerInitial_4();
//        ucTimerStart_4();
//    #endif    
//    
//    NRF_LOG_INFO(("||Initialize||-->nrf_drv_clock->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();
///*----------------------------------------------------------------------------*/     

///*3.  初始化_BLE服务 压力传感器串口客户端     
//    Create a FreeRTOS task for the BLE stack.
//    The task will run advertising_start() before entering its loop.*/    
//    vBLEFootScan_Prepare();    
//    nrf_sdh_freertos_init(vBLEFootScan_Start, &erase_bonds);
//    NRF_LOG_INFO("||Initialize||-->BLE Foot------>Finished!");
//    NRF_LOG_FLUSH();


///*任务建立_LOG日志 
//注意：此处必须放在这里，如果放在后面则不显示！！！！！ */    
//#if NRF_LOG_ENABLED
//    // Start execution.
//    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", configMINIMAL_STACK_SIZE, NULL, 1, &m_logger_thread))
//    {
//        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
//    }    
//#endif
///*----------------------------------------------------------------------------*/    
//    
///*4. 建立任务  */     
//    err_code = vTask_CreatTask();
//    NRF_LOG_INFO(("||Initialize||-->Task_Creat---->error  0x%x"),err_code);
//    NRF_LOG_FLUSH();

///*5. 启动_FreeRTOS 任务循环执行    
//    Start FreeRTOS scheduler.*/ 
//    vTaskStartScheduler();
//    
///*6. 任务循环出错  */
//    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
//    NRF_LOG_FLUSH();
//    for (;;)
//    {
//        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
//        __WFE();
//    }


}
