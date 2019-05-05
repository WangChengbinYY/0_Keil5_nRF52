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
#include "Leo_FreeRTOS_TASK.h"

extern uint8_t     G_SDCard_FileIsOpen;   

int main(void)
{  
    ret_code_t erro_code = 0;
    uint8_t   mTest[10]={0};
//0. 初始化_LOG日志输出     
    erro_code |= NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO(("||Initialize||-->LOG----------->error  0x%x"),erro_code);

    
//配置ADIS采样频率
    //（2） GPIO管脚初始化    
    erro_code |= nrfx_gpiote_init();  
    
    erro_code |= ucTimerInitial_3();        //1ms 计时 
    erro_code |= ucTimerStart_3();     
 

    //（4） 初始化 IMU 
    //初始化 IMUA(U4) MPU92  里面包含了(1)初始化两个MPU9255,(2) ADIS
    erro_code |= ucIMU_INIT_MPU_ADIS();



    
    
//    G_SDCard_FileIsOpen = 1;    
//    erro_code |= ucINTInital_IMUB();    
//    ucINTStart_IMUB();    
    
    
////1. 建立任务       
//    error_code |= vTask_CreatTask();
//    NRF_LOG_INFO(("||Initialize||-->Task_Creat---->error  0x%x"),error_code);

////2. 启动_FreeRTOS 任务循环执行  
//    vTaskStartScheduler();
//    
////3. 任务循环出错  */
//    NRF_LOG_INFO("||Wrong    ||-->FreeRTOS-->Quite !!");
//    NRF_LOG_FLUSH();
    for (;;)
    {
        //APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
        __WFE();
    }    

}
