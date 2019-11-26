/*
*********************************************************************************************************
*
*    ģ������ : FreeRTOS������ʵ��
*    �ļ����� : Leo_FreeRTOS_FootBLE
*    ��    �� : V1.0
*    ˵    �� : FreeRTOS������ʵ�����
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-17     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#ifndef LEO_FREERTOS_FOOTBLE_H
#define LEO_FREERTOS_FOOTBLE_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif


/*
 * �󲿷ִ���� ble_uart_c ���������� �ο�������  */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void db_disc_handler(ble_db_discovery_evt_t * p_evt);
void ble_nus_chars_received_uart_print(uint8_t * p_data, uint16_t data_len);
void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report);
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
void ble_stack_init(void);
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
void gatt_init(void);
void nus_c_init(void);    
void timer_init(void);
void db_discovery_init(void);

/*
 * ��������֮ǰ��׼������  */    
void vBLEFootScan_Prepare(void);    
/*
 * ����FreeRTOS����������  */  
void vBLEFootScan_Start(void * p_context); 
    
    

#ifdef __cplusplus
}
#endif


#endif 