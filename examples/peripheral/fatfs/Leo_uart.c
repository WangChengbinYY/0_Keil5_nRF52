/******************** (C) COPYRIGHT 2018 ���ɱ�********************
 * �ļ���  ��Leo_uart     
 * ƽ̨    ��nRF52832
 * ����    �����ڶ��弰ʵ��  
 * ����    �����ɱ�
**********************************************************************/



#include <Leo_uart.h>
#include "app_uart.h"
#include <pca10040.h>













#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 128                         /**< UART RX buffer size. */	
	
#define GNSS_DATA_HEAD	 0x4A													//GNSS���ݰ��ı�ʶ��	
#define GNSS_DATA_LEN    10														//GNSS���ݰ��ĳ���	


static uint8_t 	G_Uart_Rx_Data[UART_TX_BUF_SIZE] = {0};			//���ڽ��յ������ݴ��
static uint16_t	G_Uart_Rx_Number = 0;												//���ڵ�ǰ�ѽ������ݵĸ���

//static uint8_t 	G_Uart_Tx_Data[UART_TX_BUF_SIZE] = {0};			//����Ҫ���͵����ݴ��
//static uint16_t	G_Uart_Tx_Number = 0;												//���ڵ���Ҫ�������ݵĸ���





//-----------------------------------��ʽ----------------------------------------
/**********************************************************************************************
* ��  �� : 	Leo_UART_GPS_Init����
*          	��ʼ���� GPS ģ�����ӵĴ���
* ��  �� :	��ȡ�������ֽڵĸ���
* ����ֵ : 
* ע��	:		��ֹ���أ������ʣ�9600�����ֻ������ 2���ܽ� RX 15 TX16 ע��ģ���� ���� ����ķ���
***********************************************************************************************/ 
void Leo_UART_GPS_Init(void)
{
	
	
	
}



















//-----------------------------------����----------------------------------------






/**********************************************************************************************
* ��  �� : Leo_Uart_Event_Handler����
*          �����¼��ص�����,
* ��  �� : app_uart_evt_t
* ����ֵ : ��
***********************************************************************************************/ 

static void Leo_UART_Event_Handler(app_uart_evt_t * p_event)
{
	if (p_event->evt_type == APP_UART_DATA_READY)
	{
			//���ڽ��յ�����
			uint8_t cr;
			while(app_uart_get(&cr) == NRF_SUCCESS)
			{
				G_Uart_Rx_Data[G_Uart_Rx_Number] = cr;
				G_Uart_Rx_Number++;
				//������ 10�� �ַ�����ʼGNSS���ݽ���  ���飡��
//				if(G_Uart_Rx_Number >= 10)
//					leo_uart_decode_rxdata_GNSS();
				
				//���������������ֱ�Ӹ���֮ǰ�ѽ��յ�����
				if(G_Uart_Rx_Number == UART_TX_BUF_SIZE)
					G_Uart_Rx_Number = 0;
				//����ط�������ԸĽ�������һ������жϣ��������棬���� ���յ��˶��ٸ� �ֽڣ�����
			}

	}

}




/**********************************************************************************************
* ��  �� : Leo_UART_BSP_Init����
*          ���ڳ�ʼ����
								���ն˿ںţ�RX_PIN_NUMBER 
								���Ͷ˿ںţ�TX_PIN_NUMBER
* ��  �� : ��
* ����ֵ : ��
***********************************************************************************************/ 
void Leo_UART_BSP_Init(void)
{
	  uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          APP_UART_FLOW_CONTROL_DISABLED, //��ֹ����
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200//������115200
    };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,      
                         UART_TX_BUF_SIZE,
                         Leo_UART_Event_Handler,
                         APP_IRQ_PRIORITY_LOW,
                         err_code);

    APP_ERROR_CHECK(err_code);
}	


/**********************************************************************************************
* ��  �� : 	Leo_UART_GetData����
*          	��ȡ���յ��Ĵ�������
* ��  �� :	pBuffer_Get 			������������ָ��
						mBuffer_L	����Ĵ�С
						mGetNumber					��ȡ�������ֽڵĸ���
* ����ֵ : 0 ��ʾû���յ����ݣ�1 ��ʾ�յ�����
* ע��	:	������Ҫ�����ⲿ�жϣ������ô˺��������� ���ڻ����������������
***********************************************************************************************/ 
uint32_t Leo_UART_GetData(uint8_t * pBuffer_Get,uint16_t mBuffer_L,uint16_t * mGetNumbe)
{
	uint16_t i = 0;
	//-----------------------����û���յ�����-------------
	if(G_Uart_Rx_Number == 0)
		*mGetNumbe = 0;
	return 0;
	
	//-----------------------�������յ�����--------------
	//������ݵ������㹻��һ��ȫ��ȡ��ȥ��
	if(G_Uart_Rx_Number <= mBuffer_L)
	{
		for(i=0;i<G_Uart_Rx_Number;i++)
		{
			pBuffer_Get[i] = G_Uart_Rx_Data[i];
		}
		*mGetNumbe = G_Uart_Rx_Number;
		G_Uart_Rx_Number = 0;		
	}else{
	//������ݵ����鲻����ֻ����ȡһ���ֳ�ȥ	
		for(i=0;i<mBuffer_L;i++)
		{
			pBuffer_Get[i] = G_Uart_Rx_Data[i];
		}
		*mGetNumbe = mBuffer_L;
		
		//��ʣ�µ�������ǰ����		
		for(i=0;i<(G_Uart_Rx_Number-mBuffer_L);i++)
		{
			G_Uart_Rx_Data[i] = G_Uart_Rx_Data[mBuffer_L+i];
		}
		G_Uart_Rx_Number = G_Uart_Rx_Number-mBuffer_L;		
	}	
	return 1;
}




/**********************************************************************************************
* ��  �� : 	Leo_UART_SendData����
*          	�����ڷ�������
* ��  �� :	pBuffer_Send 			��Ҫ�������ݵ�����ָ��
						mBuffer_L	����Ĵ�С
* ����ֵ : err_code 0 ��ʾ���ͳɹ���
***********************************************************************************************/ 
uint32_t Leo_UART_SendData(uint8_t * pBuffer_Send,uint16_t mBuffer_L)
{
	uint32_t err_code;
	uint32_t i = 0;
	for(i=0;i<mBuffer_L;i++)
	{
		err_code = app_uart_put(pBuffer_Send[i]);
		if (err_code != NRF_SUCCESS)
			return err_code;
	}
	return err_code;
}





//---------------------�ŵ���Ľ���� ��ȥ��������������������������������������
/**********************************************************************************************
* ��  �� : Leo_UART_Decode_RxData_GNSS����
*          ���봮�ڽ��յ�������
* ��  �� : G_Uart_Rx_Data[];  G_Uart_Rx_Number 
* ����ֵ : ��
***********************************************************************************************/ 
static void Leo_UART_Decode_RxData_GNSS(void)
{
//	uint16_t i = 0;
//	uint16_t j = 0;
//	
//	if(G_Uart_Rx_Number == 0)
//		return;
//	
//	if(G_Uart_Rx_Number < GNSS_DATA_LEN)
//		return;
//	
//	//Ѱ�����ݰ���ͷ����  ��0x4A��
//	for(i=0;i<G_Uart_Rx_Number;i++)
//	{
//		if(G_Uart_Rx_Data[i] == GNSS_DATA_HEAD)
//			break;
//	}
//	//û���ҵ����ݰ��ı�ʶ������������ȫ����ȥ
//	if(i == G_Uart_Rx_Number)
//	{
//		G_Uart_Rx_Number = 0;
//		return;
//	}
//	//ȥ�����ݰ�ǰ�治��Ҫ������
//	if(i > 0)
//	{
//		for(j=0;j<(G_Uart_Rx_Number-i);j++)
//		{
//			G_Uart_Rx_Data[j] = G_Uart_Rx_Data[i+j];
//		}
//		G_Uart_Rx_Number = G_Uart_Rx_Number-i;
//		//�ж�ʣ�¶�������� �Ƿ�һ�����ݰ�������������򷵻�
//		if(G_Uart_Rx_Number < GNSS_DATA_LEN)
//			return;
//	}
//	//if(i == 0); �������յ������ݰ���һ�����Ǳ�ʶ�� 
//	
//	//����������ݵĵ�һ���ַ����Ǳ�ʶ�����������ݰ����ֽ������ڵ��� GNSS_DATA_LEN
//	//���Ƚ��� ���ݰ� �˴����� ֱ�Ӹ�ֵ�� ���ͻ���
//	for(j=0;j<GNSS_DATA_LEN;j++)
//	{
//		G_Uart_Tx_Data[j] = G_Uart_Rx_Data[j];		
//	}
//	G_Uart_Tx_Number = GNSS_DATA_LEN;
//	
//	leo_uart_send_data();
//	
//	
//	//��ʣ�µ��ֽ� ������ �������ݻ�����
//	if(G_Uart_Rx_Number == GNSS_DATA_LEN)
//	{
//		G_Uart_Rx_Number = 0;
//		return;
//	}else{
//		for(j=0;j<(G_Uart_Rx_Number-GNSS_DATA_LEN);j++)
//		{
//			G_Uart_Rx_Data[j] = G_Uart_Rx_Data[GNSS_DATA_LEN+j];
//		}
//		G_Uart_Rx_Number = G_Uart_Rx_Number-GNSS_DATA_LEN;
//		return;
//	}	
	
}




