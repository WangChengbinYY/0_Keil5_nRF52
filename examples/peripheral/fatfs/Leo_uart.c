/******************** (C) COPYRIGHT 2018 王成宾********************
 * 文件名  ：Leo_uart     
 * 平台    ：nRF52832
 * 描述    ：串口定义及实现  
 * 作者    ：王成宾
**********************************************************************/



#include <Leo_uart.h>
#include "app_uart.h"
#include <pca10040.h>













#define UART_TX_BUF_SIZE 128                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 128                         /**< UART RX buffer size. */	
	
#define GNSS_DATA_HEAD	 0x4A													//GNSS数据包的标识符	
#define GNSS_DATA_LEN    10														//GNSS数据包的长度	


static uint8_t 	G_Uart_Rx_Data[UART_TX_BUF_SIZE] = {0};			//串口接收到的数据存放
static uint16_t	G_Uart_Rx_Number = 0;												//串口当前已接收数据的个数

//static uint8_t 	G_Uart_Tx_Data[UART_TX_BUF_SIZE] = {0};			//串口要发送的数据存放
//static uint16_t	G_Uart_Tx_Number = 0;												//串口当将要发送数据的个数





//-----------------------------------正式----------------------------------------
/**********************************************************************************************
* 描  述 : 	Leo_UART_GPS_Init函数
*          	初始化和 GPS 模块连接的串口
* 入  参 :	获取到数据字节的个数
* 返回值 : 
* 注意	:		禁止流控，波特率：9600，因此只设置了 2个管脚 RX 15 TX16 注意模块上 输入 输出的方向
***********************************************************************************************/ 
void Leo_UART_GPS_Init(void)
{
	
	
	
}



















//-----------------------------------测试----------------------------------------






/**********************************************************************************************
* 描  述 : Leo_Uart_Event_Handler函数
*          串口事件回调函数,
* 入  参 : app_uart_evt_t
* 返回值 : 无
***********************************************************************************************/ 

static void Leo_UART_Event_Handler(app_uart_evt_t * p_event)
{
	if (p_event->evt_type == APP_UART_DATA_READY)
	{
			//串口接收到数据
			uint8_t cr;
			while(app_uart_get(&cr) == NRF_SUCCESS)
			{
				G_Uart_Rx_Data[G_Uart_Rx_Number] = cr;
				G_Uart_Rx_Number++;
				//接收满 10个 字符，开始GNSS数据解码  试验！！
//				if(G_Uart_Rx_Number >= 10)
//					leo_uart_decode_rxdata_GNSS();
				
				//接收数据溢出，就直接覆盖之前已接收的数据
				if(G_Uart_Rx_Number == UART_TX_BUF_SIZE)
					G_Uart_Rx_Number = 0;
				//这个地方后面可以改进，设置一个软件中断，告诉外面，串口 接收到了多少个 字节！！！
			}

	}

}




/**********************************************************************************************
* 描  述 : Leo_UART_BSP_Init函数
*          串口初始化：
								接收端口号：RX_PIN_NUMBER 
								发送端口号：TX_PIN_NUMBER
* 入  参 : 无
* 返回值 : 无
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
          APP_UART_FLOW_CONTROL_DISABLED, //禁止流控
          false,
          UART_BAUDRATE_BAUDRATE_Baud115200//波特率115200
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
* 描  述 : 	Leo_UART_GetData函数
*          	获取接收到的串口数据
* 入  参 :	pBuffer_Get 			存放数据数组的指针
						mBuffer_L	数组的大小
						mGetNumber					获取到数据字节的个数
* 返回值 : 0 表示没有收到数据，1 表示收到数据
* 注意	:	这里需要触发外部中断，来调用此函数，否则 串口缓存区会溢出！！！
***********************************************************************************************/ 
uint32_t Leo_UART_GetData(uint8_t * pBuffer_Get,uint16_t mBuffer_L,uint16_t * mGetNumbe)
{
	uint16_t i = 0;
	//-----------------------串口没有收到数据-------------
	if(G_Uart_Rx_Number == 0)
		*mGetNumbe = 0;
	return 0;
	
	//-----------------------串口有收到数据--------------
	//存放数据的数组足够大，一次全都取出去！
	if(G_Uart_Rx_Number <= mBuffer_L)
	{
		for(i=0;i<G_Uart_Rx_Number;i++)
		{
			pBuffer_Get[i] = G_Uart_Rx_Data[i];
		}
		*mGetNumbe = G_Uart_Rx_Number;
		G_Uart_Rx_Number = 0;		
	}else{
	//存放数据的数组不够大，只能先取一部分出去	
		for(i=0;i<mBuffer_L;i++)
		{
			pBuffer_Get[i] = G_Uart_Rx_Data[i];
		}
		*mGetNumbe = mBuffer_L;
		
		//把剩下的数据往前对齐		
		for(i=0;i<(G_Uart_Rx_Number-mBuffer_L);i++)
		{
			G_Uart_Rx_Data[i] = G_Uart_Rx_Data[mBuffer_L+i];
		}
		G_Uart_Rx_Number = G_Uart_Rx_Number-mBuffer_L;		
	}	
	return 1;
}




/**********************************************************************************************
* 描  述 : 	Leo_UART_SendData函数
*          	往串口发送数据
* 入  参 :	pBuffer_Send 			所要发送数据的数组指针
						mBuffer_L	数组的大小
* 返回值 : err_code 0 表示发送成功！
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





//---------------------放到别的解码库 中去！！！！！！！！！！！！！！！！！！！
/**********************************************************************************************
* 描  述 : Leo_UART_Decode_RxData_GNSS函数
*          解码串口接收到的数据
* 入  参 : G_Uart_Rx_Data[];  G_Uart_Rx_Number 
* 返回值 : 无
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
//	//寻找数据包的头文字  “0x4A”
//	for(i=0;i<G_Uart_Rx_Number;i++)
//	{
//		if(G_Uart_Rx_Data[i] == GNSS_DATA_HEAD)
//			break;
//	}
//	//没有找到数据包的标识符，所有数据全部舍去
//	if(i == G_Uart_Rx_Number)
//	{
//		G_Uart_Rx_Number = 0;
//		return;
//	}
//	//去掉数据包前面不需要的数据
//	if(i > 0)
//	{
//		for(j=0;j<(G_Uart_Rx_Number-i);j++)
//		{
//			G_Uart_Rx_Data[j] = G_Uart_Rx_Data[i+j];
//		}
//		G_Uart_Rx_Number = G_Uart_Rx_Number-i;
//		//判断剩下对齐的数据 是否够一个数据包，如果不够，则返回
//		if(G_Uart_Rx_Number < GNSS_DATA_LEN)
//			return;
//	}
//	//if(i == 0); 代表，接收到的数据包第一个就是标识符 
//	
//	//下面就是数据的第一个字符就是标识符，并且数据包总字节数大于等于 GNSS_DATA_LEN
//	//首先解析 数据包 此处试验 直接赋值给 发送缓存
//	for(j=0;j<GNSS_DATA_LEN;j++)
//	{
//		G_Uart_Tx_Data[j] = G_Uart_Rx_Data[j];		
//	}
//	G_Uart_Tx_Number = GNSS_DATA_LEN;
//	
//	leo_uart_send_data();
//	
//	
//	//将剩下的字节 保留在 接收数据缓存内
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




