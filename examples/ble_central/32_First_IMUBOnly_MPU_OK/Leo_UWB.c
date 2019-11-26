/********************************************************************************************************
*
*    模块名称 : UWB 测距
*    文件名称 :Leo_UWB
*    版    本 : V1.0
*    说    明 : UWB 测距实现
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-03-12      WangCb   
*        V1.1    2019-03-17      WangCb   
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
********************************************************************************************************/


#include "Leo_UWB.h"

extern uint16_t         G_MicroSecond; 
extern uint32_t         G_GPSWeekSecond;      

/*=============================== 通用设定 ===================================*/
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4


/* 电磁波速度 m/s，这里小于光速，应该是考虑了不在真空中传播. */
#define SPEED_OF_LIGHT 299702547


/*=============================== 发起者设定 ===================================*/
//发起者的 发送数据结构
static uint8 INIT_tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
//发起者的 接收数据结构
static uint8 INIT_rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//发起者 接收数据缓存
#define INIT_RX_BUF_LEN 20
static uint8 INIT_rx_buffer[INIT_RX_BUF_LEN];
/* 发送数据包 计数器 */
static uint8 INIT_frame_seq_nb = 0;
    
    
    
/*=============================== 响应者设定 ===================================*/
//响应者的 接收数据结构
static uint8_t RSP_rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
//响应者的 发送数据结构
static uint8_t RSP_tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//响应者 接收数据缓存
#define RSP_RX_BUF_LEN 24
static uint8_t RSP_rx_buffer[RSP_RX_BUF_LEN];
// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file  延迟0.8ms发送   测试一下
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

    
 
    
static dwt_config_t config = {
    5,                /* Channel number. */
//    DWT_PRF_16M,
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



/*========================================================= 发起者实现 ====================================================*/

/*! ------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp 
*       fields of the response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*/
/* 发起端 应用 */
static void vSS_INIT_resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

/**
 * UWB 发起端 初始化   */
uint8_t ucSS_INIT_Initial(void)
{
    uint32 status_reg = 0;
    uint8_t error_code = 0;

    //确定已设置 GPIO管脚初始化
    if(!nrfx_gpiote_is_init())
    {
        NRF_LOG_INFO("   UWB  GPIO is not set"); 
        NRF_LOG_FLUSH(); 
        error_code |=nrfx_gpiote_init();
        if(error_code != 0)
            return error_code;
    }
    
    /* Reset DW1000 */
    reset_DW1000(); 
    
    /* Set SPI clock to 2MHz */
    port_set_dw1000_slowrate();  
    
    /* Init the DW1000 */        
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        //Init of DW1000 Failed
        NRF_LOG_INFO(" UWB initial is wrong!!!!");     
        NRF_LOG_FLUSH();
        return 1;
    }    
    
    // Set SPI to 8MHz clock
    port_set_dw1000_fastrate();     
   
    /* Configure DW1000. */
    dwt_configure(&config);    
    
    /* Register RX call-back. */
    //dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    //设置中断 接收成功 各种接收错误
    dwt_setinterrupt(DWT_INT_RFCG,1);
    //dwt_setinterrupt((DWT_INT_RFCG|configUWB_INIT_SYSMASK_ALL_RX_ERR),1);
    //dwt_setinterrupt((DWT_INT_RFCG|DWT_INT_RFTO),1);
    
    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);    
    
    //接收超时设定 5ms
    dwt_setrxtimeout(5000); // Maximum value timeout with DW1000 is 65ms 

    return error_code;
}

/**
 * UWB 发起端 启动测距   */
void vSS_INIT_Start(void)
{
    dwt_rxreset();
    
    //1.发送准备
    //(1)写入发送序号 256循环
    INIT_frame_seq_nb++;
    INIT_tx_poll_msg[ALL_MSG_SN_IDX] = INIT_frame_seq_nb;
    //(2)写入所要发送的数据
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(INIT_tx_poll_msg), INIT_tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(INIT_tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */  
    
    //2.开始发送 发送之后即可转入 待接收状态 延迟由dwt_setrxaftertxdelay()设定，此处不设定，不延迟等待
    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); 
 
}


/**
 *  UWB 发起端 接收反馈 并处理   */
uint8_t ucSS_INIT_Handler(uint16* pDistance,uint8_t* pNumber)
{
    uint32 status_reg = 0;
    uint8_t tNumber = 0;
    
    //读取状态信息
    status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
   
    //接收成功      
    if (status_reg & SYS_STATUS_RXFCG)
    {  
        uint32_t frame_len;  
        
        //清楚成功接收 事件标志位，以防下一个循环误判
        // It can also be cleared explicitly by writing a 1 to it.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        

        /* A frame has been received, read it into the local buffer. */
        //读取寄存器，获取接收到数据的长度
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        //将接收到的数据，存入缓存内        
        if (frame_len <= INIT_RX_BUF_LEN)
        {
            dwt_readrxdata(INIT_rx_buffer, frame_len, 0);
        }else
        {
            NRF_LOG_INFO("UWB INIT RX Len is Wrong!");
            NRF_LOG_FLUSH();
            return 1;
        }


        //这里有个问题，只有一对可以，如果有多对，相互干扰，如果成功接收到别的发送者的信号，怎么处理？？？？？
        /* Check that the frame is the expected response from the companion "SS TWR responder" example.
        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
        //将接收到的 信息中的 数据包序号 保存下来
        tNumber = INIT_rx_buffer[ALL_MSG_SN_IDX];
        //将数据包序号置0 和原始数据包对比，以确定是自己想要的数据包
        INIT_rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(INIT_rx_buffer, INIT_rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            //和发送的序号不对应
            if(tNumber != INIT_frame_seq_nb)
            {
                dwt_rxreset();
                NRF_LOG_INFO("UWB INIT Number is Wrong! TX %d, RX %d",INIT_frame_seq_nb,tNumber);
                NRF_LOG_FLUSH(); 
                return 1;                
            }
            
            uint32_t poll_tx_ts, resp_rx_ts, resp_tx_ts, poll_rx_ts;
            int32_t  rtd_init, rtd_resp;
            double tof;
            float clockOffsetRatio ;
            uint16_t tDistance = 0;
            /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
            poll_tx_ts = dwt_readtxtimestamplo32();
            resp_rx_ts = dwt_readrxtimestamplo32();

            /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
            clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

            /* Get timestamps embedded in response message. */
            vSS_INIT_resp_msg_get_ts(&INIT_rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
            vSS_INIT_resp_msg_get_ts(&INIT_rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

            /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
            rtd_init = resp_rx_ts - poll_tx_ts;
            rtd_resp = resp_tx_ts - poll_rx_ts;

            tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
            *pDistance = (uint16_t)(tof * SPEED_OF_LIGHT*1000.0);
            *pNumber = tNumber;
            return 0;
            
        }else
        {
            NRF_LOG_INFO("UWB INIT RX Different Message!");
            NRF_LOG_FLUSH(); 

            return 1;            
        }
    }
    
    if(status_reg & SYS_STATUS_ALL_RX_TO)
    {
        //TEST
        NRF_LOG_INFO("UWB INIT Time is OUT! ms %d S %d",G_MicroSecond,G_GPSWeekSecond);
        NRF_LOG_FLUSH(); 
        
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        
        return 1;
    }    
    else
    {
        //TEST
        NRF_LOG_INFO("UWB INIT  RX is Wrong!");
        NRF_LOG_FLUSH(); 
        
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        
        return 1;
    }
    return 1;
}


/*****************************************************************************************************************************************************
* INIT NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local initiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delay is calibratred and set correctly.
*
****************************************************************************************************************************************************/







/*========================================================= 响应者实现 ====================================================*/


/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
/* 响应端 应用  设置时间*/
static void vSS_RESP_resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
/* 响应端 应用  读取时间*/
static uint64 vSS_RESP_get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}


/* UWB 响应端 初始化   */
uint8_t ucSS_RESP_Initial(void)
{
    uint32 status_reg = 0;
    uint8_t error_code = 0;

    //确定已设置 GPIO管脚初始化
    if(!nrfx_gpiote_is_init())
    {
        NRF_LOG_INFO("   UWB  GPIO is not set"); 
        NRF_LOG_FLUSH(); 
        error_code |=nrfx_gpiote_init();
        if(error_code != 0)
            return error_code;
    }
    
    /* Reset DW1000 */
    reset_DW1000(); 
    
    /* Set SPI clock to 2MHz */
    port_set_dw1000_slowrate();  
    
    /* Init the DW1000 */        
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        //Init of DW1000 Failed
        NRF_LOG_INFO(" UWB initial is wrong!!!!");     
        NRF_LOG_FLUSH();
        return 1;
    }    
    
    // Set SPI to 8MHz clock
    port_set_dw1000_fastrate();     
   
    /* Configure DW1000. */
    dwt_configure(&config);   

    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    //dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
    //此处需要设定中断响应的事件  接收成功 接收错误 发送成功 发送失败 (不需要接收超时，因为一直在等待接收)
    dwt_setinterrupt(DWT_INT_RFCG | configUWB_RESP_SYSMASK_ALL_RX_ERR ,1);
   
    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);    
    
    //开始准备接收
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    
    return error_code;
}


/**
 *  UWB 接收端 接收反馈 并处理   */
uint8_t ucSS_RESP_Handler(void)
{
    uint8_t tNumber = 0;
    uint32 status_reg = 0;
    uint16_t tTime = 0;
    
    status_reg = dwt_read32bitreg(SYS_STATUS_ID); 
     
    if (status_reg & SYS_STATUS_RXFCG)
    {
        uint32 frame_len;
        
        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        
        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RSP_RX_BUF_LEN)
        {
            dwt_readrxdata(RSP_rx_buffer, frame_len, 0);
            
            //TEST
//            NRF_LOG_INFO("UWB RESP RX is OK! ms %d S %d",G_MicroSecond,G_GPSWeekSecond);
//            NRF_LOG_FLUSH(); 
        }else
        {
            NRF_LOG_INFO("UWB Resp RX Length is Wrong!");
            NRF_LOG_FLUSH(); 
            //接收非本系统的数据包
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            return 1;
        }
       
        tNumber = RSP_rx_buffer[ALL_MSG_SN_IDX];
        RSP_rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(RSP_rx_buffer, RSP_rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
        {
            uint32_t resp_tx_time;
            uint64_t poll_rx_ts,resp_tx_ts;
            int ret;
            
            /* Retrieve poll reception timestamp. */
            poll_rx_ts = vSS_RESP_get_rx_timestamp_u64();
            
            
            /* Compute final message transmission time. See NOTE 7 below. */
            resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
            dwt_setdelayedtrxtime(resp_tx_time);
            
            /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
            resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            
            /* Write all timestamps in the final message. See NOTE 8 below. */
            vSS_RESP_resp_msg_set_ts(&RSP_tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
            vSS_RESP_resp_msg_set_ts(&RSP_tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

            /* Write and send the response message. See NOTE 9 below. */
            RSP_tx_resp_msg[ALL_MSG_SN_IDX] = tNumber;
            dwt_writetxdata(sizeof(RSP_tx_resp_msg), RSP_tx_resp_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
            dwt_writetxfctrl(sizeof(RSP_tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
            ret = dwt_starttx(DWT_START_TX_DELAYED);
            
            if (ret != DWT_SUCCESS)
            {
                //发送失败的话
                NRF_LOG_INFO("UWB Resp TX Fault! ms %d S %d",G_MicroSecond,G_GPSWeekSecond);
                NRF_LOG_FLUSH(); 
                dwt_rxreset(); 
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
                return 1;   
            }
        }else
        {
            NRF_LOG_INFO("UWB RESP RX Different Message!");
            NRF_LOG_FLUSH(); 
            dwt_rxreset(); 
        }
    }else
    {
        NRF_LOG_INFO("UWB Resp RX Fault!");
        NRF_LOG_FLUSH(); 
        //失败的话
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, configUWB_RESP_SYSSTATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();   
    }
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    return 0;
    
}






/*****************************************************************************************************************************************************
* RESP NOTES:
*
* 1. This is the task delay when using FreeRTOS. Task is delayed a given number of ticks. Useful to be able to define this out to see the effect of the RTOS
*    on timing.
* 2. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 3 below.
*     - byte 7/8: source address, see NOTE 3 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 4. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 5. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 6. POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. For slower platforms where the SPI is at a slower speed 
*    or the processor is operating at a lower frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
*    Knowing the exact time when the responder is going to send its response is vital for time of flight calculation. The specification of the time of 
*    respnse must allow the processor enough time to do its calculations and put the packet in the Tx buffer. So more time required for a slower
*    system(processor).
* 7. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
*    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
*    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
*    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
*    8 bits.
* 8. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
*    time-of-flight computation) can be handled by a 32-bit subtraction.
* 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
*10. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*    DW1000 API Guide for more details on the DW1000 driver functions.
*
****************************************************************************************************************************************************/
 






