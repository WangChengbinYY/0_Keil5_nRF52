/********************************************************************************************************
*
*    ģ������ : UWB ���
*    �ļ����� : Leo_UWB
*    ��    �� : V1.0
*    ˵    �� : UWB ���ʵ��
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-03-12     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
********************************************************************************************************/


#include "Leo_UWB.h"

/*=============================== �������趨 ===================================*/
//�����ߵ� �������ݽṹ
static uint8 INIT_tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
//�����ߵ� �������ݽṹ
static uint8 INIT_rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//������ �������ݻ���
#define INIT_RX_BUF_LEN 20
static uint8 INIT_rx_buffer[INIT_RX_BUF_LEN];
/* �������ݰ� ������ */
static uint8 INIT_frame_seq_nb = 0;
    
    
    
/*=============================== ��Ӧ���趨 ===================================*/
//��Ӧ�ߵ� �������ݽṹ
static uint8_t RSP_rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
//��Ӧ�ߵ� �������ݽṹ
static uint8_t RSP_tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//��Ӧ�� �������ݻ���
#define RSP_RX_BUF_LEN 24
static uint8_t RSP_rx_buffer[RSP_RX_BUF_LEN];
    
    
    
/*=============================== ͨ���趨 ===================================*/
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4



/* ��Ų��ٶ� m/s������С�ڹ��٣�Ӧ���ǿ����˲�������д���. */
#define SPEED_OF_LIGHT 299702547
    
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







/*=============================== ������ʵ�� ===================================*/

/*! ------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp 
*       fields of the response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*/
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
 * �����ߵ� ��ʼ��   */
uint8_t ucSS_INIT_Initial(void)
{
    uint32 status_reg = 0;
    uint8_t error_code = 0;
    
    /* Reset DW1000 */
    reset_DW1000(); 
    
    /* Set SPI clock to 2MHz */
    port_set_dw1000_slowrate();  

    /* Init the DW1000 */
    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        //Init of DW1000 Failed
        return 1;
    }   

    // Set SPI to 8MHz clock
    port_set_dw1000_fastrate();    
    
    /* Configure DW1000. */
    dwt_configure(&config);    
    
    /* Apply default antenna delay value. See NOTE 2 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);    
    
    //���ճ�ʱ�趨 5ms
    dwt_setrxtimeout(5000); // Maximum value timeout with DW1000 is 65ms  
}

/**
 * ������  �������   */
uint8_t ucSS_INIT_RUN(uint16* pDistance,uint8_t* pNumber)
{
    uint32 status_reg = 0;
    uint8_t error_code = 0;
    
//1.����׼��
    //(1)д�뷢����� 256ѭ��
    INIT_tx_poll_msg[ALL_MSG_SN_IDX] = INIT_frame_seq_nb;
    //(2)д����Ҫ���͵�����
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(INIT_tx_poll_msg), INIT_tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(INIT_tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */  
    
//2.�������� ����֮�󼴿�ת�� ������״̬ �ӳ���dwt_setrxaftertxdelay()�趨���˴����趨�����ӳٵȴ�
    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
 
//3.���ͺ󣬵ȴ�����
    /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
    //SYS_STATUS_RXFCG ���ճɹ���SYS_STATUS_ALL_RX_TO ���ճ�ʱ��SYS_STATUS_ALL_RX_ERR ���մ���
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {};    
    
    #if 1  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif
        
//4.�յ�����     
    //(1)���ճɹ�      
    if (status_reg & SYS_STATUS_RXFCG)
    {		
        uint32_t frame_len;  
        
        //����ɹ����� �¼���־λ���Է���һ��ѭ������
        // It can also be cleared explicitly by writing a 1 to it.
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        

        /* A frame has been received, read it into the local buffer. */
        //��ȡ�Ĵ�������ȡ���յ����ݵĳ���
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        //�����յ������ݣ����뻺����        
        if (frame_len <= INIT_RX_BUF_LEN)
        {
          dwt_readrxdata(INIT_rx_buffer, frame_len, 0);
        }

        //�����и����⣬ֻ��һ�Կ��ԣ�����ж�ԣ��໥���ţ�����ɹ����յ���ķ����ߵ��źţ���ô������������
        /* Check that the frame is the expected response from the companion "SS TWR responder" example.
        * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
        //�����յ��� ��Ϣ�е� ���ݰ���� ��������
        *pNumber = INIT_rx_buffer[ALL_MSG_SN_IDX];
        //�����ݰ������0 ��ԭʼ���ݰ��Աȣ���ȷ�����Լ���Ҫ�����ݰ�
        INIT_rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(INIT_rx_buffer, INIT_rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
        {	           
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
            
            INIT_frame_seq_nb++;
            return 0;
            
        }
    }
    else
    {
        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        
        INIT_frame_seq_nb++;
        return 1;
    }
    
    return 1;
}











/*****************************************************************************************************************************************************
* NOTES:
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










