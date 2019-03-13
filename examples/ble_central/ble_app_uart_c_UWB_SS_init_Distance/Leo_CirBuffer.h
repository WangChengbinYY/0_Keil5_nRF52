/*
*********************************************************************************************************
*
*    模块名称 : 环形缓冲区
*    文件名称 : Leo_CirBuffer
*    版    本 : V1.0
*    说    明 : 实现环形缓冲区的读写操作
*
*    修改记录 :
*        版本号    日期          作者     
*        V1.0    2019-01-20     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/



#ifndef LEO_CIRBUFFER_H
#define LEO_CIRBUFFER_H

#include "Leo_Includes.h"


#ifdef __cplusplus
extern "C" {
#endif

/**
 * 环形缓冲区 结构体定义 
 */    
typedef struct
{
    uint16_t    usBufferLength;                         //环形缓存区大小
    uint16_t    usUsedNum;                              //环形缓存区 内已使用的大小
    int8_t*     pucStart;                               //缓存区的起始地址
    int8_t*     pucEnd;                                 //缓存区结束地址

    int8_t*     pucSave;                                //缓存区中当前存储的位置
    int8_t*     pucLoad;                                //读取缓存区的位置
  
} CirBuffer_t;    
    
    
/*------------------------------------------------------------*
 *环形缓冲区 初始化                                            *
 *  成功返回0；失败返回1；                                     *
 *------------------------------------------------------------*/    
uint8_t ucCBuffer_INIT(CirBuffer_t * txCBuffer,uint16_t usLength);


/*------------------------------------------------------------*
 *环形缓冲区 存入数据                                          *
 *  pucData:    要存入数据的地址                               *
 *  usLength:   要存入数据的长度                               *
 *  成功返回0；失败返回1；                                     *
 *  (1) 先进先出模式FIFO                                       *
 *  (2) 这里仅考虑溢出不覆盖的情况，以后改进可以输入参数，选择    *
 *      是否覆盖旧的数据                                       * 
 *------------------------------------------------------------*/    
uint8_t ucCBuffer_Save(CirBuffer_t * txCBuffer,const int8_t * pucData, uint16_t usLength);
   
/*------------------------------------------------------------*
 *环形缓冲区 读取数据                                          *
 *  txCBuffer:    环形缓存区                                   *
 *  usGetLength:  要读取数据的长度,输入0代表读取现有的全部数据  * 
 *  pucData:        读取数据存放的地址                         *
 *  成功返回0；失败返回1(代表没有那么数据)；                    *
 *------------------------------------------------------------*/  
uint8_t ucCBuffer_Load(CirBuffer_t * txCBuffer,uint16_t usGetLength,int8_t * pucData);


/*------------------------------------------------------------*
 *环形缓冲区 初始化                                            *
 *  成功返回0；失败返回1；                                     *
 *------------------------------------------------------------*/    
uint8_t ucCBuffer_Delet(CirBuffer_t * txCBuffer);

    
    
        
#ifdef __cplusplus
}
#endif


#endif 