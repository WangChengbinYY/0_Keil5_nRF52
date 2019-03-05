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


#include "Leo_CirBuffer.h"

/*------------------------------------------------------------*/
/* 环形缓冲区 初始化                                           */
/*------------------------------------------------------------*/    
uint8_t ucCBuffer_INIT(CirBuffer_t * txCBuffer,uint16_t usLength)
{
    if(txCBuffer->pucStart != NULL)
    {
        ucCBuffer_Delet(txCBuffer);
    }
    
    txCBuffer->pucStart = (uint8_t *)malloc(usLength);    
    if(txCBuffer->pucStart != NULL)
    {
        txCBuffer->pucEnd           = txCBuffer->pucStart + usLength -1;
        txCBuffer->usBufferLength   = usLength;
        txCBuffer->usUsedNum        = 0;
        txCBuffer->pucSave          = txCBuffer->pucStart;
        txCBuffer->pucLoad          = txCBuffer->pucStart;
        return 0;
    }
    else
        return 1;    
}


/*------------------------------------------------------------*
 *环形缓冲区 存入数据                                          *
 *  pucData:    要存入数据的地址                               *
 *  usLength:   要存入数据的长度                               *
 *------------------------------------------------------------*/    
uint8_t ucCBuffer_Save(CirBuffer_t * txCBuffer,const int8_t * pucData, uint16_t usLength)
{
    /*判断是否会溢出*/
    if((usLength + txCBuffer->usUsedNum) > txCBuffer->usBufferLength)
        return 1;
    
    
    if((txCBuffer->pucSave + usLength -1) <= (txCBuffer->pucEnd))
    {
        /*(1) 存入数据不跨越缓存区尾部*/
        memcpy(txCBuffer->pucSave,pucData,usLength);
        txCBuffer->pucSave      = txCBuffer->pucSave + usLength;
        txCBuffer->usUsedNum    = txCBuffer->usUsedNum + usLength;        
    }else
    {
        /*(2) 存入数据跨越缓存区尾部*/
        uint16_t tusNumFirstSave = txCBuffer->pucEnd - txCBuffer->pucSave + 1;
        uint16_t tusNumSecondSave = usLength - tusNumFirstSave;
        /* 存入First */
        memcpy(txCBuffer->pucSave,pucData,tusNumFirstSave);
        /* 存入Second */
        memcpy(txCBuffer->pucStart,pucData+tusNumFirstSave,tusNumSecondSave);
        txCBuffer->pucSave      = txCBuffer->pucSave + tusNumSecondSave;
        txCBuffer->usUsedNum    = txCBuffer->usUsedNum + usLength;         
    }
    
    return 0;
    
}
   
/*------------------------------------------------------------*
 *环形缓冲区 读取数据                                          *
 *  pucData:    读取数据存放的地址                             *
 *  usLength:   要读取数据的长度                               *
 *------------------------------------------------------------*/  
uint8_t ucCBuffer_Load(CirBuffer_t * txCBuffer,uint16_t usGetLength,int8_t * pucData)
{
    if((txCBuffer->usUsedNum < usGetLength) || (txCBuffer->usUsedNum == 0))
        return 1;
    
    //读取全部数据
//    if(usGetLength == 0)
//    {
//        if(txCBuffer->pucSave >)
//    }
    
    
    
    
    
    return 0;
}


/*------------------------------------------------------------*/
/* 环形缓冲区 删除                                           */
/*------------------------------------------------------------*/    
uint8_t ucCBuffer_Delet(CirBuffer_t * txCBuffer)
{
    if(txCBuffer->pucStart != NULL)
    {
        free(txCBuffer->pucStart);
        txCBuffer->pucStart         = NULL;
        txCBuffer->pucEnd           = NULL;
        txCBuffer->usBufferLength   = 0;
        txCBuffer->usUsedNum        = 0;
        txCBuffer->pucSave          = NULL;
        txCBuffer->pucLoad          = NULL;
    }else
    {
        txCBuffer->pucEnd           = NULL;
        txCBuffer->usBufferLength   = 0;
        txCBuffer->usUsedNum        = 0;
        txCBuffer->pucSave          = NULL;
        txCBuffer->pucLoad          = NULL;
    }
    return 0;
}



