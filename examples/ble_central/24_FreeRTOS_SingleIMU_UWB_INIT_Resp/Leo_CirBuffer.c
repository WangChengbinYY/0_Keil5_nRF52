/*
*********************************************************************************************************
*
*    ģ������ : ���λ�����
*    �ļ����� : Leo_CirBuffer
*    ��    �� : V1.0
*    ˵    �� : ʵ�ֻ��λ������Ķ�д����
*
*    �޸ļ�¼ :
*        �汾��    ����          ����     
*        V1.0    2019-01-20     Leo   
*
*    Copyright (C), 2018-2020, Department of Precision Instrument Engineering ,Tsinghua University  
*
*********************************************************************************************************
*/


#include "Leo_CirBuffer.h"

/*------------------------------------------------------------*/
/* ���λ����� ��ʼ��                                           */
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
 *���λ����� ��������                                          *
 *  pucData:    Ҫ�������ݵĵ�ַ                               *
 *  usLength:   Ҫ�������ݵĳ���                               *
 *------------------------------------------------------------*/    
uint8_t ucCBuffer_Save(CirBuffer_t * txCBuffer,const int8_t * pucData, uint16_t usLength)
{
    /*�ж��Ƿ�����*/
    if((usLength + txCBuffer->usUsedNum) > txCBuffer->usBufferLength)
        return 1;
    
    
    if((txCBuffer->pucSave + usLength -1) <= (txCBuffer->pucEnd))
    {
        /*(1) �������ݲ���Խ������β��*/
        memcpy(txCBuffer->pucSave,pucData,usLength);
        txCBuffer->pucSave      = txCBuffer->pucSave + usLength;
        txCBuffer->usUsedNum    = txCBuffer->usUsedNum + usLength;        
    }else
    {
        /*(2) �������ݿ�Խ������β��*/
        uint16_t tusNumFirstSave = txCBuffer->pucEnd - txCBuffer->pucSave + 1;
        uint16_t tusNumSecondSave = usLength - tusNumFirstSave;
        /* ����First */
        memcpy(txCBuffer->pucSave,pucData,tusNumFirstSave);
        /* ����Second */
        memcpy(txCBuffer->pucStart,pucData+tusNumFirstSave,tusNumSecondSave);
        txCBuffer->pucSave      = txCBuffer->pucSave + tusNumSecondSave;
        txCBuffer->usUsedNum    = txCBuffer->usUsedNum + usLength;         
    }
    
    return 0;
    
}
   
/*------------------------------------------------------------*
 *���λ����� ��ȡ����                                          *
 *  pucData:    ��ȡ���ݴ�ŵĵ�ַ                             *
 *  usLength:   Ҫ��ȡ���ݵĳ���                               *
 *------------------------------------------------------------*/  
uint8_t ucCBuffer_Load(CirBuffer_t * txCBuffer,uint16_t usGetLength,int8_t * pucData)
{
    if((txCBuffer->usUsedNum < usGetLength) || (txCBuffer->usUsedNum == 0))
        return 1;
    
    //��ȡȫ������
//    if(usGetLength == 0)
//    {
//        if(txCBuffer->pucSave >)
//    }
    
    
    
    
    
    return 0;
}


/*------------------------------------------------------------*/
/* ���λ����� ɾ��                                           */
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



