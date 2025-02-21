#include <malloc.h>
#include "string.h"
#include "bsp_usart.h"
#include "usart.h"
#include "main.h"

//extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
//extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void uart_send_data(uint8_t *buffer, uint16_t length, uint8_t id)
{
    uint8_t *send_buffer = (uint8_t*)malloc(length + 4);
    send_buffer[0] = 'h';
    send_buffer[1] = id;
    send_buffer[2] = length;
    memcpy(send_buffer + 3, buffer, length);
    send_buffer[length + 3] = 'j';

    huart1.gState = HAL_UART_STATE_READY;
    hdma_usart1_tx.State = HAL_DMA_STATE_READY;
    __HAL_UNLOCK(&hdma_usart1_tx);

    HAL_UART_Transmit_DMA(&huart1, send_buffer, length + 4);
    free(send_buffer);
}

void usart1_init(uint8_t*rx_buf,uint16_t dma_buf_num)
{
//    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, dma_buf_num);
//    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

//    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);//空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//空闲中断

    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while (hdma_usart1_rx.Instance->CR&DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_HISR_TCIF5);

    hdma_usart1_rx.Instance->PAR=(uint32_t)&(USART1->DR);
    hdma_usart1_rx.Instance->M0AR=(uint32_t)(rx_buf);
    hdma_usart1_rx.Instance->NDTR=dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_usart1_rx);//使能串口dma接收


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//    __HAL_UART_ENABLE_IT(&huart6,UART_IT_TC);
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
//    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
//    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }
    //clear flag
    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);
    //Set data address
    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    //设置对应DMA数据流传输的数据量大小
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);
    //Enable DMA
    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}




void uart6_tx_one_byte(uint8_t* data){
    while (!huart6.Instance->SR&UART_FLAG_TC);//判断串口6的TC标志位是否置1 置1则退出循环
    HAL_UART_Transmit(&huart6,data,1,1);
}

void uart6_tx_mul_byte_dma(uint8_t*data,uint16_t len){

    HAL_UART_Transmit_DMA(&huart6,data,len);
}