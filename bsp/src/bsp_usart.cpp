// #include "bsp_usart.hpp"
// 对应仓库: bsp/src/bsp_usart.cpp
//
// 设计说明：同 bsp_can——用注册回调把"收到数据交给谁"留给上层决定。
// 接收用 HAL_UARTEx_ReceiveToIdle_DMA：DMA 收 + IDLE 空闲中断，
// 一包数据收完（或缓冲满）自动回调 HAL_UARTEx_RxEventCallback，
// 不用手写 IDLE 标志判断，这是处理"不定长协议"（如裁判系统串口）的标准做法。

#include "bsp_usart.hpp"
#include "usart.h"
#include "dma.h"

#define RX_BUF_SIZE 128

static uint8_t usart1_rx_buf[RX_BUF_SIZE];
static uint8_t usart6_rx_buf[RX_BUF_SIZE];


/**
 * @brief HAL 回调：ReceiveToIdle 收完一包（IDLE 触发或缓冲满）时调用
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1 )
    {
        (void)Size;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buf, RX_BUF_SIZE);
    }
    else if (huart == &huart6)
    {
        (void)Size;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_rx_buf, RX_BUF_SIZE);
    }
}





void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  // TODO: 配置 USART6 的 DMA 接收、发送及空闲中断，并启动接收。
  __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, usart6_rx_buf, RX_BUF_SIZE);
};

void USART1_Init()
{
  // TODO: 配置 USART1 的 DMA 接收、发送及空闲中断，并启动接收。
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_rx_buf, RX_BUF_SIZE);
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  // TODO: 检查参数，并按 mode 选择阻塞、DMA 或中断发送。
  
  if (huart == nullptr || pData == nullptr || Size == 0)
    {
        return;
    }
    switch (mode)
    {
    case USART_MODE_BLOCK:
        HAL_UART_Transmit(huart, pData, Size, 100); // 100ms 超时
        break;
    case USART_MODE_DMA:
        HAL_UART_Transmit_DMA(huart, pData, Size);
        break;
    case USART_MODE_IT:
        HAL_UART_Transmit_IT(huart, pData, Size);
        break;
    }
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  // TODO: 选择接收方式并启动 HAL UART 接收。
  
  if (huart == nullptr || pData == nullptr || Size == 0)
    {
        return;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, pData, Size);
}
