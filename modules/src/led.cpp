// 新增文件，建议放 modules/src/led.cpp
// 风格：一张静态查找表把"逻辑灯"映射到"引脚"，加灯只改表。

#include "led.hpp"
#include "main.h"   // HAL 与 GPIO 定义
#include "gpio.h"

// 按实际硬件核对这两列：{端口号, 引脚号}，顺序对应 Led_t
static const GPIO_TypeDef *LED_PORT[3] = {GPIOH, GPIOH, GPIOH};
static const uint16_t LED_PIN[3] = {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};

// 点亮电平：高电平点亮填 GPIO_PIN_SET，低电平点亮填 GPIO_PIN_RESET
#define LED_ON_LEVEL  GPIO_PIN_SET
#define LED_OFF_LEVEL GPIO_PIN_RESET

void LED_On(Led_t led)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)LED_PORT[led], LED_PIN[led], LED_ON_LEVEL);
}

void LED_Off(Led_t led)
{
    HAL_GPIO_WritePin((GPIO_TypeDef *)LED_PORT[led], LED_PIN[led], LED_OFF_LEVEL);
}

void LED_Toggle(Led_t led)
{
    HAL_GPIO_TogglePin((GPIO_TypeDef *)LED_PORT[led], LED_PIN[led]);
}
