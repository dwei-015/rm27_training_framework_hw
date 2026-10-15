//
// Created by cosmosmount on 2025/9/2.
//

#include "bsp_pwm.hpp"
//一次打开实际使用的多个定时器
void PWM_Init(void)
{
    // TODO: 根据实际使用的定时器和通道完成 PWM 初始化。
    PWM_Start(&htim1, TIM_CHANNEL_1); // 舵机
    PWM_Start(&htim10, TIM_CHANNEL_1);  // BMI088 加热电阻，BMI088 初始化前必须先跑这行
}
//单独开启某个定时器
void PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 启动指定定时器通道的 PWM，并检查返回状态。
    if (HAL_TIM_PWM_Start(htim, Channel) != HAL_OK)
    {
        Error_Handler();   // 启动失败进错误处理
    }
}

void PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel)
{
    // TODO: 调用 HAL 停止指定定时器通道的 PWM。
   if (HAL_TIM_PWM_Stop(htim, Channel) != HAL_OK)
    {
        Error_Handler();   // 启动失败进错误处理
    }
}

void PWM_SetPeriod(TIM_HandleTypeDef *htim, float period_s)
{
    // TODO: 根据定时器时钟和预分频值，将秒转换为 ARR 并更新周期。
   // 先求这颗定时器的真实时钟：APB 预分频 ≠ 1 时，定时器时钟 = PCLK × 2。
    // F407 上 TIM2/3/4/5/6/7/12/13/14 挂 APB1，其余（TIM1/8/9/10/11）挂 APB2。
    const bool on_apb1 = (htim->Instance == TIM2)  || (htim->Instance == TIM3)  ||
                         (htim->Instance == TIM4)  || (htim->Instance == TIM5)  ||
                         (htim->Instance == TIM6)  || (htim->Instance == TIM7)  ||
                         (htim->Instance == TIM12) || (htim->Instance == TIM13) ||
                         (htim->Instance == TIM14);

    const uint32_t pclk = on_apb1 ? HAL_RCC_GetPCLK1Freq() : HAL_RCC_GetPCLK2Freq();
    const uint32_t ppre = on_apb1 ? (RCC->CFGR & RCC_CFGR_PPRE1)
                                  : (RCC->CFGR & RCC_CFGR_PPRE2);
    const uint32_t div1 = on_apb1 ? RCC_CFGR_PPRE1_DIV1 : RCC_CFGR_PPRE2_DIV1;
    const uint32_t tim_clk = (ppre == div1) ? pclk : pclk * 2;

    // 计数频率 = 定时器时钟 / (预分频 + 1)，周期换算成 ARR
    const uint32_t cnt_freq = tim_clk / (htim->Instance->PSC + 1);
    __HAL_TIM_SET_AUTORELOAD(htim, (uint32_t)(period_s * (float)cnt_freq) - 1);
}

void PWM_SetDutyRatio(TIM_HandleTypeDef *htim, float dutyratio, uint32_t channel)
{
    // TODO: 将 [0, 1] 占空比换算为比较值，写入指定通道。
    if (dutyratio < 0.0f) dutyratio = 0.0f;
    if (dutyratio > 1.0f) dutyratio = 1.0f;
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(dutyratio * (float)(htim->Instance->ARR + 1)));
}
