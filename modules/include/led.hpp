// 新增文件，建议放 modules/include/led.hpp + modules/src/led.cpp
// （作业要求"自行实现 LED 模块"：GPIO 直连 HAL 即可，不需要进 bsp 层）
//
// 使用前先核对（你的 CubeMX 配置）：
//   - 三个灯的引脚（当前 ioc: PH10 / PH11 / PH12）
//   - 点亮电平：ioc 里初始为 RESET，若你的板子是低电平点亮，
//     则 RESET=亮，下面的 On/Off 定义要反过来

#ifndef LED_HPP
#define LED_HPP

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LED_R = 0,
    LED_G = 1,
    LED_B = 2,
} Led_t;

void LED_On(Led_t led);
void LED_Off(Led_t led);
void LED_Toggle(Led_t led);

#ifdef __cplusplus
}
#endif

#endif