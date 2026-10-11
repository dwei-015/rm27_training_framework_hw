// 新增文件，建议放 threads/src/threads.cpp —— 线程的"装配车间"

#include "threads.hpp"
#include "bsp.hpp"
#include "bsp_pwm.hpp"
//线程
TX_THREAD alive_thread;
TX_THREAD imu_thread;
TX_THREAD ctrl_thread;
//缓冲区
UCHAR alive_stack[ALIVE_STACK_SIZE];
UCHAR imu_stack[IMU_STACK_SIZE];
UCHAR ctrl_stack[CTRL_STACK_SIZE];
//信号量
TX_SEMAPHORE imu_alive_sem;
TX_SEMAPHORE ctrl_alive_sem;
//硬件
BMI088::cBMI088 bmi088;
M2006 test_motor;

volatile bool imu_run_enable = true;

void Threads_Init(void)
{
    bsp_Init();   // CAN 滤波+启动+接收中断、串口 DMA —— 电机反馈的生命线
    PWM_Init(); 
    // 计数信号量，容量 1：心跳天然"最新一次有效"，积压无意义
    tx_semaphore_create(&imu_alive_sem, (CHAR *)"imu_alive", 0);
    tx_semaphore_create(&ctrl_alive_sem, (CHAR *)"ctrl_alive", 0);

    tx_thread_create(&alive_thread, (CHAR *)"alive", AliveThreadEntry, 0,
                     alive_stack, ALIVE_STACK_SIZE, PRIO_ALIVE, PRIO_ALIVE,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&imu_thread, (CHAR *)"imu", IMUThreadEntry, 0,
                     imu_stack, IMU_STACK_SIZE, PRIO_IMU, PRIO_IMU,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&ctrl_thread, (CHAR *)"ctrl", CtrlThreadEntry, 0,
                     ctrl_stack, CTRL_STACK_SIZE, PRIO_CTRL, PRIO_CTRL,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
}

// 在 board/AZURE_RTOS/App/app_azure_rtos.c 的
// USER CODE BEGIN tx_application_define 区加一行：Threads_Init();