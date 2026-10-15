// 新增文件，建议放 threads/include/threads.hpp
// 设计思路：
//   - 三个线程三个入口函数，一个 Threads_Init() 统一创建；
//   - 共享对象（IMU、电机、心跳信号量）声明在这，定义在 threads.cpp，
//     各线程文件 include 它即可，不用互相 include。

#ifndef THREADS_HPP
#define THREADS_HPP

#include "tx_api.h"
#include "BMI088.hpp"
#include "M2006.hpp"
//栈内存
#define ALIVE_STACK_SIZE 512
#define IMU_STACK_SIZE   1024
#define CTRL_STACK_SIZE  1024
//优先级
#define PRIO_ALIVE  15 // 最低优先级：监控类，让路给控制
#define PRIO_IMU    10
#define PRIO_CTRL   5  // 控制环优先级最高
//线程
extern TX_THREAD alive_thread;
extern TX_THREAD imu_thread;
extern TX_THREAD ctrl_thread;
//缓冲区
extern UCHAR alive_stack[ALIVE_STACK_SIZE];
extern UCHAR imu_stack[IMU_STACK_SIZE];
extern UCHAR ctrl_stack[CTRL_STACK_SIZE];

// 心跳信号量：imu/ctrl 每循环一次 put 一次，alive 用带超时的 get 监控
extern TX_SEMAPHORE imu_alive_sem;
extern TX_SEMAPHORE ctrl_alive_sem;

// 跨线程共享的硬件对象
extern BMI088::cBMI088 bmi088;
extern M2006 test_motor;
//使能
// extern volatile bool imu_run_enable;
//函数
void AliveThreadEntry(ULONG arg);
void IMUThreadEntry(ULONG arg);
void CtrlThreadEntry(ULONG arg);

#ifdef __cplusplus
extern "C" {
#endif
void Threads_Init(void);
#ifdef __cplusplus
}
#endif // 在 tx_application_define 的 USER CODE 区调用

#endif
