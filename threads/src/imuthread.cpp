// 新增文件，建议放 threads/src/imuthread.cpp
//
// 流程：Config -> 校验芯片 ID -> Calibrate -> 循环读数 + 恒温控制。
// 任何一步失败：点亮红灯后挂起本线程（alive 线程会因为等不到心跳而报警，形成联动）。

#include "threads.hpp"
#include "led.hpp"
extern volatile bool imu_run_enable;   // 运行开关，定义在 threads.cpp，初值 true

#define IMU_TARGET_TEMP 40.0f // ℃，C 板 IMU 常见恒温点

void IMUThreadEntry(ULONG arg)
{
    (void)arg;

    bmi088.Config();
    bmi088.VerifyAccChipID();
    bmi088.VerifyGyroChipID();

    if (bmi088.self_test.INIT_ERR)
    {
        LED_On(LED_R); // 初始化失败，红灯示警
        tx_thread_suspend(tx_thread_identify()); // 挂起自己，心跳停止 -> alive 报警
    }

    bmi088.Calibrate();

    while (1)
    {
        if (!imu_run_enable)
        {
            tx_thread_suspend(tx_thread_identify());  // 自我停车
        }
        bmi088.ReadAccData(&bmi088.acc_data);
        bmi088.ReadGyroData(&bmi088.gyro_data);
        bmi088.ReadAccTemperature(&bmi088.acc_data.temperature);
        bmi088.TemperatureControl(IMU_TARGET_TEMP);

        tx_semaphore_put(&imu_alive_sem); // 心跳
        tx_thread_sleep(1);               // 1 tick = 1ms（tick 1000/s），即 1kHz 采样
    }
}