// 新增文件，建议放 threads/src/imuthread.cpp
//
// 流程：Config -> 校验芯片 ID -> Calibrate -> 循环读数 + 恒温控制。
// 任何一步失败：点亮红灯后挂起本线程（alive 线程会因为等不到心跳而报警，形成联动）。

#include "threads.hpp"
#include "led.hpp"

// extern volatile bool imu_run_enable;   // 运行开关，定义在 threads.cpp，初值 true

#define IMU_TARGET_TEMP 40.0f // ℃，C 板 IMU 常见恒温点
// volatile uint8_t dbg_pwr_ctrl = 0, dbg_pwr_conf = 0, dbg_acc_status = 0, dbg_gyro_lpm1 = 0,dbg_acc_conf = 0;
static uint8_t dbg_pwr_ctrl, dbg_pwr_conf, dbg_acc_conf, dbg_err_reg;
static uint8_t dbg_raw[6];
static float dbg_st_x, dbg_st_y, dbg_st_z;
uint8_t st = ACC_SELF_TEST_POS;  
void IMUThreadEntry(ULONG arg)
{
    (void)arg;

    bmi088.Config();
    

    // uint8_t tmp;
    // bmi088.ReadReg(BMI088::BMI088_CS_ACC,  0x7C, &tmp, 1); dbg_pwr_ctrl  = tmp;
    // bmi088.ReadReg(BMI088::BMI088_CS_ACC,  0x7B, &tmp, 1); dbg_pwr_conf  = tmp;
    // bmi088.ReadReg(BMI088::BMI088_CS_ACC,  0x03, &tmp, 1); dbg_acc_status = tmp;
    // bmi088.ReadReg(BMI088::BMI088_CS_GYRO, 0x11, &tmp, 1); dbg_gyro_lpm1 = tmp;
    // uint8_t t;
    // bmi088.ReadReg(BMI088::BMI088_CS_ACC, 0x40, &t, 1);
    // dbg_acc_conf = t;  
    // uint8_t v;
    // v = 0xB6; bmi088.WriteReg(BMI088::BMI088_CS_ACC, 0x7E, &v, 1); tx_thread_sleep(150);  // 软复位
    // v = 0x04; bmi088.WriteReg(BMI088::BMI088_CS_ACC, 0x7C, &v, 1); tx_thread_sleep(100);  // 使能
    // v = 0x00; bmi088.WriteReg(BMI088::BMI088_CS_ACC, 0x7B, &v, 1); tx_thread_sleep(10);   // 正常模式
    // v = 0x01; bmi088.WriteReg(BMI088::BMI088_CS_ACC, 0x41, &v, 1); tx_thread_sleep(5);    // 量程±6G
    // v = 0xA8; bmi088.WriteReg(BMI088::BMI088_CS_ACC, 0x40, &v, 1); tx_thread_sleep(5);    // 200Hz
    
bmi088.ReadReg(BMI088::BMI088_CS_ACC, ACC_PWR_CTRL_ADDR, &dbg_pwr_ctrl, 1);
bmi088.ReadReg(BMI088::BMI088_CS_ACC, ACC_PWR_CONF_ADDR, &dbg_pwr_conf,  1);
bmi088.ReadReg(BMI088::BMI088_CS_ACC, ACC_CONF_ADDR,     &dbg_acc_conf,  1);
bmi088.ReadReg(BMI088::BMI088_CS_ACC, ACC_ERR_REG_ADDR,  &dbg_err_reg,   1);
bmi088.ReadReg(BMI088::BMI088_CS_ACC, ACC_X_LSB_ADDR,    dbg_raw,        6);
    bmi088.VerifyAccChipID();
    bmi088.VerifyGyroChipID();
bmi088.WriteReg(BMI088::BMI088_CS_ACC, ACC_SELF_TEST_ADDR, &st, 1);
tx_thread_sleep(100);   
bmi088.ReadAccData(&bmi088.acc_data);            // 等自检稳定
dbg_st_x = bmi088.acc_data.x;     // 此时循环已在跑，直接抄当前值
dbg_st_y = bmi088.acc_data.y;
dbg_st_z = bmi088.acc_data.z;
st = ACC_SELF_TEST_OFF;           // 0x00，关掉
bmi088.WriteReg(BMI088::BMI088_CS_ACC, ACC_SELF_TEST_ADDR, &st, 1);
    if (bmi088.self_test.INIT_ERR)
    {
        LED_On(LED_R); // 初始化失败，红灯示警
        tx_thread_suspend(tx_thread_identify()); // 挂起自己，心跳停止 -> alive 报警
    }

    bmi088.Calibrate();

    while (1)
    {
        // if (!imu_run_enable)
        // {
        //     tx_thread_suspend(tx_thread_identify());  // 自我停车
        // }
        
        bmi088.ReadAccData(&bmi088.acc_data);
        bmi088.ReadGyroData(&bmi088.gyro_data);
        bmi088.ReadAccTemperature(&bmi088.acc_data.temperature);
        bmi088.TemperatureControl(IMU_TARGET_TEMP);

        tx_semaphore_put(&imu_alive_sem); // 心跳
        tx_thread_sleep(1);               // 1 tick = 1ms（tick 1000/s），即 1kHz 采样
    }
}