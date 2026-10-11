// 新增文件，建议放 threads/src/ctrlthread.cpp
//
// 单电机双环验证流程（作业检查项）：
//   1. 注册电机 -> RELAX_MODE：确认 currentSet=0，电机松手能转动
//   2. SPD_MODE：给恒定 speedSet，验证速度环收敛
//   3. POS_MODE：给阶跃 positionSet，验证双环收敛到目标位置
// 换阶段时改下面的宏或加条件编译，一次只验证一个功能。

#include "threads.hpp"
#include "led.hpp"
#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include "can.h"

#define MOTOR_CAN  (&hcan1)
#define MOTOR_ID   0x201

// 验证阶段：0=RELAX  1=速度环  2=位置速度双环
#define TEST_STAGE 2

// 速度环验证目标：输出轴 5 rad/s；双环验证目标：转 3.14 rad
#define TEST_SPEED  5.0f
#define TEST_POS    3.14159265358979f

uint32_t loop_cnt = 0;

void CtrlThreadEntry(ULONG arg)
{
    // (void)arg;

    // MotorCanRx_Bind(); // bsp CAN 回调 -> 电机反馈
    DJIMotorHandler::Instance()->registerMotor(&test_motor, MOTOR_CAN, MOTOR_ID);  //注册测试电机
    // M2006::MotorStateTypedef AliveCheck();
    test_motor.controlMode = DJIMotor::RELAX_MODE;  //设置松开模式
DJIMotor::MotorControlModeType target_mode = DJIMotor::RELAX_MODE;
#if (TEST_STAGE == 1)
    target_mode = DJIMotor::SPD_MODE;
    test_motor.speedSet = TEST_SPEED;
    tx_thread_sleep(100); 

#elif (TEST_STAGE == 2)
    target_mode = DJIMotor::POS_MODE;
    test_motor.positionSet = TEST_POS;
#endif

    while (1)
    {
        // 离线保护：电机掉了立刻松开，不给幽灵电流
        test_motor.controlMode =
            (test_motor.MotorState == DJIMotor::MOTOR_ONLINE)
                ? target_mode
                : DJIMotor::RELAX_MODE;

        test_motor.setOutput();
        DJIMotorHandler::Instance()->sendControlData();
        if (++loop_cnt >= 100)              // 每 100ms 查一次在线状态
        {
            loop_cnt = 0;
            DJIMotorHandler::Instance()->AllMotorAliveCheck();
        }

        tx_semaphore_put(&ctrl_alive_sem);
        tx_thread_sleep(1); // 1ms 控制周期
    }
}
