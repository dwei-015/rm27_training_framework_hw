#include "M2006.hpp"

M2006::M2006()
{
    // TODO: 根据 M2006 手册设置减速比、允许电流和速度/位置环 PID 参数。
    // 基类初始化为松开模式、零电流；完成配置前不允许驱动电机。
    gearBox = GearBox_M2006;
    maxCurrent = 3000;

    // 速度环：输出直接是电流给定，maxOut 给最大电流，maxIOut 给电流的 1/4 防饱和
    speedPid = PID(500.0f, 5.0f, 800.0f, (float)maxCurrent, 2500.0f, PID_POSITION);
    // 位置环：输出是"期望速度(rad/s)"，由速度环去追，maxOut 给最大期望速度
    positionPid = PID(3.0f, 0.0f, 0.0f, 300.0f, 0.0f, PID_POSITION);
    
}

void M2006::setOutput()
{
    // TODO: 分别实现松开、速度和位置模式；位置模式需要串联位置环与速度环。
    // TODO: 将 PID 输出限制在 maxCurrent 范围内后写入 currentSet。
    currentSet = 0;
    switch (controlMode)
    {
    case RELAX_MODE:
        currentSet = 0;
        break;

    case SPD_MODE: // 单环：速度环
        speedPid.ref = speedSet;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)Numeric::LimitABS(speedPid.result, (float)maxCurrent);
        break;

    case POS_MODE: // 双环串级：位置环输出当速度环目标
        positionPid.ref = positionSet;
        positionPid.fdb = motorFeedback.positionFdb;
        positionPid.UpdateResult();

        speedPid.ref = positionPid.result;
        speedPid.fdb = motorFeedback.speedFdb;
        speedPid.UpdateResult();
        currentSet = (int16_t)Numeric::LimitABS(speedPid.result, (float)maxCurrent);
        break;
    }
}

M2006::MotorStateTypedef M2006::AliveCheck()
{
    // TODO: 比较 AliveFlag 与 Pre_AliveFlag，更新在线状态并保存本次计数。
    // 反馈中断每收一帧会++ AliveFlag；两次检查之间完全没动 = 掉线
    if (AliveFlag == Pre_AliveFlag)
    {
        MotorState = MOTOR_OFFLINE;
    }
    else
    {
        MotorState = MOTOR_ONLINE;
    }
    Pre_AliveFlag = AliveFlag;
    return MotorState;
    
}
