#include "DJIMotorHandler.hpp"
#include "bsp_can.hpp"
#include "DJIMotor.hpp"

#define M2006_REDUCTION_RATIO 36.0f     // M2006 减速比
#define M2006_ECD_PER_ROUND   8192.0f   // 编码器每转计数
#define CURRENT_LSB_AMP       (10.0f / 16384.0f) // 电流反馈 LSB -> A

void DJIMotorHandler::registerMotor(DJIMotor *motor, CAN_HandleTypeDef *hcan, uint16_t canId)
{
    // TODO: 校验 CAN 句柄和 0x201~0x208 ID，再登记到对应总线与槽位。
    // TODO: 按 ID 设置对应的 0x200/0x1FF 报文存在标志。
    // (void)motor;
    // (void)hcan;
    // (void)canId;
    if (motor == nullptr) return;
    if (hcan != &hcan1 && hcan != &hcan2) return;  // 只认这两路总线
    if (canId < 0x201 || canId > 0x208) return;    // 只支持电调标准 ID 段

    uint8_t bus = (hcan == &hcan1) ? 0 : 1;
    uint8_t slot = canId - 0x201;

    motor->canId = canId;   //把canid存入motor档案
    motor->hcan = hcan;  //把can总线存入motor档案
    DJIMotorList[bus][slot] = motor;  //把motor档案存入总登记表

    // 该槽位的电机由哪条命令帧承载
    if (bus == 0)
    {
        canId <= 0x204 ? CAN1_0x200_Exist = true : CAN1_0x1FF_Exist = true;
    }
    else
    {
        canId <= 0x204 ? CAN2_0x200_Exist = true : CAN2_0x1FF_Exist = true;
    }
}


void DJIMotorHandler::sendControlData()
{
    // TODO: 将各电机 currentSet 按大端序放入正确的 8 字节控制帧。
    // TODO: 仅发送包含已注册电机的报文；未完成前不发送 CAN 电流命令。
    for (int i = 0; i < 4; i++)
    {
        int16_t c1 = DJIMotorList[0][i] ? DJIMotorList[0][i]->currentSet : 0;
        int16_t c2 = DJIMotorList[1][i] ? DJIMotorList[1][i]->currentSet : 0;
        can1_send_data_0[2 * i]     = (uint8_t)(c1 >> 8);
        can1_send_data_0[2 * i + 1] = (uint8_t)(c1 & 0xFF);
        can2_send_data_0[2 * i]     = (uint8_t)(c2 >> 8);
        can2_send_data_0[2 * i + 1] = (uint8_t)(c2 & 0xFF);

        int16_t d1 = DJIMotorList[0][i + 4] ? DJIMotorList[0][i + 4]->currentSet : 0;
        int16_t d2 = DJIMotorList[1][i + 4] ? DJIMotorList[1][i + 4]->currentSet : 0;
        can1_send_data_1[2 * i]     = (uint8_t)(d1 >> 8);
        can1_send_data_1[2 * i + 1] = (uint8_t)(d1 & 0xFF);
        can2_send_data_1[2 * i]     = (uint8_t)(d2 >> 8);
        can2_send_data_1[2 * i + 1] = (uint8_t)(d2 & 0xFF);
    }

    if (CAN1_0x200_Exist) CAN_Transmit(&hcan1, 0x200, can1_send_data_0, 8);
    if (CAN1_0x1FF_Exist) CAN_Transmit(&hcan1, 0x1FF, can1_send_data_1, 8);
    if (CAN2_0x200_Exist) CAN_Transmit(&hcan2, 0x200, can2_send_data_0, 8);
    if (CAN2_0x1FF_Exist) CAN_Transmit(&hcan2, 0x1FF, can2_send_data_1, 8);
}

void DJIMotorHandler::updateFeedback(CAN_HandleTypeDef *hcan, uint8_t *rx_data, int index)
{
    // TODO: 校验总线、反馈数据和索引，再找到对应电机并调用 UpdateSensorData。
    // (void)hcan;
    // (void)rx_data;
    // (void)index;
    if (hcan != &hcan1 && hcan != &hcan2) return;
    if (rx_data == nullptr || index < 0 || index > 7) return;

    uint8_t bus = (hcan == &hcan1) ? 0 : 1;
    DJIMotor *motor = DJIMotorList[bus][index];
    if (motor == nullptr) return;

    UpdateSensorData(motor, rx_data);
    motor->AliveFlag++; // 心跳：AliveCheck 靠它判断在线
}


void DJIMotorHandler::UpdateSensorData(DJIMotor *motor, uint8_t *can_data)
{
    // TODO: 从 CAN 数据解析编码器、转速、电流和温度。
    // TODO: 处理编码器回绕，并按减速比换算输出轴位置和角速度。
    // (void)motor;
    // (void)can_data;

    DJIMotor::MotorFeedBack *fb = &motor->motorFeedback;
    
    fb->last_ecd = fb->ecd;
    fb->lastSpeedFdb = fb->speedFdb;
    fb->lastPositionFdb = fb->positionFdb;

    fb->ecd = (uint16_t)((can_data[0] << 8) | can_data[1]);
    fb->speed_rpm = (int16_t)((can_data[2] << 8) | can_data[3]);
    fb->currentFdb = (int16_t)((can_data[4] << 8) | can_data[5]) * CURRENT_LSB_AMP;
    fb->temperatureFdb = can_data[6];

    // 编码器差分：uint16 相减再转 int16，65536 的回绕自动变成正确的小负数/正数
    int16_t diff = (int16_t)(fb->ecd - fb->last_ecd);
    if (diff > 4096)  diff -= 8192;   // 正向跨边界：8191->0，-8191 修正为 +1
    if (diff < -4096) diff += 8192; 
    // 输出轴位置(rad)连续累加：先换转子角，再除减速比
    fb->positionFdb += (float)diff * (2.0f * 3.14159265358979f / M2006_ECD_PER_ROUND)
                       / M2006_REDUCTION_RATIO;
    // 输出轴角速度(rad/s)：rpm -> rad/s，再除减速比
    fb->speedFdb = (float)fb->speed_rpm * (2.0f * 3.14159265358979f / 60.0f)
                   / M2006_REDUCTION_RATIO;
}

void DJIMotorHandler::AllMotorAliveCheck()
{
    // TODO: 遍历已注册电机并调用 AliveCheck。
    for (int b = 0; b < 2; b++)
        for (int i = 0; i < 8; i++)
            if (DJIMotorList[b][i] != nullptr)
                DJIMotorList[b][i]->AliveCheck();
}
