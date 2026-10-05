// 对应仓库: threads/src/alivethread.cpp（该文件原本是空的，内容全部新增）
//
// 监控逻辑：心跳 = 被监控线程每循环一次 put 一次信号量。
// alive 线程用带超时的 get 等心跳：等到 = 活着，等不到 = 掉线。
// 灯效约定（按实际板子电平调整）：
//   蓝灯常亮      = 系统初始化成功
//   绿灯闪        = imu 线程活着；灭 = imu 掉线
//   红灯闪        = ctrl 线程活着；常亮 = ctrl 掉线（异常示警）

#include "threads.hpp"
#include "led.hpp"


void AliveThreadEntry(ULONG arg)
{
    (void)arg;
    UINT imu_status, ctrl_status;

    LED_On(LED_B); // 能跑到这里说明 Threads_Init 成功

    while (1)
    {
        // 各等 100 tick（100ms）；返回 TX_SUCCESS = 这期间有心跳
        imu_status  = tx_semaphore_get(&imu_alive_sem, 100);
        ctrl_status = tx_semaphore_get(&ctrl_alive_sem, 100);
        if(imu_run_enable == true)
        {
        if (imu_status == TX_SUCCESS)
        {
            LED_Toggle(LED_G);
        }
        else
        {
            LED_Off(LED_G);
        }

        if (ctrl_status != TX_SUCCESS)
        {
            LED_On(LED_R); // ctrl 掉线是异常，红灯常亮示警
        }

        // 正常时红灯随 ctrl 心跳闪；为区分"掉线常亮"，这里只在掉线时点亮，
        // 恢复后由下一次循环不再点亮——简化处理，掉了又好的瞬间灯会自己灭
        if (ctrl_status == TX_SUCCESS)
        {
            LED_Toggle(LED_R);
        }
    }
    //主动关闭imu线程
    else
    {
        LED_Off(LED_G);       //绿灯灭但不报警
    }

        tx_thread_sleep(100); // 灯效周期 100ms，别忘了睡觉
    }
}