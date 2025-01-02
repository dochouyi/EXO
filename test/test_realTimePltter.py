import time
import math
from realTimePlotter import RealTimePlotter


# 测试代码
def test_real_time_plotter():
    plotter = RealTimePlotter(window_size=40, update_interval=10)  # 设置横坐标窗口大小为 40 秒，绘图频率为每 10 步更新一次

    for t in range(2000):  # 模拟 2000 个时间步
        time_val = t * 0.1
        position = math.sin(time_val * 0.1)  # 模拟位置
        velocity = math.cos(time_val * 0.1)  # 模拟速度
        torque = math.sin(time_val * 0.5) * 0.5  # 模拟转矩
        external_torque = math.cos(time_val * 0.3) * 0.3  # 模拟外部转矩

        plotter.update(time_val, position, velocity, torque, external_torque)
        time.sleep(0.0001)  # 增加一个小的延迟来模拟实时性

    plotter.finalize()

if __name__ == "__main__":
    test_real_time_plotter()
