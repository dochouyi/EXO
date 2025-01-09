import math
from collections import deque
import numpy as np
from scipy.optimize import curve_fit


class SineTrajectoryHandler:
    """
    正弦轨迹生成与估计器。

    该类支持生成正弦轨迹的位移、速度和加速度，同时可以通过输入的时间和位移数据拟合正弦轨迹参数（振幅、频率和相位）。
    """

    def __init__(self, amplitude=1.0, frequency=1.0, phase=0.0, window_size=5):
        """
        初始化正弦轨迹生成器和估计器。

        :param amplitude: 初始振幅
        :param frequency: 初始频率
        :param phase: 初始相位
        :param window_size: 用于拟合的滑动窗口大小
        """
        # 轨迹生成参数
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase

        # 滑动窗口存储时间和位移数据
        self.window_size = window_size
        self.time_window = deque(maxlen=window_size)
        self.position_data = deque(maxlen=window_size)

    # 正弦轨迹生成相关方法
    def get_position(self, t):
        """
        获取给定时间 t 的位移。
        """
        return self.amplitude * math.cos(2 * math.pi * self.frequency * t + self.phase)

    def get_velocity(self, t):
        """
        获取给定时间 t 的速度。
        """
        return -self.amplitude * 2 * math.pi * self.frequency * math.sin(2 * math.pi * self.frequency * t + self.phase)

    def get_acceleration(self, t):
        """
        获取给定时间 t 的加速度。
        """
        omega_squared = (2 * math.pi * self.frequency) ** 2
        return -self.amplitude * omega_squared * math.cos(2 * math.pi * self.frequency * t + self.phase)


    # 轨迹估计相关方法
    def update_data(self, time, position):
        """
        更新滑动窗口中的时间和位移数据，并尝试拟合轨迹参数。

        :param time: 时间数据
        :param position: 位移数据
        """
        self.time_window.append(time)
        self.position_data.append(position)


    def fit_and_update(self):
        """
        对当前窗口中的数据进行正弦拟合，然后重置参数。

        :return: (amplitude, frequency, phase)
        """
        def sinusoidal_model(t, amplitude, frequency, phase):
            return amplitude * np.cos(2 * np.pi * frequency * t + phase)

        if len(self.time_window) < 3:
            raise ValueError("数据不足，无法进行正弦拟合。至少需要 3 个数据点。")

        # 转换为 NumPy 数组以便计算
        time_data = np.array(self.time_window)
        position_data = np.array(self.position_data)

        # 初始参数估计
        initial_guess = [1, 0.1, 0]

        # 使用 curve_fit 进行拟合
        params = curve_fit(sinusoidal_model, time_data, position_data, p0=initial_guess)
        try:
            self.amplitude, self.frequency, self.phase = params[0]

            # self.amplitude = min(self.amplitude, 2)
            # self.frequency = min(self.frequency, 0.5)
            # print("frequency:", self.frequency)

        except ValueError as e:
            print(f"轨迹拟合失败: {e}")





if __name__ == "__main__":
    # 创建 SineTrajectoryHandler 实例
    handler = SineTrajectoryHandler(amplitude=2.0, frequency=0.5, phase=np.pi / 4, window_size=50)

    # 生成一段时间内的正弦轨迹
    time_points = np.linspace(0, 10, 100)  # 生成 0 到 10 秒的时间点
    positions = [handler.get_position(t) for t in time_points]  # 计算每个时间点的位移

    # 打印生成的位移、速度和加速度
    print("时间点\t位移\t\t速度\t\t加速度")
    for t in time_points[:5]:  # 仅显示前 5 个时间点的结果
        position = handler.get_position(t)
        velocity = handler.get_velocity(t)
        acceleration = handler.get_acceleration(t)
        print(f"{t:.2f}\t{position:.4f}\t{velocity:.4f}\t{acceleration:.4f}")

    # 模拟更新滑动窗口数据
    for t, pos in zip(time_points, positions):
        handler.update_data(t, pos)

    # 拟合正弦轨迹参数
    try:
        handler.fit_and_update()
        print("\n拟合后的参数：")
        print(f"振幅: {handler.amplitude:.4f}")
        print(f"频率: {handler.frequency:.4f}")
        print(f"相位: {handler.phase:.4f}")
    except ValueError as e:
        print(f"拟合失败: {e}")