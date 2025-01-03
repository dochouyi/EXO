import numpy as np
from collections import deque



class PolynomialTrajectoryHandler:
    """
    多项式轨迹生成器与拟合器
    """
    def __init__(self, coefficients, window_size=50, degree=None):
        """
        初始化多项式系数和拟合器
        :param coefficients: 多项式系数，从高次到低次排列，例如 [a, b, c] 表示 ax^2 + bx + c
        :param window_size: 用于拟合的滑动窗口大小
        :param degree: 多项式拟合的阶数（如果需要动态拟合）
        """
        self.coefficients = coefficients
        self.window_size = window_size
        self.degree = degree if degree is not None else len(coefficients) - 1

        # 滑动窗口存储时间和位移数据
        self.time_window = deque(maxlen=window_size)
        self.position_data = deque(maxlen=window_size)

    # 多项式轨迹生成相关方法
    def get_position(self, t):
        """
        获取给定时间 t 的位移。
        """
        return np.polyval(self.coefficients, t)

    def get_velocity(self, t):
        """
        获取给定时间 t 的速度。
        """
        derivative_coefficients = np.polyder(self.coefficients)
        return np.polyval(derivative_coefficients, t)

    def get_acceleration(self, t):
        """
        获取给定时间 t 的加速度。
        """
        second_derivative_coefficients = np.polyder(self.coefficients, 2)
        return np.polyval(second_derivative_coefficients, t)

    # 数据更新与拟合相关方法
    def update_data(self, time, position):
        """
        更新滑动窗口中的时间和位移数据，并尝试拟合多项式参数。
        :param time: 时间数据
        :param position: 位移数据
        """
        self.time_window.append(time)
        self.position_data.append(position)

    def fit_and_update(self):
        """
        对当前窗口中的数据进行多项式拟合，然后重置系数。
        """
        if len(self.time_window) < self.degree + 1:
            raise ValueError(f"数据不足，无法进行多项式拟合。至少需要 {self.degree + 1} 个数据点。")

        # 转换为 NumPy 数组以便计算
        time_data = np.array(self.time_window)
        position_data = np.array(self.position_data)

        # 使用 NumPy 的 polyfit 进行多项式拟合
        try:
            fitted_coefficients = np.polyfit(time_data, position_data, self.degree)
            self.coefficients = fitted_coefficients
        except Exception as e:
            print(f"多项式拟合失败: {e}")






if __name__ == "__main__":

    # 创建 PolynomialTrajectoryHandler 实例
    # 假设初始多项式为 f(t) = 2t^2 + 3t + 1
    initial_coefficients = [2, 3, 1]
    handler = PolynomialTrajectoryHandler(coefficients=initial_coefficients, window_size=50)

    # 生成一段时间内的多项式轨迹
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

    # 拟合多项式参数
    try:
        handler.fit_and_update()
        print("\n拟合后的多项式系数：")
        print(handler.coefficients)
    except ValueError as e:
        print(f"拟合失败: {e}")
