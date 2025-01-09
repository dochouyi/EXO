import time
import math
from utils.realTimePlotterBase import RealTimePlotterBase


class RealTimePlotterMul4(RealTimePlotterBase):
    def __init__(self):
        """
        初始化 RealTimePlotterMul4 子类，默认布局为 4 行 1 列
        """
        data_labels = ["Position", "Velocity", "Torque", "External Torque"]
        layout = (4, 1)  # 设置子图为 4 行 1 列
        y_lims = {
            "Position": (-10, 10),
            "Velocity": (-50, 50),
            "Torque": (-20, 20),
            "External Torque": (-15, 15),
        }
        window_size = 40
        update_interval = 10
        super().__init__(data_labels, window_size, update_interval, layout, y_lims, fig_width=10)

    def update_data(self, time, position, velocity, torque, external_torque):
        data_values = {
            "Position": position,
            "Velocity": velocity,
            "Torque": torque,
            "External Torque": external_torque,
        }
        self.update(time, **data_values)


class RealTimePlotterMul4X2(RealTimePlotterBase):
    def __init__(self):
        """
        初始化 RealTimePlotterMul4 子类，默认布局为 4 行 1 列
        """
        data_labels = ["Position_Left", "Velocity_Left", "Torque_Left", "External Torque_Left", "Position_Right",
                       "Velocity_Right", "Torque_Right", "External Torque_Right"]
        layout = (4, 2)  # 设置子图为 4 行 1 列
        y_lims = {
            "Position_Left": (-10, 10),
            "Velocity_Left": (-50, 50),
            "Torque_Left": (-20, 20),
            "External Torque_Left": (-15, 15),
            "Position_Right": (-10, 10),
            "Velocity_Right": (-50, 50),
            "Torque_Right": (-20, 20),
            "External Torque_Right": (-15, 15),
        }
        window_size = 40
        update_interval = 10
        super().__init__(data_labels, window_size, update_interval, layout, y_lims, fig_width=15)

    def update_data(self, time, position_l, velocity_l, torque_l, external_torque_l, position_r, velocity_r, torque_r,
                    external_torque_r):
        data_values = {
            "Position_Left": position_l,
            "Velocity_Left": velocity_l,
            "Torque_Left": torque_l,
            "External Torque_Left": external_torque_l,
            "Position_Right": position_r,
            "Velocity_Right": velocity_r,
            "Torque_Right": torque_r,
            "External Torque_Right": external_torque_r,
        }
        self.update(time, **data_values)


class RealTimePlotterMul3(RealTimePlotterBase):
    def __init__(self):
        """
        初始化 RealTimePlotterMul4 子类，默认布局为 4 行 1 列
        """
        data_labels = ["Position", "Velocity", "Torque"]
        layout = (3, 1)  # 设置子图为 4 行 1 列
        y_lims = {
            "Position": (-10, 10),
            "Velocity": (-50, 50),
            "Torque": (-20, 20),
        }
        window_size = 40
        update_interval = 10
        super().__init__(data_labels, window_size, update_interval, layout, y_lims, fig_width=10)

    def update_data(self, time, position, velocity, torque):
        data_values = {
            "Position": position,
            "Velocity": velocity,
            "Torque": torque,
        }
        self.update(time, **data_values)


class RealTimePlotterMul3X2(RealTimePlotterBase):
    def __init__(self):
        """
        初始化 RealTimePlotterMul4 子类，默认布局为 4 行 1 列
        """
        data_labels = ["Position_Left", "Velocity_Left", "Torque_Left",
                       "Position_Right", "Velocity_Right", "Torque_Right"]
        layout = (3, 2)  # 设置子图为 4 行 1 列
        y_lims = {
            "Position_Left": (-10, 10),
            "Velocity_Left": (-50, 50),
            "Torque_Left": (-20, 20),
            "Position_Right": (-10, 10),
            "Velocity_Right": (-50, 50),
            "Torque_Right": (-20, 20),
        }
        window_size = 40
        update_interval = 10
        super().__init__(data_labels, window_size, update_interval, layout, y_lims, fig_width=15)

    def update_data(self, time, position_l, velocity_l, torque_l, position_r, velocity_r, torque_r,):
        data_values = {
            "Position_Left": position_l,
            "Velocity_Left": velocity_l,
            "Torque_Left": torque_l,
            "Position_Right": position_r,
            "Velocity_Right": velocity_r,
            "Torque_Right": torque_r,
        }
        self.update(time, **data_values)

def test_real_time_plotter_mul4():
    plotter = RealTimePlotterMul4()
    for t in range(2000):  # 模拟 2000 个时间步
        time_val = t * 0.1
        plotter.update_data(time_val, math.sin(time_val * 0.1), math.cos(time_val * 0.1) * 40,
                            math.sin(time_val * 0.5) * 10, math.cos(time_val * 0.3) * 5)  # 分别填入时间+四个数据
        time.sleep(0.0001)  # 增加一个小的延迟来模拟实时性

    plotter.finalize()


def test_real_time_plotter_mul4X2():
    plotter = RealTimePlotterMul4X2()
    for t in range(2000):  # 模拟 2000 个时间步
        time_val = t * 0.1
        plotter.update_data(time_val, math.sin(time_val * 0.1), math.cos(time_val * 0.1) * 40,
                            math.sin(time_val * 0.5) * 10, math.cos(time_val * 0.3) * 5, math.sin(time_val * 0.1),
                            math.cos(time_val * 0.1) * 40, math.sin(time_val * 0.5) * 10,
                            math.cos(time_val * 0.3) * 5)  # 分别填入时间+四个数据
        time.sleep(0.0001)  # 增加一个小的延迟来模拟实时性

    plotter.finalize()



if __name__ == "__main__":
    # test_real_time_plotter_mul4()
    test_real_time_plotter_mul4X2()