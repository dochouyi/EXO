from typing import Optional
from utils.butterworthFilter import ButterworthFilter
from motor.motorController import MotorController


class FilteredMotorController(MotorController):
    def __init__(self, odrv_serial: Optional[str] = None,
                 order: int = 2, cutoff_freq: float = 200, sampling_freq: float = 1000):
        """
        初始化双电机控制器以及滤波器。
        :param odrv_serial: 第一个 ODrive 的序列号。
        :param order: 滤波器阶数。
        :param cutoff_freq: 滤波器的截止频率。
        :param sampling_freq: 滤波器的采样频率。
        """
        # 调用父类的初始化方法
        super().__init__(odrv_serial)
        # 初始化第一个电机的滤波器
        self.order=order
        self.position_filter = ButterworthFilter(order=self.order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.velocity_filter = ButterworthFilter(order=self.order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.current_filter = ButterworthFilter(order=self.order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.torque_filter = ButterworthFilter(order=self.order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)



    def get_Iq_measured_filtered(self):
        """
        获取滤波后的测量电流 Iq。
        :return: Iq 测量值。
        """
        value=self.current_filter.filter_signal(self.get_Iq_measured())
        return value

    def get_torque_estimate_filtered(self):
        """
        获取滤波后的力矩估算值。
        :return: 力矩估算值。
        """
        value=self.torque_filter.filter_signal(self.get_torque_estimate())
        return value

    def get_vel_estimate_filtered(self):
        """
        获取滤波后的速度估算值。
        :return: 速度估算值。
        """
        value=self.velocity_filter.filter_signal(self.get_vel_estimate())
        return value

    def get_pos_estimate_filtered(self):
        """
        获取滤波后的位置估算值。
        :return: 位置估算值。
        """
        value=self.position_filter.filter_signal(self.get_pos_estimate())

        return value

    def estimate_external_torque(self, input_torque):
        """
        估计外部力矩
        :param input_torque: 输入力矩
        :return: 外部力矩
        """
        return self.get_torque_estimate_filtered() - input_torque