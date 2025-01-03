from utils.butterworthFilter import ButterworthFilter
from motorController import MotorController


class DoubleMotorController:
    """
    双电机控制类，用于同时控制两个 ODrive 电机。
    """
    def __init__(self, odrv_serial_1:str, odrv_serial_2:str):
        """
        初始化双电机控制器。
        第一个序列号是左腿电机，第二个序列号是右腿电机
        :param odrv_serial_1: 第一个 ODrive 的序列号。
        :param odrv_serial_2: 第二个 ODrive 的序列号。
        """
        self.motor1 = MotorController(odrv_serial_1)
        self.motor2 = MotorController(odrv_serial_2)

    def initialize_odrive(self) -> None:
        """
        初始化两个 ODrive 设备。
        """
        self.motor1.initialize_odrive()
        self.motor2.initialize_odrive()

    def calibrate_motor(self) -> None:
        """
        校准两个电机。
        """
        self.motor1.calibrate_motor()
        self.motor2.calibrate_motor()

    def set_torque_control_mode(self) -> None:
        """
        设置两个电机为力矩控制模式。
        """
        self.motor1.set_torque_control_mode()
        self.motor2.set_torque_control_mode()

    def stop_motor(self) -> None:
        """
        停止两个电机。
        """
        self.motor1.stop_motor()
        self.motor2.stop_motor()

    def set_input_torque(self, torque1: float, torque2: float) -> None:
        """
        设置两个电机的输入力矩。
        :param torque1: 第一个电机的力矩值。
        :param torque2: 第二个电机的力矩值。
        """
        self.motor1.set_input_torque(torque1)
        self.motor2.set_input_torque(torque2)

    def get_torque_constant(self) -> float:
        """
        获取力矩常数,两个电机的力矩常数完全一致
        :return: 力矩常数值。
        """
        return self.motor1.get_torque_constant()

    def get_Iq_measured(self):
        """
        获取测量的电流 Iq。
        :return: Iq 测量值。
        """
        return [self.motor1.get_Iq_measured(), self.motor2.get_Iq_measured()]

    def get_torque_estimate(self):
        """
        获取力矩估算值。
        :return: 力矩估算值。
        """
        return [self.motor1.get_torque_estimate(), self.motor2.get_torque_estimate()]

    def get_vel_estimate(self):
        """
        获取速度估算值。
        :return: 速度估算值。
        """
        return [self.motor1.get_vel_estimate(), self.motor2.get_vel_estimate()]

    def get_pos_estimate(self):
        """
        获取位置估算值。
        :return: 位置估算值。
        """
        return [self.motor1.get_pos_estimate(), self.motor2.get_pos_estimate()]


class FilteredDoubleMotorController(DoubleMotorController):
    """
    双电机控制类，用于同时控制两个 ODrive 电机。
    """
    def __init__(self, odrv_serial_1: str, odrv_serial_2: str,
                 order: int = 2, cutoff_freq: float = 10, sampling_freq: float = 1000):
        """
        初始化双电机控制器以及滤波器。
        :param odrv_serial_1: 第一个 ODrive 的序列号。
        :param odrv_serial_2: 第二个 ODrive 的序列号。
        :param order: 滤波器阶数。
        :param cutoff_freq: 滤波器的截止频率。
        :param sampling_freq: 滤波器的采样频率。
        """
        # 调用父类的初始化方法
        super().__init__(odrv_serial_1, odrv_serial_2)
        # 初始化第一个电机的滤波器
        self.position_filter_1 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.velocity_filter_1 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.current_filter_1 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.torque_filter_1 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)

        # 初始化第二个电机的滤波器
        self.position_filter_2 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.velocity_filter_2 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.current_filter_2 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.torque_filter_2 = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)

    def get_Iq_measured_filtered(self):
        """
        获取滤波后的测量电流 Iq。
        :return: Iq 测量值。
        """
        value_1=self.current_filter_1.filter_signal(self.motor1.get_Iq_measured())
        value_2=self.current_filter_2.filter_signal(self.motor2.get_Iq_measured())
        return [value_1,value_2]

    def get_torque_estimate_filtered(self):
        """
        获取滤波后的力矩估算值。
        :return: 力矩估算值。
        """
        value_1=self.torque_filter_1.filter_signal(self.motor1.get_torque_estimate())
        value_2=self.torque_filter_2.filter_signal(self.motor2.get_torque_estimate())
        return [value_1,value_2]

    def get_vel_estimate_filtered(self):
        """
        获取滤波后的速度估算值。
        :return: 速度估算值。
        """
        value_1=self.velocity_filter_1.filter_signal(self.motor1.get_vel_estimate())
        value_2=self.velocity_filter_2.filter_signal(self.motor2.get_vel_estimate())
        return [value_1,value_2]

    def get_pos_estimate_filtered(self):
        """
        获取滤波后的位置估算值。
        :return: 位置估算值。
        """
        value_1=self.position_filter_1.filter_signal(self.motor1.get_pos_estimate())
        value_2=self.position_filter_2.filter_signal(self.motor2.get_pos_estimate())
        return [value_1,value_2]