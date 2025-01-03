import odrive
from odrive.enums import *
from typing import Optional
from utils.butterworthFilter import ButterworthFilter

class MotorController:
    """
    电机控制类，用于控制和管理 ODrive 电机。
    """
    def __init__(self, odrv_serial: Optional[str] = None):
        """
        初始化电机控制器。
        :param odrv_serial: 可选，指定 ODrive 的序列号。
        """
        self.odrv0 = None
        self.odrv_serial = odrv_serial

    def initialize_odrive(self) -> None:
        """
        初始化 ODrive 设备。
        """
        try:
            print("正在寻找 ODrive...")
            self.odrv0 = odrive.find_any(serial_number=self.odrv_serial)

            if self.odrv0:
                print("ODrive 已连接！")
            else:
                raise ConnectionError("未找到 ODrive，请检查连接。")

            print("清除错误...")
            self.odrv0.clear_errors()
        except Exception as e:
            print(f"初始化 ODrive 失败: {e}")
            exit()

    def calibrate_motor(self) -> None:
        """
        校准电机，包括电机和编码器的校准。
        """
        try:
            print("开始校准...")
            self._set_axis_state(AXIS_STATE_MOTOR_CALIBRATION)
            self._set_axis_state(AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
            self.odrv0.save_configuration()
            print("校准完成。")
        except Exception as e:
            print(f"校准失败: {e}")
            exit()

    def set_torque_control_mode(self) -> None:
        """
        设置电机为力矩控制模式。
        """
        try:
            self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self._set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)
            print("已设置为力矩控制模式。")
        except Exception as e:
            print(f"设置力矩控制模式失败: {e}")

    def stop_motor(self) -> None:
        """
        停止电机。
        """
        try:
            self.odrv0.axis0.controller.input_torque = 0
            self._set_axis_state(AXIS_STATE_IDLE)
            print("电机已停止。")
        except Exception as e:
            print(f"停止电机失败: {e}")

    def set_input_torque(self, torque_value: float) -> None:
        """
        设置输入力矩。
        :param torque_value: 力矩值。
        """
        try:
            self.odrv0.axis0.controller.input_torque = torque_value
        except Exception as e:
            print(f"设置输入力矩失败: {e}")

    def get_torque_constant(self) -> float:
        """
        获取力矩常数。
        :return: 力矩常数值。
        """
        return self.odrv0.axis0.motor.config.torque_constant

    def get_Iq_measured(self) -> float:
        """
        获取测量的电流 Iq。
        :return: Iq 测量值。
        """
        return self.odrv0.axis0.motor.current_control.Iq_measured

    def get_torque_estimate(self) -> float:
        """
        获取力矩估算值。
        :return: 力矩估算值。
        """
        return self.odrv0.axis0.motor.torque_estimate

    def get_vel_estimate(self) -> float:
        """
        获取速度估算值。
        :return: 速度估算值。
        """
        return self.odrv0.axis0.encoder.vel_estimate

    def get_pos_estimate(self) -> float:
        """
        获取位置估算值。
        :return: 位置估算值。
        """
        return self.odrv0.axis0.encoder.pos_estimate

    def _set_axis_state(self, state: int) -> None:
        """
        设置轴的状态，并等待状态切换完成。
        :param state: 目标状态。
        """
        self.odrv0.axis0.requested_state = state
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass


class FilteredMotorController(MotorController):
    def __init__(self, odrv_serial: Optional[str] = None,
                 order: int = 2, cutoff_freq: float = 10, sampling_freq: float = 1000):
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
        self.position_filter = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.velocity_filter = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.current_filter = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)
        self.torque_filter = ButterworthFilter(order=order, cutoff_freq=cutoff_freq, sampling_freq=sampling_freq)

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