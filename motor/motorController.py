import odrive
from odrive.enums import *
from typing import Optional


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

    def display_values(self) -> None:
        print("Motor Configuration:")
        print(f"  Pole Pairs: {self.odrv0.axis0.motor.config.pole_pairs}")
        print(f"  Torque Constant: {self.odrv0.axis0.motor.config.torque_constant}")
        print(f"  Calibration Current: {self.odrv0.axis0.motor.config.calibration_current}")
        print(f"  Resistance Calibration Max Voltage: {self.odrv0.axis0.motor.config.resistance_calib_max_voltage}")
        print(f"  Current Limit: {self.odrv0.axis0.motor.config.current_lim}")

        print("\nController Configuration:")
        print(f"  Enable Velocity Limit: {self.odrv0.axis0.controller.config.enable_vel_limit}")
        print(f"  Enable Torque Mode Velocity Limit: {self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit}")
        print(f"  Input Filter Bandwidth: {self.odrv0.axis0.controller.config.input_filter_bandwidth}")
        print(f"  Velocity Limit: {self.odrv0.axis0.controller.config.vel_limit}")

        print("\nMotor Thermistor Configuration:")
        print(f"  Enabled: {self.odrv0.axis0.motor.motor_thermistor.config.enabled}")
        print(f"  Temp Limit Lower: {self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower}")
        print(f"  Temp Limit Upper: {self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper}")

        print("\nFET Thermistor Configuration:")
        print(f"  Enabled: {self.odrv0.axis0.motor.fet_thermistor.config.enabled}")
        print(f"  Temp Limit Lower: {self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower}")
        print(f"  Temp Limit Upper: {self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper}")

        print("\nDC Bus Configuration:")
        print(f"  Overvoltage Trip Level: {self.odrv0.config.dc_bus_overvoltage_trip_level}")
        print(f"  Undervoltage Trip Level: {self.odrv0.config.dc_bus_undervoltage_trip_level}")
        print(f"  Max Positive Current: {self.odrv0.config.dc_max_positive_current}")
        print(f"  Max Negative Current: {self.odrv0.config.dc_max_negative_current}")
        print(f"  Enable Brake Resistor: {self.odrv0.config.enable_brake_resistor}")
        print(f"  Enable DC Bus Overvoltage Ramp: {self.odrv0.config.enable_dc_bus_overvoltage_ramp}")
        print(f"  DC Bus Overvoltage Ramp Start: {self.odrv0.config.dc_bus_overvoltage_ramp_start}")
        print(f"  DC Bus Overvoltage Ramp End: {self.odrv0.config.dc_bus_overvoltage_ramp_end}")
        print(f"  Brake Resistance: {self.odrv0.config.brake_resistance}")
        print(f"  Max Regen Current: {self.odrv0.config.max_regen_current}")

        print("\nCAN Configuration:")
        print(f"  Node ID: {self.odrv0.axis0.config.can.node_id}")
        print(f"  Baud Rate: {self.odrv0.can.config.baud_rate}")
        print(f"  R120 GPIO Num: {self.odrv0.can.config.r120_gpio_num}")
        print(f"  Enable R120: {self.odrv0.can.config.enable_r120}")

        print("\nConfiguration saved successfully.")


    def set_values_switch_power(self) -> None:
        # 完全确定的参数
        self.odrv0.axis0.motor.config.pole_pairs = 10
        self.odrv0.axis0.motor.config.torque_constant = 0.042
        self.odrv0.axis0.motor.config.calibration_current = 5
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = 5
        self.odrv0.axis0.controller.config.enable_vel_limit = True
        self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit = 1
        self.odrv0.axis0.controller.config.input_filter_bandwidth = 0

        # 温度相关的一系列参数
        self.odrv0.axis0.motor.motor_thermistor.config.enabled = 1
        self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower = -20
        self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper = 90
        self.odrv0.axis0.motor.fet_thermistor.config.enabled = 1
        self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower = -20
        self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper = 90

        self.odrv0.config.dc_bus_overvoltage_trip_level = 30.0
        self.odrv0.config.dc_bus_undervoltage_trip_level = 18
        self.odrv0.config.dc_max_positive_current = 25
        self.odrv0.config.dc_max_negative_current = 0
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.enable_dc_bus_overvoltage_ramp = True
        self.odrv0.config.dc_bus_overvoltage_ramp_start = 28
        self.odrv0.config.dc_bus_overvoltage_ramp_end = 29

        self.odrv0.config.brake_resistance = 1
        self.odrv0.config.max_regen_current = 0 #再生电流设置为10A，防止刹车时的能量回流超过电源或刹车电阻的承受能力

        self.odrv0.axis0.motor.config.current_lim = 30
        self.odrv0.axis0.controller.config.vel_limit = 5000

        self.odrv0.axis0.config.can.node_id = 1
        self.odrv0.can.config.baud_rate = 500000
        self.odrv0.can.config.r120_gpio_num = 5
        self.odrv0.can.config.enable_r120 = True

        self.odrv0.save_configuration()

    def set_values_battery(self) -> None:
        # 完全确定的参数
        self.odrv0.axis0.motor.config.pole_pairs = 10
        self.odrv0.axis0.motor.config.torque_constant = 0.042
        self.odrv0.axis0.motor.config.calibration_current = 5
        self.odrv0.axis0.motor.config.resistance_calib_max_voltage = 5
        self.odrv0.axis0.controller.config.enable_vel_limit = True
        self.odrv0.axis0.controller.config.enable_torque_mode_vel_limit = 1
        self.odrv0.axis0.controller.config.input_filter_bandwidth = 0

        # 温度相关的一系列参数
        self.odrv0.axis0.motor.motor_thermistor.config.enabled = 1
        self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_lower = -20
        self.odrv0.axis0.motor.motor_thermistor.config.temp_limit_upper = 90
        self.odrv0.axis0.motor.fet_thermistor.config.enabled = 1
        self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_lower = -20
        self.odrv0.axis0.motor.fet_thermistor.config.temp_limit_upper = 90

        #以下参数需要配置
        self.odrv0.config.dc_bus_overvoltage_trip_level = 30.0
        self.odrv0.config.dc_bus_undervoltage_trip_level = 18
        self.odrv0.config.dc_max_positive_current = 25
        self.odrv0.config.dc_max_negative_current = -10
        self.odrv0.config.enable_brake_resistor = True
        self.odrv0.config.enable_dc_bus_overvoltage_ramp = True
        self.odrv0.config.dc_bus_overvoltage_ramp_start = 28
        self.odrv0.config.dc_bus_overvoltage_ramp_end = 29

        self.odrv0.config.brake_resistance = 1
        self.odrv0.config.max_regen_current = 10

        self.odrv0.axis0.motor.config.current_lim = 30
        self.odrv0.axis0.controller.config.vel_limit = 5000

        self.odrv0.axis0.config.can.node_id = 1
        self.odrv0.can.config.baud_rate = 500000
        self.odrv0.can.config.r120_gpio_num = 5
        self.odrv0.can.config.enable_r120 = True

        self.odrv0.save_configuration()


    def calibrate_motor(self) -> None:
        """
        校准电机，包括电机和编码器的校准。
        """
        try:
            print("开始校准...")
            self._set_axis_state(AXIS_STATE_MOTOR_CALIBRATION)
            self._set_axis_state(AXIS_STATE_ENCODER_OFFSET_CALIBRATION)

            self.odrv0.axis0.motor.config.pre_calibrated = 1
            self.odrv0.axis0.encoder.config.pre_calibrated = 1
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
        except Exception as e:
            print(f"设置力矩控制模式失败: {e}")

    def stop_motor(self) -> None:
        """
        停止电机。
        """
        try:
            self.odrv0.axis0.controller.input_torque = 0
            self._set_axis_state(AXIS_STATE_IDLE)
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
        return self.get_Iq_measured()*self.get_torque_constant()

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
        while self.odrv0.axis0.current_state != state:
            pass

    def reset_origin(self, new_angle) -> None:
        """
        重置电机的原点位置。
        用户输入新的角度值，并将其设置为当前位置估算值。
        """
        try:
            self.odrv0.axis0.encoder.index_offset= new_angle
        except Exception as e:
            print(f"重置原点失败: {e}")