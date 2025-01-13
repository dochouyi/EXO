import math
import time
from utils.realTimePlotter import RealTimePlotterMul4X2
from motor.doubleMotorController import FilteredDoubleMotorController
import numpy as np
from trajectory_handler.sineGenerator import SineTrajectoryHandler



class DualLegImpedanceController:
    """
    左右腿独立阻抗控制类，支持通过速度等级控制行走速度。
    """
    MAX_TORQUE = 6.0  # 最大力矩限制

    def __init__(self, motor_controller, trajectory_handler_left, trajectory_handler_right, duration,
                 speed_level=5,  # 默认速度等级为5
                 Kp_left=1.0, Kd_left=0.1, Ki_left=0.01, Kf_left=0.5,
                 Kp_right=1.0, Kd_right=0.1, Ki_right=0.01, Kf_right=0.5):
        self.motor_controller = motor_controller
        self.trajectory_handler_left = trajectory_handler_left
        self.trajectory_handler_right = trajectory_handler_right
        self.duration = duration

        self.Kp_left = Kp_left
        self.Kd_left = Kd_left
        self.Kf_left = Kf_left
        self.Ki_left = Ki_left

        self.Kp_right = Kp_right
        self.Kd_right = Kd_right
        self.Kf_right = Kf_right
        self.Ki_right = Ki_right

        self.integral_limit = 1.0
        self.integral_error_left = 0.0
        self.integral_error_right = 0.0
        self.error_log_left = []
        self.error_log_right = []
        self.plotter = RealTimePlotterMul4X2()

        # 设置速度等级
        self.speed_level = speed_level
        self._apply_speed_level()



    def control_motor_forward(self, max_rotation=180, resistance_threshold=1.0):
        """
        控制电机正向缓慢运动，当外界阻力大于 1 Nm 时停止并返回 True；
        如果旋转 180 度仍未达到力矩阈值，则返回 False。

        :param max_rotation: 最大旋转角度（度）
        :param resistance_threshold: 阻力阈值 (Nm)
        :return: bool
        """
        self.motor_controller.set_input_torque(0.1,-0.1)  # 设置初始缓慢运动力矩
        start_position_left, start_position_right= self.motor_controller.get_pos_estimate_filtered()  # 记录初始位置

        while True:
            current_position_left, current_position_right = self.motor_controller.get_pos_estimate_filtered()
            external_torque_left = self.motor_controller.motor1.estimate_external_torque(0.1)
            external_torque_right = self.motor_controller.motor2.estimate_external_torque(0.1)

            # 检查力矩是否超过阈值
            if external_torque_left > resistance_threshold and external_torque_right > resistance_threshold:
                self.motor_controller.set_input_torque(0,0)
                print("外界阻力超过阈值，停止电机。")
                return True

            # 检查是否旋转超过最大角度
            rotation_angle = abs(current_position_left - start_position_left) * 180 / math.pi  # 转换为角度
            if rotation_angle >= max_rotation:
                self.motor_controller.set_input_torque(0,0)
                print("旋转角度达到最大值，未检测到足够的外界阻力。")
                return False

            time.sleep(0.01)  # 控制循环频率



    def _apply_speed_level(self):
        """
        根据速度等级调整轨迹生成器的频率。
        """
        # 将速度等级（1-10）映射到频率范围（例如：0.1 Hz 到 2 Hz）
        min_frequency = 0.1
        max_frequency = 2.0
        frequency = min_frequency + (self.speed_level - 1) * (max_frequency - min_frequency) / 9

        # 更新左右腿轨迹生成器的频率
        self.trajectory_handler_left.frequency = frequency
        self.trajectory_handler_right.frequency = frequency

    def adaptive_control_parameters(self, position_error, velocity_error, side="left"):
        """
        自适应调整控制参数，根据误差动态调整Kp, Kd和Ki。
        :param position_error: 当前的位置信误差
        :param velocity_error: 当前的速度误差
        :param side: 调整哪一侧的参数，"left"或"right"
        """
        # 定义调整步长
        learning_rate_Kp = 0.01
        learning_rate_Kd = 0.005
        learning_rate_Ki = 0.001

        # 计算误差的绝对值
        abs_position_error = abs(position_error)
        abs_velocity_error = abs(velocity_error)

        if side == "left":
            # 根据误差调整左腿的控制参数
            if abs_position_error > 0.1:  # 如果误差较大，增加Kp
                self.Kp_left += learning_rate_Kp * abs_position_error
            else:  # 如果误差较小，减小Kp
                self.Kp_left -= learning_rate_Kp * abs_position_error

            if abs_velocity_error > 0.1:  # 如果速度误差较大，增加Kd
                self.Kd_left += learning_rate_Kd * abs_velocity_error
            else:  # 如果速度误差较小，减小Kd
                self.Kd_left -= learning_rate_Kd * abs_velocity_error

            # 调整积分增益Ki（防止积分误差过大）
            self.Ki_left += learning_rate_Ki * position_error
            self.Ki_left = max(min(self.Ki_left, 0.1), 0.001)  # 限制Ki的范围
        elif side == "right":
            # 根据误差调整右腿的控制参数
            if abs_position_error > 0.1:
                self.Kp_right += learning_rate_Kp * abs_position_error
            else:
                self.Kp_right -= learning_rate_Kp * abs_position_error

            if abs_velocity_error > 0.1:
                self.Kd_right += learning_rate_Kd * abs_velocity_error
            else:
                self.Kd_right -= learning_rate_Kd * abs_velocity_error

            self.Ki_right += learning_rate_Ki * position_error
            self.Ki_right = max(min(self.Ki_right, 0.1), 0.001)

        # 限制Kp和Kd的范围，防止过大或过小
        self.Kp_left = max(min(self.Kp_left, 10.0), 0.1)
        self.Kd_left = max(min(self.Kd_left, 5.0), 0.01)
        self.Kp_right = max(min(self.Kp_right, 10.0), 0.1)
        self.Kd_right = max(min(self.Kd_right, 5.0), 0.01)


    def run(self):
        start_time = time.time()
        last_time = start_time
        print(f"开始左右腿独立阻抗控制，速度等级：{self.speed_level} ...")

        while time.time() - start_time < self.duration:
            current_time = time.time()
            t = current_time - start_time
            dt = current_time - last_time
            last_time = current_time

            # 获取左右腿当前状态
            current_position_left, current_position_right = self.motor_controller.get_pos_estimate_filtered()
            current_velocity_left, current_velocity_right = self.motor_controller.get_vel_estimate_filtered()

            # 更新轨迹
            self.trajectory_handler_left.update_data(t, current_position_left)
            self.trajectory_handler_right.update_data(t, current_position_right)

            if t % 0.01 < dt:  # 每隔 10ms 更新一次轨迹参数
                self.trajectory_handler_left.fit_and_update()
                self.trajectory_handler_right.fit_and_update()

            # 假设轨迹周期为T
            phase_offset = 180
            T = 1.0 / self.trajectory_handler_left.frequency
            phase_offset_time = (phase_offset / 360.0) * T
            # 然后在时间上进行偏移
            desired_position_left_sinle = self.trajectory_handler_left.get_position(t)
            desired_velocity_left_single = self.trajectory_handler_left.get_velocity(t)
            desired_position_left_couple = self.trajectory_handler_right.get_position(t + phase_offset_time)
            desired_velocity_left_couple = self.trajectory_handler_right.get_velocity(t + phase_offset_time)

            desired_position_right_single = self.trajectory_handler_right.get_position(t)
            desired_velocity_right_single = self.trajectory_handler_right.get_velocity(t)
            desired_position_right_couple = self.trajectory_handler_left.get_position(t + phase_offset_time)
            desired_velocity_right_couple = self.trajectory_handler_left.get_velocity(t + phase_offset_time)

            desired_position_left = (desired_position_left_sinle + desired_position_left_couple) / 2
            desired_velocity_left = (desired_velocity_left_single + desired_velocity_left_couple) / 2
            desired_position_right = (desired_position_right_single + desired_position_right_couple) / 2
            desired_velocity_right = (desired_velocity_right_single + desired_velocity_right_couple) / 2

            # 左腿控制
            position_error_left = desired_position_left - current_position_left
            velocity_error_left = desired_velocity_left - current_velocity_left
            self.integral_error_left += position_error_left * dt
            self.integral_error_left = max(min(self.integral_error_left, self.integral_limit), -self.integral_limit)
            target_torque_left = (
                self.Kp_left * position_error_left +
                self.Kd_left * velocity_error_left +
                self.Ki_left * self.integral_error_left
            )
            external_torque_left = self.motor_controller.motor1.estimate_external_torque(target_torque_left)
            adjusted_torque_left = target_torque_left + self.Kf_left * external_torque_left
            adjusted_torque_left = max(min(adjusted_torque_left, self.MAX_TORQUE), -self.MAX_TORQUE)

            # 右腿控制
            position_error_right = desired_position_right - current_position_right
            velocity_error_right = desired_velocity_right - current_velocity_right
            self.integral_error_right += position_error_right * dt
            self.integral_error_right = max(min(self.integral_error_right, self.integral_limit), -self.integral_limit)
            target_torque_right = (
                self.Kp_right * position_error_right +
                self.Kd_right * velocity_error_right +
                self.Ki_right * self.integral_error_right
            )
            external_torque_right = self.motor_controller.motor2.estimate_external_torque(target_torque_right)
            adjusted_torque_right = target_torque_right + self.Kf_right * external_torque_right
            adjusted_torque_right = max(min(adjusted_torque_right, self.MAX_TORQUE), -self.MAX_TORQUE)

            # 设置左右腿力矩
            self.motor_controller.set_input_torque(adjusted_torque_left, adjusted_torque_right)

            # 记录误差
            self.error_log_left.append(position_error_left)
            self.error_log_right.append(position_error_right)

            # 控制循环频率
            time.sleep(0.001)

        print("左右腿独立阻抗控制完成！")
        self.analyze_performance()

    def run_ajdust(self):
        start_time = time.time()
        last_time = start_time
        print(f"开始左右腿独立阻抗控制，速度等级：{self.speed_level} ...")

        while time.time() - start_time < self.duration:
            current_time = time.time()
            t = current_time - start_time
            dt = current_time - last_time
            last_time = current_time

            # 获取左右腿当前状态
            current_position_left, current_position_right = self.motor_controller.get_pos_estimate_filtered()
            current_velocity_left, current_velocity_right = self.motor_controller.get_vel_estimate_filtered()

            # 更新轨迹
            self.trajectory_handler_left.update_data(t, current_position_left)
            self.trajectory_handler_right.update_data(t, current_position_right)

            if t % 0.01 < dt:  # 每隔 10ms 更新一次轨迹参数
                self.trajectory_handler_left.fit_and_update()
                self.trajectory_handler_right.fit_and_update()

            # 假设轨迹周期为T
            phase_offset = 180
            T = 1.0 / self.trajectory_handler_left.frequency
            phase_offset_time = (phase_offset / 360.0) * T
            # 然后在时间上进行偏移
            desired_position_left_sinle = self.trajectory_handler_left.get_position(t)
            desired_velocity_left_single = self.trajectory_handler_left.get_velocity(t)
            desired_position_left_couple = self.trajectory_handler_right.get_position(t + phase_offset_time)
            desired_velocity_left_couple = self.trajectory_handler_right.get_velocity(t + phase_offset_time)

            desired_position_right_single = self.trajectory_handler_right.get_position(t)
            desired_velocity_right_single = self.trajectory_handler_right.get_velocity(t)
            desired_position_right_couple = self.trajectory_handler_left.get_position(t + phase_offset_time)
            desired_velocity_right_couple = self.trajectory_handler_left.get_velocity(t + phase_offset_time)

            desired_position_left = (desired_position_left_sinle + desired_position_left_couple) / 2
            desired_velocity_left = (desired_velocity_left_single + desired_velocity_left_couple) / 2
            desired_position_right = (desired_position_right_single + desired_position_right_couple) / 2
            desired_velocity_right = (desired_velocity_right_single + desired_velocity_right_couple) / 2

            # 左腿控制
            position_error_left = desired_position_left - current_position_left
            velocity_error_left = desired_velocity_left - current_velocity_left
            self.integral_error_left += position_error_left * dt
            self.integral_error_left = max(min(self.integral_error_left, self.integral_limit), -self.integral_limit)
            target_torque_left = (
                    self.Kp_left * position_error_left +
                    self.Kd_left * velocity_error_left +
                    self.Ki_left * self.integral_error_left
            )
            external_torque_left = self.motor_controller.motor1.estimate_external_torque(target_torque_left)
            adjusted_torque_left = target_torque_left + self.Kf_left * external_torque_left
            adjusted_torque_left = max(min(adjusted_torque_left, self.MAX_TORQUE), -self.MAX_TORQUE)

            # 自适应调整左腿控制参数
            self.adaptive_control_parameters(position_error_left, velocity_error_left, side="left")
            # 右腿控制
            position_error_right = desired_position_right - current_position_right
            velocity_error_right = desired_velocity_right - current_velocity_right
            self.integral_error_right += position_error_right * dt
            self.integral_error_right = max(min(self.integral_error_right, self.integral_limit), -self.integral_limit)
            target_torque_right = (
                    self.Kp_right * position_error_right +
                    self.Kd_right * velocity_error_right +
                    self.Ki_right * self.integral_error_right
            )
            external_torque_right = self.motor_controller.motor2.estimate_external_torque(target_torque_right)
            adjusted_torque_right = target_torque_right + self.Kf_right * external_torque_right
            adjusted_torque_right = max(min(adjusted_torque_right, self.MAX_TORQUE), -self.MAX_TORQUE)


            # 自适应调整右腿控制参数
            self.adaptive_control_parameters(position_error_right, velocity_error_right, side="right")
            # 设置左右腿力矩
            self.motor_controller.set_input_torque(adjusted_torque_left, adjusted_torque_right)

            # 记录误差
            self.error_log_left.append(position_error_left)
            self.error_log_right.append(position_error_right)

            # 控制循环频率
            time.sleep(0.001)

        print("左右腿独立阻抗控制完成！")
        self.analyze_performance()



    def analyze_performance(self):
        rms_error_left = np.sqrt(np.mean(np.square(self.error_log_left)))
        rms_error_right = np.sqrt(np.mean(np.square(self.error_log_right)))
        print(f"左腿 - RMS 误差: {rms_error_left:.4f}")
        print(f"右腿 - RMS 误差: {rms_error_right:.4f}")



# 主程序
if __name__ == "__main__":
    motor = FilteredDoubleMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()

    trajectory_handler_left = SineTrajectoryHandler(amplitude=0.5, frequency=0.5)
    trajectory_handler_right = SineTrajectoryHandler(amplitude=0.5, frequency=0.5)

    # 设置速度等级（1-10）
    speed_level = 7

    controller = DualLegImpedanceController(motor, trajectory_handler_left, trajectory_handler_right, duration=10, speed_level=speed_level)
    controller.run()
