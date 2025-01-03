import time
from utils.realTimePlotter import RealTimePlotter
from motor.motorController import FilteredMotorController
from motor.doubleMotorController import DoubleMotorController,FilteredDoubleMotorController
import numpy as np
from trajectory_handler.sineGenerator import SineTrajectoryHandler


class DualLegImpedanceController:
    """
    左右腿独立阻抗控制类
    """
    MAX_TORQUE = 2.0  # 最大力矩限制

    def __init__(self, motor_controller, trajectory_handler_left, trajectory_handler_right, duration,
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


    def run(self):
        start_time = time.time()
        last_time = start_time
        print("开始左右腿独立阻抗控制...")

        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            dt = time.time() - last_time
            last_time = time.time()

            # 获取左右腿当前状态
            current_position_left, current_position_right = self.motor_controller.get_pos_estimate_filtered()
            current_velocity_left, current_velocity_right = self.motor_controller.get_vel_estimate_filtered()

            # 更新轨迹
            self.trajectory_handler_left.update_data(t, current_position_left)
            self.trajectory_handler_right.update_data(t,current_position_right)

            if t % 0.01 < dt:  # 每隔 10ms 更新一次轨迹参数
                self.trajectory_handler_left.fit_and_update()
                self.trajectory_handler_right.fit_and_update()

            phase_offset=180
            desired_position_left_sinle = self.trajectory_handler_left.get_position(t)
            desired_velocity_left_single = self.trajectory_handler_left.get_velocity(t)
            desired_position_left_couple = self.trajectory_handler_right.get_position(t+phase_offset)
            desired_velocity_left_couple = self.trajectory_handler_right.get_velocity(t+phase_offset)

            desired_position_right_single = self.trajectory_handler_right.get_position(t)
            desired_velocity_right_single = self.trajectory_handler_right.get_velocity(t)
            desired_position_right_couple = self.trajectory_handler_left.get_position(t+phase_offset)
            desired_velocity_right_couple = self.trajectory_handler_left.get_velocity(t+phase_offset)

            desired_position_left=(desired_position_left_sinle+desired_position_left_couple)/2
            desired_velocity_left=(desired_velocity_left_single+desired_velocity_left_couple)/2
            desired_position_right=(desired_position_right_single+desired_position_right_couple)/2
            desired_velocity_right=(desired_velocity_right_single+desired_velocity_right_couple)/2

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

    controller = DualLegImpedanceController(motor, trajectory_handler_left, trajectory_handler_right, duration=10)
    controller.run()