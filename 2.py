import time
from utils.realTimePlotter import RealTimePlotter
from motor.doubleMotorController import FilteredDoubleMotorController
import numpy as np
from trajectory_handler.sineGenerator import SineTrajectoryHandler


class DualLegImpedanceController:
    # 省略已有的代码...

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

            # 计算期望轨迹
            desired_position_left = self.trajectory_handler_left.get_position(t)
            desired_velocity_left = self.trajectory_handler_left.get_velocity(t)
            desired_position_right = self.trajectory_handler_right.get_position(t)
            desired_velocity_right = self.trajectory_handler_right.get_velocity(t)

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
