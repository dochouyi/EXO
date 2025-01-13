import math
import time
from utils.realTimePlotter import RealTimePlotterMul4
from motor.filteredmotorController import FilteredMotorController
import numpy as np
from trajectory_handler.sineGenerator import SineTrajectoryHandler


class ImpedanceController:
    """
    单腿阻抗控制类
    """
    MAX_TORQUE = 2.0  # 最大力矩限制

    def __init__(self, motor_controller, trajectory_handler, duration,
                 Kp=1.0, Kd=0.1, Ki=0.01, Kf=0.5):
        self.motor_controller = motor_controller
        self.trajectory_handler = trajectory_handler
        self.duration = duration

        self.Kp = Kp
        self.Kd = Kd
        self.Kf = Kf
        self.Ki = Ki

        self.integral_limit = 1.0
        self.integral_error = 0.0  #积分项初始化
        self.error_log = []
        self.plotter = RealTimePlotterMul4()


    def control_motor_forward(self, max_rotation=180, resistance_threshold=1.0):
        """
        控制电机正向缓慢运动，当外界阻力大于 1 Nm 时停止并返回 True；
        如果旋转 180 度仍未达到力矩阈值，则返回 False。

        :param max_rotation: 最大旋转角度（度）
        :param resistance_threshold: 阻力阈值 (Nm)
        :return: bool
        """
        self.motor_controller.set_input_torque(0.1)  # 设置初始缓慢运动力矩
        start_position = self.motor_controller.get_pos_estimate_filtered()  # 记录初始位置

        while True:
            current_position = self.motor_controller.get_pos_estimate_filtered()
            external_torque = self.motor_controller.estimate_external_torque(0.1)

            # 检查力矩是否超过阈值
            if external_torque > resistance_threshold:
                self.motor_controller.set_input_torque(0.0)  # 停止电机
                print("外界阻力超过阈值，停止电机。")
                return True

            # 检查是否旋转超过最大角度
            rotation_angle = abs(current_position - start_position) * 180 / math.pi  # 转换为角度
            if rotation_angle >= max_rotation:
                self.motor_controller.set_input_torque(0.0)  # 停止电机
                print("旋转角度达到最大值，未检测到足够的外界阻力。")
                return False

            time.sleep(0.01)  # 控制循环频率


    def run(self):
        start_time = time.time()
        last_time = start_time
        print("开始阻抗控制...")

        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            dt = time.time() - last_time
            last_time = time.time()

            current_position = self.motor_controller.get_pos_estimate_filtered()
            current_velocity = self.motor_controller.get_vel_estimate_filtered()

            # 更新轨迹
            self.trajectory_handler.update_data(t, current_position)
            if t % 0.01 < dt:  # 每隔 10ms 更新一次轨迹参数
                self.trajectory_handler.fit_and_update()

            desired_position = self.trajectory_handler.get_position(t)
            desired_velocity = self.trajectory_handler.get_velocity(t)

            position_error = desired_position - current_position
            velocity_error = desired_velocity - current_velocity

            # PID 控制
            self.integral_error += position_error * dt
            self.integral_error = max(
                min(self.integral_error, self.integral_limit), -self.integral_limit
            )
            target_torque = (
                self.Kp * position_error +
                self.Kd * velocity_error +
                self.Ki * self.integral_error
            )

            # 力矩补偿与限制
            external_torque = self.motor_controller.estimate_external_torque(target_torque)
            adjusted_torque = target_torque + self.Kf * external_torque
            adjusted_torque = max(min(adjusted_torque, self.MAX_TORQUE), -self.MAX_TORQUE)


            self.motor_controller.set_input_torque(adjusted_torque)

            # 记录误差
            self.error_log.append(position_error)
            # 控制循环频率
            self.plotter.update_data(t, current_position, current_velocity, target_torque, adjusted_torque)
            # 控制循环频率
            time.sleep(0.001)

        print("阻抗控制完成！")
        self.plotter.finalize()
        self.analyze_performance()
        self.motor_controller.stop_motor()


    def analyze_performance(self):
        rms_error = np.sqrt(np.mean(np.square(self.error_log)))
        max_error = np.max(np.abs(self.error_log))
        print(f"RMS 误差: {rms_error:.4f}, 最大误差: {max_error:.4f}")



# 主程序
if __name__ == "__main__":
    motor = FilteredMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()

    trajectory_handler = SineTrajectoryHandler(amplitude=0.5, frequency=0.5)

    controller = ImpedanceController(motor, trajectory_handler, duration=1000)
    controller.run()