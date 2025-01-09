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
            # if t % 0.01 < dt:  # 每隔 10ms 更新一次轨迹参数
            #     self.trajectory_handler.fit_and_update()

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

    controller = ImpedanceController(motor, trajectory_handler, duration=10)
    controller.run()