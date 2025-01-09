import time
from utils.realTimePlotter import RealTimePlotterMul4
from motor.filteredmotorController import FilteredMotorController
import numpy as np
from trajectory_handler.sineGenerator import SineTrajectoryHandler


class AdmittanceController:
    """
    单腿导纳控制类
    """
    MAX_POSITION = 1.0  # 最大位置限制

    def __init__(self, motor_controller, trajectory_handler, duration,
                 M=1.0, B=0.1, K=0.5):
        self.motor_controller = motor_controller
        self.trajectory_handler = trajectory_handler
        self.duration = duration

        self.M = M  # 虚拟质量
        self.B = B  # 虚拟阻尼
        self.K = K  # 虚拟刚度

        self.desired_position = 0.0
        self.desired_velocity = 0.0
        self.plotter = RealTimePlotterMul4()

    def run(self):
        start_time = time.time()
        last_time = start_time
        print("开始导纳控制...")

        # 初始化位置和速度
        current_position = self.motor_controller.get_pos_estimate_filtered()
        current_velocity = self.motor_controller.get_vel_estimate_filtered()
        self.desired_position = current_position
        self.desired_velocity = current_velocity

        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            dt = time.time() - last_time
            if dt == 0:
                continue  # 避免除以零
            last_time = time.time()

            current_position = self.motor_controller.get_pos_estimate_filtered()
            current_velocity = self.motor_controller.get_vel_estimate_filtered()

            # 获得外部力矩
            external_torque = self.motor_controller.estimate_external_torque(0)

            # 计算期望加速度
            desired_acceleration = (external_torque - self.B * self.desired_velocity - self.K * (self.desired_position - current_position)) / self.M

            # 更新期望速度和位置
            self.desired_velocity += desired_acceleration * dt
            self.desired_position += self.desired_velocity * dt

            # 位置限制
            self.desired_position = max(min(self.desired_position, self.MAX_POSITION), -self.MAX_POSITION)

            # 设置电机期望位置
            self.motor_controller.set_input_pos(self.desired_position)

            # 更新绘图数据
            self.plotter.update_data(t, current_position, current_velocity, external_torque, self.desired_position)

            # 控制循环频率
            time.sleep(0.001)

        print("导纳控制完成！")
        self.plotter.finalize()
        self.motor_controller.stop_motor()


# 主程序
if __name__ == "__main__":
    motor = FilteredMotorController()
    motor.initialize_odrive()
    motor.set_pos_control_mode()  # 设置为位置控制模式

    trajectory_handler = SineTrajectoryHandler(amplitude=0.5, frequency=0.5)

    controller = AdmittanceController(motor, trajectory_handler, duration=10)
    controller.run()
