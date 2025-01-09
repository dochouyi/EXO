import time
import csv
from motor.doubleMotorController import FilteredDoubleMotorController


class DataCollector:

    def __init__(self, motor_controller, data_file="data_log.csv"):
        """
        初始化双腿阻抗控制器。
        :param motor_controller: 电机控制器实例
        :param data_file: 数据存储文件名
        """
        self.motor_controller = motor_controller
        self.data_file = data_file

        # 创建 CSV 文件并写入表头
        with open(self.data_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "timestamp",
                "position_left", "velocity_left", "torque_left", "current_left",
                "position_right", "velocity_right", "torque_right", "current_right"
            ])

    def run(self):
        """
        主运行函数，用于实时采集数据并存储。
        """
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            # 获取左右腿当前状态
            position_left, position_right = self.motor_controller.get_pos_estimate_filtered()
            velocity_left, velocity_right = self.motor_controller.get_vel_estimate_filtered()
            torque_left, torque_right = self.motor_controller.get_torque_estimate_filtered()
            current_left, current_right = self.motor_controller.get_Iq_measured_filtered()

            # 将数据写入 CSV 文件
            with open(self.data_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    elapsed_time,
                    position_left, velocity_left, torque_left, current_left,
                    position_right, velocity_right, torque_right, current_right
                ])

            # 控制循环频率
            time.sleep(0.001)


# 主程序
if __name__ == "__main__":
    motor = FilteredDoubleMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()


    controller = DataCollector(motor)
    controller.run()
