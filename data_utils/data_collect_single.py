import time
import csv
from motor.filteredmotorController import FilteredMotorController
from utils.realTimePlotter import RealTimePlotterMul3


class DataCollectorSingle:

    def __init__(self, motor_controller, data_file="data_log_single.csv", is_show_graph=True):
        """
        初始化阻抗控制器。
        :param motor_controller: 电机控制器实例
        :param data_file: 数据存储文件名
        """
        self.motor_controller = motor_controller
        self.data_file = data_file
        self.is_show_graph = is_show_graph
        self.plotter = RealTimePlotterMul3()

        # 创建 CSV 文件并写入表头
        with open(self.data_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "timestamp",
                "position", "velocity", "torque",
            ])

    def run(self):
        """
        主运行函数，用于实时采集数据并存储。记录的数据是没有经过滤波的原始数据
        """
        start_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time

            # 获取左右腿当前状态
            position = self.motor_controller.get_pos_estimate()
            velocity = self.motor_controller.get_vel_estimate()
            torque= self.motor_controller.get_torque_estimate()

            # 将数据写入 CSV 文件
            with open(self.data_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    elapsed_time,
                    position, velocity, torque,
                ])
            if self.is_show_graph:
                self.plotter.update_data(elapsed_time,position,velocity,torque)
            # 控制循环频率
            time.sleep(0.001)

        # self.plotter.finalize()

    def replay_data(self):
        """
        从存储的 CSV 文件中读取数据，并调用 plotter.update_data 以重新绘制。
        """
        try:
            with open(self.data_file, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  # 跳过表头
                for row in reader:
                    timestamp, position, velocity, torque = map(float, row)
                    self.plotter.update_data(timestamp, position, velocity, torque)
        except FileNotFoundError:
            print(f"文件 {self.data_file} 未找到，请确保数据已被记录。")


# 主程序
if __name__ == "__main__":
    motor = FilteredMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()

    controller = DataCollectorSingle(motor)
    controller.run()
