import time
import csv
from motor.doubleMotorController import FilteredDoubleMotorController
from utils.realTimePlotter import RealTimePlotterMul3X2


class DataCollectorDouble:

    def __init__(self, motor_controller, data_file="data_log_double.csv", is_show_graph=True):
        """
        初始化双腿阻抗控制器。
        :param motor_controller: 电机控制器实例
        :param data_file: 数据存储文件名
        """
        self.motor_controller = motor_controller
        self.data_file = data_file
        self.is_show_graph = is_show_graph
        self.plotter = RealTimePlotterMul3X2()

        # 创建 CSV 文件并写入表头
        with open(self.data_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "timestamp",
                "position_left", "velocity_left", "torque_left",
                "position_right", "velocity_right", "torque_right"
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

            # 将数据写入 CSV 文件
            with open(self.data_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    elapsed_time,
                    position_left, velocity_left, torque_left,
                    position_right, velocity_right, torque_right
                ])
            if self.is_show_graph:
                self.plotter.update_data(elapsed_time,position_left,velocity_left,torque_left,position_right,velocity_right,torque_right)

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
                    elapsed_time, position_left, velocity_left, torque_left, position_right, velocity_right, torque_right= map(float, row)
                    self.plotter.update_data(elapsed_time,position_left,velocity_left,torque_left,position_right,velocity_right,torque_right)

        except FileNotFoundError:
            print(f"文件 {self.data_file} 未找到，请确保数据已被记录。")



# 主程序
if __name__ == "__main__":
    motor = FilteredDoubleMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()


    controller = DataCollectorDouble(motor)
    controller.run()
