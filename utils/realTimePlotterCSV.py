import csv
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
matplotlib.use('TkAgg')  # 设置后端为 TkAgg


class RealTimePlotter:
    def __init__(self, csv_file, window_size=40):
        """
        实时绘图器，用于从 CSV 文件中读取数据并绘制。
        :param csv_file: CSV 文件路径
        :param window_size: 显示的时间窗口大小（秒）
        """
        self.csv_file = csv_file
        self.window_size = window_size

        # 初始化数据存储
        self.time_data = []
        self.left_position = []
        self.left_velocity = []
        self.left_torque = []
        self.left_current = []
        self.right_position = []
        self.right_velocity = []
        self.right_torque = []
        self.right_current = []

        # 初始化绘图
        plt.ion()
        self.fig, self.axs = plt.subplots(4, 2, figsize=(12, 10), sharex=True)
        self.fig.suptitle("Real-Time Visualization of Motor Data", fontsize=16)

        # 左电机
        self.axs[0, 0].set_title("Left Motor Position")
        self.axs[1, 0].set_title("Left Motor Velocity")
        self.axs[2, 0].set_title("Left Motor Torque")
        self.axs[3, 0].set_title("Left Motor Current")

        # 右电机
        self.axs[0, 1].set_title("Right Motor Position")
        self.axs[1, 1].set_title("Right Motor Velocity")
        self.axs[2, 1].set_title("Right Motor Torque")
        self.axs[3, 1].set_title("Right Motor Current")

        # 设置标签
        for i in range(4):
            self.axs[i, 0].set_ylabel("Value")
            self.axs[i, 0].set_xlim(0, self.window_size)
            self.axs[i, 1].set_xlim(0, self.window_size)

        self.axs[3, 0].set_xlabel("Time (s)")
        self.axs[3, 1].set_xlabel("Time (s)")

        # 初始化线条
        self.left_position_line, = self.axs[0, 0].plot([], [], label="Position", color="blue")
        self.left_velocity_line, = self.axs[1, 0].plot([], [], label="Velocity", color="green")
        self.left_torque_line, = self.axs[2, 0].plot([], [], label="Torque", color="red")
        self.left_current_line, = self.axs[3, 0].plot([], [], label="Current", color="purple")

        self.right_position_line, = self.axs[0, 1].plot([], [], label="Position", color="blue")
        self.right_velocity_line, = self.axs[1, 1].plot([], [], label="Velocity", color="green")
        self.right_torque_line, = self.axs[2, 1].plot([], [], label="Torque", color="red")
        self.right_current_line, = self.axs[3, 1].plot([], [], label="Current", color="purple")

        for ax in self.axs.flat:
            ax.legend(fontsize=8)

    def update_plot(self, frame):
        """
        更新绘图数据。
        """
        # 从 CSV 文件中读取数据
        with open(self.csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            rows = list(reader)

        # 清空数据并重新加载最新的数据
        self.time_data.clear()
        self.left_position.clear()
        self.left_velocity.clear()
        self.left_torque.clear()
        self.left_current.clear()
        self.right_position.clear()
        self.right_velocity.clear()
        self.right_torque.clear()
        self.right_current.clear()

        for row in rows:
            self.time_data.append(float(row["timestamp"]))
            self.left_position.append(float(row["position_left"]))
            self.left_velocity.append(float(row["velocity_left"]))
            self.left_torque.append(float(row["torque_left"]))
            self.left_current.append(float(row["current_left"]))
            self.right_position.append(float(row["position_right"]))
            self.right_velocity.append(float(row["velocity_right"]))
            self.right_torque.append(float(row["torque_right"]))
            self.right_current.append(float(row["current_right"]))

        # 保留窗口内的数据
        if len(self.time_data) > 0 and self.time_data[-1] > self.window_size:
            start_index = next(i for i, t in enumerate(self.time_data) if t >= self.time_data[-1] - self.window_size)
            self.time_data = self.time_data[start_index:]
            self.left_position = self.left_position[start_index:]
            self.left_velocity = self.left_velocity[start_index:]
            self.left_torque = self.left_torque[start_index:]
            self.left_current = self.left_current[start_index:]
            self.right_position = self.right_position[start_index:]
            self.right_velocity = self.right_velocity[start_index:]
            self.right_torque = self.right_torque[start_index:]
            self.right_current = self.right_current[start_index:]

        # 更新绘图
        self.left_position_line.set_data(self.time_data, self.left_position)
        self.left_velocity_line.set_data(self.time_data, self.left_velocity)
        self.left_torque_line.set_data(self.time_data, self.left_torque)
        self.left_current_line.set_data(self.time_data, self.left_current)

        self.right_position_line.set_data(self.time_data, self.right_position)
        self.right_velocity_line.set_data(self.time_data, self.right_velocity)
        self.right_torque_line.set_data(self.time_data, self.right_torque)
        self.right_current_line.set_data(self.time_data, self.right_current)

        # 设置坐标范围
        for ax, data in zip(self.axs[:, 0], [self.left_position, self.left_velocity, self.left_torque, self.left_current]):
            ax.set_ylim(min(data) - 0.1, max(data) + 0.1 if len(data) > 0 else 1)
        for ax, data in zip(self.axs[:, 1], [self.right_position, self.right_velocity, self.right_torque, self.right_current]):
            ax.set_ylim(min(data) - 0.1, max(data) + 0.1 if len(data) > 0 else 1)

        self.axs[0, 0].set_xlim(self.time_data[0], self.time_data[-1])
        self.axs[0, 1].set_xlim(self.time_data[0], self.time_data[-1])


    def run(self):
        """
        运行实时绘图。
        """
        ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        plt.show()


if __name__ == "__main__":
    # 使用保存的 CSV 文件进行实时绘图
    csv_file_path = "leg_impedance_data.csv"
    plotter = RealTimePlotter(csv_file=csv_file_path, window_size=40)
    plotter.run()
