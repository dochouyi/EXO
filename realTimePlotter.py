import time
import math
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('TkAgg')  # 设置后端为 TkAgg

class RealTimePlotter:
    def __init__(self, window_size=40, update_interval=10):  # 增加 update_interval 参数
        self.time_data = []
        self.position_data = []
        self.velocity_data = []
        self.torque_data = []
        self.external_torque_data = []
        self.window_size = window_size
        self.update_interval = update_interval  # 控制绘图频率
        self.update_counter = 0

        plt.ion()
        self.fig, self.axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

        self.axs[0].set_title("Motor Position Over Time", fontsize=14)
        self.axs[0].set_ylabel("Position (units)", fontsize=12)

        self.axs[1].set_title("Motor Velocity Over Time", fontsize=14)
        self.axs[1].set_ylabel("Velocity (units)", fontsize=12)

        self.axs[2].set_title("Motor Torque Over Time", fontsize=14)
        self.axs[2].set_ylabel("Torque (Nm)", fontsize=12)

        self.axs[3].set_title("External Input Torque Over Time", fontsize=14)
        self.axs[3].set_xlabel("Time (s)", fontsize=12)
        self.axs[3].set_ylabel("External Torque (Nm)", fontsize=12)

        self.position_line, = self.axs[0].plot([], [], label="Position", color="blue")
        self.velocity_line, = self.axs[1].plot([], [], label="Velocity", color="green")
        self.torque_line, = self.axs[2].plot([], [], label="Torque", color="red")
        self.external_torque_line, = self.axs[3].plot([], [], label="External Torque", color="purple")

        for ax in self.axs:
            ax.legend(fontsize=10)
            ax.set_xlim(0, 50)  # 初始化时，横坐标范围固定为 [0, 40]

        self.axs[0].set_ylim(-1.5, 1.5)  # 位置的默认纵坐标范围
        self.axs[1].set_ylim(-1.5, 1.5)  # 速度的默认纵坐标范围
        self.axs[2].set_ylim(-0.6, 0.6)  # 转矩的默认纵坐标范围
        self.axs[3].set_ylim(-0.4, 0.4)  # 外部转矩的默认纵坐标范围

    def update(self, time, position, velocity, torque, external_torque):
        self.time_data.append(time)
        self.position_data.append(position)
        self.velocity_data.append(velocity)
        self.torque_data.append(torque)
        self.external_torque_data.append(external_torque)

        # 移除超出窗口的数据
        if self.time_data[-1] - self.time_data[0] > self.window_size:
            self.time_data.pop(0)
            self.position_data.pop(0)
            self.velocity_data.pop(0)
            self.torque_data.pop(0)
            self.external_torque_data.pop(0)

        self.update_counter += 1
        if self.update_counter % self.update_interval == 0:  # 每隔一定步数更新一次图像
            self.position_line.set_data(self.time_data, self.position_data)
            self.velocity_line.set_data(self.time_data, self.velocity_data)
            self.torque_line.set_data(self.time_data, self.torque_data)
            self.external_torque_line.set_data(self.time_data, self.external_torque_data)

            # 设置横坐标范围
            for ax in self.axs:
                if self.time_data[-1] < 40:  # 如果时间不足 40 秒
                    ax.set_xlim(0, 40)  # 固定横坐标范围为 [0, 40]
                else:  # 如果时间超过 40 秒
                    ax.set_xlim(self.time_data[-1] - 40, self.time_data[-1])  # 显示最近 40 秒

            # 强制更新绘图
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def finalize(self):
        plt.ioff()
        plt.show()

