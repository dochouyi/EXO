import time
import math
import matplotlib
import matplotlib.pyplot as plt

matplotlib.use('TkAgg')  # 设置后端为 TkAgg


class RealTimePlotterBase:
    def __init__(self, data_labels, window_size=40, update_interval=10, layout=(1, 1), y_lims=None, fig_width=10):
        """
        :param data_labels: 数据标签列表，例如 ["Position", "Velocity", "Torque", "External Torque"]
        :param window_size: 横坐标窗口大小
        :param update_interval: 更新图像的步数间隔
        :param layout: 子图布局，(行数, 列数)
        :param y_lims: 每个数据的 y 轴范围，字典格式 {label: (y_min, y_max)}
        """
        self.time_data = []
        self.data = {label: [] for label in data_labels}  # 动态存储每种数据
        self.window_size = window_size
        self.update_interval = update_interval
        self.update_counter = 0
        self.y_lims = y_lims if y_lims else {}  # 如果未提供 y_lims，则初始化为空字典

        # 根据数据标签数量和布局动态调整子图
        num_plots = len(data_labels)
        rows, cols = layout
        if rows * cols < num_plots:
            raise ValueError(f"布局 ({rows}, {cols}) 不足以容纳 {num_plots} 个子图，请调整布局参数。")

        plt.ion()
        self.fig, self.axs = plt.subplots(rows, cols, figsize=(fig_width, 12), sharex=True)

        # 将 axs 转换为一维列表，方便索引
        if isinstance(self.axs, plt.Axes):
            self.axs = [self.axs]
        else:
            self.axs = self.axs.flatten()

        self.lines = {}
        for i, label in enumerate(data_labels):
            ax = self.axs[i]
            ax.set_title(f"{label} Over Time", fontsize=14)
            ax.set_ylabel(f"{label} (units)", fontsize=12)
            ax.set_xlim(0, 50)  # 初始化横坐标范围

            # 设置 y 轴范围
            if label in self.y_lims:
                y_min, y_max = self.y_lims[label]
                ax.set_ylim(y_min, y_max)
            else:
                ax.set_ylim(-10, 10)  # 默认纵坐标范围

            # 动态生成每个子图的曲线
            self.lines[label], = ax.plot([], [], label=label)

        # 隐藏多余的子图
        for j in range(num_plots, len(self.axs)):
            self.axs[j].set_visible(False)

        self.axs[-1].set_xlabel("Time (s)", fontsize=12)  # 最后一张子图设置横坐标标签

    def update(self, time, **data_values):
        """
        更新图像数据
        :param time: 时间点
        :param data_values: 动态关键字参数，键为数据标签，值为对应的值
        """
        self.time_data.append(time)
        for label, value in data_values.items():
            if label in self.data:
                self.data[label].append(value)

        # 移除超出窗口的数据
        if self.time_data[-1] - self.time_data[0] > self.window_size:
            self.time_data.pop(0)
            for label in self.data:
                self.data[label].pop(0)

        self.update_counter += 1
        if self.update_counter % self.update_interval == 0:  # 每隔一定步数更新一次图像
            for label, line in self.lines.items():
                line.set_data(self.time_data, self.data[label])

            # 设置横坐标范围
            for ax in self.axs:
                if ax.get_visible():  # 仅更新可见子图
                    if self.time_data[-1] < self.window_size:  # 如果时间不足窗口大小
                        ax.set_xlim(0, self.window_size)
                    else:  # 如果时间超过窗口大小
                        ax.set_xlim(self.time_data[-1] - self.window_size, self.time_data[-1])

            # 强制更新绘图
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def finalize(self):
        plt.ioff()
        plt.show()




