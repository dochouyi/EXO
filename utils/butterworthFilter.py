from scipy.signal import butter, filtfilt
from collections import deque


class ButterworthFilter:
    """
    2阶巴特沃斯滤波器类
    """
    HISTORY_LIMIT = 100  # 历史数据存储限制

    def __init__(self, order, cutoff_freq, sampling_freq):
        """
        初始化滤波器参数
        :param order: 滤波器阶数
        :param cutoff_freq: 截止频率 (Hz)
        :param sampling_freq: 采样频率 (Hz)
        """
        self.b, self.a = butter(order, cutoff_freq / (0.5 * sampling_freq), btype='low')
        self.signal_list = deque(maxlen=self.HISTORY_LIMIT)  # 使用 deque 代替列表

    def apply(self, data):
        """
        应用巴特沃斯滤波器（双向滤波）
        :param data: 输入数据
        :return: 滤波后的数据
        """
        return filtfilt(self.b, self.a, data)

    def filter_signal(self, new_value):
        """
        更新历史数据并进行滤波
        :param new_value: 新的信号值
        :return: 滤波后的信号值
        """
        self.signal_list.append(new_value)
        # 返回滤波后的值
        return self.apply(list(self.signal_list))[-1]
