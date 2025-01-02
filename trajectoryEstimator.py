from collections import deque
import numpy as np
from scipy.optimize import curve_fit


class TrajectoryEstimator:
    """
    用户腿部动作轨迹估计器
    """
    def __init__(self, window_size):
        self.window_size = window_size
        self.time_window = deque(maxlen=window_size)
        self.position_data = deque(maxlen=window_size)

    def update_data(self, time, position):
        self.time_window.append(time)
        self.position_data.append(position)

    def sinusoidal_fit(self):
        def sinusoidal_model(t, amplitude, frequency, phase):
            return amplitude * np.cos(2 * np.pi * frequency * t + phase)

        if len(self.time_window) < 3:
            raise ValueError("Not enough data for sinusoidal fitting")

        time_data = np.array(self.time_window)
        position_data = np.array(self.position_data)
        initial_guess = [np.max(position_data), 1.0, 0.0]
        params, _ = curve_fit(sinusoidal_model, time_data, position_data, p0=initial_guess)
        return params