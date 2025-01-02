import math



class TrajectoryGenerator:
    """
    预期轨迹和速度生成器
    """
    def __init__(self, amplitude, frequency, phase=0.0):
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase  # 新增 phase 属性

    def get_position(self, t):
        # 考虑 phase 的影响
        return self.amplitude * math.cos(2 * math.pi * self.frequency * t + self.phase)

    def get_velocity(self, t):
        # 考虑 phase 的影响
        return -self.amplitude * 2 * math.pi * self.frequency * math.sin(2 * math.pi * self.frequency * t + self.phase)

    def update_parameters(self, amplitude, frequency, phase):
        """
        实时更新轨迹参数，包括 phase
        """
        self.amplitude = amplitude
        self.frequency = frequency
        self.phase = phase  # 更新 phase