import time
from realTimePlotter import RealTimePlotter
from motorController import MotorController
from trajectoryGenerator import TrajectoryGenerator
from trajectoryEstimator import TrajectoryEstimator
from scipy.signal import butter, lfilter


class ButterworthFilter:
    """
    2阶巴特沃斯滤波器类
    """
    def __init__(self, order, cutoff_freq, sampling_freq):
        """
        初始化滤波器参数
        :param order: 滤波器阶数
        :param cutoff_freq: 截止频率 (Hz)
        :param sampling_freq: 采样频率 (Hz)
        """
        self.b, self.a = butter(order, cutoff_freq / (0.5 * sampling_freq), btype='low')

    def apply(self, data):
        """
        应用巴特沃斯滤波器
        :param data: 输入数据
        :return: 滤波后的数据
        """
        return lfilter(self.b, self.a, data)


class ImpedanceController:
    """
    阻抗控制类
    """
    MAX_TORQUE = 2.0  # 最大力矩限制
    HISTORY_LIMIT = 100  # 历史数据存储限制

    def __init__(self, motor: MotorController, trajectory: TrajectoryGenerator, estimator: TrajectoryEstimator, duration: float, Kp=1.0, Kd=0.1, Kf=0.5):
        self.motor = motor
        self.trajectory = trajectory
        self.estimator = estimator
        self.duration = duration
        self.Kp = Kp
        self.Kd = Kd
        self.Kf = Kf
        self.plotter = RealTimePlotter()

        # 初始化巴特沃斯滤波器
        self.position_filter = ButterworthFilter(order=2, cutoff_freq=10, sampling_freq=1000)
        self.velocity_filter = ButterworthFilter(order=2, cutoff_freq=10, sampling_freq=1000)
        self.current_filter = ButterworthFilter(order=2, cutoff_freq=10, sampling_freq=1000)

    def estimate_external_torque(self, input_torque):
        """
        估计外部力矩
        :param input_torque: 输入力矩
        :return: 外部力矩
        """
        torque_constant = self.motor.odrv0.axis0.motor.config.torque_constant
        measured_current = self.motor.odrv0.axis0.motor.current_control.Iq_measured

        # 对电流测量值进行滤波
        filtered_current = self.current_filter.apply([measured_current])[-1]
        measured_torque = filtered_current * torque_constant
        return measured_torque - input_torque

    def filter_signal(self, signal_history, new_value, filter_instance):
        """
        更新历史数据并进行滤波
        :param signal_history: 信号历史记录
        :param new_value: 新的信号值
        :param filter_instance: 滤波器实例
        :return: 滤波后的信号值
        """
        signal_history.append(new_value)
        if len(signal_history) > self.HISTORY_LIMIT:
            signal_history.pop(0)
        # 返回滤波后的值
        return filter_instance.apply(signal_history)[-1]

    def run(self):
        """
        执行阻抗控制
        """
        start_time = time.time()
        print("开始阻抗控制...")

        pos_history = []
        vel_history = []

        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            current_position = self.motor.odrv0.axis0.encoder.pos_estimate
            current_velocity = self.motor.odrv0.axis0.encoder.vel_estimate

            # 对位置和速度进行滤波，同时更新历史记录
            filtered_position = self.filter_signal(pos_history, current_position, self.position_filter)
            filtered_velocity = self.filter_signal(vel_history, current_velocity, self.velocity_filter)

            # 更新估计器数据
            self.estimator.update_data(t, filtered_position)

            # 更新轨迹参数
            try:
                amplitude, frequency, _ = self.estimator.sinusoidal_fit()
                self.trajectory.update_parameters(amplitude, frequency)
            except ValueError:
                pass

            # 获取期望位置和速度
            desired_position = self.trajectory.get_position(t)
            desired_velocity = self.trajectory.get_velocity(t)

            # 计算目标力矩
            target_torque = self.Kp * (desired_position - filtered_position) + self.Kd * (desired_velocity - filtered_velocity)

            # 调整目标力矩
            external_torque = self.estimate_external_torque(target_torque)
            adjusted_torque = target_torque + self.Kf * external_torque

            # 限制力矩范围
            adjusted_torque = max(min(adjusted_torque, self.MAX_TORQUE), -self.MAX_TORQUE)

            # 设置目标力矩
            self.motor.odrv0.axis0.controller.input_torque = adjusted_torque

            # 更新实时图形
            self.plotter.update(t, filtered_position, filtered_velocity, adjusted_torque, external_torque)

            # 控制循环频率
            time.sleep(0.001)

        print("阻抗控制完成！")
        self.plotter.finalize()


# 主程序
if __name__ == "__main__":
    motor = MotorController()
    motor.initialize_odrive()
    motor.set_torque_control()

    trajectory = TrajectoryGenerator(amplitude=0.5, frequency=0.5)
    estimator = TrajectoryEstimator()

    controller = ImpedanceController(motor, trajectory, estimator, duration=10)
    controller.run()