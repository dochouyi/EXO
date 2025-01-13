import time
import numpy as np
from collections import defaultdict
from motor.motorController import FilteredMotorController
from utils.realTimePlotterBase import RealTimePlotter
from trajectory_handler.sineGenerator import TrajectoryGenerator


class QLearningControllerWithTrajectory:
    """
    结合预期轨迹的Q学习控制器
    """
    def __init__(self, motor, trajectory, duration=10, num_states=20, num_actions=11, alpha=0.1, gamma=0.9, epsilon=0.1):
        self.motor = motor
        self.trajectory = trajectory
        self.duration = duration
        self.num_states = num_states
        self.num_actions = num_actions
        self.alpha = alpha  # 学习率
        self.gamma = gamma  # 折扣因子
        self.epsilon = epsilon  # 探索概率
        self.q_table = defaultdict(lambda: np.zeros(num_actions))  # Q表
        self.actions = np.linspace(-2.0, 2.0, num_actions)  # 动作（连续力矩离散化）
        self.plotter = RealTimePlotter()

    def discretize_state(self, position_error, velocity_error):
        """
        将连续的状态（误差）离散化
        """
        position_bin = int((position_error + 1) / 2 * (self.num_states - 1))
        velocity_bin = int((velocity_error + 1) / 2 * (self.num_states - 1))
        position_bin = np.clip(position_bin, 0, self.num_states - 1)
        velocity_bin = np.clip(velocity_bin, 0, self.num_states - 1)
        return (position_bin, velocity_bin)

    def choose_action(self, state):
        """
        根据ε-贪婪策略选择动作
        """
        if np.random.rand() < self.epsilon:
            return np.random.choice(len(self.actions))
        else:
            return np.argmax(self.q_table[state])

    def compute_reward(self, position_error, velocity_error):
        """
        根据误差计算奖励
        """
        return - (position_error**2 + 0.1 * velocity_error**2)

    def update_q_table(self, state, action, reward, next_state):
        """
        Q学习更新公式
        """
        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.gamma * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.alpha * td_error

    def run(self):
        """
        执行Q学习控制
        """
        start_time = time.time()
        print("开始结合预期轨迹的Q学习控制...")

        while time.time() - start_time < self.duration:
            t = time.time() - start_time
            current_position = self.motor.get_pos_estimate_filtered()
            current_velocity = self.motor.get_vel_estimate_filtered()

            # 获取预期轨迹
            desired_position = self.trajectory.get_position(t)
            desired_velocity = self.trajectory.get_velocity(t)

            # 计算误差
            position_error = desired_position - current_position
            velocity_error = desired_velocity - current_velocity

            # 离散化状态
            state = self.discretize_state(position_error, velocity_error)

            # 选择动作
            action_index = self.choose_action(state)
            torque = self.actions[action_index]

            # 执行动作
            self.motor.set_input_torque(torque)

            # 获取下一个状态
            next_position = self.motor.get_pos_estimate_filtered()
            next_velocity = self.motor.get_vel_estimate_filtered()
            next_position_error = desired_position - next_position
            next_velocity_error = desired_velocity - next_velocity
            next_state = self.discretize_state(next_position_error, next_velocity_error)

            # 计算奖励
            reward = self.compute_reward(next_position_error, next_velocity_error)

            # 更新Q表
            self.update_q_table(state, action_index, reward, next_state)


            # 控制循环频率
            time.sleep(0.001)

        print("结合预期轨迹的Q学习控制完成！")
        self.plotter.finalize()

# 主程序
if __name__ == "__main__":
    motor = FilteredMotorController()
    motor.initialize_odrive()
    motor.set_torque_control_mode()

    # 创建预期轨迹生成器
    trajectory = TrajectoryGenerator(amplitude=0.5, frequency=0.5)

    controller = QLearningControllerWithTrajectory(motor, trajectory, duration=10)
    controller.run()