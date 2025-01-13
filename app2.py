
import sys
import threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFormLayout, QLabel, QLineEdit, QPushButton, QTextEdit
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter
from PyQt5.QtChart import QChart, QChartView, QLineSeries
from motor.filteredmotorController import FilteredMotorController
from trajectory_handler.sineGenerator import SineTrajectoryHandler
from main_impedance_single import ImpedanceController  # 假设该类在 impedance_controller.py 中

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("阻抗控制界面")
        self.setGeometry(100, 100, 1200, 800)

        # 主窗口布局
        main_layout = QHBoxLayout()

        # 左侧参数设置区域
        self.left_widget = self.create_settings_panel()
        main_layout.addWidget(self.left_widget, 2)

        # 右侧实时可视化和日志区域
        self.right_widget = self.create_visualization_panel()
        main_layout.addWidget(self.right_widget, 3)

        # 主窗口容器
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # 初始化控制器相关变量
        self.controller_thread = None
        self.controller_running = False

    def create_settings_panel(self):
        """创建左侧参数设置区域"""
        left_widget = QWidget()
        layout = QVBoxLayout()

        # 阻抗控制参数组
        impedance_group = QGroupBox("阻抗控制参数")
        impedance_layout = QFormLayout()

        self.kp_input = QLineEdit("1.0")
        self.kd_input = QLineEdit("0.1")
        self.ki_input = QLineEdit("0.01")
        self.kf_input = QLineEdit("0.5")
        self.duration_input = QLineEdit("10")

        impedance_layout.addRow("Kp:", self.kp_input)
        impedance_layout.addRow("Kd:", self.kd_input)
        impedance_layout.addRow("Ki:", self.ki_input)
        impedance_layout.addRow("Kf:", self.kf_input)
        impedance_layout.addRow("持续时间 (秒):", self.duration_input)

        impedance_group.setLayout(impedance_layout)
        layout.addWidget(impedance_group)

        # 预期轨迹参数组
        trajectory_group = QGroupBox("预期轨迹参数")
        trajectory_layout = QFormLayout()

        self.amplitude_input = QLineEdit("0.5")
        self.frequency_input = QLineEdit("0.5")

        trajectory_layout.addRow("振幅:", self.amplitude_input)
        trajectory_layout.addRow("频率:", self.frequency_input)

        trajectory_group.setLayout(trajectory_layout)
        layout.addWidget(trajectory_group)

        # 操作按钮
        self.start_button = QPushButton("开始运行")
        self.stop_button = QPushButton("结束运行")
        self.start_button.clicked.connect(self.start_controller)
        self.stop_button.clicked.connect(self.stop_controller)

        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)

        # 占位符
        layout.addStretch()

        left_widget.setLayout(layout)
        return left_widget

    def create_visualization_panel(self):
        """创建右侧实时可视化和日志区域"""
        right_widget = QWidget()
        layout = QVBoxLayout()

        # 创建一个图表
        self.chart = QChart()
        self.chart.setTitle("实时数据可视化")
        self.series = QLineSeries()
        self.chart.addSeries(self.series)
        self.chart.createDefaultAxes()

        # 图表视图
        self.chart_view = QChartView(self.chart)
        self.chart_view.setRenderHint(QPainter.Antialiasing)
        layout.addWidget(self.chart_view)

        # 日志区域
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        layout.addWidget(self.log_output)

        right_widget.setLayout(layout)
        return right_widget

    def log_message(self, message):
        """在日志区域显示消息"""
        self.log_output.append(message)

    def start_controller(self):
        """启动阻抗控制器"""
        if self.controller_running:
            self.log_message("控制器正在运行中，请先停止当前运行！")
            return

        try:
            # 获取参数
            Kp = float(self.kp_input.text())
            Kd = float(self.kd_input.text())
            Ki = float(self.ki_input.text())
            Kf = float(self.kf_input.text())
            duration = float(self.duration_input.text())
            amplitude = float(self.amplitude_input.text())
            frequency = float(self.frequency_input.text())

            # 初始化电机和轨迹生成器
            motor = FilteredMotorController()
            motor.initialize_odrive()
            motor.set_torque_control_mode()
            trajectory_handler = SineTrajectoryHandler(amplitude=amplitude, frequency=frequency)

            # 创建控制器实例
            self.controller = ImpedanceController(
                motor_controller=motor,
                trajectory_handler=trajectory_handler,
                duration=duration,
                Kp=Kp,
                Kd=Kd,
                Ki=Ki,
                Kf=Kf
            )

            # 启动控制器线程
            self.controller_thread = threading.Thread(target=self.run_controller)
            self.controller_thread.start()
            self.controller_running = True
            self.log_message("控制器已启动！")

        except Exception as e:
            self.log_message(f"启动控制器时发生错误: {e}")

    def run_controller(self):
        """运行阻抗控制器"""
        try:
            self.controller.run()
            self.log_message("阻抗控制完成！")
        except Exception as e:
            self.log_message(f"运行控制器时发生错误: {e}")
        finally:
            self.controller_running = False

    def stop_controller(self):
        """停止阻抗控制器"""
        if self.controller_running:
            self.controller_running = False
            self.log_message("控制器已停止！")
        else:
            self.log_message("控制器未在运行中！")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())