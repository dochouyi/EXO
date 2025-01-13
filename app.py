import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFormLayout, QLabel, QLineEdit, QPushButton
)
from PyQt5.QtCore import Qt
from PyQt5.QtChart import QChart, QChartView, QLineSeries
from PyQt5.QtGui import QPainter  # 添加 QPainter 导入


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("阻抗控制与实时可视化")
        self.setGeometry(100, 100, 1000, 600)

        # 主窗口的布局
        main_layout = QHBoxLayout()

        # 左侧设置区域
        self.left_widget = self.create_settings_panel()
        main_layout.addWidget(self.left_widget, 2)

        # 右侧实时可视化区域
        self.right_widget = self.create_visualization_panel()
        main_layout.addWidget(self.right_widget, 3)

        # 主窗口容器
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

    def create_settings_panel(self):
        """创建左侧参数设置区域"""
        left_widget = QWidget()
        layout = QVBoxLayout()

        # 阻抗控制参数组
        impedance_group = QGroupBox("阻抗控制参数")
        impedance_layout = QFormLayout()

        self.kp_input = QLineEdit()
        self.kd_input = QLineEdit()
        self.ki_input = QLineEdit()
        self.kf_input = QLineEdit()

        impedance_layout.addRow("Kp:", self.kp_input)
        impedance_layout.addRow("Kd:", self.kd_input)
        impedance_layout.addRow("Ki:", self.ki_input)
        impedance_layout.addRow("Kf:", self.kf_input)

        impedance_group.setLayout(impedance_layout)
        layout.addWidget(impedance_group)

        # 预期轨迹-正弦参数组
        trajectory_group = QGroupBox("预期轨迹-正弦")
        trajectory_layout = QFormLayout()

        self.amplitude_input = QLineEdit()
        self.period_input = QLineEdit()
        self.frequency_input = QLineEdit()
        self.phase_input = QLineEdit()
        self.offset_input = QLineEdit()

        trajectory_layout.addRow("振幅:", self.amplitude_input)
        trajectory_layout.addRow("周期:", self.period_input)
        trajectory_layout.addRow("频率:", self.frequency_input)
        trajectory_layout.addRow("相位:", self.phase_input)
        trajectory_layout.addRow("偏置:", self.offset_input)

        trajectory_group.setLayout(trajectory_layout)
        layout.addWidget(trajectory_group)

        # 操作按钮
        self.start_button = QPushButton("开始运行")
        self.stop_button = QPushButton("结束运行")
        self.start_record_button = QPushButton("开始记录")
        self.stop_record_button = QPushButton("结束记录")

        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.start_record_button)
        layout.addWidget(self.stop_record_button)

        # 占位符
        layout.addStretch()

        left_widget.setLayout(layout)
        return left_widget

    def create_visualization_panel(self):
        """创建右侧实时可视化区域"""
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
        right_widget.setLayout(layout)
        return right_widget


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())