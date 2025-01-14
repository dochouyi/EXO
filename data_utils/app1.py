import sys
import csv
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QAction, QFileDialog, QVBoxLayout, QWidget, QLabel,
    QSlider, QTreeWidget, QTreeWidgetItem, QHBoxLayout, QInputDialog
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from motor.doubleMotorController import FilteredDoubleMotorController


class DataCollectorThread(QThread):
    data_collected = pyqtSignal(float, float, float, float, float, float, float)

    def __init__(self, motor_controller, data_file="data_log_double.csv"):
        super().__init__()
        self.motor_controller = motor_controller
        self.data_file = data_file
        self.running = False

        # 创建 CSV 文件并写入表头
        with open(self.data_file, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "timestamp",
                "position_left", "velocity_left", "torque_left",
                "position_right", "velocity_right", "torque_right"
            ])

    def run(self):
        self.running = True
        start_time = time.time()

        while self.running:
            current_time = time.time()
            elapsed_time = current_time - start_time

            # # 获取左右腿当前状态
            # position_left, position_right = self.motor_controller.get_pos_estimate()
            # velocity_left, velocity_right = self.motor_controller.get_vel_estimate()
            # torque_left, torque_right = self.motor_controller.get_torque_estimate()
            # 获取左右腿当前状态
            position_left, position_right = self.motor_controller.get_fake_pos_estimate()
            velocity_left, velocity_right = self.motor_controller.get_fake_vel_estimate()
            torque_left, torque_right = self.motor_controller.get_fake_torque_estimate()

            # 将数据写入 CSV 文件
            with open(self.data_file, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    elapsed_time,
                    position_left, velocity_left, torque_left,
                    position_right, velocity_right, torque_right
                ])

            # 发射信号更新界面
            self.data_collected.emit(
                elapsed_time,
                position_left, velocity_left, torque_left,
                position_right, velocity_right, torque_right
            )

            # 控制循环频率
            time.sleep(0.001)

    def stop(self):
        self.running = False


class ReplayThread(QThread):
    data_collected = pyqtSignal(float, float, float, float, float, float, float)

    def __init__(self, data_file, speed_multiplier=1):
        super().__init__()
        self.data_file = data_file
        self.speed_multiplier = speed_multiplier

    def run(self):
        try:
            with open(self.data_file, mode='r') as file:
                reader = csv.reader(file)
                next(reader)  # 跳过表头
                previous_time = 0
                for row in reader:
                    elapsed_time, position_left, velocity_left, torque_left, position_right, velocity_right, torque_right = map(float, row)
                    # 控制重放速度
                    time.sleep((elapsed_time - previous_time) / self.speed_multiplier)
                    previous_time = elapsed_time

                    # 发射信号更新界面
                    self.data_collected.emit(
                        elapsed_time,
                        position_left, velocity_left, torque_left,
                        position_right, velocity_right, torque_right
                    )
        except FileNotFoundError:
            print(f"文件 {self.data_file} 未找到，请确保数据已被记录。")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("数据采集与重放")
        self.setGeometry(100, 100, 800, 600)

        # 初始化电机控制器
        self.motor_controller = FilteredDoubleMotorController("1", "2")

        # 初始化数据采集线程
        self.data_collector_thread = None

        # 初始化界面
        self.initUI()

    def initUI(self):
        # 创建菜单
        menu = self.menuBar()
        file_menu = menu.addMenu("操作")

        start_action = QAction("开始采集数据", self)
        start_action.triggered.connect(self.start_data_collection)
        file_menu.addAction(start_action)

        stop_action = QAction("结束采集数据", self)
        stop_action.triggered.connect(self.stop_data_collection)
        file_menu.addAction(stop_action)

        replay_action = QAction("重放数据", self)
        replay_action.triggered.connect(self.replay_data)
        file_menu.addAction(replay_action)

        create_project_action = QAction("创建新项目", self)
        create_project_action.triggered.connect(self.create_project)
        file_menu.addAction(create_project_action)

        # 创建主窗口部件和布局
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.main_layout = QHBoxLayout(self.central_widget)

        # 添加树状列表
        self.project_tree = QTreeWidget()
        self.project_tree.setHeaderLabel("项目列表")
        self.project_tree.itemClicked.connect(self.load_project_data)
        self.main_layout.addWidget(self.project_tree)

        # 添加右侧布局
        self.right_layout = QVBoxLayout()
        self.main_layout.addLayout(self.right_layout)

        # 添加标签显示数据
        self.data_label = QLabel("实时数据：")
        self.right_layout.addWidget(self.data_label)

        # 添加加速滑块
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 8)
        self.speed_slider.setValue(1)
        self.speed_slider.setTickInterval(1)
        self.speed_slider.setTickPosition(QSlider.TicksBelow)
        self.right_layout.addWidget(self.speed_slider)

    def start_data_collection(self):
        if self.data_collector_thread is None or not self.data_collector_thread.isRunning():
            self.data_collector_thread = DataCollectorThread(self.motor_controller)
            self.data_collector_thread.data_collected.connect(self.update_data)
            self.data_collector_thread.start()

    def stop_data_collection(self):
        if self.data_collector_thread and self.data_collector_thread.isRunning():
            self.data_collector_thread.stop()
            self.data_collector_thread.wait()

    def replay_data(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "选择CSV文件", "", "CSV Files (*.csv);;All Files (*)", options=options)
        if file_name:
            speed_multiplier = self.speed_slider.value()
            replay_thread = ReplayThread(file_name, speed_multiplier)
            replay_thread.data_collected.connect(self.update_data)
            replay_thread.start()

    def create_project(self):
        # 创建新项目
        project_name, ok = QInputDialog.getText(self, "创建新项目", "请输入项目名称：")
        if ok and project_name:
            # 创建对应的 CSV 文件
            csv_file = f"{project_name}.csv"
            with open(csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "timestamp",
                    "position_left", "velocity_left", "torque_left",
                    "position_right", "velocity_right", "torque_right"
                ])
            # 添加到树状列表
            QTreeWidgetItem(self.project_tree, [project_name])

    def load_project_data(self, item):
        # 加载项目数据
        project_name = item.text(0)
        csv_file = f"{project_name}.csv"
        self.replay_data_from_file(csv_file)

    def replay_data_from_file(self, file_name):
        speed_multiplier = self.speed_slider.value()
        replay_thread = ReplayThread(file_name, speed_multiplier)
        replay_thread.data_collected.connect(self.update_data)
        replay_thread.start()

    def update_data(self, elapsed_time, position_left, velocity_left, torque_left, position_right, velocity_right, torque_right):
        self.data_label.setText(
            f"实时数据：\n"
            f"时间: {elapsed_time:.2f} s\n"
            f"左腿 - 位置: {position_left:.2f}, 速度: {velocity_left:.2f}, 力矩: {torque_left:.2f}\n"
            f"右腿 - 位置: {position_right:.2f}, 速度: {velocity_right:.2f}, 力矩: {torque_right:.2f}"
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
