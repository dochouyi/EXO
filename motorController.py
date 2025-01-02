import odrive
from odrive.enums import *


class MotorController:
    """
    电机控制类
    """
    def __init__(self):
        self.odrv0 = None

    def initialize_odrive(self):
        print("正在寻找 ODrive...")
        self.odrv0 = odrive.find_any()

        if self.odrv0:
            print("ODrive 已连接！")
        else:
            print("未找到 ODrive，请检查连接。")
            exit()

        print("清除错误...")
        self.odrv0.clear_errors()

        print("校准电机...")
        self.odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass
        self.odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            pass
        print("校准完成")

    def set_torque_control(self):
        print("设置控制模式为力矩控制...")
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        print("进入闭环控制模式...")
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    def stop_motor(self):
        print("停止电机...")
        self.odrv0.axis0.controller.input_torque = 0
        print("退出闭环控制模式...")
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE

