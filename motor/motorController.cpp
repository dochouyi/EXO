#include <iostream>
#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>
#include "can_library.h" // 假设存在一个 CAN 通信库

class MotorController {
public:
    /**
     * 电机控制类，用于控制和管理 ODrive 电机。
     */
    MotorController(int can_id)
        : can_id(can_id), can_interface("can0") {}

    /**
     * 初始化 CAN 通信。
     */
    void initialize_can() {
        try {
            std::cout << "正在初始化 CAN 通信..." << std::endl;
            if (!can.open(can_interface)) {
                throw std::runtime_error("无法打开 CAN 接口，请检查连接。");
            }
            std::cout << "CAN 通信已初始化。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "初始化 CAN 失败: " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    /**
     * 校准电机，包括电机和编码器的校准。
     */
    void calibrate_motor() {
        try {
            std::cout << "开始校准..." << std::endl;
            send_can_command(AXIS_STATE_MOTOR_CALIBRATION);
            wait_for_idle_state();
            send_can_command(AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
            wait_for_idle_state();
            std::cout << "校准完成。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "校准失败: " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    /**
     * 设置电机为力矩控制模式。
     */
    void set_torque_control_mode() {
        try {
            send_can_command(CONTROL_MODE_TORQUE_CONTROL, true);
            send_can_command(AXIS_STATE_CLOSED_LOOP_CONTROL);
            std::cout << "已设置为力矩控制模式。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "设置力矩控制模式失败: " << e.what() << std::endl;
        }
    }

    /**
     * 停止电机。
     */
    void stop_motor() {
        try {
            send_can_command(AXIS_STATE_IDLE);
            std::cout << "电机已停止。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "停止电机失败: " << e.what() << std::endl;
        }
    }

    /**
     * 设置输入力矩。
     * @param torque_value 力矩值。
     */
    void set_input_torque(float torque_value) {
        try {
            send_can_data(INPUT_TORQUE_CAN_ID, torque_value);
        } catch (const std::exception& e) {
            std::cerr << "设置输入力矩失败: " << e.what() << std::endl;
        }
    }

    /**
     * 获取力矩常数。
     * @return 力矩常数值。
     */
    float get_torque_constant() const {
        return read_can_data(TORQUE_CONSTANT_CAN_ID);
    }

    /**
     * 获取测量的电流 Iq。
     * @return Iq 测量值。
     */
    float get_Iq_measured() const {
        return read_can_data(IQ_MEASURED_CAN_ID);
    }

    /**
     * 获取力矩估算值。
     * @return 力矩估算值。
     */
    float get_torque_estimate() const {
        return read_can_data(TORQUE_ESTIMATE_CAN_ID);
    }

    /**
     * 获取速度估算值。
     * @return 速度估算值。
     */
    float get_vel_estimate() const {
        return read_can_data(VEL_ESTIMATE_CAN_ID);
    }

    /**
     * 获取位置估算值。
     * @return 位置估算值。
     */
    float get_pos_estimate() const {
        return read_can_data(POS_ESTIMATE_CAN_ID);
    }

private:
    int can_id; // ODrive 的 CAN ID
    std::string can_interface;
    CANLibrary can; // 假设 CANLibrary 是一个封装的 CAN 通信类

    static constexpr int AXIS_STATE_MOTOR_CALIBRATION = 0x01;
    static constexpr int AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 0x03;
    static constexpr int AXIS_STATE_CLOSED_LOOP_CONTROL = 0x08;
    static constexpr int AXIS_STATE_IDLE = 0x00;

    static constexpr int CONTROL_MODE_TORQUE_CONTROL = 0x0A;

    static constexpr int INPUT_TORQUE_CAN_ID = 0x0C;
    static constexpr int TORQUE_CONSTANT_CAN_ID = 0x0D;
    static constexpr int IQ_MEASURED_CAN_ID = 0x0E;
    static constexpr int TORQUE_ESTIMATE_CAN_ID = 0x0F;
    static constexpr int VEL_ESTIMATE_CAN_ID = 0x10;
    static constexpr int POS_ESTIMATE_CAN_ID = 0x11;

    /**
     * 通过 CAN 发送命令。
     * @param command 命令。
     * @param is_control_mode 是否为控制模式命令。
     */
    void send_can_command(int command, bool is_control_mode = false) {
        CANMessage msg;
        msg.id = can_id | (is_control_mode ? 0x100 : 0x200);
        msg.data[0] = command;
        msg.dlc = 1; // 数据长度
        can.send(msg);
    }

    /**
     * 通过 CAN 发送数据。
     * @param can_id 数据对应的 CAN ID。
     * @param value 数据值。
     */
    void send_can_data(int can_id, float value) {
        CANMessage msg;
        msg.id = this->can_id | can_id;
        std::memcpy(msg.data, &value, sizeof(float));
        msg.dlc = sizeof(float);
        can.send(msg);
    }

    /**
     * 通过 CAN 读取数据。
     * @param can_id 数据对应的 CAN ID。
     * @return 读取到的浮点数值。
     */
    float read_can_data(int can_id) const {
        CANMessage msg;
        if (can.receive(this->can_id | can_id, msg)) {
            float value;
            std::memcpy(&value, msg.data, sizeof(float));
            return value;
        } else {
            throw std::runtime_error("读取 CAN 数据失败");
        }
    }

    /**
     * 等待电机空闲状态。
     */
    void wait_for_idle_state() {
        while (read_can_data(AXIS_STATE_IDLE) != AXIS_STATE_IDLE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};
