#pragma once

#include "actuator.h"

#include <memory>
#include <string>
#include <string_view>
#include <vector>

// 前向声明，避免将 SerialPort.h（含非 inline 全局定义）引入使用方
class SerialPort;

namespace actuator {

/**
 * 机械臂执行器组管理器。
 *
 * 对外只暴露此类，底层电机驱动（DmActuator / Motor_Control）完全封装在 .cpp 内。
 *
 * 快速上手：
 *   // 从 YAML + 串口路径一步构造（最简方式）
 *   auto arm = ArmActuatorGroup::from_yaml("/dev/ttyACM0", "config/arm.yaml");
 *
 *   // 扫描哪些关节有响应
 *   auto connected = arm.scan_connectivity();   // vector<bool>，顺序同配置
 *
 *   arm.enable_all();
 *   arm["joint1"].set_position(0.5f);           // 按名字
 *   arm[0].set_velocity(1.0f);                  // 按索引
 *   auto pos = arm.get_all_positions();
 *   arm.disable_all();
 */
class ArmActuatorGroup {
public:
    ~ArmActuatorGroup();

    // ── 构造 ───────────────────────────────────────────────────────────────────

    /** 代码内嵌配置构造 */
    explicit ArmActuatorGroup(
        std::shared_ptr<SerialPort> serial,
        std::vector<JointConfig>    config
    );

    /** 从 YAML 文件构造，需要外部已创建的 SerialPort */
    static ArmActuatorGroup from_yaml(
        std::shared_ptr<SerialPort> serial,
        const std::string&          yaml_path
    );

    /** 从 YAML 文件构造，内部自动创建 SerialPort（最常用） */
    static ArmActuatorGroup from_yaml(
        const std::string& device_path,
        const std::string& yaml_path,
        int                baud_timeout_ms = 5
    );

    // ── 批量操作 ───────────────────────────────────────────────────────────────

    void enable_all();
    void disable_all();

    /**
     * 扫描各关节是否有响应。
     * @param timeout_ms  等待单次回包的超时（ms），100 ms 适合大多数场景
     * @return  vector<bool>，顺序同配置文件，true = 有响应
     */
    std::vector<bool> scan_connectivity(int timeout_ms = 100);

    /**
     * 批量位置控制（MIT 模式，增益取自配置文件）。
     * @param positions  各关节目标角度 [rad]，顺序同配置文件，长度须等于 size()
     */
    void set_all_positions(const std::vector<float>& positions);

    /**
     * 批量位置+速度控制（POS_VEL 模式）。
     * @param positions   各关节目标角度 [rad]，顺序同配置文件，长度须等于 size()
     * @param velocities  各关节运动速度 [rad/s]，长度须等于 size()
     */
    void set_all_positions_velocities(const std::vector<float>& positions,
                                      const std::vector<float>& velocities);

    /**
     * 批量位置控制（带统一速度上限）。
     *
     * 对于支持位置速度模式的执行器，将调用底层的位置速度控制接口，
     * 以 dq_max 作为 v_des 上限；其他执行器则退化为普通位置控制。
     *
     * @param positions  各关节目标角度 [rad]
     * @param dq_max     统一的最大角速度上限 [rad/s]，应为正值
     */
    void set_all_positions_with_speed_limit(const std::vector<float>& positions,
                                            float dq_max);

    /** 读取所有关节位置（顺序同配置文件） */
    std::vector<float> get_all_positions() const;

    /** 返回所有关节名称（顺序同配置文件） */
    std::vector<std::string> joint_names() const;

    // ── 单关节访问 ─────────────────────────────────────────────────────────────

    ActuatorBase&       operator[](std::string_view name);
    const ActuatorBase& operator[](std::string_view name) const;

    ActuatorBase&       operator[](std::size_t idx);
    const ActuatorBase& operator[](std::size_t idx) const;

    std::size_t size() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace actuator
