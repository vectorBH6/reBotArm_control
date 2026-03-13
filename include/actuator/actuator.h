#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace actuator {

/**
 * 关节控制模式，对应达妙电机底层控制框架。
 *
 *   MIT     : 阻抗控制，τ = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
 *             参数 mit_kp / mit_kd 取自配置文件。
 *   POS_VEL : 位置串级模式（位置环→速度环→电流环），
 *             p_des 为目标角度，v_des 为运动速度上限（须为正值）。
 *   VEL     : 纯速度模式。
 */
enum class CtrlMode { MIT, POS_VEL, VEL };

/**
 * 单关节执行器抽象接口。
 *
 * 统一封装不同品牌/协议的电机驱动，上层运动控制代码只依赖此接口。
 * 当前支持达妙（DaMiao）MIT 协议，后续可扩展至其他品牌。
 */
class ActuatorBase {
public:
    virtual ~ActuatorBase() = default;

    /** 上电使能（进入闭环控制状态） */
    virtual void enable()  = 0;

    /** 下电失能（释放扭矩，电机自由转动） */
    virtual void disable() = 0;

    /**
     * MIT 阻抗控制模式（Motor Impedance Torque）。
     *
     * 输出力矩 τ = kp*(q_des - q) + kd*(dq_des - dq) + tau_ff
     *
     * @param kp    位置刚度增益 [N·m/rad]，典型值 0~500
     * @param kd    速度阻尼增益 [N·m·s/rad]，典型值 0~5
     * @param q     目标关节角度 [rad]
     * @param dq    目标关节角速度 [rad/s]，纯位置控制时填 0
     * @param tau   前馈力矩 [N·m]，无前馈时填 0
     */
    virtual void set_mit(float kp, float kd, float q, float dq, float tau) = 0;

    /**
     * 位置控制模式（内部以 MIT 实现，kp/kd 取自 JointConfig 中的配置值）。
     *
     * 等价于 set_mit(cfg.kp, cfg.kd, q, 0, 0)，适用于低动态的点到点运动。
     * @param q  目标关节角度 [rad]
     */
    virtual void set_position(float q) = 0;

    /**
     * 位置控制（带速度上限）。
     *
     * 默认实现直接调用 set_position(q)，忽略 dq_max，仅对底层支持位置速度模式
     * 的执行器（如达妙 POS_VEL 模式）重载以生效。
     *
     * @param q      目标关节角度 [rad]
     * @param dq_max 运动过程中的最大绝对角速度上限 [rad/s]，应为正值
     */
    virtual void set_position_with_speed_limit(float q, float dq_max) {
        (void)dq_max;
        set_position(q);
    }

    /**
     * 位置+速度控制模式（POS_VEL 模式）。
     *
     * 以目标位置为终点、目标速度为运动速度发送控制帧，
     *
     * @param q   目标关节角度 [rad]
     * @param dq  运动速度 [rad/s]，须为正值，表示接近目标的速度上限
     */
    virtual void set_position_velocity(float q, float dq) = 0;

    /**
     * 速度控制模式。
     * @param dq  目标关节角速度 [rad/s]
     */
    virtual void set_velocity(float dq) = 0;

    /** 读取最近一次反馈的关节角度 [rad] */
    virtual float get_position() const = 0;

    /** 读取最近一次反馈的关节角速度 [rad/s] */
    virtual float get_velocity() const = 0;

    /** 读取最近一次反馈的关节输出力矩 [N·m] */
    virtual float get_torque() const = 0;

    /**
     * 主动查询电机当前状态，刷新 position/velocity/torque 反馈缓存。
     * 不下发任何控制指令，适合待机状态下的状态监控。
     */
    virtual void refresh_status() = 0;

    /**
     * 将电机当前位置写入固件作为新零点（发送 0xFE 命令帧）。
     *
     * @warning 此操作直接写入电机非易失存储，掉电后仍有效，不可撤销。
     *          执行后 get_position() 将从 0 开始重新计数。
     *          调用前请确保机械臂处于预期的零点姿态。
     */
    virtual void set_zero_position() = 0;
};

// ─── 关节配置描述符 ────────────────────────────────────────────────────────────

/**
 * 单关节配置描述符，与 config/arm.yaml 中的每一条 joints 条目一一对应。
 *
 * CAN ID 说明：
 *   达妙电机采用扩展 CAN 帧，slave_id 即电机拨码开关设定的 ESC_ID，
 *   master_id 为上位机发送帧的 ID（默认 slave_id + 0x10）。
 *
 * MIT 增益说明：
 *   set_position() 内部以 MIT 模式实现，kp/kd 为该关节的默认刚度/阻尼。
 *   需要更高动态精度时可直接调用 set_mit() 并在运行时覆盖增益。
 */
struct JointConfig {
    std::string name;       // 关节标识符，如 "joint1"，用于按名索引
    std::string type;       // 电机型号，决定力矩/速度/位置的量程上限
    uint32_t    slave_id;   // 电机 CAN ID（对应拨码开关 ESC_ID）
    uint32_t    master_id;  // 主机 CAN ID（默认 slave_id + 0x10）
    float       mit_kp;     // MIT 位置刚度 [N·m/rad]，用于 set_position()（MIT 模式）
    float       mit_kd;     // MIT 速度阻尼 [N·m·s/rad]，用于 set_position()（MIT 模式）
    CtrlMode    ctrl_mode     = CtrlMode::MIT;  // 控制模式，默认 MIT；可在 yaml 中设为 POS_VEL / VEL
    float       pos_vel_speed = 5.0f;           // POS_VEL 模式下的速度上限 [rad/s]，须为正值
};

}  // namespace actuator
