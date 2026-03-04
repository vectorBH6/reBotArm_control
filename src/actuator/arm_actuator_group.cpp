#include "actuator/arm_actuator_group.h"
#include "actuator/dm_hw/u2can/damiao.h"

#include <yaml-cpp/yaml.h>
#include <memory>
#include <stdexcept>
#include <utility>
#include <unordered_map>
#include <fcntl.h>   // open()
#include <unistd.h>  // close()

namespace actuator {

// ═══════════════════════════════════════════════════════════════════════════════
//  DmActuator — 达妙电机单关节驱动
//
//  通信链路：上位机 → USB-to-CAN (u2can) → CAN 总线 → 电机 ESC
//
//  每个关节持有：
//    mc_     : 整条 CAN 总线的控制器引用（多电机共用，由 DmActuatorFactory 管理）
//    motor_  : 该关节电机的状态缓存（ID、型号、位置/速度/力矩反馈）
//    kp_/kd_ : set_position() 的默认 MIT 刚度/阻尼增益
// ═══════════════════════════════════════════════════════════════════════════════

class DmActuator : public ActuatorBase {
public:
    DmActuator(damiao::Motor_Control& mc,
               damiao::DM_Motor_Type  type,
               uint32_t slave_id, uint32_t master_id,
               float kp, float kd)
        : mc_(mc), motor_(type, slave_id, master_id), kp_(kp), kd_(kd)
    {
        mc_.addMotor(&motor_);  // 向控制器注册，后续收包时通过 CAN ID 路由反馈
    }

    // 发送 0xFC 使能帧，电机进入闭环控制
    void enable()  override { mc_.enable(motor_); }

    // 发送 0xFD 失能帧，电机释放扭矩（约 100 ms 完成）
    void disable() override { mc_.disable(motor_); }

    // MIT 阻抗控制：τ = kp*(q_des-q) + kd*(dq_des-dq) + tau_ff
    // 参数经 float_to_uint 量化后打包进 8 字节 CAN 数据帧发出
    void set_mit(float kp, float kd, float q, float dq, float tau) override {
        mc_.control_mit(motor_, kp, kd, q, dq, tau);
    }

    // 位置控制：以配置增益做 MIT，速度目标和前馈力矩均置零
    void set_position(float q) override {
        mc_.control_mit(motor_, kp_, kd_, q, 0.f, 0.f);
    }

    void set_position_velocity(float q, float dq) override {
        mc_.control_pos_vel(motor_, q, dq);
    }

    void set_velocity(float dq) override {
        mc_.control_vel(motor_, dq);
    }

    // 发送 0xCC 状态查询帧，等待电机回包后更新 motor_ 中的反馈缓存
    void refresh_status() override {
        mc_.refresh_motor_status(motor_);
    }

    // 发送 0xFE 零点写入命令（写入电机非易失存储，约 100 ms 完成）
    void set_zero_position() override {
        mc_.set_zero_position(motor_);
    }

    // 从 motor_ 缓存中读取最近一次 CAN 回包解析出的反馈值
    float get_position() const override { return motor_.Get_Position(); }
    float get_velocity() const override { return motor_.Get_Velocity(); }
    float get_torque()   const override { return motor_.Get_tau(); }

private:
    damiao::Motor_Control& mc_;     // 共享的 CAN 总线控制器（同总线所有电机公用）
    damiao::Motor          motor_;  // 本关节电机状态（CAN ID + 型号 + 反馈缓存）
    float kp_, kd_;                 // set_position() 的默认 MIT 刚度/阻尼
};

// ═══════════════════════════════════════════════════════════════════════════════
//  ActuatorFactory — 执行器工厂接口
//
//  职责：按电机型号字符串识别品牌，创建对应的 ActuatorBase 实例。
//  每种品牌实现一个子类，并在 ArmActuatorGroup 构造函数中注册。
//
//  扩展新品牌的步骤：
//    1. 实现 XxxActuator : ActuatorBase（该品牌的通信协议）
//    2. 实现 XxxActuatorFactory : ActuatorFactory（型号识别 + 实例创建）
//    3. 在 ArmActuatorGroup 构造函数「注册工厂」处添加一行
// ═══════════════════════════════════════════════════════════════════════════════

class ActuatorFactory {
public:
    virtual ~ActuatorFactory() = default;

    /**
     * 判断此工厂是否负责处理给定型号。
     * @param type  来自 JointConfig::type 的型号字符串，如 "DM4340"
     */
    virtual bool accepts(const std::string& type) const = 0;

    /**
     * 根据关节配置创建一个执行器实例。
     * 工厂内部持有该品牌的共享控制器，并将新电机注册到控制器。
     */
    virtual std::unique_ptr<ActuatorBase> create(const JointConfig& cfg) = 0;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  DmActuatorFactory — 达妙电机工厂
//
//  持有一个 Motor_Control 实例（对应一个 u2can 适配器 / 一条 CAN 总线）。
//  同一总线上的所有达妙电机共享此控制器，由工厂负责统一管理。
//
//  型号识别规则：达妙所有型号均以 "DM" 为前缀（DM4310、DMH3510、DMG6220 等）。
// ═══════════════════════════════════════════════════════════════════════════════

class DmActuatorFactory : public ActuatorFactory {
public:
    explicit DmActuatorFactory(std::shared_ptr<SerialPort> serial)
        : mc_(serial) {}

    bool accepts(const std::string& type) const override {
        return type.rfind("DM", 0) == 0;
    }

    std::unique_ptr<ActuatorBase> create(const JointConfig& cfg) override {
        return std::make_unique<DmActuator>(
            mc_,
            parse_type(cfg.type),
            cfg.slave_id, cfg.master_id,
            cfg.mit_kp, cfg.mit_kd
        );
    }

private:
    /**
     * 将 YAML 中的型号字符串映射到 DM_Motor_Type 枚举。
     *
     * 不同型号的量程上限（Q_MAX / DQ_MAX / TAU_MAX）由枚举值索引到
     * damiao::limit_param[] 表格，影响 CAN 帧中的 float↔uint 量化精度。
     * 若型号填错会导致数值解析异常，因此在此处做强校验。
     */
    static damiao::DM_Motor_Type parse_type(const std::string& s) {
        static const std::unordered_map<std::string, damiao::DM_Motor_Type> table = {
            { "DM4310",    damiao::DM4310     }, { "DM4310_48V", damiao::DM4310_48V },
            { "DM4340",    damiao::DM4340     }, { "DM4340_48V", damiao::DM4340_48V },
            { "DM6006",    damiao::DM6006     }, { "DM6248P",    damiao::DM6248P    },
            { "DM8006",    damiao::DM8006     }, { "DM8009",     damiao::DM8009     },
            { "DM10010L",  damiao::DM10010L   }, { "DM10010",    damiao::DM10010    },
            { "DMH3510",   damiao::DMH3510    }, { "DMH6215",    damiao::DMH6215    },
            { "DMG6220",   damiao::DMG6220    }, { "DMJH11",     damiao::DMJH11     },
        };
        auto it = table.find(s);
        if (it == table.end())
            throw std::runtime_error("Unknown DM motor type: \"" + s + "\"");
        return it->second;
    }

    damiao::Motor_Control mc_;  // 该 CAN 总线的帧收发控制器
};

// ─────────────────────────────────────────────────────────────────────────────
//  其他品牌工厂模板（按需取消注释并补全实现）
//
//  class MiActuator : public ActuatorBase { ... };
//
//  class MiActuatorFactory : public ActuatorFactory {
//  public:
//      explicit MiActuatorFactory(std::shared_ptr<SerialPort> serial) : ... {}
//      bool accepts(const std::string& type) const override {
//          return type.rfind("MI", 0) == 0;  // 小米电机型号前缀
//      }
//      std::unique_ptr<ActuatorBase> create(const JointConfig& cfg) override {
//          return std::make_unique<MiActuator>(...);
//      }
//  };
// ─────────────────────────────────────────────────────────────────────────────

// ═══════════════════════════════════════════════════════════════════════════════
//  Pimpl 数据（对外隐藏所有底层依赖）
// ═══════════════════════════════════════════════════════════════════════════════

struct ArmActuatorGroup::Impl {
    std::shared_ptr<SerialPort>                   serial;      // 物理串口句柄，用于扫描时调整超时
    std::vector<std::unique_ptr<ActuatorFactory>> factories;   // 已注册的品牌工厂列表
    std::vector<std::unique_ptr<ActuatorBase>>    joints;      // 所有关节（可多品牌混用）
    std::vector<std::string>                      names;       // 关节名有序列表（索引同 joints）
    std::unordered_map<std::string, std::size_t>  name_to_idx; // 名称 → 索引快速查找
};

// ═══════════════════════════════════════════════════════════════════════════════
//  YAML 解析
// ═══════════════════════════════════════════════════════════════════════════════

static std::vector<JointConfig> parse_yaml(const std::string& yaml_path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load YAML \"" + yaml_path + "\": " + e.what());
    }

    std::vector<JointConfig> config;
    for (const auto& node : root["joints"]) {
        config.push_back({
            node["name"].as<std::string>(),
            node["type"].as<std::string>(),
            node["slave_id"].as<uint32_t>(),
            node["master_id"].as<uint32_t>(),
            node["mit_kp"].as<float>(),
            node["mit_kd"].as<float>(),
        });
    }
    return config;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  构造
// ═══════════════════════════════════════════════════════════════════════════════

ArmActuatorGroup::ArmActuatorGroup(
    std::shared_ptr<SerialPort> serial,
    std::vector<JointConfig>    config
)
    : impl_(std::make_unique<Impl>())
{
    impl_->serial = serial;

    // ── 注册品牌工厂（新增品牌在此添加一行）────────────────────────────────────
    impl_->factories.push_back(std::make_unique<DmActuatorFactory>(serial));
    // impl_->factories.push_back(std::make_unique<MiActuatorFactory>(serial));

    // ── 按配置顺序创建各关节执行器 ──────────────────────────────────────────────
    impl_->joints.reserve(config.size());
    impl_->names.reserve(config.size());

    for (std::size_t i = 0; i < config.size(); ++i) {
        const auto& cfg = config[i];

        ActuatorFactory* factory = nullptr;
        for (auto& f : impl_->factories) {
            if (f->accepts(cfg.type)) { factory = f.get(); break; }
        }
        if (!factory)
            throw std::runtime_error(
                "No factory registered for motor type: \"" + cfg.type + "\"");

        impl_->joints.push_back(factory->create(cfg));
        impl_->names.push_back(cfg.name);
        impl_->name_to_idx[cfg.name] = i;
    }
}

ArmActuatorGroup ArmActuatorGroup::from_yaml(
    std::shared_ptr<SerialPort> serial,
    const std::string&          yaml_path
)
{
    return ArmActuatorGroup(std::move(serial), parse_yaml(yaml_path));
}

ArmActuatorGroup ArmActuatorGroup::from_yaml(
    const std::string& device_path,
    const std::string& yaml_path,
    int                baud_timeout_ms
)
{
    // SerialPort::Init() 在打开失败时直接 exit(-1)，没有异常可捕获。
    // 此处提前用 open() 探测设备可访问性，确保错误以异常形式上报给调用者。
    int probe = ::open(device_path.c_str(), O_RDWR | O_NOCTTY);
    if (probe < 0) {
        throw std::runtime_error(
            "Cannot open serial port \"" + device_path + "\": " + strerror(errno) +
            "\n  请确认 u2can 已连接，或以 sudo 运行（需要 /dev/ttyACM* 读写权限）");
    }
    ::close(probe);

    auto serial = std::make_shared<SerialPort>(device_path, B921600, baud_timeout_ms);
    return ArmActuatorGroup(std::move(serial), parse_yaml(yaml_path));
}

ArmActuatorGroup::~ArmActuatorGroup() = default;

// ═══════════════════════════════════════════════════════════════════════════════
//  批量操作
// ═══════════════════════════════════════════════════════════════════════════════

void ArmActuatorGroup::enable_all()
{
    for (auto& j : impl_->joints) j->enable();
}

void ArmActuatorGroup::disable_all()
{
    for (auto& j : impl_->joints) j->disable();
}

std::vector<bool> ArmActuatorGroup::scan_connectivity(int timeout_ms)
{
    // 加大串口超时以等待电机 0xCC 状态回包（正常通信约 5 ms，此处留 100 ms 余量）
    impl_->serial->set_timeout(timeout_ms);

    std::vector<bool> result;
    result.reserve(impl_->joints.size());
    for (auto& j : impl_->joints) {
        j->refresh_status();
        // 达妙 SDK 的量化编解码保证：任何真实反馈值经 float↔uint 往返后
        // 不会恰好得到精确的 {0, 0, 0}，以此区分"无回包"和"零位静止"
        bool connected = (j->get_position() != 0.f ||
                          j->get_velocity() != 0.f ||
                          j->get_torque()   != 0.f);
        result.push_back(connected);
    }

    impl_->serial->set_timeout(5);  // 恢复正常控制循环超时
    return result;
}

void ArmActuatorGroup::set_all_positions(const std::vector<float>& positions)
{
    if (positions.size() != impl_->joints.size())
        throw std::invalid_argument("set_all_positions: size mismatch");
    for (std::size_t i = 0; i < impl_->joints.size(); ++i)
        impl_->joints[i]->set_position(positions[i]);
}

void ArmActuatorGroup::set_all_positions_velocities(
    const std::vector<float>& positions,
    const std::vector<float>& velocities)
{
    if (positions.size() != impl_->joints.size() ||
        velocities.size() != impl_->joints.size())
        throw std::invalid_argument("set_all_positions_velocities: size mismatch");
    for (std::size_t i = 0; i < impl_->joints.size(); ++i)
        impl_->joints[i]->set_position_velocity(positions[i], velocities[i]);
}

std::vector<float> ArmActuatorGroup::get_all_positions() const
{
    std::vector<float> out;
    out.reserve(impl_->joints.size());
    for (const auto& j : impl_->joints)
        out.push_back(j->get_position());
    return out;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  单关节访问
// ═══════════════════════════════════════════════════════════════════════════════

ActuatorBase& ArmActuatorGroup::operator[](std::string_view name)
{
    auto it = impl_->name_to_idx.find(std::string(name));
    if (it == impl_->name_to_idx.end())
        throw std::out_of_range("Unknown joint: " + std::string(name));
    return *impl_->joints[it->second];
}

const ActuatorBase& ArmActuatorGroup::operator[](std::string_view name) const
{
    auto it = impl_->name_to_idx.find(std::string(name));
    if (it == impl_->name_to_idx.end())
        throw std::out_of_range("Unknown joint: " + std::string(name));
    return *impl_->joints[it->second];
}

ActuatorBase& ArmActuatorGroup::operator[](std::size_t idx)
{
    if (idx >= impl_->joints.size())
        throw std::out_of_range("Joint index out of range");
    return *impl_->joints[idx];
}

const ActuatorBase& ArmActuatorGroup::operator[](std::size_t idx) const
{
    if (idx >= impl_->joints.size())
        throw std::out_of_range("Joint index out of range");
    return *impl_->joints[idx];
}

std::size_t ArmActuatorGroup::size() const      { return impl_->joints.size(); }
std::vector<std::string> ArmActuatorGroup::joint_names() const { return impl_->names; }

}  // namespace actuator
