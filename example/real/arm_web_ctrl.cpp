/**
 * @brief 机械臂多关节 Web 拖动控制
 *
 * 启动时扫描各关节是否响应，网页显示连接/使能状态，支持单关节或全部使能/失能。
 * 使用: sudo ./arm_web_ctrl [串口设备] [YAML配置] [HTTP端口]
 *       默认: /dev/ttyACM0  config/arm.yaml  8080
 */
#include "actuator/arm_actuator_group.h"
#include "utils/control_loop.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <cstdio>
#include <csignal>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>  // timeval / SO_RCVTIMEO
#include <limits.h>    // PATH_MAX

using namespace actuator;

static constexpr float  Q_LIMIT_RAD = 3.14159f;
// 控制指令发送频率目标值 [Hz]。
// 实际频率受 USB-CAN 串行往返延迟限制（7 关节约 150-200 Hz），
// 此处设为 1000 使控制线程以最大吞吐运行（sleep_until 不会产生额外等待）。
static constexpr double CTRL_HZ     = 1000.0;

// ─── 关节共享状态（HTTP线程写请求，控制线程读/执行）─────────────────────────
// 状态码：0=待命(已连接)  1=运行中  2=无连接
struct JointState {
    std::atomic<float> target  {0.f};   // 目标位置 (rad)
    std::atomic<float> feedback{0.f};   // 反馈位置 (rad)
    std::atomic<int>   status  {0};     // 0/1/2
    std::atomic<int>   req     {-1};    // 使能请求：-1=无 0=失能 1=使能
    std::atomic<int>   zero_req{0};     // 设零请求：0=无 1=设零
};

static volatile bool                      g_running = true;
static std::unique_ptr<JointState[]>      g_joints;        // 由 main() 在构造 arm 后分配
static size_t                             g_num_joints = 0;
static std::atomic<int>                   g_all_req{-1};   // 全部使能/失能：-1=无 0=失能 1=使能
static std::atomic<float>                 g_ctrl_hz{0.f};  // 实际控制频率（由控制线程测量，HTTP 线程读取）

static void on_signal(int) { g_running = false; }

// ─── 嵌入式 HTML/JS ───────────────────────────────────────────────────────────
// JS 启动时通过 GET /config 获取关节名，动态生成控制行，无需硬编码关节数。
static const char HTML[] = R"html(<!DOCTYPE html>
<html><head>
<meta charset="utf-8"><title>机械臂控制</title>
<style>
  body{font-family:monospace;max-width:780px;margin:36px auto;background:#1a1a2e;color:#ddd}
  h1{text-align:center;color:#4fc3f7;margin-bottom:16px}
  table{width:100%;border-collapse:collapse}
  th{font-size:11px;color:#777;padding:4px 6px;text-align:left;border-bottom:1px solid #333}
  td{padding:5px 6px;vertical-align:middle}
  .name{width:64px;font-size:13px;color:#aaa}
  .badge{width:76px;font-size:12px;text-align:center;padding:3px 8px;border-radius:3px;font-weight:bold}
  .s0{background:#1c3320;color:#81c784}.s1{background:#0d2a5c;color:#90caf9}.s2{background:#3b1212;color:#ef9a9a}
  .slider td input[type=range]{width:100%;accent-color:#4fc3f7}
  .val{width:60px;text-align:right;font-size:12px}
  .cmd{color:#81c784}.fb{color:#ff8a65}
  button{font-size:11px;padding:3px 8px;border:none;border-radius:3px;cursor:pointer;margin:1px}
  .en {background:#1b5e20;color:#c8e6c9}.dis{background:#4a0e0e;color:#ffcdd2}
  .zer{background:#4a3000;color:#ffcc80}
  .ab {padding:8px 28px;margin:4px;border:none;border-radius:4px;cursor:pointer;font-size:13px}
  #enAll{background:#2e7d32;color:#fff}#disAll{background:#b71c1c;color:#fff}
  .btns{text-align:center;margin:18px 0}
  tr.dead td:not(.name) td:not(.badge){opacity:.35;pointer-events:none}
</style>
</head><body>
<h1>机械臂关节控制</h1>
<div style="text-align:center;margin-bottom:10px;font-size:13px;color:#aaa">
  控制频率: <span id="hz" style="color:#4fc3f7;font-weight:bold">—</span> Hz
</div>
<table>
  <tr><th>关节</th><th>状态</th><th>← 角度 →</th><th>指令</th><th>反馈</th><th>操作</th></tr>
  <tbody id="tb"></tbody>
</table>
<div class="btns">
  <button class="ab" id="enAll"  onclick="reqAll(1)">全部使能</button>
  <button class="ab" id="disAll" onclick="reqAll(0)">全部失能</button>
</div>
<script>
let names = [];

// 动态生成关节行
fetch('/config').then(r=>r.json()).then(d=>{
  names = d.names;
  const tb = document.getElementById('tb');
  names.forEach((nm, i) => {
    tb.innerHTML += `<tr id="r${i}">
      <td class="name">${nm}</td>
      <td><span class="badge s0" id="st${i}">待命</span></td>
      <td class="slider"><input type="range" id="s${i}"
          min="-180" max="180" step="0.5" value="0"
          oninput="sendCmd(${i}, +this.value)"></td>
      <td class="val cmd" id="c${i}">0.0°</td>
      <td class="val fb"  id="f${i}">—°</td>
      <td>
        <button class="en"  onclick="reqOne(${i},1)">使能</button>
        <button class="dis" onclick="reqOne(${i},0)">失能</button>
        <button class="zer" onclick="reqZero(${i})">设零</button>
      </td>
    </tr>`;
  });
});

// 节流：每个关节独立维护一个 pending 值，用 rAF 合并高频 oninput 事件，
// 每帧最多向服务器发送一次指令，避免 TCP 请求堆积造成延迟。
const pendingCmd = {};
function sendCmd(i, deg) {
  document.getElementById('c'+i).textContent = deg.toFixed(1) + '°';
  pendingCmd[i] = deg;
  if (pendingCmd[i+'_raf']) return;
  pendingCmd[i+'_raf'] = requestAnimationFrame(() => {
    pendingCmd[i+'_raf'] = null;
    fetch('/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({joint: i, pos: pendingCmd[i] * Math.PI / 180})
    });
  });
}

function reqOne(i, en) {
  fetch('/enable', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({joint: i, enable: en})});
}

function reqAll(en) {
  fetch('/enable', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({joint: -1, enable: en})});
}

function reqZero(i) {
  if (!confirm('确认将 ' + names[i] + ' 当前位置设为零点？\n此操作写入电机固件，不可撤销！')) return;
  fetch('/zero', {method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({joint: i})
  }).then(() => {
    document.getElementById('s'+i).value = 0;
    document.getElementById('c'+i).textContent = '0.0°';
  });
}

// 轮询状态（100ms）
const sCls = ['s0','s1','s2'], sTxt = ['待命','运行','无连接'];
let prevSt = [];
setInterval(() => {
  fetch('/state').then(r=>r.json()).then(d => {
    if (d.hz !== undefined)
      document.getElementById('hz').textContent = d.hz.toFixed(1);
    d.st.forEach((s, i) => {
      const badge = document.getElementById('st'+i);
      badge.className = 'badge ' + sCls[s];
      badge.textContent = sTxt[s];
      document.getElementById('r'+i).className = (s===2) ? 'dead' : '';
      // 刚使能时将滑块同步到反馈位置，避免突跳
      if (prevSt[i] !== 1 && s === 1) {
        const deg = d.pos[i] * 180 / Math.PI;
        document.getElementById('s'+i).value = deg;
        document.getElementById('c'+i).textContent = deg.toFixed(1) + '°';
      }
      prevSt[i] = s;
      document.getElementById('f'+i).textContent = (d.pos[i]*180/Math.PI).toFixed(1) + '°';
    });
  }).catch(() => {});
}, 100);
</script>
</body></html>)html";

// ─── 最小 HTTP 工具 ───────────────────────────────────────────────────────────

static void http_respond(int fd, const char* content_type, const std::string& body)
{
    std::string header =
        "HTTP/1.1 200 OK\r\nContent-Type: " + std::string(content_type) +
        "\r\nContent-Length: " + std::to_string(body.size()) +
        "\r\nAccess-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n";
    send(fd, header.c_str(), header.size(), 0);
    send(fd, body.c_str(),   body.size(),   0);
}

// 从 JSON body 里取 key 对应的数字（仅支持整数/浮点）
static float json_get(const std::string& body, const char* key)
{
    std::string pat = std::string("\"") + key + "\":";
    auto pos = body.find(pat);
    if (pos == std::string::npos) return 0.f;
    try { return std::stof(body.substr(pos + pat.size())); }
    catch (...) { return 0.f; }
}

// ─── HTTP 连接处理（每个连接在独立线程中运行）───────────────────────────────

static void handle_client(int cli, const std::string& config_json)
{
    // 设置接收超时，避免慢速客户端阻塞线程
    struct timeval tv{};
    tv.tv_usec = 200000; // 200 ms
    setsockopt(cli, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    const size_t N = g_num_joints;
    char buf[4096]{};
    int  n = recv(cli, buf, sizeof(buf) - 1, 0);
    if (n > 0) {
        std::string req(buf, n);
        std::string body;
        auto sep = req.find("\r\n\r\n");
        if (sep != std::string::npos) body = req.substr(sep + 4);

        if (req.find("GET /config") != std::string::npos) {
            http_respond(cli, "application/json", config_json);

        } else if (req.find("GET /state") != std::string::npos) {
            std::string j = "{\"pos\":[";
            for (size_t i = 0; i < N; ++i) {
                if (i) j += ',';
                char tmp[24];
                snprintf(tmp, sizeof(tmp), "%.4f", g_joints[i].feedback.load());
                j += tmp;
            }
            j += "],\"st\":[";
            for (size_t i = 0; i < N; ++i) {
                if (i) j += ',';
                j += std::to_string(g_joints[i].status.load());
            }
            char hz_buf[32];
            snprintf(hz_buf, sizeof(hz_buf), "%.1f", g_ctrl_hz.load());
            http_respond(cli, "application/json", j + "],\"hz\":" + hz_buf + "}");

        } else if (req.find("POST /cmd") != std::string::npos) {
            int   ji  = static_cast<int>(json_get(body, "joint"));
            float pos = json_get(body, "pos");
            if (ji >= 0 && ji < static_cast<int>(N))
                g_joints[ji].target = std::clamp(pos, -Q_LIMIT_RAD, Q_LIMIT_RAD);
            http_respond(cli, "application/json", "{}");

        } else if (req.find("POST /enable") != std::string::npos) {
            int ji = static_cast<int>(json_get(body, "joint"));
            int en = static_cast<int>(json_get(body, "enable"));
            if (ji < 0)
                g_all_req = en;
            else if (ji < static_cast<int>(N))
                g_joints[ji].req = en;
            http_respond(cli, "application/json", "{}");

        } else if (req.find("POST /zero") != std::string::npos) {
            int ji = static_cast<int>(json_get(body, "joint"));
            if (ji >= 0 && ji < static_cast<int>(N) && g_joints[ji].status != 2)
                g_joints[ji].zero_req = 1;
            http_respond(cli, "application/json", "{}");

        } else {
            http_respond(cli, "text/html; charset=utf-8", HTML);
        }
    }
    close(cli);
}

// ─── HTTP 服务线程 ────────────────────────────────────────────────────────────

static void http_server(int port, const std::vector<std::string>& joint_names)
{
    // 预先构造 /config 响应（不变，只算一次）
    std::string config_json = "{\"names\":[";
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (i) config_json += ',';
        config_json += '"' + joint_names[i] + '"';
    }
    config_json += "]}";

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0 || listen(srv, 32) < 0) {
        perror("[web] bind/listen");
        return;
    }

    while (g_running) {
        int cli = accept(srv, nullptr, nullptr);
        if (cli < 0) continue;
        // 每个连接用独立 detached 线程处理，accept 立即返回继续监听，
        // 避免串行阻塞导致 /cmd 请求在 TCP 队列中积压。
        std::thread(handle_client, cli, config_json).detach();
    }
    close(srv);
}

// ─── 主程序 ───────────────────────────────────────────────────────────────────

// 获取可执行文件所在目录的父目录（即项目根目录），用于解析默认配置路径。
// 这样无论从哪个工作目录运行，config/arm.yaml 始终能找到。
static std::string exe_dir()
{
    char buf[PATH_MAX]{};
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    std::string path(buf, n);
    auto slash = path.rfind('/');
    return (slash == std::string::npos) ? "." : path.substr(0, slash);
}

int main(int argc, char* argv[])
{
    const std::string project_root = exe_dir() + "/..";

    std::string dev       = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    std::string yaml_path = (argc > 2) ? argv[2] : project_root + "/config/arm.yaml";
    int         port      = (argc > 3) ? std::stoi(argv[3]) : 8080;

    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // ── 构造执行器组 ──────────────────────────────────────────────────────────
    ArmActuatorGroup arm = [&]() -> ArmActuatorGroup {
        try {
            return ArmActuatorGroup::from_yaml(dev, yaml_path);
        } catch (const std::exception& e) {
            fprintf(stderr, "[错误] 初始化失败: %s\n", e.what());
            std::exit(1);
        }
    }();
    const size_t N = arm.size();

    // ── 初始化共享状态 ────────────────────────────────────────────────────────
    g_num_joints = N;
    g_joints = std::make_unique<JointState[]>(N);  // atomic 成员有默认值，直接零初始化

    // ── 扫描连接 ─────────────────────────────────────────────────────────────
    // 扫描结果仅作提示，不锁死关节。
    // 原因：电机恰好停在零位时三个反馈值均为 0，与超时无回包无法区分；
    //       CAN 总线偶发丢包也可能导致误判。
    // 策略：无论扫描结果如何，状态均初始化为"待命(0)"，让用户自主决定是否使能；
    //       扫描未响应的关节打印警告，提醒用户注意但不禁止操作。
    puts("[arm] 扫描各关节...");
    auto connected   = arm.scan_connectivity();
    auto joint_names = arm.joint_names();
    for (size_t i = 0; i < N; ++i) {
        g_joints[i].status = 0;  // 统一初始化为待命，不因扫描失败锁死
        if (connected[i]) {
            printf("  %s: 已识别\n", joint_names[i].c_str());
        } else {
            printf("  %s: 未响应（可能在零位或总线偶发丢包，仍可尝试使能）\n",
                   joint_names[i].c_str());
        }
    }

    // ── 启动 HTTP 服务 ────────────────────────────────────────────────────────
    std::thread(http_server, port, joint_names).detach();
    printf("[arm] 打开 http://localhost:%d\n", port);

    // ── 控制循环（目标 CTRL_HZ，实际受 CAN 吞吐限制）────────────────────────
    // ControlLoop 在独立线程中以固定频率运行 callback，回调返回 false 时自动停止。
    // enable/disable/set_zero 均在 callback 内部串行处理，无需额外互斥。
    // Hz 测量：每完成一次完整迭代计数，每 1 s 刷新一次 g_ctrl_hz。
    int  hz_counter = 0;
    auto hz_last_tp = std::chrono::steady_clock::now();

    ControlLoop ctrl_loop(CTRL_HZ, [&]() -> bool {
        if (!g_running) return false;

        // 全部使能/失能（优先处理）
        int all = g_all_req.exchange(-1);
        if (all >= 0) {
            for (size_t i = 0; i < N; ++i) {
                if (g_joints[i].status == 2) continue;
                if (all == 1) {
                    g_joints[i].target = arm[i].get_position();
                    arm[i].enable();
                    g_joints[i].status = 1;
                } else {
                    arm[i].disable();
                    g_joints[i].status = 0;
                }
            }
        }

        for (size_t i = 0; i < N; ++i) {
            if (g_joints[i].status == 2) continue;

            // 单关节使能/失能
            int req = g_joints[i].req.exchange(-1);
            if (req == 1) {
                g_joints[i].target = arm[i].get_position();
                arm[i].enable();
                g_joints[i].status = 1;
            } else if (req == 0) {
                arm[i].disable();
                g_joints[i].status = 0;
            }

            // 设零（内部已加 ~100ms 延时，占用当前迭代但不阻塞其他线程）
            if (g_joints[i].zero_req.exchange(0) == 1) {
                arm[i].set_zero_position();
                g_joints[i].target = 0.f;
                printf("[arm] %s 零点已设置\n", joint_names[i].c_str());
            }

            // 发控制指令或刷新状态
            if (g_joints[i].status == 1)
                arm[i].set_position(g_joints[i].target.load());
            else
                arm[i].refresh_status();

            g_joints[i].feedback = arm[i].get_position();
        }

        // 测量实际控制频率（每 1 s 更新一次 g_ctrl_hz）
        ++hz_counter;
        auto now_tp = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now_tp - hz_last_tp).count();
        if (elapsed >= 1.0f) {
            g_ctrl_hz.store(hz_counter / elapsed, std::memory_order_relaxed);
            hz_counter = 0;
            hz_last_tp = now_tp;
        }

        return true;
    });

    // 主线程等待 SIGINT/SIGTERM
    while (g_running) usleep(50000);

    // ── 退出前失能所有运行中的关节 ───────────────────────────────────────────
    ctrl_loop.stop();
    for (size_t i = 0; i < N; ++i)
        if (g_joints[i].status == 1) arm[i].disable();

    puts("[arm] 退出");
    return 0;
}
