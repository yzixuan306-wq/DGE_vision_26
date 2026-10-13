/* C板通信模块（保持 DGE 当前固件协议，小端整数）
 * IMU：10 字节，AA + int16 w/x/y/z（x10000）+ DD。
 * 状态：9 字节，BB + int16 speed（x100）/mode/ShootMode/ft_angle（x10000）。
 * 控制：8 字节，CC + uint8 control/shoot + int16 yaw/pitch（rad x10000）+ EE。
 */
#include "cboard.hpp"

#include <algorithm>

#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{

//初始化C板通信
CBoard::CBoard(const std::string & config_path)
: bullet_speed(0),
  mode(Mode::idle),
  // shoot_mode(ShootMode::left_shoot),
  // bullet_speed(0),
  ft_angle(0),
  // queue_ initialization removed (using default deque constructor)
  serial_(read_yaml(config_path), read_baudrate(config_path),
          std::bind(&CBoard::callback, this, std::placeholders::_1))
{
  tools::logger()->info("[Cboard] Waiting for q...");
  auto now = std::chrono::steady_clock::now();
  
  // Manually push initial data with locking
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back({Eigen::Quaterniond::Identity(), now - std::chrono::milliseconds(10)});
    queue_.push_back({Eigen::Quaterniond::Identity(), now});
    
    // Pop the initial data into data_ahead_ and data_behind_
    if (!queue_.empty()) {
        data_ahead_ = queue_.front();
        queue_.pop_front();
    }
    if (!queue_.empty()) {
        data_behind_ = queue_.front();
        queue_.pop_front();
    }
  }

  tools::logger()->info("[Cboard] Opened.");
}

// 根据给定的时间戳，通过插值计算出该时刻对应的 IMU 四元
Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    IMUData temp;
    bool has_data = false;
    
    // Thread-safe pop
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (!queue_.empty()) {
            temp = queue_.front();
            queue_.pop_front();
            has_data = true;
        }
    }
    
    if (!has_data) break; // Queue was empty
    
    data_behind_ = temp;
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  
  double dt = std::chrono::duration<double>(t_b - t_a).count();
  if (dt <= 1e-6) return q_b; 

  std::chrono::duration<double> t_ac = t_c - t_a;
  double k = std::clamp(t_ac.count() / dt, 0.0, 1.0);
  
  return q_a.slerp(k, q_b).normalized();
}

// 发送控制指令到C板
CBoardState CBoard::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

void CBoard::send(Command command, bool replace_pending) const
{
  ControlDataFrame frame;
  frame.header = 0xCC;
  
  frame.control = (command.control) ? 1 : 0;
  frame.shoot = (command.shoot) ? 1 : 0;
  frame.yaw = (int16_t)(command.yaw * 10000);
  frame.pitch = (int16_t)(command.pitch * 10000);
  frame.tail = 0xEE;
  // frame.yaw_vel = (int16_t)(command.yaw_vel * 1000);
  // frame.pitch_vel = (int16_t)(command.pitch_vel * 1000);
  // frame.horizon_distance = (int16_t)(command.horizon_distance * 10000);

  std::vector<uint8_t> data(sizeof(frame));
  std::memcpy(data.data(), &frame, sizeof(frame));

  try 
  {
    serial_.write(data, replace_pending);
  } catch (const std::exception & e) {
    tools::logger()->warn("Serial write failed: {}", e.what());
  }
}

// 解析并存储IMU和状态数据
void CBoard::callback(const std::vector<uint8_t> & frame)
{
  auto timestamp = std::chrono::steady_clock::now();
  
  if (frame.empty()) return;
  uint8_t header = frame[0];

  if (header == 0xAA) {
    if (frame.size() != sizeof(IMUDataFrame)) {
      tools::logger()->warn("IMU frame size mismatch: expected {}, got {}", sizeof(IMUDataFrame), frame.size());
      return;
    }
    
    const auto* data = reinterpret_cast<const IMUDataFrame*>(frame.data());
    
    if (data->tail != 0xDD) { 
      tools::logger()->warn("Invalid IMU frame tail: expected 0xDD, got 0x{:02X}", data->tail);
      return; // 帧尾不对，说明发生了数据错位，直接丢弃该帧
    }

    double w = data->w / 10000.0;
    double x = data->x / 10000.0;
    double y = data->y / 10000.0;
    double z = data->z / 10000.0;
    // double w = data->w / 10000.0;

    double norm_sq = x * x + y * y + z * z + w * w;
    if (std::abs(norm_sq - 1.0) > 0.1) {
      tools::logger()->warn("Invalid quaternion norm: {:.4f}", norm_sq);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.q = Eigen::Quaterniond(w, x, y, z).normalized();
      state_.imu_time = timestamp;
    }

    // Thread-safe push with capacity check
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.size() >= queue_capacity_) {
            queue_.pop_front();
        }
        // queue_.push_back({{x, y, z, w}, timestamp});
        queue_.push_back({{w, x, y, z}, timestamp});
    }
  } 
  else if (header == 0xBB) {
    if (frame.size() != sizeof(StatusDataFrame)) {
      tools::logger()->warn("Status frame size mismatch");
      return;
    }

    const auto* data = reinterpret_cast<const StatusDataFrame*>(frame.data());

    bullet_speed = data->bullet_speed / 100.0;
    int m = data->mode;
    mode = (m >= 0 && m < static_cast<int>(MODES.size())) ? Mode(m) : Mode::idle;
    
    // int sm = data->shoot_mode;
    // if (sm >= 0 && sm < static_cast<int>(SHOOT_MODES.size())) shoot_mode = ShootMode(sm);

    ft_angle = data->ft_angle / 10000.0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      state_.bullet_speed = bullet_speed;
      state_.mode = mode;
      state_.ft_angle = ft_angle;
      state_.status_time = timestamp;
    }

    static auto last_log_time = std::chrono::steady_clock::time_point{};
    if (tools::delta_time(timestamp, last_log_time) >= 1.0) {
      tools::logger()->info("[CBoard] Speed: {:.2f}, Mode: {}", bullet_speed, MODES[mode]);
      last_log_time = timestamp;
    }
  } else {
    tools::logger()->debug("Unknown frame header: 0x{:02X}", header);
  }
}

std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  return yaml["serial_device"].as<std::string>();
}

int CBoard::read_baudrate(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  if (!yaml["serial_baudrate"]) {
    return 115200;
  }
  return yaml["serial_baudrate"].as<int>();
}

}  // namespace io
