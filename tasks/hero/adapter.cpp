#include "adapter.hpp"

#include <yaml-cpp/yaml.h>

#include <limits>
#include <stdexcept>

namespace hero
{
Config load_config(const std::string & path)
{
  const auto root = YAML::LoadFile(path);
  const auto yaml = root["hero"];
  if (!yaml || !yaml.IsMap()) throw std::invalid_argument("Missing hero configuration block");
  Config c;
  // Required keys deliberately catch incomplete profiles instead of inheriting infantry values.
  c.auto_fire = yaml["auto_fire"].as<bool>();
  c.min_speed = yaml["min_speed"].as<double>();
  c.max_speed = yaml["max_speed"].as<double>();
  c.launch_delay = yaml["launch_delay"].as<double>();
  c.max_frame_age = yaml["max_frame_age"].as<double>();
  c.max_range = yaml["max_range"].as<double>();
  c.spin_threshold = yaml["spin_threshold"].as<double>();
  c.facing_tolerance = yaml["facing_tolerance"].as<double>();
  c.position_tolerance = yaml["position_tolerance"].as<double>();
  c.yaw_tolerance = yaml["yaw_tolerance"].as<double>();
  c.pitch_tolerance = yaml["pitch_tolerance"].as<double>();
  c.alignment_tolerance = yaml["alignment_tolerance"].as<double>();
  c.max_phase_std = yaml["max_phase_std"].as<double>();
  c.min_interval = yaml["min_interval"].as<double>();
  c.yaw_offset = yaml["yaw_offset"].as<double>();
  c.pitch_offset = yaml["pitch_offset"].as<double>();
  c.settle_frames = yaml["settle_frames"].as<int>();
  const auto ballistic = yaml["ballistic"];
  c.ballistic.gravity = ballistic["gravity"].as<double>();
  c.ballistic.drag = ballistic["drag"].as<double>();
  c.ballistic.min_pitch = ballistic["min_pitch"].as<double>();
  c.ballistic.max_pitch = ballistic["max_pitch"].as<double>();
  c.ballistic.max_time = ballistic["max_time"].as<double>();
  c.ballistic.step = ballistic["step"].as<double>();
  Controller validate(c);
  return c;
}

double seconds(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration<double>(time.time_since_epoch()).count();
}

Adapter::Adapter(const std::string & path) : controller_(load_config(path)) {}

void Adapter::reset_tracking()
{
  controller_.reset_tracking();
  session_active_ = false;
  was_detecting_ = false;
  // Keep the session counter and frame watermark across target loss and mode changes.
}

Decision Adapter::update(
  const std::list<auto_aim::Target> & targets, const std::string & tracker_state,
  std::chrono::steady_clock::time_point image_time,
  std::chrono::steady_clock::time_point now, double speed, double yaw, double pitch, bool enabled)
{
  Input input;
  input.image_time = seconds(image_time);
  input.now = seconds(now);
  input.bullet_speed = speed;
  input.gimbal_yaw = yaw;
  input.gimbal_pitch = pitch;
  input.enabled = enabled;
  input.tracking = tracker_state == "tracking";
  // Outpost adaptation is outside this feature group; do not reinterpret it as a vehicle.
  if (!targets.empty() && targets.front().name != auto_aim::ArmorName::outpost) {
    const auto & target = targets.front();
    const auto x = target.ekf_x();
    const auto & p = target.ekf().P;
    if (x.size() == 11 && p.rows() == 11 && p.cols() == 11 && x.allFinite() && p.allFinite()) {
      TargetState state;
      for (int i = 0; i < 11; ++i) state.x[i] = x[i];
      state.armor_count = static_cast<int>(target.armor_xyza_list().size());
      state.rotation_observed = target.jumped;
      state.phase_variance = p(6, 6);
      state.phase_speed_covariance = (p(6, 7) + p(7, 6)) / 2;
      state.speed_variance = p(7, 7);
      input.target = state;
    }
  }
  // Keep hero-specific session bookkeeping outside the native Tracker.
  const bool eligible = enabled && input.target &&
    (tracker_state == "detecting" || tracker_state == "tracking" || tracker_state == "temp_lost");
  if (!eligible) session_active_ = false;
  if (!last_image_time_ || image_time > *last_image_time_) {
    // Native Tracker starts a new detection after a gap longer than 100 ms,
    // including a gap while it is already in the detecting state.
    const bool gap = last_image_time_ &&
      image_time - *last_image_time_ > std::chrono::milliseconds(100);
    const bool detecting = tracker_state == "detecting";
    if (eligible && (!session_active_ || (detecting && (!was_detecting_ || gap)))) {
      session_id_ = session_id_ == std::numeric_limits<int>::max() ? 0 : session_id_ + 1;
      session_active_ = true;
    }
    was_detecting_ = detecting;
    last_image_time_ = image_time;
  }
  if (input.target) input.target->identity = session_id_;
  return controller_.update(input);
}
}  // namespace hero
