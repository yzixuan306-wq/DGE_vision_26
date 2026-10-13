#ifndef HERO__CONTROLLER_HPP
#define HERO__CONTROLLER_HPP

#include <array>
#include <optional>

#include "io/command.hpp"
#include "tools/ballistics.hpp"

namespace hero
{
struct Config
{
  tools::BallisticConfig ballistic;
  bool auto_fire = false;
  double min_speed = 10;
  double max_speed = 20;
  double launch_delay = 0.15;
  double max_frame_age = 0.1;
  double max_range = 10;
  double spin_threshold = 1.5;
  double facing_tolerance = 0.20;
  double position_tolerance = 0.03;
  double yaw_tolerance = 0.01;
  double pitch_tolerance = 0.01;
  double alignment_tolerance = 0.03;  // m; angular caps above tighten with distance.
  double max_phase_std = 0.15;
  double min_interval = 0.7;
  double yaw_offset = 0;
  double pitch_offset = 0;
  int settle_frames = 5;
};

// Snapshot at image_time, in SP25's IMU/world frame. No device or ML dependencies.
struct TargetState
{
  // x vx y vy z vz angle omega radius radius_delta height_delta
  std::array<double, 11> x{};
  int armor_count = 4;
  int identity = -1;  // Tracking session identifier, not just armor class.
  bool rotation_observed = false;
  double phase_variance = 0;
  double phase_speed_covariance = 0;
  double speed_variance = 0;
};

struct Input
{
  std::optional<TargetState> target;
  double image_time = 0;  // All times use the same steady_clock epoch, seconds.
  double now = 0;
  double bullet_speed = 0;
  double gimbal_yaw = 0;
  double gimbal_pitch = 0;  // SP25: negative upwards.
  bool enabled = false;
  bool tracking = false;  // False for detecting, temp_lost, switching, lost.
};

struct Decision
{
  io::Command command{false, false, 0, 0};
  std::array<double, 3> aim_point{};
  double flight_time = 0;
  double impact_phase = 0;
  double phase_std = 0;
  bool spin = false;
  bool window_open = false;
  int armor_id = -1;
  const char * reason = "disabled";
};

class Controller
{
public:
  explicit Controller(Config config);
  Decision update(const Input & input);
  // Preserve the shot cooldown across target loss and mode changes.
  void reset_tracking();
private:
  Config config_;
  int identity_ = -1;
  int settled_ = 0;
  std::optional<double> last_image_time_;
  std::optional<double> last_now_;
  std::optional<double> last_shot_;
};
double wrap_angle(double angle);
}  // namespace hero
#endif
