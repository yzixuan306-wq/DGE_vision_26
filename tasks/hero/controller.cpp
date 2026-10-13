#include "controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hero
{
namespace
{
constexpr double pi = 3.14159265358979323846;
bool finite(double x) { return std::isfinite(x); }
bool positive(double x) { return finite(x) && x > 0; }
struct Candidate
{
  bool valid = false;
  std::array<double, 3> point{};
  double phase = 0;
  double miss = 0;
  double horizon = 0;
  tools::BallisticSolution ballistic;
};
Candidate solve_candidate(const TargetState & t, int id, bool spin, double delay, double speed,
                          const Config & c)
{
  const auto & x = t.x;
  const bool alternate = t.armor_count == 4 && id % 2 == 1;
  const double radius = x[8] + (alternate ? x[9] : 0);
  const double height_delta = alternate ? x[10] : 0;
  double flight = std::hypot(x[0], x[2]) / speed;
  Candidate result;
  for (int iteration = 0; iteration < 16; ++iteration) {
    const double horizon = delay + flight;
    const double cx = x[0] + x[1] * horizon, cy = x[2] + x[3] * horizon;
    const double az = x[4] + x[5] * horizon + height_delta;
    const double center_angle = std::atan2(cy, cx);
    const double angle = wrap_angle(x[6] + x[7] * horizon + 2 * pi * id / t.armor_count);
    const double ax = cx - radius * std::cos(angle), ay = cy - radius * std::sin(angle);
    // Fixed front intersection of the rotating ring; preserve each plate's radius and height.
    const double px = spin ? cx - radius * std::cos(center_angle) : ax;
    const double py = spin ? cy - radius * std::sin(center_angle) : ay;
    const double distance = std::hypot(px, py);
    if (!finite(distance) || distance > c.max_range || distance < 0.1) return {};
    auto ballistic = tools::solve_ballistic(speed, distance, az, c.ballistic);
    if (!ballistic.valid) return {};
    if (std::abs(ballistic.time - flight) < 0.0001) {
      result.valid = true;
      result.point = {px, py, az};
      result.phase = wrap_angle(angle - center_angle);
      result.miss = std::hypot(ax - px, ay - py);
      result.horizon = horizon;
      result.ballistic = ballistic;
      return result;
    }
    flight = ballistic.time;
  }
  return {};  // No command from an unconverged moving-target solution.
}
bool valid_target(const TargetState & t)
{
  for (double v : t.x) if (!finite(v)) return false;
  return t.identity >= 0 && (t.armor_count == 2 || t.armor_count == 3 || t.armor_count == 4) &&
         t.x[8] > 0.05 && t.x[8] < 0.5 &&
         (t.armor_count != 4 || (t.x[8] + t.x[9] > 0.05 && t.x[8] + t.x[9] < 0.5)) &&
         finite(t.phase_variance) && t.phase_variance >= 0 && finite(t.speed_variance) &&
         t.speed_variance >= 0 && finite(t.phase_speed_covariance) &&
         std::abs(t.phase_speed_covariance) <=
           std::sqrt(t.phase_variance) * std::sqrt(t.speed_variance) + 1e-12;
}
}  // namespace

double wrap_angle(double angle) { return std::remainder(angle, 2 * pi); }

Controller::Controller(Config c) : config_(c)
{
  if (!tools::valid_ballistic_config(c.ballistic) || !positive(c.min_speed) || !positive(c.max_speed) ||
      c.max_speed <= c.min_speed || c.max_speed > 100 || !finite(c.launch_delay) ||
      c.launch_delay < 0 || c.launch_delay > 2 || !positive(c.max_frame_age) ||
      c.max_frame_age > 1 || !positive(c.max_range) || !positive(c.spin_threshold) ||
      !positive(c.facing_tolerance) || c.facing_tolerance >= pi / 2 ||
      !positive(c.position_tolerance) || !positive(c.yaw_tolerance) ||
      !positive(c.pitch_tolerance) || !positive(c.alignment_tolerance) ||
      !positive(c.max_phase_std) || !positive(c.min_interval) ||
      !finite(c.yaw_offset) || !finite(c.pitch_offset) || c.settle_frames < 1)
    throw std::invalid_argument("Invalid hero configuration (check units, limits and finite values)");
}

void Controller::reset_tracking()
{
  identity_ = -1;
  settled_ = 0;
  // Keep the timestamp watermark: replaying a rejected frame must not become valid after reset.
}

Decision Controller::update(const Input & in)
{
  Decision result;
  auto reject = [&](const char * reason) {
    reset_tracking();
    result.reason = reason;
    return result;
  };
  if (!finite(in.now) || !finite(in.image_time)) return reject("invalid_time");
  if (last_now_ && in.now <= *last_now_) return reject("nonmonotonic_time");
  last_now_ = in.now;
  if (!in.enabled) return reject("disabled");
  if (!in.tracking || !in.target) return reject("not_tracking");
  if (in.now < in.image_time || in.now - in.image_time > config_.max_frame_age)
    return reject("stale_frame");
  if (last_image_time_ && in.image_time <= *last_image_time_) return reject("duplicate_frame");
  if (!finite(in.bullet_speed) || in.bullet_speed < config_.min_speed ||
      in.bullet_speed > config_.max_speed) return reject("invalid_speed");
  if (!finite(in.gimbal_yaw) || !finite(in.gimbal_pitch)) return reject("invalid_gimbal");
  const auto & target = *in.target;
  if (!valid_target(target)) return reject("invalid_target");
  if (identity_ != target.identity ||
      (last_image_time_ && in.image_time - *last_image_time_ > config_.max_frame_age)) {
    reset_tracking();
    identity_ = target.identity;
  }
  last_image_time_ = in.image_time;
  settled_ = std::min(settled_ + 1, config_.settle_frames);
  result.spin = std::abs(target.x[7]) >= config_.spin_threshold;
  const double delay = in.now - in.image_time + config_.launch_delay;
  Candidate best;
  double best_score = std::numeric_limits<double>::infinity();
  // Until another plate has been observed, only the original plate's geometry is known.
  const int count = target.rotation_observed ? target.armor_count : 1;
  for (int id = 0; id < count; ++id) {
    auto candidate = solve_candidate(target, id, result.spin, delay, in.bullet_speed, config_);
    if (!candidate.valid) continue;
    const double score = std::abs(candidate.phase);
    if (score < best_score) {
      best = candidate;
      best_score = score;
      result.armor_id = id;
    }
  }
  if (!best.valid) return reject("no_solution");
  result.aim_point = best.point;
  result.flight_time = best.ballistic.time;
  result.impact_phase = best.phase;
  const double variance = target.phase_variance +
    2 * best.horizon * target.phase_speed_covariance +
    best.horizon * best.horizon * target.speed_variance;
  if (!finite(variance) || variance < -1e-10) return reject("invalid_covariance");
  result.phase_std = std::sqrt(std::max(0.0, variance));
  const double yaw = wrap_angle(std::atan2(best.point[1], best.point[0]) + config_.yaw_offset);
  const double pitch = -(best.ballistic.pitch + config_.pitch_offset);
  if (!finite(yaw) || !finite(pitch) || !finite(result.phase_std) ||
      -pitch < config_.ballistic.min_pitch || -pitch > config_.ballistic.max_pitch)
    return reject("invalid_solution");
  result.command = {true, false, yaw, pitch};
  const double horizontal_range = std::hypot(best.point[0], best.point[1]);
  const double yaw_tolerance = std::min(config_.yaw_tolerance,
    std::atan2(config_.alignment_tolerance, horizontal_range));
  const double pitch_tolerance = std::min(config_.pitch_tolerance,
    std::atan2(config_.alignment_tolerance, std::hypot(horizontal_range, best.point[2])));
  result.window_open = std::abs(best.phase) <= config_.facing_tolerance &&
    (!result.spin || (target.rotation_observed && best.miss <= config_.position_tolerance &&
                     result.phase_std <= config_.max_phase_std));
  if (!result.window_open) result.reason = "outside_window";
  else if (settled_ < config_.settle_frames) result.reason = "settling";
  else if (std::abs(wrap_angle(in.gimbal_yaw - yaw)) > yaw_tolerance ||
           std::abs(in.gimbal_pitch - pitch) > pitch_tolerance) result.reason = "unaligned";
  else if (!config_.auto_fire) result.reason = "auto_fire_off";
  else if (last_shot_ && in.now - *last_shot_ < config_.min_interval) result.reason = "cooldown";
  else {
    result.command.shoot = true;  // One update pulse; never schedule detached future writes.
    last_shot_ = in.now;
    result.reason = "fire_pulse";
  }
  return result;
}
}  // namespace hero
