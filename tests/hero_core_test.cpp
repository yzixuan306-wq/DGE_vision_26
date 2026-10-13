#include "tasks/hero/controller.hpp"
#include "tools/ballistics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
int checks = 0;
void require(bool value, const std::string & message)
{
  ++checks;
  if (!value) throw std::runtime_error(message);
}
hero::Input sample(double time = 10)
{
  hero::Input in;
  in.enabled = true;
  in.tracking = true;
  in.image_time = in.now = time;
  in.bullet_speed = 15;
  hero::TargetState target;
  target.identity = 1;
  target.x = {5, 0, 0, 0, 0, 0, 0, 0, 0.2765, 0, 0};
  in.target = target;
  auto b = tools::solve_ballistic(15, 5 - 0.2765, 0, {});
  in.gimbal_pitch = -b.pitch;
  return in;
}
hero::Config firing_config()
{
  hero::Config config;
  config.auto_fire = true;
  config.settle_frames = 1;
  return config;
}
void advance(hero::Input & in, double dt)
{
  in.now += dt;
  in.image_time += dt;
}
void ballistic_tests()
{
  tools::BallisticConfig config;
  for (double d : {1.0, 3.0, 5.0, 8.0}) {
    auto b = tools::solve_ballistic(15, d, 0, config);
    double exact = std::asin(config.gravity * d / (15 * 15)) / 2;
    require(b.valid && std::abs(b.pitch - exact) < 1e-5, "vacuum analytical pitch");
    require(std::abs(b.time - d / (15 * std::cos(exact))) < 1e-5, "vacuum analytical time");
  }
  auto vacuum = tools::solve_ballistic(15, 5, 0, config);
  config.drag = 0.02;
  auto drag = tools::solve_ballistic(15, 5, 0, config);
  require(drag.valid && drag.pitch > vacuum.pitch && drag.time > vacuum.time, "drag increases compensation");
  config.step = 0.001;
  auto refined = tools::solve_ballistic(15, 5, 0, config);
  require(std::abs(drag.pitch - refined.pitch) < 2e-5 && std::abs(drag.time - refined.time) < 2e-5,
          "RK4 step convergence");
  require(tools::solve_ballistic(15, 5, -0.5, config).valid, "lower target");
  require(tools::solve_ballistic(15, 5, 0.5, config).valid, "higher target");
  require(!tools::solve_ballistic(0, 5, 0, config).valid, "zero speed");
  require(!tools::solve_ballistic(15, 0, 0, config).valid, "zero range");
  require(!tools::solve_ballistic(15, 100, 0, config).valid, "unreachable range");
  require(!tools::solve_ballistic(15, 5, 100, config).valid, "unreachable height");
  require(!tools::solve_ballistic(std::nan(""), 5, 0, config).valid, "NaN speed");
  config.max_time = 0.01;
  require(!tools::solve_ballistic(15, 5, 0, config).valid, "bounded time");
}
void gating_tests()
{
  auto config = firing_config();
  auto in = sample();
  hero::Controller controller(config);
  auto first = controller.update(in);
  require(first.command.control && first.command.shoot, "aligned stationary target");
  advance(in, 0.02);
  require(!controller.update(in).command.shoot, "cooldown");
  controller.reset_tracking();
  advance(in, 0.02);
  require(!controller.update(in).command.shoot, "reset preserves cooldown");
  advance(in, 0.8);
  require(controller.update(in).command.shoot, "cooldown expires");

  auto check_reject = [&](hero::Input invalid, const char * message) {
    hero::Controller c(config);
    auto d = c.update(invalid);
    require(!d.command.control && !d.command.shoot, message);
  };
  in = sample(); in.enabled = false; check_reject(in, "idle");
  in = sample(); in.tracking = false; check_reject(in, "temp_lost/detecting");
  in = sample(); in.target.reset(); check_reject(in, "no target");
  in = sample(); in.now += 0.2; check_reject(in, "stale frame");
  in = sample(); in.image_time += 0.1; check_reject(in, "future frame");
  in = sample(); in.bullet_speed = 0; check_reject(in, "no infantry speed fallback");
  in = sample(); in.bullet_speed = 23; check_reject(in, "speed outside hero range");
  in = sample(); in.bullet_speed = std::nan(""); check_reject(in, "NaN telemetry");
  in = sample(); in.gimbal_yaw = std::nan(""); check_reject(in, "NaN gimbal");
  in = sample(); in.target->x[0] = std::nan(""); check_reject(in, "NaN state");
  in = sample(); in.target->armor_count = 0; check_reject(in, "invalid plate count");
  in = sample(); in.target->phase_speed_covariance = 1; check_reject(in, "invalid covariance");
  in = sample(); in.target->phase_variance = std::numeric_limits<double>::max();
  in.target->speed_variance = std::numeric_limits<double>::max();
  check_reject(in, "overflowing predicted covariance");
  in = sample(); in.target->x[0] = 40; check_reject(in, "range limit");
  in = sample(); in.target->x[4] = 100; check_reject(in, "unreachable geometry");

  in = sample(); in.gimbal_pitch += 0.1;
  hero::Controller pitch(config);
  require(!pitch.update(in).command.shoot, "pitch alignment required");
  in = sample(); in.gimbal_yaw = 2 * 3.141592653589793;
  hero::Controller wrap(config);
  require(wrap.update(in).command.shoot, "yaw wrap alignment");
  hero::Controller disabled(hero::Config{});
  require(!disabled.update(sample()).command.shoot, "default auto fire off");
  in = sample(); in.gimbal_yaw = 0.005;
  hero::Controller near(config);
  require(near.update(in).command.shoot, "near target accepts small angular error");
  in.target->x[0] = 9;
  in.gimbal_pitch = -tools::solve_ballistic(15, 9 - in.target->x[8], 0, config.ballistic).pitch;
  hero::Controller far(config);
  require(!far.update(in).command.shoot, "same angular error rejected at longer distance");
  in.gimbal_yaw = 0; in.gimbal_pitch += 0.005;
  hero::Controller far_pitch(config);
  require(!far_pitch.update(in).command.shoot, "pitch tolerance also tightens with distance");

  hero::Controller duplicates(config);
  in = sample(); duplicates.update(in);
  in.now += 0.01;
  require(!duplicates.update(in).command.control, "duplicate timestamp");
  in.now += 0.01;
  require(!duplicates.update(in).command.control, "repeated duplicate remains rejected");
  in = sample(9);
  require(!duplicates.update(in).command.control, "backwards clock");

  config.settle_frames = 2;
  hero::Controller settling(config);
  in = sample(); require(!settling.update(in).command.shoot, "settling first frame");
  advance(in, 0.01); require(settling.update(in).command.shoot, "settled next frame");
  advance(in, 1); in.target->identity = 2;
  require(!settling.update(in).command.shoot, "new track settles again");
}
void rotating_tests()
{
  for (double omega : {2.5, -2.5, 6.0, -6.0}) {
    auto config = firing_config();
    auto in = sample();
    auto & target = *in.target;
    target.armor_count = 4;
    target.rotation_observed = true;
    target.x[7] = omega;
    const auto ballistic = tools::solve_ballistic(15, 5 - target.x[8], 0, config.ballistic);
    target.x[6] = -omega * (config.launch_delay + ballistic.time);
    hero::Controller c(config);
    auto result = c.update(in);
    require(result.spin && result.window_open && result.command.shoot, "both rotation directions");
    require(std::abs(result.aim_point[0] - (5 - target.x[8])) < 1e-10 &&
            std::abs(result.aim_point[1]) < 1e-10, "front surface, not rotation axis");

    target.x[6] += 0.5;
    hero::Controller outside(config);
    result = outside.update(in);
    require(result.command.control && !result.command.shoot && !result.window_open, "outside crossing window");
    require(std::abs(result.command.yaw) < 1e-10, "spin aim remains stationary");
    target.x[6] -= 0.5;
    target.speed_variance = 1;
    hero::Controller uncertain(config);
    require(!uncertain.update(in).command.shoot, "future phase uncertainty gate");
    target.speed_variance = 0;
    target.rotation_observed = false;
    hero::Controller unobserved(config);
    require(!unobserved.update(in).command.shoot, "no spin firing before plate switch observed");
  }
  auto in = sample();
  in.target->armor_count = 2;
  hero::Controller two(firing_config());
  require(two.update(in).command.control, "two plate support");
  in = sample();
  in.target->rotation_observed = true;
  in.target->x[6] = -3.141592653589793 / 2;
  in.target->x[9] = 0.1;
  in.target->x[10] = 0.2;
  hero::Controller alternate(firing_config());
  auto result = alternate.update(in);
  require(result.armor_id == 1 && std::abs(result.aim_point[2] - 0.2) < 1e-10,
          "alternating plate radius/height");

  in = sample(); in.target->x[3] = 0.5;
  hero::Controller moving(firing_config());
  result = moving.update(in);
  require(result.command.control && result.aim_point[1] > 0.15, "translation leads target");
}
}  // namespace

int main()
{
  try {
    ballistic_tests(); gating_tests(); rotating_tests();
    auto invalid = firing_config(); invalid.ballistic.drag = -1;
    bool threw = false;
    try { hero::Controller c(invalid); } catch (const std::invalid_argument &) { threw = true; }
    require(threw, "invalid config rejected");
    std::cout << "hero_core_test: " << checks << " checks passed\n";
    return 0;
  } catch (const std::exception & e) {
    std::cerr << "hero_core_test failed: " << e.what() << '\n';
    return 1;
  }
}
