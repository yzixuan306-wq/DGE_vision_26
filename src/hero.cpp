#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/hero/adapter.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  const std::string keys =
    "{help h usage ? | | 输出命令行参数说明}"
    "{@config-path | configs/hero.yaml | 英雄配置文件}"
    "{display | false | 显示检测与决策状态}";
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  const auto config_path = cli.get<std::string>(0);
  const bool display = cli.get<bool>("display");
  if (!cli.check()) { cli.printErrors(); return 1; }

  std::unique_ptr<io::CBoard> board;
  try {
    const auto yaml = YAML::LoadFile(config_path);
    const auto settings = yaml["hero"];
    hero::Adapter adapter(config_path);  // Validate before opening devices.
    const double telemetry_timeout = settings["telemetry_timeout"].as<double>();
    const double imu_offset = settings["imu_time_offset"].as<double>();
    const double max_frame_age = hero::load_config(config_path).max_frame_age;
    if (!std::isfinite(telemetry_timeout) || telemetry_timeout <= 0 || telemetry_timeout > 1 ||
        !std::isfinite(imu_offset) || std::abs(imu_offset) > 0.1)
      throw std::invalid_argument("Invalid telemetry_timeout or imu_time_offset");
    if (!settings["calibration_confirmed"].as<bool>())
      throw std::invalid_argument("Copy measured hero camera/extrinsics to the profile, then set hero.calibration_confirmed=true");

    tools::Exiter exiter;
    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
    board = std::make_unique<io::CBoard>(config_path);
    io::Camera camera(config_path);
    bool was_enabled = false;
    io::Mode previous_mode = io::Mode::idle;
    auto log_time = std::chrono::steady_clock::now();
    auto fresh = [&](const io::CBoardState & state, auto now) {
      const double imu_age = hero::seconds(now) - hero::seconds(state.imu_time);
      const double status_age = hero::seconds(now) - hero::seconds(state.status_time);
      return state.imu_time != std::chrono::steady_clock::time_point{} &&
             state.status_time != std::chrono::steady_clock::time_point{} &&
             imu_age >= 0 && imu_age <= telemetry_timeout &&
             status_age >= 0 && status_age <= telemetry_timeout;
    };

    while (!exiter.exit()) {
      cv::Mat image;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(image, timestamp);
      if (image.empty()) {
        board->send({false, false, 0, 0}, true);
        adapter.reset_tracking();
        was_enabled = false;
        std::this_thread::sleep_for(10ms);
        continue;
      }
      auto state = board->state();
      auto now = std::chrono::steady_clock::now();
      bool enabled = fresh(state, now) && state.mode == io::Mode::auto_aim;
      if (!enabled) {
        board->send({false, false, 0, 0}, true);
        adapter.reset_tracking();
        was_enabled = false;
        continue;
      }
      if (!was_enabled || state.mode != previous_mode) {
        tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
        adapter.reset_tracking();
      }
      previous_mode = state.mode;
      was_enabled = true;
      const auto offset = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(imu_offset));
      solver.set_R_gimbal2world(board->imu_at(timestamp + offset));
      auto armors = detector.detect(image);
      // This feature group has no outpost model; leave shared Tracker behavior unchanged.
      armors.remove_if([](const auto & a) { return a.name == auto_aim::ArmorName::outpost; });
      auto targets = tracker->track(armors, timestamp);
      // Re-read after inference: a mode change or stale serial data invalidates this frame.
      const auto current = board->state();
      now = std::chrono::steady_clock::now();
      enabled = fresh(current, now) && current.mode == state.mode;
      solver.set_R_gimbal2world(current.q);
      const auto ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto decision = adapter.update(targets, tracker->state(),
        timestamp, now, current.bullet_speed, ypr[0], ypr[1], enabled);
      const auto send_state = board->state();
      const auto send_time = std::chrono::steady_clock::now();
      if (!fresh(send_state, send_time) || send_state.mode != current.mode ||
          hero::seconds(send_time) - hero::seconds(timestamp) > max_frame_age) {
        decision.command = {false, false, 0, 0};
        decision.reason = "stale_before_send";
        adapter.reset_tracking();
      }
      board->send(decision.command, true);

      if (now - log_time >= 1s || decision.command.shoot) {
        tools::logger()->info("[Hero] {} spin={} control={} shoot={} flight={:.3f}s phase={:.3f}rad",
          decision.reason, decision.spin, decision.command.control, decision.command.shoot,
          decision.flight_time, decision.impact_phase);
        log_time = now;
      }
      if (display) {
        for (const auto & armor : armors)
          for (std::size_t i = 0; i < armor.points.size(); ++i)
            cv::line(image, armor.points[i], armor.points[(i + 1) % armor.points.size()], {0, 255, 0}, 2);
        cv::putText(image, decision.reason, {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 255}, 2);
        cv::imshow("Hero", image);
        if (cv::waitKey(1) == 'q') break;
      }
    }
    board->send({false, false, 0, 0}, true);
    return 0;
  } catch (const std::exception & error) {
    if (board) board->send({false, false, 0, 0}, true);
    tools::logger()->error("[Hero] {}", error.what());
    return 1;
  }
}
