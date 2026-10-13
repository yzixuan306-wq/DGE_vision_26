#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/hero/adapter.hpp"

namespace
{
void require(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}
auto_aim::Armor observation(auto_aim::Solver & solver, auto_aim::ArmorName name,
                            auto_aim::ArmorType type,
                            Eigen::Vector3d position = {5, 0, 0}, double yaw = 0)
{
  auto points = solver.reproject_armor(position, yaw, type, name);
  // Set explicit test properties after constructing the detector-shaped observation.
  auto_aim::Armor armor(0, 0.99f, cv::boundingRect(points), points);
  armor.name = name;
  armor.type = type;
  armor.color = auto_aim::Color::red;
  armor.priority = auto_aim::ArmorPriority::first;
  return armor;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc != 2) throw std::runtime_error("Expected hero config path");
    std::string path = argv[1];
    auto config = hero::load_config(path);
    require(!config.auto_fire, "example must default to no fire");
    auto_aim::Solver solver(path);
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
    auto_aim::Tracker tracker(path, solver);
    hero::Adapter adapter(path);
    auto time = std::chrono::steady_clock::now();
    std::list<auto_aim::Target> targets;
    hero::Decision d;
    int vehicle_session = 0;
    for (int i = 0; i < 8; ++i) {
      time += std::chrono::milliseconds(10);
      std::list<auto_aim::Armor> armors{observation(solver, auto_aim::three, auto_aim::small)};
      targets = tracker.track(armors, time);
      d = adapter.update(targets, tracker.state(), time,
        time + std::chrono::milliseconds(5), 15, 0, 0, true);
      if (i == 0) vehicle_session = adapter.session_id();
      require(vehicle_session > 0 && adapter.session_id() == vehicle_session,
              "hero session stays stable through detection and tracking");
    }
    require(!targets.empty() && tracker.state() == "tracking", "vehicle reaches tracking");
    require(targets.front().armor_xyza_list().size() == 4, "vehicle four-plate model");
    require(d.command.control && !d.command.shoot && !d.spin, "SP25 vehicle to hero adapter");
    time += std::chrono::milliseconds(10);
    std::list<auto_aim::Armor> none;
    targets = tracker.track(none, time);
    d = adapter.update(targets, tracker.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(!d.command.control && !d.command.shoot, "temp_lost rejects cached target");
    const int old_id = adapter.session_id();
    time += std::chrono::milliseconds(10);
    std::list<auto_aim::Armor> recovered{observation(solver, auto_aim::three, auto_aim::small)};
    targets = tracker.track(recovered, time);
    d = adapter.update(targets, tracker.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(tracker.state() == "tracking" && adapter.session_id() == old_id,
            "temporary loss recovery keeps the native tracking session");
    time += std::chrono::seconds(1);
    std::list<auto_aim::Armor> reacquired{observation(solver, auto_aim::three, auto_aim::small)};
    targets = tracker.track(reacquired, time);
    d = adapter.update(targets, tracker.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(adapter.session_id() > old_id && tracker.state() == "detecting" && !d.command.control,
            "reacquisition starts a new hero session without changing Tracker");
    const int detecting_session = adapter.session_id();
    time += std::chrono::seconds(1);
    std::list<auto_aim::Armor> restarted{observation(solver, auto_aim::three, auto_aim::small)};
    targets = tracker.track(restarted, time);
    d = adapter.update(targets, tracker.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(adapter.session_id() > detecting_session && tracker.state() == "detecting",
            "a gap while detecting creates another session");
    for (int i = 0; i < 4; ++i) {
      time += std::chrono::milliseconds(10);
      std::list<auto_aim::Armor> armors{observation(solver, auto_aim::three, auto_aim::small)};
      targets = tracker.track(armors, time);
      d = adapter.update(targets, tracker.state(), time,
        time + std::chrono::milliseconds(5), 15, 0, 0, true);
    }
    require(tracker.state() == "tracking" && std::string(d.reason) == "settling" && !d.command.shoot,
            "reacquisition must rebuild hero settling frames");
    const int before_reset = adapter.session_id();
    adapter.reset_tracking();
    require(adapter.session_id() == before_reset, "reset preserves the hero session counter");
    auto_aim::Tracker replacement(path, solver);
    time += std::chrono::milliseconds(10);
    std::list<auto_aim::Armor> fresh{observation(solver, auto_aim::three, auto_aim::small)};
    targets = replacement.track(fresh, time);
    d = adapter.update(targets, replacement.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(adapter.session_id() > before_reset && !d.command.control,
            "mode reset and a replacement Tracker start a fresh hero session");

    auto_aim::Tracker balance(path, solver);
    for (int i = 0; i < 8; ++i) {
      time += std::chrono::milliseconds(10);
      std::list<auto_aim::Armor> armors{observation(solver, auto_aim::three, auto_aim::big)};
      targets = balance.track(armors, time);
    }
    require(!targets.empty() && targets.front().armor_xyza_list().size() == 2,
            "two-plate tracking without out-of-bounds access");

    auto_aim::Tracker base(path, solver);
    hero::Adapter base_adapter(path);
    for (int i = 0; i < 8; ++i) {
      time += std::chrono::milliseconds(10);
      std::list<auto_aim::Armor> armors{observation(solver, auto_aim::base, auto_aim::big)};
      targets = base.track(armors, time);
    }
    require(!targets.empty() && targets.front().armor_type == auto_aim::big &&
            targets.front().armor_xyza_list().size() == 3, "large base plate tracking");
    d = base_adapter.update(targets, base.state(), time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(d.command.control && !d.command.shoot, "large base plate hero adaptation");

    // Even if a caller bypasses the entry-point filter, no outpost command may escape.
    auto unsupported = targets;
    unsupported.front().name = auto_aim::outpost;
    time += std::chrono::milliseconds(10);
    d = base_adapter.update(unsupported, "tracking", time,
      time + std::chrono::milliseconds(5), 15, 0, 0, true);
    require(!d.command.control && !d.command.shoot, "outpost excluded from hero adaptation");

    // Exercise the native coordinate chain with a rotated IMU, not only identity poses.
    solver.set_R_gimbal2world(Eigen::Quaterniond(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY())));
    auto_aim::Tracker rotated(path, solver);
    hero::Adapter rotated_adapter(path);
    const Eigen::Vector3d world_point(5, 0.2, 0.1);
    for (int i = 0; i < 8; ++i) {
      time += std::chrono::milliseconds(10);
      std::list<auto_aim::Armor> armors{
        observation(solver, auto_aim::three, auto_aim::small, world_point)};
      targets = rotated.track(armors, time);
      require((armors.front().xyz_in_world - world_point).norm() < 0.005,
              "native Solver preserves world coordinates with rotated IMU");
    }
    require(!targets.empty() && rotated.state() == "tracking", "rotated-IMU vehicle tracking");
    const auto before = targets.front().ekf_x();
    d = rotated_adapter.update(targets, rotated.state(), time,
      time + std::chrono::milliseconds(5), 15, 0.3, -0.1, true);
    require(d.command.control && std::abs(d.command.yaw - std::atan2(0.2, 5)) < 0.01 &&
            d.command.pitch < 0 && !d.command.shoot, "world yaw and upward-negative pitch contract");
    require(targets.front().ekf_x().isApprox(before, 0), "hero prediction must not advance Tracker state");
    std::cout << "hero_integration_test: passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "hero_integration_test: " << error.what() << '\n';
    return 1;
  }
}
