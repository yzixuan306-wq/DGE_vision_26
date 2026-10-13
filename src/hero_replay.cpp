#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/hero/adapter.hpp"
#include "tools/math_tools.hpp"

// SP25 Recorder format: <prefix>.avi + <prefix>.txt, each text row: seconds w x y z.
// Replay has no CBoard/Camera instance and never writes control packets.
int main(int argc, char ** argv)
{
  const std::string keys =
    "{help h usage ? | | 输出参数说明}"
    "{@input | assets/demo/demo | avi/txt共同前缀}"
    "{config c | configs/hero.yaml | 英雄配置}"
    "{speed | 15.0 | 回放弹速 m/s，原始录像不含弹速}"
    "{latency | 0.005 | 模拟图像到决策的耗时 s}"
    "{output | hero_replay.csv | CSV输出路径}";
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) { cli.printMessage(); return 0; }
  try {
    const auto prefix = cli.get<std::string>(0);
    const auto path = cli.get<std::string>("config");
    const auto output = cli.get<std::string>("output");
    const double speed = cli.get<double>("speed");
    const double latency = cli.get<double>("latency");
    if (!cli.check()) { cli.printErrors(); return 1; }
    const auto config = hero::load_config(path);
    if (!std::isfinite(speed) || speed < config.min_speed || speed > config.max_speed ||
        !std::isfinite(latency) || latency < 0 || latency > config.max_frame_age)
      throw std::invalid_argument("Invalid replay speed or latency");
    cv::VideoCapture video(prefix + ".avi");
    std::ifstream poses(prefix + ".txt");
    if (!video.isOpened() || !poses) throw std::runtime_error("Cannot open avi/txt recording pair");
    if (output == prefix + ".avi" || output == prefix + ".txt" || output == path)
      throw std::invalid_argument("Output must not overwrite an input");
    std::ofstream csv(output);
    if (!csv) throw std::runtime_error("Cannot open replay CSV output");
    csv << "frame,time,tracker,reason,control,shoot,yaw,pitch,flight_time,phase,phase_std,spin,armor_id,x,y,z\n";
    csv << std::setprecision(10);
    auto_aim::YOLO detector(path, false);
    auto_aim::Solver solver(path);
    auto_aim::Tracker tracker(path, solver);
    hero::Adapter adapter(path);
    const auto epoch = std::chrono::steady_clock::now();
    int frame = 0;
    double previous = -1;
    cv::Mat image;
    while (video.read(image)) {
      double t, w, x, y, z;
      if (!(poses >> t >> w >> x >> y >> z)) throw std::runtime_error("Missing/invalid pose row");
      Eigen::Quaterniond q(w, x, y, z);
      if (!std::isfinite(t) || t < 0 || t <= previous || !q.coeffs().allFinite() ||
          std::abs(q.norm() - 1) > 0.1) throw std::runtime_error("Invalid timestamp/quaternion in recording");
      previous = t;
      const auto timestamp = epoch + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(t));
      const auto now = timestamp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(latency));
      solver.set_R_gimbal2world(q.normalized());
      auto armors = detector.detect(image, frame);
      armors.remove_if([](const auto & a) { return a.name == auto_aim::ArmorName::outpost; });
      auto targets = tracker.track(armors, timestamp);
      const auto ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      auto decision = adapter.update(targets, tracker.state(), timestamp,
        now, speed, ypr[0], ypr[1], true);
      const auto & cmd = decision.command;
      csv << frame++ << ',' << t << ',' << tracker.state() << ',' << decision.reason << ','
          << cmd.control << ',' << cmd.shoot << ',' << cmd.yaw << ',' << cmd.pitch << ','
          << decision.flight_time << ',' << decision.impact_phase << ',' << decision.phase_std << ','
          << decision.spin << ',' << decision.armor_id << ',' << decision.aim_point[0] << ','
          << decision.aim_point[1] << ',' << decision.aim_point[2] << '\n';
      if (!csv) throw std::runtime_error("Replay CSV write failed");
    }
    if (frame == 0) throw std::runtime_error("Recording has no readable video frames");
    std::cout << "Replayed " << frame << " frames to " << output << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "hero_replay: " << error.what() << '\n';
    return 1;
  }
}
