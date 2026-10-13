#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "io/cboard.hpp"

using namespace std::chrono_literals;
static_assert(sizeof(io::IMUDataFrame) == 10);
static_assert(sizeof(io::StatusDataFrame) == 9);
static_assert(sizeof(io::ControlDataFrame) == 8);

int main()
{
  int master = -1, slave = -1;
  char name[128]{};
  const auto config = std::filesystem::temp_directory_path() /
    ("hero-cboard-test-" + std::to_string(getpid()) + ".yaml");
  try {
    if (openpty(&master, &slave, name, nullptr, nullptr) != 0) throw std::runtime_error("openpty failed");
    fcntl(master, F_SETFL, O_NONBLOCK);
    std::ofstream yaml(config);
    yaml << "serial_device: " << name << "\nserial_baudrate: 115200\n";
    yaml.close();
    {
      io::CBoard board(config.string());
      auto initial = board.state();
      if (initial.bullet_speed != 0 || initial.mode != io::idle ||
          initial.imu_time != std::chrono::steady_clock::time_point{} ||
          initial.status_time != std::chrono::steady_clock::time_point{})
        throw std::runtime_error("snapshot must start unavailable");
      // Preserve DGE's 29-byte radar packet. Embedded AA/BB bytes must not become telemetry.
      std::array<unsigned char, 33> radar{};
      radar.fill(0x42);
      radar[0] = 0x5A;
      radar[28] = 0x0D;
      const std::array<unsigned char, 10> embedded_imu{0xAA, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0xDD};
      const std::array<unsigned char, 9> embedded_status{0xBB, 0x40, 0x06, 1, 0, 0, 0, 0, 0};
      std::copy(embedded_imu.begin(), embedded_imu.end(), radar.begin() + 1);
      std::copy(embedded_status.begin(), embedded_status.end(), radar.begin() + 12);
      if (write(master, radar.data(), radar.size()) != static_cast<ssize_t>(radar.size()))
        throw std::runtime_error("PTY radar write failed");
      std::this_thread::sleep_for(20ms);
      if (board.state().imu_time != initial.imu_time || board.state().status_time != initial.status_time)
        throw std::runtime_error("DGE radar packet leaked into AA/BB telemetry");
      // Noise + deliberately split IMU frame, then one status frame.
      const std::array<unsigned char, 21> packet{
        0x01, 0x02, 0xAA, 0x10, 0x27, 0, 0, 0, 0, 0, 0, 0xDD,
        0xBB, 0xDC, 0x05, 0x04, 0, 0, 0, 0, 0};
      if (write(master, packet.data(), 6) != 6) throw std::runtime_error("PTY write failed");
      std::this_thread::sleep_for(5ms);
      if (write(master, packet.data() + 6, packet.size() - 6) != 15)
        throw std::runtime_error("PTY write failed");
      auto deadline = std::chrono::steady_clock::now() + 1s;
      io::CBoardState state;
      do {
        state = board.state();
        if (state.status_time != std::chrono::steady_clock::time_point{}) break;
        std::this_thread::sleep_for(2ms);
      } while (std::chrono::steady_clock::now() < deadline);
      if (state.mode != io::outpost || std::abs(state.bullet_speed - 15) > 1e-9 ||
          std::abs(state.q.w() - 1) > 1e-9 || state.imu_time == initial.imu_time)
        throw std::runtime_error("CBoard snapshot decode failed");
      board.send({true, true, 0.1, -0.2}, true);
      std::vector<unsigned char> output;
      deadline = std::chrono::steady_clock::now() + 1s;
      while (output.size() < 8 && std::chrono::steady_clock::now() < deadline) {
        unsigned char buffer[64];
        auto size = read(master, buffer, sizeof(buffer));
        if (size > 0) output.insert(output.end(), buffer, buffer + size);
        else std::this_thread::sleep_for(2ms);
      }
      const std::vector<unsigned char> expected{0xCC, 1, 1, 0xE8, 0x03, 0x30, 0xF8, 0xEE};
      if (output != expected) throw std::runtime_error("CBoard control wire format changed");
      const std::array<unsigned char, 9> invalid_mode{0xBB, 0xDC, 0x05, 0x7F, 0, 0, 0, 0, 0};
      if (write(master, invalid_mode.data(), invalid_mode.size()) != 9)
        throw std::runtime_error("PTY write failed");
      deadline = std::chrono::steady_clock::now() + 1s;
      while (board.state().status_time == state.status_time && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(2ms);
      if (board.state().mode != io::idle) throw std::runtime_error("Invalid mode must become idle");
      const auto stamp = board.state().status_time;
      std::this_thread::sleep_for(10ms);
      if (board.state().status_time != stamp) throw std::runtime_error("No fabricated telemetry freshness");
    }
    close(master); close(slave);
    std::filesystem::remove(config);
    std::cout << "hero_cboard_test: PTY receive/snapshot/transmit passed\n";
    return 0;
  } catch (const std::exception & error) {
    if (master >= 0) close(master);
    if (slave >= 0) close(slave);
    std::filesystem::remove(config);
    std::cerr << "hero_cboard_test: " << error.what() << '\n';
    return 1;
  }
}
