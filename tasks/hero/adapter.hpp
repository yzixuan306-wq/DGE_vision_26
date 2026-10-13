#ifndef HERO__ADAPTER_HPP
#define HERO__ADAPTER_HPP

#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "controller.hpp"
#include "tasks/auto_aim/target.hpp"

namespace hero
{
Config load_config(const std::string & path);
double seconds(std::chrono::steady_clock::time_point time);

// The adapter is the only bridge from SP25/Eigen/YAML types to the standalone controller.
class Adapter
{
public:
  explicit Adapter(const std::string & path);
  Decision update(
    const std::list<auto_aim::Target> & targets, const std::string & tracker_state,
    std::chrono::steady_clock::time_point image_time,
    std::chrono::steady_clock::time_point now, double speed, double yaw, double pitch,
    bool enabled);
  // Observe each native Tracker result. Reset when the caller replaces its Tracker.
  void reset_tracking();
  int session_id() const { return session_id_; }
private:
  Controller controller_;
  int session_id_ = 0;
  bool session_active_ = false;
  bool was_detecting_ = false;
  std::optional<std::chrono::steady_clock::time_point> last_image_time_;
};
}  // namespace hero
#endif
