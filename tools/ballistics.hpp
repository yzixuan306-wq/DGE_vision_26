#ifndef TOOLS__BALLISTICS_HPP
#define TOOLS__BALLISTICS_HPP

namespace tools
{
// SI units; pitch is positive upwards. drag = rho * Cd * area / (2 * mass), in 1/m.
struct BallisticConfig
{
  double gravity = 9.7833;
  double drag = 0.0;  // Must be calibrated; zero provides a vacuum baseline.
  double min_pitch = -0.35;
  double max_pitch = 0.70;
  double max_time = 3.0;
  double step = 0.002;
};

struct BallisticSolution
{
  bool valid = false;
  double pitch = 0;
  double time = 0;
};

// Bounded RK4 integration of quadratic drag in both axes; selects the first (low) arc.
BallisticSolution solve_ballistic(
  double speed, double distance, double height, const BallisticConfig & config);
bool valid_ballistic_config(const BallisticConfig & config);
}  // namespace tools
#endif
