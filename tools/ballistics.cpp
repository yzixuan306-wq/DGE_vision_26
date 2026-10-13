#include "ballistics.hpp"

#include <array>
#include <algorithm>
#include <cmath>

namespace tools
{
namespace
{
using State = std::array<double, 4>;  // horizontal position, height, horizontal/vertical speed
State derivative(const State & s, const BallisticConfig & c)
{
  const double v = std::hypot(s[2], s[3]);
  return {s[2], s[3], -c.drag * v * s[2], -c.gravity - c.drag * v * s[3]};
}
State add(const State & a, const State & b, double scale)
{
  return {a[0] + scale * b[0], a[1] + scale * b[1], a[2] + scale * b[2],
          a[3] + scale * b[3]};
}
struct Crossing
{
  bool valid = false;
  double height = 0;
  double time = 0;
};
Crossing integrate(double speed, double pitch, double distance, const BallisticConfig & c)
{
  State s{0, 0, speed * std::cos(pitch), speed * std::sin(pitch)};
  const int steps = static_cast<int>(std::ceil(c.max_time / c.step));
  for (int i = 0; i < steps; ++i) {
    const double elapsed = i * c.step;
    const double dt = std::min(c.step, c.max_time - elapsed);
    auto k1 = derivative(s, c);
    auto k2 = derivative(add(s, k1, dt / 2), c);
    auto k3 = derivative(add(s, k2, dt / 2), c);
    auto k4 = derivative(add(s, k3, dt), c);
    State next{};
    for (int j = 0; j < 4; ++j)
      next[j] = s[j] + dt * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]) / 6;
    if (!std::isfinite(next[0]) || !std::isfinite(next[1]) || next[2] <= 0) return {};
    if (next[0] >= distance) {
      const double fraction = (distance - s[0]) / (next[0] - s[0]);
      return {true, s[1] + fraction * (next[1] - s[1]), elapsed + fraction * dt};
    }
    s = next;
  }
  return {};
}
}  // namespace

bool valid_ballistic_config(const BallisticConfig & c)
{
  return std::isfinite(c.gravity) && c.gravity > 0 && c.gravity <= 30 &&
         std::isfinite(c.drag) && c.drag >= 0 && c.drag <= 1 &&
         std::isfinite(c.min_pitch) && std::isfinite(c.max_pitch) &&
         c.min_pitch > -1.5 && c.max_pitch < 1.5 && c.min_pitch < c.max_pitch &&
         std::isfinite(c.max_time) && c.max_time > 0 && c.max_time <= 10 &&
         std::isfinite(c.step) && c.step >= 0.0005 && c.step <= 0.02;
}

BallisticSolution solve_ballistic(double speed, double distance, double height, const BallisticConfig & c)
{
  if (!valid_ballistic_config(c) || !std::isfinite(speed) || speed <= 0 || speed > 100 ||
      !std::isfinite(distance) || distance <= 0.001 || !std::isfinite(height)) return {};
  // Bracket the first ascending root. Do not silently substitute a high arc or an endpoint.
  double previous_pitch = c.min_pitch;
  Crossing previous = integrate(speed, previous_pitch, distance, c);
  if (previous.valid && std::abs(previous.height - height) < 1e-5)
    return {true, previous_pitch, previous.time};
  constexpr int samples = 80;
  for (int i = 1; i <= samples; ++i) {
    double pitch = c.min_pitch + (c.max_pitch - c.min_pitch) * i / samples;
    Crossing current = integrate(speed, pitch, distance, c);
    if (previous.valid && current.valid && previous.height <= height && current.height >= height) {
      double low = previous_pitch, high = pitch;
      for (int n = 0; n < 32; ++n) {
        const double mid = (low + high) / 2;
        Crossing crossing = integrate(speed, mid, distance, c);
        if (!crossing.valid) return {};
        if (std::abs(crossing.height - height) < 1e-5) return {true, mid, crossing.time};
        if (crossing.height < height) low = mid;
        else high = mid;
      }
      const double mid = (low + high) / 2;
      Crossing crossing = integrate(speed, mid, distance, c);
      if (crossing.valid && std::abs(crossing.height - height) < 1e-4)
        return {true, mid, crossing.time};
      return {};
    }
    previous_pitch = pitch;
    previous = current;
  }
  return {};
}
}  // namespace tools
