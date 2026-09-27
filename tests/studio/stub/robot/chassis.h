// Minimal stand-in for 4613R's drive::VantageChassis, matching the signatures
// Studio's "chassis" export calls, so CI can check that export compiles.
#pragma once
#include <vantage/vantage.hpp>
#include <vector>

namespace pros { inline void delay(unsigned) {} }

namespace drive {
struct MotionParams {};
class VantageChassis {
 public:
  void setPoseCorner(double, double, double) {}
  void followPath(const std::vector<vantage::Waypoint>&, bool, int, MotionParams = {}, bool = true) {}
  bool waitUntilProgress(double) const { return true; }
  void waitUntilDone() const {}
  void turnToHeading(double, int, MotionParams = {}, bool = true) {}
};
}  // namespace drive
