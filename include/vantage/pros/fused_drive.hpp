#pragma once

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "vantage/sensor_fusion.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace vantage::pros {

struct DriveMotorSpec {
  std::int8_t port = 0;
  ::pros::v5::MotorGears gearset = ::pros::v5::MotorGears::blue;
  // Multiplies this motor's encoder into common-shaft units.
  double toCommon = 1.0;
};

struct FusedDriveConfig {
  double rejectFloorDegrees = 12.0;
  double rejectFraction = 0.5;
  bool rejectOutliers = true;
};

// A PROS MotorGroup that drives every motor but reports one fault-tolerant,
// common-shaft encoder position. Per-motor gearing and encoder ratios allow
// mixed motor cartridges and external coupling on one drive side.
class FusedDrive : public ::pros::MotorGroup {
 public:
  FusedDrive(std::vector<DriveMotorSpec> motors, FusedDriveConfig config = {});

  // Apply each physical motor's true cartridge after PROS device startup.
  void initialize();
  double get_position(std::uint8_t index = 0) const override;

  void setOutlierRejection(bool enabled);
  bool outlierRejection() const;
  void setReadingEnabled(std::size_t index, bool enabled);
  bool readingEnabled(std::size_t index) const;
  void setOutputEnabled(std::size_t index, bool enabled);
  bool outputEnabled(std::size_t index) const;

  double motorPositionDegrees(std::size_t index) const;
  bool alive(std::size_t index) const;
  bool rejected(std::size_t index) const;
  std::size_t contributing() const;
  std::size_t motorCount() const { return motors_.size(); }

 private:
  double fuse() const;

  std::vector<DriveMotorSpec> motors_;
  FusedDriveConfig config_;
  mutable ::pros::Mutex mutex_;
  mutable IncrementalSensorFusion fusion_;
  std::vector<bool> outputEnabled_;
};

}  // namespace vantage::pros
