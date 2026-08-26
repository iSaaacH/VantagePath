#include "vantage/pros/fused_drive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace vantage::pros {
namespace {

std::vector<std::int8_t> portsOf(const std::vector<DriveMotorSpec>& motors) {
  std::vector<std::int8_t> ports;
  ports.reserve(motors.size());
  for (const auto& motor : motors) ports.push_back(motor.port);
  return ports;
}

::pros::v5::MotorGears majorityGearset(
    const std::vector<DriveMotorSpec>& motors) {
  if (motors.empty()) return ::pros::v5::MotorGears::blue;
  std::size_t bestCount = 0;
  auto best = motors.front().gearset;
  for (const auto& candidate : motors) {
    const std::size_t count = static_cast<std::size_t>(std::count_if(
        motors.begin(), motors.end(), [&](const DriveMotorSpec& motor) {
          return motor.gearset == candidate.gearset;
        }));
    if (count > bestCount) {
      bestCount = count;
      best = candidate.gearset;
    }
  }
  return best;
}

}  // namespace

FusedDrive::FusedDrive(std::vector<DriveMotorSpec> motors,
                       FusedDriveConfig config)
    : ::pros::MotorGroup(portsOf(motors), majorityGearset(motors)),
      motors_(std::move(motors)), config_(config),
      fusion_(motors_.size(), {config.rejectFloorDegrees,
                               config.rejectFraction, config.rejectOutliers}),
      outputEnabled_(motors_.size(), true) {
  for (const auto& motor : motors_) {
    if (!std::isfinite(motor.toCommon) || motor.toCommon == 0.0) {
      throw std::invalid_argument("motor toCommon ratio must be finite and nonzero");
    }
  }
}

void FusedDrive::initialize() {
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    ::pros::MotorGroup::set_gearing(motors_[index].gearset,
                                    static_cast<std::uint8_t>(index));
  }
  ::pros::MotorGroup::set_brake_mode(::pros::E_MOTOR_BRAKE_COAST);
  for (std::size_t index = 0; index < outputEnabled_.size(); ++index) {
    setOutputEnabled(index, outputEnabled_[index]);
  }
}

double FusedDrive::fuse() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  std::vector<double> readings(motors_.size());
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    const double raw = ::pros::MotorGroup::get_position(
        static_cast<std::uint8_t>(index));
    readings[index] = std::isfinite(raw)
        ? raw * motors_[index].toCommon
        : std::numeric_limits<double>::infinity();
  }
  return fusion_.update(readings).value;
}

double FusedDrive::get_position(std::uint8_t) const { return fuse(); }

void FusedDrive::setOutlierRejection(bool enabled) {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  config_.rejectOutliers = enabled;
  // Keep the current fused position while replacing the configured engine.
  const double position = fusion_.output().value;
  IncrementalSensorFusion replacement(
      motors_.size(), {config_.rejectFloorDegrees, config_.rejectFraction,
                       config_.rejectOutliers});
  for (std::size_t index = 0; index < motors_.size(); ++index) {
    replacement.setEnabled(index, fusion_.enabled(index));
  }
  replacement.reset(position);
  fusion_ = std::move(replacement);
}

bool FusedDrive::outlierRejection() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return config_.rejectOutliers;
}

void FusedDrive::setReadingEnabled(std::size_t index, bool enabled) {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  if (index < motors_.size()) fusion_.setEnabled(index, enabled);
}

bool FusedDrive::readingEnabled(std::size_t index) const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.enabled(index);
}

void FusedDrive::setOutputEnabled(std::size_t index, bool enabled) {
  if (index >= outputEnabled_.size()) return;
  outputEnabled_[index] = enabled;
  ::pros::MotorGroup::set_voltage_limit(enabled ? 12000 : 0,
                                        static_cast<std::uint8_t>(index));
}

bool FusedDrive::outputEnabled(std::size_t index) const {
  return index < outputEnabled_.size() && outputEnabled_[index];
}

double FusedDrive::motorPositionDegrees(std::size_t index) const {
  if (index >= motors_.size()) return std::numeric_limits<double>::infinity();
  const double raw = ::pros::MotorGroup::get_position(
      static_cast<std::uint8_t>(index));
  return std::isfinite(raw) ? raw * motors_[index].toCommon : raw;
}

bool FusedDrive::alive(std::size_t index) const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.alive(index);
}

bool FusedDrive::rejected(std::size_t index) const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.rejected(index);
}

std::size_t FusedDrive::contributing() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.output().contributing;
}

}  // namespace vantage::pros
