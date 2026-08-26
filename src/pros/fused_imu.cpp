#include "vantage/pros/fused_imu.hpp"

#include "pros/error.h"

#include <cmath>
#include <limits>
#include <mutex>

namespace vantage::pros {

FusedImu::FusedImu(std::int8_t primaryPort, std::int8_t secondaryPort,
                   double agreementDeltaDegrees)
    : ::pros::Imu(primaryPort), secondary_(secondaryPort),
      fusion_(2, {agreementDeltaDegrees, 0.0, true}) {}

double FusedImu::fuse() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  const FusionOutput output = fusion_.update(
      {::pros::Imu::get_rotation(), secondary_.get_rotation()});
  return output.available ? output.value
                          : std::numeric_limits<double>::infinity();
}

double FusedImu::get_rotation() const { return fuse(); }

double FusedImu::get_heading() const {
  const double rotation = fuse();
  if (!std::isfinite(rotation)) return rotation;
  double heading = std::fmod(rotation, 360.0);
  return heading < 0.0 ? heading + 360.0 : heading;
}

std::int32_t FusedImu::reset(bool blocking) const {
  const std::int32_t primaryResult = ::pros::Imu::reset(blocking);
  const std::int32_t secondaryResult = secondary_.reset(blocking);
  std::lock_guard<::pros::Mutex> lock(mutex_);
  const bool primaryFailed = fusion_.enabled(0) && primaryResult == PROS_ERR;
  const bool secondaryFailed = fusion_.enabled(1) && secondaryResult == PROS_ERR;
  fusion_.reset();
  return primaryFailed || secondaryFailed ? PROS_ERR : 1;
}

bool FusedImu::is_calibrating() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return (fusion_.enabled(0) && ::pros::Imu::is_calibrating()) ||
         (fusion_.enabled(1) && secondary_.is_calibrating());
}

std::int32_t FusedImu::set_data_rate(std::uint32_t rate) const {
  const std::int32_t primaryResult = ::pros::Imu::set_data_rate(rate);
  const std::int32_t secondaryResult = secondary_.set_data_rate(rate);
  std::lock_guard<::pros::Mutex> lock(mutex_);
  const bool primaryFailed = fusion_.enabled(0) && primaryResult == PROS_ERR;
  const bool secondaryFailed = fusion_.enabled(1) && secondaryResult == PROS_ERR;
  return primaryFailed || secondaryFailed ? PROS_ERR : 1;
}

void FusedImu::setReadingEnabled(std::size_t index, bool enabled) {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  if (index < 2) fusion_.setEnabled(index, enabled);
}

bool FusedImu::readingEnabled(std::size_t index) const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.enabled(index);
}

std::size_t FusedImu::contributing() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.output().contributing;
}

bool FusedImu::primaryAlive() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.alive(0);
}

bool FusedImu::secondaryAlive() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.alive(1);
}

bool FusedImu::primaryRejected() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.rejected(0);
}

bool FusedImu::secondaryRejected() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return fusion_.rejected(1);
}

double FusedImu::disagreementDegrees() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  const double primary = ::pros::Imu::get_rotation();
  const double secondary = secondary_.get_rotation();
  return std::isfinite(primary) && std::isfinite(secondary)
      ? std::abs(primary - secondary) : 0.0;
}

double FusedImu::primaryRotationDegrees() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return ::pros::Imu::get_rotation();
}

double FusedImu::secondaryRotationDegrees() const {
  std::lock_guard<::pros::Mutex> lock(mutex_);
  return secondary_.get_rotation();
}

}  // namespace vantage::pros
