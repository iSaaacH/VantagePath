#pragma once

#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include "vantage/sensor_fusion.hpp"

#include <cstddef>
#include <cstdint>

namespace vantage::pros {

// Two V5 inertial sensors exposed as one continuous, fault-tolerant IMU.
class FusedImu : public ::pros::Imu {
 public:
  FusedImu(std::int8_t primaryPort, std::int8_t secondaryPort,
           double agreementDeltaDegrees);

  double get_rotation() const override;
  double get_heading() const override;
  std::int32_t reset(bool blocking = false) const override;
  bool is_calibrating() const override;
  std::int32_t set_data_rate(std::uint32_t rate) const override;

  void setReadingEnabled(std::size_t index, bool enabled);
  bool readingEnabled(std::size_t index) const;
  std::size_t contributing() const;
  bool primaryAlive() const;
  bool secondaryAlive() const;
  bool primaryRejected() const;
  bool secondaryRejected() const;
  double disagreementDegrees() const;
  double primaryRotationDegrees() const;
  double secondaryRotationDegrees() const;

 private:
  double fuse() const;

  ::pros::Imu secondary_;
  mutable ::pros::Mutex mutex_;
  mutable IncrementalSensorFusion fusion_;
};

}  // namespace vantage::pros
