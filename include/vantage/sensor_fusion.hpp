#pragma once

#include <cstddef>
#include <vector>

namespace vantage {

struct IncrementalFusionConfig {
  double absoluteThreshold = 0.0;
  double relativeThreshold = 0.0;
  bool rejectOutliers = true;
};

struct FusionOutput {
  double value = 0.0;
  double delta = 0.0;
  std::size_t contributing = 0;
  bool available = false;
};

// Robustly fuses cumulative sensors that measure the same quantity. Fusion is
// performed on deltas, so sensors may have different absolute zeros. Non-finite
// readings are unavailable; a returning sensor is reseeded without a jump.
class IncrementalSensorFusion {
 public:
  explicit IncrementalSensorFusion(std::size_t sensorCount,
                                   IncrementalFusionConfig config = {});

  FusionOutput update(const std::vector<double>& readings);
  void reset(double value = 0.0);

  void setEnabled(std::size_t index, bool enabled);
  bool enabled(std::size_t index) const;
  bool alive(std::size_t index) const;
  bool rejected(std::size_t index) const;

  std::size_t sensorCount() const { return enabled_.size(); }
  const FusionOutput& output() const { return output_; }

 private:
  IncrementalFusionConfig config_;
  std::vector<double> previous_;
  std::vector<bool> enabled_;
  std::vector<bool> alive_;
  std::vector<bool> rejected_;
  FusionOutput output_;
  double previousDelta_ = 0.0;
  bool seeded_ = false;
};

}  // namespace vantage
