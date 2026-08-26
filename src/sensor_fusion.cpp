#include "vantage/sensor_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vantage {
namespace {

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t size = values.size();
  return size % 2 ? values[size / 2]
                  : 0.5 * (values[size / 2 - 1] + values[size / 2]);
}

double gateFor(double reference, const IncrementalFusionConfig& config) {
  return std::max(config.absoluteThreshold,
                  config.relativeThreshold * std::abs(reference));
}

double combine(const std::vector<double>& values,
               const std::vector<std::size_t>& indices,
               const IncrementalFusionConfig& config,
               double continuityReference, std::vector<bool>* rejected,
               std::size_t* contributing) {
  const std::size_t count = values.size();
  if (count == 0) {
    *contributing = 0;
    return 0.0;
  }
  if (!config.rejectOutliers || count == 1) {
    double sum = 0.0;
    for (double value : values) sum += value;
    *contributing = count;
    return sum / count;
  }
  if (count == 2) {
    const double mean = 0.5 * (values[0] + values[1]);
    if (std::abs(values[0] - values[1]) <= gateFor(mean, config)) {
      *contributing = 2;
      return mean;
    }
    const bool keepFirst = std::abs(values[0] - continuityReference) <=
                           std::abs(values[1] - continuityReference);
    (*rejected)[indices[keepFirst ? 1 : 0]] = true;
    *contributing = 1;
    return values[keepFirst ? 0 : 1];
  }

  // Reject a sensor only when every other sensor forms a consensus without it.
  std::vector<bool> drop(count, false);
  for (std::size_t candidate = 0; candidate < count; ++candidate) {
    std::vector<double> peers;
    peers.reserve(count - 1);
    for (std::size_t index = 0; index < count; ++index) {
      if (index != candidate) peers.push_back(values[index]);
    }
    const double reference = median(peers);
    const double gate = gateFor(reference, config);
    const auto limits = std::minmax_element(peers.begin(), peers.end());
    if (*limits.second - *limits.first <= gate &&
        std::abs(values[candidate] - reference) > gate) {
      drop[candidate] = true;
    }
  }

  double sum = 0.0;
  std::size_t used = 0;
  for (std::size_t index = 0; index < count; ++index) {
    if (drop[index]) {
      (*rejected)[indices[index]] = true;
    } else {
      sum += values[index];
      ++used;
    }
  }
  if (used == 0) {
    std::fill(rejected->begin(), rejected->end(), false);
    *contributing = count;
    return median(values);
  }
  *contributing = used;
  return sum / used;
}

}  // namespace

IncrementalSensorFusion::IncrementalSensorFusion(
    std::size_t sensorCount, IncrementalFusionConfig config)
    : config_(config), previous_(sensorCount, 0.0), enabled_(sensorCount, true),
      alive_(sensorCount, false), rejected_(sensorCount, false) {
  if (sensorCount == 0 || config.absoluteThreshold < 0.0 ||
      config.relativeThreshold < 0.0) {
    throw std::invalid_argument("sensor fusion requires sensors and nonnegative gates");
  }
}

FusionOutput IncrementalSensorFusion::update(
    const std::vector<double>& readings) {
  if (readings.size() != sensorCount()) {
    throw std::invalid_argument("sensor reading count does not match fusion size");
  }
  std::vector<bool> currentAlive(sensorCount(), false);
  for (std::size_t index = 0; index < sensorCount(); ++index) {
    currentAlive[index] = enabled_[index] && std::isfinite(readings[index]);
    if (currentAlive[index] && !alive_[index]) previous_[index] = readings[index];
  }

  std::fill(rejected_.begin(), rejected_.end(), false);
  if (!seeded_) {
    for (std::size_t index = 0; index < sensorCount(); ++index) {
      if (currentAlive[index]) previous_[index] = readings[index];
    }
    alive_ = currentAlive;
    seeded_ = true;
    output_.delta = 0.0;
    output_.contributing = static_cast<std::size_t>(
        std::count(currentAlive.begin(), currentAlive.end(), true));
    output_.available = output_.contributing > 0;
    return output_;
  }

  std::vector<double> deltas;
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < sensorCount(); ++index) {
    if (!currentAlive[index]) continue;
    deltas.push_back(readings[index] - previous_[index]);
    indices.push_back(index);
  }

  std::size_t contributing = 0;
  const double applied = combine(deltas, indices, config_, previousDelta_,
                                 &rejected_, &contributing);
  if (contributing > 0) {
    output_.value += applied;
    previousDelta_ = applied;
  }
  output_.delta = contributing > 0 ? applied : 0.0;
  output_.contributing = contributing;
  output_.available = contributing > 0;
  for (std::size_t index = 0; index < sensorCount(); ++index) {
    if (currentAlive[index]) previous_[index] = readings[index];
  }
  alive_ = currentAlive;
  return output_;
}

void IncrementalSensorFusion::reset(double value) {
  std::fill(previous_.begin(), previous_.end(), 0.0);
  std::fill(alive_.begin(), alive_.end(), false);
  std::fill(rejected_.begin(), rejected_.end(), false);
  output_ = {value, 0.0, 0, false};
  previousDelta_ = 0.0;
  seeded_ = false;
}

void IncrementalSensorFusion::setEnabled(std::size_t index, bool enabledValue) {
  if (index >= sensorCount()) throw std::out_of_range("sensor index out of range");
  enabled_[index] = enabledValue;
  if (!enabledValue) alive_[index] = false;
}

bool IncrementalSensorFusion::enabled(std::size_t index) const {
  return index < sensorCount() && enabled_[index];
}

bool IncrementalSensorFusion::alive(std::size_t index) const {
  return index < sensorCount() && alive_[index];
}

bool IncrementalSensorFusion::rejected(std::size_t index) const {
  return index < sensorCount() && rejected_[index];
}

}  // namespace vantage
