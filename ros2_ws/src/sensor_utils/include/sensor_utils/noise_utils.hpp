#ifndef SENSOR_UTILS_NOISE_UTILS_HPP_
#define SENSOR_UTILS_NOISE_UTILS_HPP_

#include <random>

namespace sensor_utils
{

  double gaussian_noise(double stddev);
  double clipping_values(double value, double max, double min);
  double gaussian_drift(double current_val, double stddev, double &bias_acum);
  double hysteresis(double current_val, double &prev_val, int &prev_direction);
  double quantize(double value, double resolution);

} // namespace sensor_utils

#endif // SENSOR_UTILS_NOISE_UTILS_HPP_
