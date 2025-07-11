#ifndef SENSOR_UTILS_NOISE_UTILS_HPP_
#define SENSOR_UTILS_NOISE_UTILS_HPP_

#include <random>

namespace sensor_utils
{

  double gaussian_noise(double stddev);
  double clipping_values(double value, double max, double min);
  double gaussian_drift(double current_val, double &bias_acum);
  double hysteresis(double current_val, double &prev_val, int &prev_direction);

} // namespace sensor_utils

#endif // SENSOR_UTILS_NOISE_UTILS_HPP_
