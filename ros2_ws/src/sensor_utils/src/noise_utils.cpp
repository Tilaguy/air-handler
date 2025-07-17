#include "sensor_utils/noise_utils.hpp"

namespace sensor_utils
{

  static std::mt19937 rng(std::random_device{}());

  double gaussian_noise(double stddev)
  {
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng);
  }

  double clipping_values(double value, double max, double min)
  {
    if (value > max)
      value = max;
    if (value < min)
      value = min;
    return value;
  }

  double gaussian_drift(double current_val, double stddev, double &bias_acum)
  {
    double bias = gaussian_noise(stddev);
    double output_val = current_val + bias_acum;
    bias_acum += std::abs(bias);
    return output_val;
  }

  double hysteresis(double current_val, double hysteresis_width, double &prev_val, int &prev_direction)
  {
    int current_direction = (current_val > prev_val) ? 1 : ((current_val < prev_val) ? -1 : prev_direction);
    double output_val = current_val + current_direction * hysteresis_width;
    prev_val = current_val;
    prev_direction = current_direction;
    return output_val;
  }

  double quantize(double value, double resolution)
  {
    if (resolution <= 0.0)
      return value;
    return std::round(value / resolution) * resolution;
  }

} // namespace sensor_utils
