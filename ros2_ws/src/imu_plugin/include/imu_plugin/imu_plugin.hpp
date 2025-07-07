#ifndef IMU_PLUGIN__IMU_PLUGIN_HPP_
#define IMU_PLUGIN__IMU_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <random>

namespace gazebo_plugins
{

  class ImuPlugin : public gazebo::SensorPlugin
  {
  public:
    ImuPlugin() = default;
    virtual ~ImuPlugin() = default;

    void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();

    gazebo::sensors::ImuSensorPtr imu_sensor_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::mt19937 rng_;
    double gyro_noise_stddev_ = 0.01;
    double accel_noise_stddev_ = 0.1;
    double bias_drift_stddev_ = 0.001;
    double hysteresis_width_ = 0.01;

    double gaussian_noise(double stddev);
    double clipping_values(double value, double max, double min);
    double bias_acum = 0.0;
    double gaussian_drift(double current_val, double &bias_acum);
    double prev_val = 0.0;
    int prev_direction = 0;
    double hysteresis(double current_val, double &prev_val, int &prev_direction);
  };

} // namespace gazebo_plugins

#endif // IMU_PLUGIN__IMU_PLUGIN_HPP_
