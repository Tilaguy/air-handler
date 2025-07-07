#include "imu_plugin/imu_plugin.hpp"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sensor_msgs/msg/imu.hpp>

namespace gazebo_plugins
{

  void ImuPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    rng_ = std::mt19937(std::random_device{}());
    imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
    if (!imu_sensor_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ImuPlugin"), "Failed to cast to ImuSensor");
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(_sdf, "imu_plugin_node");

    imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Import SDF prams
    if (_sdf->HasElement("gyro_noise_stddev"))
    {
      gyro_noise_stddev_ = 1e-7; //_sdf->Get<double>("gyro_noise_stddev");
      RCLCPP_INFO(ros_node_->get_logger(), "gyro_noise_stddev: %e", gyro_noise_stddev_);
    }
    if (_sdf->HasElement("accel_noise_stddev"))
      accel_noise_stddev_ = _sdf->Get<double>("accel_noise_stddev");
    if (_sdf->HasElement("bias_drift_stddev"))
      bias_drift_stddev_ = _sdf->Get<double>("bias_drift_stddev");
    if (_sdf->HasElement("hysteresis_width"))
      hysteresis_width_ = _sdf->Get<double>("hysteresis_width");

    // Setup timer to periodically publish data
    update_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ImuPlugin::OnUpdate, this));
  }

  double ImuPlugin::gaussian_noise(double stddev)
  {
    std::normal_distribution<double> dist(0.0, stddev);
    return dist(rng_); // rng_ es un std::mt19937
  }

  double ImuPlugin::clipping_values(double value, double max, double min)
  {
    if (value > max)
      value = max;
    if (value < min)
      value = min;

    return value;
  }

  double ImuPlugin::gaussian_drift(double current_val, double &bias_acum)
  {
    double bias = gaussian_noise(bias_drift_stddev_);

    double output_val = current_val + bias_acum;
    bias_acum += std::labs(bias);

    return output_val;
  }

  double ImuPlugin::hysteresis(double current_val, double &prev_val, int &prev_direction)
  {
    int current_direction = (current_val > prev_val) ? 1 : ((current_val < prev_val) ? -1 : prev_direction);
    float output_emp = current_val + (current_direction * hysteresis_width_);

    prev_val = current_val;
    prev_direction = current_direction;

    return output_emp;
  }

  void ImuPlugin::OnUpdate()
  {
    if (!imu_sensor_)
      return;

    auto imu_msg = sensor_msgs::msg::Imu();

    // Get data from the simulation sensor
    ignition::math::Vector3d linear_acc = imu_sensor_->LinearAcceleration();
    ignition::math::Vector3d angular_vel = imu_sensor_->AngularVelocity();
    ignition::math::Quaterniond orientation = imu_sensor_->Orientation();

    // Real considerations
    angular_vel.Y() = angular_vel.X();
    linear_acc.Y() = linear_acc.X();

    angular_vel.Z() = gaussian_noise(gyro_noise_stddev_);
    linear_acc.Z() = gaussian_noise(accel_noise_stddev_);

    angular_vel.X() += angular_vel.Z();
    linear_acc.X() += linear_acc.Z();

    /*
    angular_vel.X() = clipping_values(angular_vel.X(), 10.0, -10.0);
    linear_acc.X() = clipping_values(linear_acc.X(), 5.0, -5.0);

    angular_vel.X() = gaussian_drift(angular_vel.X(), bias_acum);

    linear_acc.X() = hysteresis(linear_acc.X(), prev_val, prev_direction);
    */

    // ROS2 topick struturing
    imu_msg.header.stamp = ros_node_->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = linear_acc.X();
    imu_msg.linear_acceleration.y = linear_acc.Y();
    imu_msg.linear_acceleration.z = linear_acc.Z();

    imu_msg.angular_velocity.x = angular_vel.X();
    imu_msg.angular_velocity.y = angular_vel.Y();
    imu_msg.angular_velocity.z = angular_vel.Z();

    imu_msg.orientation.x = orientation.X();
    imu_msg.orientation.y = orientation.Y();
    imu_msg.orientation.z = orientation.Z();
    imu_msg.orientation.w = orientation.W();

    imu_pub_->publish(imu_msg);
  }

  // Register plugin with Gazebo
  GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

} // namespace gazebo_plugins
