#include "imu_plugin/imu_plugin.hpp"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_utils/noise_utils.hpp"
using sensor_utils::gaussian_noise;

namespace gazebo_plugins
{

  void ImuPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    rng_ = std::mt19937(std::random_device{}());

    // Transduction Stage: Connect to simulated IMU
    imu_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_sensor);
    if (!imu_sensor_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ImuPlugin"), "Failed to cast to ImuSensor");
      return;
    }

    // ROS node and publisher
    ros_node_ = gazebo_ros::Node::Get(_sdf, "imu_plugin_node");
    imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Import parameters from SDF
    gyro_noise_stddev_ = _sdf->Get<double>("gyro_noise_stddev", 0.0).first;
    accel_noise_stddev_ = _sdf->Get<double>("accel_noise_stddev", 0.0).first;
    bias_drift_stddev_ = _sdf->Get<double>("bias_drift_stddev", 0.0).first;
    hysteresis_width_ = _sdf->Get<double>("hysteresis_width", 0.0).first;

    // Timer to publish
    update_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ImuPlugin::OnUpdate, this));
  }

  // --- OnUpdate ---
  void ImuPlugin::OnUpdate()
  {
    if (!imu_sensor_)
      return;

    auto imu_msg = sensor_msgs::msg::Imu();

    // Stage 1: Transduction (raw values from simulator)
    ignition::math::Vector3d linear_acc = imu_sensor_->LinearAcceleration();
    ignition::math::Vector3d angular_vel = imu_sensor_->AngularVelocity();
    ignition::math::Quaterniond orientation = imu_sensor_->Orientation();

    // Stage 2: Error Modeling (imperfections)
    linear_acc.X() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Y() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Z() += gaussian_noise(accel_noise_stddev_);

    // Uncomment for more realistic effects:
    // angular_vel.X() = clipping_values(angular_vel.X(), 10.0, -10.0);
    // angular_vel.X() = gaussian_drift(angular_vel.X(), bias_acum);
    // linear_acc.X() = hysteresis(linear_acc.X(), prev_val, prev_direction);

    // Stage 3: ROS2 Encoding
    imu_msg.header.stamp = ros_node_->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.linear_acceleration.x = linear_acc.X();
    imu_msg.linear_acceleration.y = linear_acc.Y();
    imu_msg.linear_acceleration.z = linear_acc.Z();
    imu_msg.linear_acceleration_covariance[0] = std::pow(accel_noise_stddev_, 2);
    imu_msg.linear_acceleration_covariance[4] = std::pow(accel_noise_stddev_, 2);
    imu_msg.linear_acceleration_covariance[8] = std::pow(accel_noise_stddev_, 2);

    imu_msg.angular_velocity.x = angular_vel.X();
    imu_msg.angular_velocity.y = angular_vel.Y();
    imu_msg.angular_velocity.z = angular_vel.Z();

    imu_msg.orientation.x = orientation.X();
    imu_msg.orientation.y = orientation.Y();
    imu_msg.orientation.z = orientation.Z();
    imu_msg.orientation.w = orientation.W();

    imu_pub_->publish(imu_msg);
  }

  GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

} // namespace gazebo_plugins
