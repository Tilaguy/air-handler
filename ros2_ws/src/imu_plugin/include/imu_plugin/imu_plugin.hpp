// imu_plugin.hpp (actualizado como servidor UNIX)
#ifndef IMU_PLUGIN__IMU_PLUGIN_HPP_
#define IMU_PLUGIN__IMU_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <fstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <thread>

namespace imu_plugin
{

  class ImuPlugin : public gazebo::SensorPlugin
  {
  public:
    ImuPlugin() = default;
    virtual ~ImuPlugin();

    void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();

    // Socket methods
    bool initUnixSocketServer();
    void acceptUnixSocketClient();
    void sendToSocketCSV(const ignition::math::Vector3d &acc, const ignition::math::Vector3d &gyro);

    // Sensor and ROS objects
    gazebo::sensors::ImuSensorPtr imu_sensor_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Noise generator
    std::mt19937 rng_;
    double gyro_noise_stddev_ = 0.0;
    double gyro_lim_ = 0.0;
    double gyro_drift_stddev_ = 0.0;
    double gyro_resolution_ = 0.0;
    double accel_noise_stddev_ = 0.0;
    double accel_lim_ = 0.0;
    double accel_drift_stddev_ = 0.0;
    double accel_resolution_ = 0.0;

    double gyro_bias_acum = 0.0;
    double accel_bias_acum = 0.0;
    double prev_val = 0.0;
    int prev_direction = 0;

    // Socket communication
    int server_fd_ = -1;
    int client_fd_ = -1;
    struct sockaddr_un socket_addr_;
    std::thread socket_thread_;
    bool socket_ready_ = false;
  };

} // namespace imu_plugin

#endif // IMU_PLUGIN__IMU_PLUGIN_HPP_
