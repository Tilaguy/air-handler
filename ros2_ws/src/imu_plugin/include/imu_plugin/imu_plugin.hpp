#ifndef IMU_PLUGIN__IMU_PLUGIN_HPP_
#define IMU_PLUGIN__IMU_PLUGIN_HPP_

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <random>
#include <thread>

#include "sensor_utils/noise_utils.hpp"

namespace imu_plugin
{

  class ImuPlugin : public ignition::gazebo::System,
                    public ignition::gazebo::ISystemConfigure,
                    public ignition::gazebo::ISystemPreUpdate
  {
  public:
    ImuPlugin() = default;
    virtual ~ImuPlugin();

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                   ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    ignition::gazebo::Entity entity_;
    std::optional<ignition::math::Vector3d> linear_acceleration_;
    std::optional<ignition::math::Vector3d> angular_velocity_;
    std::chrono::steady_clock::time_point last_update_time_;
    std::chrono::milliseconds update_interval_;
    bool first_update_ = true;

    void OnUpdate();
    bool initUnixSocketServer();
    void acceptUnixSocketClient();
    void sendToSocket(const ignition::math::Vector3d &acc, const ignition::math::Vector3d &gyro);

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    std::thread socket_thread_;

    // Socket communication
    int server_fd_ = -1;
    int client_fd_ = -1;
    struct sockaddr_un socket_addr_;
    bool socket_ready_ = false;

    // Noise generation
    std::mt19937 rng_;
    double gyro_bias_acum = 0;
    double accel_bias_acum = 0;

    double gyro_noise_stddev_ = 0.0;
    double gyro_lim_ = 0.0;
    double gyro_drift_stddev_ = 0.0;
    double gyro_resolution_ = 0.0;
    double accel_noise_stddev_ = 0.0;
    double accel_lim_ = 0.0;
    double accel_drift_stddev_ = 0.0;
    double accel_resolution_ = 0.0;
  };

} // namespace imu_plugin

#endif
