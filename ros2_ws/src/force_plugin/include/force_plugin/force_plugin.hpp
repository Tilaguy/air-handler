#ifndef FORCE_PLUGIN__FORCE_PLUGIN_HPP_
#define FORCE_PLUGIN__FORCE_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <random>
#include <fstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <thread>
/*==================================================================================
  Inculde corresponding sensor message from Gazebo sensor_msgs
====================================================================================*/
#include <gazebo/sensors/ForceTorqueSensor.hh>

namespace force_plugin
{

  class ForcePlugin : public gazebo::SensorPlugin
  {
  public:
    ForcePlugin() = default;
    virtual ~ForcePlugin();

    void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  private:
    void OnUpdate();

    // Socket methods
    bool initUnixSocketServer();
    void acceptUnixSocketClient();
    void sendToSocket(const double &force);

    // Sensor and ROS objects
    /*==================================================================================
      Create variable from Gazebo sensor_msgs
    ====================================================================================*/
    gazebo::sensors::ForceTorqueSensorPtr force_sensor_;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::mt19937 rng_;
    /*==================================================================================
      Initialize Error model variables
    ====================================================================================*/
    double noise_stddev_ = 0;
    double min_lin_force_ = 0;
    double max_lin_force_ = 0;
    double min_force_ = 0;
    double max_force_ = 0;
    double resolution_ = 0;

    double bias_acum = 0.0;
    double prev_val = 0.0;
    int prev_direction = 0;

    // Socket communication
    int server_fd_ = -1;
    int client_fd_ = -1;
    struct sockaddr_un socket_addr_;
    std::thread socket_thread_;
    bool socket_ready_ = false;
  };

} // namespace force_plugin

#endif // FORCE_PLUGIN__FORCE_PLUGIN_HPP_