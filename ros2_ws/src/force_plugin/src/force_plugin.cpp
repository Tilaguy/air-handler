#include "force_plugin/force_plugin.hpp"
#include <sensor_utils/noise_utils.hpp>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <iomanip> // std::setprecision
#include <sstream>

/*==================================================================================
  Use the Error model functions from sensor_utils
====================================================================================*/
using sensor_utils::clipping_values;
using sensor_utils::gaussian_noise;
using sensor_utils::quantize;

namespace force_plugin
{
  // Claing resources: If the socket is open, it will be closed when the plugin is destroyed.
  ForcePlugin::~ForcePlugin()
  {
    if (client_fd_ != -1)
      close(client_fd_);
    if (server_fd_ != -1)
      close(server_fd_);

    const char *SOCKET_PATH = "/tmp/force_socket";
    unlink(SOCKET_PATH);

    if (ros_node_)
      RCLCPP_INFO(ros_node_->get_logger(), "Force Plugin shutting down and removing socket.");
  }

  void ForcePlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    rng_ = std::mt19937(std::random_device{}());

    /*=================================================================================
      Create sensor variable from Gazebo sensor_msgs
    ====================================================================================*/
    force_sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ForceTorqueSensor>(_sensor);
    if (!force_sensor_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("ForcePlugin"), "Failed to cast to forceSensor");
      return;
    }

    ros_node_ = gazebo_ros::Node::Get(_sdf, "force_plugin_node");
    /*==================================================================================
      Import parameters from SDF file
    ====================================================================================*/
    noise_stddev_ = _sdf->Get<double>("noise_stddev", 0.0).first;
    min_lin_force_ = _sdf->Get<double>("min_lin_force", 0.0).first;
    max_lin_force_ = _sdf->Get<double>("max_lin_force", 0.0).first;
    min_force_ = _sdf->Get<double>("min_force", 0.0).first;
    max_force_ = _sdf->Get<double>("max_force", 0.0).first;
    resolution_ = _sdf->Get<double>("resolution", 0.0).first;

    // Timer to publish
    if (ros_node_)
      update_timer_ = ros_node_->create_wall_timer(
          std::chrono::milliseconds(50),
          std::bind(&ForcePlugin::OnUpdate, this));

    socket_thread_ = std::thread([this]()
                                 {
      if (initUnixSocketServer())
        acceptUnixSocketClient(); });
  }

  bool ForcePlugin::initUnixSocketServer()
  {
    const char *SOCKET_PATH = "/tmp/force_socket";
    unlink(SOCKET_PATH);

    server_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd_ < 0)
    {
      perror("socket");
      return false;
    }

    memset(&socket_addr_, 0, sizeof(socket_addr_));
    socket_addr_.sun_family = AF_UNIX;
    strncpy(socket_addr_.sun_path, SOCKET_PATH, sizeof(socket_addr_.sun_path) - 1);

    if (bind(server_fd_, (struct sockaddr *)&socket_addr_, sizeof(socket_addr_)) == -1)
    {
      perror("bind");
      close(server_fd_);
      server_fd_ = -1;
      return false;
    }

    if (listen(server_fd_, 1) == -1)
    {
      perror("listen");
      close(server_fd_);
      server_fd_ = -1;
      return false;
    }

    return true;
  }

  void ForcePlugin::acceptUnixSocketClient()
  {
    socklen_t addr_len = sizeof(socket_addr_);
    client_fd_ = accept(server_fd_, (struct sockaddr *)&socket_addr_, &addr_len);
    if (client_fd_ == -1)
    {
      perror("accept");
      return;
    }

    socket_ready_ = true;
    if (ros_node_)
      RCLCPP_INFO(ros_node_->get_logger(), "force socket client connected.");
  }

  void ForcePlugin::sendToSocket(const double &force)
  {
    if (!socket_ready_)
      return;

    /*=======================================TODO=======================================
      Codding the information like real sensor does
      Example
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(6)
          << acc.X() << "\n";

      std::string msg = oss.str();
      send(client_fd_, msg.c_str(), msg.size(), 0);
    ====================================================================================*/
  }

  void ForcePlugin::OnUpdate()
  {
    if (!force_sensor_)
      return;

    /*==================================================================================
      Transduction Stage: Connect to simulated force
    ====================================================================================*/
    ignition::math::Vector3d force = force_sensor_->Force();
    double perpendicular_force = std::abs(force.Z());

    /*==================================================================================
      Error model Stage: realistic imperfections effects
    ====================================================================================*/
    perpendicular_force += gaussian_noise(noise_stddev_);
    perpendicular_force = clipping_values(perpendicular_force, max_force_, min_force_);
    // Non-lienar zone
    if (perpendicular_force < min_lin_force_ || perpendicular_force > max_lin_force_)
    {
      // add more noise
      perpendicular_force += gaussian_noise(5.0 * noise_stddev_);
      // add non-linearty -> 0.02X²
      perpendicular_force = perpendicular_force * perpendicular_force * 0.02f;
    }
    perpendicular_force = quantize(perpendicular_force, resolution_);

    /*=======================================TODO=======================================
      Codification Stage: Encoding to communication protocol
    ====================================================================================*/
    sendToSocket(perpendicular_force);
  }

  GZ_REGISTER_SENSOR_PLUGIN(ForcePlugin)

} // namespace force_plugin