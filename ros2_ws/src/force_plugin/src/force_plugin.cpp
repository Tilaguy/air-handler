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
    socket_ready_ = false;

    if (client_fd_ != -1)
      close(client_fd_);
    if (server_fd_ != -1)
      close(server_fd_);
    if (request_thread_.joinable())
      request_thread_.join();

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

    request_thread_ = std::thread(&ForcePlugin::handleClientRequest, this);
  }

  void ForcePlugin::handleClientRequest()
  {
    while (true)
    {
      uint8_t request;
      ssize_t bytes_received = recv(client_fd_, &request, sizeof(request), 0);

      if (bytes_received <= 0)
      {
        perror("recv");
        RCLCPP_WARN(ros_node_->get_logger(), "Client disconnected or socket error.");
        socket_ready_ = false;
        break;
      }

      switch (request)
      {
      case 0x28:
        sendToSocket();
        break;
      default:
        RCLCPP_WARN(ros_node_->get_logger(), "Unknown command received: 0x%02X", request);
      }
    }
  }

  void ForcePlugin::sendToSocket()
  {
    if (!socket_ready_)
      return;

    /*==================================================================================
      Codding the information like real sensor does
    ====================================================================================*/
    uint16_t output_min = static_cast<uint16_t>(16384 * 0.1f);
    uint16_t output_max = static_cast<uint16_t>(16384 * 0.9f);

    uint16_t digital_output = static_cast<uint16_t>(
        ((perpendicular_force_ / max_force_) * (output_max - output_min)) + output_min);

    uint8_t status_bits = 0b00 << 6;

    uint8_t byte1 = status_bits | ((digital_output >> 8) & 0x3F); // bits 13–8
    uint8_t byte2 = digital_output & 0xFF;

    uint8_t buffer[2] = {byte1, byte2};
    send(client_fd_, buffer, sizeof(buffer), 0);
  }

  void ForcePlugin::OnUpdate()
  {
    if (!force_sensor_)
      return;

    /*==================================================================================
      Transduction Stage: Connect to simulated force
    ====================================================================================*/
    ignition::math::Vector3d force = force_sensor_->Force();
    RCLCPP_WARN(ros_node_->get_logger(), "%0.1f, %0.1f, %0.1f", force.X(), force.Y(), force.Z());
    perpendicular_force_ = std::abs(force.Z());

    /*==================================================================================
      Error model Stage: realistic imperfections effects
    ====================================================================================*/
    perpendicular_force_ += gaussian_noise(noise_stddev_);
    perpendicular_force_ = clipping_values(perpendicular_force_, max_force_, min_force_);
    // Non-lienar zone
    if (perpendicular_force_ < min_lin_force_ || perpendicular_force_ > max_lin_force_)
    {
      // add more noise
      perpendicular_force_ += gaussian_noise(5.0 * noise_stddev_);
      // add non-linearty -> 0.02X²
      perpendicular_force_ = perpendicular_force_ * perpendicular_force_ * 0.02f;
    }
    perpendicular_force_ = quantize(perpendicular_force_, resolution_);

    /*==================================================================================
      Codification Stage: Encoding to communication protocol
    ====================================================================================*/
    // sendToSocket(perpendicular_force_);
  }

  GZ_REGISTER_SENSOR_PLUGIN(ForcePlugin)

} // namespace force_plugin