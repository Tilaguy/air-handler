#include "imu_plugin/imu_plugin.hpp"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sensor_msgs/msg/imu.hpp>

#include "sensor_utils/noise_utils.hpp"
using sensor_utils::gaussian_noise;

#include <iomanip> // std::setprecision
#include <sstream>

namespace imu_plugin
{

  // Claing resources: If the socket is open, it will be closed when the plugin is destroyed.
  ImuPlugin::~ImuPlugin()
  {
    if (client_fd_ != -1)
      close(client_fd_);
    if (server_fd_ != -1)
      close(server_fd_);

    const char *SOCKET_PATH = "/tmp/imu_socket";
    unlink(SOCKET_PATH);

    if (ros_node_)
      RCLCPP_INFO(ros_node_->get_logger(), "IMU Plugin shutting down and removing socket.");
  }

  void ImuPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    rng_ = std::mt19937(std::random_device{}());

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

    // Iniciar servidor socket en hilo aparte
    socket_thread_ = std::thread([this]()
                                 {
      if (initUnixSocketServer())
        acceptUnixSocketClient(); });
  }

  bool ImuPlugin::initUnixSocketServer()
  {
    const char *SOCKET_PATH = "/tmp/imu_socket";
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

  void ImuPlugin::acceptUnixSocketClient()
  {
    socklen_t addr_len = sizeof(socket_addr_);
    client_fd_ = accept(server_fd_, (struct sockaddr *)&socket_addr_, &addr_len);
    if (client_fd_ == -1)
    {
      perror("accept");
      return;
    }

    socket_ready_ = true;
    RCLCPP_INFO(ros_node_->get_logger(), "IMU socket client connected.");
  }

  void ImuPlugin::sendToSocketCSV(const ignition::math::Vector3d &acc, const ignition::math::Vector3d &gyro)
  {
    if (!socket_ready_)
      return;

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6)
        << acc.X() << "," << acc.Y() << "," << acc.Z() << ","
        << gyro.X() << "," << gyro.Y() << "," << gyro.Z() << "\n";

    std::string msg = oss.str();
    send(client_fd_, msg.c_str(), msg.size(), 0);
  }

  void ImuPlugin::OnUpdate()
  {
    if (!imu_sensor_)
      return;

    auto imu_msg = sensor_msgs::msg::Imu();

    /*=============================================
      Transduction Stage: Connect to simulated IMU
      =============================================*/
    ignition::math::Vector3d linear_acc = imu_sensor_->LinearAcceleration();
    ignition::math::Vector3d angular_vel = imu_sensor_->AngularVelocity();
    ignition::math::Quaterniond orientation = imu_sensor_->Orientation();

    /*===================================================
      Error model Stage: realistic imperfections effects
      ===================================================*/
    linear_acc.X() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Y() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Z() += gaussian_noise(accel_noise_stddev_);
    // Uncomment for more realistic effects:
    // angular_vel.X() = clipping_values(angular_vel.X(), 10.0, -10.0);
    // angular_vel.X() = gaussian_drift(angular_vel.X(), bias_acum);
    // linear_acc.X() = hysteresis(linear_acc.X(), prev_val, prev_direction);

    /*=======================================================
      Codification Stage: Encoding to communication protocol
      =======================================================*/
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
    sendToSocketCSV(linear_acc, angular_vel);
  }

  GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

} // namespace imu_plugin
