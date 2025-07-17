#include "imu_plugin/imu_plugin.hpp"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "sensor_utils/noise_utils.hpp"
using sensor_utils::clipping_values;
using sensor_utils::gaussian_drift;
using sensor_utils::gaussian_noise;
using sensor_utils::quantize;

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

    // Import parameters from SDF
    gyro_noise_stddev_ = _sdf->Get<double>("gyro_noise_stddev", 0.0).first;
    gyro_lim_ = _sdf->Get<double>("gyro_lim", 0.0).first;
    gyro_drift_stddev_ = _sdf->Get<double>("gyro_drift_stddev", 0.0).first;
    gyro_resolution_ = _sdf->Get<double>("gyro_resolution", 0.0).first;
    accel_noise_stddev_ = _sdf->Get<double>("accel_noise_stddev", 0.0).first;
    accel_lim_ = _sdf->Get<double>("accel_lim", 0.0).first;
    accel_drift_stddev_ = _sdf->Get<double>("accel_drift_stddev", 0.0).first;
    accel_resolution_ = _sdf->Get<double>("accel_resolution", 0.0).first;

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

  void ImuPlugin::sendToSocket(const ignition::math::Vector3d &acc, const ignition::math::Vector3d &gyro)
  {
    if (!socket_ready_)
      return;

    int16_t acc_x_raw = static_cast<int16_t>(acc.X());
    int16_t acc_y_raw = static_cast<int16_t>(acc.Y());
    int16_t acc_z_raw = static_cast<int16_t>(acc.Z());
    int16_t gyro_x_raw = static_cast<int16_t>(gyro.X());
    int16_t gyro_y_raw = static_cast<int16_t>(gyro.Y());
    int16_t gyro_z_raw = static_cast<int16_t>(gyro.Z());

    uint8_t buffer[15];
    buffer[0] = 0xA5; // header
    memcpy(&buffer[1], &acc_x_raw, sizeof(int16_t));
    memcpy(&buffer[3], &acc_y_raw, sizeof(int16_t));
    memcpy(&buffer[5], &acc_z_raw, sizeof(int16_t));
    memcpy(&buffer[7], &gyro_x_raw, sizeof(int16_t));
    memcpy(&buffer[9], &gyro_y_raw, sizeof(int16_t));
    memcpy(&buffer[11], &gyro_z_raw, sizeof(int16_t));

    // Simplehe cksum (XOR of every bytes except the last one)
    uint8_t checksum = 0;
    for (int i = 0; i < 13; ++i)
      checksum ^= buffer[i];
    buffer[13] = checksum;

    // Send by socket
    send(client_fd_, buffer, sizeof(buffer), 0);
  }

  void ImuPlugin::OnUpdate()
  {
    if (!imu_sensor_)
      return;

    /*=============================================
      Transduction Stage: Connect to simulated IMU
      =============================================*/
    ignition::math::Vector3d linear_acc = imu_sensor_->LinearAcceleration();
    ignition::math::Vector3d angular_vel = imu_sensor_->AngularVelocity();

    /*===================================================
      Error model Stage: realistic imperfections effects
      ===================================================*/
    angular_vel.X() += gaussian_noise(gyro_noise_stddev_);
    angular_vel.Y() += gaussian_noise(gyro_noise_stddev_);
    angular_vel.Z() += gaussian_noise(gyro_noise_stddev_);
    angular_vel.X() = clipping_values(angular_vel.X(), gyro_lim_, -gyro_lim_);
    angular_vel.Y() = clipping_values(angular_vel.Y(), gyro_lim_, -gyro_lim_);
    angular_vel.Z() = clipping_values(angular_vel.Z(), gyro_lim_, -gyro_lim_);
    angular_vel.X() = gaussian_drift(angular_vel.X(), gyro_drift_stddev_, gyro_bias_acum);
    angular_vel.Y() = gaussian_drift(angular_vel.Y(), gyro_drift_stddev_, gyro_bias_acum);
    angular_vel.Z() = gaussian_drift(angular_vel.Z(), gyro_drift_stddev_, gyro_bias_acum);
    angular_vel.X() = quantize(angular_vel.X(), gyro_resolution_);
    angular_vel.Y() = quantize(angular_vel.Y(), gyro_resolution_);
    angular_vel.Z() = quantize(angular_vel.Z(), gyro_resolution_);

    linear_acc.X() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Y() += gaussian_noise(accel_noise_stddev_);
    linear_acc.Z() += gaussian_noise(accel_noise_stddev_);
    linear_acc.X() = clipping_values(linear_acc.X(), accel_lim_, -accel_lim_);
    linear_acc.Y() = clipping_values(linear_acc.Y(), accel_lim_, -accel_lim_);
    linear_acc.Z() = clipping_values(linear_acc.Z(), accel_lim_, -accel_lim_);
    linear_acc.X() = gaussian_drift(linear_acc.X(), accel_drift_stddev_, accel_bias_acum);
    linear_acc.Y() = gaussian_drift(linear_acc.Y(), accel_drift_stddev_, accel_bias_acum);
    linear_acc.Z() = gaussian_drift(linear_acc.Z(), accel_drift_stddev_, accel_bias_acum);
    linear_acc.X() = quantize(linear_acc.X(), accel_resolution_);
    linear_acc.Y() = quantize(linear_acc.Y(), accel_resolution_);
    linear_acc.Z() = quantize(linear_acc.Z(), accel_resolution_);

    /*=======================================================
      Codification Stage: Encoding to communication protocol
      =======================================================*/
    sendToSocket(linear_acc, angular_vel);
  }

  GZ_REGISTER_SENSOR_PLUGIN(ImuPlugin)

} // namespace imu_plugin
