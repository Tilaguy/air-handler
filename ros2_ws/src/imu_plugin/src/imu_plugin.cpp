#include "imu_plugin/imu_plugin.hpp"

#include <ignition/msgs/imu.pb.h>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/Name.hh>

#include "sensor_utils/noise_utils.hpp"
using sensor_utils::clipping_values;
using sensor_utils::gaussian_drift;
using sensor_utils::gaussian_noise;
using sensor_utils::quantize;

#include <iomanip>
#include <sstream>
#include <memory>

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

  void ImuPlugin::Configure(const ignition::gazebo::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            ignition::gazebo::EntityComponentManager &_ecm,
                            ignition::gazebo::EventManager &_eventMgr)
  {
    entity_ = _entity;

    int64_t ts = _sdf->Get<double>("update_rate", 50).first;
    update_interval_ = std::chrono::milliseconds(ts);

    rng_ = std::mt19937(std::random_device{}());

    // Initialize ROS node
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    ros_node_ = std::make_shared<rclcpp::Node>("imu_plugin_node");

    // Import parameters from SDF
    gyro_noise_stddev_ = _sdf->Get<double>("gyro_noise_stddev", 0.0).first;
    gyro_lim_ = _sdf->Get<double>("gyro_lim", 0.0).first;
    gyro_drift_stddev_ = _sdf->Get<double>("gyro_drift_stddev", 0.0).first;
    gyro_resolution_ = _sdf->Get<double>("gyro_resolution", 0.0).first;
    accel_noise_stddev_ = _sdf->Get<double>("accel_noise_stddev", 0.0).first;
    accel_lim_ = _sdf->Get<double>("accel_lim", 0.0).first;
    accel_drift_stddev_ = _sdf->Get<double>("accel_drift_stddev", 0.0).first;
    accel_resolution_ = _sdf->Get<double>("accel_resolution", 0.0).first;

    if (!_ecm.EntityHasComponentType(entity_, ignition::gazebo::components::ImuSensor::typeId))
    {
      _ecm.CreateComponent(entity_, ignition::gazebo::components::ImuSensor());
    }

    // Habilitar los componentes que necesitamos
    _ecm.SetComponentData<ignition::gazebo::components::ImuAngularVelocity>(entity_, {true});
    _ecm.SetComponentData<ignition::gazebo::components::ImuLinearAcceleration>(entity_, {true});

    // Timer to publish
    update_timer_ = ros_node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ImuPlugin::OnUpdate, this));

    // Start socket server in separate thread
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

  void ImuPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
                            ignition::gazebo::EntityComponentManager &_ecm)
  {
    if (_info.paused)
    {
      return;
    }

    /*=============================================
      Transduction Stage: Connect to simulated IMU
      =============================================*/
    auto imu_ang_vel = _ecm.Component<ignition::gazebo::components::ImuAngularVelocity>(entity_);
    auto imu_lin_acc = _ecm.Component<ignition::gazebo::components::ImuLinearAcceleration>(entity_);

    if (imu_ang_vel && imu_lin_acc)
    {
      angular_velocity_ = imu_ang_vel->Data();
      linear_acceleration_ = imu_lin_acc->Data();
    }
  }

  void ImuPlugin::OnUpdate()
  {

    auto now = std::chrono::steady_clock::now();
    if (first_update_)
    {
      last_update_time_ = now;
      first_update_ = false;
      return;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_update_time_);
    if (elapsed < update_interval_)
    {
      return;
    }

    last_update_time_ = now;

    if (!angular_velocity_.has_value() || !linear_acceleration_.has_value())
    {
      RCLCPP_WARN(ros_node_->get_logger(), "No IMU data available yet");
      return;
    }

    ignition::math::Vector3d angular_vel = angular_velocity_.value();
    ignition::math::Vector3d linear_acc = linear_acceleration_.value();

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

  IGNITION_ADD_PLUGIN(ImuPlugin,
                      ignition::gazebo::System,
                      ImuPlugin::ISystemConfigure,
                      ImuPlugin::ISystemPreUpdate)

} // namespace imu_plugin
