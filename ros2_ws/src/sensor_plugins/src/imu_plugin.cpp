#include <gz/sim/System.hh> // Nueva cabecera para Ignition Gazebo
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace sensor_plugins
{
  class ImuPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPostUpdate
  {
  public:
    void Configure(const gz::sim::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   gz::sim::EntityComponentManager &_ecm,
                   gz::sim::EventManager &_eventMgr) override
    {
      // Inicializar ROS 2
      if (!rclcpp::ok())
      {
        rclcpp::init(0, nullptr);
      }
      this->ros_node_ = rclcpp::Node::make_shared("imu_plugin");

      // Crear publisher
      this->publisher_ = this->ros_node_->create_publisher<sensor_msgs::msg::Imu>(
          "imu/data", 10);
    }

    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override
    {
      // Publicar datos IMU aquí
      auto msg = sensor_msgs::msg::Imu();
      // ... (llenar con datos del IMU)
      this->publisher_->publish(msg);
    }

  private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  };

  // Registro del plugin
  GZ_ADD_PLUGIN(
      ImuPlugin,
      gz::sim::System,
      ImuPlugin::ISystemConfigure,
      ImuPlugin::ISystemPostUpdate)
}