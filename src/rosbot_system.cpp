#include "rosbot_hardware_interfaces/rosbot_system.hpp"

#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rosbot_hardware_interfaces
{
CallbackReturn RosbotSystem::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  node_ = std::make_shared<rclcpp::Node>("hardware_node");

  try
  {
    node_->declare_parameter<double>("wheel_radius");
    node_->declare_parameter<std::vector<std::string>>("velocity_command_joint_order");
  }
  catch (rclcpp::exceptions::UninitializedStaticallyTypedParameterException& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RosbotSystem"), "Required parameter missing: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (auto& j : info_.joints)
  {
    RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Joint '%s' found", j.name.c_str());

    pos_state_[j.name] = std::numeric_limits<double>::quiet_NaN();
    vel_state_[j.name] = std::numeric_limits<double>::quiet_NaN();
    vel_commands_[j.name] = std::numeric_limits<double>::quiet_NaN();
  }

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> RosbotSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_state_[info_.joints[i].name]));
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_state_[info_.joints[i].name]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> RosbotSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_commands_[info_.joints[i].name]));
  }

  return command_interfaces;
}

CallbackReturn RosbotSystem::on_configure(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Configuring...");

  motor_command_publisher_ = node_->create_publisher<Float32MultiArray>("~/motors_cmd", rclcpp::SensorDataQoS());
  realtime_motor_command_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<Float32MultiArray>>(motor_command_publisher_);

  motor_state_subscriber_ =
      node_->create_subscription<JointState>("~/motors_response", rclcpp::SensorDataQoS(),
                                             std::bind(&RosbotSystem::motor_state_cb, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("RosbotSystem"), "Successfully configured");

  executor_.add_node(node_);
  executor_thread_ =
      std::make_unique<std::thread>(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, &executor_));

  node_->get_parameter("wheel_radius", wheel_radius_);
  node_->get_parameter("velocity_command_joint_order", velocity_command_joint_order_);

  return CallbackReturn::SUCCESS;
}

void RosbotSystem::motor_state_cb(const std::shared_ptr<JointState> msg)
{
  RCLCPP_DEBUG(node_->get_logger(), "Received motors response");

  if (!subscriber_is_active_)
  {
    RCLCPP_WARN(node_->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  received_motor_state_msg_ptr_.set(std::move(msg));
}

CallbackReturn RosbotSystem::on_activate(const rclcpp_lifecycle::State&)
{
  // method where hardware “power” is enabled.

  for (auto& j : info_.joints)
  {
    pos_state_[j.name] = 0.0;
    vel_state_[j.name] = 0.0;
    vel_commands_[j.name] = 0.0;
  }

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(node_->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_deactivate(const rclcpp_lifecycle::State&)
{
  // method, which does the opposite of on_activate.
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_cleanup(const rclcpp_lifecycle::State&)
{
  // method, which does the opposite of on_configure.
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_shutdown(const rclcpp_lifecycle::State&)
{
  // method where hardware is shutdown gracefully.
  return CallbackReturn::SUCCESS;
}

CallbackReturn RosbotSystem::on_error(const rclcpp_lifecycle::State&)
{
  // method where different errors from all states are handled.
  return CallbackReturn::SUCCESS;
}

return_type RosbotSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::shared_ptr<JointState> motor_state;
  received_motor_state_msg_ptr_.get(motor_state);

  RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"), "Reading motors state");

  if (!motor_state)
  {
    RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"), "Feedback message from motors wasn't received yet");
    // returning ERROR causes controller to freeze
    return return_type::OK;
  }

  for (auto i = 0u; i < motor_state->name.size(); i++)
  {
    if (pos_state_.find(motor_state->name[i]) == pos_state_.end() ||
        vel_state_.find(motor_state->name[i]) == vel_state_.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("RosbotSystem"), "Position or velocity feedback not found for joint %s",
                   motor_state->name[i].c_str());
      return return_type::ERROR;
    }

    pos_state_[motor_state->name[i]] = motor_state->position[i];
    vel_state_[motor_state->name[i]] = motor_state->velocity[i] * wheel_radius_;

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"), "Position feedback: %f, velocity feedback: %f",
                 pos_state_[motor_state->name[i]], vel_state_[motor_state->name[i]]);
  }

  return return_type::OK;
}

return_type RosbotSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  if (realtime_motor_command_publisher_->trylock())
  {
    auto& motor_command = realtime_motor_command_publisher_->msg_;
    motor_command.data.clear();

    RCLCPP_DEBUG(rclcpp::get_logger("RosbotSystem"), "Wrtiting motors cmd message");

    for (auto const& joint : velocity_command_joint_order_)
    {
      motor_command.data.push_back(vel_commands_[joint]);
    }

    realtime_motor_command_publisher_->unlockAndPublish();
  }

  return return_type::OK;
}

}  // namespace rosbot_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rosbot_hardware_interfaces::RosbotSystem, hardware_interface::SystemInterface)