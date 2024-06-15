#include "sew_agv_drivers/sew_agv_hardware_interface.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

SewAgvHardwareInterface::~SewAgvHardwareInterface() {
  //virtual destructor enables proper cleanup in polymorphic class hierarchies by ensuring the correct destructor is invoked for objects of derived classes when deleted through a base class pointer
  on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn SewAgvHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read parameters from the ros2_control.xacro file
  cfg_.agvHost = info_.hardware_parameters["agvHost"];
  cfg_.agvPort = std::stoi(info_.hardware_parameters["agvPort"]);
  cfg_.localIp = info_.hardware_parameters["localIp"];
  cfg_.localPort = std::stoi(info_.hardware_parameters["localPort"]);
  
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SewAgvHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SewAgvHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SewAgvHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SewAgvHardwareInterface"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SewAgvHardwareInterface"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}



// export_state_interfaces and export_command_interfaces to tell ros2_control what the hardware interface has accessable
std::vector<hardware_interface::StateInterface> SewAgvHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of state interfaces to controller manager finished sucessfully");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SewAgvHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of command interfaces to controller manager finished sucessfully");

  return command_interfaces;
}



return_type SewAgvHardwareInterface::on_activate()
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Activating ...please wait...");

  // Connect to AGV
  bool connected = agv_endpoint_.connect(cfg_.agvHost, cfg_.agvPort, cfg_.localIp, cfg_.localPort);

  // check if connection was successful
  if (connected) {
      RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully connected to AGV.");
      status_ = hardware_interface::status::STARTED;
  } else {
      RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to connect to AGV.");
      status_ = hardware_interface::status::STOPPED;
      return return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn SewAgvHardwareInterface::on_deactivate(
const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Deactivating ...please wait...");

  // Disconnect from AGV
  agv_endpoint_.disconnect();

  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Successfully deactivated!");

  status_ = hardware_interface::status::STOPPED;

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::return_type SewAgvHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Writing...");

  // Connect to the AGV endpoint if not connected
  static AgvEndpoint agvEndpoint;
  if (!agvEndpoint.connected) {
    if (!agvEndpoint.connect("AGV_IP", AGV_PORT, "LOCAL_IP", LOCAL_PORT)) {
      RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to connect to AGV");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Prepare the message to be sent to the AGV
  AgvTxMsg agvTxMsg;
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    agvTxMsg.setCommand(i, hw_commands_[i]);
    RCLCPP_INFO(
      rclcpp::get_logger("SewAgvHardwareInterface"), "Setting command %.5f for '%s'", hw_commands_[i],
      info_.joints[i].name.c_str());
  }

  // Send the message
  agvEndpoint.setMsg(agvTxMsg);
  try {
    agvEndpoint.sendMsg();
    RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Joints successfully written!");
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to send message: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  SewAgvHardwareInterface,
  hardware_interface::SystemInterface
)