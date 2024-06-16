#include "sew_agv_drivers/sew_agv_hardware_interface.hpp"

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
  cfg_.agv_ip = info_.hardware_parameters["agv_ip"];
  cfg_.agv_port = std::stoi(info_.hardware_parameters["agv_port"]);
  cfg_.local_ip = info_.hardware_parameters["local_ip"];
  cfg_.local_port = std::stoi(info_.hardware_parameters["local_port"]);
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.wheel_separation_ = std::stod(info_.hardware_parameters["wheel_separation"]);
  cfg_.wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);

  // set the wheels parameters
  wheels_.set(cfg_.left_wheel_name, cfg_.right_wheel_name, cfg_.wheel_separation_, cfg_.wheel_radius_);
  
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

  // link the hardware interface with the variables
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.l_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.l_wheel_.name, hardware_interface::HW_IF_POSITION, &wheels_.l_wheel_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.r_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_.r_wheel_.name, hardware_interface::HW_IF_POSITION, &wheels_.r_wheel_.pos));

  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of state interfaces to controller manager finished sucessfully");

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SewAgvHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_.l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.l_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_.r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &wheels_.r_wheel_.cmd));


  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Export of command interfaces to controller manager finished sucessfully");

  return command_interfaces;
}



return_type SewAgvHardwareInterface::on_activate()
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Activating ...please wait...");

  // Connect to AGV
  bool connected = agv_endpoint_.connect(cfg_.agv_ip, cfg_.agv_port, cfg_.local_ip, cfg_.local_port);

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



hardware_interface::return_type DiffBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Reading from AGV...");
  // #####################################################################################################
  // TODO: Reading status from AGV using the agvEndpoint and print ros info
  // #####################################################################################################

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type SewAgvHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SewAgvHardwareInterface"), "Writing to AGV...");

  // Connect to the AGV endpoint if not connected
  static AgvEndpoint agvEndpoint;
  if (!agvEndpoint.connected) {
    if (!agvEndpoint.connect("AGV_IP", AGV_PORT, "LOCAL_IP", LOCAL_PORT)) {
      RCLCPP_ERROR(rclcpp::get_logger("SewAgvHardwareInterface"), "Failed to connect to AGV");
      return hardware_interface::return_type::ERROR;
    }
  }


  // #####################################################################################################
  // TODO: Write the commands to the AGV using the agvEndpoint and wheels_to_vel_and_dir
  // #####################################################################################################

  return hardware_interface::return_type::OK;
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  SewAgvHardwareInterface,
  hardware_interface::SystemInterface
)