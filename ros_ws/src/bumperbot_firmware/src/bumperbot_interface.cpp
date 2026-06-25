#include "bumperbot_firmware/bumperbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace bumperbot_firmware
{
BumperbotInterface::BumperbotInterface()
{
}


BumperbotInterface::~BumperbotInterface()
{
  if (STM32_.IsOpen())
  {
    try
    {
      STM32_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn BumperbotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("BumperbotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);
  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BumperbotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> BumperbotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a velocity Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn BumperbotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  velocity_commands_ = { 0.0, 0.0 };
  position_states_ = { 0.0, 0.0 };
  velocity_states_ = { 0.0, 0.0 };

  try
  {
    STM32_.Open(port_);
    STM32_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn BumperbotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Stopping robot hardware ...");

  if (STM32_.IsOpen())
  {
    try
    {
      STM32_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("BumperbotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type BumperbotInterface::read(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  // Interpret the string
  if(STM32_.IsDataAvailable()){

    //auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;
    STM32_.ReadLine(message);

    if(message.empty() || message[0] != '@') {
      return hardware_interface::return_type::OK;
    }

    message = message.substr(1);  // togli '@'
    std::stringstream ss(message);
    std::string res;
    while(std::getline(ss, res, ',')){

      // rimuovi sporcizia
      res.erase(std::remove(res.begin(), res.end(), '\r'), res.end());
      res.erase(std::remove(res.begin(), res.end(), '\n'), res.end());

      // skip stringhe vuote o troppo corte
      if(res.size() < 2) continue;

      char wheel = res.at(0);

      double value;
      try {
        value = std::stod(res.substr(1));
      } catch(...) {
        RCLCPP_WARN(rclcpp::get_logger("BumperbotInterface"),
                    "Parse error on: '%s'", res.c_str());
        continue;
      }

      if(wheel == 'r'){
        velocity_states_.at(1) = value;
        //position_states_.at(1) += value * dt;
      }
      else if(wheel == 'l'){
        velocity_states_.at(0) = value;
        //position_states_.at(0) += value * dt;
      }
      
      else if(wheel == 'R'){
        position_states_.at(1) = value;
      }
      else if(wheel == 'L'){
        position_states_.at(0) = value;
      }
        
      /*DEBUG*/
      /*
      RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
              "Vr: %f, Vl: %f", velocity_states_[1], velocity_states_[0]);  
      //last_run_ = rclcpp::Clock().now();
      */
    }
  }
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type BumperbotInterface::write(const rclcpp::Time &,
                                                          const rclcpp::Duration &)
{
  std::stringstream message_stream;

  message_stream << std::fixed << std::setprecision(2)
    << "r" << velocity_commands_.at(1)
    << ",l" << velocity_commands_.at(0)
    << "\n";

  try
  {
    STM32_.Write(message_stream.str());
  }
  catch (...)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("BumperbotInterface"),
                        "Error sending: " << message_stream.str());
    return hardware_interface::return_type::ERROR;
  }
   /*DEBUG*/
   /*
    RCLCPP_INFO(rclcpp::get_logger("BumperbotInterface"),
            "R: %f, L: %f", velocity_commands_[1], velocity_commands_[0]);
            */
  return hardware_interface::return_type::OK;
}
}  // namespace bumperbot_firmware

PLUGINLIB_EXPORT_CLASS(bumperbot_firmware::BumperbotInterface, hardware_interface::SystemInterface)