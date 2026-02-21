#include "arm_hardware/arm_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_hardware
{

hardware_interface::CallbackReturn
ArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info.joints)
    joint_names_.push_back(joint.name);

  size_t n = joint_names_.size();
  positions_.assign(n, 0.0);
  velocities_.assign(n, 0.0);
  commands_.assign(n, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("ArmHardware"),
              "Initialized %zu joints", n);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmHardware::on_configure(const rclcpp_lifecycle::State &)
{
  socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0)
    return hardware_interface::CallbackReturn::ERROR;

  struct ifreq ifr;
  std::strcpy(ifr.ifr_name, "can0");
  ioctl(socket_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    return hardware_interface::CallbackReturn::ERROR;

  RCLCPP_INFO(rclcpp::get_logger("ArmHardware"),
              "CAN socket bound");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  request_closed_loop();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (socket_ >= 0)
    close(socket_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    states.emplace_back(joint_names_[i],
                        hardware_interface::HW_IF_POSITION,
                        &positions_[i]);

    states.emplace_back(joint_names_[i],
                        hardware_interface::HW_IF_VELOCITY,
                        &velocities_[i]);
  }

  return states;
}

std::vector<hardware_interface::CommandInterface>
ArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    cmds.emplace_back(joint_names_[i],
                      hardware_interface::HW_IF_POSITION,
                      &commands_[i]);
  }

  return cmds;
}

hardware_interface::return_type
ArmHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame;
  int nbytes = recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT);
  if (nbytes <= 0)
    return hardware_interface::return_type::OK;

  uint8_t axis_id = frame.can_id >> 5;
  uint8_t cmd_id  = frame.can_id & 0x1F;

  if (cmd_id == 0x09 && axis_id < positions_.size())
  {
    float pos_turns;
    float vel_turns;

    std::memcpy(&pos_turns, &frame.data[0], 4);
    std::memcpy(&vel_turns, &frame.data[4], 4);

    positions_[axis_id]  = pos_turns * TWO_PI;
    velocities_[axis_id] = vel_turns * TWO_PI;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
ArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < commands_.size(); ++i)
  {
    float turns = commands_[i] / TWO_PI;

    struct can_frame frame;
    frame.can_id  = (i << 5) | 0x0C;
    frame.can_dlc = 8;

    float vel_ff = 0.0f;
    int16_t torque_ff = 0;

    std::memcpy(&frame.data[0], &turns, 4);
    std::memcpy(&frame.data[4], &vel_ff, 2);
    std::memcpy(&frame.data[6], &torque_ff, 2);

    send(socket_, &frame, sizeof(frame), 0);
  }

  return hardware_interface::return_type::OK;
}

void ArmHardware::request_closed_loop()
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    struct can_frame frame;
    frame.can_id  = (i << 5) | 0x07;
    frame.can_dlc = 4;

    uint32_t state = 8;
    std::memcpy(frame.data, &state, 4);

    send(socket_, &frame, sizeof(frame), 0);
  }
}

}  // namespace arm_hardware

PLUGINLIB_EXPORT_CLASS(
  arm_hardware::ArmHardware,
  hardware_interface::SystemInterface)