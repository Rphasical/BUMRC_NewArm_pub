#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <vector>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <sys/ioctl.h>
#include <cmath>

namespace arm_hardware
{

class ArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardware)

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
      const rclcpp::Time &,
      const rclcpp::Duration &) override;

  hardware_interface::return_type write(
      const rclcpp::Time &,
      const rclcpp::Duration &) override;

private:
  void request_closed_loop();

  static constexpr double TWO_PI = 2.0 * M_PI;

  int socket_{-1};
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
};

}  // namespace arm_hardware