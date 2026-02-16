#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <string>
#include <memory>

using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

class ARMHardware : public hardware_interface::SystemInterface
{
public:
    return_type configure(const hardware_interface::HardwareInfo & info) override {
        joint_names_.clear();
        for (const auto & joint : info.joints) {
            joint_names_.push_back(joint.name);
        }

        positions_.resize(joint_names_.size(), 0.0);
        velocities_.resize(joint_names_.size(), 0.0);
        commands_.resize(joint_names_.size(), 0.0);

        RCLCPP_INFO(rclcpp::get_logger("ARMHardware"), "Configured %zu joints.", joint_names_.size());
        return return_type::OK;
    }

    std::vector<StateInterface> export_state_interfaces() override {
        std::vector<StateInterface> state_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            state_interfaces.emplace_back(joint_names_[i], "position", &positions_[i]);
            state_interfaces.emplace_back(joint_names_[i], "velocity", &velocities_[i]);
        }
        return state_interfaces;
    }

    std::vector<CommandInterface> export_command_interfaces() override {
        std::vector<CommandInterface> command_interfaces;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            command_interfaces.emplace_back(joint_names_[i], "position", &commands_[i]);
        }
        return command_interfaces;
    }

    return_type start() override {
        RCLCPP_INFO(rclcpp::get_logger("ARMHardware"), "Starting hardware...");
        // -----------------------------
        // Initialize motor drivers here
        // For example: open serial/CAN/Ethernet connections
        // Example:
        // for(auto & motor : motors_) motor.connect();
        // -----------------------------
        return return_type::OK;
    }

    return_type stop() override {
        RCLCPP_INFO(rclcpp::get_logger("ARMHardware"), "Stopping hardware...");
        // -----------------------------
        // Safely close motor connections
        // Example:
        // for(auto & motor : motors_) motor.disconnect();
        // -----------------------------
        return return_type::OK;
    }

    return_type read() override {
        // -----------------------------
        // Read each motor's encoder
        // Example:
        // positions_[i] = motors_[i].get_position();
        // velocities_[i] = motors_[i].get_velocity();
        // -----------------------------
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // Placeholder: Replace with real motor read
            positions_[i] = positions_[i];  // e.g., motors_[i].read_position();
            velocities_[i] = 0.0;           // e.g., motors_[i].read_velocity();
        }
        return return_type::OK;
    }

    return_type write() override {
        // -----------------------------
        // Send commands_[i] to each motor
        // Example:
        // motors_[i].set_position(commands_[i]);
        // -----------------------------
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            // Placeholder: Replace with real motor write
            // motors_[i].write_position(commands_[i]);
        }
        return return_type::OK;
    }

private:
    std::vector<std::string> joint_names_;
    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<double> commands_;

    // Example motor interface objects
    // Replace with your motor driver class
    // std::vector<MotorDriver> motors_;
};

// Register as plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ARMHardware, hardware_interface::SystemInterface)
