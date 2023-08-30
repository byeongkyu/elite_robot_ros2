#include "elite_robot_hardware_interface/elite_robot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <time.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace elite_robot_hardware
{
    hardware_interface::CallbackReturn EliteRobotSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
    {
        // Get info parameters from URDF
        if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Name: %s", info_.name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Number of Joints %zu", info_.joints.size());

        // Initialize hardware_interface
        hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),  joint.command_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 3)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
                    joint.state_interfaces.size()
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY
                );
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EliteRobotSystemHardware"),
                    "Joint '%s' have '%s' as third state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT
                );
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        dashboard_socket_ = std::make_shared<tcp::socket>(io_context_dashboard_);
        controller_socket_ = std::make_shared<tcp::socket>(io_context_controller_);
        rtsi_socket_ = std::make_shared<tcp::socket>(io_context_rtsi_);

        auto robot_ip = info_.hardware_parameters["robot_ip"];
        auto dashboard_port = stoi(info_.hardware_parameters["dashboard_port"]);
        auto controller_port = stoi(info_.hardware_parameters["controller_port"]);
        auto rtsi_port = stoi(info_.hardware_parameters["rtsi_port"]);

        dashboard_socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), dashboard_port));
        controller_socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), controller_port));
        rtsi_socket_->connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), rtsi_port));

        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Successfully initialized!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> EliteRobotSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> EliteRobotSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn EliteRobotSystemHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Elite Robot hardware is activating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn EliteRobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
    {
        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Elite Robot hardware is deactivating ...please wait...");

        RCLCPP_INFO(rclcpp::get_logger("EliteRobotSystemHardware"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type EliteRobotSystemHardware::read(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("EliteRobotSystemHardware"), "READ");

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type EliteRobotSystemHardware::write(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("EliteRobotSystemHardware"), "WRITE");

        return hardware_interface::return_type::OK;
    }
} // end elite_robot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(elite_robot_hardware::EliteRobotSystemHardware, hardware_interface::SystemInterface)