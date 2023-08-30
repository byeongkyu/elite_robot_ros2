#ifndef ELITE_ROBOT_HARDWARE_INTERFACE__ELITE_ROBOT_SYHSTEM_HPP_
#define ELITE_ROBOT_HARDWARE_INTERFACE__ELITE_ROBOT_SYHSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/math/constants/constants.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "hardware_interface/visibility_control.h"

namespace elite_robot_hardware
{

using boost::asio::ip::tcp;
using boost::asio::deadline_timer;

class EliteRobotSystemHardware : public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(EliteRobotSystemHardware)

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_efforts_;

        boost::asio::io_service io_context_dashboard_;
        boost::asio::io_service io_context_controller_;
        boost::asio::io_service io_context_rtsi_;

        std::shared_ptr<tcp::socket> dashboard_socket_;
        std::shared_ptr<tcp::socket> controller_socket_;
        std::shared_ptr<tcp::socket> rtsi_socket_;
};
} // namespace elite_robot_hardware

#endif //ELITE_ROBOT_HARDWARE_INTERFACE__ELITE_ROBOT_SYHSTEM_HPP_