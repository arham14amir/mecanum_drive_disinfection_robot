#ifndef ROBOT_HARDWARE_INTERFACE_HPP
#define ROBOT_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware_interface
{
class RealRobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RealRobotSystem)

  // Lifecycle Node Interface
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial Port handling
  std::string device_name_;
  int baud_rate_;
  int serial_conn_; 
  
  // Robot params
  double enc_counts_per_rev_;
  
  // --- TIMER VARIABLE ---
  // Stores the time of the last serial message to control the rate
  rclcpp::Time last_send_time_; 

  // Store values for the 4 wheels
  // 0=FL, 1=FR, 2=RL, 3=RR
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  
  // Helper to send/receive data
  void send_serial_command(double w1, double w2, double w3, double w4);
  void read_serial_feedback();
};

}  // namespace robot_hardware_interface

#endif  // ROBOT_HARDWARE_INTERFACE_HPP