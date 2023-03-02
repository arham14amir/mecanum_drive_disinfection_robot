#include "robot_hardware_interface/robot_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

// Linux Serial headers
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

namespace robot_hardware_interface
{

hardware_interface::CallbackReturn RealRobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. Read parameters
  device_name_ = info_.hardware_parameters["device"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  enc_counts_per_rev_ = std::stod(info_.hardware_parameters["enc_counts_per_rev"]);

  // 2. Initialize storage vectors
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RealRobotSystem"), "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RealRobotSystem"), "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RealRobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RealRobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RealRobotSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRobotSystem"), "Configuring Serial Port: %s", device_name_.c_str());

  // Open Serial Port
  serial_conn_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_conn_ < 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RealRobotSystem"), "Unable to open serial port %s", device_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Configure Serial Port
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(serial_conn_, &tty) != 0) {
      RCLCPP_FATAL(rclcpp::get_logger("RealRobotSystem"), "Error from tcgetattr");
      return hardware_interface::CallbackReturn::ERROR;
  }
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);
  tty.c_cflag &= ~PARENB; 
  tty.c_cflag &= ~CSTOPB; 
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;     
  tty.c_cflag |= CREAD | CLOCAL; 
  
  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; 
  tty.c_lflag &= ~ECHOE; 
  tty.c_lflag &= ~ISIG; 
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
  
  tcsetattr(serial_conn_, TCSANOW, &tty);
  
  RCLCPP_INFO(rclcpp::get_logger("RealRobotSystem"), "Serial Port Initialized Successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealRobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRobotSystem"), "Activating... Motors Ready!");
  
  // Set initial commands to 0
  for (auto & cmd : hw_commands_) cmd = 0.0;
  for (auto & pos : hw_positions_) pos = 0.0;
  for (auto & vel : hw_velocities_) vel = 0.0;
  
  // Reset Timer
  last_send_time_ = rclcpp::Time(0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RealRobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RealRobotSystem"), "Deactivating... Stopping Motors");
  send_serial_command(0, 0, 0, 0); 
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RealRobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::vector<double> prev_positions = hw_positions_;
  read_serial_feedback();

  double dt = period.seconds();
  
  if (dt > 0.0) {
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      if (!std::isnan(prev_positions[i]) && !std::isnan(hw_positions_[i])) {
         hw_velocities_[i] = (hw_positions_[i] - prev_positions[i]) / dt;
      } else {
         hw_velocities_[i] = 0.0;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RealRobotSystem::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // 1. SAFETY CHECK: Time Source Synchronization
  if (last_send_time_.nanoseconds() == 0 || last_send_time_.get_clock_type() != time.get_clock_type()) 
  {
      last_send_time_ = time;
      return hardware_interface::return_type::OK; 
  }

  // 2. RATE LIMITER (50ms)
  double seconds_since_last_send = (time - last_send_time_).seconds();
  if (seconds_since_last_send >= 0.05) 
  {
      send_serial_command(hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);
      last_send_time_ = time;
  }

  return hardware_interface::return_type::OK;
}

void RealRobotSystem::send_serial_command(double w1, double w2, double w3, double w4)
{
  char buffer[50];
  int len = sprintf(buffer, "v %.2f %.2f %.2f %.2f\n", w1, w2, w3, w4);
  ::write(serial_conn_, buffer, len);
}

// ================= FINAL FIXED READ FUNCTION (CHECKS ALL 4 WHEELS) =================
void RealRobotSystem::read_serial_feedback()
{
  char buf[256];
  int n = ::read(serial_conn_, buf, sizeof(buf));
  
  static double offset_0 = 0, offset_1 = 0, offset_2 = 0, offset_3 = 0;
  static bool first_run = true;
  
  // Keep track of the LAST valid ROS positions to check for glitches
  static double last_ros_p1 = 0, last_ros_p2 = 0, last_ros_p3 = 0, last_ros_p4 = 0;

  if (n > 0) {
    buf[n] = '\0'; 
    std::string data(buf);
    
    // Find the LAST 'e' in the buffer
    size_t last_e = data.rfind('e');
    if (last_e != std::string::npos) {
        long t1 = 0, t2 = 0, t3 = 0, t4 = 0;
        int count = sscanf(data.c_str() + last_e, "e %ld %ld %ld %ld", &t1, &t2, &t3, &t4);
        
        if (count == 4) {
            double rads_per_tick = (2 * M_PI) / enc_counts_per_rev_; 
            
            // 1. Raw Arduino Values
            double raw_p1 = t1 * rads_per_tick;
            double raw_p2 = t2 * rads_per_tick;
            double raw_p3 = t3 * rads_per_tick;
            double raw_p4 = t4 * rads_per_tick;

            // --- STARTUP ZEROING ---
            if (first_run) {
                offset_0 = -raw_p1;
                offset_1 = -raw_p2;
                offset_2 = -raw_p3;
                offset_3 = -raw_p4;
                last_ros_p1 = 0; last_ros_p2 = 0; last_ros_p3 = 0; last_ros_p4 = 0;
                first_run = false;
                RCLCPP_INFO(rclcpp::get_logger("RealRobotSystem"), "Startup TARE Complete.");
            }

            // 2. Calculate CANDIDATE ROS Positions
            double cand_p1 = raw_p1 + offset_0;
            double cand_p2 = raw_p2 + offset_1;
            double cand_p3 = raw_p3 + offset_2;
            double cand_p4 = raw_p4 + offset_3;

            // 3. GLITCH FILTER (Checking ALL 4 Wheels)
            double d1 = abs(cand_p1 - last_ros_p1);
            double d2 = abs(cand_p2 - last_ros_p2);
            double d3 = abs(cand_p3 - last_ros_p3);
            double d4 = abs(cand_p4 - last_ros_p4);

            // CHANGED: Increased threshold to 10.0 to allow driving, but stop glitches
            if (d1 > 10.0 || d2 > 10.0 || d3 > 10.0 || d4 > 10.0) {
                
                // CRITICAL CHECK: Is it a Brownout (ALL wheels near zero)?
                bool all_zero = (abs(raw_p1) < 0.2 && abs(raw_p2) < 0.2 && abs(raw_p3) < 0.2 && abs(raw_p4) < 0.2);

                if (all_zero) {
                    RCLCPP_WARN(rclcpp::get_logger("RealRobotSystem"), "Real Brownout Reset Detected. Recalibrating.");
                    offset_0 = last_ros_p1 - raw_p1;
                    offset_1 = last_ros_p2 - raw_p2;
                    offset_2 = last_ros_p3 - raw_p3;
                    offset_3 = last_ros_p4 - raw_p4;
                    
                    cand_p1 = raw_p1 + offset_0;
                    cand_p2 = raw_p2 + offset_1;
                    cand_p3 = raw_p3 + offset_2;
                    cand_p4 = raw_p4 + offset_3;
                } 
                else {
                    // It jumped, but NOT to zero. This is NOISE/CORRUPTION.
                    RCLCPP_WARN(rclcpp::get_logger("RealRobotSystem"), "Packet Rejected: Jump too large");
                    return; 
                }
            }

            // 4. Update ROS & History
            hw_positions_[0] = cand_p1;
            hw_positions_[1] = cand_p2;
            hw_positions_[2] = cand_p3;
            hw_positions_[3] = cand_p4;

            last_ros_p1 = cand_p1;
            last_ros_p2 = cand_p2;
            last_ros_p3 = cand_p3;
            last_ros_p4 = cand_p4;
        }
    }
}
}
} // namespace robot_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_hardware_interface::RealRobotSystem,
  hardware_interface::SystemInterface)