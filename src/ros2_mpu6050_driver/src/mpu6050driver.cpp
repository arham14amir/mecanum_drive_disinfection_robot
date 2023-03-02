#include "mpu6050driver/mpu6050driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MPU6050Driver::MPU6050Driver()
    : Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
{
  // Declare parameters
  declareParameters();
  // Set parameters
  mpu6050_->setGyroscopeRange(
      static_cast<MPU6050Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
  mpu6050_->setAccelerometerRange(
      static_cast<MPU6050Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
  mpu6050_->setDlpfBandwidth(
      static_cast<MPU6050Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
  mpu6050_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                               this->get_parameter("gyro_y_offset").as_double(),
                               this->get_parameter("gyro_z_offset").as_double());
  mpu6050_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                   this->get_parameter("accel_y_offset").as_double(),
                                   this->get_parameter("accel_z_offset").as_double());
  // Check if we want to calibrate the sensor
  if (this->get_parameter("calibrate").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Calibrating...");
    mpu6050_->calibrate();
  }
  mpu6050_->printConfig();
  mpu6050_->printOffsets();
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / this->get_parameter("gyro_range").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU6050Driver::handleInput, this));
}

void MPU6050Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_link";

  // --- FIX 1: SET COVARIANCE (Crucial for EKF) ---
  // We initialize everything to 0.0 first
  // Then we set the diagonals to a small value (0.01) to indicate "High Trust" but not "Infinite Trust"
  
  // Linear Acceleration Covariance
  message.linear_acceleration_covariance[0] = 0.01; // X
  message.linear_acceleration_covariance[4] = 0.01; // Y
  message.linear_acceleration_covariance[8] = 0.01; // Z

  // Angular Velocity Covariance
  message.angular_velocity_covariance[0] = 0.01; // X
  message.angular_velocity_covariance[4] = 0.01; // Y
  message.angular_velocity_covariance[8] = 0.01; // Z

  // --- FIX 2: CONVERT DEGREES TO RADIANS ---
  // The sensor gives degrees/sec. ROS needs radians/sec.
  // 1 Degree = 0.0174533 Radians
  double deg_to_rad = 0.0174533;

  message.linear_acceleration.x = mpu6050_->getAccelerationX();
  message.linear_acceleration.y = mpu6050_->getAccelerationY();
  message.linear_acceleration.z = mpu6050_->getAccelerationZ();

  message.angular_velocity.x = mpu6050_->getAngularVelocityX() * deg_to_rad;
  message.angular_velocity.y = mpu6050_->getAngularVelocityY() * deg_to_rad;
  message.angular_velocity.z = mpu6050_->getAngularVelocityZ() * deg_to_rad;

  // Invalidate quaternion (we don't calculate orientation here, EKF will do it)
  message.orientation_covariance[0] = -1;
  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;

  publisher_->publish(message);
}

void MPU6050Driver::declareParameters()
{
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<int>("gyro_range", MPU6050Sensor::GyroRange::GYR_250_DEG_S);
  this->declare_parameter<int>("accel_range", MPU6050Sensor::AccelRange::ACC_2_G);
  this->declare_parameter<int>("dlpf_bandwidth", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);
  this->declare_parameter<double>("gyro_x_offset", 0.0);
  this->declare_parameter<double>("gyro_y_offset", 0.0);
  this->declare_parameter<double>("gyro_z_offset", 0.0);
  this->declare_parameter<double>("accel_x_offset", 0.0);
  this->declare_parameter<double>("accel_y_offset", 0.0);
  this->declare_parameter<double>("accel_z_offset", 0.0);
  this->declare_parameter<int>("frequency", 0.0);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU6050Driver>());
  rclcpp::shutdown();
  return 0;
}