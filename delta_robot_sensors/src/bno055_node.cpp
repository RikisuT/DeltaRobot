#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "bno055_node.hpp"
#include "bno055.hpp"

#define I2C_DEVICE "/dev/i2c-1"

BNO055Node::BNO055Node() : Node("BNO055_Sensor") {
  // Declare parameters
  float sensor_freq = this->declare_parameter("sensor_freq", 100.0); // [Hz]
  // Get parameter from yaml file
  sensor_freq = this->get_parameter("sensor_freq").as_double();

  // Initialize the BNO055 sensor
  try {
    this->sensor.init(I2C_DEVICE, BNO055_ADDRESS_A);
  }
  catch (const std::exception& e) {
    rclcpp::shutdown();
    throw std::runtime_error("BNO055 Sensor offline, shutting down node: " + std::string(e.what()));
  }

  RCLCPP_INFO(this->get_logger(), "BNO055 Sensor online!");

  // Topics
  const std::string imu_topic = "bno055/raw_imu";
  const std::string mag_topic = "bno055/raw_mag";
  const std::string temp_topic = "bno055/raw_temp";

  // Initialize the publishers
  this->imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
  this->mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 10);
  this->temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>(temp_topic, 10);

  RCLCPP_INFO(this->get_logger(), "BNO055 Sensor publishing on topics: (%s), (%s), (%s) at %.1f Hz",
    imu_topic.c_str(), mag_topic.c_str(), temp_topic.c_str(), sensor_freq);

  // Create a timer with a 100ms period to read and publish
  this->timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / sensor_freq)),
    [this]() -> void {
    IMURecord record = this->sensor.read();
    const std::string sensor_frame_id = "BNO055_frame";

    // IMU data
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = sensor_frame_id;
    imu_msg.linear_acceleration.x = record.raw_linear_acceleration_x;
    imu_msg.linear_acceleration.y = record.raw_linear_acceleration_y;
    imu_msg.linear_acceleration.z = record.raw_linear_acceleration_z;
    imu_msg.angular_velocity.x = record.raw_angular_velocity_x;
    imu_msg.angular_velocity.y = record.raw_angular_velocity_y;
    imu_msg.angular_velocity.z = record.raw_angular_velocity_z;
    imu_msg.orientation.w = record.fused_orientation_w;
    imu_msg.orientation.x = record.fused_orientation_x;
    imu_msg.orientation.y = record.fused_orientation_y;
    imu_msg.orientation.z = record.fused_orientation_z;

    // Magnetic field data
    sensor_msgs::msg::MagneticField mag_msg;
    mag_msg.header.stamp = this->now();
    mag_msg.header.frame_id = sensor_frame_id;
    mag_msg.magnetic_field.x = record.raw_magnetic_field_x;
    mag_msg.magnetic_field.y = record.raw_magnetic_field_y;
    mag_msg.magnetic_field.z = record.raw_magnetic_field_z;

    // Temperature data
    sensor_msgs::msg::Temperature temp_msg;
    temp_msg.header.stamp = this->now();
    temp_msg.header.frame_id = sensor_frame_id;
    temp_msg.temperature = record.temperature;

    // Publish the data
    this->imu_pub->publish(imu_msg);
    this->mag_pub->publish(mag_msg);
    this->temp_pub->publish(temp_msg);
  }
  );
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<BNO055Node>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), e.what());
  }
  rclcpp::shutdown();
  return 0;
}