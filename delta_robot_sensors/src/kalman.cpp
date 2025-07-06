#include "rclcpp/rclcpp.hpp"
#include "kalman.hpp"
#include <eigen3/Eigen/Dense>

typedef struct {
  rclcpp::Time timestamp; // timestamp of the state vector
  float x; // x position of the end effector [mm]
  float y; // y position of the end effector [mm]
  float z; // z position of the end effector [mm]
  float x_dot; // x velocity of the end effector [mm/s]
  float y_dot; // y velocity of the end effector [mm/s]
  float z_dot; // z velocity of the end effector [mm/s]
  float x_ddot; // x acceleration of the end effector [mm/s^2]
  float y_ddot; // y acceleration of the end effector [mm/s^2]
  float z_ddot; // z acceleration of the end effector [mm/s^2]
} StateVector;

KalmanFilter::KalmanFilter() : Node("kalman") {
  // Declare parameters
  float timer_freq = this->declare_parameter("timer_frequency", 100.0); // [Hz]
  this->H = this->declare_parameter("ground_to_base_height", 400.0); // [mm]
  this->tofOffsetMM = this->declare_parameter("tof_offset_mm", 0.0); // [mm]
  this->rangeFilter.alpha = this->declare_parameter("range_filter_alpha", 0.1);
  this->rangeFilter.beta = this->declare_parameter("range_filter_beta", 0.01);
  this->tempFilter.alpha = this->declare_parameter("temp_filter_alpha", 0.1);
  this->tempFilter.beta = this->declare_parameter("temp_filter_beta", 0.01);
  // Get parameters from yaml file
  timer_freq = this->get_parameter("timer_frequency").as_double();
  this->H = this->get_parameter("ground_to_base_height").as_double();
  this->rangeFilter.alpha = this->get_parameter("range_filter_alpha").as_double();
  this->rangeFilter.beta = this->get_parameter("range_filter_beta").as_double();
  this->tempFilter.alpha = this->get_parameter("temp_filter_alpha").as_double();
  this->tempFilter.beta = this->get_parameter("temp_filter_beta").as_double();
  this->tofOffsetMM = this->get_parameter("tof_offset_mm").as_double();

  // Initialize time for the filters
  this->rangeFilter.prevMeasureTime = this->now();
  this->tempFilter.prevMeasureTime = this->now();

  // Raw data Topics
  const std::string imu_raw_topic = "bno055/raw_imu";
  const std::string mag_raw_topic = "bno055/raw_mag";
  const std::string temp_raw_topic = "bno055/raw_temp";
  const std::string range_raw_topic = "vl53l1x/raw_range";
  const std::string robot_config_topic = "delta_robot/robot_config";

  // Filtered Data Topics
  const std::string imu_filtered_topic = "bno055/filtered_imu";
  const std::string mag_filtered_topic = "bno055/filtered_mag";
  const std::string temp_filtered_topic = "bno055/filtered_temp";
  const std::string range_filtered_topic = "vl53l1x/filtered_range";
  const std::string range_filtered_error_topic = "vl53l1x/filtered_range_error";

  // Raw Sensor Data (Subscribers)
  auto rawDataQoS = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
  // Filtered Sensor Data (Publishers)
  auto filteredDataQoS = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Initialize the subscribers
  this->raw_imu_sub = this->create_subscription<IMU>(
    imu_raw_topic, rawDataQoS, std::bind(&KalmanFilter::imuCallback, this, std::placeholders::_1));
  this->raw_mag_sub = this->create_subscription<MagField>(
    mag_raw_topic, rawDataQoS, std::bind(&KalmanFilter::magCallback, this, std::placeholders::_1));
  this->raw_temp_sub = this->create_subscription<Temp>(
    temp_raw_topic, rawDataQoS, std::bind(&KalmanFilter::tempCallback, this, std::placeholders::_1));
  this->raw_range_sub = this->create_subscription<Range>(
    range_raw_topic, rawDataQoS, std::bind(&KalmanFilter::rangeCallback, this, std::placeholders::_1));
  this->robot_config_sub = this->create_subscription<RobotConfig>(robot_config_topic, rawDataQoS,
    [this](const RobotConfig::SharedPtr msg) -> void {this->latestConfig = *msg;});

  // Initialize Publishers
  this->filtered_imu_pub = this->create_publisher<IMU>(imu_filtered_topic, filteredDataQoS);
  this->filtered_mag_pub = this->create_publisher<MagField>(mag_filtered_topic, filteredDataQoS);
  this->filtered_temp_pub = this->create_publisher<Temp>(temp_filtered_topic, filteredDataQoS);
  this->filtered_range_pub = this->create_publisher<Range>(range_filtered_topic, filteredDataQoS);
  this->filtered_range_error_pub = this->create_publisher<Float32>(range_filtered_error_topic, filteredDataQoS);

  // Log topics being used
  RCLCPP_INFO(this->get_logger(), "Kalman Filter subscribing to raw data on topics: (%s), (%s), (%s), (%s)",
    imu_raw_topic.c_str(), mag_raw_topic.c_str(), temp_raw_topic.c_str(), range_raw_topic.c_str()
  );
  RCLCPP_INFO(this->get_logger(), "Kalman Filter publishing filtered data on topics: (%s), (%s), (%s), (%s)",
    imu_filtered_topic.c_str(), mag_filtered_topic.c_str(), temp_filtered_topic.c_str(), range_filtered_topic.c_str()
  );

  // Log filter parameters
  RCLCPP_INFO(this->get_logger(), "Alpha-Beta Filter Parameters: Range (alpha=%.2f, beta=%.2f), Temp (alpha=%.2f, beta=%.2f)",
    this->rangeFilter.alpha, this->rangeFilter.beta, this->tempFilter.alpha, this->tempFilter.beta
  );

  // Create a timer to run the filter
  this->timer = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / timer_freq), // [s]
    std::bind(&KalmanFilter::timerCallback, this)
  );
}

void KalmanFilter::timerCallback() {
  // Assumming that Kinematics is our "truth", we can estimate the error of the range filter
  // |EE.Z| + TOF = H (EE.Z should be a negative number so we take the absolute value)
  float eeZ = this->latestConfig.end_effector_position.z;
  float tof = (this->rangeFilter.estimate * 1000) + this->tofOffsetMM; // Convert to mm
  float error = this->H - (std::abs(eeZ) + tof);
  auto error_msg = Float32();
  error_msg.data = error / 1000; // Convert back to meters for plotting
  this->filtered_range_error_pub->publish(error_msg);
}

void KalmanFilter::applyAlphaBetaFilter(float z, AlphaBetaFilter& filter) {
  // Handle first measurement case
  if (filter.previousEstimate == 0 && filter.previousRateEstimate == 0) {
    filter.prevMeasureTime = this->now();
    filter.previousEstimate = z;
    filter.estimate = z; // Use current measurement as the initial estimate
    return; // Skip prediction for first measurement
  }

  // Update Timestep and save the previous measurement time
  float dt = (this->now() - filter.prevMeasureTime).seconds();
  filter.prevMeasureTime = this->now();

  // Prediction Step
  filter.estimate = filter.previousEstimate + (filter.rateEstimate * dt);

  // Update Step
  float residual = z - filter.estimate;
  filter.estimate += filter.alpha * residual;
  filter.rateEstimate += filter.beta * (residual / dt);

  // Update the filter state
  filter.previousRateEstimate = filter.rateEstimate;
  filter.previousEstimate = filter.estimate;
}

void KalmanFilter::imuCallback(const IMU::SharedPtr msg) {
  (void)msg;
}

void KalmanFilter::magCallback(const MagField::SharedPtr msg) {
  (void)msg;
}

void KalmanFilter::tempCallback(const Temp::SharedPtr msg) {
  // Alpha-Beta Filter for Temperature data from BNO055 Sensor
  this->applyAlphaBetaFilter(msg->temperature, this->tempFilter);

  // Publish the filtered temperature value
  auto filtered_msg = std::make_shared<Temp>(*msg);
  filtered_msg->temperature = this->tempFilter.estimate; // Update the temperature value to the filtered value
  filtered_msg->header.stamp = this->now(); // Update the timestamp to the current time
  this->filtered_temp_pub->publish(*filtered_msg);
}

void KalmanFilter::rangeCallback(const Range::SharedPtr msg) {
  // Alpha-Beta Filter for Range Data from Time-of-Flight Sensor
  this->applyAlphaBetaFilter(msg->range, this->rangeFilter);

  // Publish the filtered range value
  auto filtered_msg = std::make_shared<Range>(*msg);
  filtered_msg->range = this->rangeFilter.estimate; // Update the range value to the filtered value
  filtered_msg->header.stamp = this->now(); // Update the timestamp to the current time
  // Only publish the filtered range if it is within the min and max range values
  if ((filtered_msg->range >= filtered_msg->min_range) && (filtered_msg->range <= filtered_msg->max_range)) {
    this->filtered_range_pub->publish(*filtered_msg);
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KalmanFilter>());
  rclcpp::shutdown();
  return 0;
}