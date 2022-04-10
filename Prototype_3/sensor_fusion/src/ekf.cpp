#include "sensor_fusion/ekf.hpp"

#include <rclcpp/time.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>

#include "sensor_fusion/logging.hpp"

namespace ekf {

// Declare Logger
std::shared_ptr<rclcpp::Logger> LOGGER;

EKF::EKF() : rclcpp::Node("ekf") {
  // Initialise logger
  LOGGER = std::make_shared<rclcpp::Logger>(get_logger());

  publish_debug_ = declare_parameter<bool>("debug.publish_debug", false);
  enable_debug = declare_parameter<bool>("debug.enable", false);
  if (declare_parameter<bool>("debug.logging", true)) LOGGER->set_level(rclcpp::Logger::Level::Debug);


  // initialize publishers
  velocity_publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/velocity", 1);

  // initialize subscribers
  sbg_imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, std::bind(&EKF::sbg_imu_callback, this, _1));
  zed_imu_subscriber_ = create_subscription<sensor_msgs::msg::Imu>("/camera/imu/data", 10, std::bind(&EKF::zed_imu_callback, this, _1));
  wheel_speeds_subscriber_ = create_subscription<eufs_msgs::msg::WheelSpeedsStamped>("/ros_can/wheel_speeds", 10, std::bind(&EKF::wheel_speed_callback, this, _1));

  RCLCPP_DEBUG((*LOGGER), "EKF is running");
}

void EKF::sbg_imu_callback(const Imu::SharedPtr imu_data) {

  RCLCPP_DEBUG((*LOGGER), "SBG correction step started");
  // RCLCPP_DEBUG((*LOGGER), "SBG imu data %f, %f, %f", imu_data->angular_velocity.z, imu_data->linear_acceleration.x, imu_data->linear_acceleration.y);

  VectorXd z(3);
  z << imu_data->angular_velocity.z, imu_data->linear_acceleration.x, imu_data->linear_acceleration.y;
  // RCLCPP_DEBUG((*LOGGER), "z: %s", to_str(z).c_str());

  VectorXd h(3);
  h << state_.yaw_rate, state_.acceleration_x, state_.acceleration_y;
  // RCLCPP_DEBUG((*LOGGER), "h: %s", to_str(h).c_str());

  MatrixXd H = H_jacobian(imu_data);
  // RCLCPP_DEBUG((*LOGGER), "H: %s", to_str(H).c_str());

  MatrixXd K;
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  // RCLCPP_DEBUG((*LOGGER), "K: %s", to_str(K).c_str());

  VectorXd change = K * (z - h);
  state_.velocity_x += change(0);
  state_.velocity_y += change(1);
  state_.yaw_rate += change(2);
  state_.acceleration_x += change(3);
  state_.acceleration_y += change(4);

  P = (MatrixXd::Identity(5, 5) - K * H) * P;
  // RCLCPP_DEBUG((*LOGGER), "P: %s", to_str(P).c_str());

  previous_time_ = imu_data->header.stamp.nanosec;

  RCLCPP_DEBUG((*LOGGER), "SBG correction step complete");

  publish();
}

void EKF::zed_imu_callback(const Imu::SharedPtr imu_data) {

  RCLCPP_DEBUG((*LOGGER), "ZED correction step started");

  VectorXd z(3);
  z << imu_data->angular_velocity.z, imu_data->linear_acceleration.x, imu_data->linear_acceleration.y;
  // RCLCPP_DEBUG((*LOGGER), "z: %s", to_str(z).c_str());

  VectorXd h(3);
  h << state_.yaw_rate, state_.acceleration_x, state_.acceleration_y;
  // RCLCPP_DEBUG((*LOGGER), "h: %s", to_str(h).c_str());

  MatrixXd H = H_jacobian(imu_data);
  // RCLCPP_DEBUG((*LOGGER), "H: %s", to_str(H).c_str());

  MatrixXd K;
  K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
  // RCLCPP_DEBUG((*LOGGER), "K: %s", to_str(K).c_str());

  VectorXd change = K * (z - h);
  state_.velocity_x += change(0);
  state_.velocity_y += change(1);
  state_.yaw_rate += change(2);
  state_.acceleration_x += change(3);
  state_.acceleration_y += change(4);

  P = (MatrixXd::Identity(5, 5) - K * H) * P;
  // RCLCPP_DEBUG((*LOGGER), "P: %s", to_str(P).c_str());

  previous_time_ = imu_data->header.stamp.nanosec;

  RCLCPP_DEBUG((*LOGGER), "ZED correction step complete");

  publish();
}

void EKF::wheel_speed_callback(const WheelSpeedsStamped::SharedPtr wheel_speeds_data) {

  RCLCPP_DEBUG((*LOGGER), "Prediction step started");

  double vx = state_.velocity_x;
  double vy = state_.velocity_y;
  double ax = state_.acceleration_x;
  double ay = state_.acceleration_y;

  // applies basic motion model to update the state
  auto dt = wheel_speeds_data->header.stamp.nanosec - previous_time_;
  state_.velocity_x = vx + ax * dt;
  state_.velocity_y = vy + ay * dt;
  state_.yaw_rate = (vx * std::sin(wheel_speeds_data->speeds.steering)) / wheelbase_length;
  state_.acceleration_x = ax;
  state_.acceleration_y = ay;
  previous_time_ = wheel_speeds_data->header.stamp.nanosec;

  MatrixXd G = G_jacobian(wheel_speeds_data, dt);
  // RCLCPP_DEBUG((*LOGGER), "G: %s", to_str(G).c_str());

  P = G * P * G.transpose() + Q;
  // RCLCPP_DEBUG((*LOGGER), "P: %s", to_str(P).c_str());
  
  RCLCPP_DEBUG((*LOGGER), "Prediction step complete");
}

MatrixXd EKF::G_jacobian(const WheelSpeedsStamped::SharedPtr wheel_speeds_data, const auto dt) {
  
  RCLCPP_DEBUG((*LOGGER), "Calculating G_jacobian");

  MatrixXd G;
  G = MatrixXd::Zero(5, 5);
  
  G(0,0) = 1;
  G(0,3) = dt;
  G(1,1) = 1;
  G(1,4) = dt;
  G(2,0) = std::sin(wheel_speeds_data->speeds.steering) / wheelbase_length;
  G(3,3) = 1;
  G(4,4) = 1;

  RCLCPP_DEBUG((*LOGGER), "Calculated G_jacobian");

  return G;
}

MatrixXd EKF::H_jacobian(const Imu::SharedPtr imu_data) {

  RCLCPP_DEBUG((*LOGGER), "Calculating H_jacobian");

  MatrixXd H;
  H = MatrixXd::Zero(3, 5);

  H(0,2) = 1;
  H(1,3) = 1;
  H(2,4) = 1;

  RCLCPP_DEBUG((*LOGGER), "Calculated H_jacobian");

  return H;
} 

void EKF::publish() {
  // create a twist message
  TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp = rclcpp::Time(previous_time_);
  twist_msg.twist.twist.linear.x = state_.velocity_x;
  twist_msg.twist.twist.linear.y = state_.velocity_y;
  twist_msg.twist.twist.angular.z = state_.yaw_rate;

  // publish the message
  velocity_publisher_->publish(twist_msg);
}

} // namespace ekf

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ekf = std::make_shared<ekf::EKF>();
  rclcpp::spin(ekf);
  rclcpp::shutdown();
  return 0;
}
