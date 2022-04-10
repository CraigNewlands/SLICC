#ifndef EKF_INCLUDE_EFK_EFK_H_
#define EKF_INCLUDE_EKF_EKF_H_

#include <string>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::placeholders::_1;
using sensor_msgs::msg::Imu;
using eufs_msgs::msg::WheelSpeedsStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;

namespace ekf {

struct State {
  double velocity_x;
  double velocity_y;
  double yaw_rate;
  double acceleration_x;
  double acceleration_y;
};

class EKF final : public rclcpp::Node{
  public:
    EKF(); // constructor

    void sbg_imu_callback(const Imu::SharedPtr imu_data);
    void zed_imu_callback(const Imu::SharedPtr imu_data);
    void wheel_speed_callback(const WheelSpeedsStamped::SharedPtr wheel_speed_data);

    MatrixXd G_jacobian(const WheelSpeedsStamped::SharedPtr wheel_speeds_data, const auto dt);
    MatrixXd H_jacobian(const Imu::SharedPtr imu_data);

    void publish();

  private:

    bool publish_debug_;
    bool enable_debug;

    // state vector and matrix
    State state_;

    // previous time
    double previous_time_;

    // constants
    double wheelbase_length = 1.53;

    // covariance matrix for the uncertainty in the initial state
    MatrixXd P = MatrixXd::Identity(5, 5) * 500;

    // process noise covariance matrix
    MatrixXd Q = MatrixXd::Identity(5, 5);

    // measurement noise covariance matrix
    MatrixXd R = MatrixXd::Identity(3, 3) * 0.01;

    // ROS publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr velocity_publisher_;

    // ROS subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sbg_imu_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr zed_imu_subscriber_;
    rclcpp::Subscription<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_speeds_subscriber_;    

};

} // namespace ekf

#endif  // EKF_INCLUDE_EFK_EFK_H_