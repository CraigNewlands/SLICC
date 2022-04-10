#ifndef EKF_INCLUDE_EKF_LOGGING_HPP_
#define EKF_INCLUDE_EKF_LOGGING_HPP_

#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ekf {

// Logger
extern std::shared_ptr<rclcpp::Logger> LOGGER;

std::string to_str(const Eigen::MatrixXd &vec);

}  // namespace ekf

#endif  // EKF_INCLUDE_EKF_LOGGING_HPP_
