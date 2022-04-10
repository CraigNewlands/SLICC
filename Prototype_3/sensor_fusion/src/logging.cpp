#include "sensor_fusion/logging.hpp"

namespace ekf {

// Convert Eigen matrix to string
std::string to_str(const Eigen::MatrixXd &vec) {
  std::string s = "{ ";
  for (int row = 0; row < vec.rows(); row++) {
    for (int col = 0; col < vec.cols(); col++) {
      s += std::to_string(vec(row, col));
      if (row == vec.rows() - 1) {
        s += (col == vec.cols() - 1) ? " }" : ", ";
      } else {
        s += (col == vec.cols() - 1) ? " || " : ", ";
      }
    }
  }
  return s;
}

}  // namespace ekf
