// Prediction step using imu to get [x_velocity, y_velocity, yaw_velocity]
// The first correction step uses wheel speed
// The second using velocity data from the zed camera
// Returns the estimated velocity [x_velocity, y_velocity, yaw_velocity]

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
using namespace std;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

int main() {
    double dt = 0.25;
    int count = 31;

    vector<Vector3d> imu_data;      // [x_acceleration, y_acceleration, yaw_angular_acceleration]
    vector<Vector3d> odom_data;     // [x_velocity, y_velocity, yaw_velocity] NOTE that we do not recieve a y_velocity from odom so this measurement value will always be 0
    vector<Vector3d> zed_data;      // [x_velocity, y_velocity, yaw_velocity]
    vector<Vector3d> true_velocity;
    vector<Vector3d> estimates;

    imu_data.push_back({0, 0, 0});
    odom_data.push_back({0, 0, 0});
    zed_data.push_back({0, 0, 0});

    default_random_engine generator;
    normal_distribution<double> distribution(0,1.0);

    double x_vel;
    double y_vel;
    double yaw_vel;

    for (int i = 0; i < count; i++) {
        double x_acc = 6 + distribution(generator);
        double y_acc = 6 + distribution(generator);
        double yaw_acc = 6 + distribution(generator);

        x_vel = x_vel + x_acc * dt;
        y_vel = y_vel + y_acc * dt;
        yaw_vel = yaw_vel + yaw_acc * dt;

        imu_data.push_back({x_acc, y_acc, yaw_acc});
        odom_data.push_back({x_vel + distribution(generator), 0, yaw_vel + distribution(generator)});       // NOTE we do not recieve a y_velocity from odom so it is set to 0
        zed_data.push_back({x_vel + distribution(generator), y_vel + distribution(generator), yaw_vel + distribution(generator)});
        true_velocity.push_back({x_vel, y_vel, yaw_vel});
    }

    double std_acceleration = 0.01;

    vector<double> x_axis(count-1);
    double j = 0;
    for (int i = 0; i < count-1; i++) {
        x_axis[i] = j;
        j+= 0.25;
    }

    Vector3d x;         // initial state vector [x_velocity, y_velocity, yaw_velocity]
    x << 0, 0, 0;

    Matrix3d P;         // covariance matrix for the uncertainty in the initial state
    P << 500, 0, 0,
         0, 500, 0,
         0, 0, 500;

    for (int i = 1; i < count; i++) {
        // Prediction step
        Matrix3d F;     // state transformation matrtix
        F << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

        Matrix3d G;     // control matrix to convert the data from the imu [x_acceleration, y_acceleration, yaw_angular_acceleration] into the same form as a our state vector [x_velocity, y_velocity, yaw_velocity]
        G << dt, 0, 0,
             0, dt, 0,
             0, 0, dt;

        Matrix3d Q;         // Discrete noise model 
        Q = G * std_acceleration * G.transpose();

        x = F * x + G * imu_data[i-1];
        P = F * P * F.transpose() + Q;


        // Correction step using wheel speed measurements [x_velocity, yaw_velocity]
        Matrix3d H;      // observation matrix 
        H << 1, 0, 0,
             0, 0, 0,
             0, 0, 1;

        Matrix3d R;         // measurement noise covariance matrix (how confident are we with the measurement data)
        R << 400, 0, 0,     // we have assumed that x_velocity and yaw_velocity are uncorrelated
             0, 1, 0,       // matrix is singular if this is removed and therefore cannot find inverse
             0, 0, 400;

        Matrix3d K;
        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x = x + K * (odom_data[i] - (H * x));
        P = (MatrixXd::Identity(3,3) - K * H) * P * (MatrixXd::Identity(3,3) - K * H).transpose() + K * R * K.transpose();
        

        // Correction step using zed camera measurements [x_velocity, y_velocity, yaw_velocity]
        H << 1, 0, 0,   // observation matrix 
             0, 1, 0,
             0, 0, 1;

        R << 400, 0, 0,   // measurement noise covariance matrix (how confident are we with the measurement data)
             0, 400, 0,     // we have assumed that x_velocity, y_velocity and yaw_velocity are uncorrelated
             0, 0, 400;

        K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        x = x + K * (zed_data[i] - (H * x));
        P = (MatrixXd::Identity(3,3) - K * H) * P * (MatrixXd::Identity(3,3) - K * H).transpose() + K * R * K.transpose();
        
        estimates.push_back(x);
    }

    imu_data.erase(imu_data.begin());
    odom_data.erase(odom_data.begin());
    zed_data.erase(zed_data.begin());

    vector<vector<double>> estimated_velocities( 3 , vector<double> (count-1, 0));
    vector<vector<double>> true_velocities( 3 , vector<double> (count-1, 0));
    vector<vector<double>> odom_velocities( 3 , vector<double> (count-1, 0));
    vector<vector<double>> zed_velocities( 3 , vector<double> (count-1, 0));

    for (int i = 0; i < estimates.size(); i++) { 
        estimated_velocities[0][i] = estimates[i][0];
        true_velocities[0][i] = true_velocity[i][0];
        odom_velocities[0][i] = odom_data[i][0];
        zed_velocities[0][i] = zed_data[i][0];

        estimated_velocities[1][i] = estimates[i][1];
        true_velocities[1][i] = true_velocity[i][1];
        odom_velocities[1][i] = odom_data[i][1];
        zed_velocities[1][i] = zed_data[i][1];

        estimated_velocities[2][i] = estimates[i][2];
        true_velocities[2][i] = true_velocity[i][2];
        odom_velocities[2][i] = odom_data[i][2];
        zed_velocities[2][i] = zed_data[i][2];
    }

    plt::figure();
    plt::title("X Velocity against Time");
    plt::named_plot("Estimated velocity x", x_axis, estimated_velocities[0], "tab:red"); // estimate
    plt::named_plot("True velocity x", x_axis, true_velocities[0], "tab:green"); // true position
    plt::named_plot("Odom measured velocity x", x_axis, odom_velocities[0], "tab:purple"); // measurements
    plt::named_plot("Zed camera measured velocity x", x_axis, zed_velocities[0], "tab:blue"); // measurements
    plt::xlabel("Time (s)");
    plt::ylabel("x velocity (m/s)");
    plt::legend();
    plt::save("x_velocity.png");
    plt::show(); // show the figure instead of saving it

    plt::figure();
    plt::title("Y Velocity against Time");
    plt::named_plot("Estimated velocity y", x_axis, estimated_velocities[1], "tab:red"); // estimate
    plt::named_plot("True velocity y", x_axis, true_velocities[1], "tab:green"); // true position
//     plt::named_plot("Odom measured velocity y", x_axis, odom_velocities[1], "tab:purple"); // measurements
    plt::named_plot("Zed camera measured velocity y", x_axis, zed_velocities[1], "tab:blue"); // measurements
    plt::xlabel("Time (s)");
    plt::ylabel("y velocity (m/s)");
    plt::legend();
    plt::save("y_velocity.png");
    plt::show(); // show the figure instead of saving it

    plt::figure();
    plt::title("Yaw against Time");
    plt::named_plot("Estimated velocity yaw", x_axis, estimated_velocities[2], "tab:red"); // estimate
    plt::named_plot("True velocity yaw", x_axis, true_velocities[2], "tab:green"); // true position
    plt::named_plot("Odom measured velocity yaw", x_axis, odom_velocities[2], "tab:purple"); // measurements
    plt::named_plot("Zed camera measured velocity yaw", x_axis, zed_velocities[2], "tab:blue"); // measurements
    plt::xlabel("Time (s)");
    plt::ylabel("yaw velocity (m/s)");
    plt::legend();
    plt::save("yaw_velocity.png");
    plt::show(); // show the figure instead of saving it
}
