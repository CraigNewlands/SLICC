import rclpy
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rclpy.node import Node
from datetime import datetime
import time
import message_filters
import numpy as np
from numpy.linalg import inv
import math
import jax
from jax import jacfwd
from jax.numpy import asarray, sin
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Imu
from eufs_msgs.msg import WheelSpeedsStamped
from geometry_msgs.msg import TwistWithCovarianceStamped


class EKF(Node):

    def __init__(self):
        super().__init__('ekf_publisher')
        self.publisher_ = self.create_publisher(TwistWithCovarianceStamped, 'ekf_data', 10)

        # BAG FILE SUBSCRIBERS
        self.wheel_data_sub = message_filters.Subscriber(self, WheelSpeedsStamped, 'ros_can/wheel_speeds')
        self.sbg_imu_data_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        self.zed_imu_data_sub = message_filters.Subscriber(self, Imu, '/camera/imu/data')


        # SIMULATION SUBSCRIBERS
        # self.zed_imu_data_sub = message_filters.Subscriber(self, Imu, 'imu', qos_profile=QoSPresetProfiles.SENSOR_DATA.value)
        # self.wheel_data_sub = message_filters.Subscriber(self, WheelSpeedsStamped, 'ros_can/wheel_speeds')
        # self.gps_velocity_data_sub = message_filters.Subscriber(self, TwistWithCovarianceStamped, 'fix_velocity')
        # self.sbg_imu_data_sub = message_filters.Subscriber(self, Imu, '/imu/data')

        # Synchronises the data from the bag file subscribers
        time_synchronizer = message_filters.ApproximateTimeSynchronizer([self.wheel_data_sub, self.sbg_imu_data_sub, self.zed_imu_data_sub], queue_size=10, slop=1)
        time_synchronizer.registerCallback(self.callback)

        self.previous_time = time.time()

        # Initial state vector [vel_x, vel_y, yaw_rate, acc_x, acc_y]
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Covariance matrix for the uncertainty in the initial state
        self.P = np.array([[500, 0, 0, 0, 0],
                           [0, 500, 0, 0, 0],
                           [0, 0, 500, 0, 0],
                           [0, 0, 0, 500, 0],
                           [0, 0, 0, 0, 500]])

        self.wheelbase_length = 1.53

        #TODO figure out how to compute this (hint: look at robot_localization)
        self.process_noise_covariance = np.array([[1, 0, 0, 0, 0],
                                                  [0, 1, 0, 0, 0],
                                                  [0, 0, 1, 0, 0],
                                                  [0, 0, 0, 1, 0],
                                                  [0, 0, 0, 0, 1]])

        #TODO figure out how to compute this 
        self.measurement_noise_covariance = np.array([[1, 0, 0],
                                                      [0, 1, 0],
                                                      [0, 0, 1]])

                                        
    # Returns the new state based on the prediction state measurement and the previous state
    # Uses the motion model to calcualte the new state
    def state_prediction(self, previous_state, wheel_control_data, dt):

        return asarray([previous_state[0] + previous_state[3] * dt,
                        previous_state[1] + previous_state[4] * dt,
                        (previous_state[0] * sin(wheel_control_data[0])) / self.wheelbase_length,
                        previous_state[3],
                        previous_state[4]])

    # Applies the measurment model
    def measurement_prediction(self, predicted_state):

        predicted_measurement = asarray([predicted_state[2], predicted_state[3], predicted_state[4]])

        return predicted_measurement

    # Takes in the sensor measurements and applies the EKF equations 
    def callback(self, wheel_data, sbg_data, zed_data):
        # Assigns the sensor measurement values that we need to variables
        wheel_control_data = np.array([wheel_data.speeds.steering])
        sbg_imu_data = np.array([sbg_data.angular_velocity.z, sbg_data.linear_acceleration.x, sbg_data.linear_acceleration.y])
        zed_imu_data = np.array([zed_data.angular_velocity.z, zed_data.linear_acceleration.x, zed_data.linear_acceleration.y])

        # Calculates the change in time between sensor readings
        current_time = time.time()
        dt = time.time() - self.previous_time
        self.previous_time = current_time

        # Computes the Jacobian for the prediction step
        G = jacfwd(lambda steering: self.state_prediction(self.x, steering, dt), 0)(wheel_control_data)

        # Prediction step - control data (steering angle)
        self.x = self.state_prediction(self.x, wheel_control_data, dt)
        self.P = G.transpose().dot(self.P).dot(G) + self.process_noise_covariance


        # Correction step - SBG IMU data
        h = self.measurement_prediction(self.x)
        # Computes the Jacobian for the correction step
        H = jacfwd(lambda x: self.measurement_prediction(x))(self.x)

        # Computes the Kalman gain
        K = self.P.dot(H.transpose()).dot(inv(H.dot(self.P).dot(H.transpose()) + self.measurement_noise_covariance))

        self.x = self.x + K.dot(sbg_imu_data - h)

        I = np.identity(5)
        self.P = (I - K.dot(H)).dot(self.P)


        # Correction step - ZED IMU data
        h = self.measurement_prediction(self.x)
        # Computes the Jacobian for the correction step
        H = jacfwd(lambda x: self.measurement_prediction(x))(self.x)

        # Computes the Kalman gain
        K = self.P.dot(H.transpose()).dot(inv(H.dot(self.P).dot(H.transpose()) + self.measurement_noise_covariance))

        self.x = self.x + K.dot(zed_imu_data - h)

        I = np.identity(5)
        self.P = (I - K.dot(H)).dot(self.P)


        # print(self.x)

        # Publishes the velocity estimate
        msg = TwistWithCovarianceStamped()
        msg.header.frame_id = 'base_footprint'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.twist.linear.x = float(self.x[0])
        msg.twist.twist.linear.y = float(self.x[1])
        msg.twist.twist.angular.z = float(self.x[2])
        msg.twist.covariance[0] = float(self.P[0][0])
        msg.twist.covariance[7] = float(self.P[1][1])
        msg.twist.covariance[35] = float(self.P[2][2])
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ekf = EKF()

    rclpy.spin(ekf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
