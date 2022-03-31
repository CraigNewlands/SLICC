# Prototype 3 - C++ Extended Kalman Filter
This EKF implementation in C++ estimates the velocity given the sensor inputs. Similar to prototype 2, it works on real time data; however, it is a lot more abstract and the sensors for the prediction and correction steps can be specified in the config file. 

## How to use
1. Install ROS2 Galactic from here: https://docs.ros.org/en/galactic/Installation.html
2. Download to bag file from here: 
3. Pull this folder
4. Source ros2 `source /opt/ros/galactic/setup.zsh` or ` /opt/ros/galactic/setup.bash`
5. Go to the Prototype_3 folder and run `colcon build --packages-select sensor_fusion` 
7. Run the bag file in another terminal `ros2 bag play -l <bag file name>`
8. Run the ekf node `ros2 run sensor_fusion efk`

Now the EKF is running and the output velocity can be seen by subscribing to the ekf_data topic.
1. Open a new terminal and source ros2
2. Subscribe to the ekf_data `ros2 topic echo ekf`
