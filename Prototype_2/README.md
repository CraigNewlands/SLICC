# Prototype 2 - Python Extended Kalman Filter
This EKF implementation uses a bag file that we collected during our testing and we use steering angle for the predicition step, SBG IMU data and ZED camera IMU data for the correction steps.

## How to use
1. Install ROS2 Galactic from here: https://docs.ros.org/en/galactic/Installation.html
2. Download to bag file from here: https://uoe-my.sharepoint.com/:f:/g/personal/s1943761_ed_ac_uk/EmqO_kZhVrhOhhuyR0Ry3tkBB3BL281DnJLJBQjGLJHB2w?e=q03XYJ
3. Pull this SLICC repo
4. Source ros2 `source /opt/ros/galactic/setup.zsh` or ` /opt/ros/galactic/setup.bash`
5. Go to the Prototype_2 folder and run `colcon build --packages-select python_ekf` 
7. Run the bag file in another terminal `ros2 bag play -l <bag file name>` (remember to source in this terminal too)
8. Run the ekf node `ros2 run python_ekf ekf_data`

Now the EKF is running and the output velocity can be seen by subscribing to the ekf_data topic.
1. Open a new terminal and source ros2
2. Subscribe to the ekf_data `ros2 topic echo ekf_data`
