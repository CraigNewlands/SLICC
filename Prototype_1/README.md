# Prototype 1 - Velocity Kalman Filter

This was the first prototype I created for my SLICC project and it generates its own wheel speed data, SBG data and ZED camera data.

## How to run
1. Open the SLICC folder in VS code
2. Install eigen from this guide https://eigen.tuxfamily.org/dox/GettingStarted.html 
3. Add the location of the eigen folder to the tasks.json file
4. Open the velocityKalmanFilter.cpp file and press **ctrl+shift+b** to build
5. Run **./velocityKalmanFilter** in the terminal and the graphs should appear one at a time

## Graphs

The three graphs showing the kalman filter at work can also be found in this folder under the names:
- x_velocity.png
- y_velocity.png
- yaw_velocity.png
