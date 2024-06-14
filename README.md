<img src="https://jacobsschool.ucsd.edu/sites/default/files/groups/jsoe/img/logos/jacobs-school/digital/UCSDLogo_JSOE_BlueGold_Web.jpg" width="256">

# MAE 148 Spring 2024 Team 5 Final Project 

##  RC Car Tennis Ball Boy with Odometry and YOLO 
Team 5 - Saimai Lau - MAE, Ryan Salas - MAE, Cynthia Do - MAE, Theo Emery - MAE

<details>
<summary> Table of Contents </summary>

[Overview](#overview)

[Pathing Journey](#pathing-journey)

[References & Acknowledgements](#references) 

</details> 

# Overview
We wanted to make an RC car that would be able to fulfill the responsibilities of a ball boy in tennis matches. The duties of a ball boy are primarily to retrieve the ball when out of play and to provide balls to the players. We aimed to primarily achieve this first task with the second being a nice-to-have feature. 

### RC Car 
<img src="https://media.discordapp.net/attachments/879993760937816084/1248458336668749895/IMG_5586.jpg?ex=6663bcee&is=66626b6e&hm=20503c054fa5e9fecb3b17bac04db99c57acab082e7b6bc326c7debe89365978&=&format=webp&width=501&height=669" width = "400">

### Schematic Diagram 
<img src="https://media.discordapp.net/attachments/879993760937816084/1248463998148542585/image.png?ex=6663c234&is=666270b4&hm=3e615b0727b4fbb52704b3ab336dd23508fa6cbe6dd9843741c3a985cbde84ab&=&format=webp&quality=lossless&width=929&height=669" width = "500">

### Main Goals
- Identifies a tennis ball
- Gets the relative spatial location of the ball
- Goes to the ball
- Retrieves the ball

### Nice-to-Haves
- Relay ball to player serving (when needed)
- Obstacle avoidance

### Main Hardware Utilized 
- Jetson Nano
- OAK-D Camera LITE 
- GNSS 
- VESC Motor Controller 

### Ball Recognition with YOLO Model Trained Using Roboflow
<img src="https://media.discordapp.net/attachments/1224900279640789092/1237598768279588945/image.png?ex=666b086b&is=6669b6eb&hm=77f567fdb34778838dedec47933bc472ec1856f1d8093feaf891f2f6452aa505&=&format=webp&quality=lossless&width=1227&height=669" width = "840">

## Pathing Journey
Initially, we began with using the GNSS localization and switched over to the GNSS IMU to attempt to stabilize the car's drift. Now we're using Odometry instead due to 

### LQR Control
For path tracking, like PID control, it adjusts the inputs for steering the RC Car while accounting for errors from the calculated trajectory and works with a feedback loop for continuous updating of the inputs and path deviation.

[![LQR Simulation in ROS2](http://img.youtube.com/vi/OvJ9mGfcxE4/0.jpg)](http://www.youtube.com/watch?v=OvJ9mGfcxE4 "LQR Simualtion in ROS2")

### GNSS / IMU
Using the GNSS functioned properly, however, when additionally overlaying it with the Oak-D Lite camera, the ball-finding function did not work. With both on, the position tracking was paused and the ball could not be located. We later attempted to update the firmware using the GNSS AP module that utilizes the GPS signal and GNS’s IMU but there were still issues with the simultaneous usage of the camera and the storing of location coordinates. With the IMU’s sensor fusion, we were able to detect a change in orientation better than with the AM module that does not use the IMU. Then, we tested getting the initial location coordinates with the GPS on and the camera off, then turned the camera on for ball finding. The coordinates were transformed from the camera frame to the global GPS frame and then we switched off the camera. With this, the GPS was getting readings but they were unstable and not accurate enough to map the tennis ball and retrieve it. The error was ranging from +/- 1 meter to +/- 15 meters off from the actual position. This is when we switched to Odometry. 


### Odometry Pathing  
Initially, the odometry of the car was determined by reading the rpm from VESC and subscribing to the driving message for a change in orientation based on servo position. This was successful in converting rpm to speed, but the steering angle was highly non-linear and the drive message didn't accurately reflect the turning angle in reality. This led to error buildup and loss of localization when turning. To combat this we decided to use a seed IMU instead for the orientation then add the odometry package to subscribe to an IMU topic, then integrate the change in position using linear velocity from VESC and orientation from IMU.


Overall, to start the program do the following:

1. Open 3 terminals, all ssh into the Jetson

2. On Terminal 1: docker start -ai test_container

3. On Terminal 2 & 3: docker exec -it test_container bash

4. On all 3 Terminals: source_ros2

5. On Terminal 1: ros2 launch ucsd_robocar_nav2_pkg all_nodes.launch.py

6. Wait for it to show [razor_imu_node]: Publishing IMU data...

7. On Terminal2: ros2 run team5_project find_tennis_ball --ros-args -r __ns:=/team5

8. On Terminal3: ros2 run team5_project follow_ball --ros-args -r __ns:=/team5


[![Odometry Demo](http://img.youtube.com/vi/VSpBKpGFqQQ/0.jpg)](http://www.youtube.com/watch?v=VSpBKpGFqQQ "Odometry Demo")


### CAD 

Early Grabber Mechanism Designs and Ideas: 
<p float="left">
<img src="https://media.discordapp.net/attachments/879993760937816084/1248464859385827398/image.png?ex=6663c301&is=66627181&hm=6d428517fefcfe690a2f5fd98ff99379ab4b092fa4bc75389e3ca4e9ab9ed7d0&=&format=webp&quality=lossless&width=482&height=288" width = "320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465094829019146/image.png?ex=6663c339&is=666271b9&hm=41664c44e0a79ffe2ef2c6e1e0d8bc95407864aa4c4c076e9b7d8f018741ec63&=&format=webp&quality=lossless&width=312&height=299" width = "320">


</p>

<p float="left">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465316921606235/image.png?ex=6663c36e&is=666271ee&hm=1c0ee1f97ca4425291c06dd32b8791541302cadcb91a2de84e3c6a45665462d5&=&format=webp&quality=lossless&width=306&height=257" width = "320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465615220506695/image.png?ex=6663c3b6&is=66627236&hm=bd5b181ff9e1f285fef4e4513d8f4caf359b04edb60d0ee62232aefabe716704&=&format=webp&quality=lossless&width=583&height=508" width = "300"> 

<p float="left">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465784078991360/image.png?ex=6663c3de&is=6662725e&hm=cc8bc3d95eaeb72ac6ae31aa5d6edf7789a9fd18a8ec842b3d0a96693ec93334&=&format=webp&quality=lossless&width=691&height=627" width = "300">

<img src="https://media.discordapp.net/attachments/879993760937816084/1250581603759947796/IMG_5601.jpg?ex=666b7661&is=666a24e1&hm=36307877986e9204bed57e5265c702ce9228f5cde6f8a0b3abe86a55fc11bc6e&=&format=webp&width=501&height=669" width = "250"> 
</p>

### References 

Odometry Calibration Guide: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html 

ROS Driver NasSat: https://github.com/ros-drivers/nmea_navsat_driver/tree/master 

IMU: https://www.seeedstudio.com/Seeed-XIAO-BLE-Sense-nRF52840-p-5253.html 

RazorIMU: https://github.com/NikitaB04/razorIMU_9dof/tree/main 

PointOne ROS2 Driver: https://github.com/PointOneNav/ros2-fusion-engine-driver/tree/main 

PointOne Calibration INS-RTK: https://pointonenav.com/news/calibration-for-ins-rtk/ 

PointOne Firmware: https://github.com/PointOneNav/firmware-tools/tree/main 

PointOne FusionEngine:  https://pointonenav.com/wp-content/uploads/2023/09/FusionEngine-Message-Specification-v0.18.pdf

ROS2Foxy: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html 

VESC Controller: https://github.com/f1tenth/vesc/tree/ros2 

ROS Ackermann Steering: https://github.com/ros-drivers/ackermann_msgs/tree/ros2 

PointOne Standard Dev Kit: https://pointonenav.com/wp-content/uploads/2024/05/Point-One-Standard-Dev-Kit-User-Manual-v1.23.pdf?_gl=1*boyar8*_gcl_aw*R0NMLjE3MTMxODU4NTkuQ2p3S0NBandvUE93QmhBZUVpd0FKdVhSaDU1bFFKQm5NRnZ1N3c4aGMxNjAybXlTdllZZlBxelRYU1FFLWx3Vmdpc0FvZDd6V01tZkl4b0NqUXdRQXZEX0J3RQ..*_gcl_au*MjEwNzEwMTY2MS4xNzEyMTgyNzg2 

Configure PWM on Jetson Nano: https://www.seeedstudio.com/blog/2020/05/27/configure-pwm-output-on-jetson-nano-m/ 

Python Library of NVIDIA Jetson GPIO: https://github.com/NVIDIA/jetson-gpio/tree/master 

Python Robotics: https://github.com/AtsushiSakai/PythonRobotics/tree/master 

### Acknowledgements 
Very thankful to Professor Jack Silberman, TA Arjun Naageshwaran, and the entire MAE 148 Spring 2024 Class!

