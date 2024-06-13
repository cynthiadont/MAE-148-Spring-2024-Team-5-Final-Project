<img src="https://jacobsschool.ucsd.edu/sites/default/files/groups/jsoe/img/logos/jacobs-school/digital/UCSDLogo_JSOE_BlueGold_Web.jpg" width="256">

# MAE 148 Spring 2024 Team 5 Final Project 

##  RC Car Tennis Ball Boy with Odometry and YOLO 
Team 5 - Saimai Lau - MAE, Ryan Salas - MAE, Cynthia Do - MAE, Theo Emery - MAE

<details>
<summary> Table of Contents </summary>
 
### Overview 
### RC Car 
### Main Goals
### Nice-to-Haves 
### P
### Issues and Solutions 
</details> 

# Overview
We wanted to make an RC car that would be able to fulfill the responsibilities of a ball boy in tennis matches. The responsibilities 

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
Initially, we began with using the GNSS localization and switched over to the GNSS IMU to attempt to stabilize the drift of the car. Now we're using Odometry instead due to 

### LQR Control
For path tracking, like PID control, it adjusts the inputs for steering the RC Car while accounting for error from the calculated trajectory and works with a feedback loop for continuous updating of the inputs and path deviation.  but penalizes accuator efforts, 

[![LQR Simulation in ROS2](http://img.youtube.com/vi/OvJ9mGfcxE4/0.jpg)](http://www.youtube.com/watch?v=OvJ9mGfcxE4 "LQR Simualtion in ROS2")

### GNSS / IMU
unstable with camera, tried update the firmware to use the AP module which uses the gps signal and IMU, but still had issues with the camera where the location coordinates were not stored and kept, with the imu for sensor fusion, then it detects change in orientation better htan the AM module which does not use the IMU, 
we tried getting the initial location coordinates first then turn on the camera to find the tennis ball which was transformed from camera frame into global gps frame then switched off the camera 
then the gps wasnt accurate enough to get to the tennis ball with this, error was about +/- 1 meter so we switched to odometry

### Odometry Pathing  
calirating odometry on the car by reading the rpm from VESC and suscribing to the drive message for chang in orientation, successful in convertin rpm to speed, but hte steering angle was highly non-linear and the drive message doesnt accurate reflect the turning angle in reality, so we decided to use a seed IMU for the orientation then added the odometry package to subscribe to an IMU topic, then integrate the chagne in position using linear velocity from VESC and orientation from IMU 

### CAD 

Early Grabber Mechanism Designs and Ideas: 
<p float="left">
<img src="https://media.discordapp.net/attachments/879993760937816084/1248464859385827398/image.png?ex=6663c301&is=66627181&hm=6d428517fefcfe690a2f5fd98ff99379ab4b092fa4bc75389e3ca4e9ab9ed7d0&=&format=webp&quality=lossless&width=482&height=288" width = "320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465094829019146/image.png?ex=6663c339&is=666271b9&hm=41664c44e0a79ffe2ef2c6e1e0d8bc95407864aa4c4c076e9b7d8f018741ec63&=&format=webp&quality=lossless&width=312&height=299" width = "320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465316921606235/image.png?ex=6663c36e&is=666271ee&hm=1c0ee1f97ca4425291c06dd32b8791541302cadcb91a2de84e3c6a45665462d5&=&format=webp&quality=lossless&width=306&height=257" width = "320">

</p>

<p float="left">
<img src="https://media.discordapp.net/attachments/879993760937816084/1248465615220506695/image.png?ex=6663c3b6&is=66627236&hm=bd5b181ff9e1f285fef4e4513d8f4caf359b04edb60d0ee62232aefabe716704&=&format=webp&quality=lossless&width=583&height=508" width = "300"> 

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465784078991360/image.png?ex=6663c3de&is=6662725e&hm=cc8bc3d95eaeb72ac6ae31aa5d6edf7789a9fd18a8ec842b3d0a96693ec93334&=&format=webp&quality=lossless&width=691&height=627" width = "300">

<img src="https://media.discordapp.net/attachments/879993760937816084/1250581603759947796/IMG_5601.jpg?ex=666b7661&is=666a24e1&hm=36307877986e9204bed57e5265c702ce9228f5cde6f8a0b3abe86a55fc11bc6e&=&format=webp&width=501&height=669" width = "250"> 
</p>

### References 
Odometry Calibration Guide: https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html

### Acknowledgements 
