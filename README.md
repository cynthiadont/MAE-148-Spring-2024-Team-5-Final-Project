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
- Not get hit

### Main Hardware Utilized 
- Jetson Nano
- OAK-D Camera LITE 
- GNSS 
- VESC Motor Controller 

### Ball Recognition with YOLO & ROS2 Packages
yes

## Pathing Journey
Initially, we began with using the GNSS localization and switched over to the GNSS IMU to attempt to stabilize the drift of the car. Now we're using Odometry instead due to 

### GNSS / IMU

### LQR 

### Odometry Pathing  
so true

### CAD 

Early Grabber Mechanism Designs and Ideas: 
<p float="left">
<img src="https://media.discordapp.net/attachments/879993760937816084/1248464859385827398/image.png?ex=6663c301&is=66627181&hm=6d428517fefcfe690a2f5fd98ff99379ab4b092fa4bc75389e3ca4e9ab9ed7d0&=&format=webp&quality=lossless&width=482&height=288" width = 320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465094829019146/image.png?ex=6663c339&is=666271b9&hm=41664c44e0a79ffe2ef2c6e1e0d8bc95407864aa4c4c076e9b7d8f018741ec63&=&format=webp&quality=lossless&width=312&height=299" width = "320">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465316921606235/image.png?ex=6663c36e&is=666271ee&hm=1c0ee1f97ca4425291c06dd32b8791541302cadcb91a2de84e3c6a45665462d5&=&format=webp&quality=lossless&width=306&height=257" width = "320">

</p>

<p float="left">
<img src="https://media.discordapp.net/attachments/879993760937816084/1248465615220506695/image.png?ex=6663c3b6&is=66627236&hm=bd5b181ff9e1f285fef4e4513d8f4caf359b04edb60d0ee62232aefabe716704&=&format=webp&quality=lossless&width=583&height=508" width = "300"> 

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465784078991360/image.png?ex=6663c3de&is=6662725e&hm=cc8bc3d95eaeb72ac6ae31aa5d6edf7789a9fd18a8ec842b3d0a96693ec93334&=&format=webp&quality=lossless&width=691&height=627" width = "300">

<img src="https://media.discordapp.net/attachments/879993760937816084/1248465935971520603/image.png?ex=6663c402&is=66627282&hm=501ed8f4bcc4f8e0e7052f0c0eaaa6370fc6f4e306b83fb6f84d6435881c96f3&=&format=webp&quality=lossless&width=449&height=400" width = "300"> 
</p>


### Acknowledgements 
