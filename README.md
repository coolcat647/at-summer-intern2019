# at-summer-intern2019
This repository is prepared for the AT team of ARG summer intern. The materials include NodeMCU, Haptic device design, TrailNet and Guiding robot v1

## Setup repo
```bash
$ git clone https://github.com/coolcat647/at-summer-intern2019
$ cd at-summer-intern2019
$ git submodule init && git submodule update
```

## Lecture
### 01-play-with-NodeMCU
This section introduces **NodeMCU**, and its communication method with **ROS**. We will refer concept of ROS topic through arduino code examples. Here 's the [Google document link](https://docs.google.com/document/d/1wuj0i9Z-HVB7Gg2elkC2kIU19ypu7MpqO-8UQTSBxZQ/edit?usp=sharing)
- The example 1 will let NodeMCU connect to ROS system via Wi-Fi, and implement both 1 publisher and 1 subscriber on NodeMCU.
- The example 2 will introduce TF and implement fake /odom to /base_link transform. The fake robot with try to drive in circle.

### 02-ros-haptic-device
This section show the **rospy** example code for publishing fake vibration command to NodeMCU, and converting ros message to vibration.

### 03-ros-tutorial1
This section we took **Sensing and Intelligent System** course's repository as reference to demonstrate the roscpp and rospy formally.

### 04-trail-following-robot
This section we introduce the **Docker**, **Jupyter notebook** and demonstate deep learing code with **Pytorch** , then create ROS workspace for deep trail-following robot. The workspace included sensing, deep learning and robot control part. 
