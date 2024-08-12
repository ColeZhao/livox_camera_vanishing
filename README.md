# Livox_Camera_Vanishing_Calib
## Extrinsic Calibration of High REsolution LiDAR and Camera Based on Vanishing Points
LCVC (Livox_Camera_Vanishing_Calib) is a project used for extrinsic calibration between LiDAR and cameras. This method is a target-less calibration method that does not require an optimization process. It utilizes the spatial geometric constraints expressed by the shared vanishing points in both the LiDAR's projected image and the camera image to achieve the calibration. 

In theory, if accurate vanishing points are extracted, this project can achieve precise extrinsic calibration. The current version of the project uses line extraction to identify vanishing points in the scene and serves as a test version of the algorithm. A maintained version will be available soon.

**Authors:** [YiLin Zhao](https://github.com/ColeZhao), Fengli Yang, Wangfang Li, Yi Sun, and Long Zhao from the [BUAA Digital Navigation Center](https://dnc.buaa.edu.cn/)

**Related Papers**

* **Extrinsic Calibration of High REsolution LiDAR and Camera Based on Vanishing Points**, Yilin Zhao, Long Zhao, TIM

* **Closed-Form Solution of Principal Line for Camera Calibration Based on Orthogonal Vanishing Points**, FengLi Yang, Long Zhao, IEEE TCSVT
## Demo Video

<a href="https://youtu.be/17XT2uk6Mf8" target="_blank"><img src="https://i9.ytimg.com/vi_webp/17XT2uk6Mf8/mq1.webp?sqp=COD-5bUG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGGUgZShlMA8=&rs=AOn4CLCjSJ0h13AynXrP8y_Vx3Kdyo257g" alt="cla" width="480"  border="10" /></a>

## 1.Requirement
1.1 **Ubuntu** and **ROS1**
Ubuntu 64-bit and ROS1. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2 **Eigen**

1.3 **Ceres**

1.4 **PCL**

## 2. Build
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/ColeZhao/livox_camera_vanishing.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run Our Example

## 4. Acknowledgements
The code for converting raw LiDAR data to PCD format is based on the LCC [(Livox_Camera _Calib)](https://github.com/hku-mars/livox_camera_calib) algorithm.

## 5. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.