# Livox_Camera_vanishing_calib
## Extrinsic Calibration of High REsolution LiDAR and Camera Based on Vanishing Points
LCVC (Livox_Camera_Vanishing_Calib) is a project used for extrinsic calibration between LiDAR and cameras. This method is a target-less calibration method that does not require an optimization process. It utilizes the spatial geometric constraints expressed by the shared vanishing points in both the LiDAR's projected image and the camera image to achieve the calibration. 

In theory, if accurate vanishing points are extracted, this project can achieve precise extrinsic calibration. The current version of the project uses line extraction to identify vanishing points in the scene and serves as a test version of the algorithm. A maintained version will be available soon.

**Authors:** [YiLin Zhao], [Fengli Yang], [Wangfang Li], [Yi Sun], and [Long Zhao] from the [BUAA Digital Navigation Center]

## Demo Video

<a href="https://youtu.be/17XT2uk6Mf8" target="_blank"><img src="https://i9.ytimg.com/vi_webp/17XT2uk6Mf8/mq1.webp?sqp=COD-5bUG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGGUgZShlMA8=&rs=AOn4CLCjSJ0h13AynXrP8y_Vx3Kdyo257g" alt="cla" width="480"  border="10" /></a>

