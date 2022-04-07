# LIO_SAM_6AXIS
LIO_SAM for 6 axis IMU and normal GNSS.

Sincen LIO_SAM is only suitable for 9-axis IMU, this is mainly based on the initialization module consideration, but the orientation information of IMU is not used in state estimation. Therefore, only minor changes to the original code are required. In addition, the back-end GNSS-based position optimization relies on the robot_localization node, and also requires a 9-axis IMU, which can directly use GPS coordinates of good quality for optimization. Finally, we also made some explanations for some common lidars, as well as coordinate system adaptation and external parameters between lidars and IMUs, such as Hesai lidars.

# Run

`roslaunch lio_sam_6axis pandar.launch`



## Acknowledgments

Thanks for [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM)



