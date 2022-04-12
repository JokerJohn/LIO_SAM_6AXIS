# LIO_SAM_6AXIS
LIO_SAM for 6 axis IMU and normal GNSS.

Since LIO_SAM is only suitable for 9-axis IMU, this is mainly based on the initialization module consideration, but the orientation information of IMU is not used in state estimation. Therefore, only minor changes to the original code are required. In addition, the back-end GNSS-based position optimization relies on the robot_localization node, and also requires a 9-axis IMU, which can directly use GPS coordinates of good quality for optimization. Finally, we also made some explanations for some common lidars, as well as coordinate system adaptation and external parameters between lidars and IMUs, such as Hesai lidars.

# Run

`roslaunch lio_sam_6axis pandar.launch`



## Adaptation for 6 axis IMU

Because LIO_SAM only uses the acceleration and angular velocity in the IMU data to estimate the system state, and its orientation data only initializes the orientation of the system in the back-end optimization module, it is very simple to adapt to the 6-axis IMU. On the premise that the IMU coordinate system and the radar coordinate system are aligned, it is only necessary to modify the imuConverter function. The specific method is to modify the `imuConverter` function in the `LIO-SAM-6AXIS/include/utility.h` file, as follows.

```c++
  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
    // we only need to align the coordinate system.
    // Eigen::Quaterniond q_final = q_from * extQRPY;
    Eigen::Quaterniond q_final = extQRPY;
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(
        q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w())
        < 0.1) {
      ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
      ros::shutdown();
    }

    return imu_out;
  }
```

`q_final` represents the orientation result taken from the IMU data. For the 6-axis IMU, we generally integrate our own in the SLAM system instead of directly adopting this result. But in the 9-axis IMU, the orentation part is the global attitude, that is, the result of the fusion with the magnetometer, which represents the angle between the IMU and the magnetic north pole. So here we only need to align the coordinate axis of the orentation data with the base_link, that is, multiply the lidar to the external parameter of the IMU.

## Adaptation for different types of lidar

Make sure the point cloud timestamp, ring channel is ok. In addition, the lidar coordinate system should meet the REP105 standard, that is, xyz represents the front and upper right direction. Note that the structure of the point cloud is related to your lidar driver, and may not be exactly the same. The format below is for reference only.

### Pandar

The coordinate system of Hesai LiDAR is different from that of the traditional REP105, and its xyz represent the left 、back and top respectively.

```c++
struct PandarPointXYZIRT {
  PCL_ADD_POINT4D
  float intensity;
  double timestamp;
  uint16_t ring;                      ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PandarPointXYZIRT,
(float, x, x)
(float, y, y)
(float, z, z)
(float, intensity, intensity)
(double, timestamp, timestamp)
(uint16_t, ring, ring)
)
```

### Ouster

The point cloud timestamp of ouster lidar is not necessarily ustc time。

```c++
struct OusterPointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
//  uint32_t time;
  uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;  // additional property of p.ouster
  float time;
  uint16_t noise;
  uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
(float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
(uint16_t, reflectivity, reflectivity)
(uint8_t, ring, ring)
(std::uint16_t, ambient, ambient)
(float, time, time)
(uint16_t, noise, noise)
(uint32_t, range, range)
)
```

## Acknowledgments

Thanks for  [Guoqing Zhang](https://github.com/MyEvolution)  from Tsinghua University.

Thanks for [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM).



