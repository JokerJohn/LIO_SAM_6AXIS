# LIO_SAM_6AXIS

This repo may help to adapt LIO_SAM for your own sensors! It has some changes comparing with the origin system.

- support a 6-axis IMU, since the orientation information of IMU is not used in state estimation module.
- support normal GNSS, we do not need to adapt for the robot_localization node.
- support the gps constraint visualization module to help debugging the normal GNSS.(the following picture)

<img align="center" src="README/test-16548681575981.gif" style="zoom: 200%;" />

<img src="README/image-20220531015953876.png" alt="image-20220531015953876" style="zoom: 50%;" />

## Latest News(2022-06-08)

- More accurate origin LLA point.
- Support KML file and we can see the trajectory in GoogleMaps.
- Add video tutorial 

<img src="README/image-20220609035032131.png" alt="image-20220609035032131" style="zoom: 67%;" />

# Introduction

LIO_SAM is only designed for 9-axis IMU, for the following reasons.

- the initialization module need absolute orientation to initialize the LIO system.
- the back-end GNSS-based optimization relies on the robot_localization node, and also requires a 9-axis IMU.

Therefore, only minor changes to the original code are required.  which can directly use GPS points of good quality for optimization. Finally, we also made some explanations for some common lidars, as well as coordinate system adaptation and extrinsics between lidars and IMUs, such as Hesai.

we add the gps constraint visualization module to help debugging the normal gps(red lines represents for gps constraint).

<img src="README/image-20220421113413972.png" alt="GPS constrain visualization" style="zoom:50%;" />

# Run

## Dependences

the same as LIO_SAM.

my previous and current system: 

`Ubuntu18.04 /PCL1.8/OpenCV4.5/GTSAM4.0.2`

`Ubuntu20.04/PCL1.10/OpenCV3.4.16/GTSAM4.1`

## Simple Video Tutorial

Whether you are running the sample data provided by me or adapting your own sensor, you can watch the detailed teaching video below.

**Video Tutorial**：[Bilibili](https://www.bilibili.com/video/BV1YS4y1i7nX/)、[Youtube](https://youtu.be/TgKSeNLkExc)


## Single Sequence

when you set `useGPS` as true,  remember to test the params `gpsCovThreshold`. Just **make sure your vehicles are in a good position at the first beginning of the sequence where the status of GNSS is stable encough**, or you can not initialize your system successfully! 
[Gps initialization video](https://www.bilibili.com/video/BV1dY411M7hr/)

```
roslaunch lio_sam_6axis run.launch
```

when you set the `useGPS` true, you may get the following los. It means that these gps points are used for optimization.

```bash
[ INFO] [1651092699.914940274]: curr gps cov: 11.022400, 11.022400 , 176.358400
[ INFO] [1651092700.516013418]: curr gps pose: 13.806815, 7.928380 , 5.147980
[ INFO] [1651092700.516037958]: curr gps cov: 11.022400, 11.022400 , 176.358400
[ INFO] [1651092700.516045476]: curr gps pose: 13.868968, 8.179937 , 4.978980
[ INFO] [1651092700.516052422]: curr gps cov: 11.022400, 11.022400 , 176.358400
```

## Batch Scripts

when you want to test on multi-sequence rosbag with the same set of sensor equipment. You just need to modify the script `LIO-SAM-6AXIS/scripts/lio_loop_batch.py`.

1. put all your rosbag info one folder , and set it as the `bag_path_download`params. set your rosbag file name(`bag_path_list`)
2. set your sequence name (`plat_data_pair_list`) 
3. source your workspace(`source devel/setup.zsh` )

4. run the script

```python
python3 src/LIO-SAM-6AXIS/scripts/lio_loop_batch.py
```

## Save Results

I will give the map and related example results constructed based on the instance data using LIO_SAM_6AXIS, once the sharing function of Baidu netdisk is normal.

```bash
rosservice call /lio_sam_6axis/save_map
```

- `campus_result.bag`: inlcude 2 topics, the distorted point cloud and the optimzed odometry

- `odom_tum.txt`

- `optimized_odom_kitti.txt`

- `optimized_odom_tum.txt`

- `pose_graph.g2o`: the final pose graph g2o file

- `globalmap_lidar.pcd`: global map in lidar frame.

- `globalmap_imu.pcd`: global map in IMU body frame, but you need to set proper extrinsics.

- `globalmap_lidar_feature.pcd`: edge+planer points map, based on lidar frame.

- `origin.txt`: The origin of the point cloud map, which can be used for prior map-based localization.  

- `optimized_gps_trajectry.kml`: KML file for optimized trajectory, you can show it in GoogleMaps.

<img src="README/image-20220609044824460.png" alt="image-20220609044824460" style="zoom: 80%;" />

# Dataset and Adaption

see [this doc](doc/adaption.md).

# TO DO

1. colored point cloud  map
2. dynamic object removal
3. Using GNSS Raw Observations

As soon as I have time I will continue to update this repo and release more data.

# Acknowledgments

Thanks to  [Guoqing Zhang](https://github.com/MyEvolution), [Jianhao Jiao](https://github.com/gogojjh).

Thanks to [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM).



