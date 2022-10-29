
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <deque>
#include <mutex>
#include <queue>

#include "gpsTools.hpp"
#include "ros/ros.h"
#include "utility.h"

class GNSSOdom : public ParamServer {
 public:
  GNSSOdom(ros::NodeHandle &_nh) {
    nh = _nh;
    gpsSub = nh.subscribe(gpsTopic, 100, &GNSSOdom::GNSSCB, this,
                          ros::TransportHints().tcpNoDelay());
    gpsOdomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
    fusedPathPub = nh.advertise<nav_msgs::Path>("/gps_path", 100);
  }

 private:
  void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
    // gps status
    // std::cout << "gps status: " << msg->status.status << std::endl;
    if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
      ROS_ERROR("POS LLA NAN...");
      return;
    }
    double gps_time = msg->header.stamp.toSec();
    Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
    std::cout << "LLA: " << lla.transpose() << std::endl;
    if (!initXyz) {
      ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude,
               msg->altitude);
      gtools.lla_origin_ = lla;
      initXyz = true;
      return;
    }

    //  convert  LLA to XYZ
    Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
    Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
    ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

    // sometimes you may get a wrong origin at the beginning
    if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000) {
      ROS_INFO("Error ogigin : %f, %f, %f", enu(0), enu(1), enu(2));
      ResetOrigin(lla);
      return;
    }

    // maybe you need to get the extrinsics between your gnss and imu
    // most of the time, they are in the same frame
    Eigen::Vector3d calib_enu = enu;

    double distance =
        sqrt(pow(enu(1) - prevPos(1), 2) + pow(enu(0) - prevPos(0), 2));
    if (distance > 0.1) {
      // 返回值是此点与远点连线与x轴正方向的夹角
      yaw = atan2(enu(1) - prevPos(1), enu(0) - prevPos(0));
      yawQuat = tf::createQuaternionMsgFromYaw(yaw);
      prevPos = enu;
      prevYaw = yaw;

      orientationReady_ = true;
      if (!firstYawInit) {
        ROS_WARN("INIT YAW SUCCESS!!!!!!!!!!!!!!!!!!!!");
        firstYawInit = true;
        ResetOrigin(lla);
        prevYaw = yaw;
      }
      ROS_INFO("gps yaw : %f", yaw);
    } else {
      orientationReady_ = false;
      return;
    }

    // make sure your initial yaw and origin postion are consistent
    if (!firstYawInit || !orientationReady_) {
      ROS_ERROR("waiting init origin yaw");
      return;
    }

    // pub gps odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = odometryFrame;
    odom_msg.child_frame_id = "gps";

    // ----------------- 1. use utm -----------------------
    //        odom_msg.pose.pose.position.x = utm_x - origin_utm_x;
    //        odom_msg.pose.pose.position.y = utm_y - origin_utm_y;
    //        odom_msg.pose.pose.position.z = msg->altitude - origin_al;

    // ----------------- 2. use enu -----------------------
    odom_msg.pose.pose.position.x = calib_enu(0);
    odom_msg.pose.pose.position.y = calib_enu(1);
    odom_msg.pose.pose.position.z = calib_enu(2);
    odom_msg.pose.covariance[0] = msg->position_covariance[0];
    odom_msg.pose.covariance[7] = msg->position_covariance[4];
    odom_msg.pose.covariance[14] = msg->position_covariance[8];
    odom_msg.pose.covariance[1] = lla[0];
    odom_msg.pose.covariance[2] = lla[1];
    odom_msg.pose.covariance[3] = lla[2];
    odom_msg.pose.covariance[4] = msg->status.status;
    // if (orientationReady_)
    odom_msg.pose.pose.orientation = yawQuat;
    //    else {
    //      // when we do not get the proper yaw, we set it NAN
    //      geometry_msgs::Quaternion quaternion =
    //          tf::createQuaternionMsgFromYaw(NAN);
    //    }
    gpsOdomPub.publish(odom_msg);

    // publish path
    rospath.header.frame_id = odometryFrame;
    rospath.header.stamp = msg->header.stamp;
    geometry_msgs::PoseStamped pose;
    pose.header = rospath.header;
    pose.pose.position.x = calib_enu(0);
    pose.pose.position.y = calib_enu(1);
    pose.pose.position.z = calib_enu(2);
    pose.pose.orientation.x = yawQuat.x;
    pose.pose.orientation.y = yawQuat.y;
    pose.pose.orientation.z = yawQuat.z;
    pose.pose.orientation.w = yawQuat.w;
    rospath.poses.push_back(pose);
    fusedPathPub.publish(rospath);
  }

  void ResetOrigin(Eigen::Vector3d &_lla) { gtools.lla_origin_ = _lla; }

  ros::NodeHandle nh;
  GpsTools gtools;

  ros::Publisher gpsOdomPub, fusedPathPub;
  ros::Subscriber gpsSub;

  std::mutex mutexLock;
  std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

  bool orientationReady_ = false;
  bool initXyz = false;
  bool firstYawInit = false;
  Eigen::Vector3d prevPos;
  double yaw = 0.0, prevYaw = 0.0;
  geometry_msgs::Quaternion yawQuat;
  nav_msgs::Path rospath;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lio_sam_6axis");
  ros::NodeHandle nh;
  GNSSOdom gps(nh);
  ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
  ros::spin();
  return 1;
}