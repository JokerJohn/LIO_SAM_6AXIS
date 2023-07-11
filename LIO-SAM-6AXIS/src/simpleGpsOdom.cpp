
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geoid.hpp>
#include <deque>
#include <mutex>

//#include "gpsTools.hpp"
#include "ros/ros.h"
#include "utility.h"

class GNSSOdom : public ParamServer {
public:
    GNSSOdom(ros::NodeHandle &_nh) {
        nh = _nh;
        gpsSub = nh.subscribe(gpsTopic, 100, &GNSSOdom::GNSSCB, this,
                              ros::TransportHints().tcpNoDelay());
        left_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
        init_origin_pub = nh.advertise<nav_msgs::Odometry>("/init_odom", 10000, false);
        left_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 100);
    }

private:
    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
        // gps status
        // std::cout << "gps status: " << msg->status.status << std::endl;
/*    if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
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
    }*/


        std::cout << "gps status: " << msg->status.status << std::endl;
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            ROS_ERROR("POS LLA NAN...");
            return;
        }
        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        std::cout << "LLA: " << lla.transpose() << std::endl;
        if (!initXyz) {
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude,
                     msg->altitude);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initXyz = true;

            nav_msgs::Odometry init_msg;
            init_msg.header.stamp = msg->header.stamp;
            init_msg.header.frame_id = odometryFrame;
            init_msg.child_frame_id = "gps";
            init_msg.pose.pose.position.x = lla[0];
            init_msg.pose.pose.position.y = lla[1];
            init_msg.pose.pose.position.z = lla[2];
            init_msg.pose.covariance[0] = msg->position_covariance[0];
            init_msg.pose.covariance[7] = msg->position_covariance[4];
            init_msg.pose.covariance[14] = msg->position_covariance[8];
            init_msg.pose.pose.orientation = yaw_quat_left;
            init_origin_pub.publish(init_msg);
            return;
        }

        int status = -1;
        int satell_num = -1;
        std::cout << "simple rtk: " << status << " " << satell_num << std::endl;

//        GeographicLib::Geoid edm2008("egm2008-5", edm_path);
//        GeographicLib::Geoid edm2008("egm2008-5");
//        double h = edm2008(lla[0], lla[1]); // 这将返回高度的校正信息，单位是米
//        double correction_h = (lla[2] + Geoid::GEOIDTOELLIPSOID * h);
//        lla[2] = correction_h;

        //  convert  LLA to XYZ
//        Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
//        Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
//        ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));
        double x, y, z;
        // 将LLA转换为ENU
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
//        ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));
        Eigen::Vector3d enu(x, y, z);
        // sometimes you may get a wrong origin at the beginning
        if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000) {
            ROS_INFO("Error ogigin : %f, %f, %f", enu(0), enu(1), enu(2));
            //ResetOrigin(lla);
            return;
        }

        // maybe you need to get the extrinsics between your gnss and imu
        // most of the time, they are in the same frame
        //Eigen::Vector3d calib_enu = enu;
        bool orientationReady = false;
        double yaw = 0.0;
        double distance =
                sqrt(pow(enu(1) - prev_pose_left(1), 2) + pow(enu(0) - prev_pose_left(0), 2));
        if (distance > 0.1) {
            // 返回值是此点与远点连线与x轴正方向的夹角
            yaw = atan2(enu(1) - prev_pose_left(1), enu(0) - prev_pose_left(0));
            yaw_quat_left = tf::createQuaternionMsgFromYaw(yaw);
            prev_pose_left = enu;
            orientationReady = true;
        }

/*        // pub gps odometry
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
        fusedPathPub.publish(rospath);*/

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
        odom_msg.pose.pose.position.x = enu(0);
        odom_msg.pose.pose.position.y = enu(1);
        odom_msg.pose.pose.position.z = enu(2);
        odom_msg.pose.covariance[0] = msg->position_covariance[0];
        odom_msg.pose.covariance[7] = msg->position_covariance[4];
        odom_msg.pose.covariance[14] = msg->position_covariance[8];
        odom_msg.pose.covariance[1] = lla[0];
        odom_msg.pose.covariance[2] = lla[1];
        odom_msg.pose.covariance[3] = lla[2];
        odom_msg.pose.covariance[4] = status;
        odom_msg.pose.covariance[5] = satell_num;
        odom_msg.pose.covariance[6] = orientationReady;
        odom_msg.pose.pose.orientation = yaw_quat_left;
        left_odom_pub.publish(odom_msg);

        // publish path
        left_path.header.frame_id = odometryFrame;
        left_path.header.stamp = msg->header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = left_path.header;
        pose.pose.position.x = enu(0);
        pose.pose.position.y = enu(1);
        pose.pose.position.z = enu(2);
        pose.pose.orientation.x = yaw_quat_left.x;
        pose.pose.orientation.y = yaw_quat_left.y;
        pose.pose.orientation.z = yaw_quat_left.z;
        pose.pose.orientation.w = yaw_quat_left.w;
        left_path.poses.push_back(pose);
        left_path_pub.publish(left_path);
    }

    //void ResetOrigin(Eigen::Vector3d &_lla) { gtools.lla_origin_ = _lla; }

    ros::NodeHandle nh;
//    GpsTools gtools;

    ros::Publisher left_odom_pub, left_path_pub, init_origin_pub;
    ros::Subscriber gpsSub;

    std::mutex mutexLock;
    std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

    //bool orientationReady_ = false;
    bool initXyz = false;
//    bool firstYawInit = false;
//    Eigen::Vector3d prevPos;
//    double yaw = 0.0, prevYaw = 0.0;
//    geometry_msgs::Quaternion yawQuat;
    nav_msgs::Path left_path;
    GeographicLib::LocalCartesian geo_converter;
    Eigen::Vector3d prev_pose_left, prev_pose_right;
    geometry_msgs::Quaternion yaw_quat_left;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lio_sam_6axis");
    ros::NodeHandle nh;
    GNSSOdom gps(nh);
    ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
    ros::spin();
    return 1;
}