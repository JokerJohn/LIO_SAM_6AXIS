
#include <queue>
#include <deque>
#include <mutex>

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "utility.h"
#include "gpsTools.hpp"

class GNSSOdom {
public:
    GNSSOdom() : nh("~") {
        gpsSub = nh.subscribe("/fix", 100, &GNSSOdom::GNSSCB, this, ros::TransportHints().tcpNoDelay());
        gpsOdomPub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
        fusedPathPub = nh.advertise<nav_msgs::Path>("/fused_gps_path", 100);
    }

private:
    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            ROS_ERROR("POS LLA NAN...");
            return;
        }
        if (!initXyz) {
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude, msg->altitude);
            gtools.lla_origin_ << msg->latitude, msg->longitude, msg->altitude;
            initXyz = true;
        }
        if (!initXyz) {
            ROS_ERROR("waiting init origin axis");
            return;
        }
        double gps_time = msg->header.stamp.toSec();
        //  convert  LLA to XYZ
        Eigen::Vector3d lla = gtools.GpsMsg2Eigen(*msg);
        Eigen::Vector3d ecef = gtools.LLA2ECEF(lla);
        Eigen::Vector3d enu = gtools.ECEF2ENU(ecef);
        ROS_INFO("GPS ENU XYZ : %f, %f, %f", enu(0), enu(1), enu(2));

        // maybe you need to get the extrinsics between your gnss and imu
        // most of the time, they are in the same frame
        Eigen::Vector3d calib_enu = enu;

        // caculate yaw
        //bool orientation_ready_ = false;
        double distance = sqrt(pow(enu(1) - prevPos(1), 2) +
                               pow(enu(0) - prevPos(0), 2));
        if (distance > 0.1) {
            yaw = atan2(enu(1) - prevPos(1),
                        enu(0) - prevPos(0)); // 返回值是此点与远点连线与x轴正方向的夹角
            yawQuat = tf::createQuaternionMsgFromYaw(yaw);
            prevPos = enu;
            prevYaw = yaw;
            orientationReady_ = true;
            if (!firstYawInit) {
                firstYawInit = true;
                gtools.lla_origin_ << msg->latitude, msg->longitude, msg->altitude;
            }
            ROS_INFO("gps yaw : %f", yaw);
        } else
            orientationReady_ = false;

        // make sure your initial yaw and origin postion are consistent
        if (!firstYawInit) {
            // ROS_ERROR("waiting init origin yaw");
            return;
        }


        // pub gps odometry
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map";
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
        if (orientationReady_)
            odom_msg.pose.pose.orientation = yawQuat;
        else {
            // when we do not get the proper yaw, we set it NAN
            geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(NAN);
        }
        gpsOdomPub.publish(odom_msg);

        // publish path
        rospath.header.frame_id = "map";
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
    GNSSOdom gps;
    ROS_INFO("\033[1;32m----> Simple GPS Odmetry Started.\033[0m");
    ros::spin();
    return 0;
}