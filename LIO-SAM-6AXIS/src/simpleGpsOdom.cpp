#include "ros/ros.h"
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

#include "utility.h"

class GNSSOdom : public ParamServer {
public:
    GNSSOdom(ros::NodeHandle &_nh) {
        nh = _nh;
        gpsSub = nh.subscribe(gpsTopic, 1000, &GNSSOdom::GNSSCB, this,
                              ros::TransportHints().tcpNoDelay());
        left_odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 100, false);
        init_origin_pub = nh.advertise<nav_msgs::Odometry>("/init_odom", 10000, false);
        left_path_pub = nh.advertise<nav_msgs::Path>("/gps_path", 100);
    }

private:
    void GNSSCB(const sensor_msgs::NavSatFixConstPtr &msg) {
        std::cout << "gps status: " << msg->status.status << std::endl;
        if (std::isnan(msg->latitude + msg->longitude + msg->altitude)) {
            return;
        }
        Eigen::Vector3d lla(msg->latitude, msg->longitude, msg->altitude);
        //std::cout << "LLA: " << lla.transpose() << std::endl;
        if (!initENU) {
            ROS_INFO("Init Orgin GPS LLA  %f, %f, %f", msg->latitude, msg->longitude,
                     msg->altitude);
            geo_converter.Reset(lla[0], lla[1], lla[2]);
            initENU = true;

            /** publish initial pose from GNSS ENU Frame*/
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

        /** if you have some satellite info or rtk status info, put it here*/
        int status = -1;
        int satell_num = -1;
        double x, y, z;
        // LLA->ENU, better accuacy than gpsTools especially for z value
        geo_converter.Forward(lla[0], lla[1], lla[2], x, y, z);
        Eigen::Vector3d enu(x, y, z);
        if (abs(enu.x()) > 10000 || abs(enu.x()) > 10000 || abs(enu.x()) > 10000) {
            /** check your lla coordinate */
            ROS_INFO("Error ogigin : %f, %f, %f", enu(0), enu(1), enu(2));
            return;
        }

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

        /** pub gps odometry*/
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = odometryFrame;
        odom_msg.child_frame_id = "gps";

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


        /** just for gnss visualization */
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


    ros::NodeHandle nh;
    ros::Publisher left_odom_pub, left_path_pub, init_origin_pub;
    ros::Subscriber gpsSub;

    std::mutex mutexLock;
    std::deque<sensor_msgs::NavSatFixConstPtr> gpsBuf;

    bool initENU = false;
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