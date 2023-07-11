//
// Created by xchu on 2022/5/19.
//

#include "dataSaver.h"

DataSaver::DataSaver() {}

DataSaver::~DataSaver() {}

DataSaver::DataSaver(string _base_dir, string _sequence_name) {
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/') {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';
    std::cout << "SAVE DIR:" << save_directory << std::endl;

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());
}

void DataSaver::saveOptimizedVerticesTUM(gtsam::Values _estimates) {
    std::fstream stream(save_directory + "optimized_odom_tum.txt",
                        std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < _estimates.size(); i++) {
        auto &pose = _estimates.at(i).cast<gtsam::Pose3>();
        gtsam::Point3 p = pose.translation();
        gtsam::Quaternion q = pose.rotation().toQuaternion();
        stream << keyframeTimes.at(i) << " " << p.x() << " " << p.y() << " "
               << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
               << q.w() << std::endl;
    }
}

void DataSaver::setDir(string _base_dir, string _sequence_name) {
    this->base_dir = _base_dir;
    this->sequence_name = _sequence_name;

    if (_base_dir.back() != '/') {
        _base_dir.append("/");
    }
    save_directory = _base_dir + sequence_name + '/';

    auto unused = system((std::string("exec rm -r ") + save_directory).c_str());
    unused = system((std::string("mkdir -p ") + save_directory).c_str());
}

void DataSaver::setConfigDir(string _config_dir) {
    if (_config_dir.back() != '/') {
        _config_dir.append("/");
    }
    this->config_directory = _config_dir;
}

void DataSaver::setExtrinc(bool _use_imu, Eigen::Vector3d _t_body_sensor,
                           Eigen::Quaterniond _q_body_sensor) {
    this->use_imu_frame = _use_imu;
    this->t_body_sensor = _t_body_sensor;
    this->q_body_sensor = _q_body_sensor;
}

void DataSaver::saveOptimizedVerticesKITTI(gtsam::Values _estimates) {
    std::fstream stream(save_directory + "optimized_odom_kitti.txt",
                        std::fstream::out);
    stream.precision(15);
    for (const auto &key_value : _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3> *>(&key_value.value);
        if (!p) continue;

        const Pose3 &pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1);  // Point3
        auto col2 = R.column(2);  // Point3
        auto col3 = R.column(3);  // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
               << " " << col1.y() << " " << col2.y() << " " << col3.y() << " "
               << t.y() << " " << col1.z() << " " << col2.z() << " " << col3.z()
               << " " << t.z() << std::endl;
    }
}

void DataSaver::saveOdometryVerticesKITTI(std::string _filename) {
    //  std::fstream stream(_filename.c_str(), std::fstream::out);
    //  stream.precision(15);
    //  for (const auto &_pose6d: keyframePoses) {
    //    gtsam::Pose3 pose = Pose6DtoGTSAMPose3(_pose6d);
    //    Point3 t = pose.translation();
    //    Rot3 R = pose.rotation();
    //    auto col1 = R.column(1); // Point3
    //    auto col2 = R.column(2); // Point3
    //    auto col3 = R.column(3); // Point3
    //
    //    stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x()
    //    << " "
    //           << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y()
    //           << " "
    //           << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z()
    //           << std::endl;
    //  }
}

void DataSaver::saveOriginGPS(Eigen::Vector3d gps_point) {
    std::fstream originStream(save_directory + "origin.txt", std::fstream::out);
    originStream.precision(15);
    originStream << gps_point[0] << " " << gps_point[1] << " " << gps_point[2]
                 << std::endl;
    originStream.close();
}

void DataSaver::saveTimes(vector<double> _keyframeTimes) {
    if (_keyframeTimes.empty()) {
        //    LOG(ERROR) << "EMPTY KEYFRAME TIMES!";
        return;
    }
    this->keyframeTimes = _keyframeTimes;
    std::fstream pgTimeSaveStream(save_directory + "times.txt",
                                  std::fstream::out);
    pgTimeSaveStream.precision(15);
    // save timestamp
    for (auto const timestamp : keyframeTimes) {
        pgTimeSaveStream << timestamp << std::endl;
    }
    pgTimeSaveStream.close();
}

void DataSaver::saveOdometryVerticesTUM(
        std::vector<nav_msgs::Odometry> keyframePosesOdom) {
    std::fstream stream(save_directory + "odom_tum.txt", std::fstream::out);
    stream.precision(15);
    for (int i = 0; i < keyframePosesOdom.size(); i++) {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();
        // check the size of keyframeTimes
        stream << time << " " << odometry.pose.pose.position.x << " "
               << odometry.pose.pose.position.y << " "
               << odometry.pose.pose.position.z << " "
               << odometry.pose.pose.orientation.x << " "
               << odometry.pose.pose.orientation.y << " "
               << odometry.pose.pose.orientation.z << " "
               << odometry.pose.pose.orientation.w << std::endl;
    }
}

void DataSaver::saveGraphGtsam(gtsam::NonlinearFactorGraph gtSAMgraph,
                               gtsam::ISAM2 *isam,
                               gtsam::Values isamCurrentEstimate) {
    gtsam::writeG2o(gtSAMgraph, isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    gtsam::writeG2o(isam->getFactorsUnsafe(), isamCurrentEstimate,
                    save_directory + "pose_graph.g2o");
    //  LOG(INFO) << "WRITE G2O FILE: " << save_directory + "pose_graph.g2o";
    //  LOG(INFO) << "Variable size: " << isamCurrentEstimate.size();
    //  LOG(INFO) << "Nonlinear factor size: " << isam->getFactorsUnsafe().size();
}

void DataSaver::saveGraph(std::vector<nav_msgs::Odometry> keyframePosesOdom) {
    std::fstream g2o_outfile(save_directory + "odom.g2o", std::fstream::out);
    g2o_outfile.precision(15);
    // g2o_outfile << std::fixed << std::setprecision(9);

    for (int i = 0; i < keyframePosesOdom.size(); i++) {
        nav_msgs::Odometry odometry = keyframePosesOdom.at(i);
        double time = odometry.header.stamp.toSec();

        g2o_outfile << "VERTEX_SE3:QUAT " << std::to_string(i) << " ";
        g2o_outfile << odometry.pose.pose.position.x << " ";
        g2o_outfile << odometry.pose.pose.position.y << " ";
        g2o_outfile << odometry.pose.pose.position.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.x << " ";
        g2o_outfile << odometry.pose.pose.orientation.y << " ";
        g2o_outfile << odometry.pose.pose.orientation.z << " ";
        g2o_outfile << odometry.pose.pose.orientation.w << std::endl;
    }
    //  LOG(INFO) << "WRITE G2O VERTICES: " << keyframePosesOdom.size();
    g2o_outfile.close();
}

void DataSaver::saveResultBag(std::vector<nav_msgs::Odometry> allOdometryVec,
                              std::vector<sensor_msgs::PointCloud2> allResVec) {
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();
    for (int i = 0; i < allOdometryVec.size(); i++) {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);
    }

    for (int i = 0; i < allResVec.size(); i++) {
        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
                         _laserCloudFullRes);
    }
    result_bag.close();
    //  LOG(INFO) << "WRITE ROSBAG: " << save_directory + "_result.bag" << ",
    //  SIZE: " << result_bag.getSize();
}

void DataSaver::saveResultBag(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<sensor_msgs::PointCloud2> allResVec,
        std::vector<geometry_msgs::TransformStamped> trans_vec) {
    rosbag::Bag result_bag;
    result_bag.open(save_directory + sequence_name + "_result.bag",
                    rosbag::bagmode::Write);

    //  LOG(INFO) << "ODOM AND PCD SIZE:" << allOdometryVec.size() << ", " <<
    //  allResVec.size();
    tf2_msgs::TFMessage tf_message;
    for (int i = 0; i < allOdometryVec.size(); i++) {
        nav_msgs::Odometry _laserOdometry = allOdometryVec.at(i);
        result_bag.write("pgo_odometry", _laserOdometry.header.stamp,
                         _laserOdometry);

        sensor_msgs::PointCloud2 _laserCloudFullRes = allResVec.at(i);
        result_bag.write("cloud_deskewed", _laserCloudFullRes.header.stamp,
                         _laserCloudFullRes);

        geometry_msgs::TransformStamped transform_stamped = trans_vec.at(i);
        tf_message.transforms.push_back(transform_stamped);
        result_bag.write("tf", transform_stamped.header.stamp, tf_message);
    }
    result_bag.close();
}

void DataSaver::saveLoopandImagePair(
        std::map<int, int> loopIndexCheckedMap,
        std::vector<std::vector<int>> all_camera_corre_match_pair) {
    std::ofstream loop_outfile;
    loop_outfile.open(save_directory + "lidar_loop.txt", std::ios::out);
    loop_outfile.precision(15);
    //  LOG(INFO) << "WRITE Lidar LOOP FILE: " << save_directory +
    //  "lidar_loop.txt";

    int j = 0;
    for (auto it = loopIndexCheckedMap.begin(); it != loopIndexCheckedMap.end();
         ++it) {
        int curr_node_idx = it->first;
        int prev_node_idx = it->second;

        //    geometry_msgs::Point p;
        //    p.x = keyframePosesUpdated[curr_node_idx].x;
        //    p.y = keyframePosesUpdated[curr_node_idx].y;
        //    p.z = keyframePosesUpdated[curr_node_idx].z;
        //
        //    p.x = keyframePosesUpdated[prev_node_idx].x;
        //    p.y = keyframePosesUpdated[prev_node_idx].y;
        //    p.z = keyframePosesUpdated[prev_node_idx].z;
        //
        //    // we can write some edges to g2o file
        //    //    g2o_out<<"EDGE_SE3:QUAT "<<curr_node_idx<<" "<<prev_node_idx<<"
        //    "
        //    //        <<p.x() <<" "<<p.y() <<" "<<p.z() <<" "
        //    //        <<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<" ";
        //
        //    if (saveLoopdata) {
        //      std::string common_name = std::to_string(curr_node_idx) + "_" +
        //      std::to_string(prev_node_idx);
        //
        //      std::string pcd_name_0 = common_name + "_0.pcd";
        //      std::string pcd_name_1 = common_name + "_1.pcd";
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_0,
        //      *keyframeLaserRawClouds[curr_node_idx]);
        //      pcl::io::savePCDFileBinary(pgScansDirectory + pcd_name_1,
        //      *keyframeLaserRawClouds[prev_node_idx]);
        //
        ////      cv::imwrite(pgImageDirectory + common_name + "_0_0.png",
        /// keyMeasures.at(curr_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_1.png",
        /// keyMeasures.at(curr_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_2.png",
        /// keyMeasures.at(curr_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_3.png",
        /// keyMeasures.at(curr_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_0_4.png",
        /// keyMeasures.at(curr_node_idx).camera4.front()); /      //
        /// cv::imshow("imgCallback", image_mat);
        ////
        ////      cv::imwrite(pgImageDirectory + common_name + "_1_0.png",
        /// keyMeasures.at(prev_node_idx).camera0.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_1.png",
        /// keyMeasures.at(prev_node_idx).camera1.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_2.png",
        /// keyMeasures.at(prev_node_idx).camera2.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_3.png",
        /// keyMeasures.at(prev_node_idx).camera3.front()); /
        /// cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        /// keyMeasures.at(prev_node_idx).camera4.front());
        //      cv::imwrite(pgImageDirectory + common_name + "_1_4.png",
        //      resultMat_vec.at(j));
        //    }
        j++;
        loop_outfile.precision(15);
        loop_outfile << std::to_string(curr_node_idx) << " "
                     << std::to_string(prev_node_idx);
        loop_outfile << std::endl;
    }
    loop_outfile.close();
    //  LOG(INFO) << "SAVE LOOP FILE: " << loopIndexCheckedMap.size();

    // save camera pairs if their correspondences are sufficient
    std::ofstream camera_pair_outfile;
    camera_pair_outfile.open(save_directory + "camera_pair_indices.txt",
                             std::ios::out);
    //  LOG(INFO) << "WRITE CAMERA PAIR FILE: " << save_directory +
    //  "camera_pair_indices.txt"; LOG(INFO) << "Matching camera size: " <<
    //  all_camera_corre_match_pair.size();
    for (const auto &camera_pair : all_camera_corre_match_pair) {
        int lidar_idx_1 = camera_pair[0];
        int lidar_idx_2 = camera_pair[1];
        int cam_idx_1 = camera_pair[2];
        int cam_idx_2 = camera_pair[3];
        int num_corr = camera_pair[4];
        camera_pair_outfile << lidar_idx_1 << " " << lidar_idx_2 << " " << cam_idx_1
                            << " " << cam_idx_2 << " " << num_corr << std::endl;
    }
    camera_pair_outfile.close();
}

void DataSaver::savePointCloudMap(
        std::vector<nav_msgs::Odometry> allOdometryVec,
        std::vector<pcl::PointCloud<PointT>::Ptr> allResVec) {
    std::cout << "odom and cloud size: " << allOdometryVec.size() << ", "
              << allResVec.size();

    int odom_size = std::min(allOdometryVec.size(), allResVec.size());

    if (allOdometryVec.size() != allResVec.size()) {
        std::cout << " point cloud size do not equal to odom size!";
        // return;
    }

    pcl::PointCloud<PointT>::Ptr laserCloudRaw(
            new pcl::PointCloud<PointT>());  // giseop
    pcl::PointCloud<PointT>::Ptr laserCloudTrans(
            new pcl::PointCloud<PointT>());  // giseop
    pcl::PointCloud<PointT>::Ptr globalmap(
            new pcl::PointCloud<PointT>());  // giseop
    for (int i = 0; i < odom_size; ++i) {
        nav_msgs::Odometry odom = allOdometryVec.at(i);
        laserCloudRaw = allResVec.at(i);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.rotate(Eigen::Quaterniond(
                odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z));
        transform.pretranslate(Eigen::Vector3d(odom.pose.pose.position.x,
                                               odom.pose.pose.position.y,
                                               odom.pose.pose.position.z));

        pcl::transformPointCloud(*laserCloudRaw, *laserCloudTrans,
                                 transform.matrix());
        *globalmap += *laserCloudTrans;
    }

    // save point cloud in lidar frame
    // if you want to save it in body frame(imu)
    // i will update it later
    if (!globalmap->empty()) {
        globalmap->width = globalmap->points.size();
        globalmap->height = 1;
        globalmap->is_dense = false;

        try {
            pcl::io::savePCDFileASCII(save_directory + "global_map_lidar.pcd",
                                      *globalmap);
            cout << "current scan saved to : " << save_directory << ", "
                 << globalmap->points.size() << endl;
        } catch (std::exception e) {
            ROS_ERROR("SAVE PCD ERROR", globalmap->points.size());
        }

        // all cloud must rotate to body axis
        if (use_imu_frame) {
            for (int j = 0; j < globalmap->points.size(); ++j) {
                PointT &pt = globalmap->points.at(j);
                Eigen::Vector3d translation(pt.x, pt.y, pt.z);
                translation = q_body_sensor * translation + t_body_sensor;

                pt.x = translation[0];
                pt.y = translation[1];
                pt.z = translation[2];
            }
            try {
                pcl::io::savePCDFileASCII(save_directory + "globalmap_imu.pcd",
                                          *globalmap);
                cout << "current scan saved to : " << save_directory << ", "
                     << globalmap->points.size() << endl;
            } catch (std::exception e) {
                ROS_ERROR("SAVE PCD ERROR", globalmap->points.size());
            }
        }
    } else
        std::cout << "EMPTY POINT CLOUD";
}

void DataSaver::savePointCloudMap(pcl::PointCloud<PointT> &cloud_ptr) {
    if (cloud_ptr.empty()) {
        std::cout << "empty global map cloud!" << std::endl;
        return;
    }
    try {
        pcl::io::savePCDFileASCII(save_directory + "globalmap_lidar_feature.pcd",
                                  cloud_ptr);
    } catch (pcl::IOException) {
        std::cout << "  save map failed!!! " << cloud_ptr.size() << std::endl;

    }

}

/*read parameter from kml_config.xml*/
int DataSaver::readParameter() {
    xmlDocPtr pDoc = xmlReadFile((config_directory + "kml_config.xml").c_str(),
                                 "UTF-8", XML_PARSE_RECOVER);
    if (NULL == pDoc) {
        std::cout << "open config.xml error\n" << std::endl;
        return 1;
    }

    xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);
    if (NULL == pRoot) {
        std::cout << "get config.xml root error\n" << std::endl;
        return 1;
    }

    xmlNodePtr pFirst = pRoot->children;

    while (NULL != pFirst) {
        xmlChar *value = NULL;
        if (!xmlStrcmp(pFirst->name, (const xmlChar *) ("style"))) {
            xmlNodePtr pStyle = pFirst->children;
            while (NULL != pStyle) {
                value = xmlNodeGetContent(pStyle);
                if (xmlStrcmp(pStyle->name, (const xmlChar *) ("text"))) {
                    configParameter.push_back((char *) value);
                }
                pStyle = pStyle->next;
            }
        } else if (!xmlStrcmp(pFirst->name, (const xmlChar *) ("Placemark"))) {
            xmlNodePtr pPlacemark = pFirst->children;
            while (NULL != pPlacemark) {
                value = xmlNodeGetContent(pPlacemark);
                if (xmlStrcmp(pPlacemark->name, (const xmlChar *) ("text"))) {
                    configParameter.push_back((char *) value);
                }
                pPlacemark = pPlacemark->next;
            }
        } else {
            value = xmlNodeGetContent(pFirst);
            if (xmlStrcmp(pFirst->name, (const xmlChar *) ("text"))) {
                configParameter.push_back((char *) value);
            }
        }
        pFirst = pFirst->next;
    }
    return 0;
}

int DataSaver::saveKMLTrajectory(const std::vector<Eigen::Vector3d> lla_vec) {
    if (1 == readParameter()) {
        return 1;
    }

    // longitude, latitude, height
    std::fstream ofile(save_directory + "optimized_gps_trajectry.kml",
                       std::fstream::out);
    ofile.precision(15);

    int index = 0;
    ofile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    ofile << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << endl;
    ofile << "<Document>" << endl;
    ofile << "<name>"
          << "GPS Trajectory"
          << "</name>" << endl;
    ofile << "<description>"
          << "GPS Trajectory"
          << "</description>" << endl;

    ofile << "<Style id=\"" << configParameter[index++] << "\">" << endl;
    ofile << "<LineStyle>" << endl;
    ofile << "<color>"
          << "7fFF00FF"
          << "</color>" << endl;
    ofile << "<width>" << configParameter[index++] << "</width>" << endl;
    ofile << "</LineStyle>" << endl;
    ofile << "<PolyStyle>" << endl;
    ofile << "<color>"
          << "7fFF00FF"
          << "</color>" << endl;
    ofile << "</PolyStyle>" << endl;
    ofile << "</Style>" << endl;
    ofile << "<Placemark>" << endl;
    ofile << "<styleUrl>" << configParameter[index++] << "</styleUrl>" << endl;
    ofile << "<LineString>" << endl;
    ofile << "<extrude>" << configParameter[index++] << "</extrude>" << endl;
    ofile << "<tessellate>" << configParameter[index++] << "</tessellate>"
          << endl;
    ofile << "<altitudeMode>" << configParameter[index++] << "</altitudeMode>"
          << endl;
    ofile << "<coordinates>" << endl;

    for (int i = 0; i < lla_vec.size(); i++) {
        ofile << lla_vec.at(i)[1] << ',' << lla_vec.at(i)[0] << ','
              << lla_vec.at(i)[2] << endl;
    }

    ofile << "</coordinates>" << endl;
    ofile << "</LineString></Placemark>" << endl;
    ofile << "</Document></kml>" << endl;
    return 0;
}