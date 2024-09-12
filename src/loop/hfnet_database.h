#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>

#include "BoostArchiver.h"



class HFNetManager
{
public: 
    HFNetManager( ) = default; 


    void saveHFNetDataBase(pcl::PointCloud<pcl::PointXYZI> & cur_cloud, cv::Mat & cur_image, 
                            Eigen::VectorXf & cur_desc, std::vector<double> & frame_pose);


public:
    std::vector<std::vector<double>> pose_database;
    std::vector<cv::Mat> image_database;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_database;
    std::vector<Eigen::VectorXf> descriptor_database;



private:
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive &ar, const unsigned int version);

}; 

