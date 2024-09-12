#include "hfnet_database.h"


void HFNetManager::saveHFNetDataBase(pcl::PointCloud<pcl::PointXYZI> & cur_cloud, cv::Mat & cur_image, 
                            Eigen::VectorXf & cur_desc, std::vector<double> & frame_pose) 
{
    cloud_database.push_back(cur_cloud);
    image_database.push_back(cur_image);
    descriptor_database.push_back(cur_desc);
    pose_database.push_back(frame_pose);
}



template<class Archive>
void HFNetManager::serialize(Archive &ar, const unsigned int version)
{
    ar & pose_database;
    ar & cloud_database;
    ar & image_database;
    ar & descriptor_database;

}
template void HFNetManager::serialize(boost::archive::binary_iarchive&, const unsigned int);
template void HFNetManager::serialize(boost::archive::binary_oarchive&, const unsigned int);

