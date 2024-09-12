#ifndef INTERACTIVE_KEYFRAME_HPP
#define INTERACTIVE_KEYFRAME_HPP

#include <boost/any.hpp>
#include <loop/keyframe.h>
#include <opencv2/opencv.hpp>


namespace zjloc {

struct InteractiveKeyFrame: public KeyFrame{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<InteractiveKeyFrame>;

  InteractiveKeyFrame(const double& timestamp, const Eigen::Isometry3d& odom, const pcl::PointCloud<PointT>::Ptr& cloud, long node_id, g2o::HyperGraph* graph, cv::Mat img);
  // InteractiveKeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~InteractiveKeyFrame();

  // std::vector<int> neighbors(const Eigen::Vector3f& pt, double radius);

  // pcl::PointCloud<pcl::Normal>::Ptr normals();

public:
  Eigen::Vector3f min_pt;
  Eigen::Vector3f max_pt;

  boost::any kdtree_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  cv::Mat img_;
};

}

#endif