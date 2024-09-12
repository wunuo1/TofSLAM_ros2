// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

namespace g2o {
class VertexSE3;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace zjloc {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const double& timestamp, const Eigen::Isometry3d& odom, const pcl::PointCloud<PointT>::ConstPtr& cloud, long node_id, g2o::HyperGraph* graph);
  // KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  // void save(const std::string& directory);
  // bool load(const std::string& directory, g2o::HyperGraph* graph);

  long id() const;
  Eigen::Isometry3d estimate() const;

public:
  double timestamp;                                // timestamp
  Eigen::Isometry3d odom;                         // odometry (estimated by scan_matching_odometry)
  // double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  // boost::optional<Eigen::Vector4d> floor_coeffs;  // detected floor's coefficients
  // boost::optional<Eigen::Vector3d> utm_coord;     // UTM coord obtained by GPS

  // boost::optional<Eigen::Vector3d> acceleration;    //
  // boost::optional<Eigen::Quaterniond> orientation;  //

  g2o::VertexSE3* node;  // node instance
  long node_id;
};

// /**
//  * @brief KeyFramesnapshot for map cloud generation
//  */
// struct KeyFrameSnapshot {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   using PointT = KeyFrame::PointT;
//   using Ptr = std::shared_ptr<KeyFrameSnapshot>;

//   KeyFrameSnapshot(const KeyFrame::Ptr& key);
//   KeyFrameSnapshot(const Eigen::Isometry3d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

//   ~KeyFrameSnapshot();

// public:
//   Eigen::Isometry3d pose;                   // pose estimated by graph optimization
//   pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
// };

}  // namespace zjloc

#endif  // KEYFRAME_HPP
