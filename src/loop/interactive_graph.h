#ifndef HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP
#define HDL_GRAPH_SLAM_INTERACTIVE_GRAPH_HPP

#include <regex>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <Eigen/Dense>
#include <boost/format.hpp>

// #include <guik/progress_interface.hpp>

#include <loop/graph_slam.h>
#include <loop/parameter_server.hpp>
#include <loop/interactive_keyframe.h>
#include "liw/lio_utils.h"
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include "Scancontext.h"
#include "hfnet_database.h"

namespace g2o {
class VertexPlane;
class VertexSE3;
class VertexSE3Edge;
}  // namespace g2o

namespace zjloc {

class InformationMatrixCalculator;

/**
 * @brief core class to interactively manipulate a pose graph
 *
 */
class InteractiveGraph : protected GraphSLAM {
public:
  InteractiveGraph(std::vector<OdometryFrame::Ptr> keyframes);
  InteractiveGraph();
  virtual ~InteractiveGraph();

  void InteractiveGraphinit(std::string model);

  bool load_map_data(const std::string& directory);
  bool load_map_data();
  // bool merge_map_data(InteractiveGraph& graph_, const InteractiveKeyFrame::Ptr& key1, const InteractiveKeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose);

  long anchor_node_id() const;

  g2o::EdgeSE3* add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2, const Eigen::Isometry3d& relative_pose, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);

  // g2o::VertexPlane* add_plane(const Eigen::Vector4d& coeffs);
  // g2o::EdgeSE3Plane* add_edge(const KeyFrame::Ptr& v_se3, g2o::VertexPlane* v_plane, const Eigen::Vector4d& coeffs, const Eigen::MatrixXd& information, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);

  void apply_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& robust_kernel, double robust_kernel_delta);

  // void add_edge_identity(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);
  // void add_edge_parallel(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);
  // void add_edge_perpendicular(g2o::VertexPlane* v1, g2o::VertexPlane* v2, double information_scale, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);
  // bool add_edge_prior_normal(long plane_vertex_id, const Eigen::Vector3d& normal, double information_scale, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);
  // bool add_edge_prior_distance(long plane_vertex_id, double distance, double information_scale, const std::string& robust_kernel = "NONE", double robust_kernel_delta = 0.1);

  void optimize(int num_iterations = -1);
  void optimize_background(int num_iterations = -1);

  std::string graph_statistics(bool update=false);
  std::string optimization_messages() const;

  // void dump(const std::string& directory, guik::ProgressInterface& progress);
  bool save_pointcloud(const std::string& filename);
  pcl::PointCloud<pcl::PointXYZI>::Ptr get_pointcloud();

  using GraphSLAM::graph;
  using GraphSLAM::num_edges;
  using GraphSLAM::num_vertices;
  using GraphSLAM::set_solver;

private:
  // bool load_special_nodes(const std::string& directory, guik::ProgressInterface& progress);
  bool load_keyframes();
  bool load_graph();
public:
  bool update_graph();
  bool update_keyframe();
  // bool update_db();

private:
  g2o::VertexSE3* anchor_node;
  g2o::EdgeSE3* anchor_edge;
  g2o::VertexPlane* floor_node;

  long edge_id_gen;
  std::thread optimization_thread;
  std::string modelpath;

public:
  mutable std::mutex optimization_mutex;

  std::string graph_stats;
  std::stringstream optimization_stream;
  std::shared_ptr<Ort::Session> session_;


  ParameterServer params;

  // optimization statistics (just for visualization)
  int iterations;
  double chi2_before;
  double chi2_after;
  double elapsed_time_msec;

  std::unordered_map<long, InteractiveKeyFrame::Ptr> keyframes;
  std::vector<InteractiveKeyFrame::Ptr> vectorkeyframes;
  std::unique_ptr<InformationMatrixCalculator> inf_calclator;
  std::vector<OdometryFrame::Ptr> keyframesodom;
  bool isopt=false;
  long optlastid=0;

  SCManager *scManager;
  HFNetManager *hfNetManager;

  std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>> db;
  
  // std::map<InteractiveKeyFrame::Ptr, Eigen::VectorXf> db;
};

}  // namespace zjloc

#endif