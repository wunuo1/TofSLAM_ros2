#ifndef LOOP_CLOSE_
#define LOOP_CLOSE_

#include <Eigen/Dense>
#include "liw/lio_utils.h"
// #include <g2o/types/slam3d/edge_se3.h>
// #include <g2o/types/slam3d/vertex_se3.h>
#include <loop/interactive_graph.h>
#include <loop/interactive_keyframe.h>
#include <loop/automatic_loop_close_window.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/edge_se3_plane.hpp>
#include <g2o/edge_se3_priorxy.hpp>
#include <g2o/edge_se3_priorxyz.hpp>
#include <g2o/edge_se3_priorvec.hpp>
#include <g2o/edge_se3_priorquat.hpp>
#include <g2o/edge_plane_prior.hpp>
#include <g2o/edge_plane_identity.hpp>
#include <g2o/edge_plane_parallel.hpp>
#include <g2o/robust_kernel_io.hpp>
#include <fstream>


namespace zjloc {
    class Loop{
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          using Ptr = std::shared_ptr<Loop>;

          Loop(std::vector<OdometryFrame::Ptr> keyframes);
          Loop();
          Loop(std::shared_ptr<InteractiveGraph>& graph);
          // Loop(std::shared_ptr<InteractiveGraph>& graph, std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& tmp_db);
          // void saveg2o();
          void loopinit(std::string output);
          void buildgraph();
          void setAutomaticLoopCloseWindow(int id);
          void LoopDetection();
          void LoopDetectionAll();
          void updatekeyframe(OdometryFrame::Ptr kf);
          void updategraph();
          bool isrunningdetection();
          int getoptnum();
          pcl::PointCloud<pcl::PointXYZI>::Ptr get_pointcloud();

        private:
          // std::vector<OdometryFrame::Ptr> keyframes_;
          std::unique_ptr<AutomaticLoopCloseWindow> automatic_loop_close_window;
          std::shared_ptr<InteractiveGraph> graph;
          std::string outputpath;


    };
} // namespace zjloc

#endif