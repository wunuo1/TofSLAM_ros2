#ifndef AUTOMATIC_LOOP_CLOSE_WINDOW_H_
#define AUTOMATIC_LOOP_CLOSE_WINDOW_H_
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <boost/optional.hpp>

// #include <imgui.h>
#include "common/read_onnx.h"
#include <loop/robust_kernels.hpp>
#include <loop/registration_methods.h>
#include <loop/interactive_graph.h>
// #include <hdl_graph_slam/view/keyframe_view.hpp>
// #include <hdl_graph_slam/view/interactive_graph_view.hpp>

namespace zjloc {

class AutomaticLoopCloseWindow {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AutomaticLoopCloseWindow(std::shared_ptr<InteractiveGraph>& graph);
  // AutomaticLoopCloseWindow(std::shared_ptr<InteractiveGraph>& graph, std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& tmp_db);
  ~AutomaticLoopCloseWindow();
  void Loop_detection_thread();
  void setgraph(int id_);
  bool isrunningdetection();
  void loop_detection_all();
  void AutomaticLoopCloseWindowinit(std::string output);
  // void draw_ui();

  // void draw_gl(glk::GLSLShader& shader);

  // void show();

  // void start();

  // void close();

private:
  void loop_detection();
  std::vector<InteractiveKeyFrame::Ptr> find_loop_candidates_img(const InteractiveKeyFrame::Ptr& keyframe, 
                                                                std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& db);
  // std::vector<InteractiveKeyFrame::Ptr> find_loop_candidates_img(const InteractiveKeyFrame::Ptr& keyframe);
  std::vector<InteractiveKeyFrame::Ptr> find_loop_candidates(const InteractiveKeyFrame::Ptr& keyframe);
  std::vector<InteractiveKeyFrame::Ptr> find_loop_candidates_scancontext(const InteractiveKeyFrame::Ptr& keyframe);

private:
  // Ort::Session* session_;
  bool show_window;
  std::shared_ptr<InteractiveGraph>& graph;

  std::mutex loop_detection_mutex;
  std::thread loop_detection_thread;

  std::atomic_bool running;
  int loop_detection_source;

  InteractiveKeyFrame::Ptr loop_source;
  std::vector<InteractiveKeyFrame::Ptr> loop_candidates;

  float fitness_score_thresh; //两团点云沿着对准方法计算出的pose变换后，逐点在另一团点云上找最近邻，计算的均值距离
  float fitness_score_max_range;

  int search_method;
  float distance_thresh; //候选点条件1：距离当前点欧式距离小于distance_thresh
  float accum_distance_thresh; //候选点条件2：距离当前点沿着边的链式距离大于accum_distance_thresh

  RegistrationMethods registration_method;
  RobustKernels robust_kernel;

  bool optimize;
  int id;
  bool runningdetection=false;
  std::string outputpath;
  
public:
  int optnum=0;
  int cloudthresh = 10;
};
}

#endif
