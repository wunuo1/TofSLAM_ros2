#include <loop/automatic_loop_close_window.h>

#include <unordered_set>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <loop/information_matrix_calculator.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

namespace zjloc {

AutomaticLoopCloseWindow::AutomaticLoopCloseWindow(std::shared_ptr<InteractiveGraph>& graph)
    :graph(graph), show_window(false), running(false), loop_detection_source(0), fitness_score_thresh(0.03f), 
    fitness_score_max_range(2.0f), search_method(1), distance_thresh(2.0f), accum_distance_thresh(5.0f), optimize(true) {
  // Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
  // Ort::SessionOptions session_options;
  // session_options.SetIntraOpNumThreads(1);
  // // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
  // session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
  // const char* model_path = "/root/catkin_ws/src/ct-lio/model/hfnet.onnx";
  // Ort::Session session(env, model_path, session_options);
  // std::cout << "session: " << &session << std::endl;
  // std::cout << "session_0: " << session_ << std::endl;
  // session_ = &session;
  // std::cout << "session_: " << session_ << std::endl;
  // std::pair<std::string, Eigen::VectorXf> query1 = inference_img("/shared_dir/1.png", *session_);
  // std::pair<std::string, Eigen::VectorXf> query2 = inference_img("/shared_dir/1.png", *session_);
  // std::pair<std::string, Eigen::VectorXf> query3 = inference_img("/shared_dir/1.png", *session_);
}

// AutomaticLoopCloseWindow::AutomaticLoopCloseWindow(std::shared_ptr<InteractiveGraph>& graph, 
//                     std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& tmp_db)
//     :graph(graph), show_window(false), running(false), loop_detection_source(0), fitness_score_thresh(0.03f), 
//     fitness_score_max_range(2.0f), search_method(1), distance_thresh(2.0f), accum_distance_thresh(5.0f), optimize(true) {
//       db = tmp_db;
// }

AutomaticLoopCloseWindow::~AutomaticLoopCloseWindow() {
  // if (running) {
  //   running = false;
  // } 

  if (loop_detection_thread.joinable()) {
    loop_detection_thread.join();
  }
}

void AutomaticLoopCloseWindow::AutomaticLoopCloseWindowinit(std::string output)
{
  outputpath = output;
}
// void AutomaticLoopCloseWindow::draw_ui() {
//   if (!show_window) {
//     running = false;
//     return;
//   }

//   ImGui::Begin("automatic loop close", &show_window, ImGuiWindowFlags_AlwaysAutoResize);

//   registration_method.draw_ui();
//   ImGui::DragFloat("Fitness score thresh", &fitness_score_thresh, 0.01f, 0.01f, 10.0f);

//   ImGui::Text("Loop detection");
//   const char* search_methods[] = {"RANDOM", "SEQUENTIAL"};
//   ImGui::Combo("Search method", &search_method, search_methods, IM_ARRAYSIZE(search_methods));
//   ImGui::DragFloat("Distance thresh", &distance_thresh, 0.5f, 0.5f, 100.0f);
//   ImGui::DragFloat("Accum distance thresh", &accum_distance_thresh, 0.5f, 0.5f, 100.0f);

//   robust_kernel.draw_ui();

//   ImGui::Checkbox("Optimization", &optimize);

//   if (ImGui::Button("Start")) {
//     if (!running) {
//       running = true;
//       loop_detection_thread = std::thread([&]() { loop_detection(); });
//     }
//   }

//   ImGui::SameLine();
//   if (ImGui::Button("Stop")) {
//     if (running) {
//       running = false;
//       loop_detection_thread.join();
//     }
//   }

//   if (running) {
//     ImGui::SameLine();
//     ImGui::Text("%c Running", "|/-\\"[(int)(ImGui::GetTime() / 0.05f) & 3]);
//   }

//   ImGui::End();
// }
void AutomaticLoopCloseWindow::setgraph(int id_)
{
  id = id_;
}
void AutomaticLoopCloseWindow::Loop_detection_thread()
{

  if (loop_detection_thread.joinable()) {
    loop_detection_thread.join();
  }
  runningdetection = true;
  loop_detection_thread = std::thread([&]() { loop_detection(); });
}

bool AutomaticLoopCloseWindow::isrunningdetection()
{
  return runningdetection;
}

int Plane_fitting(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input, int id)
{

     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
     pcl::SACSegmentation<pcl::PointXYZI> seg;
     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>());

     seg.setOptimizeCoefficients(true);
     seg.setModelType(pcl::SACMODEL_PLANE);
     seg.setMethodType(pcl::SAC_RANSAC);
     seg.setMaxIterations(300);
     seg.setDistanceThreshold(0.03);

    int m=0;
    while (cloud_input->size() > 100)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZI>);
        seg.setInputCloud(cloud_input);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            break;
        }
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud_input);
        extract.setIndices(inliers);
        extract.filter(*cloud_plane);//输出平面

        if (cloud_plane->size()>100)
        {
            m++;
            // pcl::io::savePCDFileBinary("/home/mengxinrui/ROS/ctlio_ws_loop/cloud/"+std::to_string(id)+"-"+std::to_string(m)+"-0.pcd", *cloud_plane);
        }
        // 移除plane
        extract.setNegative(true);
        extract.filter(*cloud_p);
        *cloud_input = *cloud_p;
    }
    return m;

}

void AutomaticLoopCloseWindow::loop_detection() {
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration = registration_method.method();

  // LOG(INFO) << "------------------load loop_detection db----------------------";
  // LOG(INFO) << "db size: " + std::to_string(graph->db.size());
  // // std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>> db;
  // for (int i = 0; i < graph->vectorkeyframes.size(); i++) {
  //   LOG(INFO) << "prepare to find kf key, id: " + std::to_string(graph->vectorkeyframes[i]->id());
  //   bool found_flag = true;
  //   for (int j = 0; j < graph->db.size(); j++) {
  //     if(graph->db[j].first == graph->vectorkeyframes[i]) {
  //       found_flag = false;
  //     }
  //   }
  //   if(found_flag) {
  //     LOG(INFO) << "not find kf key, id: " + std::to_string(graph->vectorkeyframes[i]->id());
  //     Eigen::VectorXf query = inference_img(graph->vectorkeyframes[i]->img_);
  //     cv::imwrite("/shared_dir/kf_" + std::to_string(graph->vectorkeyframes[i]->id()) + ".png", graph->vectorkeyframes[i]->img_);
  //     std::pair p(graph->vectorkeyframes[i], query);
  //     graph->db.push_back(p);
  //   } else {
  //     LOG(INFO) << "found kf key in db, id: " + std::to_string(graph->vectorkeyframes[i]->id());
  //   }
  //   found_flag = true;
  // }
  // LOG(INFO) << "--------------load loop_detection db finished-----------------";
  
  // while (loop_detection_source<graph->vectorkeyframes.size()) 
  {
    loop_detection_source = graph->vectorkeyframes.size()-1;
    InteractiveKeyFrame::Ptr source = graph->vectorkeyframes[loop_detection_source];
    std::cout<<"source:"<<source->id()<<std::endl;
    // pcl::PointCloud<pcl::PointXYZI> sourcecloudin;
    // pcl::copyPointCloud(*source->cloud, sourcecloudin);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr sourcecloudin_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // sourcecloudin_ptr=sourcecloudin.makeShared();
    // int sourceplanenum = Plane_fitting(sourcecloudin_ptr, source->id());
    // if(sourceplanenum<3)
    // {
    //   runningdetection = false;
    //   return;
    // }
    std::cout << "sourcesize:" << source->cloud->points.size() << std::endl;
      if(source->cloud->points.size()<cloudthresh)
      {
         runningdetection = false;
         return;
      }
    // pcl::io::savePCDFileBinary("/home/mengxinrui/ROS/ctlio_ws_loop/source.pcd", *source->cloud);
    Eigen::Isometry3d source_pose = source->node->estimate();
    // auto candidates = find_loop_candidates(source);
    auto candidates = find_loop_candidates_img(source, graph->db);
    // auto candidates = find_loop_candidates_scancontext(source);

    // std::cout << ANSI_COLOR_BLUE << "candidate size:" << candidates.size() << ANSI_COLOR_RESET << std::endl;
      // {
      //   std::lock_guard<std::mutex> lock(loop_detection_mutex);
      //   loop_source = source;
      //   loop_candidates = candidates;
      // }
      // t1 = std::chrono::steady_clock::now();
      bool edge_inserted = false;
      registration->setInputTarget(source->cloud);
      for (int i = 0; i < candidates.size(); i++)
      {
        if(candidates[i]->cloud->points.size()<cloudthresh) continue;
        //  auto t13 = std::chrono::steady_clock::now();
        // auto t11 = std::chrono::steady_clock::now();
        registration->setInputSource(candidates[i]->cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Isometry3d relative = source_pose.inverse() * candidates[i]->node->estimate();
        
        registration->align(*aligned, relative.matrix().cast<float>());

        relative.matrix() = registration->getFinalTransformation().cast<double>();
        // auto t22 = std::chrono::steady_clock::now();
        // auto time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(t22 - t11).count() * 1000;
        // std::cout << ANSI_COLOR_YELLOW << "registration time:" << time_used2 << ANSI_COLOR_RESET << std::endl;
        int nr = 0;
        
        // t11 = std::chrono::steady_clock::now();
        double fitness_score = InformationMatrixCalculator::calc_fitness_score(source->cloud, candidates[i]->cloud, relative, nr, fitness_score_max_range);
        
        // t22 = std::chrono::steady_clock::now();
        // time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(t22 - t11).count() * 1000;
        // std::cout << ANSI_COLOR_YELLOW << "fitness time:" << time_used2 << ANSI_COLOR_RESET << std::endl;
        // float ratio1 = float(nr) / float(source->cloud->points.size());
        // float ratio2 = float(nr) / float(candidates[i]->cloud->points.size());
        // float ratio = std::min(ratio1, ratio2);
        std::cout << "fitness_score" << fitness_score << std::endl;
        // std::cout << "fitness_score2" << fitness_score2 << std::endl;
        // std::cout<<std::setprecision(14)<<candidates[i]->timestamp<<std::endl;
        std::cout << candidates[i]->id() << std::endl;
        
        // pcl::io::savePCDFileBinary("/home/mengxinrui/ROS/ctlio_ws_loop/candidate.pcd", *candidates[i]->cloud);
        if (fitness_score < fitness_score_thresh /*&& fitness_score2 < fitness_score_thresh*//*&& fitness_score>0.025*/ /*&& planenum>5*/)
        {
          // int nr2 = 0;
          // double fitness_score2 = InformationMatrixCalculator::calc_fitness_score(candidates[i]->cloud,source->cloud,  relative.inverse(), nr2, fitness_score_max_range);
          // pcl::PointCloud<pcl::PointXYZI> cloudin;
          // pcl::copyPointCloud(*candidates[i]->cloud, cloudin);
          // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
          // cloud_ptr=cloudin.makeShared();
          // int planenum = Plane_fitting(cloud_ptr, candidates[i]->id());

          // Eigen::Vector4f mean1, mean2;
          // pcl::compute3DCentroid(*source->cloud, mean1);
          // pcl::compute3DCentroid(*candidates[i]->cloud, mean2);
          // if(mean1.z()<2 || mean2.z()<2)
          // {
          //   continue;
          // }

          // if(planenum>5)
          // if(mean1.z()>=2 && mean2.z()>=2)
          // if(fitness_score2 < fitness_score_thresh)
          {
            cv::imwrite(outputpath+"/"+std::to_string(source->id())+"-source.jpg", source->img_);
            cv::imwrite(outputpath+"/"+std::to_string(source->id())+"-"+std::to_string(candidates[i]->id())+"-candidate.jpg", candidates[i]->img_);
            pcl::PointCloud<pcl::PointXYZI> input_transformed;
            pcl::transformPointCloud(*candidates[i]->cloud, input_transformed, relative.cast<float>());
            pcl::io::savePCDFileBinary(outputpath+"/"+std::to_string(source->id())+"-"+/*std::to_string(mean1.z())+*/"-source.pcd", *source->cloud);
            pcl::io::savePCDFileBinary(outputpath+"/"+std::to_string(source->id())+"-"+std::to_string(candidates[i]->id())+/*std::to_string(mean2.z())+*/"-candidate.pcd", input_transformed);
            // t11 = std::chrono::steady_clock::now();
            edge_inserted = true;
            auto edge = graph->add_edge(source, candidates[i], relative, robust_kernel.type(), robust_kernel.delta());
        //     t22 = std::chrono::steady_clock::now();
        // time_used2 = std::chrono::duration_cast<std::chrono::duration<double>>(t22 - t11).count() * 1000;
        // std::cout << ANSI_COLOR_YELLOW << "edge time:" << time_used2 << ANSI_COLOR_RESET << std::endl;
            break;
          }
        }
      // auto t23 = std::chrono::steady_clock::now();
      //   auto time_used3 = std::chrono::duration_cast<std::chrono::duration<double>>(t23 - t13).count() * 1000;
      //   std::cout << ANSI_COLOR_YELLOW << "oneiter time:" << time_used3 << ANSI_COLOR_RESET << std::endl;

      }
      // t2 = std::chrono::steady_clock::now();
      // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
      // std::cout << ANSI_COLOR_BLUE << "addedge time:" << time_used << ANSI_COLOR_RESET << std::endl;
      // int lastid = graph->graph->vertices().size()-1;
      // t1 = std::chrono::steady_clock::now();
      if (edge_inserted && optimize)
      {
        std::lock_guard<std::mutex> lock(graph->optimization_mutex);
        // g2o::VertexSE3 v1 = graph->graph->vertices()[lastid];
        graph->optimize();
        optnum++;
        // g2o::VertexSE3 v2 = graph->graph->vertices()[lastid];
      }
      // t2 = std::chrono::steady_clock::now();
      // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
      // std::cout << ANSI_COLOR_BLUE << "opt time:" << time_used << ANSI_COLOR_RESET << std::endl;
      // loop_detection_source++;
      // if (search_method == 0) {
      //   loop_detection_source = rand() % graph->vectorkeyframes.size();
      // }

      // loop_detection_source = loop_detection_source % graph->keyframes.size();
    }
  //graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/"+std::to_string(id)+".pcd");
  runningdetection = false;
}

void AutomaticLoopCloseWindow::loop_detection_all() {
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration = registration_method.method();
  // std::cout << "[kf size]: " << graph->vectorkeyframes.size() << std::endl;
  // LOG(INFO) << "------------------load loop_detection_all db----------------------";
  // LOG(INFO) << "db size: " + std::to_string(graph->db.size());
  // // std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>> db;
  // for (int i = 0; i < graph->vectorkeyframes.size(); i++) {
  //   LOG(INFO) << "prepare to find kf key, id: " + std::to_string(graph->vectorkeyframes[i]->id());
  //   bool found_flag = true;
  //   for (int j = 0; j < graph->db.size(); j++) {
  //     if(graph->db[j].first == graph->vectorkeyframes[i]) {
  //       found_flag = false;
  //     }
  //   }
  //   if(found_flag) {
  //     LOG(INFO) << "not fond kf key, id: " + std::to_string(graph->vectorkeyframes[i]->id());
  //     Eigen::VectorXf query = inference_img(graph->vectorkeyframes[i]->img_);
  //     cv::imwrite("/shared_dir/kf_" + std::to_string(graph->vectorkeyframes[i]->id()) + ".png", graph->vectorkeyframes[i]->img_);
  //     std::pair p(graph->vectorkeyframes[i], query);
  //     graph->db.push_back(p); 
  //   } else {
  //     LOG(INFO) << "fond kf key in db, id:  " + std::to_string(graph->vectorkeyframes[i]->id());
  //   }
  //   found_flag = true;
  // }
  // LOG(INFO) << "--------------load loop_detection_all db finished-----------------";
  // std::cout<<"aaa"<<graph->vectorkeyframes.size()<<std::endl;
  // std::cout<<"bbb"<<graph->db.size()<<std::endl;
  while (loop_detection_source < graph->vectorkeyframes.size()) 
  {
    // loop_detection_source = graph->vectorkeyframes.size()-1;
    InteractiveKeyFrame::Ptr source = graph->vectorkeyframes[loop_detection_source];
    std::cout<<"source:"<<source->id()<<std::endl;
    std::cout << "sourcesize:" << source->cloud->points.size() << std::endl;
    // pcl::PointCloud<pcl::PointXYZI> sourcecloudin;
    // pcl::copyPointCloud(*source->cloud, sourcecloudin);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr sourcecloudin_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    // sourcecloudin_ptr=sourcecloudin.makeShared();
    // int sourceplanenum = Plane_fitting(sourcecloudin_ptr, source->id());
    if(source->cloud->points.size()<cloudthresh)
      {
         loop_detection_source++;
        //  runningdetection = false;
         continue;
      }
    // pcl::io::savePCDFileBinary("/home/mengxinrui/ROS/ctlio_ws_loop/source.pcd", *source->cloud);
    Eigen::Isometry3d source_pose = source->node->estimate();
    // auto candidates = find_loop_candidates(source);
     auto candidates = find_loop_candidates_img(source, graph->db);
   // auto candidates = find_loop_candidates_scancontext(source);
    std::cout << "[candidates size]: " << candidates.size() << std::endl;

    // {
    //   std::lock_guard<std::mutex> lock(loop_detection_mutex);
    //   loop_source = source;
    //   loop_candidates = candidates;
    // }

    bool edge_inserted = false;
      registration->setInputTarget(source->cloud);
      for (int i = 0; i < candidates.size(); i++)
      {
        if(candidates[i]->cloud->points.size()<cloudthresh) continue;
        registration->setInputSource(candidates[i]->cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());
        Eigen::Isometry3d relative = source_pose.inverse() * candidates[i]->node->estimate();
        registration->align(*aligned, relative.matrix().cast<float>());

        relative.matrix() = registration->getFinalTransformation().cast<double>();
        int nr = 0;
        double fitness_score = InformationMatrixCalculator::calc_fitness_score(source->cloud, candidates[i]->cloud, relative, nr, fitness_score_max_range);
        std::cout << "fitness_score" << fitness_score << std::endl;
        std::cout << candidates[i]->id() << std::endl;
        // pcl::PointCloud<pcl::PointXYZI> cloudin;
        // pcl::copyPointCloud(*candidates[i]->cloud, cloudin);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // cloud_ptr=cloudin.makeShared();
        // int planenum = Plane_fitting(cloud_ptr, candidates[i]->id());
        // pcl::io::savePCDFileBinary("/home/mengxinrui/ROS/ctlio_ws_loop/candidate.pcd", *candidates[i]->cloud);
        if (fitness_score < fitness_score_thresh /* && planenum>2*/)
        {
          // int nr2 = 0;
          // double fitness_score2 = InformationMatrixCalculator::calc_fitness_score(candidates[i]->cloud,source->cloud,  relative.inverse(), nr2, fitness_score_max_range);
          // std::cout << "fitness_score2" << fitness_score2 << std::endl;
          // if(fitness_score2 < fitness_score_thresh)
          {
            cv::imwrite(outputpath+"/"+std::to_string(source->id())+"-source.jpg", source->img_);
            cv::imwrite(outputpath+"/"+std::to_string(source->id())+"-"+std::to_string(candidates[i]->id())+"-candidate.jpg", candidates[i]->img_);
            pcl::PointCloud<pcl::PointXYZI> input_transformed;
            pcl::transformPointCloud(*candidates[i]->cloud, input_transformed, relative.cast<float>());
            pcl::io::savePCDFileBinary(outputpath+"/"+std::to_string(source->id())+"-source.pcd", *source->cloud);
            pcl::io::savePCDFileBinary(outputpath+"/"+std::to_string(source->id())+"-"+std::to_string(candidates[i]->id())+"-candidate.pcd", input_transformed);
            edge_inserted = true;
            auto edge = graph->add_edge(source, candidates[i], relative, robust_kernel.type(), robust_kernel.delta());
          }
        }
      }
      // int lastid = graph->graph->vertices().size()-1;
      if (edge_inserted && optimize)
      {
        std::lock_guard<std::mutex> lock(graph->optimization_mutex);
        // g2o::VertexSE3 v1 = graph->graph->vertices()[lastid];
        graph->optimize();
        optnum++;
        // g2o::VertexSE3 v2 = graph->graph->vertices()[lastid];
      }

      loop_detection_source++;
      // if (search_method == 0) {
      //   loop_detection_source = rand() % graph->vectorkeyframes.size();
      // }

      // loop_detection_source = loop_detection_source % graph->keyframes.size();
    }


  //graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/"+std::to_string(id)+".pcd");
  runningdetection = false;
}

// std::vector<InteractiveKeyFrame::Ptr> AutomaticLoopCloseWindow::find_loop_candidates_img(const InteractiveKeyFrame::Ptr& keyframe) {
//   std::vector<InteractiveKeyFrame::Ptr> loop_candidates;
//   Eigen::VectorXf query = inference_img(keyframe->img_);
//   // std::pair<std::string, Eigen::VectorXf> query = inference_img("/shared_dir/1.png");
//   for (const auto& candidate: graph->vectorkeyframes) {
//     Eigen::VectorXf tmp_db = inference_img(candidate->img_);
//     // std::pair<std::string, Eigen::VectorXf> tmp_db = inference_img("/shared_dir/1.png");
//     double result = global_match(query, tmp_db);
//     std::cout << "[score]: img global score: " << result << std::endl;
//     // loop_candidates.push_back(candidate);
//     if (result < 5e-6) {
//         loop_candidates.push_back(candidate);
//     }
//   }
//   return loop_candidates;
// }

std::vector<InteractiveKeyFrame::Ptr> AutomaticLoopCloseWindow::find_loop_candidates_img(const InteractiveKeyFrame::Ptr& keyframe,
                                                                                        std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& db) {
  std::unordered_map<long, float> accum_distances;
  accum_distances[keyframe->id()] = 0.0f;

  std::deque<InteractiveKeyFrame::Ptr> search_queue = {keyframe};

  while (!search_queue.empty()) {
    auto target = search_queue.front();
    float target_accum_distance = accum_distances[target->id()];
    Eigen::Vector3d target_pos = target->estimate().translation();
    search_queue.pop_front();

    for (auto& edge_ : target->node->edges()) {
      g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
      if (edge == nullptr) {
        continue;
      }

      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

      g2o::VertexSE3* next = (v1->id() == target->node->id()) ? v2 : v1;
      if (graph->keyframes.find(next->id()) == graph->keyframes.end()) {
        continue;
      }

      float delta = (next->estimate().translation() - target_pos).norm();
      float accum_distance = target_accum_distance + delta;

      auto found = accum_distances.find(next->id());
      if (found == accum_distances.end() || found->second > accum_distance) {
        accum_distances[next->id()] = accum_distance;
        search_queue.push_back(graph->keyframes[next->id()]);
      }
    }
  }

  std::unordered_set<long> excluded_edges;
  for (auto& edge_ : keyframe->node->edges()) {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
    if (edge == nullptr) {
      continue;
    }

    excluded_edges.insert(edge->vertices()[0]->id());
    excluded_edges.insert(edge->vertices()[1]->id());
  }

  Eigen::Vector3d keyframe_pos = keyframe->node->estimate().translation();
  
  std::vector<InteractiveKeyFrame::Ptr> loop_candidates;
  Eigen::VectorXf query; // = inference_img(keyframe->img_);
  for (int i = 0; i < db.size(); i++) {
    if(keyframe->id() == db[i].first->id())
    {
      query = db[i].second;
      break;
    }
  }
  // std::pair<std::string, Eigen::VectorXf> query = inference_img("/shared_dir/1.png");
  // for (const auto& candidate: graph->vectorkeyframes) {
  //   Eigen::VectorXf tmp_db = inference_img(candidate->img_);
  //   // std::pair<std::string, Eigen::VectorXf> tmp_db = inference_img("/shared_dir/1.png");
  //   double result = global_match(query, tmp_db);
  //   std::cout << "[score]: img global score: " << result << std::endl;
  //   // loop_candidates.push_back(candidate);
  //   if (result < 5e-6) {
  //       loop_candidates.push_back(candidate);
  //   }
  // }
  for (int i = 0; i < db.size(); i++) {
    if (excluded_edges.find(db[i].first->id()) != excluded_edges.end()) {
      continue;
    }
    double dist = (db[i].first->node->estimate().translation() - keyframe_pos).norm();
    Eigen::Matrix3d deltarot = db[i].first->node->estimate().rotation()*keyframe->node->estimate().rotation().inverse();
    Eigen::AngleAxisd rotation_vector(deltarot);

    auto found = accum_distances.find(db[i].first->id());
    double accum_dist = found->second;
    // clock_t startTime = clock();
    double result = global_match(query, db[i].second);
    // clock_t endTime = clock();
    // std::cout << "The global match run time is: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    if (found == accum_distances.end()) {
      if(result < 5e-6 && abs(rotation_vector.angle())<1 && dist < distance_thresh) {
        loop_candidates.push_back(db[i].first);
      }
      continue;
    }
    if((accum_dist > 5.0f) && (result < 5e-6) && abs(rotation_vector.angle())<1 && dist < distance_thresh) {
      loop_candidates.push_back(db[i].first);
    }
  }
  return loop_candidates;
}

std::vector<InteractiveKeyFrame::Ptr> AutomaticLoopCloseWindow::find_loop_candidates(const InteractiveKeyFrame::Ptr& keyframe) {
  std::unordered_map<long, float> accum_distances;
  accum_distances[keyframe->id()] = 0.0f;

  std::deque<InteractiveKeyFrame::Ptr> search_queue = {keyframe};

  while (!search_queue.empty()) {
    auto target = search_queue.front();
    float target_accum_distance = accum_distances[target->id()];
    Eigen::Vector3d target_pos = target->estimate().translation();
    search_queue.pop_front();

    for (auto& edge_ : target->node->edges()) {
      g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
      if (edge == nullptr) {
        continue;
      }

      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

      g2o::VertexSE3* next = (v1->id() == target->node->id()) ? v2 : v1;
      if (graph->keyframes.find(next->id()) == graph->keyframes.end()) {
        continue;
      }

      float delta = (next->estimate().translation() - target_pos).norm();
      float accum_distance = target_accum_distance + delta;

      auto found = accum_distances.find(next->id());
      if (found == accum_distances.end() || found->second > accum_distance) {
        accum_distances[next->id()] = accum_distance;
        search_queue.push_back(graph->keyframes[next->id()]);
      }
    }
  }

  std::unordered_set<long> excluded_edges;
  for (auto& edge_ : keyframe->node->edges()) {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
    if (edge == nullptr) {
      continue;
    }

    excluded_edges.insert(edge->vertices()[0]->id());
    excluded_edges.insert(edge->vertices()[1]->id());
  }

  Eigen::Vector3d keyframe_pos = keyframe->node->estimate().translation();
  std::vector<InteractiveKeyFrame::Ptr> loop_candidates;
  for (const auto& candidate : graph->vectorkeyframes) {
    // auto candidate = mapit.second;
    if (excluded_edges.find(candidate->id()) != excluded_edges.end()) {
      continue;
    }

    double dist = (candidate->node->estimate().translation() - keyframe_pos).norm();
    Eigen::Matrix3d deltarot = candidate->node->estimate().rotation()*keyframe->node->estimate().rotation().inverse();
    Eigen::AngleAxisd rotation_vector(deltarot);
    auto found = accum_distances.find(candidate->id());
    if (found == accum_distances.end()) {
      if(dist < distance_thresh && abs(rotation_vector.angle())<1) {
        loop_candidates.push_back(candidate);
        //std::cout<<rotation_vector.angle()<<std::endl;
      }
      continue;
    }

    double accum_dist = found->second;
    if (accum_dist > accum_distance_thresh && dist < distance_thresh && abs(rotation_vector.angle())<1) {
      loop_candidates.push_back(candidate);
      //std::cout<<rotation_vector.angle()<<std::endl;
    }
  }

  return loop_candidates;
}

std::vector<InteractiveKeyFrame::Ptr> AutomaticLoopCloseWindow::find_loop_candidates_scancontext(const InteractiveKeyFrame::Ptr& keyframe) {
  std::unordered_map<long, float> accum_distances;
  accum_distances[keyframe->id()] = 0.0f;

  std::deque<InteractiveKeyFrame::Ptr> search_queue = {keyframe};

  while (!search_queue.empty()) {
    auto target = search_queue.front();
    float target_accum_distance = accum_distances[target->id()];
    Eigen::Vector3d target_pos = target->estimate().translation();
    search_queue.pop_front();

    for (auto& edge_ : target->node->edges()) {
      g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
      if (edge == nullptr) {
        continue;
      }

      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
      g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

      g2o::VertexSE3* next = (v1->id() == target->node->id()) ? v2 : v1;
      if (graph->keyframes.find(next->id()) == graph->keyframes.end()) {
        continue;
      }

      float delta = (next->estimate().translation() - target_pos).norm();
      float accum_distance = target_accum_distance + delta;

      auto found = accum_distances.find(next->id());
      if (found == accum_distances.end() || found->second > accum_distance) {
        accum_distances[next->id()] = accum_distance;
        search_queue.push_back(graph->keyframes[next->id()]);
      }
    }
  }

  std::unordered_set<long> excluded_edges;
  for (auto& edge_ : keyframe->node->edges()) {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_);
    if (edge == nullptr) {
      continue;
    }

    excluded_edges.insert(edge->vertices()[0]->id());
    excluded_edges.insert(edge->vertices()[1]->id());
  }

  Eigen::Vector3d keyframe_pos = keyframe->node->estimate().translation();
  
  std::vector<InteractiveKeyFrame::Ptr> loop_candidates;

  std::map<int, float> candidate_map = graph->scManager->detectDistanceLoopClosureID(keyframe->id());

  for (auto & m: candidate_map) {
    InteractiveKeyFrame::Ptr frame_candidate;
    for (int i = 0; i < graph->keyframes.size(); i++) {
      if (m.first == graph->keyframes[i]->id()) {
        frame_candidate = graph->keyframes[i];
      }
    }

    if (excluded_edges.find(frame_candidate->id()) != excluded_edges.end()) {
      continue;
    }

    double dist = (frame_candidate->node->estimate().translation() - keyframe_pos).norm();
    Eigen::Matrix3d deltarot = frame_candidate->node->estimate().rotation()*keyframe->node->estimate().rotation().inverse();
    Eigen::AngleAxisd rotation_vector(deltarot);

    auto found = accum_distances.find(m.first);
    double accum_dist = found->second;

    if (found == accum_distances.end()) {
      if(abs(rotation_vector.angle())<1 && dist < distance_thresh) {
        loop_candidates.push_back(frame_candidate);
      }
      continue;
    }
    if(accum_dist > 5.0f && abs(rotation_vector.angle())<1 && dist < distance_thresh) {
      loop_candidates.push_back(frame_candidate);
    }
  }
  return loop_candidates;
}

// void AutomaticLoopCloseWindow::draw_gl(glk::GLSLShader& shader) {
//   if (!running) {
//     return;
//   }

//   std::lock_guard<std::mutex> lock(loop_detection_mutex);
//   if (loop_source == nullptr) {
//     return;
//   }

//   shader.set_uniform("color_mode", 1);
//   shader.set_uniform("point_scale", 2.0f);

//   DrawFlags draw_flags;
//   loop_source->draw(draw_flags, shader, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f), loop_source->lock()->node->estimate().matrix().cast<float>());

//   for (const auto& candidate : loop_candidates) {
//     candidate->draw(draw_flags, shader, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f), candidate->lock()->node->estimate().matrix().cast<float>());
//   }
// }

// void AutomaticLoopCloseWindow::show() { show_window = true; }

// void AutomaticLoopCloseWindow::close() {
//   loop_source = nullptr;
//   loop_candidates.clear();

//   if(running) {
//     running = false;
//     loop_detection_thread.join();
//   }
//   show_window = false;
// }

}  // namespace zjloc
