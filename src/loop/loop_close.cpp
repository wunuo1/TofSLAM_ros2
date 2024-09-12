
#include<loop/loop_close.h>

namespace zjloc {
    Loop::Loop(std::vector<OdometryFrame::Ptr> keyframes)
    {
        // keyframes_=keyframes;
        graph.reset(new InteractiveGraph(keyframes));
        automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(graph));
        
    }

    Loop::Loop()
    {
        graph.reset(new InteractiveGraph());
        automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(graph));
        
    }

    Loop::Loop(std::shared_ptr<InteractiveGraph>& graph): graph(graph) {
      automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(graph));
    }

    // Loop::Loop(std::shared_ptr<InteractiveGraph>& graph, std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& tmp_db): graph(graph) {
    //   automatic_loop_close_window.reset(new AutomaticLoopCloseWindow(graph, tmp_db));
    // }
    void Loop::loopinit(std::string output)
    {
      outputpath = output;
      automatic_loop_close_window->AutomaticLoopCloseWindowinit(outputpath);
    }
    void Loop::updatekeyframe(OdometryFrame::Ptr kf)
    {
      // keyframes_.push_back(kf);
      graph->keyframesodom.push_back(kf);
    }
    
    void Loop::updategraph()
    {
      graph->update_graph();
      std::cout<<"update graph"<<std::endl;
      graph->update_keyframe();
      std::cout<<"update keyframe"<<std::endl;
      // graph->update_db();
      // std::cout<<"update db"<<std::endl;
    }

    void Loop::buildgraph()
    {
         
        // graph->load_map_data("/home/mengxinrui/ROS/ctlio_ws_loop/test.g2o");
        graph->load_map_data();
        return;

    }

    // void Loop::saveg2o()
    //  {
    //       std::ofstream ofs("/home/mengxinrui/ROS/ctlio_ws_loop/test.g2o");
    //       if(!ofs) {
    //            std::cout<<"wrong!"<<std::endl;
    //          return;
    //       }
    //       for(int i = 0; i < keyframes_.size(); i++) 
    //       {
    //           std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
    //           v->setEstimate(keyframes_[i]->pose);
    //           ofs << "VERTEX_SE3:QUAT " << i << " ";
    //           v->write(ofs);
    //           ofs << std::endl;
    //       }
    //       ofs << "FIX 0" << std::endl;
    //       for(int i = 0; i < keyframes_.size() - 1; i++) {
    //           const auto& delta_pose = keyframes_[i]->pose.inverse() * keyframes_[i+1]->pose;
    //           std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
    //           e->setMeasurement(delta_pose);

    //           Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    //           inf.block<3, 3>(0, 0) *= 10.0;
    //           inf.block<3, 3>(3, 3) *= 20.0;

    //           e->setInformation(inf);
    //           ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
    //           e->write(ofs);
    //           ofs << std::endl;
    //       }

    //       ofs.close();
    //  }

    void Loop::setAutomaticLoopCloseWindow(int id)
    {
      automatic_loop_close_window->setgraph(id);
    }

    void Loop::LoopDetection()
    {
      //graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/beforeloop.pcd");
      automatic_loop_close_window->Loop_detection_thread();
      // graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/afterloop.pcd");
    }

    void Loop::LoopDetectionAll()
    {
      //graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/beforeloop.pcd");
      automatic_loop_close_window->loop_detection_all();
      // graph->save_pointcloud("/home/mengxinrui/ROS/ctlio_ws_loop/afterloop.pcd");
    }

    bool Loop::isrunningdetection()
    {
      return automatic_loop_close_window->isrunningdetection();
    }

    int Loop::getoptnum()
    {
      return automatic_loop_close_window->optnum;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr Loop::get_pointcloud()
    {
      return graph->get_pointcloud();
    }
}  //namespace zjloc