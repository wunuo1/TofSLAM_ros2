#include <yaml-cpp/yaml.h>
#include "lidarodom.h"
#include <thread>

namespace zjloc
{
#define USE_ANALYTICAL_DERIVATE 1 //    是否使用解析求导
     std::mutex mtx;
     lidarodom::lidarodom(/* args */)
     {
          laser_point_cov = 0.001;
          current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

          CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

          index_frame = 1;
          points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
          points_distort.reset(new pcl::PointCloud<pcl::PointXYZI>());
          map_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
          

          // std::cout << "=============================test===============================" << std::endl;
          // std::pair<std::string, Eigen::VectorXf> p = inference_img("/shared_dir/1.png", session);

          // Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
          // Ort::SessionOptions session_options;
          // session_options.SetIntraOpNumThreads(1);
          // // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
          // session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
          // const char* model_path = "/root/catkin_ws/src/ct-lio/model/hfnet.onnx";
          // Ort::Session session(env, model_path, session_options);
          // session_ = &session;
          // std::pair<std::string, Eigen::VectorXf> p = inference_img("/shared_dir/1.png", *session_);
     }

     // lidarodom::lidarodom(Loop *loop_tmp)
     // {
     //      laser_point_cov = 0.001;
     //      current_state = new state(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

     //      CT_ICP::LidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

     //      CT_ICP::CTLidarPlaneNormFactor::sqrt_info = sqrt(1 / laser_point_cov);

     //      index_frame = 1;
     //      points_world.reset(new pcl::PointCloud<pcl::PointXYZI>());
     //      points_distort.reset(new pcl::PointCloud<pcl::PointXYZI>());
     //      loop = loop_tmp;
     //      f.open("/shared_dir/pose_output.txt");
     //      f << std::fixed;

     //      // std::cout << "=============================test===============================" << std::endl;
     //      // std::pair<std::string, Eigen::VectorXf> p = inference_img("/shared_dir/1.png", session);

     //      // Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "HFNET");
     //      // Ort::SessionOptions session_options;
     //      // session_options.SetIntraOpNumThreads(1);
     //      // // OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0);
     //      // session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
     //      // const char* model_path = "/root/catkin_ws/src/ct-lio/model/hfnet.onnx";
     //      // Ort::Session session(env, model_path, session_options);
     //      // session_ = &session;
     //      // std::pair<std::string, Eigen::VectorXf> p = inference_img("/shared_dir/1.png", *session_);
     // }

     lidarodom::~lidarodom()
     {
     }

     void lidarodom::loadOptions()
     {
          auto yaml = YAML::LoadFile(config_yaml_);
          options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
          options_.log_print = yaml["odometry"]["log_print"].as<bool>();
          options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

          options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
          options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
          options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
          options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
          options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
          options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
          options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
          options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
          options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
          options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
          options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
          options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
          options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
          options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
          options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

          options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
          options_.ratio_of_nonground = yaml["odometry"]["ratio_of_nonground"].as<double>();
          options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
          options_.min_num_residuals = yaml["odometry"]["min_num_residuals"].as<int>();
          options_.wheelmode = yaml["odometry"]["wheelmode"].as<int>();
          options_.use_ground_constraint = yaml["odometry"]["use_ground_constraint"].as<int>();
          std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
          if (str_motion_compensation == "NONE")
               options_.motion_compensation = MotionCompensation::NONE1;
          else if (str_motion_compensation == "CONSTANT_VELOCITY")
               options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
          else if (str_motion_compensation == "ITERATIVE")
               options_.motion_compensation = MotionCompensation::ITERATIVE;
          else if (str_motion_compensation == "CONTINUOUS")
               options_.motion_compensation = MotionCompensation::CONTINUOUS;
          else
               std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

          std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
          if (str_icpmodel == "POINT_TO_PLANE")
               options_.icpmodel = POINT_TO_PLANE;
          else if (str_icpmodel == "CT_POINT_TO_PLANE")
               options_.icpmodel = CT_POINT_TO_PLANE;
          else
               std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

          options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
          options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
          options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
          options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
          options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
          options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();

          options_.mapfile = yaml["reloc"]["mapfile"].as<std::string>();
          options_.scancontext_dbfile = yaml["reloc"]["scancontext_dbfile"].as<std::string>();
          options_.hfnet_dbfile = yaml["reloc"]["hfnet_dbfile"].as<std::string>();
          options_.reloc_mode = yaml["reloc"]["reloc_mode"].as<int>();
          options_.icp_threshold = yaml["reloc"]["icp_threshold"].as<double>();
          options_.localization_mode = yaml["common"]["Localization_mode"].as<bool>();
     }

     bool lidarodom::init(const std::string &config_yaml){

          config_yaml_ = config_yaml;
          StaticIMUInit::Options imu_init_options;
          imu_init_options.use_speed_for_static_checking_ = false; // 本节数据不需要轮速计
          imu_init_ = StaticIMUInit(imu_init_options);

          auto yaml = YAML::LoadFile(config_yaml_);
          delay_time_ = yaml["delay_time"].as<double>();
          // lidar和IMU外参
          std::vector<double> ext_t = yaml["mapping"]["tof2imu_extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_r = yaml["mapping"]["tof2imu_extrinsic_R"].as<std::vector<double>>();
          Vec3d lidar_T_wrt_IMU = math::VecFromArray(ext_t);
          Mat3d lidar_R_wrt_IMU = math::MatFromArray(ext_r);
          std::cout << yaml["mapping"]["tof2imu_extrinsic_R"] << std::endl;
          Eigen::Quaterniond q_IL(lidar_R_wrt_IMU);
          q_IL.normalized();
          lidar_R_wrt_IMU = q_IL;
          // init TIL
          TIL_ = SE3(q_IL, lidar_T_wrt_IMU);
          R_imu_lidar = lidar_R_wrt_IMU;
          t_imu_lidar = lidar_T_wrt_IMU;
          std::cout << "RIL:\n"
                    << R_imu_lidar << std::endl;
          std::cout << "tIL:" << t_imu_lidar.transpose() << std::endl;
          // Robot2imu
          std::vector<double> ext_robot_t = yaml["mapping"]["robot2imu_extrinsic_T"].as<std::vector<double>>();
          std::vector<double> ext_robot_r = yaml["mapping"]["robot2imu_extrinsic_R"].as<std::vector<double>>();
          Vec3d Robot_T_wrt_IMU = math::VecFromArray(ext_robot_t);
          Mat3d Robot_R_wrt_IMU = math::MatFromArray(ext_robot_r);
          Eigen::Quaterniond q_IR(Robot_R_wrt_IMU);
          q_IR.normalized();
          TIR_ = SE3(q_IR, Robot_T_wrt_IMU);
          Eigen::Quaterniond q_RI(Robot_R_wrt_IMU.inverse());
          q_RI.normalized();
          TRI_ = SE3(q_RI, Robot_T_wrt_IMU);

          Eigen::Quaterniond q_init_odom(1, 0, 0, 0);
          Vec3d t_init_odom(0, 0, 0);
          last_pose_of_odom = SE3(q_init_odom, t_init_odom);
          last_pose_of_liwo = SE3(q_init_odom, t_init_odom);
          //   TODO: need init here
          CT_ICP::LidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::LidarPlaneNormFactor::q_il = TIL_.rotationMatrix();
          CT_ICP::CTLidarPlaneNormFactor::t_il = t_imu_lidar;
          CT_ICP::CTLidarPlaneNormFactor::q_il = TIL_.rotationMatrix();

          output_path = yaml["common"]["output_path"].as<std::string>();
          f.open(output_path+"/poseodom.txt");
          f << std::fixed;
          
          double imuhz = yaml["common"]["imuhz"].as<double>();
          imudt = (double)1/imuhz;
          
          model_path = yaml["common"]["model_path"].as<std::string>();

          loadOptions();
          if(options_.localization_mode)
          {
               loadScanContextRelocDB();
               loadHFNetRelocDB();
          }
          switch (options_.motion_compensation)
          {
               case NONE1:
               case CONSTANT_VELOCITY:
                    options_.point_to_plane_with_distortion = false;
                    options_.icpmodel = POINT_TO_PLANE;
                    break;
               case ITERATIVE:
                    options_.point_to_plane_with_distortion = true;
                    options_.icpmodel = POINT_TO_PLANE;
                    break;
               case CONTINUOUS:
                    options_.point_to_plane_with_distortion = true;
                    options_.icpmodel = CT_POINT_TO_PLANE;
                    break;
          }
          LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;
          
          return true;
     }
     
     void lidarodom::setgraph(std::shared_ptr<InteractiveGraph>& graph)
     {
          graph_ = graph;
          loop = new Loop(graph);
          loop->loopinit(output_path);
     }

     // void lidarodom::setgraph(std::shared_ptr<InteractiveGraph>& graph, std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>>& tmp_db)
     // {
     //      graph_ = graph;
     //      loop = new Loop(graph, tmp_db);
     // }

     void lidarodom::setMode(bool ol)
     {
          online = ol;
     }

     void lidarodom::loadScanContextRelocDB()
     {
          std::string filename = options_.scancontext_dbfile;
          std::ifstream in(filename, std::ios_base::binary);
          std::cout<<"-----------------------------\n";
          std::cout << "Loading hfnet reloc db file: " << filename << std::flush<<std::endl;
          boost::archive::binary_iarchive ia(in, boost::archive::no_header);
          ia >> scManager;
          in.close();
     }

     void lidarodom::loadHFNetRelocDB()
     {
          std::string filename = options_.hfnet_dbfile;
          std::ifstream in(filename, std::ios_base::binary);
          std::cout<<"-----------------------------\n";
          std::cout << "Loading scancontext reloc db file: " << filename << std::flush<<std::endl;
          boost::archive::binary_iarchive ia(in, boost::archive::no_header);
          ia >> hfNetManager;
          in.close();
     }


     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data, rclcpp::Time time)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
               ros_time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          ros_time_buffer_.push_back(time);
          // lidar_coeff_.push_back(CT_ICP::get_plane_coeffs<pcl::PointXYZI>(cloud_tmp));
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::pushData(std::vector<point3D> msg, std::pair<double, double> data, rclcpp::Time time, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp)
     {
          if (data.first < last_timestamp_lidar_)
          {
               LOG(ERROR) << "lidar loop back, clear buffer";
               lidar_buffer_.clear();
               time_buffer_.clear();
               ros_time_buffer_.clear();
          }

          mtx_buf.lock();
          lidar_buffer_.push_back(msg);
          time_buffer_.push_back(data);
          ros_time_buffer_.push_back(time);
          lidar_coeff_buffer_.push_back(CT_ICP::get_plane_coeffs<pcl::PointXYZI>(cloud_tmp));
          last_timestamp_lidar_ = data.first;
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::pushData(cv::Mat msg, double timestamp, rclcpp::Time time) {
          if (timestamp < last_timestamp_img_) {
               LOG(ERROR) << "odom loop back, clear buffer";
               img_buffer_.clear();
               img_time_buffer_.clear();
               img_ros_time_buffer_.clear();
          }

          mtx_buf.lock();
          img_buffer_.push_back(msg);
          img_time_buffer_.push_back(timestamp);
          img_ros_time_buffer_.push_back(time);
          last_timestamp_img_ = timestamp;
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::pushData(IMUPtr imu)
     {
          double timestamp = imu->timestamp_;
          if (timestamp < last_timestamp_imu_)
          {
               LOG(WARNING) << "imu loop back, clear buffer";
               imu_buffer_.clear();
          }

          last_timestamp_imu_ = timestamp;

          mtx_buf.lock();
          imu_buffer_.emplace_back(imu);
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::pushData(OdomposPtr odom)
     {
          double timestamp = odom->timestamp_;
          if (timestamp < last_timestamp_odom_)
          {
               LOG(WARNING) << "odom loop back, clear buffer";
               odom_buffer_.clear();
          }

          last_timestamp_odom_ = timestamp;
          Eigen::Quaterniond rotodom(odom->rot_[0], odom->rot_[1], odom->rot_[2], odom->rot_[3]);
          Eigen::Vector3d tranodom(odom->pos_[0], odom->pos_[1], odom->pos_[2]);
          // Eigen::Vector3d tofpos(0.2,0.1,0);
          Eigen::Vector3d tofpos(0.32,0,0);
          Eigen::Vector3d tranodom2=rotodom.matrix()*tofpos-tofpos+tranodom;
          odom->pos_ = tranodom2;
          // std::cout<<"ddd"<<tranodom2<<std::endl;
          // odom->pos_[0]=odom->pos_[0]+0.2;
          // odom->pos_[1]=odom->pos_[1]+7;
          // odom->pos_[2]=0;
          odom->pos_=TIR_*odom->pos_;    //odom坐标系转imu坐标系
          Eigen::Quaterniond Quaterniond2=Eigen::Quaterniond(TIR_.so3().matrix()*rotodom.normalized().toRotationMatrix()*TIR_.so3().matrix().inverse());  
          odom->rot_[0] = Quaterniond2.w();
          odom->rot_[1] = Quaterniond2.x();
          odom->rot_[2] = Quaterniond2.y();
          odom->rot_[3] = Quaterniond2.z();

          Eigen::Vector3d tofvel(odom->vx_,odom->vy_,0);
          tofvel=TIR_*tofvel;
          odom->vx_ = tofvel[0];
          odom->vy_ = tofvel[1];
          odom->vz_ = tofvel[2];

          mtx_buf.lock();
          odom_buffer_.emplace_back(odom);
          mtx_buf.unlock();
          cond.notify_one();
     }

     void lidarodom::run(){
          if(options_.localization_mode)
          {
               pcl::io::loadPCDFile(options_.mapfile, *map_cloud_ptr);
               // Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
               // transform.block<3, 3>(0, 0) = TIR_.so3().matrix();
               // transform.block<3, 1>(0, 3) = TIR_.translation();
               // pcl::transformPointCloud (*map_cloud_ptr, *map_cloud_ptr, transform);
               loadmap(map_cloud_ptr);
          }

          bool repeat=false;
          if(online)
          {
               repeat = true;
          }
          do
          {
               
               std::vector<MeasureGroup> measurements;
               std::unique_lock<std::mutex> lk(mtx_buf);
               cond.wait(lk, [&]
                         { return (measurements = getMeasureMents()).size() != 0; });
               lk.unlock();

               for (auto &m : measurements){

                    auto t1 = std::chrono::steady_clock::now();

                    zjloc::common::Timer::Evaluate([&]()
                                                   { ProcessMeasurements(m); },
                                                   "processMeasurement");

                    auto t2 = std::chrono::steady_clock::now();
                    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
                    std::cout<<"ProcessMeasurementsTime:"<<time_used<<std::endl;
                    std::cout<<"end frame"<<std::endl;
                    // auto real_time = std::chrono::high_resolution_clock::now();
                    // static std::chrono::system_clock::time_point prev_real_time = real_time;

                    // if (real_time - prev_real_time > std::chrono::seconds(5)){
                    //      auto data_time = m.lidar_end_time_;
                    //      static double prev_data_time = data_time;
                    //      auto delta_real = std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                    //      auto delta_sim = data_time - prev_data_time;
                    //      printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                    //      prev_data_time = data_time;
                    //      prev_real_time = real_time;
                    // }

                    
               }
          }
          while(repeat);

     }

     bool use_icp(SE3 odom, SE3 lastodom, SE3 cloudpose, SE3 lastcloudpose)
     {

          Eigen::Vector3d deltaodom = odom.translation()-lastodom.translation();
          Eigen::Vector2d deltaodomplane(deltaodom[0],deltaodom[2]);
          Eigen::Vector3d deltacloud = cloudpose.translation()-lastcloudpose.translation();
          Eigen::Vector2d deltacloudplane(deltacloud[0],deltacloud[2]);
          double Rdelta = (deltacloudplane-deltaodomplane).norm();
          std::cout<<"R-delta:"<<Rdelta<<std::endl;
          // std::cout<<"deltaodomplane:"<<deltaodomplane.norm()<<std::endl;
          if(Rdelta>0.02)
          {
               return false;
          }

          Eigen::Vector2d odomplane(odom.translation()[0],odom.translation()[2]);
          Eigen::Vector2d cloudplane(cloudpose.translation()[0],cloudpose.translation()[2]);
          double Adelta = (cloudplane-odomplane).norm();
          std::cout<<"A-delta:"<<Adelta<<std::endl;
          // if(Adelta>0.2)
          // {
          //      return false;
          // }
          return true;
     }

     std::vector<OdometryFrame::Ptr> lidarodom::getkeyframes()
     {
          // std::vector<OdometryFrame::Ptr> myVector(5);
          // for(int i=0;i<1;i++)
          // {
          //      myVector[i]=keyframes[0];
          //      std::cout<<"odom:"<<myVector[i]->raw_cloud->size()<<std::endl;
          // }
          // std::cout<<"odom2:"<<keyframes[0]->raw_cloud->size()<<std::endl;
          // std::cout<<"odom2:"<<keyframes[0]->timestamp<<std::endl;
          return keyframes;

     }

     // void lidarodom::select_keyframes(float keyframe_delta_x, float keyframe_delta_angle)
     // {
     //      if(frames.empty()) {
     //       return;
     //      }

     //      keyframes.clear();
     //      keyframes.push_back(frames.front());
     //      for(const auto& frame : frames) {
     //           const auto& last_keyframe_pose = keyframes.back()->pose;
     //           Eigen::Isometry3d last_keyframe_pose_T = Eigen::Isometry3d::Identity();
     //           last_keyframe_pose_T.rotate(last_keyframe_pose.so3().matrix());
     //           last_keyframe_pose_T.pretranslate(last_keyframe_pose.translation());                                                  
     //           const auto& current_frame_pose = frame->pose;
     //           Eigen::Isometry3d current_frame_pose_T = Eigen::Isometry3d::Identity();
     //           current_frame_pose_T.rotate(current_frame_pose.so3().matrix());
     //           current_frame_pose_T.pretranslate(current_frame_pose.translation());   

     //            Eigen::Isometry3d delta = last_keyframe_pose_T.inverse() * current_frame_pose_T;
     //            double delta_x = delta.translation().norm();
     //            double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

     //            if(delta_x > keyframe_delta_x || delta_angle > keyframe_delta_angle) {
     //            keyframes.push_back(frame);
     //           }
     //       }
     // }

     // void lidarodom::saveg2o()
     // {
     //      std::ofstream ofs("/home/mengxinrui/ROS/ctlio_ws_loop/test.g2o");
     //      if(!ofs) {
     //           std::cout<<"wrong!"<<std::endl;
     //         return;
     //      }
     //      for(int i = 0; i < keyframes.size(); i++) 
     //      {
     //          std::unique_ptr<g2o::VertexSE3> v(new g2o::VertexSE3());
     //          Eigen::Isometry3d keyframes_i = Eigen::Isometry3d::Identity();
     //          keyframes_i.rotate(keyframes[i]->pose.so3().matrix());
     //          keyframes_i.pretranslate(keyframes[i]->pose.translation());  
     //          v->setEstimate(keyframes_i);
     //          ofs << "VERTEX_SE3:QUAT " << i << " ";
     //          v->write(ofs);
     //          ofs << std::endl;
     //      }
     //      ofs << "FIX 0" << std::endl;
     //      for(int i = 0; i < keyframes.size() - 1; i++) {
     //          Eigen::Isometry3d keyframes_i = Eigen::Isometry3d::Identity();
     //          keyframes_i.rotate(keyframes[i]->pose.so3().matrix());
     //          keyframes_i.pretranslate(keyframes[i]->pose.translation()); 
     //          Eigen::Isometry3d keyframes_i2 = Eigen::Isometry3d::Identity();
     //          keyframes_i2.rotate(keyframes[i+1]->pose.so3().matrix());
     //          keyframes_i2.pretranslate(keyframes[i+1]->pose.translation()); 

     //          const auto& delta_pose = keyframes_i.inverse() * keyframes_i2;
     //          std::unique_ptr<g2o::EdgeSE3> e(new g2o::EdgeSE3());
     //          e->setMeasurement(delta_pose);

     //          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
     //          inf.block<3, 3>(0, 0) *= 10.0;
     //          inf.block<3, 3>(3, 3) *= 20.0;

     //          e->setInformation(inf);
     //          ofs << "EDGE_SE3:QUAT " << i << " " << i + 1 << " ";
     //          e->write(ofs);
     //          ofs << std::endl;
     //      }

     //      ofs.close();
     // }

     void lidarodom::ProcessMeasurements(MeasureGroup &meas)
     {
          measures_ = meas;
          if (measures_.imu_.size() == 1)
          {
               return;
          }

          if (measures_.odom_ == nullptr)
          {
               // std::cout<<"no odom"<<std::endl;
               return;
          }

          if (measures_.lidar_.size()==0)
          {
               // std::cout<<"no odom"<<std::endl;
               return;
          }
          
          if (imu_need_init_)
          {
               TryInitIMU();
               return;
          }

          std::cout << ANSI_DELETE_LAST_LINE;
          std::cout << ANSI_COLOR_GREEN << "============== process frame: "
                    << index_frame << ANSI_COLOR_RESET << std::endl;
          imu_states_.clear(); //   need clear here
          // while (graph_->isopt)
          // {
          //      std::cout << "opt" << std::endl;
          // }
          if(!options_.localization_mode)
          {
               if (loop->getoptnum() > optnum)
               {
                    // 更新地图
                    //  std::cout<<loop->getoptnum()<<std::endl;
                    updatemap(loop->get_pointcloud());
                    optnum = loop->getoptnum();

                    // 优化后pose更新eskf
                    int lastid = graph_->optlastid; //graph_->vectorkeyframes.size() - 1;
                    // std::cout<<lastid<<std::endl;
                    Eigen::Isometry3d lastgraphpose = graph_->vectorkeyframes[lastid]->node->estimate();
                    // Eigen::Quaterniond lastgraphrotation(lastgraphpose.rotation());
                    // Eigen::Vector3d lastgraphposition = lastgraphpose.translation();
                    Eigen::Isometry3d beforeopt = keyframes[lastid]->pose;
                    optdeltapose = lastgraphpose * beforeopt.inverse();

                    // while(1){}
                    // std::cout<<lastgraphrotation.x()<<" "<<lastgraphrotation.y()<<" "<<lastgraphrotation.z()<<" "<<lastgraphrotation.w()<<std::endl;
                    // std::cout<<lastgraphposition<<std::endl;
                    // Eigen::Quaterniond lastgraphrotationq(lastgraphrotation);
                    // SE3 pose_of_graph = SE3(newesrotation, newesphposition);
                    // {
                    //      zjloc::common::Timer::Evaluate([&]()
                    //                                     { eskf_.ObserveSE3(pose_of_graph, 0.01, 0.01); },
                    //                                     "eskf_graphobs");
                    // }
                    updategraph = true;
               }
          }

          double delta_dis = std::sqrt(meas.odom_->vx_* meas.odom_->vx_+ meas.odom_->vy_* meas.odom_->vy_+ meas.odom_->vz_* meas.odom_->vz_)*0.1;

          // 利用IMU数据进行状态预测
          // P
          // auto t1 = std::chrono::steady_clock::now();
          zjloc::common::Timer::Evaluate([&]()
                                         { Predict(); },
                                         "predict");
          // auto t2 = std::chrono::steady_clock::now();
          // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "predict time:" << time_used << std::endl;

          Eigen::Quaterniond rotodom(meas.odom_->rot_[0], meas.odom_->rot_[1], meas.odom_->rot_[2], meas.odom_->rot_[3]);
          Eigen::Vector3d tranodom(meas.odom_->pos_[0], meas.odom_->pos_[1], meas.odom_->pos_[2]);
          SE3 pose_of_odom = SE3(rotodom, tranodom);
          Eigen::Vector3d odomdeltat = pose_of_odom.translation() - last_pose_of_odom.translation();
          Eigen::Matrix3d odomdeltar = pose_of_odom.so3().matrix() * last_pose_of_odom.so3().matrix().inverse();
          Eigen::Matrix3d ekfodomr = odomdeltar * imu_states_.front().R_.matrix();
          Eigen::Quaterniond ekfodomq(ekfodomr);
          Eigen::Vector3d ekfodomt = imu_states_.front().p_ + odomdeltat;
          SE3 update_odom_pose = SE3(ekfodomq, ekfodomt);

          // SE3 update_odom_pose = pose_of_odom*last_pose_of_odom.inverse()*last_pose_of_liwo;
          // Eigen::Vector3d tofpos(0.2,0.1,0);
          // Eigen::Vector3d tranodom2=pose_of_odom*tofpos-tofpos;
          // SE3 pose_of_odom2 = SE3(rotodom, tranodom2);
          // pose_of_odom = TIR_*pose_of_odom;
          // Eigen::Vector3d eulerAngle = pose_of_odom.so3().matrix().eulerAngles(2,1,0);
          // double z = eulerAngle[1] * 180.0 / 3.1415926;
          // std::cout<<"aaa:"<<pose_of_odom.translation()<<std::endl;
          // std::cout<<"bbb:"<<update_odom_pose.translation()<<std::endl;
          // std::cout<<"cccccccccccccccccccccccc"<<std::endl;
          // std::cout<<"aaa:"<<z<<std::endl;
          // std::cout<<"bbb:"<<z3<<std::endl;
          // t1 = std::chrono::steady_clock::now();
          if(options_.wheelmode==0)
          {
               if (last_pose_of_odom.translation().norm() != 0) // 防止odom初始数据不是原点的情况
               {
                    zjloc::common::Timer::Evaluate([&]()
                                                  { eskf_.ObserveSE3(update_odom_pose, 0.1, 0.1); },
                                                  "eskf_odomobs");
                    imu_states_.emplace_back(eskf_.GetNominalState());
               }
          }
          else
          {
               if(options_.wheelmode==1)
               {
                    zjloc::common::Timer::Evaluate([&]()
                                                  { eskf_.ObserveWheelSpeed(*meas.odom_); },
                                                  "eskf_odomobs");
                    imu_states_.emplace_back(eskf_.GetNominalState());
               }
          }
          if(updategraph)
          {
               auto es = eskf_.GetNominalState();
               Eigen::Matrix3d esr = es.R_.matrix();
               Eigen::Vector3d est = es.p_;
               Eigen::Isometry3d esT = Eigen::Isometry3d::Identity();
               esT.rotate(esr);
               esT.pretranslate(est);
               Eigen::Isometry3d newes = optdeltapose * esT;
               Eigen::Quaterniond newesrotation(newes.rotation());
               Eigen::Vector3d newesphposition = newes.translation();
               qn = newesrotation;
               tn = newesphposition;
          }
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "odom update time:" << time_used << std::endl;
          // imu_states_.emplace_back(eskf_.GetNominalState()); //初值！！！上一轮的icp结果更新+imu预测+这一轮轮速更新后的值
          // zjloc::common::Timer::Evaluate([&]()
          //                                { eskf_.ObserveWheelPos(update_odom_pose.translation(), 1.0); },
          //                                "eskf_odomobs");
          // imu_states_.emplace_back(eskf_.GetNominalState());
          //
          // t1 = std::chrono::steady_clock::now();
          if(options_.localization_mode)
          {
               if (options_.reloc_mode == 0) {
               zjloc::common::Timer::Evaluate([&]()
                                             { relocInitialization(meas.lidar_); },
                                             "state init");
               } else if (options_.reloc_mode == 1){
                zjloc::common::Timer::Evaluate([&]()
                                             { relocInitialization(meas.lidar_, meas.img_); },
                                             "state init");
               } else {
                    zjloc::common::Timer::Evaluate([&]()
                                             { stateInitialization(); },
                                             "state init");
               }

               if(!init_success && (options_.reloc_mode==0 || options_.reloc_mode==1))
               {
                   index_frame=1;
                   return;
               } 
          }
          else
          {
               zjloc::common::Timer::Evaluate([&]()
                                         { stateInitialization(); },
                                         "state init");
          }

         
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "state init time:" << time_used << std::endl;

          // SE3 pose_of_wio_ = SE3(imu_states_.back().R_.matrix(), imu_states_.back().p_);
          std::vector<point3D> const_surf;
          const_surf.insert(const_surf.end(), meas.lidar_.begin(), meas.lidar_.end());
          // const_surf.assign(meas.lidar_.begin(), meas.lidar_.end());
          // t1 = std::chrono::steady_clock::now();
          cloudFrame *p_frame;
          zjloc::common::Timer::Evaluate([&]()
                                         { p_frame = buildFrame(const_surf, current_state,
                                                                meas.lidar_begin_time_,
                                                                meas.lidar_end_time_,
                                                                meas.lidar_coeff_); },
                                         "build frame");
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "build frame time:" << time_used << std::endl;
          //   lio
          // t1 = std::chrono::steady_clock::now();
          zjloc::common::Timer::Evaluate([&]()
                                         { poseEstimation(p_frame); },
                                         "poseEstimate");
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "poseEstimate time:" << time_used << std::endl;
          //   观测
          // Eigen::Quaterniond updateq = Eigen::Quaterniond(eskf_.GetNominalState().R_.matrix());
          // Eigen::Vector3d updatet = eskf_.GetNominalState().p_;
          // SE3 pose_of_lo_ = SE3(updateq, updatet);
          SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
          Eigen::Quaterniond q_current(pose_of_lo_.so3().matrix());
          f << std::setprecision(6) << meas.lidar_begin_rostime_.seconds() << " " <<  std::setprecision(9) << pose_of_lo_.translation().x() << " " << pose_of_lo_.translation().y() << " " << pose_of_lo_.translation().z() << " " << q_current.x() << " " << q_current.y() << " " << q_current.z() << " " << q_current.w() << endl;
          // Eigen::Vector3d eulerAngle2 = current_state->rotation.matrix().eulerAngles(2,1,0);
          // Eigen::Quaterniond q_current(pose_of_lo_.so3().matrix());
          // f << std::setprecision(6) << meas.lidar_begin_rostime_.seconds() << " " <<  std::setprecision(9) << pose_of_lo_.translation().x() << " " << pose_of_lo_.translation().y() << " " << pose_of_lo_.translation().z() << " " << q_current.x() << " " << q_current.y() << " " << q_current.z() << " " << q_current.w() << endl;
          // Eigen::Vector3d eulerAngle2 = current_state->rotation.matrix().eulerAngles(2,1,0);
          // double z2 = eulerAngle2[1] * 180.0 / 3.1415926;
          // std::cout << "obs: " << current_state->translation.transpose() << ", " << z2 << std::endl;
          // std::cout<<"aaaaaaaaaaaaa"<<std::endl;
          // SE3 pred_pose = eskf_.GetNominalSE3();
          // std::cout << "pred: " << pred_pose.translation().transpose() << ", " << pred_pose.so3().log().transpose() << std::endl;
          // t1 = std::chrono::steady_clock::now();
          // bool icpupdate = use_icp(pose_of_odom, last_pose_of_odom, pose_of_lo_, last_pose_of_liwo);
          // bool icpupdate = use_icp2(delta_dis, pose_of_lo_, last_pose_of_liwo);
          if(options_.wheelmode==0)
          {
               bool icpupdate = use_icp(pose_of_odom, last_pose_of_odom, pose_of_lo_, last_pose_of_liwo);
               // bool icpupdate = use_icp2(delta_dis, pose_of_lo_, last_pose_of_liwo);
               if (icpupdate || last_pose_of_odom.translation().norm() == 0 || updategraph) ////防止odom初始数据不是原点的情况
               {
                    zjloc::common::Timer::Evaluate([&]()
                                                  { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                                  "eskf_obs");
                    right_pose_of_odom = pose_of_odom;
                    right_pose_of_liwo = pose_of_lo_;
                    
               }
               else
               {
                    Eigen::Vector3d rightodomdeltat = pose_of_odom.translation() - right_pose_of_odom.translation();
                    Eigen::Matrix3d rightodomdeltar = pose_of_odom.so3().matrix() * right_pose_of_odom.so3().matrix().inverse();
                    Eigen::Matrix3d rightekfodomr = rightodomdeltar * right_pose_of_liwo.so3().matrix();
                    Eigen::Quaterniond rightekfodomq(rightekfodomr);
                    Eigen::Vector3d rightekfodomt = right_pose_of_liwo.translation() + rightodomdeltat;
                    SE3 update_right_pose = SE3(rightekfodomq, rightekfodomt);

                    zjloc::common::Timer::Evaluate([&]()
                                                  { eskf_.ObserveSE3(update_right_pose, 0.01, 0.01); },
                                                  "eskf_odomobs");
               }
          }
          else
          {
               if(options_.wheelmode==1)
               {
                    zjloc::common::Timer::Evaluate([&]()
                                                  { eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2); },
                                                  "eskf_obs");
               }
          }
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "icp update time:" << time_used << std::endl;
          // Eigen::Quaterniond updateq = Eigen::Quaterniond(eskf_.GetNominalState().R_.matrix());
          // Eigen::Vector3d updatet = eskf_.GetNominalState().p_;
          // SE3 pose_of_ekf_ = SE3(updateq, updatet);
          SE3 pose_of_pub2 = TRI_ * pose_of_lo_ * TIL_; // robot坐标系
          SE3 pose_of_pub = pose_of_lo_ * TIL_;         // imu坐标系
          last_pose_of_odom = pose_of_odom;
          last_pose_of_liwo = pose_of_lo_;

          // Eigen::Quaterniond q_current(pose_of_pub.so3().matrix());
          // f << std::setprecision(6) << meas.lidar_begin_rostime_.seconds() << " " << std::setprecision(9) << pose_of_pub.translation().x() << " " << pose_of_pub.translation().y() << " " << pose_of_pub.translation().z() << " " << q_current.x() << " " << q_current.y() << " " << q_current.z() << " " << q_current.w() << endl;
          // pcl::io::savePCDFileBinary(output_path+std::to_string(meas.lidar_end_time_)+".pcd", *points_distort);
          // std::ofstream f;
          // f.open("/home/mengxinrui/ROS/ctlio_ws_loop/cloud/"+std::to_string(meas.lidar_end_time_)+".txt");
          // f<<pose_of_pub.translation().x()<<" "<<pose_of_pub.translation().y()<<" "<<pose_of_pub.translation().z()<<" "<<q_current.x()<<" "<<q_current.y()<<" "<<q_current.z()<<" "<<q_current.w()<<std::endl;
          // f.close();
          // Eigen::Vector3d eulerAngle2 = current_state->rotation.matrix().eulerAngles(2,1,0);
          // t1 = std::chrono::steady_clock::now();
          if(!options_.localization_mode)
          {
               // 插入关键帧
               Eigen::Isometry3d pose_of_pub_T = Eigen::Isometry3d::Identity();
               pose_of_pub_T.rotate(pose_of_lo_.so3().matrix());
               pose_of_pub_T.pretranslate(pose_of_lo_.translation());

               OdometryFrame::Ptr frame=std::make_shared<OdometryFrame>(*points_distort,pose_of_pub_T,meas.lidar_begin_rostime_.seconds(), measures_.img_);
               // frame->cloud(0.02);
               if(!frame->img_.empty())
               {
                    if (keyframes.size() == 0)
                    {
                         keyframes.push_back(frame);
                         loop->updatekeyframe(frame);
                         loop->updategraph();
                         loopinterval++;
                    }
                    else
                    {
                         const auto &last_keyframe_pose_T = keyframes.back()->pose;
                         // Eigen::Isometry3d last_keyframe_pose_T = Eigen::Isometry3d::Identity();
                         // last_keyframe_pose_T.rotate(last_keyframe_pose.so3().matrix());
                         // last_keyframe_pose_T.pretranslate(last_keyframe_pose.translation());
                         const auto &current_frame_pose_T = frame->pose;
                         // Eigen::Isometry3d current_frame_pose_T = Eigen::Isometry3d::Identity();
                         // current_frame_pose_T.rotate(current_frame_pose.so3().matrix());
                         // current_frame_pose_T.pretranslate(current_frame_pose.translation());

                         Eigen::Isometry3d delta = last_keyframe_pose_T.inverse() * current_frame_pose_T;
                         double delta_x = delta.translation().norm();
                         double delta_angle = Eigen::AngleAxisd(delta.linear()).angle();

                         if ((delta_x > 0.2 || delta_angle > 1.0))
                         {
                              // if(!graph_->isopt)
                              {
                              keyframes.push_back(frame);
                              loop->updatekeyframe(frame);
                              loop->updategraph();
                              loopinterval++;
                              }
                         }
                    }
               }
               points_distort->clear();
          }
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "graph update time:" << time_used << std::endl;
          // Eigen::Isometry3d frame2keyframe = pose_of_pub_T*keyframes[keyframes.size()-1].inverse();

          points_world->clear();
          
          // t1 = std::chrono::steady_clock::now();
          zjloc::common::Timer::Evaluate([&]()
                                         {
               std::string laser_topic = "laser";
               pub_pose_to_ros(laser_topic, pose_of_lo_, pose_of_pub2, meas.lidar_end_time_);
               // pub_pose_to_ros(laser_topic, pose_of_lo_, pose_of_pub, meas.lidar_begin_rostime_);
               laser_topic = "velocity";
               SE3 pred_pose = eskf_.GetNominalSE3();
               Eigen::Vector3d vel_world = eskf_.GetNominalVel();
               Eigen::Vector3d vel_base = pred_pose.rotationMatrix().inverse()*vel_world;
               pub_data_to_ros(laser_topic, vel_base.x(), 0);
               if(index_frame%8==0)
               {
                    laser_topic = "dist";
                    static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                    Eigen::Vector3d t = pred_pose.translation();
                    static double dist = 0;
                    dist += (t - last_t).norm();
                    last_t = t;
                    pub_data_to_ros(laser_topic, dist, 0);
                    // std::cout << eskf_.GetGravity().transpose() << std::endl;
               } },
                                         "pub cloud");
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "pub time:" << time_used << std::endl;

          p_frame->p_state = new state(current_state, true);
          // all_cloud_frame.push_back(p_frame); //   TODO:     保存这个，特别费内存
          state *tmp_state = new state(current_state, true);
          all_state_frame.push_back(tmp_state);
          current_state = new state(current_state, false);

          index_frame++;
          p_frame->release();
          std::vector<point3D>().swap(meas.lidar_);
          std::vector<point3D>().swap(const_surf);
          // t1 = std::chrono::steady_clock::now();
          if(!options_.localization_mode)
          {
               if (loopinterval >= 1) // 1个关键帧执行loop一次
               {
                    loopinterval = 0;
                    // std::cout<<index_frame<<std::endl;
                    if (!loop->isrunningdetection()) // loop没执行完，不启动loop
                    {
                         loop->setAutomaticLoopCloseWindow(keyframes.size() - 1);
                         loop->LoopDetection();
                    }
               }

               updategraph = false;
          }
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout << "loop time:" << time_used << std::endl;
     }

     void lidarodom::poseEstimation(cloudFrame *p_frame)
     {
          // if(loop->getoptnum()>optnum)
          // {
          //      std::cout<<loop->getoptnum()<<std::endl;
          //      updatemap(loop->get_pointcloud());
          //      optnum = loop->getoptnum();
          // }
          //   TODO: check current_state data
          // auto t1 = std::chrono::steady_clock::now();
          if (index_frame > 1)
          {
               zjloc::common::Timer::Evaluate([&]()
                                              { optimize(p_frame); }, 
                                              "optimize");
          }
          // auto t2 = std::chrono::steady_clock::now();
          // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout<<ANSI_COLOR_GREEN<<"optimize time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
          // SE3 pose_of_lo_ = SE3(current_state->rotation, current_state->translation);
          // eskf_.ObserveSE3(pose_of_lo_, 1e-2, 1e-2);
          // Eigen::Quaterniond updateq = Eigen::Quaterniond(eskf_.GetNominalState().R_.matrix());
          // Eigen::Vector3d updatet = eskf_.GetNominalState().p_;
          // // auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          // {
          //      Eigen::Matrix3d R;
          //      Eigen::Vector3d t;
          //      for (auto &keypoint : p_frame->point_surf)
          //      {
          //           // if (options_.point_to_plane_with_distortion ||
          //           //     options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
          //           // {
          //           //      double alpha_time = keypoint.alpha_time;

          //           //      Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
          //           //      q.normalize();
          //           //      R = q.toRotationMatrix();
          //           //      t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
          //           // }
          //           // else
          //           {
          //                R = updateq.normalized().toRotationMatrix();
          //                t = updatet;
          //           }
          //           keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
          //      }
          // }
          // t1 = std::chrono::steady_clock::now();
          if(options_.localization_mode)
          {
               if(online)
               {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_pub;
                    pcl::VoxelGrid<pcl::PointXYZI> vg;
                    vg.setInputCloud (map_cloud_ptr);
                    vg.setLeafSize (0.1, 0.1, 0.1);
                    vg.filter (*map_cloud_ptr);
                    std::string laser_topic = "laser";
                    pub_cloud_to_ros(laser_topic, map_cloud_ptr, p_frame->time_frame_end, 3);
               }
          }
          
          bool add_points = true;
          if (add_points)
          { //   update map here
               zjloc::common::Timer::Evaluate([&]()
                                              { map_incremental(p_frame); },
                                              "map update");
          }
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout<<ANSI_COLOR_GREEN<<"map update time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;

          // t1 = std::chrono::steady_clock::now();
          // zjloc::common::Timer::Evaluate([&]()
          //                                { lasermap_fov_segment(); },
          //                                "fov segment");
          // t2 = std::chrono::steady_clock::now();
          // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
          // std::cout<<ANSI_COLOR_GREEN<<"fov segment time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
     }

     void lidarodom::optimize(cloudFrame *p_frame)
     {

          state *previous_state = nullptr;
          Eigen::Vector3d previous_translation = Eigen::Vector3d::Zero();
          Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
          Eigen::Quaterniond previous_orientation = Eigen::Quaterniond::Identity();

          state *curr_state = p_frame->p_state;
          Eigen::Quaterniond begin_quat = Eigen::Quaterniond(curr_state->rotation_begin);
          Eigen::Quaterniond end_quat = Eigen::Quaterniond(curr_state->rotation);
          Eigen::Vector3d begin_t = curr_state->translation_begin;
          Eigen::Vector3d end_t = curr_state->translation;

          // double end_t_x = end_t[0];
          // double end_t_y = end_t[1];
          // double end_t_z = end_t[2];

          if (p_frame->frame_id > 1)
          {
               if (options_.log_print)
                    std::cout << "all_cloud_frame.size():" << all_state_frame.size() << ", " << p_frame->frame_id << std::endl;
               // previous_state = all_cloud_frame[p_frame->frame_id - 2]->p_state;
               previous_state = all_state_frame[p_frame->frame_id - 2];
               previous_translation = previous_state->translation;
               previous_velocity = previous_state->translation - previous_state->translation_begin;
               previous_orientation = previous_state->rotation;

               // if (p_frame->frame_id > 4) return;

               // if(std::sqrt(previous_velocity(0, 1) * previous_velocity(0, 1) + 
               //                previous_velocity(1, 1) * previous_velocity(1, 1) + 
               //                previous_velocity(2, 1) * previous_velocity(2, 1)) < 0.01) return;
          } 
          if (options_.log_print)
          {
               std::cout << "prev end: " << previous_translation.transpose() << std::endl;
               std::cout << "curr begin: " << p_frame->p_state->translation_begin.transpose()
                         << "\ncurr end: " << p_frame->p_state->translation.transpose() << std::endl;
          }
          std::cout << "----------------------point coeff------------------------" << std::endl;
          std::cout << "coeff: " << p_frame->point_coeff[0] << " " << p_frame->point_coeff[1] 
               << " " << p_frame->point_coeff[2] << " " << p_frame->point_coeff[3] << std::endl;

          double dist_tmp = std::sqrt(std::pow(previous_translation[0], 2) + std::pow(previous_translation[2], 2));
          if (p_frame->frame_id < 20) {
          // if (dist_tmp < 0.01) {
               if (p_frame->frame_id == 10) {
                    CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_height = end_t[1];
                    CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff = p_frame->point_coeff;
               }
               return;
          }

          std::vector<point3D> surf_keypoints;
          gridSampling(p_frame->point_surf, surf_keypoints,
                       options_.sampling_rate * options_.surf_res);

          size_t num_size = p_frame->point_surf.size();

          auto transformKeypoints = [&](std::vector<point3D> &point_frame)
          {
               Eigen::Matrix3d R;
               Eigen::Vector3d t;
               for (auto &keypoint : point_frame)
               {
                    if (options_.point_to_plane_with_distortion ||
                        options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
                    {
                         double alpha_time = keypoint.alpha_time;

                         Eigen::Quaterniond q = begin_quat.slerp(alpha_time, end_quat);
                         q.normalize();
                         R = q.toRotationMatrix();
                         t = (1.0 - alpha_time) * begin_t + alpha_time * end_t;
                    }
                    else
                    {
                         R = end_quat.normalized().toRotationMatrix();
                         t = end_t;
                    }
                    keypoint.point = R * (TIL_ * keypoint.raw_point) + t;
               }
          };
          // auto t11 = std::chrono::steady_clock::now();
          // std::cout << "============================before p_state->translation.x(): " << p_frame->p_state->translation[0] 
          //       << " " << p_frame->p_state->translation[1] << " " << p_frame->p_state->translation[2] << std::endl;
          //      std::cout << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() 
          //                          << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w() << std::endl;
          for (int iter(0); iter < options_.max_num_iteration; iter++)
          {
               // auto t1 = std::chrono::steady_clock::now();
               transformKeypoints(surf_keypoints);
               // auto t2 = std::chrono::steady_clock::now();
               // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_RED<<"transformKeypoints time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;

               // ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1 / (1.5e-3));
               ceres::LossFunction *loss_function = new ceres::HuberLoss(0.5);
               ceres::Problem::Options problem_options;
               ceres::Problem problem(problem_options);
#ifdef USE_ANALYTICAL_DERIVATE
               ceres::LocalParameterization *parameterization = new RotationParameterization();
#else
               auto *parameterization = new ceres::EigenQuaternionParameterization();
#endif

               switch (options_.icpmodel)
               {
               case IcpModel::CT_POINT_TO_PLANE:
                    problem.AddParameterBlock(&begin_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&begin_t.x(), 3);
                    problem.AddParameterBlock(&end_t.x(), 3);
                    break;
               case IcpModel::POINT_TO_PLANE:
                    problem.AddParameterBlock(&end_quat.x(), 4, parameterization);
                    problem.AddParameterBlock(&end_t.x(), 3);
                    // problem.AddParameterBlock(&end_t_x, 1);
                    // problem.AddParameterBlock(&end_t_y, 1);
                    // problem.AddParameterBlock(&end_t_z, 1);
                    break;
               }
               // t1 = std::chrono::steady_clock::now();
               std::vector<ceres::CostFunction *> surfFactor;
               std::vector<Eigen::Vector3d> normalVec;
               addSurfCostFactor(surfFactor, normalVec, surf_keypoints, p_frame);
               // t2 = std::chrono::steady_clock::now();
               // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_RED<<"addSurfCostFactor time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               //   TODO: 退化后，该如何处理
               checkLocalizability(normalVec);
               // t1 = std::chrono::steady_clock::now();
               int surf_num = 0;
               if (options_.log_print)
                    std::cout << "get factor: " << surfFactor.size() << std::endl;
               for (auto &e : surfFactor)
               {
                    surf_num++;
                    switch (options_.icpmodel)
                    {
                    case IcpModel::CT_POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
                         break;
                    case IcpModel::POINT_TO_PLANE:
                         problem.AddResidualBlock(e, loss_function, &end_t.x(), &end_quat.x());
                         // problem.AddResidualBlock(e, loss_function, &end_t_x, &end_t_y, &end_t_z, &end_quat.x());
                         break;
                    }
                    // if (surf_num > options_.max_num_residuals)
                    //      break;
               }
               // t2 = std::chrono::steady_clock::now();
               // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_RED<<"AddResidualBlock time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               //   release
               std::vector<Eigen::Vector3d>().swap(normalVec);
               std::vector<ceres::CostFunction *>().swap(surfFactor);
               std::cout << "options_.icpmodel: " << options_.icpmodel << std::endl;
               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    if (options_.beta_location_consistency > 0.) //   location consistency
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::LocationConsistencyFactor *cost_location_consistency =
                             new CT_ICP::LocationConsistencyFactor(previous_translation, sqrt(surf_num * options_.beta_location_consistency * laser_point_cov));
#else
                         auto *cost_location_consistency =
                             CT_ICP::LocationConsistencyFunctor::Create(previous_translation, sqrt(surf_num * options_.beta_location_consistency));
#endif
                         problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_t.x());
                    }

                    if (options_.beta_orientation_consistency > 0.) // orientation consistency
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::RotationConsistencyFactor *cost_rotation_consistency =
                             new CT_ICP::RotationConsistencyFactor(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency * laser_point_cov));
#else
                         auto *cost_rotation_consistency =
                             CT_ICP::OrientationConsistencyFunctor::Create(previous_orientation, sqrt(surf_num * options_.beta_orientation_consistency));
#endif
                         problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quat.x());
                    }

                    if (options_.beta_small_velocity > 0.) //     small velocity
                    {
#ifdef USE_ANALYTICAL_DERIVATE
                         CT_ICP::SmallVelocityFactor *cost_small_velocity =
                             new CT_ICP::SmallVelocityFactor(sqrt(surf_num * options_.beta_small_velocity * laser_point_cov));
#else
                         auto *cost_small_velocity =
                             CT_ICP::SmallVelocityFunctor::Create(sqrt(surf_num * options_.beta_small_velocity));
#endif
                         problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_t.x(), &end_t.x());
                    }

                    // if (options_.beta_constant_velocity > 0.) //  const velocity
                    // {
                    //      CT_ICP::VelocityConsistencyFactor2 *cost_velocity_consistency =
                    //          new CT_ICP::VelocityConsistencyFactor2(previous_velocity, sqrt(surf_num * options_.beta_constant_velocity * laser_point_cov));
                    //      problem.AddResidualBlock(cost_velocity_consistency, nullptr, PR_begin, PR_end);
                    // }
               }

               double tmp_y = end_t[1];
               
               // CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff = p_frame->point_coeff;
               double normal_thresh_coeff = std::sqrt(std::pow(CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff[0] - p_frame->point_coeff[0], 2)
                                                       + std::pow(CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff[1] - p_frame->point_coeff[1], 2)
                                                       + std::pow(CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff[2] - p_frame->point_coeff[2], 2));
               double dist_thresh_coeff = std::sqrt(std::pow(CT_ICP::Cost_NavState_PR_Ground::init_ground_plane_coeff[3] - p_frame->point_coeff[3], 2));
               
               std::cout << "normal_thresh_coeff: " << normal_thresh_coeff << std::endl;
               std::cout << "dist_thresh_coeff: " << dist_thresh_coeff << std::endl;
               // if (normal_thresh_coeff < 0.2 && dist_thresh_coeff < 0.05) {
               //      problem.AddResidualBlock(CT_ICP::Cost_NavState_PR_Ground::Create(), 
               //                   loss_function, &tmp_y);
               // }
               // if(options_.use_ground_constraint) {
               //      problem.AddResidualBlock(CT_ICP::Cost_NavState_PR_Ground::Create(), 
               //                   loss_function, &tmp_y);
               //      // problem.AddResidualBlock(CT_ICP::Cost_NavState_PR_Ground::Create(), 
               //      //              loss_function, &end_t_y);
               //      std::cout << "end_quat: " << end_quat.x() << " " << end_quat.y() << " " << end_quat.z() 
               //                << " " << end_quat.w() << std::endl;
               //      // problem.AddResidualBlock(CT_ICP::Cost_NavState_PR_Ground::Create(), 
               //      //              loss_function, &end_t.x(), &end_quat.x());

               // }
               
               

               if (surf_num < options_.min_num_residuals)
               {
                    std::cout<<options_.min_num_residuals<<std::endl;
                    std::stringstream ss_out;
                    ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp !" << std::endl;
                    ss_out << "[Optimization] number_of_residuals : " << surf_num << std::endl;
                    std::cout << "ERROR: " << ss_out.str();
               }

               ceres::Solver::Options options;
               options.max_num_iterations = 5;
               options.num_threads = 3;
               options.minimizer_progress_to_stdout = false;
               options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

               // ceres::Solver::Options options;
               // options.linear_solver_type = ceres::DENSE_SCHUR;
               // options.trust_region_strategy_type = ceres::DOGLEG;
               // options.max_num_iterations = 10;
               // options.minimizer_progress_to_stdout = false;
               // options.num_threads = 6;
               // t1 = std::chrono::steady_clock::now();
               ceres::Solver::Summary summary;

               ceres::Solve(options, &problem, &summary);
               // std::cout << "-------------------full report---------------------" << std::endl;
               // std::cout << summary.FullReport() << std::endl;

               // if (normal_thresh_coeff < 0.2 && dist_thresh_coeff < 0.05) 
               if(options_.use_ground_constraint) {
                    end_t[1] = tmp_y;
               }

               // end_t[0] = end_t_x;
               // end_t[1] = end_t_y;
               // end_t[2] = end_t_z;

               // t2 = std::chrono::steady_clock::now();
               // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_RED<<"Solve time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               if (!summary.IsSolutionUsable())
               {
                    std::cout << summary.FullReport() << std::endl;
                    throw std::runtime_error("Error During Optimization");
               }

               begin_quat.normalize();
               end_quat.normalize();

               double diff_trans = 0, diff_rot = 0;
               diff_trans += (current_state->translation_begin - begin_t).norm();
               diff_rot += AngularDistance(current_state->rotation_begin, begin_quat);

               diff_trans += (current_state->translation - end_t).norm();
               diff_rot += AngularDistance(current_state->rotation, end_quat);

               if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE)
               {
                    p_frame->p_state->translation_begin = begin_t;
                    p_frame->p_state->rotation_begin = begin_quat;
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation_begin = begin_t;
                    current_state->translation = end_t;
                    current_state->rotation_begin = begin_quat;
                    current_state->rotation = end_quat;
               }
               if (options_.icpmodel == IcpModel::POINT_TO_PLANE)
               {
                    p_frame->p_state->translation = end_t;
                    p_frame->p_state->rotation = end_quat;

                    current_state->translation = end_t;
                    current_state->rotation = end_quat;
               }

               if (diff_rot < options_.thres_orientation_norm &&
                   diff_trans < options_.thres_translation_norm)
               {
                    if (options_.log_print)
                         std::cout << "Optimization: Finished with N=" << iter << " ICP iterations" << std::endl;
                    break;
               }
          }
          // auto t22 = std::chrono::steady_clock::now();
          // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t22 - t11).count() * 1000;
          // std::cout<<ANSI_COLOR_RED<<"iter time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;

          std::cout << "====================================after p_state->translation.x(): " << p_frame->p_state->translation[0] 
                << " " << p_frame->p_state->translation[1] << " " << p_frame->p_state->translation[2] << std::endl;
               std::cout << p_frame->p_state->rotation.x() << " " << p_frame->p_state->rotation.y() 
                                   << " " << p_frame->p_state->rotation.z() << " " << p_frame->p_state->rotation.w() << std::endl;

          std::vector<point3D>().swap(surf_keypoints);
          if (options_.log_print)
          {
               std::cout << "opt: " << p_frame->p_state->translation_begin.transpose()
                         << ",end: " << p_frame->p_state->translation.transpose() << std::endl;
          }
          //   transpose point before added
          transformKeypoints(p_frame->point_surf);
     }

     double lidarodom::checkLocalizability(std::vector<Eigen::Vector3d> planeNormals)
     {
          //   使用参与计算的法向量分布，进行退化检测，若全是平面场景，则z轴的奇异值会特别小；一般使用小于3/4即可
          Eigen::MatrixXd mat;
          std::cout << "---------------------------------------------panelNormals size: " << planeNormals.size() << std::endl;
          if (planeNormals.size() > 10)
          {
               mat.setZero(planeNormals.size(), 3);
               for (int i = 0; i < planeNormals.size(); i++)
               {
                    mat(i, 0) = planeNormals[i].x();
                    mat(i, 1) = planeNormals[i].y();
                    mat(i, 2) = planeNormals[i].z();
               }
               Eigen::JacobiSVD<Eigen::MatrixXd> svd(planeNormals.size(), 3);
               svd.compute(mat);
               if (svd.singularValues().z() < 3.5)
                    std::cout << ANSI_COLOR_YELLOW << "Low convincing result -> singular values:"
                              << svd.singularValues().x() << ", " << svd.singularValues().y() << ", "
                              << svd.singularValues().z() << ANSI_COLOR_RESET << std::endl;
               return svd.singularValues().z();
          }
          else
          {
               std::cout << ANSI_COLOR_RED << "Too few normal vector received -> " << planeNormals.size() << ANSI_COLOR_RESET << std::endl;
               return -1;
          }
     }

     Neighborhood computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points)
     {
          Neighborhood neighborhood;
          // Compute the normals
          Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
          for (auto &point : points)
          {
               barycenter += point;
          }

          barycenter /= (double)points.size();
          neighborhood.center = barycenter;

          Eigen::Matrix3d covariance_Matrix(Eigen::Matrix3d::Zero());//下面公式是计算的位数据的协防差矩阵
          for (auto &point : points)
          {
               for (int k = 0; k < 3; ++k)
                    for (int l = k; l < 3; ++l)
                         covariance_Matrix(k, l) += (point(k) - barycenter(k)) *
                                                    (point(l) - barycenter(l));
          }
          covariance_Matrix(1, 0) = covariance_Matrix(0, 1);
          covariance_Matrix(2, 0) = covariance_Matrix(0, 2);
          covariance_Matrix(2, 1) = covariance_Matrix(1, 2);
          neighborhood.covariance = covariance_Matrix;
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(covariance_Matrix);//矩阵特征值分解
          Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
          neighborhood.normal = normal;

          double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
          double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
          double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
          neighborhood.a2D = (sigma_2 - sigma_3) / sigma_1;

          if (neighborhood.a2D != neighborhood.a2D)
          {
               throw std::runtime_error("error");
          }

          return neighborhood;
     }
     
     ///  ===================  for search neighbor  ===================================================
     using pair_distance_t = std::tuple<double, Eigen::Vector3d, voxel>;

     struct comparator
     {
          bool operator()(const pair_distance_t &left, const pair_distance_t &right) const
          {
               return std::get<0>(left) < std::get<0>(right);
          }
     };
     using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;
     
     std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
     searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
                                int nb_voxels_visited, double size_voxel_map,
                                int max_num_neighbors, int threshold_voxel_capacity,
                                std::vector<voxel> *voxels)
     {

          if (voxels != nullptr)
               voxels->reserve(max_num_neighbors);

          short kx = static_cast<short>(point[0] / size_voxel_map);
          short ky = static_cast<short>(point[1] / size_voxel_map);
          short kz = static_cast<short>(point[2] / size_voxel_map);

          priority_queue_t priority_queue;

          voxel voxel_temp(kx, ky, kz);
          for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx)
          {
               for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy)
               {
                    for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz)
                    {
                         voxel_temp.x = kxx;
                         voxel_temp.y = kyy;
                         voxel_temp.z = kzz;

                         auto search = map.find(voxel_temp);
                         if (search != map.end())
                         {
                              const auto &voxel_block = search.value();
                              if (voxel_block.NumPoints() < threshold_voxel_capacity)
                                   continue;
                              for (int i(0); i < voxel_block.NumPoints(); ++i)
                              {
                                   auto &neighbor = voxel_block.points[i];
                                   double distance = (neighbor - point).norm();
                                   if (priority_queue.size() == max_num_neighbors)
                                   {
                                        if (distance < std::get<0>(priority_queue.top()))
                                        {
                                             priority_queue.pop();
                                             priority_queue.emplace(distance, neighbor, voxel_temp);
                                        }
                                   }
                                   else
                                        priority_queue.emplace(distance, neighbor, voxel_temp);
                              }
                         }
                    }
               }
          }

          auto size = priority_queue.size();
          std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closest_neighbors(size);
          if (voxels != nullptr)
          {
               voxels->resize(size);
          }
          for (auto i = 0; i < size; ++i)
          {
               closest_neighbors[size - 1 - i] = std::get<1>(priority_queue.top());
               if (voxels != nullptr)
                    (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
               priority_queue.pop();
          }

          return closest_neighbors;
     }

     void addcost(int ki, int numkp, std::vector<point3D> &keypoints, int& num_residuals, std::vector<ceres::CostFunction *> &surf, 
          double lambda_weight, double lambda_neighborhood, const short nb_voxels_visited, const int kThresholdCapacity, 
          const cloudFrame *p_frame, std::vector<Eigen::Vector3d> &normals,  double kMaxPointToPlane, SE3 TIL, voxelHashMap *voxel_map, int max_num_residuals)
     {

          auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
                                               Eigen::Vector3d &location, double &planarity_weight)
          {
               auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
               planarity_weight = std::pow(neighborhood.a2D, 2.0);

               if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
               {
                    neighborhood.normal = -1.0 * neighborhood.normal;
               }
               return neighborhood;
          };
          // std::cout << "------------------------------------keypoints size() :" << keypoints.size() << std::endl;
          for (int k = ki; k < numkp; k+=5)
          { 
               auto &keypoint = keypoints[k];
               auto &raw_point = keypoint.raw_point;
               // auto t1 = std::chrono::steady_clock::now();
               std::vector<voxel> voxels;
               auto vector_neighbors = searchNeighbors(*voxel_map, keypoint.point,
                                                       nb_voxels_visited,
                                                       0.1,
                                                       20,
                                                       kThresholdCapacity,
                                                       nullptr);
               // auto t2 = std::chrono::steady_clock::now();
               // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_BLUE<<"searchNeighbors time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               if (vector_neighbors.size() < 20)
                    continue;

               double weight;

               Eigen::Vector3d location = TIL * raw_point;
               // t1 = std::chrono::steady_clock::now();
               auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);
               // t2 = std::chrono::steady_clock::now();
               // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_BLUE<<"estimatePointNeighborhood time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               weight = lambda_weight * weight + lambda_neighborhood *
                                                     std::exp(-(vector_neighbors[0] -
                                                                keypoint.point)
                                                                   .norm() /
                                                              (kMaxPointToPlane *
                                                               20));

               double point_to_plane_dist;
               std::set<voxel> neighbor_voxels;
               // t1 = std::chrono::steady_clock::now();
               for (int i(0); i < 1; ++i)
               {
                    point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

                    if (point_to_plane_dist < 0.1)
                    {
                         
                         Eigen::Vector3d norm_vector = neighborhood.normal;
                         norm_vector.normalize();
                         mtx.lock();
                         num_residuals++;
                         
                         normals.push_back(norm_vector); //   record normal
                         mtx.unlock();
                         double norm_offset = -norm_vector.dot(vector_neighbors[i]);

//                          switch (options_.icpmodel)
//                          {
//                          case IcpModel::CT_POINT_TO_PLANE:
//                          {
// #ifdef USE_ANALYTICAL_DERIVATE
//                               CT_ICP::CTLidarPlaneNormFactor *cost_function =
//                                   new CT_ICP::CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
// #else
//                               auto *cost_function = CT_ICP::CTPointToPlaneFunctor::Create(vector_neighbors[0],
//                                                                                           keypoints[k].raw_point,
//                                                                                           norm_vector,
//                                                                                           keypoints[k].alpha_time,
//                                                                                           weight);
// #endif
//                               surf.push_back(cost_function);
//                               // problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
//                               break;
//                          }
//                          case IcpModel::POINT_TO_PLANE:
                         {
                              Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
                                                          p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
#ifdef USE_ANALYTICAL_DERIVATE
                              CT_ICP::LidarPlaneNormFactor *cost_function =
                                  new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
#else
                              auto *cost_function = CT_ICP::PointToPlaneFunctor::Create(vector_neighbors[0],
                                                                                        point_end, norm_vector, weight);
#endif                        
                              mtx.lock();
                              surf.push_back(cost_function);
                              mtx.unlock();
                              // problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
                              break;
                         }
                         // }
                    }
               }
               // t2 = std::chrono::steady_clock::now();
               // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
               // std::cout<<ANSI_COLOR_BLUE<<"add_residuals time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
               // k+=10;
               if (num_residuals >= max_num_residuals)
                    break;
          }
               
     }

     void lidarodom::addSurfCostFactor(std::vector<ceres::CostFunction *> &surf, std::vector<Eigen::Vector3d> &normals,
                                       std::vector<point3D> &keypoints, const cloudFrame *p_frame)
     {

          

          double lambda_weight = std::abs(options_.weight_alpha);
          double lambda_neighborhood = std::abs(options_.weight_neighborhood);
          const double kMaxPointToPlane = options_.max_dist_to_plane_icp;
          const double sum = lambda_weight + lambda_neighborhood;

          lambda_weight /= sum;
          lambda_neighborhood /= sum;

          const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames
                                              ? 4
                                              : options_.voxel_neighborhood;

          const int kThresholdCapacity = p_frame->frame_id < options_.init_num_frames
                                             ? 1
                                             : options_.threshold_voxel_occupancy;

          size_t num = keypoints.size();
          int num_residuals = 0;
          // auto t11 = std::chrono::steady_clock::now();
          std::vector<std::thread> threads;
          // threads.reserve(static_cast<size_t>(10));
          for (int i = 0; i < 5; ++i)
          {
               threads.push_back(std::thread(addcost, i, num, std::ref(keypoints), std::ref(num_residuals), std::ref(surf), lambda_weight, lambda_neighborhood, 
               nb_voxels_visited, kThresholdCapacity,p_frame, std::ref(normals), kMaxPointToPlane, TIL_, &voxel_map, options_.max_num_residuals));
               // addcost(i, num, std::ref(keypoints), std::ref(num_residuals), std::ref(surf), lambda_weight, lambda_neighborhood, 
               // nb_voxels_visited, kThresholdCapacity,p_frame, std::ref(normals), kMaxPointToPlane, TIL_, &voxel_map);
          }
          
          for (auto &t : threads) {
               t.join();
          }
        

//           for (int k = 0; k < num; k++)
//           { 
// //                auto estimatePointNeighborhood = [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &vector_neighbors,
// //                                                Eigen::Vector3d &location, double &planarity_weight)
// //                {
// //                     auto neighborhood = computeNeighborhoodDistribution(vector_neighbors);
// //                     planarity_weight = std::pow(neighborhood.a2D, 2.0);

// //                     if (neighborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0)
// //                     {
// //                          neighborhood.normal = -1.0 * neighborhood.normal;
// //                     }
// //                     return neighborhood;
// //                };

// //                auto &keypoint = keypoints[k];
// //                auto &raw_point = keypoint.raw_point;
// //                // auto t1 = std::chrono::steady_clock::now();
// //                std::vector<voxel> voxels;
// //                auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point,
// //                                                        nb_voxels_visited,
// //                                                        0.1,
// //                                                        20,
// //                                                        kThresholdCapacity,
// //                                                        nullptr);
// //                // auto t2 = std::chrono::steady_clock::now();
// //                // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
// //                // std::cout<<ANSI_COLOR_BLUE<<"searchNeighbors time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
// //                if (vector_neighbors.size() < 20)
// //                     continue;

// //                double weight;

// //                Eigen::Vector3d location = TIL_ * raw_point;
// //                // t1 = std::chrono::steady_clock::now();
// //                auto neighborhood = estimatePointNeighborhood(vector_neighbors, location /*raw_point*/, weight);
// //                // t2 = std::chrono::steady_clock::now();
// //                // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
// //                // std::cout<<ANSI_COLOR_BLUE<<"estimatePointNeighborhood time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
// //                weight = lambda_weight * weight + lambda_neighborhood *
// //                                                      std::exp(-(vector_neighbors[0] -
// //                                                                 keypoint.point)
// //                                                                    .norm() /
// //                                                               (kMaxPointToPlane *
// //                                                                20));

// //                double point_to_plane_dist;
// //                std::set<voxel> neighbor_voxels;
// //                // t1 = std::chrono::steady_clock::now();
// //                for (int i(0); i < 1; ++i)
// //                {
// //                     point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neighborhood.normal);

// //                     if (point_to_plane_dist < 0.1)
// //                     {

// //                          num_residuals++;

// //                          Eigen::Vector3d norm_vector = neighborhood.normal;
// //                          norm_vector.normalize();

// //                          normals.push_back(norm_vector); //   record normal

// //                          double norm_offset = -norm_vector.dot(vector_neighbors[i]);

// // //                          switch (options_.icpmodel)
// // //                          {
// // //                          case IcpModel::CT_POINT_TO_PLANE:
// // //                          {
// // // #ifdef USE_ANALYTICAL_DERIVATE
// // //                               CT_ICP::CTLidarPlaneNormFactor *cost_function =
// // //                                   new CT_ICP::CTLidarPlaneNormFactor(keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
// // // #else
// // //                               auto *cost_function = CT_ICP::CTPointToPlaneFunctor::Create(vector_neighbors[0],
// // //                                                                                           keypoints[k].raw_point,
// // //                                                                                           norm_vector,
// // //                                                                                           keypoints[k].alpha_time,
// // //                                                                                           weight);
// // // #endif
// // //                               surf.push_back(cost_function);
// // //                               // problem.AddResidualBlock(cost_function, loss_function, &begin_t.x(), &begin_quat.x(), &end_t.x(), &end_quat.x());
// // //                               break;
// // //                          }
// // //                          case IcpModel::POINT_TO_PLANE:
// //                          {
// //                               Eigen::Vector3d point_end = p_frame->p_state->rotation.inverse() * keypoints[k].point -
// //                                                           p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
// // #ifdef USE_ANALYTICAL_DERIVATE
// //                               CT_ICP::LidarPlaneNormFactor *cost_function =
// //                                   new CT_ICP::LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
// // #else
// //                               auto *cost_function = CT_ICP::PointToPlaneFunctor::Create(vector_neighbors[0],
// //                                                                                         point_end, norm_vector, weight);
// // #endif
// //                               surf.push_back(cost_function);
// //                               // problem.AddResidualBlock(cost_function, loss_function, &end_t.x(), &end_quat.x());
// //                               // break;
// //                          }
// //                          // }
// //                     }
// //                }
//                // t2 = std::chrono::steady_clock::now();
//                // time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
//                // std::cout<<ANSI_COLOR_BLUE<<"add_residuals time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;

//                // auto t1 = std::chrono::steady_clock::now();
//                // std::thread t(addcost, k, std::ref(keypoints), std::ref(num_residuals), std::ref(surf), lambda_weight, lambda_neighborhood, 
//                // nb_voxels_visited, kThresholdCapacity,p_frame, std::ref(normals), kMaxPointToPlane, TIL_, &voxel_map);
//                // t.join();
//                // auto t2 = std::chrono::steady_clock::now();
//                // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
//                // std::cout<<ANSI_COLOR_BLUE<<"add_residuals time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
//                // addcost(k, keypoints, num_residuals, surf, lambda_weight, lambda_neighborhood, 
//                // nb_voxels_visited, kThresholdCapacity,p_frame, normals, kMaxPointToPlane, TIL_, &voxel_map);
//                if (num_residuals >= options_.max_num_residuals)
//                     break;
//           }
          // auto t22 = std::chrono::steady_clock::now();
          // auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t22 - t11).count() * 1000;
          // std::cout<<ANSI_COLOR_BLUE<<"keypointsnum time:"<<time_used<<ANSI_COLOR_RESET<<std::endl;
     }

     

     

     

     void lidarodom::addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point,
                                   const double &intensity, double voxel_size,
                                   int max_num_points_in_voxel, double min_distance_points,
                                   int min_num_points, cloudFrame *p_frame)
     {
          if(!options_.localization_mode)
          {
               short kx = static_cast<short>(point[0] / voxel_size);
               short ky = static_cast<short>(point[1] / voxel_size);
               short kz = static_cast<short>(point[2] / voxel_size);

               voxelHashMap::iterator search = map.find(voxel(kx, ky, kz));

               if (search != map.end())
               {
                    auto &voxel_block = (search.value());

                    if (!voxel_block.IsFull())
                    {
                         double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                         for (int i(0); i < voxel_block.NumPoints(); ++i)
                         {
                              auto &_point = voxel_block.points[i];
                              double sq_dist = (_point - point).squaredNorm();
                              if (sq_dist < sq_dist_min_to_points)
                              {
                                   sq_dist_min_to_points = sq_dist;
                              }
                         }
                         if (sq_dist_min_to_points > (min_distance_points * min_distance_points))
                         {
                              if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points)
                              {
                                   voxel_block.AddPoint(point);
                                   // addPointToPcl(points_world, point, intensity, p_frame);
                              }
                         }
                    }
               }
               else
               {
                    if (min_num_points <= 0)
                    {
                         voxelBlock block(max_num_points_in_voxel);
                         block.AddPoint(point);
                         map[voxel(kx, ky, kz)] = std::move(block);
                    }
               }
          }
          addPointToPcl(points_world, point, intensity, p_frame);
     }

     void lidarodom::addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d &point, const double &intensity, cloudFrame *p_frame)
     {
          pcl::PointXYZI cloudTemp;

          cloudTemp.x = point.x();
          cloudTemp.y = point.y();
          cloudTemp.z = point.z();
          cloudTemp.intensity = intensity;
          // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
          pcl_points->points.push_back(cloudTemp);
     }

     bool lidarodom::savemap(const std::string& filename)
     {
          std::cout<<"save map..."<<std::endl;
          pcl::PointCloud<pcl::PointXYZI>::Ptr mappcl(new pcl::PointCloud<pcl::PointXYZI>());
          for (auto &pair : voxel_map)
          {
               voxelBlock vb = pair.second;
               for(int i=0;i<vb.points.size();i++)
               {
                    pcl::PointXYZI cloudTemp;
                    cloudTemp.x = vb.points[i].x();
                    cloudTemp.y = vb.points[i].y();
                    cloudTemp.z = vb.points[i].z();
                    mappcl->points.push_back(cloudTemp);
               }
          }
          mappcl->is_dense = false;
          mappcl->width = mappcl->size();
          mappcl->height = 1;
          return pcl::io::savePCDFileBinary(filename, *mappcl);
     }

     void lidarodom::updatemap(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points)
     {
          voxel_map.clear();
          double voxel_size = options_.size_voxel_map;
          int max_num_points_in_voxel = options_.max_num_points_in_voxel;
          double min_distance_points = options_.min_distance_points;
          int min_num_points = 0;
          for(int i=0;i < pcl_points->points.size(); i++)
          {
               Eigen::Vector3d point(pcl_points->points[i].x,pcl_points->points[i].y,pcl_points->points[i].z);
               // 计算point 位于哪个voxel中，并在map中搜索对应的voxel
               short kx = static_cast<short>(point[0] / voxel_size);
               short ky = static_cast<short>(point[1] / voxel_size);
               short kz = static_cast<short>(point[2] / voxel_size);
               voxelHashMap::iterator search = voxel_map.find(voxel(kx, ky, kz));

               // 如果找到了对用的voxel
               if (search != voxel_map.end()){

                    // 对应的voxel
                    auto &voxel_block = (search.value());

                    // 如果voxel中点的个数没有达到最大的个数, 即没有被填满
                    if (!voxel_block.IsFull()){

                         // 计算voxel中，离point最近的点
                         double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                         for (int i(0); i < voxel_block.NumPoints(); ++i){
                              auto &_point = voxel_block.points[i];
                              double sq_dist = (_point - point).squaredNorm();
                              if (sq_dist < sq_dist_min_to_points){
                                   sq_dist_min_to_points = sq_dist;
                              }
                         }
                         // 在voxel中，point离最近的点的距离都超过了min_distance_points，才考虑将point添加到voxel中
                         if (sq_dist_min_to_points > (min_distance_points * min_distance_points)){
                              // min_num_points 默认为0
                              if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points){
                                   voxel_block.AddPoint(point);
                                   // addPointToPcl(points_world, point, intensity, p_frame);
                              }
                         }
                    }
               }
               //  point不在map当中
               else{
                    // min_num_points默认为0, 则为map创建一个block, 并将point放入block中
                    if (min_num_points <= 0){
                         voxelBlock block(max_num_points_in_voxel);
                         block.AddPoint(point);
                         voxel_map[voxel(kx, ky, kz)] = std::move(block);
                    }
               }

          }
     }
     
     void lidarodom::loadmap(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points)
     {
          double voxel_size = options_.size_voxel_map;
          int max_num_points_in_voxel = options_.max_num_points_in_voxel;
          double min_distance_points = options_.min_distance_points;
          int min_num_points = 0;
          for(int i=0;i < pcl_points->points.size(); i++)
          {
               Eigen::Vector3d point(pcl_points->points[i].x,pcl_points->points[i].y,pcl_points->points[i].z);
               // 计算point 位于哪个voxel中，并在map中搜索对应的voxel
               short kx = static_cast<short>(point[0] / voxel_size);
               short ky = static_cast<short>(point[1] / voxel_size);
               short kz = static_cast<short>(point[2] / voxel_size);
               voxelHashMap::iterator search = voxel_map.find(voxel(kx, ky, kz));

               // 如果找到了对用的voxel
               if (search != voxel_map.end()){

                    // 对应的voxel
                    auto &voxel_block = (search.value());

                    // 如果voxel中点的个数没有达到最大的个数, 即没有被填满
                    if (!voxel_block.IsFull()){

                         // 计算voxel中，离point最近的点
                         double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
                         for (int i(0); i < voxel_block.NumPoints(); ++i){
                              auto &_point = voxel_block.points[i];
                              double sq_dist = (_point - point).squaredNorm();
                              if (sq_dist < sq_dist_min_to_points){
                                   sq_dist_min_to_points = sq_dist;
                              }
                         }
                         // 在voxel中，point离最近的点的距离都超过了min_distance_points，才考虑将point添加到voxel中
                         if (sq_dist_min_to_points > (min_distance_points * min_distance_points)){
                              // min_num_points 默认为0
                              if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points){
                                   voxel_block.AddPoint(point);
                                   // addPointToPcl(points_world, point, intensity, p_frame);
                              }
                         }
                    }
               }
               //  point不在map当中
               else{
                    // min_num_points默认为0, 则为map创建一个block, 并将point放入block中
                    if (min_num_points <= 0){
                         voxelBlock block(max_num_points_in_voxel);
                         block.AddPoint(point);
                         voxel_map[voxel(kx, ky, kz)] = std::move(block);
                    }
               }

          }
     }

     void lidarodom::map_incremental(cloudFrame *p_frame, int min_num_points)
     {
          //   only surf
          for (const auto &point : p_frame->point_surf)
          {
               addPointToMap(voxel_map, point.point, point.intensity,
                             options_.size_voxel_map, options_.max_num_points_in_voxel,
                             options_.min_distance_points, min_num_points, p_frame);
               // addPointToPcl(points_distort, point.raw_point, point.intensity, p_frame);
          }
          std::string laser_topic = "laser";

          if(!options_.localization_mode)
          {
               for (auto point : p_frame->point_surf){
                    // point是世界坐标系下的点
                    // double range = point.raw_point.x() * point.raw_point.x() + point.raw_point.y() * point.raw_point.y() +
                    //             point.raw_point.z() * point.raw_point.z();
                    // if(range>8) continue;
                    Eigen::Vector3d tfpoint = TIL_ * point.raw_point;
                    pcl::PointXYZI cloudTemp;

                    cloudTemp.x = tfpoint.x();
                    cloudTemp.y = tfpoint.y();
                    cloudTemp.z = tfpoint.z();
                    cloudTemp.intensity = point.intensity;
                    points_distort->points.push_back(cloudTemp);
               }
               pub_cloud_to_ros(laser_topic, points_distort, p_frame->time_frame_end, 2);
          }
   
          
          pub_cloud_to_ros(laser_topic, points_world, p_frame->time_frame_end,1);
          // pcl::PointCloud<pcl::PointXYZI>::Ptr pcall(new pcl::PointCloud<pcl::PointXYZI>());
          // pcall = graph_->get_pointcloud();
          // pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
          // pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
          // voxel_grid.setLeafSize(0.2, 0.2, 0.2);
          // voxel_grid.setInputCloud(pcall);
          // voxel_grid.filter(*downsampled);
          // pub_cloud_to_ros(laser_topic, downsampled, p_frame->time_frame_end); 

     }

     void lidarodom::lasermap_fov_segment()
     {
          //   use predict pose here
          Eigen::Vector3d location = current_state->translation;
          std::vector<voxel> voxels_to_erase;
          for (auto &pair : voxel_map)
          {
               Eigen::Vector3d pt = pair.second.points[0];
               if ((pt - location).squaredNorm() > (options_.max_distance * options_.max_distance))
               {
                    voxels_to_erase.push_back(pair.first);
               }
          }
          for (auto &vox : voxels_to_erase)
               voxel_map.erase(vox);
          std::vector<voxel>().swap(voxels_to_erase);
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                       double timestamp_begin, double timestamp_end){

          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2){
               for (auto &point_temp : frame_surf){
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }


          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          // p_frame->point_coeff = 

          return p_frame;
     }

     cloudFrame *lidarodom::buildFrame(std::vector<point3D> &const_surf, state *cur_state,
                                       double timestamp_begin, double timestamp_end, 
                                       Eigen::VectorXf lidar_coeff){

          std::vector<point3D> frame_surf(const_surf);
          if (index_frame < 2){
               for (auto &point_temp : frame_surf){
                    point_temp.alpha_time = 1.0; //  alpha reset 0
               }
          }


          if (options_.motion_compensation == CONSTANT_VELOCITY)
               Undistort(frame_surf);

          for (auto &point_temp : frame_surf)
               transformPoint(options_.motion_compensation, point_temp, cur_state->rotation_begin,
                              cur_state->rotation, cur_state->translation_begin, cur_state->translation,
                              R_imu_lidar, t_imu_lidar);

          cloudFrame *p_frame = new cloudFrame(frame_surf, const_surf, cur_state);

          p_frame->time_frame_begin = timestamp_begin;
          p_frame->time_frame_end = timestamp_end;

          p_frame->dt_offset = 0;

          p_frame->frame_id = index_frame;

          p_frame->point_coeff = lidar_coeff;

          return p_frame;
     }

     void lidarodom::stateInitialization()
     {
          if (index_frame < 2) //   only first frame
          {
               current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
               current_state->translation_begin = imu_states_.front().p_;
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
          }
          else
          {
               //   use last pose
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
               // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
               // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
               //   use imu predict
               if(updategraph)
               {
                     current_state->rotation = qn;
                     current_state->translation = tn;
               }
               else
               {
                    current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
                    current_state->translation = imu_states_.back().p_;
               }
               // current_state->rotation = q_next_end;
               // current_state->translation = t_next_end;
          }
          imu_states_.pop_back();
     }

     void lidarodom::relocInitialization(std::vector<point3D>& cloud)
     {
          // if (index_frame < 2) //   only first frame
          if (!init_success)
          {
               std::cout<<"reloc_mode  :   scan context   "<<endl;
               pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>());

               for (int i = 0; i < cloud.size(); i++) {

                    Eigen::Vector3d point_new = TIL_ * cloud[i].raw_point;

                    pcl::PointXYZI pv;
                    pv.x = point_new(0);
                    pv.y = point_new(1);
                    pv.z = point_new(2);
                    pv.intensity = cloud[i].intensity;
                    cur_cloud->points.push_back(pv);
               }

               std::string pcd_reloc_path = output_path+"/reloc_query.pcd";
               pcl::io::savePCDFileBinary(pcd_reloc_path, *cur_cloud);

               std::cout<<"start search reloc db:      "<<endl;

               scManager->makeAndSaveScancontextAndKeys(*cur_cloud);
               auto detectResult = scManager->detectDatabaseLoopClosureIDAndPose();
               std::cout<<"reloc id : "<<detectResult.first<<std::endl;

               if(detectResult.first==-1)
               {
                    reloc_success_num = 0;
                    return;
               }

               Eigen::Vector3d translation_db(detectResult.second[0], detectResult.second[1], detectResult.second[2]);
               Eigen::Quaterniond rotation_db(detectResult.second[6], detectResult.second[3], detectResult.second[4], detectResult.second[5]);
               rotation_db.normalize();

               Eigen::Isometry3d pose_db = Eigen::Isometry3d::Identity();
               pose_db.rotate(rotation_db);
               pose_db.pretranslate(translation_db);


               pcl::PointCloud<pcl::PointXYZI> db = scManager->cloud_database_[detectResult.first];
               pcl::PointCloud<pcl::PointXYZI>::Ptr database_cloud(new pcl::PointCloud<pcl::PointXYZI>());  
               database_cloud = db.makeShared();

               std::string pcd_reloc_db_path = output_path+"/reloc_db.pcd";
               pcl::io::savePCDFileBinary(pcd_reloc_db_path, *database_cloud);


               // ICP Settings
               pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
               icp.setMaxCorrespondenceDistance(15); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
               icp.setMaximumIterations(10000);
               icp.setTransformationEpsilon(1e-6);
               icp.setEuclideanFitnessEpsilon(1e-6);
               icp.setRANSACIterations(0);


               icp.setInputSource(cur_cloud);
               icp.setInputTarget(database_cloud);
               pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
               icp.align(*unused_result);


               Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
               relative.matrix() = icp.getFinalTransformation().cast<double>();
               Eigen::Isometry3d pose_reloc_icp = Eigen::Isometry3d::Identity();
               pose_reloc_icp = pose_db * relative;

       
               std::cout<<"database init pose "<<endl;
               std::cout<<pose_db.matrix()<<std::endl;

               std::cout<<"realtive pose "<<endl;
               std::cout<<relative.matrix()<<std::endl;


               std::cout<<"icp pose "<<endl;
               std::cout<<pose_reloc_icp.matrix()<<std::endl;

               std::cout<<"icp  FitnessScore"<<endl;
               std::cout<<icp.getFitnessScore()<<std::endl;


               if (icp.getFitnessScore() > options_.icp_threshold) {

                    // index_frame = 0;
                    reloc_success_num = 0;
                    return;
               }
               reloc_success_num = reloc_success_num + 1;
               // if(reloc_success_num > 2)
               init_success = 1;

               std::cout << "icp  success  " << std::endl;


               Eigen::Vector3d trans_icp = pose_reloc_icp.translation();
               Eigen::Quaterniond quaternion_icp =Eigen::Quaterniond(pose_reloc_icp.rotation());
               Sophus::SO3<double> rotation_icp = Sophus::SO3<double>(quaternion_icp.toRotationMatrix());

        
               eskf_.SetRelocX(rotation_icp, trans_icp);


               current_state->rotation_begin = quaternion_icp;
               current_state->translation_begin = trans_icp;
               current_state->rotation = quaternion_icp;
               current_state->translation = trans_icp;
          }
          else
          {
               //   use last pose
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
               // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
               // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
               //   use imu predict
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
               // current_state->rotation = q_next_end;
               // current_state->translation = t_next_end;
          }
     }

     void lidarodom::relocInitialization(std::vector<point3D>& cloud, cv::Mat& img_mat)
     {
          // if (index_frame < 2) //   only first frame
          if (!init_success)
          {
               if(img_mat.empty())
               {
                    return;
               }
               std::cout<<"reloc_mode  :   hf_net   "<<endl;
               pcl::PointCloud<pcl::PointXYZI>::Ptr cur_cloud(new pcl::PointCloud<pcl::PointXYZI>());

               for (int i = 0; i < cloud.size(); i++) {

                    Eigen::Vector3d point_new = TIL_ * cloud[i].raw_point;

                    pcl::PointXYZI pv;
                    pv.x = point_new(0);
                    pv.y = point_new(1);
                    pv.z = point_new(2);
                    pv.intensity = cloud[i].intensity;
                    cur_cloud->points.push_back(pv);
               }

               std::string pcd_reloc_path = output_path+"/reloc_hfnet_query.pcd";
               pcl::io::savePCDFileBinary(pcd_reloc_path, *cur_cloud);

               // std::string pcd_reloc_path = "/shared_dir/dtof-loc-scancontext-hfnet/reloc_query.pcd";
               // pcl::io::savePCDFileBinary(pcd_reloc_path, *cur_cloud);

               // std::cout<<"start search reloc db:      "<<endl;

               // scManager->makeAndSaveScancontextAndKeys(*cur_cloud);
               // auto detectResult = scManager->detectDatabaseLoopClosureIDAndPose();
               // std::cout<<"reloc id : "<<detectResult.first<<std::endl;


               // Eigen::Vector3d translation_db(detectResult.second[0], detectResult.second[1], detectResult.second[2]);
               // Eigen::Quaterniond rotation_db(detectResult.second[6], detectResult.second[3], detectResult.second[4], detectResult.second[5]);
               // rotation_db.normalize();

               // Eigen::Isometry3d pose_db = Eigen::Isometry3d::Identity();
               // pose_db.rotate(rotation_db);
               // pose_db.pretranslate(translation_db);


               // pcl::PointCloud<pcl::PointXYZI> db = scManager->cloud_database_[detectResult.first];
               // pcl::PointCloud<pcl::PointXYZI>::Ptr database_cloud(new pcl::PointCloud<pcl::PointXYZI>());  
               // database_cloud = db.makeShared();

               // std::string pcd_reloc_db_path = "/shared_dir/dtof-loc-scancontext-hfnet/reloc_sc_db.pcd";
               // pcl::io::savePCDFileBinary(pcd_reloc_db_path, *database_cloud);


               // // ICP Settings
               // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
               // icp.setMaxCorrespondenceDistance(15); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
               // icp.setMaximumIterations(10000);
               // icp.setTransformationEpsilon(1e-6);
               // icp.setEuclideanFitnessEpsilon(1e-6);
               // icp.setRANSACIterations(0);


               // icp.setInputSource(cur_cloud);
               // icp.setInputTarget(database_cloud);
               // pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
               // icp.align(*unused_result);


               // Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
               // relative.matrix() = icp.getFinalTransformation().cast<double>();
               // Eigen::Isometry3d pose_reloc_icp = Eigen::Isometry3d::Identity();
               // pose_reloc_icp = pose_db * relative;

       
               // std::cout<<"sc database init pose "<<endl;
               // std::cout<<pose_db.matrix()<<std::endl;

               // std::cout<<"sc realtive pose "<<endl;
               // std::cout<<relative.matrix()<<std::endl;


               // std::cout<<"sc icp pose "<<endl;
               // std::cout<<pose_reloc_icp.matrix()<<std::endl;

               // std::cout<<"sc icp  FitnessScore"<<endl;
               // std::cout<<icp.getFitnessScore()<<std::endl;



          /************************************hfnet*************************************************/

          // std::vector<std::pair<InteractiveKeyFrame::Ptr, Eigen::VectorXf>> db;
          // Eigen::VectorXf query = inference_img(keyframe->img_);

          // cv::Mat img_mat = cv::imread("/shared_dir/1.png");
               #if defined X3
               Eigen::VectorXf query = inference_x3(img_mat, model_path);
               #else
               Eigen::VectorXf query = inference_img(img_mat, model_path);
               #endif
              

               int index_hfnet = -1;
               double min_result_hfnet = 100.0; 
               
               for (int i = 0; i < hfNetManager->descriptor_database.size(); i++) {
                    Eigen::VectorXf descriptor_tmp = hfNetManager->descriptor_database[i];
                    double result = global_match(query, descriptor_tmp);
                    if (result < min_result_hfnet) {
                         min_result_hfnet = result;
                         index_hfnet = i;
                    }
               }

               pcl::PointCloud<pcl::PointXYZI> hfnet_db = hfNetManager->cloud_database[index_hfnet];
               pcl::PointCloud<pcl::PointXYZI>::Ptr hfnet_database_cloud(new pcl::PointCloud<pcl::PointXYZI>());  
               hfnet_database_cloud = hfnet_db.makeShared();
               std::string pcd_hfnet_reloc_db_path = output_path+"/reloc_hfnet_db.pcd";
               pcl::io::savePCDFileBinary(pcd_hfnet_reloc_db_path, hfnet_db);

               std::string img_hfnet_reloc_db_path = output_path+"/img_db.png";
               std::string img_hfnet_reloc_query_path = output_path+"/img_query.png";

               cv::imwrite(img_hfnet_reloc_db_path, hfNetManager->image_database[index_hfnet]);
               cv::imwrite(img_hfnet_reloc_query_path, img_mat);



               Eigen::Vector3d translation_hfnet_db(hfNetManager->pose_database[index_hfnet][0], hfNetManager->pose_database[index_hfnet][1], hfNetManager->pose_database[index_hfnet][2]);
               Eigen::Quaterniond rotation_hfnet_db(hfNetManager->pose_database[index_hfnet][6], hfNetManager->pose_database[index_hfnet][3], hfNetManager->pose_database[index_hfnet][4], hfNetManager->pose_database[index_hfnet][5]);
               rotation_hfnet_db.normalize();

               Eigen::Isometry3d pose_db_hf = Eigen::Isometry3d::Identity();
               pose_db_hf.rotate(rotation_hfnet_db);
               pose_db_hf.pretranslate(translation_hfnet_db);


               // ICP Settings
               pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp_hf;
               icp_hf.setMaxCorrespondenceDistance(15); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
               icp_hf.setMaximumIterations(10000);
               icp_hf.setTransformationEpsilon(1e-6);
               icp_hf.setEuclideanFitnessEpsilon(1e-6);
               icp_hf.setRANSACIterations(0);


               icp_hf.setInputSource(cur_cloud);
               icp_hf.setInputTarget(hfnet_database_cloud);
               pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result_hf(new pcl::PointCloud<pcl::PointXYZI>());
               icp_hf.align(*unused_result_hf);


               Eigen::Isometry3d relative_hf = Eigen::Isometry3d::Identity();
               relative_hf.matrix() = icp_hf.getFinalTransformation().cast<double>();
               Eigen::Isometry3d pose_reloc_icp_hf = Eigen::Isometry3d::Identity();
               pose_reloc_icp_hf = pose_db_hf * relative_hf;




               std::cout<<"hfnet database init pose "<<endl;
               std::cout<<pose_db_hf.matrix()<<std::endl;

               std::cout<<"hfnet realtive pose "<<endl;
               std::cout<<relative_hf.matrix()<<std::endl;


               std::cout<<"hfnet icp pose "<<endl;
               std::cout<<pose_reloc_icp_hf.matrix()<<std::endl;

               std::cout<<"hfnet icp  FitnessScore"<<endl;
               std::cout<<icp_hf.getFitnessScore()<<std::endl;

               if (icp_hf.getFitnessScore() > options_.icp_threshold) {

                    // index_frame = 0;
                    return;
               }
               init_success = 1;

               std::cout << "hfnet icp  success  " << std::endl;


               Eigen::Vector3d trans_icp_hf = pose_reloc_icp_hf.translation();
               Eigen::Quaterniond quaternion_icp_hf =Eigen::Quaterniond(pose_reloc_icp_hf.rotation());
               Sophus::SO3<double> rotation_icp_hf = Sophus::SO3<double>(quaternion_icp_hf.toRotationMatrix());
    
               eskf_.SetRelocX(rotation_icp_hf, trans_icp_hf);


               current_state->rotation_begin = quaternion_icp_hf;
               current_state->translation_begin = trans_icp_hf;
               current_state->rotation = quaternion_icp_hf;
               current_state->translation = trans_icp_hf;

               // current_state->rotation_begin = rotation_hfnet_db;
               // current_state->translation_begin = translation_hfnet_db;
               // current_state->rotation = rotation_hfnet_db;
               // current_state->translation = translation_hfnet_db;



          /************************************hfnet*************************************************/

               // if (icp.getFitnessScore() > 0.1) {

               //      index_frame = 0;
               //      return;
               // }

               // std::cout << "icp  success  " << std::endl;


               // Eigen::Vector3d trans_icp = pose_reloc_icp.translation();
               // Eigen::Quaterniond quaternion_icp =Eigen::Quaterniond(pose_reloc_icp.rotation());
               // Sophus::SO3<double> rotation_icp = Sophus::SO3<double>(quaternion_icp.toRotationMatrix());
    
               // eskf_.SetRelocX(rotation_icp, trans_icp);


               // current_state->rotation_begin = quaternion_icp;
               // current_state->translation_begin = trans_icp;
               // current_state->rotation = quaternion_icp;
               // current_state->translation = trans_icp;

          }
          else
          {
               //   use last pose
               current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
               current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
               // current_state->rotation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->rotation;
               // current_state->translation_begin = all_cloud_frame[all_cloud_frame.size() - 1]->p_state->translation;
               //   use imu predict
               current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
               current_state->translation = imu_states_.back().p_;
               // current_state->rotation = q_next_end;
               // current_state->translation = t_next_end;
          }
     }

     // void lidarodom::stateInitialization(){

     //      if (index_frame <= 2){ //   front 2 frame
     //           current_state->rotation_begin = Eigen::Quaterniond(1, 0, 0, 0);
     //           current_state->translation_begin = Eigen::Vector3d(0, 0, 0);
     //           current_state->rotation = Eigen::Quaterniond(1, 0, 0, 0);
     //           current_state->translation = Eigen::Vector3d(0, 0, 0);
     //      }
     //      else{
     //           //   use last pose
     //           current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
     //           current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;

     //           current_state->rotation = (all_state_frame[all_state_frame.size() - 1]->rotation) *
     //                                     (all_state_frame[all_state_frame.size() - 2]->rotation).inverse() *
     //                                     (all_state_frame[all_state_frame.size() - 1]->rotation);
     //           current_state->translation = all_state_frame[all_state_frame.size() - 1]->translation +
     //                                        (all_state_frame[all_state_frame.size() - 1]->rotation) *
     //                                            (all_state_frame[all_state_frame.size() - 2]->rotation).inverse() *
     //                                            (all_state_frame[all_state_frame.size() - 1]->translation -
     //                                             all_state_frame[all_state_frame.size() - 2]->translation);
     //      }
     // }

     std::vector<MeasureGroup> lidarodom::getMeasureMents()
     {
          std::vector<MeasureGroup> measurements;
          while (true)
          {
               if (imu_buffer_.empty())
                    return measurements;

               if (odom_buffer_.empty())
                    return measurements;

               if (lidar_buffer_.empty())
                    return measurements;
               
               if (img_buffer_.empty())
                    return measurements;

               if (imu_buffer_.back()->timestamp_ - time_curr < delay_time_)
                    return measurements;

               MeasureGroup meas;

               meas.lidar_ = lidar_buffer_.front();
               meas.lidar_begin_time_ = time_buffer_.front().first;
               meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
               meas.lidar_begin_rostime_ = ros_time_buffer_.front();
               meas.lidar_coeff_ = lidar_coeff_buffer_.front();
               lidar_buffer_.pop_front();
               time_buffer_.pop_front();
               ros_time_buffer_.pop_front();
               lidar_coeff_buffer_.pop_front();

               time_curr = meas.lidar_end_time_;

               double img_time = img_time_buffer_.front();

               while ((!img_buffer_.empty()) && (img_time < meas.lidar_end_time_)) {
                    img_time = img_time_buffer_.front();
                    if (img_time > meas.lidar_end_time_) {
                         break;
                    }
                    meas.img_ = img_buffer_.front();
                    img_time_buffer_.pop_front();
                    img_buffer_.pop_front();
               }

               // meas.img_ = img_buffer_.front();
               // img_time_buffer_.pop_front();
               // img_buffer_.pop_front();


               double imu_time = imu_buffer_.front()->timestamp_;
               meas.imu_.clear();
               while ((!imu_buffer_.empty()) && (imu_time < meas.lidar_end_time_))
               {
                    imu_time = imu_buffer_.front()->timestamp_;
                    if (imu_time > meas.lidar_end_time_)
                    {
                         break;
                    }
                    meas.imu_.push_back(imu_buffer_.front());
                    imu_buffer_.pop_front();
               }

               if (!imu_buffer_.empty())
                    meas.imu_.push_back(imu_buffer_.front()); //   added for Interp
               
               double odom_time = odom_buffer_.front()->timestamp_;
               // std::cout<<"as"<<std::setprecision(19)<<odom_time<<std::endl;
               // std::cout<<"bs"<<std::setprecision(19)<<meas.lidar_end_time_<<std::endl;
               while ((!odom_buffer_.empty()) && (odom_time < meas.lidar_end_time_))
               {
                    odom_time = odom_buffer_.front()->timestamp_;
                    if (odom_time > meas.lidar_end_time_)
                    {
                         break;
                    }
                    meas.odom_ = odom_buffer_.front();
                    odom_buffer_.pop_front();
               }

               measurements.push_back(meas);
          }
     }

     // std::vector<MeasureGroup> lidarodom::getMeasureMents(){
     //      std::vector<MeasureGroup> measurements;
     //      while (true){
     //           if (lidar_buffer_.empty())
     //                return measurements;

     //           MeasureGroup meas;

     //           // lidar数据
     //           meas.lidar_ = lidar_buffer_.front();
     //           // lidar 起始时间
     //           meas.lidar_begin_time_ = time_buffer_.front().first;
     //           // lidar 终止时间
     //           meas.lidar_end_time_ = meas.lidar_begin_time_ + time_buffer_.front().second;
     //           lidar_buffer_.pop_front();
     //           time_buffer_.pop_front();
     //           // 将meas放入measurements
     //           measurements.push_back(meas);
     //      }
     // }

     void lidarodom::Predict(){
          
          imu_states_.emplace_back(eskf_.GetNominalState());//轮速更新、icp更新后的值
          
          /// 对IMU状态进行预测
          double time_current = measures_.lidar_end_time_;
          Vec3d last_gyr, last_acc;
          // std::cout<<"sss:"<<measures_.imu_.size()<<std::endl;
          // int i=0;
          for (auto &imu : measures_.imu_){
               double time_imu = imu->timestamp_;
               // std::cout<<"sss"<<std::setprecision(19)<<imu->timestamp_<<","<<std::setprecision(19)<<time_current<<std::endl;
               if (imu->timestamp_ <= time_current){
                    if (last_imu_ == nullptr)
                         last_imu_ = imu;
                    eskf_.Predict(*imu);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu;
               }
               else{
                    double dt_1 = time_imu - time_current;
                    double dt_2 = time_current - last_imu_->timestamp_;
                    double w1 = dt_1 / (dt_1 + dt_2);
                    double w2 = dt_2 / (dt_1 + dt_2);
                    Eigen::Vector3d acc_temp = w1 * last_imu_->acce_ + w2 * imu->acce_;
                    Eigen::Vector3d gyr_temp = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
                    IMUPtr imu_temp = std::make_shared<zjloc::IMU>(time_current, gyr_temp, acc_temp);
                    eskf_.Predict(*imu_temp);
                    imu_states_.emplace_back(eskf_.GetNominalState());
                    last_imu_ = imu_temp;
               }
               // i++;
          }
          // std::cout<<"eeeeeeeeeeee"<<std::endl;
     }

     void lidarodom::Undistort(std::vector<point3D> &points)
     {
          // auto &cloud = measures_.lidar_;
          // auto imu_state = eskf_.GetNominalState(); // 最后时刻的状态
          auto imu_state = imu_states_.back();
          // std::cout << __FUNCTION__ << ", " << imu_state.timestamp_ << std::endl;
          SE3 T_end = SE3(imu_state.R_, imu_state.p_);

          /// 将所有点转到最后时刻状态上
          for (auto &pt : points)
          {
               SE3 Ti = T_end;
               NavStated match;

               // 根据pt.time查找时间，pt.time是该点打到的时间与雷达开始时间之差，单位为毫秒
               math::PoseInterp<NavStated>(
                   pt.timestamp, imu_states_, [](const NavStated &s)
                   { return s.timestamp_; },
                   [](const NavStated &s)
                   { return s.GetSE3(); },
                   Ti, match);

               pt.raw_point = TIL_.inverse() * T_end.inverse() * Ti * TIL_ * pt.raw_point;
          }
     }

     void lidarodom::TryInitIMU()
     {
          for (auto imu : measures_.imu_)
          {
               imu_init_.AddIMU(*imu);
          }

          if (imu_init_.InitSuccess())
          {
               // 读取初始零偏，设置ESKF
               zjloc::ESKFD::Options options;
               // 噪声由初始化器估计
               // options.gyro_var_ = sqrt(imu_init_.GetCovGyro()[0]);
               // options.acce_var_ = sqrt(imu_init_.GetCovAcce()[0]);
               // options.update_bias_acce_ = false;
               // options.update_bias_gyro_ = false;
 
               options.imu_dt_ = imudt;
               eskf_.SetInitialConditions(options, imu_init_.GetInitBg(), imu_init_.GetInitBa(), imu_init_.GetGravity());
               imu_need_init_ = false;

               std::cout << ANSI_COLOR_GREEN_BOLD << "IMU初始化成功" << ANSI_COLOR_RESET << std::endl;
          }
     }

}
