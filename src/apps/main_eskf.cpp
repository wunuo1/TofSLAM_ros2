// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>
#include <functional>

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include "common/utility.h"
#include "preprocess/cloud_convert/cloud_convert.h"
#include "liw/lio/lidarodom.h"
#include <loop/loop_close.h>
#include "liw/lio_utils.h"

// # define _GLIBCXX_USE_CXX11_ABI 0

nav_msgs::msg::Path laserOdoPath;

zjloc::lidarodom *lio;
zjloc::CloudConvert *convert;


DEFINE_string(config_yaml, "./config/mapping.yaml", "配置文件");
#define DEBUG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "log/" + name))

/*
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg){

    std::vector<point3D> cloud_out;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>);

    zjloc::common::Timer::Evaluate([&]()
                                   { convert->Process(msg, cloud_out); },
                                   "laser convert");

    // for(int i = 0; i < cloud_out.size(); i++) {
    //     // double point_x = cloud_out[i].point.x();
    //     // double point_y = cloud_out[i].point.y();
    //     // double point_z = cloud_out[i].point.z();
    //     pcl::PointXYZI point;
    //     point.x = cloud_out[i].point.x();
    //     point.y = cloud_out[i].point.y();
    //     point.z = cloud_out[i].point.z();
    //     point.intensity = cloud_out[i].intensity;

    //     cloud_input->push_back(point);
    // }

    zjloc::common::Timer::Evaluate([&]()
                                   { 
        double sample_size = lio->getIndex() < 20 ? 0.01 : 0.01;
        // double sample_size = 0.01;
        std::mt19937_64 g;
        std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        subSampleFrame(cloud_out, sample_size);
        std::shuffle(cloud_out.begin(), cloud_out.end(), g); },
                                   "laser ds");

    // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan()), msg->header.stamp);
    lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan()), msg->header.stamp, cloud_input);
}*/


void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

    sensor_msgs::msg::PointCloud2::Ptr cloud(new sensor_msgs::msg::PointCloud2(*msg));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud_input);

    static int c = 0;
    // if (c % 2 == 0 && use_velodyne)
    {
        std::vector<point3D> cloud_out;
        zjloc::common::Timer::Evaluate([&]()
                                       { convert->Process(msg, cloud_out); },
                                       "laser convert");

        zjloc::common::Timer::Evaluate([&]() { // boost::mt19937_64 g;
            double sample_size = lio->getIndex() < 20 ? 0.2 : 0.2;
            // double sample_size = 0.05;
            std::mt19937_64 g;
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            subSampleFrame(cloud_out, sample_size);
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
                                       "laser ds");

        // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9 - convert->getTimeSpan(), convert->getTimeSpan())); //  FIXME: for staircase dataset(header timestamp is the frame end)
        // lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan())); //  normal
        lio->pushData(cloud_out, std::make_pair(msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9, convert->getTimeSpan()), msg->header.stamp, cloud_input);
    }
    c++;
}

void imgHandler(const sensor_msgs::msg::Image::SharedPtr msg) {
    try
    {
      cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      cv::Mat img = cv_ptr_compressed->image;
      lio->pushData(img, msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9, msg->header.stamp);
    //   cv::imshow("tmp", img);
    //   cv::waitKey(10);
      LOG(INFO) << "received image data";
    }
    catch (cv_bridge::Exception& e)
    {
        LOG(ERROR) << "Could not convert from 'senseor_msgs::CompressedImage' to 'bgr8'.";
    }

}

void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    sensor_msgs::msg::Imu::Ptr msg_temp(new sensor_msgs::msg::Imu(*msg));
    IMUPtr imu = std::make_shared<zjloc::IMU>(
        msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
        Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z));
    lio->pushData(imu);
}

void odomHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    nav_msgs::msg::Odometry::Ptr msg_temp(new nav_msgs::msg::Odometry(*msg));
    OdomposPtr odom = std::make_shared<zjloc::Odompos>(
        msg->header.stamp.sec+ msg->header.stamp.nanosec * 1e-9,
        Vec3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
        Vec4d(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z),
        msg->twist.twist.linear.x,
        msg->twist.twist.linear.y);
    lio->pushData(odom);
}


void updateStatus(const std_msgs::msg::Int32::SharedPtr msg)
{
    int type = msg->data;
    if (type == 1)
    {
    }
    else if (type == 2)
    {
    }
    else if (type == 3)
        ;
    else if (type == 4)
        ;
    else
        ;
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("eskf");

    std::string config_file = std::string(ROOT_DIR) + "config/mapping.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    convert = new zjloc::CloudConvert;
    convert->LoadFromYAML(config_file);
    std::cout << ANSI_COLOR_GREEN_BOLD << "init successful" << ANSI_COLOR_RESET << std::endl;

    auto yaml = YAML::LoadFile(config_file);
    std::string laser_topic = yaml["common"]["lid_topic"].as<std::string>();
    std::string imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    std::string odom_topic = yaml["common"]["odom_topic"].as<std::string>();
    std::string img_topic = yaml["common"]["img_topic"].as<std::string>();
    std::string output_path = yaml["common"]["output_path"].as<std::string>();
    std::string model_path = yaml["common"]["model_path"].as<std::string>();
    bool localization_mode = yaml["common"]["Localization_mode"].as<bool>();

    // std::vector<std::pair<zjloc::InteractiveKeyFrame::Ptr, Eigen::VectorXf>> db;

    lio = new zjloc::lidarodom();
    if (!lio->init(config_file))
    {
        return -1;
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan = 
        node->create_publisher<sensor_msgs::msg::PointCloud2>("scan", 10);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2 = 
        node->create_publisher<sensor_msgs::msg::PointCloud2>("scan_distort", 10);
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map = 
        node->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud", 10);

    auto cloud_pub_func = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time, int mode)>(
        [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time, int mode)
        {
            sensor_msgs::msg::PointCloud2::Ptr cloud_ptr_output(new sensor_msgs::msg::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_ptr_output);

            cloud_ptr_output->header.stamp = rclcpp::Time(time * 1e9);
            cloud_ptr_output->header.frame_id = "map";
            if (topic_name == "laser")
                if(mode==1)
                {
                    pub_scan->publish(*cloud_ptr_output);
                }
                else
                {
                    if(mode==2)
                    {
                        pub_scan2->publish(*cloud_ptr_output);
                    }
                    else
                    {
                        if(mode==3)
                        {
                            pub_map->publish(*cloud_ptr_output);
                        }
                    }
                }
            else
                ; // publisher_.publish(*cloud_ptr_output);
            return true;
        }

    );

    // rclcpp::Publisher pub_scan2 = nh.advertise<sensor_msgs::msg::PointCloud2>("scan_distort", 10);
    // auto cloud_pub_func2 = std::function<bool(std::string & topic_name, zjloc::CloudPtr & cloud, double time, bool distort)>(
    //     [&](std::string &topic_name, zjloc::CloudPtr &cloud, double time, bool distort)
    //     {
    //         sensor_msgs::msg::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::msg::PointCloud2());
    //         pcl::toROSMsg(*cloud, *cloud_ptr_output);

    //         cloud_ptr_output->header.stamp = ros::Time().fromSec(time);
    //         cloud_ptr_output->header.frame_id = "map";
    //         if (topic_name == "laser")
    //             pub_scan2->publish(*cloud_ptr_output);
    //         else
    //             ; // publisher_.publish(*cloud_ptr_output);
    //         return true;
    //     }

    // );
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry = 
        node->create_publisher<nav_msgs::msg::Odometry>("/icp_odom_vis", 100);
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserOdometryPath = 
        node->create_publisher<nav_msgs::msg::Path>("/icp_odometry_path", 5);
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryinrobot = 
        node->create_publisher<nav_msgs::msg::Odometry>("/icp_odom", 100);


    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, SE3 & poseinrobot, double stamp)>(
        [&](std::string &topic_name, SE3 &pose, SE3 & poseinrobot, double stamp)
        {
            //static tf2_ros::TransformBroadcaster br;
            //static tf2_ros::TransformBroadcaster br = tf2_ros::TransformBroadcaster(this);
            tf2::Transform transform;
            Eigen::Quaterniond q_current(pose.so3().matrix());
            Eigen::Quaterniond q_currentrobot(poseinrobot.so3().matrix());
            transform.setOrigin(tf2::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf2::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            if (topic_name == "laser")
            {
                /*
                geometry_msgs::msg::TransformStamped tmp_tf_stamped;
                tmp_tf_stamped.header.frame_id = "map";
                tmp_tf_stamped.header.stamp = rclcpp::Time(stamp * 1e9);
                tmp_tf_stamped.child_frame_id = "base_link";
                tf2::impl::Converter<false, true>::convert(transform, tmp_tf_stamped.transform);
                br.sendTransform(tmp_tf_stamped);
                */

                // publish odometry
                nav_msgs::msg::Odometry laserOdometry;
                laserOdometry.header.frame_id = "map";
                laserOdometry.child_frame_id = "base_link";
                laserOdometry.header.stamp = rclcpp::Time(stamp * 1e9);

                laserOdometry.pose.pose.orientation.x = q_current.x();
                laserOdometry.pose.pose.orientation.y = q_current.y();
                laserOdometry.pose.pose.orientation.z = q_current.z();
                laserOdometry.pose.pose.orientation.w = q_current.w();
                laserOdometry.pose.pose.position.x = pose.translation().x();
                laserOdometry.pose.pose.position.y = pose.translation().y();
                laserOdometry.pose.pose.position.z = pose.translation().z();
                pubLaserOdometry->publish(laserOdometry);

                // publish odometryinrobot
                nav_msgs::msg::Odometry laserOdometryinrobot;
                laserOdometryinrobot.header.frame_id = "map";
                laserOdometryinrobot.child_frame_id = "base_link";
                laserOdometryinrobot.header.stamp = rclcpp::Time(stamp * 1e9);

                laserOdometryinrobot.pose.pose.orientation.x = q_currentrobot.x();
                laserOdometryinrobot.pose.pose.orientation.y = q_currentrobot.y();
                laserOdometryinrobot.pose.pose.orientation.z = q_currentrobot.z();
                laserOdometryinrobot.pose.pose.orientation.w = q_currentrobot.w();
                laserOdometryinrobot.pose.pose.position.x = poseinrobot.translation().x();
                laserOdometryinrobot.pose.pose.position.y = poseinrobot.translation().y();
                laserOdometryinrobot.pose.pose.position.z = poseinrobot.translation().z();
                pubLaserOdometryinrobot->publish(laserOdometryinrobot);

                //  publish path
                geometry_msgs::msg::PoseStamped laserPose;
                laserPose.header = laserOdometry.header;
                laserPose.pose = laserOdometry.pose.pose;
                laserOdoPath.header.stamp = laserOdometry.header.stamp;
                laserOdoPath.poses.push_back(laserPose);
                laserOdoPath.header.frame_id = "/map";
                pubLaserOdometryPath->publish(laserOdoPath);
            }

            return true;
        }

    );

    // auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, SE3 & poseinrobot, ros::Time stamp)>(
    //     [&](std::string &topic_name, SE3 &pose, SE3 & poseinrobot, ros::Time stamp){

    //         static tf::TransformBroadcaster br;
    //         tf::Transform transform;
    //         Eigen::Quaterniond q_current(pose.so3().matrix());
    //         Eigen::Quaterniond q_currentrobot(poseinrobot.so3().matrix());

    //         transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
    //         tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
    //         transform.setRotation(q);
           
    //         if (topic_name == "laser"){
    //             br.sendTransform(tf::StampedTransform(transform, stamp, "map", "base_link"));

    //             // publish odometry
    //             nav_msgs::msg::Odometry laserOdometry;
    //             laserOdometry.header.frame_id = "map";
    //             laserOdometry.child_frame_id = "base_link";
    //             laserOdometry.header.stamp = stamp;

    //             laserOdometry.pose.pose.orientation.x = q_current.x();
    //             laserOdometry.pose.pose.orientation.y = q_current.y();
    //             laserOdometry.pose.pose.orientation.z = q_current.z();
    //             laserOdometry.pose.pose.orientation.w = q_current.w();
    //             laserOdometry.pose.pose.position.x = pose.translation().x();
    //             laserOdometry.pose.pose.position.y = pose.translation().y();
    //             laserOdometry.pose.pose.position.z = pose.translation().z();
    //             pubLaserOdometry.publish(laserOdometry);

    //             // publish odometryinrobot
    //             nav_msgs::msg::Odometry laserOdometryinrobot;
    //             laserOdometryinrobot.header.frame_id = "map";
    //             laserOdometryinrobot.child_frame_id = "base_link";
    //             laserOdometryinrobot.header.stamp = stamp;

    //             laserOdometryinrobot.pose.pose.orientation.x = q_currentrobot.x();
    //             laserOdometryinrobot.pose.pose.orientation.y = q_currentrobot.y();
    //             laserOdometryinrobot.pose.pose.orientation.z = q_currentrobot.z();
    //             laserOdometryinrobot.pose.pose.orientation.w = q_currentrobot.w();
    //             laserOdometryinrobot.pose.pose.position.x = poseinrobot.translation().x();
    //             laserOdometryinrobot.pose.pose.position.y = poseinrobot.translation().y();
    //             laserOdometryinrobot.pose.pose.position.z = poseinrobot.translation().z();
    //             pubLaserOdometryinrobot->publish(laserOdometryinrobot);

    //             //  publish path
    //             geometry_msgs::PoseStamped laserPose;
    //             laserPose.header = laserOdometry.header;
    //             laserPose.pose = laserOdometry.pose.pose;
    //             laserOdoPath.header.stamp = laserOdometry.header.stamp;
    //             laserOdoPath.poses.push_back(laserPose);
    //             laserOdoPath.header.frame_id = "/map";
    //             pubLaserOdometryPath.publish(laserOdoPath);
    //         }
    //         else if (topic_name == "world"){
    //             br.sendTransform(tf::StampedTransform(transform, stamp, "world", "map"));
    //         }

    //         return true;
    //     }

    // );


    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vel_pub = 
        node->create_publisher<std_msgs::msg::Float32>("/velocity", 1);
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub = 
        node->create_publisher<std_msgs::msg::Float32>("/move_dist", 1);

    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string &topic_name, double time1, double time2)
        {
            std_msgs::msg::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub->publish(time_rviz);
            else
                dist_pub->publish(time_rviz);

            return true;
        }

    );

    lio->setFunc(cloud_pub_func);
    // lio->setFunc(cloud_pub_func2);
    lio->setFunc(pose_pub_func);
    lio->setFunc(data_pub_func);
    std::shared_ptr<zjloc::InteractiveGraph> graph;
    graph.reset(new zjloc::InteractiveGraph());
    graph->InteractiveGraphinit(model_path);
    // lio->setgraph(graph);
    lio->setgraph(graph);
    lio->setMode(true);
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    /*if (convert->lidar_type_ == zjloc::CloudConvert::LidarType::AVIA) {
        subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            laser_topic, 100, livox_pcl_cbk);
    } else {
        subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
            laser_topic, 100, standard_pcl_cbk);
    }*/
    subLaserCloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        laser_topic, 100, std::bind(&standard_pcl_cbk,std::placeholders::_1));
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_ori = node->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 500, std::bind(&imuHandler,std::placeholders::_1));

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_ori = node->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 500,std::bind(&odomHandler,std::placeholders::_1));

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_ori = node->create_subscription<sensor_msgs::msg::Image>(
        img_topic, 500,std::bind(&imgHandler,std::placeholders::_1));


    std::thread measurement_process(&zjloc::lidarodom::run, lio);
    rclcpp::spin(node);
    if(!localization_mode)
    {
        zjloc::Loop *loop;
        // loop = new zjloc::Loop(graph);
        loop = new zjloc::Loop(graph);
        loop->loopinit(output_path);
        lio->savemap(output_path+"/front-voxel.pcd");
        loop->setAutomaticLoopCloseWindow(0);
        loop->LoopDetectionAll();
        while(loop->isrunningdetection()) {}
        graph->save_pointcloud(output_path+"/");
        lio->updatemap(loop->get_pointcloud());
        lio->savemap(output_path+"/final-voxel.pcd");
    }
    //外部回环优化
    // zjloc::Loop *loop;
    // std::vector<zjloc::OdometryFrame::Ptr> kfs;
    // kfs = lio->getkeyframes();
    // loop = new zjloc::Loop(kfs);
    // loop->buildgraph();
    // loop->setAutomaticLoopCloseWindow();
    // loop->LoopDetection();
    // std::cout<<"end loop"<<std::endl;
    
    zjloc::common::Timer::PrintAll();
    zjloc::common::Timer::DumpIntoFile(DEBUG_FILE_DIR("log_time.txt"));

    std::cout << ANSI_COLOR_GREEN_BOLD << " out done. " << ANSI_COLOR_RESET << std::endl;

    sleep(3);
    return 0;
}